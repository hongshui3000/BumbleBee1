/***************************************************************************
 Copyright (C) Realtek
 This module is a confidential and proprietary property of Realtek and
 a possession or use of this module requires written permission of Realtek.
 ***************************************************************************/

/**
 * \file
 *  le_hw_reg.h (Low Energy Hardware Register Definition)
 *
 * \author
 *  austin <austin_chen@realtek.com>, (C) 2010
 */

/** \addtogroup Low ELink Layer Driver Module
 *  @{ */
#ifndef __LE_HW_REG_H__
#define __LE_HW_REG_H__

#include "DataType.h"
#include "le_ll.h"

/* The Base of LE Register */
#define LE_REG_BASE                    0x40051000

/* The Macro to Read/Write LE HW Register */
#define WR_LE_REG(offset, val)   WR_16BIT_IO(LE_REG_BASE, offset,val)
#define RD_LE_REG(offset)        RD_16BIT_IO(LE_REG_BASE, offset)
#define UPDATE_LE_REG(offset, mask, value)\
    do {\
        UINT16 tmp = RD_LE_REG(offset) & ~(mask);\
        WR_LE_REG(offset, (tmp | ((value) & (mask))));\
    } while (0);

/* The Group of Control Parameters Register */
#define LE_REG_INSTRUCTION              0x00
#define LE_REG_CBK_CONTROL              0x02

/* The Group of Connection Request Parameter Registers */
#define LE_REG_CONN_AA_HF(x)           (0x04 + ((x) << 1))
#define LE_REG_CONN_AA_L                0x04
#define LE_REG_CONN_AA_H                0x06
#define LE_REG_CONN_WIN_SIZE            0x08
#define LE_REG_CONN_WIN_OFFSET          0x0A
#define LE_REG_CONN_LATENCY             0x0C
#define LE_REG_CONN_INTERVAL            0x0E
#define LE_REG_CONN_WINDOW              0x10
#define LE_REG_CONN_CE_LEN              0x12
#define LE_REG_CONN_CE_INTERVAL         0x14
#define LE_REG_CONN_SUPERVISION_TO      0x16
#define LE_REG_CONN_CH_MAP_HF(x)       (0x18 + ((x) << 1))
#define LE_REG_CONN_CH_MAP_L            0x18
#define LE_REG_CONN_CH_MAP_M            0x1A
#define LE_REG_CONN_CH_MAP_H            0x1C
#define LE_REG_CONN_INSTANT             0x1E

/* The Group of Connection Update Parameter Registers */
#define LE_REG_CONN_UPD_INSTANCE        0x20
#define LE_REG_CONN_UPD_WIN_SIZE        0x22
#define LE_REG_CONN_UPD_WIN_OFFSET      0x24
#define LE_REG_CONN_UPD_LATENCY         0x26
#define LE_REG_CONN_UPD_CE_LEN          0x28
#define LE_REG_CONN_UPD_CE_INTERVAL     0x2A
#define LE_REG_CONN_UPD_SUPERVISION_TO  0x2C
#define LE_REG_CONN_UPD_ENTRY           0x2E

/* The Group of Channel Update Parameter Registers */
#define LE_REG_CH_UPD_INSTANT           0x30
#define LE_REG_CH_UPD_MAP_HF(x)        (0x32 + ((x) << 1))
#define LE_REG_CH_UPD_MAP_L             0x32
#define LE_REG_CH_UPD_MAP_M             0x34
#define LE_REG_CH_UPD_MAP_H             0x36

/* The Group of Scan Request Parameter Registers */
#define LE_REG_SCAN_INSTANT             0x38
#define LE_REG_SCAN_INTERVAL            0x3A
#define LE_REG_SCAN_WINDOW              0x3C
#define LE_REG_SCAN_CONTROL             0x3E

/* The Group of Advertising Request Parameter Registers */
#define LE_REG_ADV_CONTROL              0x40
#define LE_REG_ADV_INTERVAL             0x42
#define LE_REG_ADV_INSTANCE             0x44

/* The Group of Cancel Parameter Registers */
#define LE_REG_CANCEL_ENTRY             0x46

/* The Group of Clock Registers */
#define LE_REG_CLK_COMPENSATE           0x48
#define LE_REG_CLK_RANDOM_HF(x)        (0x4A + ((x) << 1))
#define LE_REG_CLK_RANDOM_L             0x4A
#define LE_REG_CLK_RANDOM_H             0x4C
#define LE_REG_CLK_CE_COMPENSATE        0x4E
#define LE_REG_EXT_MISR                 0x50
#define LE_REG_NO_ACL                   0x52

/* The Group of Slave Connection Registers */
#define LE_REG_CONN_COUNTER             0x58
#define LE_REG_SLAVE_WIN_WIDENING_L     0x5A
#define LE_REG_SLAVE_WIN_WIDENING_H     0x5C
#define LE_REG_SLAVE_AA_HF(x)          (0x5E + ((x) << 1))
#define LE_REG_SLAVE_AA_L               0x5E
#define LE_REG_SLAVE_AA_H               0x60
#define LE_REG_SLAVE_CONN_WIN_SIZE      0x62
#define LE_REG_SLAVE_CONN_WIN_OFFSET    0x64
#define LE_REG_SLAVE_CONN_LATENCY       0x66
#define LE_REG_SLAVE_CE_LEN             0x68
#define LE_REG_SLAVE_CE_INTERVAL        0x6A
#define LE_REG_SLAVE_SUPERVISION_TO     0x6C
#define LE_REG_SLAVE_CH_MAP_HF(x)      (0x6E + ((x) << 1))
#define LE_REG_SLAVE_CH_MAP_L           0x6E
#define LE_REG_SLAVE_CH_MAP_M           0x70
#define LE_REG_SLAVE_CH_MAP_H           0x72

/* The Group of CAM Registers */
#define LE_REG_CAM_ACCESS               0x74
#define LE_REG_CAM_DATA_HF(x)          (0x76 + ((x) << 1))
#define LE_REG_CAM_DATA_L               0x76
#define LE_REG_CAM_DATA_H               0x78

/* The Group of CE Registers */
#define LE_REG_CE_RX_STATUS             0x7A
#define LE_REG_CE_TX_STATUS             0x7C

/* The Group of Interrupt Registers */
#define LE_REG_INT_CE_EARLY_INT_TIME    0x7E
#define LE_REG_INT_MISR                 0x80
#define LE_REG_INT_IMR                  0x82
#define LE_REG_RANDOM_NUM_CTRL          0x84
#define LE_REG_RANDOM_SEED_HF(x)       (0x86 + ((x) << 1))
#define LE_REG_RANDOM_SEED_L            0x86
#define LE_REG_RANDOM_SEED_H            0x88
#define LE_REG_RANDOM_NUM_HF(x)        (0x86 + ((x) << 1))
#define LE_REG_RANDOM_NUM_L             0x86
#define LE_REG_RANDOM_NUM_H             0x88
#define LE_REG_SLAVE_SESSION_KEY(x)    (0x8C + ((x) << 1))
#define LE_REG_SLAVE_IV(x)             (0xB4 + ((x) << 1))

/* The Group of Status Registers */
#define LE_REG_STATUS_RX_THRES_CTRL     0xBC
#define LE_REG_STATUS_RX_THRES_STATUS   0xBE
#define LE_REG_STATUS_ADV_TIMEOUT_CTRL  0xC0
#define LE_REG_STATUS_ADV_STATUS        0xC2
#define LE_REG_STATUS_TERMINATE_REASON  0xC4
#define LE_REG_STATUS_CONN_STATUS       0xC6
#define LE_REG_STATUS_CE_BEGIN_STATUS   0xC8
#define LE_REG_STATUS_CE_WORD_CNT       0xCA
#define LE_REG_STATUS_CE_END_EVENT      0xCC
#define LE_REG_STATUS_RX_INT_STACK      0xCE
#define LE_REG_STATUS_TX_THR_STATUS     0xD0
#define LE_REG_STATUS_CAM_VALID         0xD2

#define LE_REG_SLAVE_CLOCK_LOW          0xD4
#define LE_REG_SLAVE_CLOCK_HIGH         0xD6

/* The Group of CE End Control Registers */
#define LE_REG_CE_END_CTRL              0x100
#define LE_REG_RX_NAK_NUM               0x102

/* The Group of LE TX/RX delay control */
#define LE_REG_DELAY_TRX_ON             0x104
#define LE_REG_DELAY_RX_TURNAROUND      0x106
#define LE_REG_DELAY_TX_TURNAROUND      0x108
#define LE_REG_DELAY_CE_RX_TIMEOUT      0x10A
#define LE_REG_DELAY_RX_SEARCH_TIMEOUT  0x10C
#define LE_REG_NUM_OF_TX_PKT_IN_RF_TEST 0x10E

#define LE_REG_RX_TURNAROUND_DELAY      0x110
#ifdef _DAPE_SUPPORT_LE_TX_PKT_MODE
#define LE_REG_TX_TEST_CNT_MODE         0x112
#endif
/* The Group of Device Address */
#define LE_REG_DEVA_REMOTE_HF(x)       (0x120 + ((x) << 1))
#define LE_REG_DEVA_REMOTE_L            0x120
#define LE_REG_DEVA_REMOTE_M            0x122
#define LE_REG_DEVA_REMOTE_H            0x124
#define LE_REG_DEVA_RANDOM_LOCAL_HF(x) (0x126 + ((x) << 1))
#define LE_REG_DEVA_RANDOM_LOCAL_L      0x126
#define LE_REG_DEVA_RANDOM_LOCAL_M      0x128
#define LE_REG_DEVA_RANDOM_LOCAL_H      0x12A
#define LE_REG_DEVA_PUBLIC_LOCAL_HF(x) (0x12C + ((x) << 1))
#define LE_REG_DEVA_PUBLIC_LOCAL_L      0x12C
#define LE_REG_DEVA_PUBLIC_LOCAL_M      0x12E
#define LE_REG_DEVA_PUBLIC_LOCAL_H      0x130
#define LE_REG_DEVA_TYPE                0x132

/* The Group of RX Pkt Error Control */
#define LE_REG_RX_PKT_ERR_CTRL          0x134

/* The Group of Modem/RF Control */
#define LE_REG_MODEM_CONTROL            0x136
#define LE_REG_RF_TEST_CONTROL          0x138
#ifdef _DAPE_SUPPORT_CHG_AA_IN_LE_TEST_MODE
#define LE_REG_TEST_MODE_ACCESS_ADDR_L  0x13A
#define LE_REG_TEST_MODE_ACCESS_ADDR_H  0x13C
#endif
/* The Group of FW AES Encryption */
#define LE_REG_AES_FW_ENC_CTRL          0x13E
#define LE_REG_AES_ENC_KEY_HF(x)       (0x140 + ((x) << 1))
#define LE_REG_AES_ENC_DATA_HF(x)      (0x150 + ((x) << 1))

/* Dape Added */
#define LE_REG_TX_WIN_OFFSET_RPT            0x160
#ifdef _NEW_BLE_HW_SPEC_FROM_150320_
#define LE_REG_ANCHOR_POINT_RPT1            0x162 /* (cch) only slave mode */
#define LE_REG_ANCHOR_POINT_RPT2            0x164 /* (cch) only slave mode */
#endif
#define LE_REG_PRE_MASK_TIME                0x166

/* The BD_ADDR that Initiator send Conn Req PDU to. Can be used since 8703B. */
#define LE_REG_CONN_REQ_REMOTE_BD_ADDR_0       0x168
#define LE_REG_CONN_REQ_REMOTE_BD_ADDR_1       0x16A
#define LE_REG_CONN_REQ_REMOTE_BD_ADDR_2       0x16C

#define LE_REG_CONN_REQ_REMOTE_BD_ADDR_TYPE    0x16E

#ifdef _NEW_BLE_HW_SPEC_FROM_150320_
#define LE_REG_INST_RPT                 0x170
#define LE_REG_ANCH_DIFF_RPT0           0x172
#define LE_REG_ANCH_DIFF_RPT1           0x174
#define LE_REG_CE_EARLY_GUARD_TIME      0x176
#endif

#ifdef _BT4_2_PRIVACY_FEATURE_SUPPORT_
#define LE_REG_PRIVACY_MISC0            0x180
#define LE_REG_LOCAL_IRK_VALID_L        0x182
#define LE_REG_LOCAL_IRK_VALID_H        0x184
#define LE_REG_LOCAL_IRK_VALID_HF(x)    (0x182 + ((x) << 1))
#define LE_REG_PEER_IRK_VALID_L         0x186
#define LE_REG_PEER_IRK_VALID_H         0x188
#define LE_REG_PEER_IRK_VALID_HF(x)     (0x186 + ((x) << 1))
#define LE_REG_PEER_ADDR_VALID_L        0x18A
#define LE_REG_PEER_ADDR_VALID_H        0x18C
#define LE_REG_PEER_ADDR_VALID_HF(x)    (0x18A + ((x) << 1))
#define LE_REG_PEER_RESOLVABLE_ADDR_L   0x18E
#define LE_REG_PEER_RESOLVABLE_ADDR_M   0x190
#define LE_REG_PEER_RESOLVABLE_ADDR_H   0x192
#define LE_REG_LOCAL_RESOLVABLE_ADDR_L  0x194
#define LE_REG_LOCAL_RESOLVABLE_ADDR_M  0x196
#define LE_REG_LOCAL_RESOLVABLE_ADDR_H  0x198
#define LE_REG_LOCAL_IRK_HF(x)          (0x19A + ((x) << 1))
#define LE_REG_PEER_IRK_HF(x)           (0x1AA + ((x) << 1))
#define LE_REG_LOCAL_RAND_NUM_L         0x1BA
#define LE_REG_LOCAL_RAND_NUM_H         0x1BC
#define LE_REG_PEER_RAND_NUM_L          0x1BE
#define LE_REG_PEER_RAND_NUM_H          0x1C0
#define LE_REG_PRIVACY_MISC1            0x1C2
#define LE_REG_PRIVACY_PEER_RULE_PASS   0x1C4
#define LE_REG_PRIVACY_LOCAL_RULE_PASS  0x1C6
#endif /* _BT4_2_PRIVACY_FEATURE_SUPPORT_ */

#ifdef _CCH_SC_ECDH_P256_
#define CAM_ADDR_REG                    0x236
#define CAM_DATA_REG0                   0x238
#define CAM_DATA_REG1                   0x23A
#endif


/* The Group of CAM Register */
#define LE_CAM_ADDR_0                   0
#define LE_CAM_ADDR_1                   1
#define LE_CAM_ADDR_2                   2
#define LE_CAM_ADDR_AA                  2
#define LE_CAM_ADDR_3                   3
#define LE_CAM_ADDR_CH_MAP              3
#define LE_CAM_ADDR_4                   4
#define LE_CAM_ADDR_5                   5
#define LE_CAM_ADDR_6                   6
#define LE_CAM_ADDR_ENC_PKT_CNT_H       6
#define LE_CAM_ADDR_IV(x)              (7 + ((x) & 0x1))
#define LE_CAM_ADDR_7                   7
#define LE_CAM_ADDR_IV_LW               7
#define LE_CAM_ADDR_8                   8
#define LE_CAM_ADDR_IV_HW               8
#define LE_CAM_ADDR_AES_SES_KEY(x)     (9 + ((x) & 0x3))
#define LE_CAM_ADDR_9                   9
#define LE_CAM_ADDR_AES_SES_KEY0        9
#define LE_CAM_ADDR_10                  10
#define LE_CAM_ADDR_AES_SES_KEY1        10
#define LE_CAM_ADDR_11                  11
#define LE_CAM_ADDR_AES_SES_KEY2        11
#define LE_CAM_ADDR_12                  12
#define LE_CAM_ADDR_AES_SES_KEY3        12
#define LE_CAM_ADDR_13                  13
#define LE_CAM_ADDR_DEC_PKT_CNT_L       13
#define LE_CAM_ADDR_14                  14

#define LE_CAM_ADDR_15                  15  //slave only
#define LE_CAM_ADDR_16                  16  //slave only
#define LE_CAM_ADDR_17                  17  //slave only

/* Bluewiz HW Entry Number In CAM */
#define LE_CAM_CONN_ENTRY_NUM           8
#define LE_CAM_WHITE_LIST_NUM           32
#define LE_CAM_BLACK_LIST_NUM           32

#define LE_CAM_ONE_ENTRY_BYTE_SIZE      60
#define LE_CAM_ONE_ENTRY_DW_SIZE        15

#define LE_CAM_ONE_DEV_LIST_BYTE_SIZE   8
#define LE_CAM_ONE_DEV_LIST_DW_SIZE     2

#define LE_CAM_ENTRY_BASE(entry_id)    ((entry_id) * LE_CAM_ONE_ENTRY_DW_SIZE)
#define LE_CAM_LIST_LW_OFFSET(idx)     ((idx) * LE_CAM_ONE_DEV_LIST_DW_SIZE)

#define LE_CAM_WHITE_LIST_BASE          (LE_CAM_ONE_ENTRY_DW_SIZE * \
                                         LE_CAM_CONN_ENTRY_NUM)
#define LE_CAM_ADDR_WHITE_LIST_LW(idx) (LE_CAM_WHITE_LIST_BASE + \
                                        LE_CAM_LIST_LW_OFFSET(idx))
#define LE_CAM_ADDR_WHITE_LIST_HW(idx) (LE_CAM_ADDR_WHITE_LIST_LW(idx) + 1)

#define LE_CAM_BLACK_LIST_BASE         (LE_CAM_WHITE_LIST_BASE + \
                                        (LE_CAM_WHITE_LIST_NUM * 2))
#define LE_CAM_ADDR_BLACK_LIST_LW(idx) (LE_CAM_BLACK_LIST_BASE + \
                                        LE_CAM_LIST_LW_OFFSET(idx))
#define LE_CAM_ADDR_BLACK_LIST_HW(idx) (LE_CAM_ADDR_BLACK_LIST_LW(idx) + 1)

#define LE_CAM_RESOLVING_LIST_BASE     250 /* defined in LE_CAM.xls */

#define LE_HW_RX_PKT_TAIL_SIZE          4   /* hw rx status(2B) + rssi(2B) */

#define LE_HW_ADV_RX_PKT_MIN_SIZE      (LL_ADV_CH_PDU_HEADER_SIZE + \
                                        LL_ADV_CH_PAYLOAD_LEN_MIN + \
                                        LE_HW_RX_PKT_TAIL_SIZE)
#define LE_HW_ADV_RX_PKT_DIRECT_SIZE   (LL_ADV_CH_PDU_HEADER_SIZE + \
                                        LL_ADV_DIRECT_PL_SIZE + \
                                        LE_HW_RX_PKT_TAIL_SIZE)
#define LE_HW_ADV_RX_PKT_SCAN_REQ_SIZE (LL_ADV_CH_PDU_HEADER_SIZE + \
                                        LL_SCAN_REQ_PL_SIZE + \
                                        LE_HW_RX_PKT_TAIL_SIZE)
#define LE_HW_ADV_RX_PKT_CONN_REQ_SIZE (LL_ADV_CH_PDU_HEADER_SIZE + \
                                        LL_CONNECTION_REQ_PL_SIZE + \
                                        LE_HW_RX_PKT_TAIL_SIZE)

/* Instruction Set of LE HW */
enum LE_HW_INSTRUCTION_SET_ {
    LE_HW_INSTRUCTION_CONN_REQ          = 0, /* send connection request */
    LE_HW_INSTRUCTION_CONN_CANCEL       = 1, /* cancel connection req */
    LE_HW_INSTRUCTION_CONN_UPDATE       = 2, /* send connection update pdu */
    LE_HW_INSTRUCTION_DISCONN_REQ       = 3, /* send LL terminate pdu */
    LE_HW_INSTRUCTION_CHANNEL_UPDATE    = 4, /* channel update pdu */
    LE_HW_INSTRUCTION_ADV_REQ           = 5, /* send advertising pdu */
    LE_HW_INSTRUCTION_SCAN_REQ          = 6, /* send scan req pdu */
    LE_HW_INSTRUCTION_SCAN_CANCEL       = 7, /* cancel scanning state  */
    LE_HW_INSTRUCTION_ADV_CANCEL        = 8, /* cencel advertising state */
    LE_HW_INSTRUCTION_CONN_KILL         = 9, /* kill connection entry */
    LE_HW_INSTRUCTION_MAX               = LE_HW_INSTRUCTION_CONN_KILL,
};

/* The Terminated Reason of HW Advertising State Machine */
enum LE_ADV_TERMINATE_REASONS {
    LE_ADV_TERM_REASON_NONE,
    LE_ADV_TERM_REASON_USER_CANCEL,
    LE_ADV_TERM_REASON_CONN_CREATED,
    LE_ADV_TERM_REASON_DIRECT_ADV_TIMEOUT,
    LE_ADV_TERM_REASON_MAX = LE_ADV_TERM_REASON_DIRECT_ADV_TIMEOUT,
};

/* The Terminated Reason of HW Connection State Machine */
enum LE_CONN_TERMINATE_REASONS {
    LE_CONN_TERM_REASON_CONNECT_SUCCESS,
    LE_CONN_TERM_REASON_USER_DISCONNET,
    LE_CONN_TERM_REASON_USER_CANCEL,
    LE_CONN_TERM_REASON_SIX_FAILURE,
    LE_CONN_TERM_REASON_RX_TERM_IND,
    LE_CONN_TERM_REASON_MAX = LE_CONN_TERM_REASON_RX_TERM_IND,
};

/*===========================================================*/
/*  The Data Structure of LE HW Register                     */
/*===========================================================*/
/* The Structure of LE_REG_INSTRUCTION Register (0x00) */
typedef struct LE_REG_S_INSTRUCTION_ {
    UINT16 instruction:8;       /* bit[7:0], instruction */
    UINT16 conn_cancel_fail:1;  /* bit[8], connection cancel instruction fail */
    UINT16 rsvd:7;              /* bit[15:9], reserved */
} LE_REG_S_INSTRUCTION, *PLE_REG_S_INSTRUCTION;

/* The Structure of LE_REG_CBK_CONTROL Register (0x02) */
typedef struct LE_REG_S_CBK_CONTROL_ {
    UINT16 init_filt:1;     /* bit[0], the filter policy of initiator */
    UINT16 inst_en:1;       /* bit[1], instant enable (FW can enable this bit to
                               tell schedular when process advertising or
                               scanning in the instant */
#ifdef _DAPE_LOW_DUTY_ADV
    UINT16 low_duty_adv_en:1;/* bit[2]*/
# ifdef _SUPPORT_LBT_FUNC_FOR_RF_ADAPTIVITY_
    UINT16 rsvd2:4;         /* bit[6:3], reserved */
    UINT16 lbt_en_adv:1;    /* bit[7], enable bit of lbt control for
                                       rf adaptivity */
# else
    UINT16 rsvd2:5;         /* bit[7:3], reserved */
# endif
#else
    UINT16 rsvd2:6;         /* bit[7:2], reserved */
#endif
    UINT16 adv_tx_power:8;  /* bit[15:8], advertising tx power */
} LE_REG_S_CBK_CONTROL, *PLE_REG_S_CBK_CONTROL;

/* The Structure of LE_REG_CONN_WIN_SIZE Register (0x08) */
typedef struct LE_REG_S_CONN_WIN_SIZE_ {
    UINT16 conn_win_sz:8;    /* bit[7:0], connection window size */
    UINT16 initiator_filt:1; /* bit[8], enable initiator filter */
    UINT16 rsvd:7;           /* bit[15:9], reserved */
} LE_REG_S_CONN_WIN_SIZE, *PLE_REG_S_CONN_WIN_SIZE;

/* The Structure of LE_REG_CONN_LATENCY Register (0x0C) */
typedef struct LE_REG_S_CONN_LATENCY_ {
    UINT16 slave_latency:12; /* bit[11:0], slave latency */
    UINT16 conn_req_chm:3;   /* bit[14:12], advertising channel map of
                                connection request */
    UINT16 rsvd:1;           /* bit[15], reserved */
} LE_REG_S_CONN_LATENCY, *PLE_REG_S_CONN_LATENCY;

/* The Structure of LE_REG_CONN_CH_MAP_H Register (0x1C) */
typedef struct LE_REG_S_CONN_CH_MAP_H_ {
    UINT16 ch_map:5;        /* bit[4:0], channel map (ch36 ~ ch32) */
    UINT16 hop_inc:5;       /* bit[9:5], hopping incremental value */
    UINT16 conn_entry:6;    /* bit[15:10], connection entry */
} LE_REG_S_CONN_CH_MAP_H, *PLE_REG_S_CONN_CH_MAP_H;

// dape added
/* The Structure of LE_REG_CONN_UPD_WIN_SIZE Register (0x22) */
typedef struct LE_REG_S_CONN_UPD_WIN_SIZE_ {
    UINT16 conn_win_size:8; /* bit[7:0], conn win size */
    UINT16 conn_win_size_offset:8;   /* bit[15:8], win size offset (n*1.25ms) */
} LE_REG_S_CONN_UPD_WIN_SIZE, *PLE_REG_S_CONN_UPD_WIN_SIZE;

/* The Structure of LE_REG_CONN_UPD_ENTRY Register (0x2E) */
typedef struct LE_REG_S_CONN_UPD_ENTRY_ {
    UINT16 updt_ce_minus_one:1;     /* bit[0], minus one ce counter to update */
    UINT16 blk_legacy_one_slot:1;   /* bit[1], block legacy traffic one slot
                                       before LE anchor point (except SCO/eSCO/
                                       sniff), default on */
    UINT16 high_pri_as_no_crc_ok:1; /* bit[2], make le priority higher as no
                                       CRC correct packet received, default on*/
    UINT16 confl_avoid:1;          	/* bit[3], 1: hw avoids anchor point conflict*/
    UINT16 rsvd:6;                  /* bit[9:4], reserved */
    UINT16 conn_upd_entry:6;        /* bit[15:10], connection entry */
} LE_REG_S_CONN_UPD_ENTRY, *PLE_REG_S_CONN_UPD_ENTRY;

/* The Structure of LE_REG_CH_UPD_MAP_H Register (0x36) */
typedef struct LE_REG_S_CH_UPD_MAP_H_ {
    UINT16 ch_map:5;        /* bit[4:0], channel map (ch36 ~ ch32) */
    UINT16 rsvd:5;          /* bit[9:5], reserved */
    UINT16 ch_upd_entry:6;  /* bit[15:10], channel map update entry */
} LE_REG_S_CH_UPD_MAP_H, *PLE_REG_S_CH_UPD_MAP_H;

/* The Structure of LE_REG_SCAN_CONTROL Register (0x3E) */
typedef struct LE_REG_S_SCAN_CONTROL_ {
    UINT16 scan_filter:1;   /* bit[0], scan filter policy */
    UINT16 scan_chm:3;      /* bit[3:1], scan channel bitmap */
    UINT16 active_scan:1;   /* bit[4], channel map update entry */
    UINT16 dup_filter:1;    /* bit[5], duplicate filter */
    UINT16 scan_pri_ctrl:1; /* bit[6], scan_pri_ctrl, 1:stop scanning at the end of scan window
                               even if scan_rsp is not received */
    UINT16 rsvd:9;          /* bit[15:7], reserved */
} LE_REG_S_SCAN_CONTROL, *PLE_REG_S_SCAN_CONTROL;

/* The Structure of LE_REG_ADV_CONTROL Register (0x40) */
typedef struct LE_REG_S_ADV_CONTROL_ {
    UINT16 pkt_type:4;        /* bit[3:0], advetising pkt type */
    UINT16 filter_scan_req:1; /* bit[4], filter scan request */
    UINT16 filter_conn_req:1; /* bit[5], filter connection request */
    UINT16 adv_chm:3;         /* bit[8:6], advertising channel bitmap */
    UINT16 adv_h2h_period:7;  /* bit[15:9], head to head period (625us unit) */
} LE_REG_S_ADV_CONTROL, *PLE_REG_S_ADV_CONTROL;

/* The Structure of LE_REG_CANCEL_ENTRY Register (0x46) */
typedef struct LE_REG_S_CANCEL_ENTRY_ {
    UINT16 disconn_entry:8; /* bit[7:0], disconnect entry */
    UINT16 kill_entry:8;    /* bit[15:8], kill entry */
} LE_REG_S_CANCEL_ENTRY, *PLE_REG_S_CANCEL_ENTRY;


/* The Structure of LE_REG_EXT_MISR Register (0x50)(HW after 8821MP) */
typedef struct LE_REG_S_LE_EXT_MISR_ {
    UINT16 conn_early:1;         /* bit[0], initiator scan_early interrupt. write 1 clear.*/
    UINT16 scan_early:1;         /* bit[1], scanner scan_early interrupt. write 1 clear.*/
    UINT16 conn_early_en:1;      /* bit[2], initiator scan_early interrupt. write 1 clear.*/
    UINT16 scan_early_en:1;      /* bit[3], scanner scan_early interrupt. write 1 clear.*/
    UINT16 scan_req_en:1;        /* bit[4], scanner SCAN_REQ start interrupt mask */
    UINT16 scan_req:1;           /* bit[5], scanner SCAN_REQ start interrupt. write 1 clear.*/
    UINT16 rsvd:2;               /* bit[7:6], reserved. */
    UINT16 scan_early_time:4;    /* bit[11:8], set the intr time before scan begins.
                                    unit:slot */
#ifdef _DAPE_LE_CE_ENTRY_REPORT_IN_RX_TH_AFTER_2801
    UINT16 rxth_entry_report:4;  /* bit[15:12], report the ce entry of the
                                    rx_th_interrupt. only valid before CE-end*/
#else
    UINT16 rsvd1:4;              /* bit[15:12], reserved. */
#endif
} LE_REG_S_LE_EXT_MISR, *PLE_REG_S_LE_EXT_MISR;


/* The Structure of LE_REG_LE_NO_ACL Register (0x52) */
typedef struct LE_REG_S_LE_NO_ACL_ {
    UINT16 le_no_acl_slot:4;         /* bit[3:0], block_acl before le anchor point*/
    UINT16 rsvd:12;             /* but[15:4], reserved */
} LE_REG_S_LE_NO_ACL, *PLE_REG_S_LE_NO_ACL;


/* The Structure of LE_REG_SLAVE_WIN_WIDENING_L Register (0x5A) */
typedef struct LE_REG_S_SLAVE_WIN_WIDENING_L_ {
    UINT16 slave_ww_us:10;  /* bit[9:0], us unit of slave window widening (RO)*/
    UINT16 ww_en:1;         /* bit[10], enable the window widening */
    UINT16 sub_en:1;        /* bit[11], sub enable ? */
    UINT16 enc_en:1;        /* bit[12], encrytion enable bit (RO) */
    UINT16 dec_en:1;        /* bit[13], decrytion enable bit (RO) */
#ifdef _NEW_BLE_HW_SPEC_FROM_150320_
    UINT16 adv_h2h_en:1;    /* bit[14], enable advertising h2h period bit,
                                        if it is 0, the h2h period is invalid */
    UINT16 rsvd:1;          /* bit[15], reserved */
#else
    UINT16 rsvd:2;          /* bit[15:14], reserved */
#endif
} LE_REG_S_SLAVE_WIN_WIDENING_L, *PLE_REG_S_SLAVE_WIN_WIDENING_L;

/* The Structure of LE_REG_SLAVE_CONN_WIN_SIZE Register (0x62) */
typedef struct LE_REG_S_SLAVE_CONN_WIN_SIZE_ {
    UINT16 conn_win_size:8;  /* bit{7:0], transmit window size */
    UINT16 slave_tx_power:8; /* bit[15:8], slave tx power */
} LE_REG_S_SLAVE_CONN_WIN_SIZE, *PLE_REG_S_SLAVE_CONN_WIN_SIZE;

/* The Structure of LE_REG_SLAVE_CONN_LATENCY Register (0x66) */
typedef struct LE_REG_S_SLAVE_CONN_LATENCY_ {
    UINT16 slave_latency:12; /* bit[11:0], the slave latency */
    UINT16 rsvd:2;           /* bit[13:12], reserved */
    UINT16 lbd_type:1;       /* bit[14], local BD_ADDR type */
    UINT16 rbd_type:1;       /* bit[15], remote BD_ADDR type */
} LE_REG_S_SLAVE_CONN_LATENCY, *PLE_REG_S_SLAVE_CONN_LATENCY;

/* The Structure of LE_REG_SLAVE_CH_MAP_H Register (0x72) */
typedef struct LE_REG_S_SLAVE_CH_MAP_H_ {
    UINT16 ch_map:5;        /* bit[4:0], channel bitmap (ch32~36) */
    UINT16 hop_inc:5;       /* bit[9:5], hopping incremental */
    UINT16 local_sca:3;     /* bit[12:10], local SCA */
    UINT16 remote_sca:3;    /* bit[15:13], remote SCA */
} LE_REG_S_SLAVE_CH_MAP_H, *PLE_REG_S_SLAVE_CH_MAP_H;

/* The Structure of LE_REG_CAM_ACCESS Register (0x74) */
typedef struct LE_REG_S_CAM_ACCESS_ {
    UINT16 cam_addr:14;    /* bit[13:0], CAM Address */
    UINT16 rsvd:1;         /* bit[14], reserved */
    UINT16 cam_rw:1;       /* bit[15], CAM Read(0b)/Write(1b)*/
} LE_REG_S_CAM_ACCESS, *PLE_REG_S_CAM_ACCESS;

/* The Structure of LE_REG_CE_RX_STATUS Register (0x7A) - no use now */
typedef struct LE_REG_S_CE_RX_STATUS_ {
    UINT16 adv_rx:1;            /* bit[0], advertising channel pkt rx */
    UINT16 data_rx:1;           /* bit[1], data channel pkt rx */
    UINT16 crc_err:1;           /* bit[2], CRC error indication */
    UINT16 mic_err:1;           /* bit[3], MIC error indication */
    UINT16 pkt_type_or_llid:4;  /* bit[7:4], rx pkt type or LLID */
    UINT16 rsvd:2;              /* bit[9:8], reserved */
    UINT16 pkt_len:6;           /* bit[15:10], rx pkt length */
} LE_REG_S_CE_RX_STATUS, *PLE_REG_S_CE_RX_STATUS;

/* The Structure of LE_REG_CE_TX_STATUS Register (0x7C) - no use now */
typedef struct LE_REG_S_CE_TX_STATUS_ {
    UINT16 adv_tx:1;            /* bit[0], advertising channel pkt tx */
    UINT16 data_tx:1;           /* bit[1], data channel pkt tx */
    UINT16 rsvd:2;              /* bit[3:2], reserved */
    UINT16 pkt_type:4;          /* bit[7:4], tx pkt type */
    UINT16 pkt_num:8;           /* bit[15:8], tx pkt count */
} LE_REG_S_CE_TX_STATUS, *PLE_REG_S_CE_TX_STATUS;

/* The Structure of LE_REG_INT_MISR Register (0x80) */
typedef struct LE_REG_S_INT_MISR_ {
    UINT16 event_end_int:1;   /* bit[0], CE/AE End interrupt status.
                                (read event end status register to clear it) */
    UINT16 event_begin_int:1; /* bit[1], CE/AE begin early interrupt status.
                                (read event begin status register to clear it)*/
    UINT16 pkt_rx:1;          /* bit[2], Pkt Rx interrupt status.
                                (write 1 to clear it) */
    UINT16 pkt_tx:1;          /* bit[3], Pkt Tx interrupt status.
                                (write 1 to clear it) */
    UINT16 cam_int:1;         /* bit[4], CAM Finish interrupt status
                                (read CAM_addr register to clear it) */
    UINT16 out_of_ww_int:1;   /* bit[5], out of window widening interrupt
                                 (write 1 to clear) */
    UINT16 rx_th_int:1;       /* bit[6], Rx Theshold interrupt status
                                (read rx_fifo_status register to clear it) */
    UINT16 scan_int:1;        /* bit[7], Scan interrupt status (write 1 clear)*/
    UINT16 conn_int:1;        /* bit[8], Connection interrupt status
                                (read conn_status register to clear it)*/
    UINT16 adv_int:1;         /* bit[9], Advertising interrupt status
                                (read adv_status register to clear it) */
    UINT16 tx_thr_int:1;      /* bit[10], Tx ack threshold interrupt
                                (read tx_thres_status register to clear it) */
    UINT16 conn_updt_ce_e:1;  /* bit[11], connection update interrupt in ce end
                                 before instant (write 1 to clear it) */
    UINT16 chm_updt_ce_s:1;   /* bit[12], channel map update interrupt
                                 in ce start (write 1 to clear it) */
    UINT16 conn_updt_tw_s:1;  /* bit[13], connection update interrupt in
                                 the start of tx window (write 1 to clear it) */
    UINT16 chm_updt_ce_e:1;   /* bit[14], channel map update interrupt
                                 in ce end (write 1 to clear it) */
    UINT16 hit_conn_adv:1;    /* bit[15], the interrupt that indicate for
                                 initiator when hit a connectable adv
                                 (write 1 to clear) */
} LE_REG_S_INT_MISR, *PLE_REG_S_INT_MISR;

/* The Structure of LE_REG_INT_IMR Register (0x82) */
typedef struct LE_REG_S_INT_IMR_ {
    UINT16 event_end_int_mask:1;   /* bit[0], CE/AE End interrupt mask */
    UINT16 event_begin_int_mask:1; /* bit[1], CE/AE begin early interrupt mask*/
    UINT16 pkt_rx_mask:1;          /* bit[2], Pkt Rx interrupt mask */
    UINT16 pkt_tx_mask:1;          /* bit[3], Pkt Tx interrupt mask */
    UINT16 cam_int_mask:1;         /* bit[4], CAM Finish interrupt mask */
    UINT16 out_of_ww_int_mask:1;   /* bit[5], out of window widening interrupt
                                              mask */
    UINT16 rx_th_int_mask:1;       /* bit[6], Rx Theshold interrupt mask */
    UINT16 scan_int_mask:1;        /* bit[7], Scan interrupt mask */
    UINT16 conn_int_mask:1;        /* bit[8], Connection interrupt mask */
    UINT16 adv_int_mask:1;         /* bit[9], Advertising interrupt mask */
    UINT16 tx_thr_int_mask:1;      /* bit[10], Tx ack threshold interrupt mask*/
    UINT16 conn_updt_ce_e_mask:1;  /* bit[11], conn update interrupt mask
                                               in ce end before instant */
    UINT16 chm_updt_ce_s_mask:1;   /* bit[12], channel map update interrupt mask
                                               in ce start */
    UINT16 conn_updt_tw_s_mask:1;  /* bit[13], conn update interrupt mask
                                               in the start of tx window */
    UINT16 chm_updt_ce_e_mask:1;   /* bit[14], channel map update interrupt mask
                                               in ce end */
    UINT16 hit_conn_adv_mask:1;    /* bit[15], the interrupt mask that indicate
                                               for initiator when hit a
                                               connectable adv */

} LE_REG_S_INT_IMR, *PLE_REG_S_INT_IMR;

/* The Structure of LE_REG_RANDOM_NUM_CTRL Register (0x84) */
typedef struct LE_REG_S_RANDOM_NUM_CTRL_ {
    UINT16 random_gen:1;    /* bit[0], generate random number */
    UINT16 seed_upd:1;      /* bit[1], update random seed */
    UINT16 period_upd:1;    /* bit[2], periodically update random number */
    UINT16 rsvd:13;         /* bit[15:3], reserved */
} LE_REG_S_RANDOM_NUM_CTRL, *PLE_REG_S_RANDOM_NUM_CTRL;

/* The Structure of LE_REG_STATUS_RX_THRES_CTRL Register (0xBC) */
typedef struct LE_REG_S_STATUS_RX_THRES_CTRL_ {
    UINT16 word_cnt_thres:8;    /* bit[7:0], the threshold of RX word count */
    UINT16 thres_en:1;          /* bit[8], enable the RX threshold  */
    UINT16 pkt_cnt:7;           /* bit[15:9], accumulated rx fifo packet count[6:0] (RO)*/
} LE_REG_S_STATUS_RX_THRES_CTRL, *PLE_REG_S_STATUS_RX_THRES_CTRL;

/* The Structure of LE_REG_STATUS_RX_THRES_STATUS Register (0xBE) */
typedef struct LE_REG_S_STATUS_RX_THRES_STATUS_ {
    UINT16 pkt_cnt:2;           /* bit[1:0], accumulated rx fifo packet count[8:7] (RO) */
    UINT16 rsvd:5;              /* bit[6:2], reserved  */
    UINT16 fifo_word_cnt:9;     /* bit[15:7], fifo word count (read clear) */
} LE_REG_S_STATUS_RX_THRES_STATUS, *PLE_REG_S_STATUS_RX_THRES_STATUS;

/* The Structure of LE_REG_STATUS_ADV_STATUS Register (0xC2) */
typedef struct LE_REG_S_STATUS_ADV_STATUS_ {
    UINT16 active:1;            /* bit[0], adv command is active  */
    UINT16 term_reason:2;       /* bit[2:1], the reason of terminate  */
    UINT16 rsvd:13;             /* bit[15:3], reserved */
} LE_REG_S_STATUS_ADV_STATUS, *PLE_REG_S_STATUS_ADV_STATUS;

/* The Structure of LE_REG_STATUS_TERMINATE_REASON Register (0xC4) */
typedef struct LE_REG_S_STATUS_TERMINATE_REASON_ {
    UINT16 local_term_reason:8; /* bit[7:0],terminate reason of local device*/
    UINT16 rsvd:8;              /* bit[15:8], rsvd */
} LE_REG_S_STATUS_TERMINATE_REASON, *PLE_REG_S_STATUS_TERMINATE_REASON;

/* The Structure of LE_REG_STATUS_CONN_STATUS Register (0xC6) */
typedef struct LE_REG_S_STATUS_CONN_STATUS_ {
    UINT16 conn_num:6;          /* bit[5:0], connection number */
    UINT16 role:1;              /* bit[6], role (1:master, 0:slave) */
    UINT16 term_reason:3;       /* bit[9:7], terminate reason */
    UINT16 entry:6;             /* bit[15:10], entry id */
} LE_REG_S_STATUS_CONN_STATUS, *PLE_REG_S_STATUS_CONN_STATUS;

/* The Structure of LE_REG_STATUS_CE_BEGIN_STATUS Register (0xC8) */
typedef struct LE_REG_S_STATUS_CE_BEGIN_STATUS_ {
    UINT16 ce_begin:1;          /* bit[0], the begin of connection event */
    UINT16 rsvd1:7;             /* bit[7:2], reserved */
    UINT16 entry:6;             /* bit[13:8], entry id */
    UINT16 rsvd2:2;             /* bit[15:14], reserved */
} LE_REG_S_STATUS_CE_BEGIN_STATUS, *PLE_REG_S_STATUS_CE_BEGIN_STATUS;

/* The Structure of LE_REG_STATUS_CE_END_EVENT Register (0xCC) */
typedef struct LE_REG_S_STATUS_CE_END_EVENT_ {
    UINT16 ce_end:1;            /* bit[0], the end of connection event */
    UINT16 ae_end:1;            /* bit[1], the end of advertising event */
    UINT16 entry:5;             /* bit[6:2], the connection entry */
    UINT16 tx_pkt_num:8;        /* bit[14:7], tx packet number */
    UINT16 rx_word_cnt_b16:1;   /* bit[15], bit16 of rx word count */
} LE_REG_S_STATUS_CE_END_EVENT, *PLE_REG_S_STATUS_CE_END_EVENT;

/* The Structure of LE_REG_STATUS_RX_INT_STACK Register (0xCE) */
typedef struct LE_REG_S_STATUS_RX_INT_STACK_ {
    UINT16 rx_ind_id:1;         /* bit[0], "1" is legacy rx, "0" is LE CE end */
    UINT16 no_crc_ok:1;         /* bit[1], "1" all rx packets are no CRC ok */
    UINT16 mic_err:1;           /* bit[2], "1" at least one rx packets are
                                           MIC error */
    UINT16 no_rx_pkt:1;         /* bit[3], "1" all rx packets are miss */
    UINT16 aa_hit:1;            /* bit[4], "1" at least one rx packets are
                                           AA hit */
    UINT16 rsvd:11;             /* bit[15:5], reserved */
} LE_REG_S_STATUS_RX_INT_STACK, *PLE_REG_S_STATUS_RX_INT_STACK;

/* The Structure of LE_REG_STATUS_TX_THR_STATUS Register (0xD0) */
typedef struct LE_REG_S_STATUS_TX_THR_STATUS_ {
    UINT16 acked_tx_pkt_cnt:8;  /* bit[7:0], the acked tx packet counts */
    UINT16 acked_tx_thr:8;      /* bit[15:8], the acked tx threshold */
} LE_REG_S_STATUS_TX_THR_STATUS, *PLE_REG_S_STATUS_TX_THR_STATUS;

/* The Structure of LE_REG_STATUS_CAM_VALID Register (0xD2) */
typedef struct LE_REG_S_STATUS_CAM_VALID_ {
    UINT16 cam_valid_bitmap:8; /* bit[7:0], the bitmap of cam valid */
    UINT16 hop_freq:6;         /* bit[13:8], the hopping frequency */
    UINT16 rsvd:2;             /* bit[15:14], reserved */
} LE_REG_S_STATUS_CAM_VALID, *PLE_REG_S_STATUS_CAM_VALID;

/* The Structure of LE_REG_CE_END_CTRL Register (0x100) */
typedef struct LE_REG_S_CE_END_CTRL_ {
    UINT16 ce_tx_num:7;         /* bit[6:0], tx pkt number in CE*/
    UINT16 rsvd1:1;             /* bit[7], reserved */
    UINT16 ce_end_mode:1;       /* bit[8], CE End mode */
    UINT16 slave_end_mode:1;    /* bit[9], Slave End mode */
    UINT16 slave_ce_len_end:1;  /* bit[10], Slave CE Len End */
    UINT16 rsvd2:4;             /* bit[14:11], reserved */
    UINT16 fw_flow_ctrl:1;      /* bit[15], fw flow control */
} LE_REG_S_CE_END_CTRL, *PLE_REG_S_CE_END_CTRL;

/** LE_REG[0x102] Control TX retry in a CE. */
typedef struct LE_REG_S_RX_NAK_NUM_
{
    /** [7:0] The number of times to resend data in a CE.
     *
     * If HW receives consecutively \a rx_nak_num times of NAK with MD=0,
     * it closes CE immediately.\n
     * Disable this behavior by setting \a rx_nak_num﻿to 0.
     */
    UINT16 rx_nak_num : 8;
    UINT16 reserved : 8;
} LE_REG_S_RX_NAK_NUM;

/* The Structure of LE_REG_DELAY_TRX_ON Register (0x104) */
typedef struct LE_REG_S_DELAY_TRX_ON_ {
    UINT16 txon_delay:8;        /* bit[7:0], the delay of tx on (us) */
    UINT16 rxon_delay:8;        /* bit[15:8], the delay of rx on (us) */
} LE_REG_S_DELAY_TRX_ON, *PLE_REG_S_DELAY_TRX_ON;

/* The Structure of LE_REG_DELAY_RX_TURNAROUND Register (0x106) */
typedef struct LE_REG_S_DELAY_RX_TURNAROUND_ {
    UINT16 rx2tx_delay:8;       /* bit[7:0], the delay from rx to tx (us) */
    UINT16 rx2rx_delay:8;       /* bit[15:8], the delay from rx to rx (us) */
} LE_REG_S_DELAY_RX_TURNAROUND, *PLE_REG_S_DELAY_RX_TURNAROUND;

/* The Structure of LE_REG_DELAY_TX_TURNAROUND Register (0x108) */
typedef struct LE_REG_S_DELAY_TX_TURNAROUND_ {
    UINT16 tx2rx_delay:8;       /* bit[7:0], the delay from tx to rx (us) */
    UINT16 txon_early:6;        /* bit[13:8], the early time of tx on (us) */
#ifdef _DAPE_ENABLE_LE_EARLY_TX_TOGGLE
    UINT16 le_en_txearly_toggle:1; /* bit[14], enable LE tx on early toggle.*/
    UINT16 rsvd:1;              /* bit[15], reserved */
#else
    UINT16 rsvd:2;              /* bit[15:14], reserved */
#endif
} LE_REG_S_DELAY_TX_TURNAROUND, *PLE_REG_S_DELAY_TX_TURNAROUND;

/* The Structure of LE_REG_DELAY_CE_RX_TIMEOUT Register (0x10A) */
typedef struct LE_REG_S_DELAY_CE_RX_TIMEOUT_ {
    UINT16 rx_timeout_delay:8;  /* bit[7:0], the rx timeout delay time (us) */
    UINT16 txon_extension:4;    /* bit[11:8], the extension time of tx on(us)*/
    UINT16 rsvd:4;              /* bit[15:12], reserved */
} LE_REG_S_DELAY_CE_RX_TIMEOUT, *PLE_REG_S_DELAY_CE_RX_TIMEOUT;

#ifdef _DAPE_SUPPORT_LE_TX_PKT_MODE
/* The Structure of LE_REG_S_LE_TX_TEST_CNT_MODE Register (0x112)(HW after 8703B) */
typedef struct LE_REG_S_LE_TX_TEST_CNT_MODE_ {
    UINT16 tx_pkt_cnt:8;         /* bit[7:0], the total tx pkt cnt this time for tx test mode.*/
    UINT16 counter_mode_en:1;    /* bit[8], 1: enable counter mode. */
    UINT16 rsvd:6;               /* bit[14:9], reserved. */
    UINT16 pkt_cnt_tx_busy:1;    /* bit[15], is_tx_ing.*/
} LE_REG_S_LE_TX_TEST_CNT_MODE, *PLE_REG_S_LE_TX_TEST_CNT_MODE;
#endif

/* The Structure of LE_REG_DEVA_TYPE Register (0x132) */
typedef struct LE_REG_S_DEVA_TYPE_ {
    UINT16 local_type_init:1;   /* bit[0], local address type of initiater */
    UINT16 local_type_scan:1;   /* bit[1], local address type of scanner */
    UINT16 local_type_adv:1;    /* bit[2], local address type of advertiser */
    UINT16 remote_type:1;       /* bit[3], remote address type */
#ifdef _USE_NEW_BLE_SCANNER_OPTION_TO_RX_DIR_IND_
    UINT16 scan_rx_dir_ind_opt:2;   /* bit[5:4], option field of scanner
                                                when rx ll_dir_ind in
                                       0 = original design to compare all MAC
                                       1 = based on RxAdd value to compare MAC
                                       2 = do not compare MAC */
    UINT16 rsvd:10;                 /* bit[15:6], reserved */
#else
    UINT16 rsvd:12;             /* bit[15:4], reserved */
#endif
} LE_REG_S_DEVA_TYPE, *PLE_REG_S_DEVA_TYPE;

/* The Structure of LE_REG_RX_PKT_ERR_CTRL Register (0x134) */
typedef struct LE_REG_S_RX_PKT_ERR_CTRL_ {
    UINT16 filtered_kept:1;     /* bit[0], receive filtered rx pkt ? */
    UINT16 type_err_kept:1;     /* bit[1], receive rx pkt when type error */
    UINT16 len_err_kept:1;      /* bit[2], receive rx pkt when length error */
    UINT16 mic_err_kept:1;      /* bit[3], receive rx pkt when MIC error */
    UINT16 crc_err_kept:1;      /* bit[4], receive rx pkt when CRC error */
    UINT16 pkt_dup_kept:1;      /* bit[5], receive duplicated rx pkt */
    UINT16 empty_kept:1;        /* bit[6], receive empty pkt */
    UINT16 scan_req_kept:1;     /* bit[7], receive scan request pdu */
    UINT16 conn_req_kept:1;     /* bit[8], receive connection request pdu */
    UINT16 rsvd:4;              /* bit[12:9], reserved */
    UINT16 whiten_dis:1;        /* bit[13], disable whitening in tx/rx path */
    UINT16 tx_inval_mic:1;      /* but[14], transmit MIC error pkt for test */
    UINT16 tx_inval_crc:1;      /* but[15], transmit CRC error pkt for test */
} LE_REG_S_RX_PKT_ERR_CTRL, *PLE_REG_S_RX_PKT_ERR_CTRL;

/* The Structure of LE_REG_MODEM_CONTROL Register (0x136) */
typedef struct LE_REG_S_MODEM_CONTROL_ {
    UINT16 backup_rpt_en:5;     /* bit[4:0], backup status report enable */
#ifdef _BT5_0_LE2MBPS_SUPPORT_
    UINT16 TM_Midx:1;           /* bit[5], Test Mode Modulation Index. 0: standard(0.45~0x55
                                   1: stable(0.495~0.505). This is set for Receiver test.
                                   Currently Modem doesn't see this bit as it should be no
                                   difference receiving which index. */
    UINT16 TM_PhyRate:2;         /* bit[7:6], Test Mode Phy Rate. 
                                   0:1M; 1:2M; 2:encoded PHY(125k) */
#else
    UINT16 rsvd1:3;             /* bit[7:5], reserved */
#endif
    UINT16 test_mode_rf_ch:6;   /* bit[13:8], RF channel in test mode */
    UINT16 rsvd2:2;             /* bit[15:14], reserved */
} LE_REG_S_MODEM_CONTROL, *PLE_REG_S_MODEM_CONTROL;

/* The Structure of LE_REG_RF_TEST_CONTROL Register (0x138) */
typedef struct LE_REG_S_RF_TEST_CONTROL_ {
    UINT16 payload_type:4;      /* bit[3:0], test packet payload type */
    UINT16 prbs_mode:1;         /* bit[4], PRBS mode */
#ifdef _BT4_2_DATA_EXTENSION_FEATURE_SUPPORT_
    UINT16 prbs_fix:1;          /* bit[5], PRBS_Fix (if 1, every pkt are the same)*/
    UINT16 payload_len:8;       /* bit[13:6], test packet payload length */
#else
    UINT16 rsvd:3;              /* bit[7:5], reserved */
    UINT16 payload_len:6;       /* bit[13:8], test packet payload length */
#endif
    UINT16 rf_test_rx:1;        /* bit[14], 1 is RX mode and 0 is TX mode */
    UINT16 rf_test_mode_en:1;   /* bit[15], RF test mode enable */
} LE_REG_S_RF_TEST_CONTROL, *PLE_REG_S_RF_TEST_CONTROL;

/* The Structure of LE_REG_AES_FW_ENC_CTRL Register (0x13E) */
typedef struct LE_REG_S_AES_FW_ENC_CTRL_ {
    UINT16 enc_start:1;         /* bit[0], encrypted start */
    UINT16 enc_done:1;          /* bit[1], encrypted done */
    UINT16 rsvd:14;             /* but[15:2], reserved */
} LE_REG_S_AES_FW_ENC_CTRL, *PLE_REG_S_AES_FW_ENC_CTRL;

#ifdef _NEW_BLE_HW_SPEC_FROM_150320_
/* The Structure of LE_REG_INST_RPT Register (0x170), Read Only */
typedef union LE_REG_S_INST_RPT_ {
    struct {
    UINT16 comm_conn_kill:1;    /* bit[0] */
    UINT16 comm_adv_cancel:1;   /* bit[1] */
    UINT16 comm_scan_cancel:1;  /* but[2] */
    UINT16 comm_adv_req:1;      /* but[3] 1: idle */
    UINT16 comm_ch_upd:1;       /* but[4] */
    UINT16 comm_scan_req:1;     /* but[5] 1: idle */
    UINT16 comm_conn_upd:1;     /* but[6] */
    UINT16 comm_conn_cancel:1;  /* but[7] */
    UINT16 comm_disconn_req:1;  /* but[8] */
    UINT16 comm_conn_req:1;     /* but[9] 1: idle */
    UINT16 scan_st_idle:1;      /* but[10] */
    UINT16 conn_st_idle:1;      /* but[11] */
    UINT16 adv_st_idle:1;       /* but[12] */
    UINT16 rsvd:3;              /* but[15:13], reserved */
    };
    UINT16 Word;
} LE_REG_S_INST_RPT, *PLE_REG_S_INST_RPT;


/* The Structure of LE_REG_ANCH_DIFF_RPT0 Register (0x172), Read Only */
typedef union LE_REG_S_ANCH_DIFF_RPT0_ {
    struct {
        UINT16 anch_diff_15_0:16;    /* bit[15:0] */
    };
    UINT16 Word;
} LE_REG_S_ANCH_DIFF_RPT0, *PLE_REG_S_ANCH_DIFF_RPT0;

/* The Structure of LE_REG_ANCH_DIFF_RPT1 Register (0x174), Read Only */
typedef union LE_REG_S_ANCH_DIFF_RPT1_ {
    struct {
        UINT16 anch_diff_17_16:2;    /* bit[1:0] */
        UINT16 ce_abort:1;           /* bit[2], ce end being force abort */
        UINT16 ce_end_rx_pkt_cnt:10; /* bit[12:3], all rx packet counts of ce end */
        UINT16 rsvd:3;               /* bit[15:13], reserved */
    };
    UINT16 Word;
} LE_REG_S_ANCH_DIFF_RPT1, *PLE_REG_S_ANCH_DIFF_RPT1;

/* The Structure of LE_REG_CE_EARLY_GUARD_TIME Register (0x176), RW */
typedef struct LE_REG_S_CE_EARLY_GUARD_TIME_ {
    UINT16 guard_time_us:10;    /* bit[9:0], range 0~624 us */
    UINT16 guard_time_slot:3;   /* bit[12:10], range 0~7 slots */
    UINT16 rsvd:3;              /* but[15:13], reserved */
} LE_REG_S_CE_EARLY_GUARD_TIME, *PLE_REG_S_CE_EARLY_GUARD_TIME;
#endif

#ifdef _BT4_2_PRIVACY_FEATURE_SUPPORT_
/* The Structure of LE_REG_PRIVATE_ADDR_CTRL (0x178), RW */
typedef struct LE_REG_S_PRIVACY_MISC0_ {
    UINT16 tx_seriden_idx:5;        /* bit[4:0], current using local irk  */
    UINT16 tx_seriden_val:1;        /* bit[5], local irk valid bit */
    UINT16 tx_seriden_done:1;       /* bit[6], local irk search done bit, FW polling this bit for read local_rpa_index and tx_seriden_val */
    UINT16 fw_peer_rpa_val:1;       /* bit[7], If 1 : Peer RPA is valid */
    UINT16 fw_local_rpa_val:1;      /* bit[8], If 1 : Local RPA is valid */
    UINT16 re_gen_rand:1;           /* bit[9], re-generaton random(prand) number */
    UINT16 rand_num_sel_peer:1;     /* bit[10], 0 -> use current random number for peer RPA, 1 -> use previous random number for peerl RPA */
    UINT16 rand_num_sel_local:1;    /* bit[11], 0 -> use current random number for local RPA, 1 -> use previous random number for local RPA */
    UINT16 comp_rpa_lbd_opt:1;      /* bit[12], if AdvA of CONNECT_REQ/SCAN_RSP is RPA, 0 -> resolve AdvA, 1 -> compare with AdvA of previous Tx PDU */
    UINT16 ext_scan_filter_policy:1;/* bit[13], extended scan filter policy enable bit */
    UINT16 tx_lreso_adr_sel:1;      /* bit[14], if own_address_type = 0x2 or 0x3, this bit set 1  */
    UINT16 Addr_Resol_En:1;         /* bit[15], address resolution enable */
} LE_REG_S_PRIVACY_MISC0;

/* The Structure of LE_REG_PRIVACY_MISC1 (0x1C2), RW */
typedef struct LE_REG_S_PRIVACY_MISC1_ {
    UINT16 entry_num_sel:2;         /* bit[1:0], 0: 4 entires, 1: 8 entries, 2: 16 entries */
    UINT16 force_bt40_opt:1;        /* bit[2], write 1 to make identity addr and NRPA bypass resolving list */
    UINT16 rsvd1:13;                /* bit[15:3], reserved */
} LE_REG_S_PRIVACY_MISC1;
#endif /* _BT4_2_PRIVACY_FEATURE_SUPPORT_ */

/* The Structure of LE Register Set */
typedef union LE_REG_S_SET_ {
    LE_REG_S_INSTRUCTION instruction;
    LE_REG_S_CBK_CONTROL cbk_ctrl;
    LE_REG_S_CONN_WIN_SIZE conn_win_size;
    LE_REG_S_CONN_LATENCY conn_latency;
    LE_REG_S_CONN_CH_MAP_H conn_ch_map_h;
    LE_REG_S_CONN_UPD_WIN_SIZE conn_upd_win_size;
    LE_REG_S_CONN_UPD_ENTRY conn_upd_entry;
    LE_REG_S_CH_UPD_MAP_H ch_upd_map_h;
    LE_REG_S_SCAN_CONTROL scan_ctrl;
    LE_REG_S_ADV_CONTROL adv_ctrl;
    LE_REG_S_CANCEL_ENTRY cancel_entry;
    LE_REG_S_SLAVE_WIN_WIDENING_L slave_win_widen_l;
    LE_REG_S_SLAVE_CONN_WIN_SIZE slave_win_size;
    LE_REG_S_SLAVE_CONN_LATENCY slave_conn_latency;
    LE_REG_S_SLAVE_CH_MAP_H slave_ch_map_h;
    LE_REG_S_CAM_ACCESS cam_access;
    LE_REG_S_CE_RX_STATUS ce_rx_status;
    LE_REG_S_CE_TX_STATUS ce_tx_status;
    LE_REG_S_INT_MISR int_misr;
    LE_REG_S_INT_IMR int_imr;
    LE_REG_S_RANDOM_NUM_CTRL random_num_ctrl;
    LE_REG_S_STATUS_RX_THRES_CTRL status_rx_thres_ctrl;
    LE_REG_S_STATUS_RX_THRES_STATUS status_rx_thres_status;
    LE_REG_S_STATUS_ADV_STATUS status_adv_status;
    LE_REG_S_STATUS_TERMINATE_REASON status_term_reason;
    LE_REG_S_STATUS_CONN_STATUS status_conn_status;
    LE_REG_S_STATUS_CE_BEGIN_STATUS status_ce_begin_status;
    LE_REG_S_STATUS_CE_END_EVENT status_ce_end_event;
    LE_REG_S_STATUS_RX_INT_STACK status_rx_int_stack;
    LE_REG_S_STATUS_TX_THR_STATUS status_tx_thres;
    LE_REG_S_CE_END_CTRL ce_end_ctrl;
    LE_REG_S_RX_NAK_NUM rx_nak_num;
    LE_REG_S_DELAY_TRX_ON delay_trx_on;
    LE_REG_S_DELAY_RX_TURNAROUND delay_rx_turnaround;
    LE_REG_S_DELAY_TX_TURNAROUND delay_tx_turnaround;
    LE_REG_S_DELAY_CE_RX_TIMEOUT delay_ce_rx_timeout;
    LE_REG_S_DEVA_TYPE deva_type;
    LE_REG_S_RX_PKT_ERR_CTRL rx_pkt_err_ctrl;
    LE_REG_S_MODEM_CONTROL modem_ctrl;
    LE_REG_S_RF_TEST_CONTROL rf_test_ctrl;
    LE_REG_S_AES_FW_ENC_CTRL aes_fw_enc_ctrl;
    LE_REG_S_LE_NO_ACL le_no_acl;
    LE_REG_S_LE_EXT_MISR le_ext_misr;
#ifdef _DAPE_SUPPORT_LE_TX_PKT_MODE
    LE_REG_S_LE_TX_TEST_CNT_MODE le_tx_test_cnt_mode_ctrl;
#endif
#ifdef _NEW_BLE_HW_SPEC_FROM_150320_
    LE_REG_S_INST_RPT inst_rpt;
    LE_REG_S_ANCH_DIFF_RPT0 anch_diff_rpt0;
    LE_REG_S_ANCH_DIFF_RPT1 anch_diff_rpt1;
    LE_REG_S_CE_EARLY_GUARD_TIME ce_early_guard_time;
#endif
#ifdef _BT4_2_PRIVACY_FEATURE_SUPPORT_
    LE_REG_S_PRIVACY_MISC0 le_privacy_misc0;
    LE_REG_S_PRIVACY_MISC1 le_privacy_misc1;
#endif /* _BT4_2_PRIVACY_FEATURE_SUPPORT_ */
    UINT16 value;
} LE_REG_S_SET, *PLE_REG_S_SET;

/*===========================================================*/
/*  The Data Structure of LE CAM Register (32 bits)          */
/*===========================================================*/
/* The Structure of LE_CAM_ADDR_0 Register */
typedef union LE_CAM_ADDR_0_S_ {
    struct {
        UINT32 state:4;         /* bit[3:0], connection built state */
        UINT32 anchor_point:17; /* bit[20:4], current anchor point */
        UINT32 hop_inc:5;       /* bit[25:21], hop increment value */
        UINT32 last_hop:6;      /* bit[31:26], last hop value */
    };
    UINT32 DWord;
} LE_CAM_ADDR_0_S, *PLE_CAM_ADDR_0_S;

/* The Structure of LE_CAM_ADDR_1 Register */
typedef union LE_CAM_ADDR_1_S_ {
    struct {
        UINT32 ce_length:16;    /* bit[15:0], CE length (unit:1.25ms) */
        UINT32 ce_interval:16;  /* bit[31:16], CE Interval (unit:625us) */
    };
    UINT32 DWord;
} LE_CAM_ADDR_1_S, *PLE_CAM_ADDR_1_S;

/* The Structure of LE_CAM_ADDR_4 Register */
typedef union LE_CAM_ADDR_4_S_ {
    struct {
        UINT32 ch_map_h:5;      /* bit[4:0], channel map [36:32] */
        UINT32 encrypt:1;       /* bit[5], encryption (yes or no) */
        UINT32 decrypt:1;       /* bit[6], decryption (yes or no) */
        UINT32 sn:1;            /* bit[7], sn bit (update at the end of CE) */
        UINT32 nesn:1;          /* bit[8], nesn bit (update at the end of CE)*/
        UINT32 rsvd:5;          /* bit[13:9], reserved */
        UINT32 fail_cnt:3;      /* bit[16:14], connection created state
                                                transaction failure count */
        UINT32 ce_count:15;     /* bit[31:17], connection event counter[14:0],
                                                update at the end of CE */
    };
    UINT32 DWord;
} LE_CAM_ADDR_4_S, *PLE_CAM_ADDR_4_S;

/* The Structure of LE_CAM_ADDR_5 Register */
typedef union LE_CAM_ADDR_5_S_ {
    struct {
        UINT32 ce_count:1;      /* bit[0], connection event counter [15] */
        UINT32 crc_init:24;     /* bit[24:1], the init value of CRC */
        UINT32 enc_pkt_cnt:7;   /* bit[31:25], encryption packet count [6:0] */
    };
    UINT32 DWord;
} LE_CAM_ADDR_5_S, *PLE_CAM_ADDR_5_S;

/* The Structure of LE_CAM_ADDR_14 Register */
typedef union LE_CAM_ADDR_14_S_ {
    struct {
        UINT32 dec_pkt_cnt_h:7; /* bit[6:0], de-encryption packet count[38:32]*/
        UINT32 rsvd1:8;         /* bit[14:7], reserved for pcd/trx */
        UINT32 tx_power:8;      /* bit[22:15], transmit power (set with FW) */
        UINT32 null_txed:1;     /* bit[23], null_txed? */
#ifdef _NEW_BLE_HW_SPEC_FROM_150320_
        UINT32 no_crc:1;        /* bit[24], no crc packet */
        UINT32 mic_err_flag:1;  /* bit[25], mic error flag */
        UINT32 no_trig:1;       /* bit[26], no trigger */
        UINT32 len_ext_en:1;    /* bit[27], length extension enable bit */
        UINT32 seg_rd_ptr:4;    /* bit[31:28]*/
#else
        UINT32 rsvd2:8;         /* bit[31:24], reserved */
#endif

    };
    UINT32 DWord;
} LE_CAM_ADDR_14_S, *PLE_CAM_ADDR_14_S;


/* The Structure of LE_CAM_ADDR_15 Register (slave only) */
typedef union LE_CAM_ADDR_15_S_ {
    struct {
        UINT32 ww_val:26;       /* bit[25:0], window widening value [25:10]slot + [9:0]usec */
        UINT32 rsvd:6;          /* bit[31:26], reserved */
    };
    UINT32 DWord;
} LE_CAM_ADDR_15_S, *PLE_CAM_ADDR_15_S;

/* The Structure of LE_CAM_ADDR_16 Register (slave only) */
typedef union LE_CAM_ADDR_16_S_ {
    struct {
        UINT32 ww_val_tail:12;    /* bit[11:0], window widening value tail [11:0] */
        UINT32 ww_val_tail_add:6; /* bit[17:12], window widening value tail add */
        UINT32 ap_backup_b0:1; /* bit[18]: anchor point backup bit 0 */
        UINT32 rsvd:4;            /* bit[22:19], reserved */
        UINT32 slave_latency:9;   /* bit[31:23], slave latency */
    };
    UINT32 DWord;
} LE_CAM_ADDR_16_S, *PLE_CAM_ADDR_16_S;

/* The Structure of LE_CAM_ADDR_17 Register (slave only) */
typedef union LE_CAM_ADDR_17_S_ {
    struct {
        UINT32 ap_int:18;         /* bit[17:0], internal anchor point [17:0] */
        UINT32 rsvd:14;           /* bit[31:18], reserved */
    };
    UINT32 DWord;
} LE_CAM_ADDR_17_S, *PLE_CAM_ADDR_17_S;


/* The Structure of LE_CAM_ADDR_WHITE_LIST_HW or LE_CAM_ADDR_BLACK_LIST_HW */
typedef union LE_CAM_ADDR_LIST_HW_S_ {
    struct {
        UINT32 bd_addr:16;      /* bit[15:0], the bit[47:32] of BD_ADDR */
        UINT32 addr_type:1;     /* bit[16], 0/1: public/random address type */
        UINT32 valid:1;         /* bit[17], this address is valid ? */
        UINT32 duplicated:1;    /* bit[18], this address is duplicated ? */
        UINT32 rsvd:13;         /* bit[31:19], reserved */
    };
    UINT32 DWord;
} LE_CAM_ADDR_LIST_HW_S, *PLE_CAM_ADDR_LIST_HW_S;


/* The Structure of LE_CAM for One Connection Entry */
typedef union LE_CAM_ADDR_ENTRY_UNIT_S_ {
    struct {
      LE_CAM_ADDR_0_S word0;            /* addr 0 */
      LE_CAM_ADDR_1_S word1;            /* addr 1 */
      UINT32 access_address;            /* addr 2 */
      UINT32 map_table_low;             /* addr 3 */
      LE_CAM_ADDR_4_S word4;            /* addr 4 */
      LE_CAM_ADDR_5_S word5;            /* addr 5 */
      UINT32 enc_pkt_cnt_high;          /* addr 6 */
      UINT32 iv_low;                    /* addr 7 - fw fill */
      UINT32 iv_high;                   /* addr 8 - fw fill */
      UINT32 aes_session_key_w0;        /* addr 9 - fw fill */
      UINT32 aes_session_key_w1;        /* addr 10 - fw fill */
      UINT32 aes_session_key_w2;        /* addr 11 - fw fill */
      UINT32 aes_session_key_w3;        /* addr 12 - fw fill */
      UINT32 dec_pkt_cnt_low;           /* addr 13 */
      LE_CAM_ADDR_14_S word14;          /* addr 14 - fw fill tx power */
    };
    UINT32 addr[15];
} LE_CAM_ADDR_ENTRY_UNIT_S, *PLE_CAM_ADDR_ENTRY_UNIT_S;

/* The Structure of LE_CAM for One Device Address Unit */
typedef union LE_CAM_ADDR_LIST_DEV_UNIT_S_ {
    struct {
      UINT32 bd_addr_low;               /* addr 0 - fw fill */
      LE_CAM_ADDR_LIST_HW_S word1;      /* addr 1 - fw fill */
    };
    UINT32 addr[2];
} LE_CAM_ADDR_LIST_DEV_UNIT_S, *PLE_CAM_ADDR_LIST_DEV_UNIT_S;

/* The Structure of LE_CAM Layout */
typedef struct LE_CAM_ADDR_LAYOUT_S_ {
    LE_CAM_ADDR_ENTRY_UNIT_S     entry[LE_CAM_CONN_ENTRY_NUM];
    LE_CAM_ADDR_LIST_DEV_UNIT_S  white[LE_CAM_WHITE_LIST_NUM];
    LE_CAM_ADDR_LIST_DEV_UNIT_S  black[LE_CAM_BLACK_LIST_NUM];
} LE_CAM_ADDR_LAYOUT_S, *PLE_CAM_ADDR_LAYOUT_S;

#endif

