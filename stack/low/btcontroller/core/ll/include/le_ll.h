/***************************************************************************
 Copyright (C) Realtek
 This module is a confidential and proprietary property of Realtek and
 a possession or use of this module requires written permission of Realtek.
 ***************************************************************************/

/**
 * \file
 *  le_ll.h (Link Layer Definition)
 *
 * \author
 *  austin <austin_chen@realtek.com>, (C) 2010
 */

/** \addtogroup Low Energy Link Layer Driver Module
 *  @{ */
#ifndef __LE_LL_H__
#define __LE_LL_H__

#include "DataType.h"
#include "bt_fw_os.h"
#include "bt_fw_hci_external_defines.h"
#include "le_ll_driver.h"
#include "timer.h"
#include "lc_internal.h"
#include "bzdma.h"

/*-------------------------------------------------------------*/
/* Link Layer Spec Definition                                  */
/*-------------------------------------------------------------*/

/* Definitaion of Access Address */
#define LL_ACCESS_ADDRESS_OF_ADV_CH     0x8E89BED6

/* 0xA28F33A0 + diff = 101xxx10-10001111-00110011-1010yyyyB
    x(3 bit: 0~7), y(4 bit: 0~15), transition count: 13~21 */
#define LL_FW_ACCESS_ADDRESS_INIT0       0xA28F33A0

/* 0x7909ACC1 + diff = 01111001-0xxx1001-10101100-110yyyy1B
    x(3 bit: 0~7), y(4 bit: 0~15), transition count: 15~21 */
#define LL_FW_ACCESS_ADDRESS_INIT1      0x7909ACC1

/* 0x16E609B6 + diff = xxx10110-11100110-yyyy1001-10110110B
    x(3 bit: 0~7), y(4 bit: 0~15), transition count: 15~22 */
#define LL_FW_ACCESS_ADDRESS_INIT2      0x16E609B6

/* 0x6CC87506 + diff = 01101100-11001xxx-01110101-yyyy0110B
    x(3 bit: 0~7), y(4 bit: 0~15), transition count: 16~22 */
#define LL_FW_ACCESS_ADDRESS_INIT3      0x6CC87506

#define LE_DEAULT_HW_TX_POWER           le_tx_power_max_value
#define LE_DEAULT_HW_TX_INDEX           le_tx_power_max_index
#define LE_DEAULT_ADV_TX_POWER          le_tx_power_max_value
#define LE_DEAULT_ADV_TX_INDEX          le_tx_power_max_index

enum LL_CONNECTION_ROLE {
    LL_SLAVE  = 0,
    LL_MASTER = 1,
};

/* The LL Physical Channel Type */
enum LL_CHANNEL_TYPE_SET {
    LL_CHANNEL_TYPE_ADVERTISING = 0,
    LL_CHANNEL_TYPE_DATA        = 1
};

/* THe LL Linked List Insert Policy */
enum LL_LINKED_LIST_INSERT_POLICY {
    LL_LINKED_LIST_INSERT_POLICY_HEAD,
    LL_LINKED_LIST_INSERT_POLICY_TAIL
};

/* THe LL Linked List Type */
enum LL_LINKED_LIST_TYPES {
    LL_LINKED_LIST_TYPE_TX_PEND,
    LL_LINKED_LIST_TYPE_TX_SCHED,
    LL_LINKED_LIST_TYPE_TX_FREE
};

/*====================================================================*/
/* The State Machine of Link Layer                                    */
/*====================================================================*/
enum LL_STATES {
    LL_STATE_STANDBY            = 0x00,
    LL_STATE_ADVERTISING        = 0x01,
    LL_STATE_SCANNING           = 0x02,
    LL_STATE_INITIATING         = 0x04,
    LL_STATE_CONNECTION_MASTER  = 0x08,
    LL_STATE_CONNECTION_SLAVE   = 0x10
};

/*====================================================================*/
/* Table 2.1 Advertising Channel PDU Header's PDU Type field encoding */
/*====================================================================*/
enum LL_ADV_CH_PDU_TYPE_CODE {
    LL_ADV_PDU_TYPE_ADV_IND             = 0x00,
    LL_ADV_PDU_TYPE_ADV_DIRECT_IND      = 0x01,
    LL_ADV_PDU_TYPE_ADV_NONCONN_IND     = 0x02,
    LL_ADV_PDU_TYPE_SCAN_REQ            = 0x03,
    LL_ADV_PDU_TYPE_SCAN_RSP            = 0x04,
    LL_ADV_PDU_TYPE_CONNECT_REQ         = 0x05,
    LL_ADV_PDU_TYPE_ADV_DISCOVER_IND    = 0x06,
    LL_ADV_PDU_TYPE_MAX                 = LL_ADV_PDU_TYPE_ADV_DISCOVER_IND
};

/*====================================================================*/
/* Table 2.2 SCA Field Encoding                                       */
/*====================================================================*/
enum LL_SCA_ENCODING {
    LL_SCA_251_TO_500_PPM   = 0,
    LL_SCA_151_TO_250_PPM   = 1,
    LL_SCA_101_TO_150_PPM   = 2,
    LL_SCA_76_TO_100_PPM    = 3,
    LL_SCA_51_TO_75_PPM     = 4,
    LL_SCA_31_TO_50_PPM     = 5,
    LL_SCA_21_TO_30_PPM     = 6,
    LL_SCA_0_TO_20_PPM      = 7
};

/*====================================================================*/
/* Table 2.3 LLID Defination                                          */
/*====================================================================*/
enum LL_LLID_SET {
    LL_LLID_RSVD            = 0,
    LL_LLID_DATA_PDU_CONT   = 1,
    LL_LLID_EMPTY_PDU       = 1,
    LL_LLID_DATA_PDU_START  = 2,
    LL_LLID_CTRL_PDU        = 3
};

/*====================================================================*/
/* Table 2.4 LL Control PDU OpCodes                                   */
/*====================================================================*/
enum LL_CTRL_PDU_OPCODES {
    LL_CONNECTION_UPDATE_REQ = 0x00,
    LL_CHANNEL_MAP_REQ       = 0x01,
    LL_TERMINATE_IND         = 0x02,
    LL_ENC_REQ               = 0x03,
    LL_ENC_RSP               = 0x04,
    LL_START_ENC_REQ         = 0x05,
    LL_START_ENC_RSP         = 0x06,
    LL_UNKNOWN_RSP           = 0x07,
    LL_FEATURE_REQ           = 0x08,
    LL_FEATURE_RSP           = 0x09,
    LL_PAUSE_ENC_REQ         = 0x0A,
    LL_PAUSE_ENC_RSP         = 0x0B,
    LL_VERSION_IND           = 0x0C,
    LL_REJECT_IND            = 0x0D,
#ifdef _SUPPORT_VER_4_1_
    LL_SLAVE_FEATURE_REQ_IND = 0x0E,
    LL_CONNECTION_PARAM_REQ  = 0x0F,
    LL_CONNECTION_PARAM_RSP  = 0x10,
    LL_REJECT_IND_EXT        = 0x11,
    LL_PING_REQ              = 0x12,
    LL_PING_RSP              = 0x13,
#endif
    LL_LENGTH_REQ            = 0x14,
    LL_LENGTH_RSP            = 0x15,
    LL_CTRL_PDU_MAX          = LL_LENGTH_RSP,
};

/* The Length of LL Control PDU (Opcode(1B) + CtrlData (0~22B))*/
enum LL_CTRL_PDU_LEN_SET {
    LL_CONNECTION_UPDATE_REQ_LEN    = 12,
    LL_CHANNEL_MAP_REQ_LEN          = 8,
    LL_TERMINATE_IND_LEN            = 2,
    LL_ENC_REQ_LEN                  = 23,
    LL_ENC_RSP_LEN                  = 13,
    LL_START_ENC_REQ_LEN            = 1,
    LL_START_ENC_RSP_LEN            = 1,
    LL_UNKNOWN_RSP_LEN              = 2,
    LL_FEATURE_REQ_LEN              = 9,
    LL_FEATURE_RSP_LEN              = 9,
    LL_PAUSE_ENC_REQ_LEN            = 1,
    LL_PAUSE_ENC_RSP_LEN            = 1,
    LL_VERSION_IND_LEN              = 6,
    LL_REJECT_IND_LEN               = 2,
#ifdef _SUPPORT_VER_4_1_
    LL_SLAVE_FEATURE_REQ_LEN        = 9,
    LL_CONNECTION_PARAM_REQ_LEN     = 24,
    LL_CONNECTION_PARAM_RSP_LEN     = 24,
    LL_REJECT_IND_EXT_LEN           = 3,
    LL_PING_REQ_LEN                 = 1,
    LL_PING_RSP_LEN                 = 1,
#endif
#ifdef _BT4_2_DATA_EXTENSION_FEATURE_SUPPORT_
    LL_LENGTH_REQ_LEN               = 9,
    LL_LENGTH_RSP_LEN               = 9,
#endif
};

/* LL Address Type */
enum LL_ADDR_TYPE_SET {
    LL_ADDR_TYPE_PUBLIC,
    LL_ADDR_TYPE_RANDOM,
#ifdef _BT4_2_PRIVACY_FEATURE_SUPPORT_
    LL_ADDR_TYPE_RPA_PUBLIC,
    LL_ADDR_TYPE_RPA_RANDOM,
    LL_ADDR_TYPE_MAX = LL_ADDR_TYPE_RPA_RANDOM,
#else
    LL_ADDR_TYPE_MAX = LL_ADDR_TYPE_RANDOM,
#endif
};

/* LE Normal or Test Mode Definition */
enum LL_MODE_SET {
    LL_MODE_NORMAL   = 0,
    LL_MODE_RX_TEST  = 1,
    LL_MODE_TX_TEST  = 2,
};

/* LE Packet Type Definition */
enum LL_PKT_TYPE {
    LL_PKT_TYPE_LLC_PDU   = 0,
    LL_PKT_TYPE_ACL_DATA  = 1,
};

/* LLC Procedure Type */
enum LLC_PROCEDURE_TYPE_SET {
    LLC_PROCEDURE_TYPE_NONE = 0,
    LLC_PROCEDURE_TYPE_CONN_UPDT = 1,
    LLC_PROCEDURE_TYPE_CHMAP_UPDT = 2,
    LLC_PROCEDURE_TYPE_ENCRYPTION_START = 3,
    LLC_PROCEDURE_TYPE_ENCRYPTION_PAUSE = 4,
    LLC_PROCEDURE_TYPE_FEATURE_EXCHANGE = 5,
    LLC_PROCEDURE_TYPE_VERSION_EXCHANGE = 6,
    LLC_PROCEDURE_TYPE_TERMINATION = 7,
    LLC_PROCEDURE_TYPE_DATA_LENGTH_UPDATE = 8,
    LLC_PROCEDURE_TYPE_CONN_PARAM_REQ = 9,
    LLC_PROCEDURE_TYPE_LE_PING_REQ = 10

};

/* LLC Encryption States for three way handshakes */
enum LLC_START_ENC_THREE_WAY_STATE {
    LLC_START_ENC_STATE_IDLE = 0,
    LLC_START_ENC_STATE_BEGIN = 1,
    LLC_START_ENC_STATE_S2M_START_ENC_REQ = 2,
    LLC_START_ENC_STATE_M2S_START_ENC_RSP = 3,
    LLC_START_ENC_STATE_S2M_START_ENC_RSP = 4,
    LLC_START_ENC_STATE_END = LLC_START_ENC_STATE_S2M_START_ENC_RSP,
};

enum LLC_PAUSE_ENC_THREE_WAY_STATE {
    LLC_PAUSE_ENC_STATE_IDLE = 0,
    LLC_PAUSE_ENC_STATE_BEGIN = 1,
    LLC_PAUSE_ENC_STATE_M2S_PAUSE_ENC_REQ = 2,
    LLC_PAUSE_ENC_STATE_S2M_PAUSE_ENC_RSP = 3,
    LLC_PAUSE_ENC_STATE_M2S_PAUSE_ENC_RSP = 4,
    LLC_PAUSE_ENC_STATE_END = LLC_PAUSE_ENC_STATE_M2S_PAUSE_ENC_RSP,
};

enum LL_HANDLE_RECD_PACKET_TYPE_SET {
    LL_HANDLE_RECD_ADV_PACKET_TYPE,
    LL_HANDLE_RECD_DATA_PACKET_TYPE,
    LL_HANDLE_RECD_EXCEPTION_TYPE,
    LL_HANDLE_RECD_DATA_PACKET_MIC_ERROR_TYPE
};

enum LL_TASK_SUB_TYPES {
    LL_TASK_HANDLE_CONN_COMP,
    LL_TASK_HANDLE_DISCONN,
    LL_TASK_HANDLE_LONG_TERM_KEY_REQUEST_REPLY,
    LL_TASK_HANDLE_ENCRYPTION_KEY_REFRESH,
    LL_TASK_HANDLE_ENCRYPTION_CHANGE_EVENT,
    LL_TASK_HANDLE_CONN_UPDT_COMP,
    LL_TASK_HANDLE_ENHANCED_CONN_COMP,
    LL_TASK_HANDLE_READ_LOCAL_P256_PUBLIC_KEY_COMP,
    LL_TASK_HANDLE_GENERATE_DHKEY_COMP,
};

enum LL_TIMER_TYPE_ {
    LL_TIMER_TYPE_SUPERVISION,
    LL_TIMER_TYPE_LLC,
    LL_TIMER_TYPE_TERMINATE,
    LL_TIMER_TYPE_PING
};

enum LL_KILL_LINK_REASON_ {
    LL_KILL_REASON_SUPERVISION_TIMEOUT,
    LL_KILL_REASON_LLC_TIMEOUT,
    LL_KILL_REASON_UPDT_PAST,
    LL_KILL_REASON_MIC_ERROR,
#ifdef FIX_LE_HW_NO_BD_ADDR_CHECK
    LL_KILL_REASON_EXISTED_BD_ADDR,
#endif
};

enum LL_UPDATE_REASON_ {
    LL_UPDATE_REASON_CONN_PARAM,
    LL_UPDATE_REASON_STOP_SUPERVISION_TIMER,
    LL_UPDATE_REASON_CH_MAP,
};

#define LL_SUPERVISION_SW_TIMER_UNIT            100
#define LL_LLC_SW_TIMER_UINT                    200
#define LL_LLC_PROCEDURE_RESPONSE_TIMEOUT       4000 /* unit: 10 ms */

#define LL_RF_TIMER_SLOT_UNIT            625 /* unit: us */

#define LL_RF_TX_SETTLING_TIME           200 /* unit: us, for RTK RF */
#define LL_ADV_H2H_FW_ADJ_OFFSET         0   /* maybe we put adapted value
                                                in efuse */
/* +--------+    +---------+    +---------+                         +--------+
   | AdvPdu |TIFS| ScanReq |TIFS| ScanRsp |                         | AdvPdu |
   +--------+    +---------+    +---------+                         +--------+
   |<-------         duration     ------->|<- FW ADJ ->|<-- RF TX ->|
   |                                      |   Offset   |Setting Time|
   |<------------------ adv_h2h_period ---------------------------->|
*/

/* T_IFS = 150 us */
#define LL_T_IFS                         150 /* unit: us */

#define LL_PREAMBLE_SIZE                 1
#define LL_AA_SIZE                       4
#define LL_ADV_CH_PDU_HEADER_SIZE        2
#define LL_ADV_CH_PAYLOAD_LEN_MIN        6
#define LL_ADV_CH_PAYLOAD_LEN_MAX        37
#define LL_CRC_SIZE                      3
#define LL_ADV_CH_NON_PAYLOAD_SIZE       10 /* LL_PREAMBLE_SIZE + LL_AA_SIZE +
                                               LL_ADV_CH_PDU_HEADER_SIZE +
                                               LL_CRC_SIZE */

#define LL_ADV_CH_PDU_SIZE_MIN        \
                    (LL_ADV_CH_NON_PAYLOAD_SIZE + LL_ADV_CH_PAYLOAD_LEN_MIN)
#define LL_ADV_CH_PDU_SIZE_MAX       \
                    (LL_ADV_CH_NON_PAYLOAD_SIZE + LL_ADV_CH_PAYLOAD_LEN_MAX)

#define LL_ADV_IND_SIZE_MIN             (LL_ADV_CH_NON_PAYLOAD_SIZE + 6)
#define LL_ADV_IND_SIZE_MAX             (LL_ADV_CH_NON_PAYLOAD_SIZE + 37)
#define LL_ADV_DIRECT_SIZE              (LL_ADV_CH_NON_PAYLOAD_SIZE + 12)
#define LL_ADV_NONCONN_IND_SIZE_MIN     (LL_ADV_CH_NON_PAYLOAD_SIZE + 6)
#define LL_ADV_NONCONN_IND_SIZE_MAX     (LL_ADV_CH_NON_PAYLOAD_SIZE + 37)
#define LL_ADV_DISCOVER_IND_SIZE_MIN    (LL_ADV_CH_NON_PAYLOAD_SIZE + 6)
#define LL_ADV_DISCOVER_IND_SIZE_MAX    (LL_ADV_CH_NON_PAYLOAD_SIZE + 37)
#define LL_SCAN_REQ_SIZE                (LL_ADV_CH_NON_PAYLOAD_SIZE + 12)
#define LL_SCAN_RSP_SIZE_MIN            (LL_ADV_CH_NON_PAYLOAD_SIZE + 6)
#define LL_SCAN_RSP_SIZE_MAX            (LL_ADV_CH_NON_PAYLOAD_SIZE + 37)
#define LL_CONNECTION_REQ_SIZE          (LL_ADV_CH_NON_PAYLOAD_SIZE + 34)

#define LL_ADV_IND_PL_SIZE_MIN           6
#define LL_ADV_IND_PL_SIZE_MAX           37
#define LL_ADV_DIRECT_PL_SIZE            12
#define LL_ADV_NONCONN_IND_PL_SIZE_MIN   6
#define LL_ADV_NONCONN_IND_PL_SIZE_MAX   37
#define LL_ADV_DISCOVER_IND_PL_SIZE_MIN  6
#define LL_ADV_DISCOVER_IND_PL_SIZE_MAX  37
#define LL_SCAN_REQ_PL_SIZE              12
#define LL_SCAN_RSP_PL_SIZE_MIN          6
#define LL_SCAN_RSP_PL_SIZE_MAX          37
#define LL_CONNECTION_REQ_PL_SIZE        34

#define LE_ALLOW_RX_WORD_CNT_THRESHOLD   (LE_HW_ADV_RX_PKT_MIN_SIZE >> 1)
#define LE_ALLOW_TX_WORD_CNT_THRESHOLD   1

/* advertising interval (unit : 625us) */
#define LL_ADV_INTERVAL_MIN                 0x0020  /* 20 ms */
#define LL_ADV_INTERVAL_NONCONN_MIN         0x00A0  /* 100 ms */
#define LL_ADV_INTERVAL_MAX                 0x4000  /* 10.24 sec */
#define LL_DEFAULT_ADV_INTERVAL             0x0800  /* 1.28 sec */
#define LL_INDIRECT_ADV_H2H_MAX             0x0010  /* 10 ms */

#define LL_DEFAULT_ADV_TYPE                 LL_ADV_PDU_TYPE_ADV_IND
#define LL_DEFAULT_ADV_ADDR_TYPE            LL_ADDR_TYPE_PUBLIC
#define LL_DEFAULT_ADV_CH_MAP               0x07     /* enable ch 37,38,39 */

#define LL_ADV_DATA_LEN_MAX                 31
#define LL_SCAN_RESPONSE_DATA_LEN_MAX       31

#define LL_DIRECT_ADV_TIMEOUT_MAX           0x0800  /* 1.28 sec (unit:625us)*/

/* scanning interval and window (unit : 625us) */
#define LL_SCAN_INTERVAL_MIN                0x0004  /* 2.5 ms */
#define LL_SCAN_INTERVAL_MAX                0x4000  /* 10240 ms */
#define LL_DEFAULT_SCAN_INTERVAL            0x0010  /* 10 ms */
#define LL_SCAN_WINDOW_MIN                  0x0004  /* 2.5 ms */
#define LL_SCAN_WINDOW_MAX                  0x4000  /* 10240 ms */
#define LL_DEFAULT_SCAN_WINDOW              0x0010  /* 10 ms */

#define LL_DEFAULT_SCAN_ADDR_TYPE           LL_ADDR_TYPE_PUBLIC
#define LL_DEFAULT_SCAN_CH_MAP              0x07

/* initiator scan interval and window (unit : 625us) */
#define LL_INITIATOR_SCAN_INTERVAL_MIN      0x0004 /* 2.5 ms */
#define LL_INITIATOR_SCAN_INTERVAL_MAX      0x4000 /* 10240 ms */
#define LL_DEFAULT_INITIATOR_SCAN_INTERVAL  0x0010 /* 10 ms */
#define LL_INITIATOR_SCAN_WINDOW_MIN        0x0004 /* 2.5 ms */
#define LL_INITIATOR_SCAN_WINDOW_MAX        0x4000 /* 10240 ms */
#define LL_DEFAULT_INITIATOR_SCAN_WINDOW    0x0010 /* 10 ms */

/* connection interval (unit: 1.25ms) */
#define LL_CONN_INTERVAL_MIN                0x0006 /* 7.5 ms */
#define LL_CONN_INTERVAL_MAX                0x0C80 /* 4 sec */
#define LL_DEFAULT_CONN_INTERVAL            LL_CONN_INTERVAL_MIN

/* LL Connection Event Transmit Window (unit: 1.25ms) */
/* LL_TX_WIN_OFFSET: in the range of 0 ms to connInterval
   LL_TX_WIN_SIZE: in the range of 1.25 ms to the lesser of 10 ms and
                   (connInterval - 1.25ms) */
#define LL_TX_WIN_OFFSET_MIN                0   /* 0 ms */
#define LL_TX_WIN_OFFSET_MAX                LL_CONN_INTERVAL_MAX
#define LL_TX_WIN_SIZE_MIN                  1   /* 1.25 ms */
#define LL_TX_WIN_SIZE_MAX                  8   /* 10 ms */

/* LL slave latency (unit: ce counts) */
#define LL_CONN_LATENCY_MIN                 0x0000
#define LL_CONN_LATENCY_MAX                 0x01F4
#define LL_DEFAULT_CONN_LATENCY             LL_CONN_LATENCY_MIN

/* LL supervision timeout (unit: 10ms) */
#define LL_SUPERVISION_TO_MIN               0x000A /* 100 ms */
#define LL_SUPERVISION_TO_MAX               0x0C80 /* 32 sec */
#define LL_DEFAULT_SUPERVISION_TO           LL_SUPERVISION_TO_MIN

/* LL CE Length (unit: 0.625 ms) */
#define LL_CE_LENGTH_MIN                    0x0000 /* 0 ms */
#define LL_CE_LENGTH_MAX                    0xFFFF /* 0.625 ms * 0xFFFF */
#define LL_DEFAULT_CE_LENGTH                LL_CE_LENGTH_MIN

#define LL_DEFAULT_INIT_CH_MAP              0x07 /* enable ch 37,38,39 */

#define LL_MAX_CONNECTION_UNITS             8
#define LL_MAX_CONNECTION_ENTRY_ID          (LL_MAX_CONNECTION_UNITS - 1)
#define LL_HCI_MAX_CONNECTION_HANDLE        0x0FFF

/* Data Channel PDU Parameter Definition */
#define LL_DATA_CH_PDU_HEADER_SIZE          2
#define LL_DATA_CH_ENC_PAYLOAD_LEN_MIN      0
#define LL_DATA_CH_ENC_PAYLOAD_LEN_MAX      31
#define LL_DATA_CH_UNENC_PAYLOAD_LEN_MIN    0
#define LL_DATA_CH_UNENC_PAYLOAD_LEN_MAX    27
#define LL_DATA_CH_MIC_SIZE                 4

/* LE HCI Relative Settings */
#define LL_HCI_H2C_MAX_ACL_PKT_CNT          LL_POLL_HCI_MAX_TX_ACL_PKT_CNT
#ifndef VER4_2
#define LL_HCI_H2C_MAX_ACL_PKT_SIZE         27
#else
#define LL_HCI_H2C_MAX_ACL_PKT_SIZE         251
#endif

#define LL_MAX_ACL_UNSECURITY_PKT_SIZE      31
#define LL_MAX_ACL_SECURITY_PKT_SIZE        27

#define LL_FW_MAX_ACL_PAYLOAD_SIZE          32

#define LL_CONTROL_PDU_BUFFER_NUM           LL_POLL_CONTROL_PDU_BUFFER_NUM
#define LL_CONTROL_PDU_BUFFER_SIZE          LL_POLL_CONTROL_PDU_BUFFER_SIZE

#define LL_MAX_WHITE_LIST_SIZE              32
#define LL_MAX_BLACK_LIST_SIZE              32
#define LL_MAX_HW_RESOLVING_LIST_SZ         16
#define LL_MAX_RESOLVING_LIST_SIZE          le_resolving_list_size
#define LL_HW_RESOLVING_LIST_ENTRY_SIZE     10 /* unit: 4 bytes */

#define LL_MIN_RF_CHANNEL_INDEX             0
#define LL_MAX_RF_CHANNEL_INDEX             39
#define LL_MIN_DATA_CHANNEL_INDEX           0
#define LL_MAX_DATA_CHANNEL_INDEX           36
#define LL_MIN_ADV_CHANNEL_INDEX            37
#define LL_MAX_ADV_CHANNEL_INDEX            39

#define LL_MIN_DATA_CHANNEL_NUMBER          2
#define LL_MAX_DATA_CHANNEL_NUMBER          37


#define LL_HOP_INCREMENT_MIN                5
#define LL_HOP_INCREMENT_MAX                16

#ifdef _BT4_2_DATA_EXTENSION_FEATURE_SUPPORT_
#define LL_MAX_TEST_DATA_LEN                0xFF
#else
#define LL_MAX_TEST_DATA_LEN                0x25
#endif


#define LE_NUM_OF_ADVERTISING_REPORT_MAX    4 //4 /* BITE only support one now */

extern UINT32 bt_le_multistate_bm_ldw;
extern UINT32 bt_le_multistate_bm_hdw;

/* default bitmap of supported mutiple states */
#define LL_MULTI_STATES_SUPPORTED_BITMAP_LOW_WORD     bt_le_multistate_bm_ldw
#define LL_MULTI_STATES_SUPPORTED_BITMAP_HIGH_WORD    bt_le_multistate_bm_hdw

#define LL_CONN_UPDT_REQ_RELATIVE_INSTANT   10   /* at least 6 */
#define LL_CH_MAP_REQ_RELATIVE_INSTANT      10   /* at least 6 */
#define LL_CONN_UPDT_REQ_RELATIVE_INSTANT_W_SLV_LATENCY   3   /* no spec */
#define LL_CH_MAP_REQ_RELATIVE_INSTANT_W_SLV_LATENCY      3   /* no spec */

// dape added
#define LL_CONN_TX_WIN_SIZE_OFFSET 1 /*tx or rx first packet n*1.25 ms after the start of win_size*/

#define LL_WHITE_LIST_TYPE                  0
#define LL_BLACK_LIST_TYPE                  1
#define LL_RESOLVING_LIST_TYPE              2

#define LL_MAX_HW_RX_PKT_COUNT              4

#define LL_LOCAL_SCA                        LL_SCA_251_TO_500_PPM

#define LL_CE_BEGIN_EARLY_OFFSET            800  /* unit : us*/
#if (LL_CE_BEGIN_EARLY_OFFSET >= 625)
#define LL_CE_BEGIN_EARLY_UNIT_US           (LL_CE_BEGIN_EARLY_OFFSET % 625)
#define LL_CE_BEGIN_EARLY_UNIT_SLOT         1
#else
#define LL_CE_BEGIN_EARLY_UNIT_US           LL_CE_BEGIN_EARLY_OFFSET
#define LL_CE_BEGIN_EARLY_UNIT_SLOT         0
#endif

/* LE Connecton Event Begin Early IRQ Mask Settings */

#define LL_CE_BEGIN_EARLY_IRQ_MASK          1000 /* unit : us */

#if (LL_CE_BEGIN_EARLY_IRQ_MASK < LL_CE_BEGIN_EARLY_OFFSET)
#error *** LE IRQ MASK is smaller than LE CE BEGSIn EARLY OFFSET ***
#endif

#if (LL_CE_BEGIN_EARLY_IRQ_MASK >= 625)
#define LL_CE_BEGIN_IRQ_MASK_UNIT_US        (LL_CE_BEGIN_EARLY_IRQ_MASK % 625)
#define LL_CE_BEGIN_IRQ_MASK_UNIT_SLOT       1
#else
#define LL_CE_BEGIN_IRQ_MASK_UNIT_US         LL_CE_BEGIN_EARLY_IRQ_MASK
#define LL_CE_BEGIN_IRQ_MASK_UNIT_SLOT       0
#endif
#define LL_CE_BEGIN_IRQ_MASK_SETTING        (((LL_CE_BEGIN_IRQ_MASK_UNIT_SLOT) << 10) | (LL_CE_BEGIN_IRQ_MASK_UNIT_US))

#define LL_HW_RXBUFF_SIZE               1024

/*======================================================================*/
/* Public Device Address Structure (Page 33 of 140) of LL Spec          */
/*======================================================================*/
typedef struct LL_PUBLIC_DEV_ADDR_ {
    UINT8 u1company_assigned[3]; /* byte[2:0], 24 least significant bits */
    UINT8 u1company_id[3];       /* byte[5:3], 24 most significant bits */
} LL_PUBLIC_DEV_ADDR, *PLL_PUBLIC_DEV_ADDR;

/*======================================================================*/
/* Random Device Address Structure (Page 34 of 140) of LL Spec          */
/*======================================================================*/
typedef struct LL_RANDOM_DEV_ADDR_ {
    UINT8 u1hash[3];          /* byte[2:0], 24 least significant bits */
    UINT8 u1random[3];        /* byte[5:3], 24 most significant bits */
} LL_RANDOM_DEV_ADDR, *PLL_RANDOM_DEV_ADDR;

/*======================================================================*/
/* Device Address Structure (Page 33 of 140) of LL Spec                 */
/*======================================================================*/
typedef union LL_DEV_ADDR_ {
    LL_PUBLIC_DEV_ADDR public; /* public device address */
    LL_RANDOM_DEV_ADDR random; /* random device address */
    UINT8 u1Addr[6];           /* all fields of device address */
} LL_DEV_ADDR, *PLL_DEV_ADDR;

/*======================================================================*/
/* Advertising Channel PDU Header Structure (Page 38 of 140) of LL Spec */
/*======================================================================*/
typedef struct LL_ADVERTISING_CH_PDU_HEADER_ {
    UINT16 PDU_Type:4;   /* bit[3:0], PDU Type */
    UINT16 rsvd1:2;      /* bit[5:4], reserved */
    UINT16 TxAdd:1;      /* bit[6], the type of transmitter's address */
    UINT16 RxAdd:1;      /* bit[7], the type of receiver's address */
    UINT16 Length:6;     /* bit[13:8], the length of payload */
    UINT16 rsvd2:2;      /* bit[15:14]. reserved */
} LL_ADVERTISING_CH_PDU_HEADER, *PLL_ADVERTISING_CH_PDU_HEADER;

/*===========================================================*/
/* ADV_IND PDU Payload Structure (Page 39 of 140) of LL Spec */
/*===========================================================*/
typedef struct LL_ADV_PDU_ADV_IND_PL_ {
    LL_DEV_ADDR AdvA;    /* the address of the advertiser */
    UINT8 u1AdvData[31]; /* The data (0~31 bytes) from the advertiser */
} LL_ADV_PDU_ADV_IND_PL, *PLL_ADV_PDU_ADV_IND_PL;

/*==================================================================*/
/* ADV_DIRECT_IND PDU Payload Structure (Page 40 of 140) of LL Spec */
/*==================================================================*/
typedef struct LL_ADV_PDU_ADV_DIRECT_IND_PL_ {
    LL_DEV_ADDR AdvA;    /* the address of the advertiser */
    LL_DEV_ADDR InitA;   /* the address of the initiator */
} LL_ADV_PDU_ADV_DIRECT_IND_PL, *PLL_ADV_PDU_ADV_DIRECT_IND_PL;

/*===================================================================*/
/* ADV_NONCONN_IND PDU Payload Structure (Page 40 of 140) of LL Spec */
/*===================================================================*/
typedef struct LL_ADV_PDU_ADV_NONCONN_IND_PL_ {
    LL_DEV_ADDR AdvA;    /* the address of the advertiser */
    UINT8 u1AdvData[31]; /* the data (0~31 bytes) from the advertiser */
} LL_ADV_PDU_ADV_NONCONN_IND_PL, *PLL_ADV_PDU_ADV_NONCONN_IND_PL;

/*====================================================================*/
/* ADV_DISCOVER_IND PDU Payload Structure (Page 41 of 140) of LL Spec */
/*====================================================================*/
typedef struct LL_ADV_PDU_ADV_DISCOVER_IND_PL_ {
    LL_DEV_ADDR AdvA;    /* the address of the advertiser */
    UINT8 u1AdvData[31]; /* the data (0~31 bytes) from the advertiser */
} LL_ADV_PDU_ADV_DISCOVER_IND_PL, *PLL_ADV_PDU_ADV_DISCOVER_IND_PL;

/*============================================================*/
/* SCAN_REQ PDU Payload Structure (Page 41 of 140) of LL Spec */
/*============================================================*/
typedef struct LL_ADV_PDU_SCAN_REQ_PL_ {
    LL_DEV_ADDR ScanA; /* the address of the scanner */
    LL_DEV_ADDR AdvA;  /* the address of the advertiser */
} LL_ADV_PDU_SCAN_REQ_PL, *PLL_ADV_PDU_SCAN_REQ_PL;

/*============================================================*/
/* SCAN_RSP PDU Payload Structure (Page 42 of 140) of LL Spec */
/*============================================================*/
typedef struct LL_ADV_PDU_SCAN_RSP_PL_ {
    LL_DEV_ADDR ScanA;       /* the address of the scanner */
    UINT8 u1ScanRspData[31]; /* the scan response data (0~31 bytes)
                                from the advertiser */
} LL_ADV_PDU_SCAN_RSP_PL, *PLL_ADV_PDU_SCAN_RSP_PL;

/*===============================================================*/
/* CONNECT_REQ PDU Payload Structure (Page 42 of 140) of LL Spec */
/*===============================================================*/
typedef struct LL_ADV_PDU_CONNECT_REQ_PL_ {
    LL_DEV_ADDR InitA;   /* the address of the initiator */
    LL_DEV_ADDR AdvA;    /* the address of the advertiser */
    union {
        UINT8 u1LLData[22];
        struct {
            UINT8 u1AA[4];        /* access address */
            UINT8 u1CRCInit[3];   /* the initialization value of CRC */
            UINT8 WinSize;        /* transmiWindowSize = WinSize*1.25ms */
            UINT8 u1WinOffset[2]; /* transmitWindowOffset = WinOffset*1.25ms */
            UINT8 u1Interval[2];  /* connInterval = Interval*1.25ms */
            UINT8 u1Latency[2];   /* connSlaveLatency = Latency */
            UINT8 u1Timeout[2];   /* connSupervisionTimeout = Timeout * 10ms */
            UINT8 u1ChM[5];       /* the used channel bitMap (ch0~36) */
            UINT8 Hop:5;          /* hopIncrement value */
            UINT8 SCA:3;          /* the worse case Master's
                                     sleep clock accuracy */
        };
    };
} LL_ADV_PDU_CONNECT_REQ_PL, *PLL_ADV_PDU_CONNECT_REQ_PL;

/*===============================================================*/
/* The Structure of Advertising Channel Packet                   */
/*===============================================================*/
typedef struct LL_ADVERTISING_CH_PKT_S_ {
    UINT8 preamble; /* preamble (byte 0) */
    UINT8 u1AA[4];  /* Access Address (byte 1~5) */
    union {
        LL_ADVERTISING_CH_PDU_HEADER Header; /* PDU Header (byte 6~7) */
        UINT8 u1Header[2];
    };
    union {
        LL_ADV_PDU_ADV_IND_PL AdvInd;
        LL_ADV_PDU_ADV_DIRECT_IND_PL AdvDirInd;
        LL_ADV_PDU_ADV_NONCONN_IND_PL AdvNConnInd;
        LL_ADV_PDU_ADV_DISCOVER_IND_PL AdvDiscInd;
        LL_ADV_PDU_SCAN_REQ_PL ScanReq;
        LL_ADV_PDU_SCAN_RSP_PL ScanRsp;
        LL_ADV_PDU_CONNECT_REQ_PL ConnReq;
    };
} LL_ADVERTISING_CH_PKT_S, *PLL_ADVERTISING_CH_PKT_S;

/*===============================================================*/
/* Data Channel PDU Header Structure (Page 44 of 140) of LL Spec */
/*===============================================================*/
typedef struct LL_DATA_CH_PDU_HEADER_ {
    UINT16 llid:2;   /* bit[1:0], LLID field */
    UINT16 nesn:1;   /* bit[2], Next Expected Sequence Number */
    UINT16 sn:1;     /* bit[3], Sequence Number */
    UINT16 md:1;     /* bit[4], More Data */
    UINT16 rsvd1:3;  /* bit[7:5], reserved */
#ifdef _BT4_2_DATA_EXTENSION_FEATURE_SUPPORT_
    UINT16 length:8; /* bit[15:8] The length of payload */
#else
    UINT16 length:5; /* bit[12:8], the length of payload */
    UINT16 rsvd2:3;  /* bit[15:13], reserved */
#endif
} LL_DATA_CH_PDU_HEADER, *PLL_DATA_CH_PDU_HEADER;

/*================================================================*/
/* LL CONNECTION UPDATE REQ Structure (Page 48 of 140) of LL Spec */
/*================================================================*/
typedef struct LL_CTRDATA_CONN_UPDATE_REQ_PDU_ {
    UINT8 WinSize;        /* transmitWindowSize = WinSize * 1.25ms */
    UINT8 u1WinOffset[2]; /* transmitWindowOffset = WinOffset * 1.25ms */
    UINT8 u1Interval[2];  /* connInterval = Interval * 1.25 ms */
    UINT8 u1Latency[2];   /* connSlaveLatency = Latency */
    UINT8 u1Timeout[2];   /* connSupervisionTimeout = Timeout * 10 ms */
    UINT8 u1Instant[2];   /* connInstant, 1 ~ 32767 */
} LL_CTRDATA_CONN_UPDATE_REQ_PDU, *PLL_CTRDATA_CONN_UPDATE_REQ_PDU;

/*==========================================================*/
/* LL CHANNEL MAP REQ Structure (Page 48 of 140) of LL Spec */
/*==========================================================*/
typedef struct LL_CTRDATA_CH_MAP_REQ_PDU_ {
    UINT8 u1ChM[5];       /* indicating Used and Unused data channels */
    UINT8 u1Instant[2];   /* connInstant, 1 ~ 32767 */
} LL_CTRDATA_CH_MAP_REQ_PDU, *PLL_CTRDATA_CH_MAP_REQ_PDU;

/*==========================================================*/
/* LL TERMINARE IND Structure (Page 49 of 140) of LL Spec   */
/*==========================================================*/
typedef struct LL_CTRDATA_TERMINATE_IND_PDU_ {
    UINT8 ErrCode;      /* inform the remote device why the connection is about
                           to be terminated */
} LL_CTRDATA_TERMINATE_IND_PDU, *PLL_CTRDATA_TERMINATE_IND_PDU;

/*==========================================================*/
/* LL ENC REQ Structure (Page 49 of 140) of LL Spec         */
/*==========================================================*/
typedef struct LL_CTRDATA_ENC_REQ_PDU_ {
    UINT8 u1Rand[8]; /* a random number from the Host and used with EDIV */
    UINT8 u1EDIV[2]; /* the encrypted diversifier */
    UINT8 u1SKDm[8]; /* the master's portion of the session key identifier */
    UINT8 u1IVm[4];  /* the master's portion of the initialization vector */
} LL_CTRDATA_ENC_REQ_PDU, *PLL_CTRDATA_ENC_REQ_PDU;

/*==========================================================*/
/* LL ENC RSP Structure (Page 49 of 140) of LL Spec         */
/*==========================================================*/
typedef struct LL_CTRDATA_ENC_RSP_PDU_ {
    UINT8 u1SKDs[8]; /* the slave's portion of the session key identifier */
    UINT8 u1IVs[4];  /* the slave's portion of the initialization vector */
} LL_CTRDATA_ENC_RSP_PDU, *PLL_CTRDATA_ENC_RSP_PDU;

/*==========================================================*/
/* LL UNKNOWN RSP Structure (Page 50 of 140) of LL Spec     */
/*==========================================================*/
typedef struct LL_CTRDATA_UNKNIWN_RSP_PDU_ {
    UINT8 UnknownType; /* contain the Opcode of the received LL Control PDU */
} LL_CTRDATA_UNKNIWN_RSP_PDU, *PLL_CTRDATA_UNKNIWN_RSP_PDU;

/*==========================================================*/
/* LL FEATURE REQ Structure (Page 50 of 140) of LL Spec     */
/*==========================================================*/
typedef struct LL_CTRDATA_FEATURE_REQ_PDU_ {
    UINT8 u1FeatureSet[8]; /* the set of supported features of the master's LL */
} LL_CTRDATA_FEATURE_REQ_PDU, *PLL_CTRDATA_FEATURE_REQ_PDU;

/*==========================================================*/
/* LL FEATURE RSP Structure (Page 50 of 140) of LL Spec     */
/*==========================================================*/
typedef struct LL_CTRDATA_FEATURE_RSP_PDU_ {
    UINT8 u1FeatureSet[8]; /* the set of supported features of the slave's LL */
} LL_CTRDATA_FEATURE_RSP_PDU, *PLL_CTRDATA_FEATURE_RSP_PDU;

/*==========================================================*/
/* LL VERSION IND Structure (Page 51 of 140) of LL Spec     */
/*==========================================================*/
typedef struct LL_CTRDATA_VERSION_IND_PDU_ {
    UINT8 VersNr;         /* the version of the Bluetooth Controller Spec */
    UINT8 u1CompId[2];    /* the company identifier of the manufacturer of the
                             Bluetooth Controller */
    UINT8 u1SubVersNr[2]; /* a unique value for each implementation or revision
                             of an implementation of the Bluetooth Controller */
} LL_CTRDATA_VERSION_IND_PDU, *PLL_CTRDATA_VERSION_IND_PDU;

/*==========================================================*/
/* LL REJECT IND Structure (Page 51 of 140) of LL Spec      */
/*==========================================================*/
typedef struct LL_CTRDATA_REJECT_IND_PDU_ {
    UINT8 ErrCode;      /* the reason a request was rejected */
} LL_CTRDATA_REJECT_IND_PDU, *PLL_CTRDATA_REJECT_IND_PDU;

typedef struct LL_CTRDATA_REJECT_IND_EXT_PDU_
{
    UINT8 rej_opcode;
    UINT8 err_code;
} LL_CTRDATA_REJECT_IND_EXT_PDU;

typedef struct LL_CTRDATA_LENGTH_REQ_
{
    UINT8 max_rx_octets[2];
    UINT8 max_rx_time[2];
    UINT8 max_tx_octets[2];
    UINT8 max_tx_time[2];
} LL_CTRDATA_LENGTH_REQ;

typedef LL_CTRDATA_LENGTH_REQ LL_CTRDATA_LENGTH_RSP;

typedef struct LL_CTRDATA_CONN_PARAM_REQ_
{
    UINT8 Interval_Min[2];
    UINT8 Interval_Max[2];
    UINT8 Latency[2];
    UINT8 Timeout[2];
    UINT8 PreferredPeriodicity;
    UINT8 ReferenceConnEventCount[2];
    UINT8 Offset0[2];
    UINT8 Offset1[2];
    UINT8 Offset2[2];
    UINT8 Offset3[2];
    UINT8 Offset4[2];
    UINT8 Offset5[2];
} LL_CTRDATA_CONN_PARAM_REQ;

typedef LL_CTRDATA_CONN_PARAM_REQ LL_CTRDATA_CONN_PARAM_RSP;

/*======================================================*/
/* LL Control PDU Structure (Page 46 of 140) of LL Spec */
/*======================================================*/
typedef struct LL_CTRL_PDU_PAYLOAD_ {
    UINT8 OpCode;               /* byte 0, OpCode */
    union {
        UINT8 u1CtrData[22];    /* byte 1 ~ byte 23, Control PDU Data */
        LL_CTRDATA_CONN_UPDATE_REQ_PDU conn_updt_req;
        LL_CTRDATA_CH_MAP_REQ_PDU ch_map_req;
        LL_CTRDATA_TERMINATE_IND_PDU terminate;
        LL_CTRDATA_ENC_REQ_PDU enc_req;
        LL_CTRDATA_ENC_RSP_PDU enc_rsp;
        LL_CTRDATA_UNKNIWN_RSP_PDU unknown_rsp;
        LL_CTRDATA_FEATURE_REQ_PDU feature_req;
        LL_CTRDATA_FEATURE_RSP_PDU feature_rsp;
        LL_CTRDATA_VERSION_IND_PDU version_ind;
        LL_CTRDATA_REJECT_IND_PDU reject_ind;
        LL_CTRDATA_REJECT_IND_EXT_PDU reject_ind_ext;
        LL_CTRDATA_LENGTH_REQ len_req;
        LL_CTRDATA_LENGTH_RSP len_rsp;
        LL_CTRDATA_CONN_PARAM_REQ conn_param_req;
        LL_CTRDATA_CONN_PARAM_RSP conn_param_rsp;
    };
    void *pNext;                /* point to the next LL Control PDU */
} LL_CTRL_PDU_PAYLOAD, *PLL_CTRL_PDU_PAYLOAD;

/*===============================================================*/
/* The Data Structure of HW Advertising Channel Rx Packet        */
/*===============================================================*/
typedef struct LE_HW_ADVERTISING_CH_RX_PKT_S_ {
    union {
        LL_ADVERTISING_CH_PDU_HEADER Header; /* PDU Header (byte 0~1) */
        UINT8 u1Header[2];
        UINT16 u2Header;
    };
    union {                                  /* PDU Payload (byte 2~N) */
        LL_ADV_PDU_ADV_IND_PL AdvInd;
        LL_ADV_PDU_ADV_DIRECT_IND_PL AdvDirInd;
        LL_ADV_PDU_ADV_NONCONN_IND_PL AdvNConnInd;
        LL_ADV_PDU_ADV_DISCOVER_IND_PL AdvDiscInd;
        LL_ADV_PDU_SCAN_REQ_PL ScanReq;
        LL_ADV_PDU_SCAN_RSP_PL ScanRsp;
        LL_ADV_PDU_CONNECT_REQ_PL ConnReq;
        UINT8 u1RemoteAddr[6];
    };
} _PACKED_ LE_HW_ADVERTISING_CH_RX_PKT_S, *PLE_HW_ADVERTISING_CH_RX_PKT_S;

/*===============================================================*/
/* The Data Structure of HW Data Channel Rx Packet               */
/*===============================================================*/
typedef struct LE_HW_DATA_CH_RX_PKT_S_ {
    union {
        LL_DATA_CH_PDU_HEADER Header; /* PDU Header (byte 0~1) */
        UINT8 u1Header[2];
        UINT16 u2Header;
    };
    union {                           /* PDU Payload (byte 2~N) */
        LL_CTRL_PDU_PAYLOAD ctrl_pdu; /* LL Control PDU */
        UINT8 data_pdu[31];           /* LL Data PDU */
        UINT8 buf[31];
    };
} _PACKED_ LE_HW_DATA_CH_RX_PKT_S, *PLE_HW_DATA_CH_RX_PKT_S;

/*==========================================================*/
/* SW Structure for LL Connection Update Block              */
/*==========================================================*/
typedef struct LL_CONN_UPDT_BLK_ {
    UINT16 ce_interval_min; /* connection event interval min*/
    UINT16 ce_interval_max; /* connection event interval max */
    UINT16 ce_interval;     /* my connection event interval */
    UINT16 slave_latency;   /* the slave latency */
    UINT16 supervision_to;  /* supervision timeout */
    UINT16 ce_length_min;   /* minimum connection event length */
    UINT16 ce_length_max;   /* maximum connection event length */
    UINT16 ce_length;       /* connection event length */
    UINT16 instant;         /* connection update instant */
    UINT16 tx_win_offset;   /* transmit window offset (unit:1.25ms) */
    UINT8  tx_win_size;     /* transmit window size (unit:1.25ms) */
#if (defined(_DAPE_GET_SLOT_FOR_LE_UPDT) || defined(_DAPE_KEEP_SLOT_OFST_SAME_WHEN_CONN_UPDT))
    UINT8  tx_win_size_ofst;     /* transmit window size (unit:1.25ms) */
#endif
} LL_CONN_UPDT_BLK, *PLL_CONN_UPDT_BLK;
/*==========================================================*/
/* SW Structure for LL Connection Parameter Request Block   */
/*==========================================================*/
typedef struct LL_CONN_PARAM_REQ_BLK_ {
    UINT16 Interval_Min; /* connection event interval min (1.25ms)*/
    UINT16 Interval_Max; /* connection event interval max (1.25ms)*/
    UINT16 Latency;      /* the slave latency */
    UINT16 Timeout;      /* supervision timeout (10ms)*/
    UINT16 PreferredPeriodicity;   /* the value the ConnInterval is preferred to be
                                      a multiple of(1.25ms).
                                      0:not valid.*/
    UINT16 RefConnEventCount;   /* the value of the connEventCounter relative to which
                                   all the valid Offset0 to Offset5 fields have been
                                   calculated. */
    UINT16 Offset0;  /* The possible values of the position of the anchor point(1.25ms). 0xFFFF: not valid.*/
    UINT16 Offset1;  /* The possible values of the position of the anchor point(1.25ms). 0xFFFF: not valid.*/
    UINT16 Offset2;  /* The possible values of the position of the anchor point(1.25ms). 0xFFFF: not valid.*/
    UINT16 Offset3;  /* The possible values of the position of the anchor point(1.25ms). 0xFFFF: not valid.*/
    UINT16 Offset4;  /* The possible values of the position of the anchor point(1.25ms). 0xFFFF: not valid.*/
    UINT16 Offset5;  /* The possible values of the position of the anchor point(1.25ms). 0xFFFF: not valid.*/
} LL_CONN_PARAM_REQ_BLK, *PLL_CONN_PARAM_REQ_BLK;

/*==========================================================*/
/* SW Structure for LL Channel Map Block                    */
/*==========================================================*/
typedef struct LL_CH_MAP_BLK_ {
    UINT16 instant;         /* channel map update instant */
} LL_CH_MAP_BLK, *PLL_CH_MAP_BLK;

/*==========================================================*/
/* SW Structure for LL Encryption Block                     */
/*==========================================================*/
typedef struct LL_ENCRYPT_BLK_ {
    UINT8 rand[8];           /* 64 bit random number from host */
    UINT8 ediv[2];           /* 16 bit encrypted diversifier */
    UINT8 ltk[16];           /* 128 bit long term key */
    UINT8 skd_m[8];          /* master portion of the session key identifier */
    UINT8 skd_s[8];          /* slave portion of the session key identifier */
    UINT8 iv_m[4];           /* master portion of the initialization vector */
    UINT8 iv_s[4];           /* salve portion of the initialization vector */
    UINT8 sesskey[16];       /* 128 bit session key */
    UINT8 start_enc_state;   /* three way handshake state for start procedure */
    UINT8 pause_enc_state;   /* three way handshake state for pause procedure */
} _PACKED_ LL_ENCRYPT_BLK, *PLL_ENCRYPT_BLK;

/*==========================================================*/
/* SW Structure for LL Test Block                           */
/*==========================================================*/
typedef struct LL_TEST_BLK_ {
    UINT8 rf_channel;       /* LL RF channel (0 ~ 39) */
    UINT8 payload_len;      /* length of tx test data (bytes) */
    UINT8 payload_type;     /* the payload type */
    UINT8 rsvd_u8;          /* reserved for padding */
    UINT16 num_of_rx_pkts;  /* number of packets received */
    UINT16 num_of_rx_pkts_crc_err; /* number of crc error packets received */
    UINT16 tx_pkt_cnt:8;      /* the tx number of this time in LE TX Test mode.*/
    UINT16 tx_cnt_mode_en:1;  /* 1: enable tx counter mode. If 1, tx_pkt_cnt is used.*/
    UINT16 pkt_cnt_tx_busy:1; /* 1: still tx ing. */
    UINT16 chg_aa:1;          /* 1:will set access address  base on access_addr_l & access_addr_h.*/
    UINT16 prbs_fix:1;        /* prbs fixed or not in Tx test mode. */
    UINT16 rsvd:4;            /* rsvd */
#ifdef _DAPE_SUPPORT_CHG_AA_IN_LE_TEST_MODE
    UINT16 access_addr_l;          /* access address in test mode. */
    UINT16 access_addr_h;          /* access address in test mode. */
#endif
} LL_TEST_BLK, *PLL_TEST_BLK;

/*==========================================================*/
/* SW Structure for Link Layer Control Procedure Unit       */
/*==========================================================*/
typedef struct LL_LLC_MANAGER_ {
    UINT8 type:5;                  /* the type of LLC in procedure */
    UINT8 pend_conn_updt:1;        /* pend a connection request update ? */
    UINT8 pend_ch_map_updt:1;      /* pend a channel map update ? */
    UINT8 pend_start_encrypt:1;    /* pend a start encryption ? */

    UINT8 pend_feature_ex:1;       /* pend a feature exchange procedure ? */
    UINT8 pend_version_ex:1;       /* pend a version exchange procedure ? */
    UINT8 pend_terminate_ind:1;    /* pend a terminate ind ? */
    UINT8 sent_version_ind:1;      /* the LLC_VERSION_IND is sent ? */
    UINT8 get_version_ind:1;       /* get LLC_VERSION_IND ? */
    UINT8 pend_data_len_updt:1;    /* pend a data length update ? */
    UINT8 delay_data_len_updt:1;   /* delay a data length update until receiving a data PDU ? */
    UINT8 pend_conn_param_req:1;  /* pend a connection parameter request?  */

    UINT8 pend_ping_req:1;         /* pend a LE ping request ? */
    UINT8 wait_ack:1;              /* wait ack to finish llc procedure ? */
    UINT8 wait_sent_pdu_type:5;    /* wait which sent pdu for ack ? */
    UINT8 loc_term_from_host:1;    /* terminate from host or fw */

    UINT8 loc_terminate_reason;    /* current terminate reason */
} LL_LLC_MANAGER, *PLL_LLC_MANAGER;

/*==========================================================*/
/* HW/FW Structure of LL HCI ACL Data                       */
/*==========================================================*/
/* host to controller */
#ifdef _SUPPORT_LE_HCI_TX_PKT_FRAGMENT_REASSEMBLE_FUNC_
#define LL_POLL_HCI_MAX_TX_ACL_PKT_PAYLOAD_SIZE  (LL_POLL_HCI_MAX_TX_ACL_PKT_SIZE - 12)
#else
#define LL_POLL_HCI_MAX_TX_ACL_PKT_PAYLOAD_SIZE  (LL_POLL_HCI_MAX_TX_ACL_PKT_SIZE - 8)
#endif

typedef struct LL_HCI_ACL_DATA_PKT_ {
    union {
        struct {
            UINT32 connection_handle:12;     /* bit[11:0], connection handle */
            UINT32 packet_boundary_flag:2;   /* bit[13:12], PB flag */
            UINT32 broadcast_flag:2;         /* bit[15:14], BC flag */
            UINT32 acl_data_total_length:16; /* bit[31:16], data total length */
        };
        UINT32 header;                       /* The header of HCL ACL data pkt*/
    };
    UINT8  hci_acl_data_pkt[LL_POLL_HCI_MAX_TX_ACL_PKT_PAYLOAD_SIZE];
    void *next;
#ifdef _SUPPORT_LE_HCI_TX_PKT_FRAGMENT_REASSEMBLE_FUNC_
    UINT16 bytes_acked; /**< Number of bytes acknowledged. */
    UINT16 frag_offs; /**< Starting offset to fragment. */
#endif
} _PACKED_ LL_HCI_ACL_DATA_PKT, *PLL_HCI_ACL_DATA_PKT;

/* controller to host */

#define LL_POLL_HCI_MAX_RX_ACL_PAYLOAD_SIZE   (LL_POLL_HCI_MAX_RX_ACL_PKT_SIZE - 8)

typedef struct LL_HCI_ACL_RX_DATA_PKT_ {
    union {
        struct {
            UINT32 connection_handle:12;     /* bit[11:0], connection handle */
            UINT32 packet_boundary_flag:2;   /* bit[13:12], PB flag */
            UINT32 broadcast_flag:2;         /* bit[15:14], BC flag */
            UINT32 acl_data_total_length:16; /* bit[31:16], data total length */
        };
        UINT32 header;                       /* The header of HCL ACL data pkt*/
    };
    UINT8  hci_acl_data_pkt[LL_POLL_HCI_MAX_RX_ACL_PAYLOAD_SIZE];
    void *next;
} _PACKED_ LL_HCI_ACL_RX_DATA_PKT, *PLL_HCI_ACL_RX_DATA_PKT;

typedef struct LL_HCI_ACL_RX_DATA_PKT_TAIL_ {
    void *next;
} LL_HCI_ACL_RX_DATA_PKT_TAIL;

#ifdef _SCHEDULE_BLE_MISC_PACKET_
#define LL_MISC_ACL_DATA_PKT_MAX_FRAGS          2
#define LL_MISC_ACL_DATA_PKT_MAX_NODES          10
#define LL_MISC_ACL_PKT_LIST_RESET_VALUE        (LL_MISC_ACL_DATA_PKT_MAX_NODES | (LL_MISC_ACL_DATA_PKT_MAX_NODES << 8))

#define LL_BUFF_RAM_BASE                        (KSEG1 | 0x100000)

#define LL_AUTHENTICATED_PAYLOAD_TIMEOUT        0x0BB8 /* this unit is 10ms. and default = 30sec.*/

enum LL_MISC_ACL_DATA_PKT_NODE_TYPES {
    LL_MISC_ACL_DATA_PKT_NODE_TYPE_HCI  = 0,
    LL_MISC_ACL_DATA_PKT_NODE_TYPE_LLC  = 1,
    LL_MISC_ACL_DATA_PKT_NODE_TYPE_MISC = 2,
} ;

#ifdef _SUPPORT_LE_HCI_TX_PKT_FRAGMENT_REASSEMBLE_FUNC_
/**
 * @brief Fragment information of LL_HCI_ACL_DATA_PKT.
 */
typedef struct LL_HCI_ACL_DATA_FRAGMENT_INFO_
{
    /** Instance of LL_HCI_ACL_DATA_PKT. */
    LL_HCI_ACL_DATA_PKT *pkt;
    /** Starting offset of LL_HCI_ACL_DATA_PKT payload fragment. */
    UINT8 offs;
    /** Length of LL_HCI_ACL_DATA_PKT payload fragment. */
    UINT8 len;
} LL_HCI_ACL_DATA_FRAGMENT_INFO;

void ll_hci_acl_data_fragment_info_init(LL_HCI_ACL_DATA_FRAGMENT_INFO *frag_info, LL_HCI_ACL_DATA_PKT *pkt);
BOOLEAN ll_hci_acl_data_fragment_info_is_end(LL_HCI_ACL_DATA_FRAGMENT_INFO *frag_info);
UINT16 ll_hci_acl_data_fragment_info_next(LL_HCI_ACL_DATA_FRAGMENT_INFO *frag_info, UINT16 bytes);
#endif

/*==========================================================*/
/* FW Structure for Miscellaneous Packet Source             */
/*==========================================================*/
typedef struct LL_MISC_ACL_DATA_PKT_NODE_ {
#ifdef _NEW_BZDMA_FROM_V8_PROGRAMMABLE_CONFIG_SETTING_
    BZDMA_BLEONLY_TX_DESC_FRAGMENT *pFrags;
#ifdef _SUPPORT_LE_HCI_TX_PKT_FRAGMENT_REASSEMBLE_FUNC_
    LL_HCI_ACL_DATA_FRAGMENT_INFO *frag_info;
#endif
#else
    BZDMA_BLEONLY_TX_DESC_FRAGMENT pFrags[LL_MISC_ACL_DATA_PKT_MAX_FRAGS];
#ifdef _SUPPORT_LE_HCI_TX_PKT_FRAGMENT_REASSEMBLE_FUNC_
    LL_HCI_ACL_DATA_FRAGMENT_INFO frag_info[LL_MISC_ACL_DATA_PKT_MAX_FRAGS];
#endif
#endif
    UINT8 Src;
    UINT8 FragCount:7;
    UINT8 is1stFrag:1;
    UINT8 MyNodeId;
    UINT8 NextNodeId;
} LL_MISC_ACL_DATA_PKT_NODE, *PLL_MISC_ACL_DATA_PKT_NODE;

typedef union LE_ACL_MISC_PKT_LIST_MANAGE_ {
    struct {
        UINT8 MiscHeadId;
        UINT8 MiscTailId;
        UINT8 pktcnt;
        UINT8 in_procedure;
    };
    UINT32 DWord;
} _PACKED_ LE_ACL_MISC_PKT_LIST_MANAGE, *PLE_ACL_MISC_PKT_LIST_MANAGE;
#endif


/*==========================================================*/
/* FW Structure for Manage LE HCI ACL Packet List           */
/*==========================================================*/
typedef struct LE_ACL_PKT_LIST_MANAGE_ {
    union {
        struct {
            LL_HCI_ACL_DATA_PKT *pDataHead;
            LL_HCI_ACL_DATA_PKT *pDataTail;
        };
        struct {
            LL_HCI_ACL_RX_DATA_PKT *pRxDataHead;
            LL_HCI_ACL_RX_DATA_PKT *pRxDataTail;
        };
        struct {
            LL_CTRL_PDU_PAYLOAD *pCtrlHead;
            LL_CTRL_PDU_PAYLOAD *pCtrlTail;
        };
    };
    UINT8 pktcnt;
    UINT8 in_procedure;
} LE_ACL_PKT_LIST_MANAGE, *PLE_ACL_PKT_LIST_MANAGE;

#ifdef _SUPPORT_LE_HCI_TX_PKT_FRAGMENT_REASSEMBLE_FUNC_
void ll_hci_acl_data_pkt_list_reset(LE_ACL_PKT_LIST_MANAGE *list);
void ll_hci_acl_data_pkt_list_move(LE_ACL_PKT_LIST_MANAGE *dest, LE_ACL_PKT_LIST_MANAGE *src);
void ll_hci_acl_data_pkt_list_add_tail(LE_ACL_PKT_LIST_MANAGE *list, LL_HCI_ACL_DATA_PKT *pkt);
LL_HCI_ACL_DATA_PKT *ll_hci_acl_data_pkt_list_pop(LE_ACL_PKT_LIST_MANAGE *list);
#endif

/*==========================================================*/
/* FW Structure for LE Rx Task Parameter                    */
/*==========================================================*/
typedef struct LL_RX_TASK_PARA_S_ {
    UINT32 sub_type:5;          /* subtype */
    UINT32 conn_entry:3;        /* connection entry */
    UINT32 int_src:2;           /* the interrupt source */
    UINT32 fifo_word_cnt:17;    /* the rx fifo word count */
    UINT32 rsvd:5;              /* reserved */
} LL_RX_TASK_PARA_S, *PLL_RX_TASK_PARA_S;

/*==========================================================*/
/* FW Structure for LE LMP(LL) Task Parameter               */
/*==========================================================*/
typedef struct LL_LMP_TASK_PARA_S_ {
    UINT32 sub_type:5;          /* subtype */
    UINT32 conn_entry:3;        /* connection entry */
    UINT32 int_src:2;           /* the interrupt source */
    UINT32 status:8;            /* the rx fifo word count */
    UINT32 reason:8;            /* reserved */
    UINT32 rsvd:6;
} LL_LMP_TASK_PARA_S, *PLL_LMP_TASK_PARA_S;

typedef union LL_TASK_PARA_U_ {
    LL_RX_TASK_PARA_S rx_s;
    LL_LMP_TASK_PARA_S lmp_s;
    UINT32 Dword;
} LL_TASK_PARA_U, *PLL_TASK_PARA_U;

/*==========================================================*/
/* FW Structure for LL Event Manager                      */
/*==========================================================*/
typedef struct LL_EVENT_MANAGE_ {
    UINT8 event_type : 1;    /* advertising or connection event */
    UINT8 event_start : 1;   /* AE or CE start (only valid for CE) */
    UINT8 event_end : 1;     /* AE or CE End */
    UINT8 reserved : 5;
    UINT8 entry;             /* connection entry */

#ifdef _NEW_BLE_HW_SPEC_FROM_150320_
    UINT16 cur_rx_packet_cnt;   /* current received packet count */
#endif
    /* for Rx */
    UINT32 cur_rx_word_cnt;     /* current received word count */
#ifndef USE_NEW_LE_SCHEDULER
    UINT32 bm_conn_event_window; /* bit[N] = 0, mean handle M is out of ce now
                                    bit[N] = 1, mean handle N is in ce now */
#endif

#ifndef _NEW_BZDMA_FROM_V8_
    UINT8 cur_txcmd_id;      /* current tx command id */
    UINT8 used_txcmd_cnt;    /* current used tx command count */
    UINT8 pre_tx_entry;      /* previous conn entry to use tx command */
    UINT8 cur_tx_entry;      /* current conn entry to use tx command */
#endif
} LL_EVENT_MANAGE, *PLL_EVENT_MANAGE;

/*=============================================================*/
/* SW Structure of Remote Device Information in Connected Sate */
/*=============================================================*/
typedef struct LL_CONN_REMOTE_INFO_ {
    union {
        UINT8 addr[6];           /* remote device address */
        UINT16 u2addr[3];
    };
    union {
        UINT8 le_features[8];    /* remote le features */
        UINT16 u2le_features[4];
    };
    UINT8 addr_type;             /* remote address type (0:public 1:random) */
    UINT8 vers_nr;               /* the version of bluetooth controller spec */
    UINT16 comp_id;              /* the company identifier */
    UINT16 subvers_nr;           /* a unique value for each implementation or
                                    revision of an implementation of the BT
                                    Controller */
} LL_CONN_REMOTE_INFO, *PLL_CONN_REMOTE_INFO;

#ifdef _BT4_2_DATA_EXTENSION_FEATURE_SUPPORT_
typedef enum LLC_DATA_LEN_UPDATE_STATE_
{
    /**
     * @brief The state that data length update procedure is successfully
     * completed.
     */
    LLC_DATA_LEN_UPDATE_STATE_END,
    /**
     * @brief The state that local device initiates new data length update
     * procedure.
     */
    LLC_DATA_LEN_UPDATE_STATE_START_REQ,
    /**
     * @brief The state that remote device initiates new data length update
     * procedure.
     */
    LLC_DATA_LEN_UPDATE_STATE_RECV_REQ,
    /**
     * @brief The state that both sides initiate new data length update
     * procedure.
     */
    LLC_DATA_LEN_UPDATE_STATE_RECV_REQ_WITH_CONFLICT,
} LLC_DATA_LEN_UPDATE_STATE;

/**
 * @brief Data length update information.
 */
typedef struct LL_DATA_LEN_UPDATE_BLK_
{
    UINT16 local_max_tx_size;
    UINT16 remote_max_rx_size;
    UINT16 local_max_rx_size;
    UINT16 remote_max_tx_size;
    UINT16 local_max_tx_time;
    UINT16 remote_max_rx_time;
    UINT16 local_max_rx_time;
    UINT16 remote_max_tx_time;

    UINT16 max_tx_size;
    UINT16 max_rx_size;
    UINT16 max_tx_time;
    UINT16 max_rx_time;

    UINT8 state;
    UINT8 tx_changed;
} LL_DATA_LEN_UPDATE_BLK;
#endif

typedef struct HCI_CMD_BITMAP_
{
    UINT16 conn_updt : 1;
    UINT16 remote_feature : 1;
    UINT16 remote_version : 1;
    UINT16 reserved : 13;
} HCI_CMD_BITMAP;

/*==========================================================*/
/* SW Structure for LL Connection Unit                      */
/*==========================================================*/
typedef struct LL_CONN_HANDLE_UNIT_ {
    /* don't move first 4 byte - austin */
    UINT32 conn_handle:16;          /* connection handle */
    UINT32 unit_id:8;               /* my unit id */
    UINT32 connected:1;             /* this handle is connected ? */
    UINT32 rsvd1:7;                 /* reserved */

    UINT8 support_enc:1;           /* support encryption for this link */
    UINT8 encrypted:1;             /* the connection is encrypted ? */
    UINT8 install_key:1;           /* install session key ? */
    UINT8 recv_conn_req:1;         /* receive connection request (slave) */
    UINT8 sup_timeout_restart:1;   /* restart suppervision timer ? */
    UINT8 terminate_timer_run:1;   /* the terminate timer is run ? */
    UINT8 long_term_key_got:1;     /* got longterm key ? */
    UINT8 pause_acl_tx:1;          /* pause acl tx path after start or restart
                                       encryption ? */

    UINT8 discard_acl_rx:1;        /* discard acl rx path after start
                                       or restart encryption ? */
    UINT8 all_rx_pkt_no_crc_ok:1;  /* all received packets are no CRC OK ? */
    UINT8 rx_pkt_mic_err:1;        /* received packet is MIC error ? */
    UINT8 rx_any_pkt_in_ce:1;      /* receive any packets during CE */
    UINT8 conn_created:1;          /* connection created while receiving
                                       hw conn_interrupt */
    UINT8 pause_sco:1;             /* connection created while receiving hw
                                       conn_interrupt */
    UINT8 le_slot_updated:1;       /* update le slot to global slot offset
                                       already? */
    UINT8 conn_update_le_slot_updated:1; /* reserved */

    UINT8 rx_any_in_conn_state:1;  /* receive any packets (include CRC error)
                                       in connected state ? */
    UINT8 support_dle:1;           /* support LE data length extension */
    UINT8 is_checking_llc_procedure:1; /* is checking llc procedure */
    UINT8 early_killed:1;          /* If early killed because of a new connection created with existed BD_ADDR */
    UINT8 self_updt:1;             /* has this ce doing self-update yet? */
    UINT8 rsvd2:3;                /* reserved */

    UINT8 kill_link_reason;         /* the reason this link will be killed */

    UINT16 ce_interval;             /* connection event interval(unit:1.25ms) */
#ifdef _NEW_BLE_HW_SPEC_FROM_150320_
    UINT16 ce_rx_packet_cnt;        /* received packet count in the FIFO */
#endif
    UINT32 ce_rx_word_cnt;          /* receive rx word count in the FIFO */

    UINT32 anchor_point_by_intr;    /* anchor point get by ce_early_interrupt */
    UINT32 anchor_point_slot;       /* absolute value of anchor point, update at misr15 */
    UINT32 next_anchor_point_slot;  /* anchor_point_slot + ce_interval*/

    UINT16 ce_length;               /* connection event length */
    UINT16 slave_latency;           /* slave latency */
    UINT16 conn_counter;            /* connection counter (add one every ce) */
    UINT16 supervision_to;          /* supervision timeout value(unit:10ms) */

    UINT16 tx_sched_pkts_one_ce;    /* the tx scheduled packets in one CE */
    UINT16 ce_count_rx_any_pkt;     /* the ce counter when receive any pkt */
    UINT16 ce_count_no_crc_ok;      /* the ce counter when no CRC ok */
    UINT16 ce_count_restart_sup;    /* the ce counter when restart suppervision
                                       timer */

    UINT16 ce_count_no_any_pkt_received;     /* the ce counter when receive any pkt */
    UINT16 ce_count_crc_ok;      /* the ce counter when no CRC ok */
    UINT16 slot_offset;                      /* slot offset of this connection entry */
#ifdef _DAPE_AUTO_CONN_UPDT_WHEN_SLOT_CONFLICT_AFTER_CONN_UPDT
    UINT16 updated_conn_counter;            /* connection counter (add one every ce) */
#endif
    UINT8 bmChMap[5];               /* channel bitmap (channel 0 ~ 39) */
    UINT8 hc_complete_pkts;         /* the number of completed HCI acl pkt */
    UINT8 cur_tx_power_index;       /* transmit power index (0~7) */
    INT8 tx_power_level;            /* transmit power level */

    UINT8 ce_acked_tx_pkt_cnt;      /* acked tx packet count */
    UINT8 last_rssi;                /* the last rssi value */
    HCI_CMD_BITMAP hci_cmd_bm;      /* Bitmap of HCI commands initiated by host */

    TimerHandle_t sup_timer;             /* supervision timer */
    TimerHandle_t llc_timer;             /* llc timer */
#ifdef _BT4_1_LE_PING_FEATURE_SUPPORT_
    TIMER_ID ping_req_timer;        /* le ping timer */
#endif

    LL_CONN_REMOTE_INFO remote_info;/* the information of remote device */
    LL_LLC_MANAGER llc;             /* llc manager */
    LL_CONN_UPDT_BLK conn_updt_blk; /* connection update parameter block */
    LL_CH_MAP_BLK ch_map_blk;       /* channel map update block */
    LL_ENCRYPT_BLK encrypt_blk;     /* encryption parameter block */
#ifdef _BT4_2_DATA_EXTENSION_FEATURE_SUPPORT_
    LL_DATA_LEN_UPDATE_BLK data_len_updt; /* data length update block */
#endif
#ifdef _BT4_1_SUPPORT_LL_CONN_PARAM_REQ
    LL_CONN_PARAM_REQ_BLK conn_param_req_blk; /* connection parameter request block */
#endif
#ifdef _BT4_1_LE_PING_FEATURE_SUPPORT_
    UINT16 auth_payload_timeout;    /* Authenticated Payload Timeout for LE ping */
#endif
    LE_ACL_PKT_LIST_MANAGE tx_pend_data_pkt_list;  /* pending data pkt list */
    LE_ACL_PKT_LIST_MANAGE tx_pend_ctrl_pkt_list;  /* pending ctrl pkt list */
#ifndef USE_NEW_LE_SCHEDULER
#ifndef _SCHEDULE_BLE_MISC_PACKET_
    LE_ACL_PKT_LIST_MANAGE tx_sched_data_pkt_list; /* scheduled data pkt list */
    LL_HCI_ACL_DATA_PKT    *tx_resent_data_pkt;    /* resent data packet */
#endif
    LE_ACL_PKT_LIST_MANAGE tx_sched_ctrl_pkt_list; /* scheduled ctrl pkt list */
#endif
#if defined(_SCHEDULE_BLE_MISC_PACKET_) && defined(_NEW_BZDMA_FROM_V8_)
    LE_ACL_MISC_PKT_LIST_MANAGE tx_pend_misc_pkt_list;  /* pending misc pkt list */
#ifndef USE_NEW_LE_SCHEDULER
    LE_ACL_MISC_PKT_LIST_MANAGE tx_sched_misc_pkt_list; /* scheduled misc pkt list */
#endif
    LE_ACL_MISC_PKT_LIST_MANAGE h2l_free_misc_pkt_list; /* HCI to LL acl misc pkt free list */
#ifndef USE_NEW_LE_SCHEDULER
    LL_MISC_ACL_DATA_PKT_NODE *tx_resent_misc_pkt; /* resent misc packet */
#endif
#endif

    LE_ACL_PKT_LIST_MANAGE h2l_free_pkt_list;      /* HCI to LL acl pkt free list */
} LL_CONN_HANDLE_UNIT, *PLL_CONN_HANDLE_UNIT;

/*==========================================================*/
/* SW Structure for LL Connection Handle Unit               */
/*==========================================================*/
typedef struct LL_CONNECTION_UNIT_ {
    UINT16 enable:1;           /* enable the connection set ? */
    UINT16 master:1;           /* master or slave role ? */
    UINT16 masterSCA:3;        /* Master Clock Accuracy */
    UINT16 support_enc:1;      /* support LE encryption by myself */
    UINT16 support_dle:1;      /* support LE data length extension */
    UINT16 support_conn_para_req:1; /* support Connection Parameters Request Procedure by myself */
    UINT16 support_ext_reject_ind:1; /* support Extended Reject Indication by myself */
    UINT16 support_slave_init_feat_ext:1; /* support Slave-initiated Features Exchange by myself */
    UINT16 support_le_ping:1;  /* support LE Ping by myself */
    UINT16 en_wake_from_slave_latency:1;  /* support wakeup from slave latency policy */
    UINT16 rsvd:4;             /* reserved */
    UINT16 bmActiveHandle;     /* the bitmap of active handle(s) */
    UINT8  connection_cnts;    /* the number of connections */
    UINT8  updt_ch_map[5];     /* updated channel map */
    UINT8  conn_updt_entry;    /* connection updating connection entry */
    UINT8  chm_updt_entry;     /* channel updating connection entry */
    UINT8  ce_interval_same;           /* for master, to indicate if all ce_intervals are the same.*/
    UINT8  first_connection_entry;     /* the first connection entry */
    UINT16 first_slot_offset_for_le;   /* first slot offset for dual mode le. */
    UINT16 current_slot_offset_for_le; /* current slot offset for dual mode le. */
    UINT16 tx_win_offset;              /* the tx_win_offset when conn_req is send */
    UINT16 anchor_distance_min;        /* (625us) minimum distance of 2 anchor points*/
    UINT16 ce_interval_min;           /* (1.25ms) the minimum ce_interval of all connections */
#ifdef _BT4_2_DATA_EXTENSION_FEATURE_SUPPORT_
    UINT16 init_max_tx_size; /* Initial maximum transmitted packet size */
    UINT16 init_max_tx_time; /* Initial maximum packet transmission time */
#endif

    LL_CONN_HANDLE_UNIT handle[LL_MAX_CONNECTION_UNITS];

    LE_ACL_PKT_LIST_MANAGE l2h_acl_pkt_list;  /* LL to HCI acl pkt list */
    LE_ACL_PKT_LIST_MANAGE llc_free_pkt_list; /* LLC pkt free list */
} LL_CONNECTION_UNIT, *PLL_CONNECTION_UNIT;

/*==========================================================*/
/* SW Structure for LL Advertiser                           */
/*==========================================================*/
typedef struct LL_ADVERTISER_UNIT_ {
    UINT32 enable:1;           /* enable the advertising ? */
    UINT32 local_addr_type:2;  /* local address type (0/1/2/3:public/random/RPA public/RPA random) */
    UINT32 direct_addr_type:2; /* remote address type (0/1/2/3:public/random/RPA public/RPA random) */
    UINT32 filter_scan_req:1;  /* advertising filter policy for scan req */
    UINT32 filter_conn_req:1;  /* advertising filter policy for connection req*/
    UINT32 pdu_type:3;         /* advertising type */
    UINT32 hci_pdu_type:3;     /* advertising type from hci command */
    UINT32 ch_map:3;           /* advertising channel map */
#ifdef _NEW_BLE_HW_SPEC_FROM_150320_
    UINT32 h2h_period_en:1;    /* enable h2h period setting ? */
    UINT32 rsvd:15;            /* reserved */
#else
    UINT32 rsvd:16;            /* reserved */
#endif
    UINT16 interval_min;       /* min adv interval for non-directed adv
                                 (unit:625us)*/
    UINT16 interval_max;       /* max adv interval for non-directed adv
                                 (unit:625us)*/
    union {
        UINT8 direct_addr[6];  /* directed device address */
        UINT16 u2dir_addr[3];
    };
    UINT8 adv_len;             /* advertising data length */
    UINT8 scan_rsp_len;        /* scan response data length */
    UINT8 *adv_buf;            /* the pointer of advertising data buffer */
    UINT8 *scan_rsp_buf;       /* the pointer of scan response data buffer */
    UINT16 interval;           /* advertising interval for non-directed adv
                                  (unit:625us) */
    UINT16 instant;            /* the instant of advertising anchor point
                                  (unit:625us) */
} LL_ADVERTISER_UNIT, *PLL_ADVERTISER_UNIT;

/*==========================================================*/
/* SW Structure for Advertising Report Unit                 */
/*==========================================================*/
typedef struct LL_ADV_REPORT_UNIT_ {
    UINT8 num_of_reports;
    UINT8 event_type[LE_NUM_OF_ADVERTISING_REPORT_MAX];
    UINT8 addr_type[LE_NUM_OF_ADVERTISING_REPORT_MAX];
    UINT8 addr[LE_NUM_OF_ADVERTISING_REPORT_MAX][6];
    union {
        UINT8 data_len[LE_NUM_OF_ADVERTISING_REPORT_MAX];
        UINT8 direct_addr_type[LE_NUM_OF_ADVERTISING_REPORT_MAX];
    };
    union {
        UINT8 data_buf[LE_NUM_OF_ADVERTISING_REPORT_MAX][32];
        UINT8 direct_addr[LE_NUM_OF_ADVERTISING_REPORT_MAX][6];
    };
    UINT8 is_direct_adv_report;
    UINT8 rssi[LE_NUM_OF_ADVERTISING_REPORT_MAX];
} LL_ADV_REPORT_UNIT, *PLL_ADV_REPORT_UNIT;

/*==========================================================*/
/* SW Structure for LL Scanner                              */
/*==========================================================*/
typedef struct LL_SCANNER_UNIT_ {
    UINT16 enable:1;           /* enable the scanner ? */
    UINT16 local_addr_type:2;  /* local address type (0/1/2/3:public/random/RPA public/RPA random) */
    UINT16 filter_policy:2;    /* enable scanning filter policy for adv pdu */
    UINT16 filter_duplicate:1; /* enable duplicates filter (need generate
                                  LE advertising reports if enable) */
    UINT16 active_scan:1;      /* 1: active scan 0: passive scan */
    UINT16 ch_map:3;           /* scanning channel map */
#ifdef _CCH_RETENTION_FLOW_FOR_DLPS_LE_
    UINT16 ch_bit_in_use:2;    /* scanning channel map bit use when enter lps mode */
    UINT16 rsvd:3;             /* reserved */
#else
    UINT16 rsvd:5;             /* reserved */
#endif
    UINT16 sw_filter_policy:1; /* enable sw scanning filter policy for adv pdu */
    UINT16 interval;           /* scan interval */
    UINT16 window;             /* scan window */
    UINT16 instant;            /* the instant of scan procedure */

    LL_ADV_REPORT_UNIT report; /* the buffer of advertising report */
} LL_SCANNER_UNIT, *PLL_SCANNER_UNIT;


/*==========================================================*/
/* SW Structure for LL Initiator                            */
/*==========================================================*/
typedef struct LL_INITIATOR_UNIT_ {
    UINT16 enable:1;           /* enable the initiator ? */
    UINT16 local_addr_type:2;  /* local address type (0/1/2/3:public/random/RPA public/RPA random) */
    UINT16 remote_addr_type:2; /* remote address type (0/1/2/3:public/random/RPA public/RPA random) */
    UINT16 filter_policy:1;    /* initiator filter policy */
    UINT16 ch_map:3;           /* initiator scan channel map */
    UINT16 masterSCA:3;        /* master's sleep clock accuracy */
    UINT16 rsvd:4;             /* reserved */
    UINT16 scan_interval;      /* LE scan interval */
    UINT16 scan_window;        /* LE scan window */
    UINT16 conn_interval_min;  /* min connection interval */
    UINT16 conn_interval_max;  /* max connection interval */
    UINT16 conn_latency;       /* connection latency */
    UINT16 supervision_to;     /* supervision timeout */
    UINT16 ce_len_min;         /* min CE length */
    UINT16 ce_len_max;         /* max CE length */
    UINT32 access_address;     /* access_address */
    UINT16 tx_win_offset;      /* transmit window offset */
    UINT8 tx_win_size;         /* transmit window size */
    UINT8 conn_ch_map[5];      /* channel map in connection state */
    UINT8 hop_increment;       /* hopping channel increment value (5~16)*/
    UINT8 entry_id;            /* entry id */

    union {
        UINT8 remote_addr[6];  /* remote device address */
        UINT16 u2rem_addr[3];
    };

    UINT16 conn_interval;      /* connection interval */
    UINT16 ce_len;             /* CE length */
} LL_INITIATOR_UNIT, *PLL_INITIATOR_UNIT;

/*==========================================================*/
/* SW Structure of Device Address List Entry                */
/*==========================================================*/
typedef union LL_DEV_ADDR_LIST_ENTRY_ {
    struct {
        UINT8 addr[6];              /* the device address field */
        UINT8 type:1;               /* the type of device address */
        UINT8 valid:1;              /* current entry is valid */
        UINT8 duplicated:1;         /* current entry is duplicated */
        UINT8 rsvd:4;               /* reserved */
        UINT8 gen_report:1;         /* whether the data written to CAM */
        UINT8 next;                 /* the next entry of device address */
    };
    UINT32 DWord[2];
} LL_DEV_ADDR_LIST_ENTRY, *PLL_DEV_ADDR_LIST_ENTRY;

typedef union LL_RESOLVING_LIST_ENTRY_ {
    struct {
        UINT32 Local_IRK[4];
        UINT32 Peer_IRK[4];
        UINT8  addr[6];
        UINT8  type:1;
        UINT8  valid:1;
        UINT8  valid_local_IRK:1;
        UINT8  valid_peer_IRK:1;
        UINT8  rsvd:4;
        UINT8  next;
        UINT16 local_rpa[3];
        UINT16 peer_rpa[3];
    };
    UINT32 DWord[13];
} LL_RESOLVING_LIST_ENTRY;

/*===========================================================*/
/* SW Structure of TRX FIFO Information during the CE        */
/*===========================================================*/
typedef struct LL_DEV_ADDR_LIST_CENTER_ {
    union {
        LL_DEV_ADDR_LIST_ENTRY *entry;  /* the entry pointer for free/used list */
        LL_RESOLVING_LIST_ENTRY *item;  /* the item pointer for free/used list */
    };
    UINT8 free_list;                /* the head of free list */
    UINT8 list_head;                /* the head of used list */
    UINT8 list_tail;                /* the tail of used list */
    UINT8 rsvd;                     /* reserved */
} LL_DEV_ADDR_LIST_CENTER, *PLL_DEV_ADDR_LIST_CENTER;

/*==========================================================*/
/* The Data Structure of FW LL Manager                      */
/*==========================================================*/
typedef struct LL_MANAGER_ {
    UINT8 le_event_mask[8];             /* supported LE event mask */
    UINT8 cur_mode;                     /* current operation mode */
    UINT8 adv_tx_power_idx;             /* current advertising
                                           channel tx power index */
    UINT16 int_imr;                     /* interrupt IMR value */
    UINT16 ext_int_imr;                 /* Ext interrupt IMR value */

#ifdef _BT4_2_PRIVACY_FEATURE_SUPPORT_
    UINT16 RPA_timeout;                 /* RPA timeout sec value set by host */
    TimerHandle_t *RPA_timer;                /* resolvable private address timer */
    UINT8 local_IRK_idx;                /* index of local IRK */
#endif

    union {
        UINT8 local_random_address[6];  /* local random device address */
        UINT16 u2local_random_addr[3];
    };
    union {
        UINT8 local_public_address[6];  /* local public device address */
        UINT16 u2local_public_addr[3];
    };
    LL_ADVERTISER_UNIT adv_unit;        /* the advertising unit */
    LL_SCANNER_UNIT scan_unit;          /* the scanning unit */
    LL_INITIATOR_UNIT initiator_unit;   /* the initiator unit */
    LL_CONNECTION_UNIT conn_unit;       /* the connection unit */
    LL_TEST_BLK test_unit;              /* the test mode unit */

    LL_EVENT_MANAGE event_manager;      /* the event information of CE/AE */

    LL_DEV_ADDR_LIST_CENTER white_list; /* the white list management */
    LL_DEV_ADDR_LIST_CENTER black_list; /* the black list management */
#ifdef _BT4_2_PRIVACY_FEATURE_SUPPORT_
    LL_DEV_ADDR_LIST_CENTER resolving_list;   /* the resolving list management */
    UINT8 address_resolution_enable:1;  /* enable / disable privacy */
    UINT8 support_ll_privacy:1;         /* support LE link layer privacy */
    UINT8 debug_privacy_enable:1;       /* enable / disable debug privacy */
    UINT8 show_resolvable_addr:1;       /* enable / disable privacy log */
    UINT8 comp_rpa_lbd_opt_enable:1;    /* enable / disable BIT12 of LE reg(0x180) in ll_fw_init() */
    UINT8 force_bt40_opt:1;             /* enable / disable BIT2 of LE reg(0x1c2) in ll_fw_init() */
    UINT8 rsvd:2;                       /* reserved */
#endif
#ifdef _BT5_0_LE2MBPS_SUPPORT_
    UINT8 support_le_2mbps:1;           /* support LE 2Mbps */
    UINT8 rsvd_2m:7;                     /* reserved for new features. */
#endif
#ifdef _DAPE_UNIFY_VERSION_FEATURE_UINT
    UINT8 ll_feature_support[8];        /* list the features our device supports. */
                                        /* Byte[0]Bit 0: LE Encryption. */
                                        /* Byte[0]Bit 1: Connection Parameters Request procedure. */
                                        /* Byte[0]Bit 2: Extended Reject Indication. */
                                        /* Byte[0]Bit 3: Slave-Init Feature Exchange. */
                                        /* Byte[0]Bit 4: LE Ping. */
                                        /* Byte[0]Bit 5: LE Data Packet Length Extension. */
                                        /* Byte[0]Bit 6: LE Privacy. */
                                        /* Byte[0]Bit 7: Extended Scanner Filter Polocies. */
    UCHAR ll_version;
#endif
} LL_MANAGER, *PLL_MANAGER;

extern LL_MANAGER ll_manager;
extern POOL_ID le_llc_pdu_pool_id;
extern POOL_ID le_acl_data_to_ll_pool_id;
extern POOL_ID le_acl_data_to_host_pool_id;
extern LL_DEV_ADDR_LIST_ENTRY le_white_entry[LL_MAX_WHITE_LIST_SIZE];
extern LL_DEV_ADDR_LIST_ENTRY le_black_entry[LL_MAX_BLACK_LIST_SIZE];
#ifdef _BT4_2_PRIVACY_FEATURE_SUPPORT_
extern UINT8 le_resolving_list_size;
extern LL_RESOLVING_LIST_ENTRY le_resolving_list[LL_MAX_HW_RESOLVING_LIST_SZ];
extern TimerHandle_t resolvable_private_address_timeout_timer;
#endif
extern SECTION_SRAM UINT8 ll_adv_data_tx_buf[LL_ADV_DATA_LEN_MAX + 1];
extern SECTION_SRAM UINT8 ll_scan_res_data_tx_buf[LL_SCAN_RESPONSE_DATA_LEN_MAX + 1];
extern SECTION_SRAM UINT8 ll_hw_rxfifo_buf[1024];
extern UINT8 le_tx_power_max_value;
extern UINT8 le_tx_power_max_index;
#if defined(_SCHEDULE_BLE_MISC_PACKET_) && defined(_NEW_BZDMA_FROM_V8_)
extern LL_MISC_ACL_DATA_PKT_NODE le_tx_misc_node[LL_MISC_ACL_DATA_PKT_MAX_NODES];
#endif

#ifdef USE_NEW_LE_SCHEDULER
typedef enum LE_SCHED_PKT_TYPE_ {
    LE_SCHED_PKT_TYPE_CTRL_PDU,
    LE_SCHED_PKT_TYPE_MISC_PKT,
} LE_SCHED_PKT_TYPE;

typedef struct LE_SCHED_PKT_FIFO_NODE_
{
    union {
        LL_CTRL_PDU_PAYLOAD *ctrl_pdu;
        LL_MISC_ACL_DATA_PKT_NODE *misc_pkt;
        void *pkt;
    };
    UINT8 type;
} LE_SCHED_PKT_FIFO_NODE;

typedef struct LE_SCHED_PKT_FIFO_
{
    LE_SCHED_PKT_FIFO_NODE *node;
    UINT8 size;
    UINT8 start;
    UINT8 end;
    UINT8 len;
} LE_SCHED_PKT_FIFO;

//extern LE_SCHED_PKT_FIFO *le_sched_pkt_fifo;
extern LE_SCHED_PKT_FIFO le_sched_pkt_fifo[LL_MAX_CONNECTION_UNITS];
void le_sched_pkt_fifo_reset_entry(UINT8 entry);
void le_sched_pkt_fifo_flush_entry(UINT8 entry);
void le_sched_pkt_fifo_init(void);
static inline BOOLEAN le_sched_pkt_fifo_is_empty(UINT8 entry)
{
    return le_sched_pkt_fifo[entry].len == 0;
}
static inline BOOLEAN le_sched_pkt_fifo_is_full(UINT8 entry)
{
    return le_sched_pkt_fifo[entry].len >= le_sched_pkt_fifo[entry].size;
}
BOOLEAN le_sched_pkt_fifo_enqueue(UINT8 entry, LE_SCHED_PKT_TYPE type, void *pkt);
#endif /* USE_NEW_LE_SCHEDULER */

typedef void (*LL_ADV_HANDLER)(LE_HW_ADVERTISING_CH_RX_PKT_S *, LE_HW_RX_PKT_TAIL_S *, BOOLEAN *);
extern LL_ADV_HANDLER ll_adv_handler[LL_ADV_PDU_TYPE_MAX + 1];


#ifdef _ENABLE_BLE_LL_FLOW_CONTROL_
extern UINT8 g_le_enter_flow_stop_flag;
extern UINT8 g_le_flow_go_threshold_pkt_cnt;
extern UINT8 g_le_flow_stop_threshold_pkt_cnt;
#endif

void ll_init(void);
#ifdef _CCH_RETENTION_FLOW_FOR_DLPS_
void ll_hw_init(UINT8 dlps_flow);
#endif

LL_CONN_HANDLE_UNIT *ll_fw_search_handle_unit_via_conn_handle(UINT16 conn_handle);
UINT8 ll_fw_get_free_conn_entry(void);
void ll_fw_fill_remote_conn_entry_param(UINT8 role,UINT8 conn_entry);
void ll_fw_reset_remote_conn_entry_param(UINT8 conn_entry);
void ll_fw_init_access_address_index(void);
UINT32 ll_fw_generate_access_address(void);
UINT16 ll_fw_compute_advertiser_h2h_min_duration(void);
void ll_fw_free_all_pkts_in_conn_entry(UINT8 conn_entry);
UINT8 ll_fw_check_multiple_states_valid(UINT8 unconn_state,UINT8 type);
UINT32 ll_fw_get_gcd(UINT32 a, UINT32 b);
void ll_fw_check_all_slave_interval(void);
UINT16 ll_fw_get_adv_hit_counter_n(void);
UINT8 ll_fw_conn_update(UINT8 conn_entry, UINT8 tx_win_size,
	                    UINT16 tx_win_offset,
	                    UINT16 conn_interval, UINT16 conn_latency,
	                    UINT16 supervision_timeout);
void ll_fw_get_slot_offset(UINT8 conn_entry, UINT8 role);
void ll_fw_get_update_slot(UINT8 entry, UINT8 type);

#ifdef _CCH_SLOT_OFFSET_
#ifdef _DAPE_TEST_SLOT_OFFSET_FOR_LE
void ll_fw_compute_slot_offset_and_occupy(UINT8 conn_entry, UINT8 role);
#endif
#endif
void ll_fw_get_all_anchor_point(UINT32 *anchor_point_slot_max,
                                UINT32 *anchor_point_slot_min,
                                UINT32 *next_anchor_point_slot_min,
                                UINT8 *num_le_conn, UINT16 *empty_entry,
                                UINT32 cur_clock);

void ll_pop_all_rx_int_stack(void);
void ll_pre_cleanup_tx_schedule_of_event_end(void);
void ll_cleanup_rx_status(void);

#ifdef _BT4_2_DATA_EXTENSION_FEATURE_SUPPORT_
BOOLEAN ll_fw_validate_ll_length_param(UINT16 size, UINT16 time);
BOOLEAN ll_fw_update_data_length(LL_CONN_HANDLE_UNIT *chu);
BOOLEAN ll_fw_pend_data_length_update_procedure(LL_CONN_HANDLE_UNIT *chu);
#ifdef _BT4_2_DATA_EXTENSION_FEATURE_SUPPORT_INFO_
void ll_fw_print_data_length(LL_CONN_HANDLE_UNIT *chu);
#endif
#endif /* _BT4_2_DATA_EXTENSION_FEATURE_SUPPORT_ */
#ifdef _BT4_1_SUPPORT_LL_CONN_PARAM_REQ
void ll_decide_connection_parameters(UINT8 entry);
#endif
void ll_append_acl_rx_pkt_to_hci_list(LL_HCI_ACL_DATA_PKT *acl_pkt);
void ll_append_acl_rx_pkt_list_to_hci_list(LE_ACL_PKT_LIST_MANAGE * acl_rx_list);

void ll_append_acl_tx_pkt_to_pend_list(LL_HCI_ACL_DATA_PKT * acl_pkt,
                                       UINT8 conn_entry);
void ll_append_acl_tx_pkt_list_to_list(LE_ACL_PKT_LIST_MANAGE * acl_tx_list,
                                UINT8 conn_entry,UINT8 policy,UINT8 list_type);
void ll_free_acl_tx_pkts_in_free_list(UINT8 conn_entry);

void ll_handle_rx_data_pdu(UINT8 * payload,UINT8 len,UINT8 frag1st,UINT8 entry);
void ll_handle_generate_hci_event_signal(LL_TASK_PARA_U * param);

UINT16 ll_pre_parser_adv_channel_pdu(UINT8 * pdata,UINT16 len);

#if defined(_SCHEDULE_BLE_MISC_PACKET_) && defined(_NEW_BZDMA_FROM_V8_)
UINT8 ll_get_le_tx_misc_node(void);

void ll_free_le_tx_misc_node(UINT8 node_id);

void ll_append_acl_tx_misc_pkt_to_pend_list(UINT8 Node_Id,UINT8 conn_entry);

void ll_append_acl_tx_misc_pkt_list_to_list(LE_ACL_MISC_PKT_LIST_MANAGE *acl_tx_list,
                                       UINT8 conn_entry, UINT8 policy,
                                       UINT8 list_type);

void ll_free_acl_tx_misc_pkts_in_free_list(UINT8 conn_entry);

void ll_reassemble_misc_tx_pkt_from_acl_pend_list(UINT8 conn_entry, UINT8 max_tx_size);
void ll_reassemble_misc_tx_pkt_from_acl_misc_pend_list(UINT8 conn_entry,UINT8 max_tx_size);
#endif

void ll_parser_adv_channel_pdu(UINT8 *pdata, UINT16 len);
void ll_parser_data_channel_pdu(UINT8 *pdata, UINT16 len, UINT8 entry);
void ll_handle_master_connection_complete_process_second_part(UINT8 entry);

void ll_start_timer(UINT8 conn_entry,UINT8 timer_type);
void ll_supervision_timeout_callback(TimerHandle_t timer_handle);
void llc_timeout_callback(TimerHandle_t timer_handle);
#ifdef _BT4_1_LE_PING_FEATURE_SUPPORT_
void llc_ping_req_restart_timer(UINT8 entry);
#endif

UINT8 llc_schedule_channel_map_update_procedure(LL_CONN_HANDLE_UNIT * pHandle);
UINT8 llc_schedule_connection_update_procedure(LL_CONN_HANDLE_UNIT * pHandle);
UINT8 llc_schedule_encryption_procedure(LL_CONN_HANDLE_UNIT * pHandle);
UINT8 llc_schedule_feature_exchange_procedure(LL_CONN_HANDLE_UNIT * pHandle);
UINT8 llc_schedule_version_exchange_procedure(LL_CONN_HANDLE_UNIT * pHandle);
#ifdef _BT4_1_SUPPORT_LL_CONN_PARAM_REQ
UINT8 llc_schedule_connection_parameter_procedure(LL_CONN_HANDLE_UNIT *pHandle);
#endif
#ifdef _BT4_1_LE_PING_FEATURE_SUPPORT_
UINT8 llc_schedule_le_ping_procedure(LL_CONN_HANDLE_UNIT *pHandle);
#endif
#ifdef _BT4_2_DATA_EXTENSION_FEATURE_SUPPORT_
BOOLEAN llc_schedule_data_length_update_procedure(LL_CONN_HANDLE_UNIT *chu, LLC_DATA_LEN_UPDATE_STATE state);
BOOLEAN llc_finish_data_length_update_procedure(LL_CONN_HANDLE_UNIT *chu);
#endif
void llc_stop_procedure_by_ll_unknown_rsp(LL_CONN_HANDLE_UNIT *chu, enum LL_CTRL_PDU_OPCODES opcode);
void llc_stop_procedure_by_ll_reject_ind(LL_CONN_HANDLE_UNIT *chu, UINT8 err_code);
void llc_stop_procedure_by_ll_reject_ind_ext(LL_CONN_HANDLE_UNIT *chu, enum LL_CTRL_PDU_OPCODES rej_opcode, UINT8 err_code);
void llc_check_and_schedule_next_pend_pdu(UINT8 conn_entry);

void llc_append_pdu_list_to_free_list(LE_ACL_PKT_LIST_MANAGE *llc_pdu_list);
void llc_append_pdu_to_tx_list(LL_CTRL_PDU_PAYLOAD * llc_pdu,UINT8 conn_entry);
void llc_append_pdu_list_to_list(LE_ACL_PKT_LIST_MANAGE * llc_pdu_list,
                                UINT8 conn_entry,UINT8 policy,UINT8 list_type);
void llc_free_pdu_in_free_list(void);

void llc_handle_sent_llc_pdu_ack_recd(LL_CONN_HANDLE_UNIT * pHandle, UINT8 OpCode);
void llc_check_sent_llc_pdu_request(LL_CONN_HANDLE_UNIT *pHandle, UINT8 OpCode);

LL_CTRL_PDU_PAYLOAD* llc_generate_ll_channel_map_req(UINT8 conn_entry);
LL_CTRL_PDU_PAYLOAD* llc_generate_ll_connection_update_req(UINT8 conn_entry);
LL_CTRL_PDU_PAYLOAD* llc_generate_ll_enc_req(UINT8 conn_entry);
LL_CTRL_PDU_PAYLOAD* llc_generate_ll_enc_rsp(UINT8 conn_entry);
LL_CTRL_PDU_PAYLOAD* llc_generate_ll_feature_req(void);
LL_CTRL_PDU_PAYLOAD* llc_generate_ll_feature_rsp(UINT8 conn_entry);
LL_CTRL_PDU_PAYLOAD* llc_generate_ll_pause_enc_req(void);
LL_CTRL_PDU_PAYLOAD* llc_generate_ll_pause_enc_rsp(void);
LL_CTRL_PDU_PAYLOAD* llc_generate_ll_reject_ind(UINT8 err_code);
LL_CTRL_PDU_PAYLOAD* llc_generate_ll_reject_ind_ext(UINT8 rej_opcode, UINT8 err_code);
LL_CTRL_PDU_PAYLOAD* llc_generate_ll_start_enc_req(void);
LL_CTRL_PDU_PAYLOAD* llc_generate_ll_start_enc_rsp(void);
LL_CTRL_PDU_PAYLOAD* llc_generate_ll_terminate_req(UINT8 err_code);
LL_CTRL_PDU_PAYLOAD* llc_generate_ll_unknown_rsp(UINT8 unknown_type);
LL_CTRL_PDU_PAYLOAD* llc_generate_ll_version_ind(void);
#ifdef _BT4_2_DATA_EXTENSION_FEATURE_SUPPORT_
LL_CTRL_PDU_PAYLOAD *llc_generate_ll_length_req(LL_CONN_HANDLE_UNIT *chu);
LL_CTRL_PDU_PAYLOAD *llc_generate_ll_length_rsp(LL_CONN_HANDLE_UNIT *chu);
#endif
#ifdef _SUPPORT_VER_4_1_
LL_CTRL_PDU_PAYLOAD* llc_generate_ll_ping_req(UINT8 conn_entry);
LL_CTRL_PDU_PAYLOAD* llc_generate_ll_ping_rsp(UINT8 conn_entry);
LL_CTRL_PDU_PAYLOAD* llc_generate_ll_slave_feature_req(void);
#ifdef _BT4_1_SUPPORT_LL_CONN_PARAM_REQ
LL_CTRL_PDU_PAYLOAD* llc_generate_ll_connection_parameter_req(UINT8 conn_entry);
#endif
void llc_handle_ll_slave_feature_req(UINT8 *pdata, UINT8 len, UINT8 entry);
void llc_handle_ll_connection_param_req(UINT8 *pdata, UINT8 len, UINT8 entry);
void llc_handle_ll_connection_param_rsp(UINT8 *pdata, UINT8 len, UINT8 entry);
void llc_handle_ll_reject_ind_ext(UINT8 *pdata, UINT8 len, UINT8 entry);
void llc_handle_ll_ping_req(UINT8 *pdata, UINT8 len, UINT8 entry);
void llc_handle_ll_ping_rsp(UINT8 *pdata, UINT8 len, UINT8 entry);


#endif
UINT8 llc_generate_pdu(UINT8 opcode,LL_CTRL_PDU_PAYLOAD **ppPkt);
void llc_parser_ctrl_pdu(UINT8 * payload,UINT8 len,UINT8 conn_entry);

UINT8 ll_handle_acl_disconnect(UINT16 conn_handle, UCHAR reason,
                                     UCHAR from_host);
void ll_handle_host_data_pkt(LL_HCI_ACL_DATA_PKT * pkt);


void ll_handle_rxfifo_in_task(void * param);
SECTION_LE_ISR void ll_interrupt_handler(void);

#ifdef _MODI_LPS_AFTER_RTL8703B_
void ll_program_ae_sm_mode(void);
#endif

#ifdef _SUPPORT_LBT_FUNC_FOR_RF_ADAPTIVITY_
UINT8 rtl8723_btrf_check_and_enable_lbt(UINT8 power_index);
#endif

extern void (*ll_handle_update_interrupt)(UINT8 type);
extern void (*ll_handle_ctrl_pdu_tx_ack)(LL_CONN_HANDLE_UNIT *chu, LL_CTRL_PDU_PAYLOAD *pdu);

#endif

