/***************************************************************************
 Copyright (C) Mindtree Ltd.
 This module is a confidential and proprietary property of MindTree and
 a possession or use of this module requires written permission of MindTree.
 ***************************************************************************/

/**
 * \file
 *  Structure and macro definitions of LMP layer.
 */

/** \addtogroup lmp_external
 *  @{ */
#ifndef __LMP_EXTERNAL_DEFINES_H__
#define __LMP_EXTERNAL_DEFINES_H__

#include "bt_fw_os.h"
#include "bt_fw_types.h"
#include "bt_fw_globals.h"
#include "lmp_spec_defines.h"

#define MAX_EIR_DATA_LEN                    240
#define BT_CLOCK_26_BITS                   (0x3FFFFFF)
#define BT_CLOCK_27_BITS                   (0x7FFFFFF)
#define BT_CLOCK_28_BITS                   (0xFFFFFFF)

#define BT_CLOCK_NORMAL_CASE               (0x01)
#define BT_CLOCK_MORE_THAN_12_HRS_AWAY     (0x02)
#define BT_CLOCK_CLK_WRAP_AROUND_CASE      (0x03)

#define LMP_AFH_MAP_SIZE                    10

#define LMP_MAX_JITTER                      10
#define LMP_MAX_DRIFT                       250
#define DSM_DRIFT                           125    /* 125us */

#define LMP_NUM_PKTS_EVENT_TIMER_VAL        200    /* 200 ms. */
#ifdef _CORRECT_LMP_RESPONSE_TIMER
extern UINT32 g_lmp_response_timeout_time;
#endif
/* PDU Responce timer is increased to 90 sec to take into account pdu
 * transactions in low power modes
 */
#ifdef _CORRECT_LMP_RESPONSE_TIMER
#define LMP_RESPONSE_TIMEOUT                g_lmp_response_timeout_time  /* 30 seconds */
#else
#define LMP_RESPONSE_TIMEOUT                90000  /* 90 seconds */
#endif
#define LMP_RESPONSE_TIMEOUT_SLOT           144000  /* 90 seconds = 144000 slots */

#define LMP_AUTH_TID_TIMEOUT                30000  /* 30 seconds */


/* Only one LAP is supported by Baseband. */
#define LMP_MAX_IAC_LAPS                    1

#define LMP_FHS_PKT_LEN                     18
#define LMP_FEATURE_BIT_FOR_3SLOT_PACKETS   0x01
#define LMP_FEATURE_BIT_FOR_5SLOT_PACKETS   0x02

#define LMP_FEATURE_BIT_FOR_2MBPS_PACKETS   0x02
#define LMP_FEATURE_BIT_FOR_3MBPS_PACKETS   0x04
#define LMP_FEATURE_BIT_FOR_3SLOT_EDR_PACKETS   0x80
#define LMP_FEATURE_BIT_FOR_5SLOT_EDR_PACKETS   0x01

#define LMP_MAX_EVENT_FILTERS               16
#define LEGACY_MAX_WHITE_LIST               6

#define LMP_CLASS_OF_DEVICE_LEN             3
#define LMP_FEATURES_SIZE                   8
#define LMP_NUM_RESPONSES                   1

#define NO_RESP_OR_REQ_PDU                  0

/* Ideal value of sniff interval.... */
#define LMP_DEF_SNIFF_INTERVAL              500
#define LMP_DEF_HOLD_INTERVAL               500

#define HCI_MIN_VALUE_OF_SNIFF_MAX_SUPPORTED           0x0006
#define HCI_MIN_VALUE_OF_SNIFF_MIN_SUPPORTED           0x0002

#define	INVALID_VOICE_SETTINGS		        0xFFFF

/*
 * Minimum link supervision timeout is 0.625ms (1 slot.)
 * Maximum link supervision timeout is 40.9 seconds.
 */
#define LMP_MIN_LINK_SUPERVISION_TIMEOUT    0x190  /* 1 Slot */
#define LMP_MAX_LINK_SUPERVISION_TIMEOUT    0xFFFF /* In terms of slots */

/* This should be picked up from config. */
//#define LMP_SUPERVISION_TIMER_RESOLUTION_EXPONENT    6    /* 40 ms */
#define LMP_SUPERVISION_TIMER_RESOLUTION_EXPONENT    10     /* 640 ms */

/* The value of LMP_SUPERVISION_TIMER_RESOLUTION is in slots */
#define LMP_SUPERVISION_TIMER_RESOLUTION    (1 << LMP_SUPERVISION_TIMER_RESOLUTION_EXPONENT)
#define LMP_SUP_TO_TPOLL_DIVISOR            3

/* 40 slot offset for hold mode negotiation. */
#define LMP_PICONET_SWITCH_OFFSET       40

/* Max Tpoll is 100ms. */
//#define LMP_MAX_TPOLL 160
#define LMP_MAX_TPOLL 0x1000 /* modified by austin to meet LMP spec */

/* Min Tpoll is 10ms. */
#define LMP_MIN_TPOLL  6

#define HCI_E2P_PARAM_LOW_CLK_EXT       0x1
#define HCI_E2P_PARAM_LOW_CLK_INT       0x0


#define HCI_E2P_PARAM_LOW_CLK_FRAC      0x0
#define HCI_E2P_PARAM_LOW_CLK_WHOLE     0x1


#define HCI_E2P_PARAM_SYS_CLK_13M       0x0c    /* 13-1 */
#define HCI_E2P_PARAM_SYS_CLK_16M       0x0f    /* 16-1 */
#define HCI_E2P_PARAM_SYS_CLK_18M       0x11    /* 18-1 */
#define HCI_E2P_PARAM_SYS_CLK_24M       0x17    /* 24-1 */
#define HCI_E2P_PARAM_SYS_CLK_8M        0x07    /* 8-1  */


#define HCI_E2P_PARAM_MT_SIW_RADIO      0x1
#define HCI_E2P_PARAM_MT_BLUERF_RADIO   0x2
#define HCI_E2P_PARAM_MT_CUSTOM_RADIO_2 0x3

/* Device Information data structures... */
#define MAX_CACHE           5
#define INVALID_INDEX       0xFF
#define INVALID_ESCO_INDEX  0xFF

#define CACHE_FREE          0
#define CACHE_TAKEN         1


#define LMP_GET_TRANSACTION_ID(lmp_pdu_ptr) \
            (lmp_pdu_ptr->payload_content[0] & 0x01)

/* Convert the slot value into milliseconds. */
#define SLOT_VAL_TO_TIMER_VAL(slot_value)  \
               (UINT32)(((slot_value) * 5) >> 3)
#define TIMER_VAL_TO_SLOT_VAL(timer_value)  \
               (UINT32)(((timer_value) << 3) / 5)

#define TIMER_VAL_TO_KERNEL_TICK_VAL(timer_val) \
         ((timer_val) - ((timer_val) % (KERNEL_TICKS)))

/* Error returns */

#define FOUND       1
#define NOT_FOUND   0

#define MATCH       0

#define UNASSIGNED  0
#define ASSIGNED    1


/* Configurable parameters */

#define LMP_NUM_ELEMENTS_IN_HASH_Q           3
#define LMP_MAX_BD_TBL_HASH_BUCKETS          (LMP_MAX_BD_TO_CE_TBLS/LMP_NUM_ELEMENTS_IN_HASH_Q)

/* Page Scan and Inquiry Scan Values */
#define LMP_MIN_PAGE_SCAN_INTERVAL  0x0012
#define LMP_MIN_PAGE_SCAN_WINDOW    0x0011
#define LMP_MAX_PAGE_SCAN_INTERVAL  0x1000
#define LMP_MAX_PAGE_SCAN_WINDOW    0x1000

#define LMP_MIN_INQ_SCAN_INTERVAL   0x0012
#define LMP_MIN_INQ_SCAN_WINDOW     0x0011
#define LMP_MAX_INQ_SCAN_INTERVAL   0x1000
#define LMP_MAX_INQ_SCAN_WINDOW     0x1000

/* Number of slots after which power control is done */
#define LMP_POWER_CONTROL_CYCLE            8000
#define LMP_MID_POWER                         0
#define LMP_MAX_POWER_RCVD                    1
#define LMP_MIN_POWER_RCVD                    2
#define POWER_CONTROL_RESERVED_BYTE_2      0x00

/*
 * Link policy settings defines
 */
#define LMP_LP_DISABLE_ALL_LM_MODES        0X0000
#define LMP_LP_MASTER_SLAVE_SWITCH         0X0001
#define LMP_LP_MASTER_HOLD_MODE            0X0002
#define LMP_LP_MASTER_SNIFF_MODE           0X0004
#define LMP_LP_MASTER_PARK_MODE            0X0008

/*
 * Supported features
 */
#define LMP_THREE_SLOT_PACKET_FEATURE              0X0001
#define LMP_FIVE_SLOT_PACKET_FEATURE               0X0002
#define LMP_ENCRYPTION_FEATURE                     0X0004
#define LMP_SLOT_OFFSET_FEATURE                    0X0008
#define LMP_TIME_ACCURACY_FEATURE                  0X0010
#define LMP_SWITCH_FEATURE                         0X0020
#define LMP_HOLD_MODE_FEATURE                      0X0040
#define LMP_SNIFF_MODE_FEATURE                     0X0080

#define LMP_PARK_MODE_FEATURE                      0X0001
#define LMP_POWER_CONTROL_REQ_FEATURE              0X0002
#define LMP_CHN_QLTY_FEATURE                       0X0004
#define LMP_SCO_LINK_FEATURE                       0X0008
#define LMP_HV2_PKTS_FEATURE                       0X0010
#define LMP_HV3_PKTS_FEATURE                       0X0020
#define LMP_U_LAW_LOG_FEATURE                      0X0040
#define LMP_A_LAW_LOG_FEATURE                      0X0080

#define LMP_CVSD_FEATURE                           0X0001
#define LMP_PAGE_SCHEME_FEATURE                    0X0002
#define LMP_POWER_CONTROL_FEATURE                  0X0004
#define LMP_TRANS_SCO_DATA_FEATURE                 0X0008
#define LMP_FLOW_LAG0_FEATURE                      0X0010
#define LMP_FLOW_LAG1_FEATURE                      0X0020
#define LMP_FLOW_LAG2_FEATURE                      0X0040
#define LMP_BC_ENCRYPTION_FEATURE                  0X0080

//{{ added by guochunxia
#define LMP_EDR_3_SLOT_PACKET_FEATURE              0X0080
#define LMP_EDR_5_SLOT_PACKET_FEATURE              0X0001
#define LMP_2M_1_SLOT_PACKET_FEATURE               0X0002
#define LMP_3M_1_SLOT_PACKET_FEATURE               0X0004
//}}

/* Bit mask for Commands initiated by host */
#define REMOTE_NAME_REQ_BIT_MASK                   0x0001
#define REMOTE_SUP_FEA_BIT_MASK                    0x0002
#define REMOTE_EX_FEA_BIT_MASK                     0x0004
#define REMOTE_VER_INFO_BIT_MASK                   0x0008
#define READ_CLOCK_OFFSET_BIT_MASK                 0x0010
#define CHANGE_CONN_PKT_TYPE_BIT_MASK              0x0020
#define REMOTE_NAME_REQ_CON_BIT_MASK               0x0040

#define REMOTE_SUP_FEA_TRUE_HOST_BIT_MASK          0x8000
#define REMOTE_EX_FEA_TRUE_HOST_BIT_MASK           0x4000

#ifdef _CCH_RTL8723A_B_CUT
#define CON_REASON_CREATE_CON                      0x0000
#define CON_REASON_REMOTE_NAME_REQ                 0x0001
#ifdef _SUPPORT_CSB_RECEIVER_
#define CON_REASON_TRUNCATED_PAGE                  0x0002
#endif
#endif


/* Default Park Params */
#define DEFAULT_DELTA_b         8
#define DEFAULT_Nb              8
#define DEFAULT_M_acc           4
#define DEFAULT_T_acc           4
#define DEFAULT_N_acc_slots     4
#define DEFAULT_GAP             30
#define DEFAULT_D_acc           (DEFAULT_Nb * DEFAULT_DELTA_b) + DEFAULT_GAP
#define DEFAULT_N_poll          20
#define DEFAULT_Nb_sleep        1
#define DEFAULT_Db_sleep        0

/* Minimum Park Params */
#define MIN_DELTA_b         2
#define MIN_Nb              4
#define MIN_M_acc           2
#define MIN_T_acc           2
#define MIN_N_acc_slots     2
#define MIN_GAP             2
#define MIN_D_acc           (MIN_Nb * MIN_DELTA_b) + MIN_GAP
#define MIN_N_poll          20
#define MIN_Nb_sleep        1
#define MIN_Db_sleep        0

/* Maximum number of extended features req page */
#if defined(COMPILE_FEATURE_REQ_EXT)
/* SSP needs extended Page-1 */
    #ifndef _SUPPORT_EXT_FEATURES_PAGE_2_
#define LMP_MAX_EXTENDED_FEAT_REQ_PAGES      1
#else
#define LMP_MAX_EXTENDED_FEAT_REQ_PAGES      2
    #endif
#else
#define LMP_MAX_EXTENDED_FEAT_REQ_PAGES      0
#endif

#define LMP_MAX_FEAT_REQ_PAGES  (1+LMP_MAX_EXTENDED_FEAT_REQ_PAGES)

/* Maximum number of remote extended features pages */
#define LMP_MAX_REMOTE_EXT_FEAT_REQ_PAGE     1

/* 2 bytes are for padding */
#define MAX_PKT_TYPES        (7 + 1 + 6 + 2)


#define LMP_HV1                                 0
#define LMP_HV2                                 1
#define LMP_HV3                                 2

#define LMP_PACKET_TYPE_MASK            0x03
#define LMP_INVALID_PACKET_TYPE_MASK    0xFC

#define HCI_LAYER         4
#define LMP_LAYER         3
#define LC_LAYER          2

/* Context for calculating/validating max_slot/max_slot_req */
#define TX_MAX_SLOT_REQ     1
#define RX_MAX_SLOT_REQ     2
#define TX_MAX_SLOT         3
#define RX_MAX_SLOT         4
#define FEATURE_BASED       5

/* This the preferred max_slot when a particular feature is up */
#define PREFERRED_MAX_SLOT_IN_SNIFF             1
#define PREFERRED_MAX_SLOT_WITH_SCO             1
#define PREFERRED_MAX_SLOT_WITH_ESCO            5

#define PER_CONN_LINK_POLICY_SETTINGS           1
#define DEFAULT_LINK_POLICY_SETTINGS            2

// modified from 0x07 to 0xe0 by guochunxia
#define POLICY_SETTINGS_MASK0                   0xe0//0x07 /* Feature byte 0 */
#define POLICY_SETTINGS_MASK1                   0x01 /* Feature byte 1 */

/* Number of retries in case of NBC timeout */
#define LMP_MAX_NBC_TIMEOUT             3

/* Minimum sniff interval to skip data transfer optimization */
//#define LMP_MIN_SNIFF_INTERVAL_TO_SKIP_DATA_TX_OPT 20
//#define LC_MIN_SLOTS_TO_SKIP_MARK_AS_BAD_FOR_SNIFF 4

extern UINT16 g_lmp_min_sniff_intv_to_skip_data_tx_opt;
extern UINT32 g_lc_min_slots_to_skip_mark_as_bad_for_sniff;

#define LMP_MIN_SNIFF_INTERVAL_TO_SKIP_DATA_TX_OPT  \
                    g_lmp_min_sniff_intv_to_skip_data_tx_opt
#define LC_MIN_SLOTS_TO_SKIP_MARK_AS_BAD_FOR_SNIFF  \
                    g_lc_min_slots_to_skip_mark_as_bad_for_sniff

#define BIT_MASK_NO_PDU                              0x00000000
#define BIT_MASK_LMP_NAME_RES_OPCODE                 0x00000001
#define BIT_MASK_LMP_FEATURES_RES_OPCODE             0x00000002
#define BIT_MASK_LMP_FEATURES_RES_EXT_OPCODE         0x00000004
#define BIT_MASK_LMP_VERSION_RES_OPCODE              0x00000008
#define BIT_MASK_LMP_CLKOFFSET_RES_OPCODE            0x00000010
#define BIT_MASK_LMP_SLOT_OFFSET_OPCODE              0x00000020
#define BIT_MASK_LMP_SETUP_COMPLETE_OPCODE           0x00000040
#define BIT_MASK_LMP_SNIFF_SUBRATING_RES_OPCODE      0x00000080
#define BIT_MASK_LMP_MAX_POWER_OPCODE                0x00000100
#define BIT_MASK_LMP_MIN_POWER_OPCODE                0x00000200
#define BIT_MASK_LMP_SNIFF_REQ_OPCODE                0x00000400
#define BIT_MASK_LMP_HOLD_OPCODE                     0x00000800
#define BIT_MASK_LMP_HOLD_REQ_OPCODE                 0x00001000
#define BIT_MASK_LMP_PARK_REQ_OPCODE                 0x00002000
#define BIT_MASK_LMP_SCO_LINK_REQ_OPCODE             0x00004000
#define BIT_MASK_LMP_HOST_CONNECTION_REQ_OPCODE      0x00008000
#define BIT_MASK_LMP_VERSION_REQ_OPCODE              0x00010000
#define BIT_MASK_LMP_FEATURES_REQ_OPCODE             0x00020000
#define BIT_MASK_LMP_NAME_REQ_OPCODE                 0x00040000
#define BIT_MASK_LMP_CLKOFFSET_REQ_OPCODE            0x00080000
#define BIT_MASK_LMP_MAX_SLOT_REQ_OPCODE             0x00100000
#define BIT_MASK_LMP_SWITCH_REQ_OPCODE               0x00200000
#define BIT_MASK_LMP_QoS_REQ_OPCODE                  0x00400000
#define BIT_MASK_LMP_PAGE_SCAN_MODE_REQ_OPCODE       0x00800000
#define BIT_MASK_LMP_PAGE_MODE_REQ_OPCODE            0x01000000
#define BIT_MASK_LMP_REMOVE_SCO_LINK_REQ_OPCODE      0x02000000
#define BIT_MASK_LMP_FEATURES_REQ_EXT_OPCODE         0x04000000
#define BIT_MASK_LMP_SNIFF_SUBRATING_REQ_OPCODE      0x08000000
#define BIT_MASK_LMP_ESCO_LINK_REQ_OPCODE            0x10000000
#define BIT_MASK_LMP_REMOVE_ESCO_LINK_REQ_OPCODE     0x20000000
#ifdef _SUPPORT_SECURE_CONNECTION_
#define BIT_MASK_LMP_PING_REQ_OPCODE                 0x40000000
#define BIT_MASK_LMP_PING_RES_OPCODE                 0x80000000
#else
#define BIT_MASK_LMP_RESERVED_01                     0x40000000
#define BIT_MASK_LMP_RESERVED_02                     0x80000000
#endif

/************************************************************/
/*
 * Note: LMP_PDU_BUFFER_SIZE (defined in bt_fw_config.h) to be updated when
 * there is a change in LMP_PDU_PKT data structure.
 */

/** LMP PDU Packet structure */
typedef struct LMP_PDU_PKT_
{
	struct LMP_PDU_PKT_ *next;
    UCHAR payload_content[17];      /**< Payload content to be transmitted */
    UINT8 reserved_01;              /* padding for help hw 16 bit DMA access */
    UINT16 ce_index;             /**< Updated from from LC and used later to
                                        avoid dropping pdu due to piconet &
                                        am_addr mismatch because of role switch.*/    
    UCHAR pdu_length;               /**< Length of the PDU */
    UCHAR am_addr;                  /**< Active Member address on which the
                                         PDU has to transmitted. */
    UCHAR  piconet;                 /**< Piconet on which the PDU has to be
                                         transmitted */

    UCHAR  in_transmission;
    UCHAR use_dm1;                  /**< TRUE, if the PDU has to be transmitted
                                         using DM1 packet type. FALSE,
                                         otherwise */
    UCHAR pdu_sent;               /**< TRUE, if the PDU has been queued in
                                         the baseband FIFO at least once.
                                         FALSE, otherwise. */
    UCHAR nbc_count;             /**< Keep the number of NBC timeout for this
                                      pdu. This incremented on every NBC TO.
                                      And make the PDU invalid after N tries.
                                   */
    UCHAR reserved_02;           /* padding */                        
} LMP_PDU_PKT;

#define INVALID_CONN_HANDLE                 0xFFFF
#define INVALID_CE_INDEX                    0xff
#define BC_AM_ADDR                          0

UINT32 lmp_get_opcode_mask(UCHAR pdu_opcode, UCHAR pdu_ext_opcode);

#endif /* __LMP_EXTERNAL_DEFINES_H__ */

/** @} end: lmp_external */

