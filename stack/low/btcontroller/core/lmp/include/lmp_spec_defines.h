/***************************************************************************
 Copyright (C) Mindtree Ltd.
 This module is a confidential and proprietary property of MindTree and
 a possession or use of this module requires written permission of MindTree.
 ***************************************************************************/

/**
 * \file
 *  LMP related Spec definitions
 */

/** \addtogroup lmp_external
 *  @{ */
#ifndef __LMP_SPEC_DEFINES_H__
#define __LMP_SPEC_DEFINES_H__

/* ============================ LMP PDU OpCodes =========================== */

#define LMP_NAME_REQ_OPCODE                     1
#define LMP_NAME_RES_OPCODE                     2
#define LMP_ACCEPTED_OPCODE                     3
#define LMP_NOT_ACCEPTED_OPCODE                 4
#define LMP_CLKOFFSET_REQ_OPCODE                5
#define LMP_CLKOFFSET_RES_OPCODE                6
#define LMP_DETACH_OPCODE                       7
#define LMP_IN_RAND_OPCODE                      8
#define LMP_COMB_KEY_OPCODE                     9
#define LMP_UNIT_KEY_OPCODE                     10
#define LMP_AU_RAND_OPCODE                      11
#define LMP_SRES_OPCODE                         12
#define LMP_TEMP_RAND_OPCODE                    13
#define LMP_TEMP_KEY_OPCODE                     14
#define LMP_ENCRYPTION_MODE_REQ_OPCODE          15
#define LMP_ENCRYPTION_KEY_SIZE_REQ_OPCODE      16
#define LMP_START_ENCRYPTION_REQ_OPCODE         17
#define LMP_STOP_ENCRYPTION_REQ_OPCODE          18
#define LMP_SWITCH_REQ_OPCODE                   19
#define LMP_HOLD_OPCODE                         20
#define LMP_HOLD_REQ_OPCODE                     21
#define LMP_SNIFF_OPCODE                        22
#define LMP_SNIFF_REQ_OPCODE                    23
#define LMP_UNSNIFF_REQ_OPCODE                  24
#define LMP_PARK_REQ_OPCODE                     25
#define LMP_SET_BROADCAST_SCAN_WINDOW_OPCODE    27
#define LMP_MODIFY_BEACON_OPCODE                28
#define LMP_UNPARK_BD_ADDR_REQ_OPCODE           29
#define LMP_UNPARK_PM_ADDR_REQ_OPCODE           30
#define LMP_INCR_POWER_REQ_OPCODE               31
#define LMP_DECR_POWER_REQ_OPCODE               32
#define LMP_MAX_POWER_OPCODE                    33
#define LMP_MIN_POWER_OPCODE                    34
#define LMP_AUTO_RATE_OPCODE                    35
#define LMP_PREFERRED_RATE_OPCODE               36
#define LMP_VERSION_REQ_OPCODE                  37
#define LMP_VERSION_RES_OPCODE                  38
#define LMP_FEATURES_REQ_OPCODE                 39
#define LMP_FEATURES_RES_OPCODE                 40
#define LMP_QoS_OPCODE                          41
#define LMP_QoS_REQ_OPCODE                      42
#define LMP_SCO_LINK_REQ_OPCODE                 43
#define LMP_REMOVE_SCO_LINK_REQ_OPCODE          44
#define LMP_MAX_SLOT_OPCODE                     45
#define LMP_MAX_SLOT_REQ_OPCODE                 46
#define LMP_TIMING_ACCURACY_REQ_OPCODE          47
#define LMP_TIMING_ACCURACY_RES_OPCODE          48
#define LMP_SETUP_COMPLETE_OPCODE               49
#define LMP_USE_SEMI_PERMANENT_KEY_OPCODE       50
#define LMP_HOST_CONNECTION_REQ_OPCODE          51
#define LMP_SLOT_OFFSET_OPCODE                  52
#define LMP_PAGE_MODE_REQ_OPCODE                53
#define LMP_PAGE_SCAN_MODE_REQ_OPCODE           54
#define LMP_SUPERVISION_TIMEOUT_OPCODE          55
#define LMP_TEST_ACTIVATE_OPCODE                56
#define LMP_TEST_CONTROL_OPCODE                 57
#define LMP_ENCRYPTION_KEY_SIZE_MASK_REQ_OPCODE    58
#define LMP_ENCRYPTION_KEY_SIZE_MASK_RES_OPCODE    59
#define LMP_SET_AFH_OPCODE                          60
#define LMP_ENCAPSULATED_HEADER_OPCODE              61
#define LMP_ENCAPSULATED_PAYLOAD_OPCODE             62
#define LMP_SIMPLE_PAIRING_CONFIRM_OPCODE           63
#define LMP_SIMPLE_PAIRING_NUMBER_OPCODE            64
#define LMP_DHKEY_CHECK_OPCODE                      65
#ifdef _SUPPORT_SECURE_CONNECTION_
#define LMP_PAUSE_ENCRYPTION_AES_REQ_OPCODE             66
#endif
#define LMP_ESCAPE1_OPCODE                                  124
#define LMP_ESCAPE2_OPCODE                                  125
#define LMP_ESCAPE3_OPCODE                                  126
#define LMP_ESCAPE4_OPCODE                                  127

#define LMP_INVALID_PDU_OPCODE                              255


/* Escape 1 opcodes */

/* Escape 2 opcodes */

/* Escape 3 opcodes */

/* Escape 4 opcodes */
#define LMP_ACCEPTED_EXT_OPCODE                              1
#define LMP_NOT_ACCEPTED_EXT_OPCODE                          2
#define LMP_FEATURES_REQ_EXT_OPCODE                          3
#define LMP_FEATURES_RES_EXT_OPCODE                          4
#define LMP_PTT_REQ_OPCODE                                  11
#define LMP_ESCO_LINK_REQ_OPCODE                            12
#define LMP_REMOVE_ESCO_LINK_REQ_OPCODE                     13
#define LMP_CHANNEL_CLASSIFICATION_REQ_OPCODE               16
#define LMP_CHANNEL_CLASSIFICATION_OPCODE                   17
#define LMP_SNIFF_SUBRATING_REQ_OPCODE                    21
#define LMP_SNIFF_SUBRATING_RES_OPCODE                    22
#define LMP_PAUSE_ENCRYPTION_REQ_OPCODE                   23
#define LMP_RESUME_ENCRYPTION_REQ_OPCODE                  24
#define LMP_IO_CAPABILITY_REQ_OPCODE                      25
#define LMP_IO_CAPABILITY_RES_OPCODE                      26
#define LMP_NUMERIC_COMPARISON_FAILED_OPCODE              27
#define LMP_PASSKEY_FAILED_OPCODE                         28
#define LMP_OOB_FAILED_OPCODE                             29
#define LMP_KEYPRESS_NOTIFICATION_OPCODE                  30

#ifdef VER_3_0
#define LMP_POWER_CTRL_REQ_OPCODE                       31
#define LMP_POWER_CTRL_RES_OPCODE                       32
#endif

#ifdef _SUPPORT_SECURE_CONNECTION_
#define LMP_PING_REQ_OPCODE                                33
#define LMP_PING_RES_OPCODE                                34
#endif
#ifdef _SUPPORT_PCA_ADJUST
#define LMP_CLK_ADJ_OPCODE                             5
#define LMP_CLK_ADJ_ACK_OPCODE                         6
#define LMP_CLK_ADJ_REQ_OPCODE                         7
#endif
/*  PDU Lengths (Length+1) */
#define LMP_ACCEPTED_LEN                     3
#define LMP_AU_RAND_LEN                      18
#define LMP_AUTO_RATE_LEN                    2
#define LMP_CLKOFFSET_REQ_LEN                2
#define LMP_CLKOFFSET_RES_LEN                4
#define LMP_COMB_KEY_LEN                     18
#define LMP_DECR_POWER_REQ_LEN               3
#define LMP_DETACH_LEN                       3
#define LMP_ENCRYPTION_KEY_SIZE_REQ_LEN      3
#define LMP_ENCRYPTION_MODE_REQ_LEN          3
#define LMP_FEATURES_REQ_LEN                 10
#define LMP_FEATURES_RES_LEN                 10
#define LMP_HOST_CONNECTION_REQ_LEN          2
#define LMP_HOLD_LEN                         8
#define LMP_HOLD_REQ_LEN                     8
#define LMP_INCR_POWER_REQ_LEN               3
#define LMP_IN_RAND_LEN                      18
#define LMP_MAX_POWER_LEN                    2
#define LMP_MAX_SLOT_LEN                     3
#define LMP_MAX_SLOT_REQ_LEN                 3
#define LMP_MIN_POWER_LEN                    2
#define LMP_MODIFY_BEACON_LEN                14
#define LMP_NAME_REQ_LEN                     3
#define LMP_NAME_RES_LEN                     18
#define LMP_NOT_ACCEPTED_LEN                 4
#define LMP_PAGE_MODE_REQ_LEN                4
#define LMP_PAGE_SCAN_MODE_REQ_LEN           4
#define LMP_PARK_LEN                         18
#define LMP_PARK_REQ_LEN                     18
#define LMP_UNPARK_REQ_LEN                   18
#define LMP_PREFERRED_RATE_LEN               3
#define LMP_QoS_LEN                          5
#define LMP_QoS_REQ_LEN                      5
#define LMP_REMOVE_SCO_LINK_REQ_LEN          4
#define LMP_SCO_LINK_REQ_LEN                 8
#define LMP_SET_BROADCAST_SCAN_WINDOW_LEN    7
#define LMP_SETUP_COMPLETE_LEN               2
#define LMP_SLOT_OFFSET_LEN                  10
#define LMP_SNIFF_REQ_LEN                    11
#define LMP_SRES_LEN                         6
#define LMP_START_ENCRYPTION_REQ_LEN         18
#define LMP_STOP_ENCRYPTION_REQ_LEN          2
#define LMP_SUPERVISION_TIMEOUT_LEN          4
#define LMP_SWITCH_REQ_LEN                   6
#define LMP_TEMP_RAND_LEN                    18
#define LMP_TEMP_KEY_LEN                     18
#define LMP_TIMING_ACCURACY_REQ_LEN          2
#define LMP_TIMING_ACCURACY_RES_LEN          4
#define LMP_UNIT_KEY_LEN                     18
#define LMP_UNPARK_BD_ADDR_REQ_LEN           18
#define LMP_UNPARK_PM_ADDR_REQ_LEN           18
#define LMP_UNSNIFF_REQ_LEN                  2
#define LMP_USE_SEMI_PERMANENT_KEY_LEN       2
#define LMP_VERSION_REQ_LEN                  7
#define LMP_VERSION_RES_LEN                  7
#define LMP_PDU_MAX_LEN                      18
#define LMP_TEST_CONTROL_LEN                 11
#define LMP_TEST_ACTIVATE_LEN                2
#define LMP_ENCRYPTION_KEY_SIZE_MASK_REQ_LEN    2
#define LMP_ENCRYPTION_KEY_SIZE_MASK_RES_LEN    4
#define LMP_SET_AFH_LEN                            17
#define LMP_ENCAPSULATED_HEADER_LEN                 5
#define LMP_ENCAPSULATED_PAYLOAD_LEN                18
#define LMP_SIMPLE_PAIRING_CONFIRM_LEN              18
#define LMP_SIMPLE_PAIRING_NUMBER_LEN               18
#define LMP_DHKEY_CHECK_LEN                         18
#define LMP_PAUSE_ENCRYPTION_AES_REQ_LEN            18

/* Escape 1 opcodes */

/* Escape 2 opcodes */

/* Escape 3 opcodes */

/* Escape 4 opcodes */
#define LMP_ACCEPTED_EXT_LEN                                 5
#define LMP_NOT_ACCEPTED_EXT_LEN                             6
#define LMP_FEATURES_REQ_EXT_LEN                            13
#define LMP_FEATURES_RES_EXT_LEN                            13
#define LMP_PTT_REQ_LEN                                     4
#define LMP_ESCO_LINK_REQ_LEN                               17
#define LMP_REMOVE_ESCO_LINK_REQ_LEN                         5
#define LMP_CHANNEL_CLASSIFICATION_REQ_LEN                   8
#define LMP_CHANNEL_CLASSIFICATION_LEN                      13
#define LMP_SNIFF_SUBRATING_REQ_LEN                 10
#define LMP_SNIFF_SUBRATING_RES_LEN                 10
#define LMP_PAUSE_ENCRYPTION_REQ_LEN                3
#define LMP_RESUME_ENCRYPTION_REQ_LEN               3
#define LMP_IO_CAPABILITY_REQ_LEN                   6
#define LMP_IO_CAPABILITY_RES_LEN                   6
#define LMP_NUMERIC_COMPARISON_FAILED_LEN           3 
#define LMP_PASSKEY_FAILED_LEN                      3
#define LMP_OOB_FAILED_LEN                          3
#define LMP_KEYPRESS_NOTIFICATION_LEN               4

#ifdef VER_3_0
#define LMP_POWER_CTRL_REQ_LEN                      4
#define LMP_POWER_CTRL_RES_LEN                      4
#endif

#ifdef _SUPPORT_SECURE_CONNECTION_
#define LMP_PING_REQ_LEN                            3
#define LMP_PING_RES_LEN                            3
#endif

#ifdef _SUPPORT_PCA_ADJUST
#define LMP_CLK_ADJ_LEN                             16
#define LMP_CLK_ADJ_ACK_LEN                         4
#define LMP_CLK_ADJ_REQ_LEN                         7
#endif

/* 1.2 Feature Masks */
/* Byte 0 */
#define SUPPORT_3_SLOT_PACKETS          0x01    //1.1
#define SUPPORT_5_SLOT_PACKETS          0x02    //1.1
#define SUPPORT_ENCRYPTION              0x04    //1.1
#define SUPPORT_SLOT_OFFSET             0x08    //1.1
#define SUPPORT_TIMING_ACCURACY         0x10    //1.1
#define SUPPORT_ROLE_SWITCH             0x20    //1.1
#define SUPPORT_HOLD_MODE               0x40    //1.1
#define SUPPORT_SNIFF_MODE              0x80    //1.1

/* Byte 1 */
#define SUPPORT_PARK_STATE              0x01    //1.1
#define SUPPORT_POWER_CONTROL_REQUESTS  0x02    //1.1
#define SUPPORT_CQDDR                   0x04    //1.1
#define SUPPORT_SCO_LINK                0x08    //1.1
#define SUPPORT_HV2_PACKETS             0x10    //1.1
#define SUPPORT_HV3_PACKETS             0x20    //1.1
#define U_LAW_LOG_SYNCHRONOUS_DATA      0x40    //1.1
#define A_LAW_LOG_SYNCHRONOUS_DATA      0x80    //1.1

/* Byte 2 */
#define CVSD_SYNCHRONOUS_DATA           0x01    //1.1
#define PAGING_PARAMETER_NEGOTIATION    0x02    //1.1
#define POWER_CONTROL_FEATURE           0x04    //1.1
#define TRANSPARENT_SYNCHRONOUS_DATA    0x08    //1.1
#define FLOW_CONTROL_LAG_LSB            0x10    //1.1
#define FLOW_CONTROL_LAG_MB             0x20    //1.1
#define FLOW_CONTROL_LAG_MSB            0x40    //1.1
#define BROADCAST_ENCRYPTION            0x80    //1.1

/* Byte 3 */
#define SCATTER_MODE                0x01    //1.1
#define EDR_ACL_2MBPS               0x02    //2.0
#define EDR_ACL_3MBPS               0x04    //2.0
#define ENHANCED_INQ_SCAN           0x08    //1.2
#define INTERLACED_INQ_SCAN         0x10    //1.2
#define INTERLACED_PAGE_SCAN        0x20    //1.2
#define RSSI_WITH_INQ_RESULT        0x40    //1.2
#define ESCO_EV3                    0x80    //1.2

/* Byte 4 */
#define ESCO_EV4                    0x01    //1.2
#define ESCO_EV5                    0x02    //1.2
#define AFH_CAPABLE_SLAVE           0x08    //1.2
#define AFH_CLASSIFICATION_SLAVE    0x10    //1.2
#define LE_CONTROLER_SUPPORT        0x20    //4.0
#define BR_EDR_NOT_SUPPORT          0x40    //4.0
#define EDR_ACL_3_SLOT              0x80    //2.0

/* Byte 5 */
#define EDR_ACL_5_SLOT              0x01    //2.0
#define SNIFF_SUBRATING             0x02    //2.1
#define PAUSE_ENCRYPTION            0x04    //2.1
#define AFH_CAPABLE_MASTER          0x08    //1.2
#define AFH_CLASSIFICATION_MASTER   0x10    //1.2
#define EDR_ESCO_2MBPS              0x20    //2.0
#define EDR_ESCO_3MBPS              0x40    //2.0
#define EDR_ESCO_3_SLOT             0x80    //2.0

/* Byte 6 */
#define EXTENDED_INQUIRY_RESPONSE   0x01    //2.1
#define DUAL_MODE_CONTROLER_SUPPORT 0x02    //4.0
#define SECURE_SIMPLE_PAIRING       0x08    //2.1
#define ENCAPSULATED_PDU            0x10    //2.1
#define ERRONEOUS_DATA_REPORTING    0x20    //2.1
#define NON_FLUSHABLE_PKT_BF        0x40    //2.1
#define PERSISTENT_SNIFF            0x80    //2.1

/* Byte 7 */
#define LINK_SUPERVISION_TO_EVENT           0x01    //2.1
#define INQUIRY_TX_POWER_LEVEL              0x02    //2.1
#define ENHANCED_POWER_CONTROL              0x04    //3.0
#define OPTIONAL_PAGING_SCHEME2_BASELINE    0x20    //4.1
#define OPTIONAL_PAGING_SCHEME2_FAST_DATA   0x40    //4.1
#define EXTENDED_FEATURE                    0x80    //1.2

/* Extentended Features Page 1 */
/* Byte 0 */
#define SECURE_SIMPLE_PAIRING_HOST_SUPPORT  0x01    //2.1
#define LE_HOST_SUPPORT                     0x02    //4.0
#define DUAL_MODE_HOST_SUPPORT              0x04    //4.0
#define SECURE_CONNECTION_HOST_SUPPORT      0x08    //4.1

/* Extentended Features Page 2 */
/* Byte 0 */
#define CSB_MASTER                              0x01    //csa4
#define CSB_SLAVE                               0x02    //csa4
#define SYNCHRONIZATION_TRAIN_MASTER            0x04    //csa4
#define SYNCHRONIZATION_TRAIN_SCAN_SLAVE        0x08    //csa4
#define INQ_NOTIFICATION_EVT                    0x10    //budapest
#define GENERALIEZD_INTERLACED_SCAN             0x20    //budapest
#define COARSE_CLOCK_ADJUSTMENT                 0x40    //budapest
#define TIMING_DRAIFT                           0x80    //budapest

/* Byte 1 */
#define SECURE_CONNECTION_CONTROLLER_SUPPORT    0x01    //budapest
#define SUPPORT_PING                            0x02    //budapest
#define SLOT_AVAILABILITY_MASK                  0x04    //budapest
#define TRAIN_NUDGING                           0x08    //?


#define LMP_BD_ADDR_SIZE                    6

#define MASTER                              0
#define SLAVE                               1

#endif /* __LMP_SPEC_DEFINES_H__ */

/** @} end: lmp_external */
