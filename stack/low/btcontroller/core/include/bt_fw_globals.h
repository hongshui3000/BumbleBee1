/***************************************************************************
 Copyright (C) Mindtree Ltd.
 This module is a confidential and proprietary property of MindTree and
 a possession or use of this module requires written permission of MindTree.
 ***************************************************************************/

#ifndef __BT_FW_GLOBALS_H__
#define __BT_FW_GLOBALS_H__


#include "bt_fw_os.h"
#include "otp.h"

#define BD_ADDR_SIZE                     6
#define LMP_MAX_NAME_LENGTH         (248)
#define LMP_EVENT_MASK_SIZE              8

/* Define for page scan repetition */
#define SR_R0                   0
#define SR_R1                   1
#define SR_R2                   2

/* Define for page scan period  */
#define SP_P0                   0
#define SP_P1                   1
#define SP_P2                   2

/* Firmware Build Information FW_VERSION_NUMBER Format is
 * 0x(Major Number(1-Nibble))(Minor Number(1-Nibble))(Build Number(2-Nibbles))
 * Example For a tag in the repository BTC_FW_REL_00_01_BUILD_036
 * The value to be added will be 0x0136
 */
/* Current Version is BTC_FW_REL_01_02_BUILD_000 */
#define  FW_VERSION_NUMBER                        fw_lmp_sub_version

/*
 * Run time configurable parameteres
 */
#define BT_FW_HCI_REVISION                        fw_hci_sub_version

/* 
 *  HCI_VERSION numbers
 *  -----------------------------------------
 *    0     Bluetooth HCI Specification 1.0B  
 *    1     Bluetooth HCI Specification 1.1 
 *    2     Bluetooth HCI Specification 1.2 
 *    3     Bluetooth HCI Specification 2.0 + EDR
 *    4     Bluetooth HCI Specification 2.1 + EDR 
 *    5     Bluetooth HCI Specification 3.0 + HS
 *    6     Bluetooth HCI Specification 4.0
 *    7     Bluetooth HCI Specification 4.1
 *    8     Bluetooth HCI Specification 4.2
 *    9-255 Reserved
 *
 *  LMP_VERSION numbers
 *  -----------------------------------------
 *    0     Bluetooth LMP 1.0
 *    1     Bluetooth LMP 1.1  
 *    2     Bluetooth LMP 1.2  
 *    3     Bluetooth LMP 2.0 + EDR
 *    4     Bluetooth LMP 2.1 + EDR
 *    5     Bluetooth LMP 3.0 + HS
 *    6     Bluetooth HCI Specification 4.0
 *    7     Bluetooth HCI Specification 4.1
 *    8     Bluetooth HCI Specification 4.2
 *    9-255 Reserved
 *
 *  LL_VERSION numbers
 *  -----------------------------------------
 *    0-5   Reserved
 *    6     Bluetooth HCI Specification 4.0
 *    7     Bluetooth HCI Specification 4.1
 *    8     Bluetooth HCI Specification 4.2
 *    9-255 Reserved
 *
 */
enum BT_FW_HCI_VERSION_TABLE {
    BT_FW_HCI_VERSION_BT10,
    BT_FW_HCI_VERSION_BT11,
    BT_FW_HCI_VERSION_BT12,
    BT_FW_HCI_VERSION_BT20_PLUS_EDR,
    BT_FW_HCI_VERSION_BT21_PLUS_EDR,    
    BT_FW_HCI_VERSION_BT30,
    BT_FW_HCI_VERSION_BT40,
    BT_FW_HCI_VERSION_BT41,
    BT_FW_HCI_VERSION_BT42,
    BT_FW_HCI_VERSION_BT50    
};

enum BT_FW_LMP_VERSION_TABLE {
    BT_FW_LMP_VERSION_BT10,
    BT_FW_LMP_VERSION_BT11,
    BT_FW_LMP_VERSION_BT12,
    BT_FW_LMP_VERSION_BT20_PLUS_EDR,
    BT_FW_LMP_VERSION_BT21_PLUS_EDR,    
    BT_FW_LMP_VERSION_BT30,
    BT_FW_LMP_VERSION_BT40,       
    BT_FW_LMP_VERSION_BT41,
    BT_FW_LMP_VERSION_BT42,
    BT_FW_LMP_VERSION_BT50
};

enum BT_FW_LL_VERSION_TABLE {
    BT_FW_LL_VERSION_BT40 = 6,       
    BT_FW_LL_VERSION_BT41,
    BT_FW_LL_VERSION_BT42,
    BT_FW_LL_VERSION_BT50
};


#if defined(_SUPPORT_VER_5_0_)
#define BT_FW_LL_VERSION                        BT_FW_LL_VERSION_BT50
#elif defined(_SUPPORT_VER_4_2_)
#define BT_FW_LL_VERSION                        BT_FW_LL_VERSION_BT42
#elif defined(_SUPPORT_VER_4_1_)
#define BT_FW_LL_VERSION                        BT_FW_LL_VERSION_BT41 
#else
#define BT_FW_LL_VERSION                        BT_FW_LL_VERSION_BT40 
#endif

#if defined(_SUPPORT_VER_5_0_)
#define BT_FW_HCI_VERSION                       BT_FW_HCI_VERSION_BT50
#elif defined(_SUPPORT_VER_4_2_)
#define BT_FW_HCI_VERSION                       BT_FW_HCI_VERSION_BT42
#elif defined(_SUPPORT_VER_4_1_)
#define BT_FW_HCI_VERSION                       BT_FW_HCI_VERSION_BT41
#elif defined (LE_MODE_EN)
#define BT_FW_HCI_VERSION                       BT_FW_HCI_VERSION_BT40
#elif defined(VER_3_0)
#define BT_FW_HCI_VERSION                       BT_FW_HCI_VERSION_BT30
#else
#define BT_FW_HCI_VERSION                       BT_FW_HCI_VERSION_BT21_PLUS_EDR
#endif 


/* CompID - used for testing. (use Realtek in the future) */
#define BT_FW_MANUFACTURER_NAME                 otp_str_data.bt_manufacturer_name

#define BT_FW_LMP_SUBVERSION					FW_VERSION_NUMBER
#define BT_FW_LL_SUBVERSION                     FW_VERSION_NUMBER

#if defined(_SUPPORT_VER_5_0_)
#define BT_FW_LMP_VERSION                       BT_FW_LMP_VERSION_BT50
#elif defined(_SUPPORT_VER_4_2_)
#define BT_FW_LMP_VERSION                       BT_FW_LMP_VERSION_BT42
#elif defined(_SUPPORT_VER_4_1_)
#define BT_FW_LMP_VERSION                       BT_FW_LMP_VERSION_BT41
#elif defined(LE_MODE_EN)
#define BT_FW_LMP_VERSION                       BT_FW_LMP_VERSION_BT40
#elif defined(VER_3_0)
#define BT_FW_LMP_VERSION                       BT_FW_LMP_VERSION_BT30
#else
#define BT_FW_LMP_VERSION                       BT_FW_LMP_VERSION_BT21_PLUS_EDR
#endif

#define SUPPORT_LE_ENCRYPTION                   BIT0
#define SUPPORT_CONN_PARAM_REQ_PROCEDURE        BIT1
#define SUPPORT_REJ_IND_EXT                     BIT2
#define SUPPORT_SLV_INIT_FEATURE_REQ            BIT3
#define SUPPORT_LE_PING                         BIT4
#define SUPPORT_LE_DATA_PKT_LEN_EXT             BIT5
#define SUPPORT_LL_PRIVACY                      BIT6
#define SUPPORT_EXT_SCANNER_FILTER_POLICIES     BIT7
#define SUPPORT_2MBPS_PHY                       BIT8
#define SUPPORT_STABLE_MODULATION_IDX           BIT9


/*---------------------------------------------------------------------
	Chris add for USB_DMA 2010.03.23
-----------------------------------------------------------------------*/
#define ENTER_TEST_MODE                                1
#define LEAVE_TEST_MODE                                2
#define DUT_TEST_MODE_FIFO_FREE                        3
#define ENTER_SUSPEND_MODE                             4
#define LEAVE_SUSPEND_MODE                             5

#define BT_FW_TOTAL_ACL_PKTS                           8 
#define BT_FW_TOTAL_ACL_PKTS_TO_HOST                   8 
#define BT_FW_TOTAL_ACL_PKTS_FROM_HOST                 8 
#define BT_FW_ACL_RX_FLOW_ZERO_DIFF                    2

#ifndef USE_FREERTOS
/*
*  memory allocat form sram or dmem
*/
#define MEM_SRAM_MODE	            0
#define MEM_DMEM_MODE	            1
#define MEM_SRAM_PARTIAL_OFF_MODE   2
#endif

#if defined(COMPILE_ESCO) && defined(SCO_OVER_HCI)
#define BT_FW_TOTAL_SYNCHRONOUS_PKTS_FROM_HOST         12 //12 in 0360
#define BT_FW_TOTAL_SYNCHRONOUS_PKTS_TO_HOST           12 //12 in 0360
#elif defined(COMPILE_ESCO)
#define BT_FW_TOTAL_SYNCHRONOUS_PKTS_FROM_HOST         12 //6 in 0360
#define BT_FW_TOTAL_SYNCHRONOUS_PKTS_TO_HOST           12 //6 in 0360
#elif defined(SCO_OVER_HCI)
#define BT_FW_TOTAL_SYNCHRONOUS_PKTS_FROM_HOST         12 //6 in 0360
#define BT_FW_TOTAL_SYNCHRONOUS_PKTS_TO_HOST           12 //6 in 0360
#else
#define BT_FW_TOTAL_SYNCHRONOUS_PKTS_FROM_HOST         0
#define BT_FW_TOTAL_SYNCHRONOUS_PKTS_TO_HOST           0
#endif /* COMPILE_ESCO */

#define BT_FW_TOTAL_SYNCHRONOUS_TX_ENTRY    (BT_FW_TOTAL_SYNCHRONOUS_PKTS_FROM_HOST)

#define BT_FW_COUNTRY_CODE                             0

#define BT_FW_EVENT_MASK                       0xFFFFFFFF

/* This value is in seconds : default value according to BB spec.*/
#define BT_FW_SUPERVISION_TIMEOUT                  0x7D00

#define BT_FW_CONN_ACCEPT_TIMEOUT                  0x1f40

#define BT_FW_PAGE_TIMEOUT                         0x2000

/* Milliseconds */
#define BT_FW_NUM_COMPLETE_PACKETS_TIME              2000

#define BT_FW_MAX_EVENT_FILTERS                      0x16

#define BT_FW_CLASS_OF_DEVICE                  0x0040020c

#define BT_FW_T_POLL                                   36

#define BT_FW_CMD_BUFFERS                              4

#define BT_FW_EVENT_BUFFERS                             8

#define BT_FW_PDU_BUFFERS                              32

#define BT_FW_FHS_PKT_BUFFERS                           4 /* Make sure this is 2^n */

#define BT_FW_SCAN_MODE                                 0

#define BT_FW_PAGE_SCAN_MODE                            0

#define BT_FW_PAGE_SCAN_PERIOD_MODE                     0

#define BT_FW_PAGE_SCAN_REPETITION_MODE             SR_R1

#define BT_FW_PAGE_SCAN_INTERVAL                   0x0800

#define BT_FW_PAGE_SCAN_WINDOW                     0x0100

#define BT_FW_INQUIRY_SCAN_INTERVAL                0x1000

#define BT_FW_INQUIRY_SCAN_WINDOW                  0x0048

/* In Slots */
#define BT_FW_HOLD_MAX_INTERVAL                     65535

/* In Slots */
#define BT_FW_HOLD_MIN_INTERVAL                         1

#define BT_FW_SNIFF_MAX_INTERVAL                    65535

#define BT_FW_SNIFF_MIN_INTERVAL                        1

#define BT_FW_SNIFF_ATTEMPT                             1

#define BT_FW_SNIFF_TIMEOUT                             1

#define BT_FW_BEACON_MAX_INTERVAL                    0x2000

#define BT_FW_BEACON_MIN_INTERVAL                    0x200

#define BT_FW_DBEACON                                   0

#define BT_FW_TBEACON                                   0

#define BT_FW_NBEACON                                   0

#define BT_FW_PARK_BEACON_LSTO_MIN_DIFF              0x50


/*
 * Disable Link Policy settings by default.
 */
#define BT_FW_LINK_POLICY_SETTINGS                0x00000

/*
 * Structure to store configurble parameteres.
 */
#define HCI_STANDARD_INQ_RESULT_EVENT                   0
#define HCI_INQ_RESULT_EVENT_WITH_RSSI                  1
#define HCI_INQ_RESULT_EVENT_WITH_EIR                   2

#define HCI_STANDARD_INQ_SCAN                           0
#define HCI_INTERLACED_INQ_SCAN                         1

#define HCI_3DD_INQ_SCAN                                128

#define HCI_STANDARD_PAGE_SCAN                          0
#define HCI_INTERLACED_PAGE_SCAN                        1

/* Qos and flow specification related defines. */
#define HCI_INVALID_TPOLL                            0xFF

#define QOS_TN_INVALID                               0xFF
#define QOS_TN_FLOW_SPEC_TX_SIDE                     0x00
#define QOS_TN_FLOW_SPEC_RX_SIDE                     0x01
#define QOS_TN_QOS_SETUP                             0x02

#define FLOW_SPEC_RX_SIDE                            0x00
#define FLOW_SPEC_TX_SIDE                            0x01
#define QOS_SETUP_DUAL_SIDE                          0x02

#define LC_SCATTERNET_PRIORITY1_REG_VAL             0xF671
#define LC_SCATTERNET_PRIORITY2_REG_VAL             0x0F77

#define POWER_SAVE_FEATURE_DISABLE       0x00
#define POWER_SAVE_FEATURE_ENABLE        0x01
        
#endif /* __BT_FW_GLOBALS_H__ */

