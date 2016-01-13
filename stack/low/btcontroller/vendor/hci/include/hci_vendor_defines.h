/***************************************************************************
 Copyright (C) Mindtree Ltd.
 This module is a confidential and proprietary property of MindTree and
 a possession or use of this module requires written permission of MindTree.
 ***************************************************************************/

/**
 * @file
 * HCI vendor command/event definition.
 */

/**
 * @addtogroup HCIVendor
 * @{
 */
#ifndef __HCI_VENDOR_DEFINES_H__
 #define __HCI_VENDOR_DEFINES_H__

#include "bt_fw_types.h"
#include "bt_fw_common.h"
#include "lmp_1_2_defines.h"

#define LMP_VENDOR_IDLE    0x00
#define LMP_VENDOR_READY   0x01
#define LMP_VENDOR_SENT    0x02

/* Vendor-specific HCI commands - OGF: 0x3F */
#ifdef _CCH_RTL8723A_B_CUT
// _CCH_ISSC_VENDOR_
#define HCI_VENDOR_SET_BD_ADDR                               0xFC0D
#endif

#define HCI_VENDOR_UART_MODE_SUSPEND                         0xFC10
#define HCI_VENDOR_EXECUTE_LPS                               0xFC11
#ifdef _MODIFY_OLD_LOOPBACK_COMMAND_
#define HCI_VENDOR_HCI_LOOPBACK_MODE                         0xFC12
#endif
#define HCI_VENDOR_PROFILE_REPORT                            0xFC13
#define HCI_VENDOR_WIFI_CL_EN                                0xFC14
#define HCI_VENDOR_GPIO_PKT_TYPE_DBG                         0xFC15
#define HCI_VENDOR_HOST_ENTER_SLEEP_MODE                     0xFC16
#define HCI_VENDOR_SET_BAUDRATE                              0xFC17
#define HCI_VENDOR_ENABLE_PROFILE_RPT                        0xFC18
#define HCI_VENDOR_SET_PROFILE_RPT                           0xFC19

/*	Added by Wallice for USB LPM HCI Vendor command.	2012/03/19	*/
#define HCI_VENDOR_USB_LPM_CTRL                             0xFC1A
/*	eND Added by Wallice for USB LPM HCI Vendor command.	2012/03/19	*/

#define HCI_VENDOR_DOWNLOAD                                  0xFC20
#define HCI_VENDOR_SET_ASSOC                                 0xFC21
#define HCI_VENDOR_GET_ASSOC                                 0xFC22
#define HCI_VENDOR_SPI_INIT                                  0xFC23
#define HCI_VENDOR_SPI_ERASE                                 0xFC24
#define HCI_VENDOR_SPI_WRITE                                 0xFC25
#define HCI_VENDOR_SPI_READ                                  0xFC26
#define HCI_VENDOR_SET_LOG_ENABLE                            0xFC27
#define HCI_VENDOR_BT_DEV_MGR_ENABLE                         0xFC28

#ifdef _YL_RTL8723A_B_CUT
#define HCI_VENDOR_SET_HOST_WAKE_BT                          0xFC30
#define HCI_VENDOR_CLR_HOST_WAKE_BT                          0xFC31
#endif

#ifdef _DAPE_TEST_AUTO_CONN
#define HCI_VENDOR_CLEAR_WHITE_LIST                          0xFC30
#define HCI_VENDOR_ADD_DEVICE_TO_WHITE_LIST                  0xFC31
#define HCI_VENDOR_REMOVE_DEVICE_FROM_WHITE_LIST             0xFC32
#define LEGACY_MAX_WHITE_LIST_SIZE                           6
#endif

#define HCI_VENDOR_SET_LE_RSSI_THRESHOLD                     0xFC35
#define HCI_VENDOR_SET_BT_LED_MODE                           0xFC37

#define HCI_VENDOR_8703B_BTGPIO_LOG_ENABLE                   0xFC39

#define HCI_VENDOR_SET_MUTE_ENABLE                           0xFC50


#ifdef _UART_BAUD_ESTIMATE_
#define HCI_VENDOR_UART_SYNC                                 0xFC55
#endif
#ifdef _8821A_BTON_DESIGN_
#define HCI_VENDOR_UART_PARA_CHANGE                          0xFC56
#endif
#ifdef _YL_H5_ISSC_WARM_RST
#define HCI_VENDOR_ISSC_WARM_RESET                           0xFC5F
#endif

#ifdef _CCH_RTL8723A_B_CUT
// _CCH_ISSC_VENDOR_
#define HCI_VENDOR_ISSC_CHIP_VERI                            0xFC60
#endif

#define HCI_VENDOR_READ                                      0xFC61
#define HCI_VENDOR_WRITE                                     0xFC62
#define HCI_VENDOR_WRITE_BB_REGISTER                         0xFC64
#define HCI_VENDOR_READ_BB_REGISTER                          0xFC65

#define HCI_VENDOR_FORCE_RESET_AND_ALLOW_PATCH               0xFC66
#define HCI_VENDOR_FORCE_RESET                               0xFC67
#define HCI_VENDOR_SET_THERMAL_DEFAULT                       0xFC68
#define HCI_VENDOR_SET_THERMAL_TIMER_INTERVAL                0xFC69
#define HCI_VENDOR_SET_CFO_TRACK_ENABLE                      0xFC6A

#ifdef _BT_ONLY_
#define HCI_VENDOR_WRITE_EFUSE_DATA                          0xFC6B
#define HCI_VENDOR_READ_EFUSE_DATA                           0xFC6C
#endif

#define HCI_VENDOR_READ_RTK_ROM_VERSION                      0xFC6D
#define HCI_VENDOR_LPS_SETTING                               0xFC6E
#define HCI_VENDOR_READ_RTK_RTK_CHIP_ID                      0xFC6F
#define HCI_VENDOR_RF_HOPPING_TX_ENABLE_FOR_ADB              0xFC70

#define HCI_VENDOR_ENTER_WHCK                                0xFC71
#define HCI_VENDOR_LEAVE_WHCK                                0xFC72


#define HCI_VENDOR_TV_POWER_ON_OFF                           0xFC79
#define HCI_VENDOR_TV_SET_LE_SCAN_PARAM                      0xFC7A

#define HCI_VENDOR_SET_THERMAL_TRACK_PAUSE                   0xFC80
#define HCI_VENDOR_READ_LE_RX_TEST_PKT                       0xFC81

#define HCI_VENDOR_SEND_3DH5                                 0xFC82

#ifdef _BT4_2_PRIVACY_FEATURE_SUPPORT_
#define HCI_VENDOR_PRIVACY_ENABLE_OPTION                     0xFC83
enum {
    HCI_SUB_VENDOR_PRIVACY_ENABLE_LOG,
    HCI_SUB_VENDOR_PRIVACY_ENABLE_DBG,
    HCI_SUB_VENDOR_PRIVACY_ENABLE_COMPARE_RPA,
    HCI_SUB_VENDOR_PRIVACY_ENABLE_FORCE_BT40_OPT,
    HCI_SUB_VENDOR_PRIVACY_MAX,
};
#endif

#ifdef BT2FM_INDIRECT_ACCESS
/** Opcode of HCI vendor command to switch on/off FM.
 * @ref HCIVendorFMSwitch */
#define HCI_VENDOR_FM_SWITCH                                 0xFC85
/** Opcode of HCI vendor command to read data from FM.
 * @ref HCIVendorFMRead */
#define HCI_VENDOR_FM_READ                                   0xFC86
/** Opcode of HCI vendor command to write data to FM.
 * @ref HCIVendorFMWrite */
#define HCI_VENDOR_FM_WRITE                                  0xFC87
/** Opcode of HCI vendor command to set value according to FM address.
 * @ref HCIVendorFMSet */
#define HCI_VENDOR_FM_SET                                    0xFC88

/** Maximal length in bytes that can be read into command complete event. */
#define FM_READ_MAX_LEN (HCI_MAX_EVENT_PARAM_LEN - 4)

#ifdef FM_DEBUG
#define HCI_VENDOR_FM_TEST_STATUS                            0xFC89
#endif
#endif /* BT2FM_INDIRECT_ACCESS */
#define HCI_VENDOR_CTRL_BTON_GPIO                            0xFC8A
#define HCI_VENDOR_READ_SIE                                  0xFC8B

#define HCI_VENDOR_SEND_TEST_DATA                            0xFC90

/********************************************************************/
/*  HCI Vendor command Opcode for setting PCM/I2S register          */
/********************************************************************/
#define HCI_VENDOR_SET_PCMI2S_PARAM                          0xFC93

//(baron) test for MWS coex
#ifdef  MWS_ENABLE
#define HCI_VENDOR_MWS_INDIRECT_READ                         0xFCA1
#define HCI_VENDOR_MWS_INDIRECT_WRITE                        0xFCA2
#endif

#ifdef _CCH_TEST_DISABLE_EIR_BY_VENDOR_CMD
#define HCI_VENDOR_DISABLE_EIR                               0xFCC0
#endif
#ifdef _CCH_VENDOR_CMD_ESCO_DAYCOUNTER
#define HCI_VENDOR_WRITE_ESCO_DAYCOUNTER                     0xFCC1
#define HCI_VENDOR_READ_ESCO_DAYCOUNTER                      0xFCC2
#endif

#ifdef _DAPE_TEST_GEN_FAKE_ESCO_DATA
#define HCI_VENDOR_GEN_ESCO_DATA                             0xFCC3
#endif
#ifdef _DAPE_TEST_CHG_NATIVE_CLK_FOR_ESCO_DAY_COUNTER
#define HCI_VENDOR_CHG_NATIVE_CLOCK                          0xFCC4
#endif
#ifdef _DAPE_TEST_DISABLE_AFH_POWER_CTRL_FOR_PING_TEST_BY_VENDOR_CMD
#define HCI_VENDOR_DISALE_AFH_N_POWER_CTRL                   0xFCC5
#endif
#ifdef _CSB_RX_DBG_LOG
#define HCI_VENDOR_ENABLE_CSB_RX_LOG                         0xFCC6
#endif
#ifdef _CSB_RX_SET_XTOL_BY_VENDOR_CMD
#define HCI_VENDOR_SET_CSB_RX_XTOL                           0xFCC7
#endif
#ifdef _CSB_RX_SET_PTT_BY_VENDOR_CMD
#define HCI_VENDOR_SET_CSB_RX_PTT                            0xFCC8
#endif
#ifdef _DAPE_TEST_DISABLE_EDR_BY_VENDOR_CMD_FOR_BR_PKT
#define HCI_VENDOR_SET_PTT                                   0xFCC9
#endif
#ifdef _DAPE_TEST_CSB_RX_SELF_CALC_CLK_OFST_BY_VENDOR_CMD
#define HCI_VENDOR_CSB_RX_SELF_CALC_CLK_OFST                 0xFCCA
#endif
#ifdef _DAPE_TEST_DISABLE_DM1_FOR_CCPT_BY_VENDOR_CMD
#define HCI_VENDOR_DISABLE_DM1                               0xFCCB
#endif
#ifdef _SECURE_CONN_TEST_CHK_ACLU_DATA_BY_VENDOR_CMD
#define HCI_VENDOR_CHK_SECURE_CONN_DATA                      0xFCCC
#endif
#ifdef _SECURE_CONN_REFRESH_KEY_BY_VENDOR_CMD
#define HCI_VENDOR_REFRESH_KEY                               0xFCCD
#endif
#ifdef _SECURE_CONN_REFRESH_KEY_WHEN_CONTINUOUS_MIC
#define HCI_VENDOR_REFRESH_KEY_WHEN_CONTINUOUS_MIC_ERR       0xFCCE
#endif
#ifdef _DAPE_TEST_SEND_ZERO_LEN_PKT_BY_VENDOR_CMD
#define HCI_VENDOR_SEND_ZERO_LEN_PKT                         0xFCCF
#endif
#ifdef _DAPE_TEST_SEND_PTT_REQ_BY_VENDOR_CMD
#define HCI_VENDOR_SEND_PTT_REQ                              0xFCD0
#endif
#ifdef _DAPE_TEST_SEND_PING_RES_BY_VENDOR_CMD
#define HCI_VENDOR_RESPOND_TO_PING_REQ                       0xFCD1
#endif
#ifdef _DAPE_TEST_KILL_LE_CE_BY_VENDOR_CMD
#define HCI_VENDOR_KILL_LE_CE                                0xFCD2
#endif
#ifdef _DAPE_CHANGE_ESCO_RETRY_PRIORITY_BY_VENDOR_CMD
#define HCI_VENDOR_CHANGE_ESCO_RETRY_PRI_CMD                 0xFCD3
#endif
#ifdef _DAPE_TEST_ALWAYZ_NAK_ESCO_BY_VENDOR_CMD
#define HCI_VENDOR_ALWAYS_NAK_ESCO                           0xFCD4
#endif
#ifdef _DAPE_SET_PCA_CLK_US_BY_VENDOR_CMD
#define HCI_VENDOR_SET_PCA_PARAM                             0xFCD5
#endif
#ifdef _DAPE_TEST_CHK_MWS_FRAME_SYNC
#define HCI_VENDOR_GEN_FAKE_FRAME_SYNC                       0xFCD6
#define HCI_VENDOR_READ_FRAME_SYNC                           0xFCD7
#define HCI_VENDOR_WRITE_FRAME_SYNC                          0xFCD8
#endif
#ifdef _DAPE_SEND_PCA_ADJ_BY_VENDOR_CMD
#define HCI_VENDOR_SEND_CLK_ADJ                              0xFCD9
#define HCI_VENDOR_SEND_CLK_ADJ_REQ                          0xFCDA
#endif
#ifdef _DAPE_TEST_FRAME_SYNC_UPDATE_CHOOSE_WAY
#define HCI_VENDOR_CHOOSE_MODIFY_WAY                         0xFCDB
#endif
#ifdef _DAPE_SUPPORT_LE_TX_PKT_MODE
#define HCI_VENDOR_ENABLE_LE_TX_PKT_MODE                     0xFCDC
#endif
#if defined (_TEST_PI_WRITE_PAGE_3_BY_VENDOR_CMD) || defined (_TEST_ADAPTIVITY_FUNC_2)
#define HCI_VENDOR_ENABLE_WRITE_PI_PAGE_3                    0xFCDD  //// test only.
#endif

#ifdef _SUPPORT_MSFT_BT_HCI_EXTENSION_
#define HCI_VENDOR_MSFT_EXTENSION                            0xFCF0

#define HCI_SUB_VENDOR_MSFT_READ_SUPPORTED_FEATURES              0
#define HCI_SUB_VENDOR_MSFT_MONITOR_RSSI                         1
#define HCI_SUB_VENDOR_MSFT_CANCEL_MONITOR_RSSI                  2
#define HCI_SUB_VENDOR_MSFT_LE_MONITOR_ADVERTISEMENT             3
#define HCI_SUB_VENDOR_MSFT_CANCEL_MONITOR_ADVERTISEMENT         4
#define HCI_SUB_VENDOR_MSFT_LE_SET_ADVERTISEMENT_FILTER_ENABLE   5
#define HCI_SUB_VENDOR_MSFT_READ_ABSOLUTE_RSSI                   6
#define HCI_SUB_VENDOR_MSFT_MAX_OPCODE                           HCI_SUB_VENDOR_MSFT_READ_ABSOLUTE_RSSI
#endif

#define HCI_VENDOR_SPECIFIC_EVENT                            0xFF


/* This command will decide weather the Esco link has to be established
   over codec.
*/

/************************* Trasmit Tests **************************/
#define HCI_VENDOR_WRITE_RF_TRANSMIT_TEST_CONFIG             0xFD40
#define HCI_VENDOR_READ_RF_TRANSMIT_TEST_CONFIG              0xFD41
#define HCI_VENDOR_START_RF_TRANSMIT_TEST                    0xFD42
#define HCI_VENDOR_STOP_RF_TRANSMIT_TEST                     0xFD43
#define HCI_VENDOR_RF_RADIO_INIT                             0xFD47
#define HCI_VENDOR_RF_RADIO_REG_READ                         0xFD49
#define HCI_VENDOR_RF_RADIO_REG_WRITE                        0xFD4A
#ifdef _NEW_MODEM_PI_ACCESS_
#define HCI_VENDOR_RF_RADIO_REG_READ_PI                      0xFD4B
#define HCI_VENDOR_RF_RADIO_REG_WRITE_PI                     0xFD4C
#endif
#ifdef _MODEM_LNA_CONSTRAINT_
#define HCI_VENDOR_FORCE_MODEM_LNA_CONST_OFF                 0xFD4D
#endif
#if 0
#ifdef _MODEM_HCI_VENDOR_8821A_LOK_
#define HCI_VENDOR_EXECUTE_8821A_LOK                         0xFD4E
#endif
#endif

/* For MP: 0xFD4D (read), x0xFD4E (write) */

#ifdef UPF_TESTER_CODE
#define HCI_VENDOR_PAIRING_REJ_AFT_RS_REJ                 0xFD50
#endif

#ifdef UPF_TESTER_CODE

#define HCI_VENDOR_PAIRING_REJ_AFT_RS_REJ                          0xFD50

#endif /* UPF_TESTER_CODE */

/*
 * Return values for Vendor-specific HCI commands.
 */
#define HCI_TEST_CONTROL_EVENT                        0xFF
/********************************* Events *************************/

#define HCI_VENDOR_SPECIFIC_EVENT                                  0xFF
#define HCI_VENDOR_POWER_CONTROL_EVENT                             0xFE
#define HCI_VENDOR_2_1_TEST_EVENTS                                 0xFD
#define VENDOR_LOG_PACKET_TYPE_EVENT                                0
#define VENDOR_LOG_PACKET_TYPE_ACL                                  1
#define VENDOR_LOG_PACKET_TYPE_INVALID                              0xFF

/** TV power on key press vendor event code. */
#define HCI_VENDOR_TV_POWERON_KEY_SUBEVENT     0x2F

/**
 * @struct HCI_VENDOR_TV_POWERON_KEY_EVT_PARAM_
 * @brief TV power on key press event.
 */
typedef struct HCI_VENDOR_TV_POWERON_KEY_EVT_PARAM_
{
    /**
     * @brief Subevent code.
     *
     * Should set to HCI_VENDOR_TV_POWERON_KEY_SUBEVENT.
     */
    UINT8 subevt_code;
    UINT8 index; /**< Key index. */
    UINT8 type; /**< Key type. (0: short press; 1: long press) */
} HCI_VENDOR_TV_POWERON_KEY_EVT_PARAM;

typedef struct _PACKED_ HCI_VENDOR_WATCHDOG_REBOOT_EVT_PARAM_
{
    UINT8 subevt_code;
    UINT8 reserved;
    UINT8 in_isr;
    UINT8 isr_num;
    UINT8 in_cpu_idle;
    UINT16 signal_id;
    UINT32 isr_cause;
    UINT32 isr_cnts_during_one_callback;
    UINT32 last_epc;
    UINT32 os_timer_handle;
    UINT8 os_timer_cal_table_index;
    UINT8 os_timer_timer_count;
    UINT32 os_timer_timer_value;
    UINT32 os_timer_timeout_function;
    UINT8 os_timer_timer_type;
    UINT32 os_timer_args;
    UINT32 os_timer_next;
    UINT8 os_timer_state;
    UINT32 os_timer_sniff_tick_timer;
} HCI_VENDOR_WATCHDOG_REBOOT_EVT_PARAM;
#define HCI_VENDOR_WATCHDOG_REBOOT_SUBEVENT     0x34

/*
 * Return values for Vendor-specific HCI commands.
 */
#define HCI_TEST_CONTROL_EVENT                        0xFF

/* ESCO packet types in Test_control pdu */
#define   TCM_BB_EV3                0x17
#define   TCM_BB_EV4                0x1c
#define   TCM_BB_EV5                0x1d

#define MAX_SCO_CONN_ENTITIES                  LMP_MAX_SCO_CONNECTIONS

/* Maximum number of synchronous connection entities */
#ifdef SCO_OVER_HCI
#define     MAX_SYNC_CONN_ENTITIES      ((LMP_MAX_ESCO_CONN_ENTITIES) + \
                                        (MAX_SCO_CONN_ENTITIES))
#else
#define     MAX_SYNC_CONN_ENTITIES      (LMP_MAX_ESCO_CONN_ENTITIES)
#endif /*  SCO_OVER_HCI */


#define     MAX_VENDOR_VALID_PKT_TYPE   15



#define NORMAL_ACCEPT_CONN      0x00
#define MAX_TEST_SCENARIO_VALUE 0x01
#define LCH_START               0x02
#define LCH_CONT                0x01


/*
   This structure is used for storing the vendor specific(MindTree) commands
   HCI_VEDNOR_BLOCK_HOST_DATA
*/

typedef struct
{
    UINT16 connection_handle;
    UINT16 no_of_trials;
    UINT16 rx_connection_handle;
    UINT16 num_pkts_sent;
    UCHAR status;
    UCHAR pkt_size;
    UCHAR data_pattern;
    UCHAR enable_data_to_host;
    UCHAR start_of_data_pattern;
}SEND_DATA_INFO;

/*
 * These #defines are used for parameter checking
 */
#define TC_MAX_ERROR_TYPE                       0x2000

#define VENDOR_TX_TEST_SINGLE_FREQ                 0x0

#define VENDOR_TX_TEST_2_DH1                      0x24
#define VENDOR_TX_TEST_3_DH1                      0x28
#define VENDOR_TX_TEST_2_DH3                      0x2A
#define VENDOR_TX_TEST_3_DH3                      0x2B
#define VENDOR_TX_TEST_2_DH5                      0x2E
#define VENDOR_TX_TEST_3_DH5                      0x2F

/*
 * Set of baseband register values corresponding to the hops
 */
#define     BASEBAND_HOP_79_SLAVE       0x20

#define     BASEBAND_HOP_1      0x60
#define     BASEBAND_HOP_5      0x40

#ifdef RT_VENDOR_CMDS
extern UINT8 dbg_vendor_log_interface;
extern UINT16 dbg_vendor_log_conn_handle;
extern UINT8 *dbg_vendor_log_buf;
extern UINT8 *dbg_vendor_set_log_complete_event_buf;
#endif

#ifdef _8821A_BTON_DESIGN_
extern HCI_CMD_PKT *g_hci_cmd_ptr_for_evt_gen;
#endif

/*
 * This macro will isolate only the two bits used for hop frequency.
 */
BOOLEAN hci_vendor_cmd_generate_event(UINT16 cmd_opcode, UINT8 temp_status, void *arg);
void hci_set_vendor_rf_transmit_test_config(UCHAR *params);
void hci_start_rf_transmit_test(void);
void hci_stop_rf_transmit_test(void);
UCHAR hci_vendor_generate_log_data_packet(void);
UINT8 hci_vendor_check_log_data_packet(UINT8 type, UINT8 *buf, UINT8 free);

#ifdef _VENDOR_RESET_BB_
extern UINT8 lc_baseband_has_reset;
void hci_vendor_reset_bb(void);
#endif

UCHAR hci_handle_vendor_set_PCMI2SParam_cmd(HCI_CMD_PKT *hci_cmd_ptr);

#endif /* __HCI_VENDOR_DEFINES_H__ */
/**@}*/
