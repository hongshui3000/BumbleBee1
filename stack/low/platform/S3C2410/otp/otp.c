/***************************************************************************
 Copyright (C) MindTree Ltd.
 This module is a confidential and proprietary property of MindTree and
 a possession or use of this module requires written permission of MindTree.
 ***************************************************************************/
enum { __FILE_NUM__= 107 };

#include "otp.h"
#include "lmp.h"
#ifdef _RTK8723_UART_INIT_
#include "dma_usb.h"
#endif
#include "power_control.h"
#include "h5.h"
#include "lc_internal.h"
#include "tv.h"
#ifdef HAS_FPGA_BD_ADDR
#include "fpga_bd_addr.h"
#endif

#if defined(_RTL8821A_SPECIFIC_)|| defined(_RTL8821A_MP_SPECIFIC_)
UINT16 fw_lmp_sub_version = 0x8821;
UINT16 fw_hci_sub_version = 0x000A;
#elif defined(_RLE0523_SPECIFIC_) || defined(_RTL8723B_SPECIFIC_)
UINT16 fw_lmp_sub_version = 0x8723;
UINT16 fw_hci_sub_version = 0x000B;
#elif defined(_RLE0557_SPECIFIC_) || defined(_RTL2801_SPECIFIC_)
UINT16 fw_lmp_sub_version = 0x2801;
UINT16 fw_hci_sub_version = 0x000A;
#elif defined(_RTL8821B_TC_SPECIFIC_)
UINT16 fw_lmp_sub_version = 0x8821;
UINT16 fw_hci_sub_version = 0x000B;
#elif defined(_RTL8703B_SPECIFIC_)
UINT16 fw_lmp_sub_version = 0x8703;
UINT16 fw_hci_sub_version = 0x000B;
#elif defined(_RTL8822B_SPECIFIC_)
UINT16 fw_lmp_sub_version = 0x8822;
UINT16 fw_hci_sub_version = 0x000B;
#elif defined(_RTL8723D_SPECIFIC_)
UINT16 fw_lmp_sub_version = 0x8723;
UINT16 fw_hci_sub_version = 0x000D;
#elif defined(_RTL8821C_SPECIFIC_)
UINT16 fw_lmp_sub_version = 0x8821;
UINT16 fw_hci_sub_version = 0x000C;
#else
UINT16 fw_lmp_sub_version = 0x1200;
UINT16 fw_hci_sub_version = 0x000B;
#endif

UINT8 g_global_slot_min_allow_sniff_int = 18;
UINT8 g_block_legacy_for_le_slot_num = 7;
UINT8 g_le_use_interval_slot_pair = 5;
#ifdef _CSB_RX_SET_XTOL_BY_EFUSE
UINT16 g_beacon_rx_xtol = 4;
#endif
#ifdef _SECURE_CONN_REFRESH_KEY_WHEN_CONTINUOUS_MIC
UINT8 g_sc_refresh_key_when_continuous_mic_err = 1;
#endif

/* Nish 20140804 [s] */
/* Nish 20140804 notes:
   To run mute on FPGA, please define _DAPE_TEST_FOR_MUTE,
   To run mute on chip, please search _DAPE_TEST_FOR_MUTE in otp.c to p efuse
*/
/* bt_priority_3 */
#ifndef _DAPE_TEST_NEW_HW
#define BT_PRIORITY_3 0x0000                    //bt_padding_2b
#else
#if defined(_DAPE_TEST_FOR_MUTE)
#define BT_PRIORITY_3 (BIT11|BIT12|BIT14)       //0x5800
#else
#if defined (_MONITOR_WDG_TIMEOUT_THROUGH_HCI_EVENT)
#define BT_PRIORITY_3 (BIT11|BIT12|BIT9)        //0x1A00
#else
#define BT_PRIORITY_3 (BIT11|BIT12)             //0x1800
#endif
#endif
#endif
/* Nish 20140804 [e] */

OTP_STRUCT otp_str_data =
{
    //**************for OTP managerment***************
    //*    size: 4 Bytes                             *
    //************************************************
    0xA5000100,     //EFUSE[00~03] Format: SyntaxByte_VersionByte_Reserve0Byte_Reserve1Byte
                    //SyntaxByte: [0xA5=use OTP value] [others = use default value]
                    //VersionByte: OTP version
                    //in Main(), this value will be checked first



    //**************for SYS managerment***************
    //*    size: 12 Bytes                             *
    //************************************************
    0x02625A00,       //EFUSE[04~07] UINT32 crystal_clk;//40000000
    0x0001C200,       //EFUSE[08~0B] UINT32 SYS_log_uart_baudrate;		//115200

#ifdef _RTK8723_UART_INIT_
    BAUD_FORMAT_D24_115200_NO_ADJ,  //EFUSE[0C~0F] UINT32 SYS_hci_uart_baudrate; FORMAT: refer to BAUD_FORMAT_TYPE
#else
    115200,                     //EFUSE[0C~0F] UINT32 921600Hz, UINT32 SYS_hci_uart_baudrate; Unit: Hz
#endif

    0x19EAD550,                 //EFUSE[10~13] UINT32 rtl8723_data_uart_settings (refer the structure "RLT8723_DATA_UART_SETTINGS_S") 01EAC550 is preferred ???
    0xAFF11BE0,                 //EFUSE[14~17] UINT32 rtl8723_data_uart_settings_2 (refer the structure "RLT8723_DATA_UART_SETTINGS_2_S")
    0x0BA40158,                 //EFUSE[18~1B] UINT32 rtl8723_data_uart_settings_3 (refer the structure "RLT8723_DATA_UART_SETTINGS_3_S")

    0x01FFE8B6,                 //EFUSE[1C~1F] UINT32 efuse_lps_setting_1_d32 (refer the structure "EFUSE_LPS_SETTING_1_S")
    0x88016038 ,                //EFUSE[20~23] UINT32 efuse_lps_setting_2_d32 (refer the structure "EFUSE_LPS_SETTING_2_S")
    0x73DF030C,                 //EFUSE[24~27]  UINT32 efuse_lps_setting_3_d32 (refer the structure "EFUSE_LPS_SETTING_3_S")
    0xD2,                       //EFUSE[28] UINT8 efuse_lps_setting_4_d8 (refer the structure "EFUSE_LPS_SETTING_4_S")
    0x69,                       //EFUSE[29] UINT8 efuse_lps_setting_5_d8 (refer the structure "EFUSE_LPS_SETTING_5_S")
    0x00,                       //EFUSE[2A] UINT8 efuse_lps_setting_6_d8 (refer the structure "EFUSE_LPS_SETTING_6_S")
    0xff,                       //EFUSE[2B]  UINT8 efuse_lps_setting_rsvd_d8;

#ifdef _DLPS_SIMU_TEST_
    5, //default 360                      //EFUSE[2C~2F] UINT32 lps_mode_max_time unit: 10msec
#else
    600, //default 360                   //EFUSE[2C~2F] UINT32 lps_mode_max_time unit: 10msec
#endif

    EFUSE_POWER_OPTION,         //EFUSE[30~31] UINT16 power_option[]  (refer to EFUSE_POW_OPTION_S_TYPE)
    EFUSE_POWER_PARAMETER,      //EFUSE[32~33] UINT16 power_seq_param;
    GENERAL_CONTROL_OPTION,     //EFUSE[34~35] UINT16 general_control
    100,                        //EFUSE[36] UINT8 default_max_flush_time
    30,                         //EFUSE[37] UINT8 mailbox_max_timeout unit:10ms
    1,                          //EFUSE[38] UINT8 gpio_wake_up_time uint: 10ms
    3,                          //EFUSE[39] UINT8 pta_pkt_unit
    CONTROL_TIME_PARAMETER,     //EFUSE[3A] UINT8 control_time_parameter
    3,                          //EFUSE[3B] UINT8 pre_detect_pkt_num
    3,                          //EFUSE[3C] UINT8 pdn_delay_tim byte0
    2,                          //EFUSE[3D] UINT8 pdn_delay_tim byte1
    0x40,                       //EFUSE[3E] UINT8 efuse_pow_setting_1_d8 (refer the structure "EFUSE_POW_SETTING_1_S")
    0x80,                       //EFUSE[3F] UINT8 efuse_pow_setting_2_d8 (refer the structure "EFUSE_POW_SETTING_2_S")
#ifndef _SUPPORT_BT_CTRL_FM_
    0x36,                       //EFUSE[40]  UINT8 efuse_pow_setting_3_d8; (refer the structure "EFUSE_POW_SETTING_3_S")
#else
    //0x26,   //EFUSE[1D1]  UINT8 efuse_pow_setting_3_d8; (refer the structure "EFUSE_POW_SETTING_3_S")
    0x36,                       //EFUSE[40]  UINT8 efuse_pow_setting_3_d8; (refer the structure "EFUSE_POW_SETTING_3_S")
#endif
    0x01,                       //EFUSE[41]  UINT8 efuse_pow_setting_4_d8; (refer the structure "EFUSE_POW_SETTING_4_S")
    0x00,                       //EFUSE[42]  UINT8 efuse_pow_setting_5_d8; (refer the structure "EFUSE_POW_SETTING_5_S")
    0x00,                       //EFUSE[43] UINT8 efuse_pow_setting_6_d8 (refer the structure "EFUSE_POW_SETTING_6_S")

    //**************for Bluetooth Para*************
    //*     size: ?? Bytes                        *
    //*********************************************
#ifdef TMP_FIX_OOB
      {0x11, 0x33, 0x55, 0x33, 0x11, 0x00}, //EFUSE[44~49]
#else
#ifdef _CCH_SC_ECDH_P256_TEST_SET_1
#ifdef _CCH_SC_ECDH_P256_TEST_SET_1_M
    {0xce, 0xbf, 0x37, 0x37, 0x12, 0x56},   //EFUSE[44~49]
#else
    {0xc1, 0xcf, 0x2d, 0x70, 0x13, 0xa7},   //EFUSE[44~49]
#endif
#elif defined(HAS_FPGA_BD_ADDR)
    FPGA_BD_ADDR,
#elif defined(_DAPE_TEST_FOR_MUTE)
      {0x95, 0x57, 0x12, 0x03, 0x87, 0xAA},       //EFUSE[44~49] _DAPE_TEST_FOR_MUTE, nish 20140502
#else
    //{0x33, 0x55, 0xb0, 0x21, 0x88, 0x00},       //EFUSE[44~49] UINT8 bt_bd_addr[6];//0x7c, 0x77, 0xd3, 0x72, 0x02, 0x03
    {0x87, 0x99, 0x23, 0x4c, 0xe0, 0x00},
    //{0x01, 0x20, 0x06, 0x21, 0x88, 0x00},       //EFUSE[44~49] UINT8 bt_bd_addr[6];//0x7c, 0x77, 0xd3, 0x72, 0x02, 0x03
    //{0x01, 0x20, 0x06, 0x03, 0x78, 0x00},       //EFUSE[44~49] UINT8 bt_bd_addr[6];//0x7c, 0x77, 0xd3, 0x72, 0x02, 0x03
    //{0x11, 0x33, 0x55, 0x33, 0x11, 0x00},       //EFUSE[44~49] UINT8 bt_bd_addr[6];//0x7c, 0x77, 0xd3, 0x72, 0x02, 0x03
#endif
#endif
    {
        'R', 'T', 'K', '_', 'B', 'T', '_', '4', //EFUSE[4A~89]  UINT8 bt_local_name[64];//'RTK_BT_4.0'
        '.', '1', '\0',   0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
        0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
        0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
        0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
        0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
        0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
        0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF
    },

    // Nish 20140804 notes: _DAPE_TEST_FOR_MUTE, for running mute,
    // please p efuse FMD_10_CQDDR as off
    {
        FMD_PAGE0_BYTE0, FMD_PAGE0_BYTE1, FMD_PAGE0_BYTE2, FMD_PAGE0_BYTE3, //EFUSE[8A~91] page 0 of LMP feature mask
        FMD_PAGE0_BYTE4, FMD_PAGE0_BYTE5, FMD_PAGE0_BYTE6, FMD_PAGE0_BYTE7
    },

    {
        0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
        0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF
    },                                      //EFUSE[92~A1] UINT8 bt_security_key[16];

    0x00,                                   //EFUSE[A2] UINT8 bt_country_code;//0x0

    0x02,                                   //EFUSE[A3] UINT8 bt_default_tx_power_index_minus;


    0x5d,                                   //EFUSE[A4~A5] UINT16 bt_manufacturer_name;//0x000a //0xffff
                                            //[1]0x000a is CSR's name, and this value help us be recognized by win7
    0xfffe,                                 //EFUSE[A6~A7] UINT16 bt_sleep_mode_threshold;//0x800

#ifdef _DLPS_SIMU_TEST_
    0x0030,                                 //EFUSE[A8~A9] UINT16 bt_deep_sleep_mode_threshold;//0xfffe
#else
    0x0120,                                 //EFUSE[A8~A9] UINT16 bt_deep_sleep_mode_threshold;//0xfffe
#endif

    0x0000,                                 //EFUSE[AA~AB] UINT16 bt_def_link_policy_settings;//0
    0x0024,                                 //EFUSE[AC~AD] UINT16 bt_t_poll_slave;//0x24
    0x0024,                                 //EFUSE[AE~AF] UINT16 bt_t_poll;//0x24
    0x7D00,                                 //EFUSE[B0~B1] UINT16 bt_supervision_timeout;//0x7D00
    0x2000,                                 //EFUSE[B2~B3] UINT16 bt_page_timeout;//0x2000
    0x1f40,                                 //EFUSE[B4~B5] UINT16 bt_conn_accept_timeout;//0x1f40

#ifdef _CCH_SC_ECDH_P256_TEST_PKT
    0x3FD,                                  //EFUSE[B6~B7] 0x0334,  UINT16 bt_read_buffer_size;//1021
#else
    0x0334,                                 //EFUSE[B6~B7] HCI_ACL_DATA_PAYLOAD_SIZE,  UINT16 bt_read_buffer_size;//1021
#endif

#ifdef _NISH_CHECK_EIR_
    0x0200,                                 //EFUSE[B8~B9] UINT16 bt_page_scan_interval;//0x200
    0x0012,                                 //EFUSE[BA~BB] UINT16 bt_page_scan_window;//0x12
    0x0600,                                 //EFUSE[BC~BD] UINT16 bt_inquiry_scan_interval;//0x600
    0x0012,                                 //EFUSE[BE~BF] UINT16 bt_inquiry_scan_window;//0x12
#else
#ifdef _DLPS_SIMU_TEST_
    0x040,                                  //EFUSE[B8~B9] UINT16 bt_page_scan_interval;//0x800
    0x005,                                  //EFUSE[BA~BB] UINT16 bt_page_scan_window;//0x12
    0x040,                                  //EFUSE[BC~BD] UINT16 bt_inquiry_scan_interval;//0x1000
    0x005,                                  //EFUSE[BE~BF] UINT16 bt_inquiry_scan_window;//0x12
#else
    0x0800,                                 //EFUSE[B8~B9] UINT16 bt_page_scan_interval;//0x800
    0x0012,                                 //EFUSE[BA~BB] UINT16 bt_page_scan_window;//0x12
    0x1000,                                 //EFUSE[BC~BD] UINT16 bt_inquiry_scan_interval;//0x1000
    0x0012,                                 //EFUSE[BE~BF] UINT16 bt_inquiry_scan_window;//0x12
#endif
#endif

    0xffff,                                 //EFUSE[C0~C1] UINT16 bt_hold_max_interval;//0xffff
    0x0001,                                 //EFUSE[C2~C3] UINT16 bt_hold_min_interval;//0x01
    0xffff,                                 //EFUSE[C4~C5] UINT16 bt_sniff_max_interval;//0xffff
    0x0001,                                 //EFUSE[C6~C7] UINT16 bt_sniff_min_interval;//0x01
    0x2000,                                 //EFUSE[C8~C9] UINT16 bt_beacon_max_interval;//0x2000
    0x00c0,                                 //EFUSE[CA~CB] UINT16 bt_beacon_min_interval;//0xc0
    0x0000,                                 //EFUSE[CC~CD] UINT16 bt_priority_low;
    0x0000,                                 //EFUSE[CE~CF] UINT16 bt_priority_high;
#ifdef _SUPPORT_VER_5_0_
    (BT_PRIORITY_3 | BIT15),                //EFUSE[D0~D1] 0x0000 bit[7:0]: bt_priority_3
                                            // bit[8]: IS_HARDWARE_ERR_EVENT_TO_HOST_WHEN_HOST_ACL
                                            // bit[9]: IS_SEND_EVT_WHEN_WDG_TO
                                            // bit[11:10]: NUM_RETX_TO_PAUSE_ESCO
                                            // bit[13:12]: PAUSE_ESCO_INTERVAL
                                            // bit[14]: IS_USE_FOR_MUTE
                                            // bit[15]: EN_FW_FIX_SCO_NAK_HID/IS_BT50
#else
    BT_PRIORITY_3,                          //EFUSE[D0~D1]
#endif    
    0xffff,                                 //EFUSE[D2~D3]  UINT16 bt_para_rsvd;

    0x0,                                    //EFUSE[D4~D7] UINT32 bt_class_of_device;//0x0

#if defined(_SUPPORT_SECURE_CONNECTION_)
    0xC0103FDF,                             //EFUSE[D8~DB] UINT32 bt_func_support_policy;
#elif defined(VER_CSA4)
    0xC0101FDF,                             //EFUSE[D8~DB] UINT32 bt_func_support_policy;
#elif (SUPPORT_BLUETOOTH_SPEC==40)
    0xC0100FDF,                             //EFUSE[D8~DB] UINT32 bt_func_support_policy;
#elif (SUPPORT_BLUETOOTH_SPEC==30)
    0xC0100FDD,                             //EFUSE[D8~DB] UINT32 bt_func_support_policy;
#else
    0xC0100FDC,                             //EFUSE[D8~DB] UINT32 bt_func_support_policy;
#endif

    0x0000002B,                             //EFUSE[DC~DF] UINT32 bt_le_fw_policy1;

#ifndef _DAPE_TEST_NEW_HW_LE_UNCONN_STATE_HIGHER_THAN_LEGACY_UNCONN
    0x1A9C4018,                             //EFUSE[E0~E3] UINT32 bt_le_fw_policy;  //0x029C4018
#else
#ifdef _DAPE_NO_LE_AUTO_CONN_UPDATE
// Nish 20140627 set adv report constraint directly
// Nish 20140627 #ifdef _DAPE_TEST_FOR_MUTE
// Nish 20140627 #else
    0x5A980018,                             //EFUSE[E0~E3] UINT32 bt_le_fw_policy;  //0x429C4018
// Nish 20140627 #endif
#else
// Nish 20140627 #ifdef _DAPE_TEST_FOR_MUTE
// Nish 20140627     //0x42984018,                             //UINT32 bt_le_fw_policy;  //0x429C4018
// Nish 20140627     0x42904018,                             //UINT32 bt_le_fw_policy;  //0x429C4018
// Nish 20140627 #else
    //0x5A984018,                             //UINT32 bt_le_fw_policy;  //0x429C4018
    0x5A904018,                             //EFUSE[E0~E3] UINT32 bt_le_fw_policy;  //0x429C4018
// Nish 20140627 #endif
#endif
#endif

#ifdef _CSB_RX_SET_XTOL_BY_EFUSE
#if defined(_SUPPORT_VER_4_2_)
    0x4415CE09,
#elif defined(_SUPPORT_VER_4_1_)
    0x4415CE08,                             //EFUSE[E4~E7] UINT32 bt_func_support_policy_ext;
#else
    0x0415CE08,                             //EFUSE[E4~E7] UINT32 bt_func_support_policy_ext;
#endif
#else
    0x0015CE08,                             //EFUSE[E4~E7] UINT32 bt_func_support_policy_ext;
#endif

    0x0220,                                 //EFUSE[E8~E9] UINT16 pta_default_setting;
#ifndef _DAPE_TEST_FOR_MUTE
    0x12,                                   //EFUSE[EA] UINT8 bt_global_slot_min_sniff_intv;
#else
    0x12,                                   //EFUSE[EA] UINT8 bt_global_slot_min_sniff_intv;
#endif

#ifdef _SUPPORT_WL_BT_SCOREBOARD_
// _SUPPORT_USB_TIMEOUT_INTERRUPT_
// check struct ISR_CONTRL
    0x00,                                   // EFUSE[0xEB]
                                            // bit[0]: wl2bt_int_msk, 1: can receive wl interrupt
                                            // bit[1]: bt2bt_mode, 1: interrupt, 0: polling
                                            // bit[2]: enable evt timeout interrupt, default off(0)
                                            // bit[3]: enable iso in timeout interrupt, default off(0)
                                            // bit[4]: enable iso out timeout interrupt, default off(0)
                                            // bit[5]: enable USB_token_timeout_en to SIE, default off(0)
#else
    0xFF,                                   // EFUSE[0xEB], original rsvd
#endif

#ifdef _DAPE_CHOOSE_LE_INTV_BY_CONTROLLER
#ifdef _DAPE_TEST_FOR_MUTE
    0x00,                                   //EFUSE[EC] UINT8 bt_auto_choose_le_intv;
#else
    0x01,                                   //EFUSE[EC] UINT8 bt_auto_choose_le_intv;
#endif
#else
    0xff,                                   //EFUSE[EC]  UINT8 rsvd_field_4;
#endif

#ifndef _OTP_BTON_GPIO_DEF_
    0xFF,                                   //EFUSE[ED]
#else
    0x00,                                   //EFUSE[ED]
                                            // bit[0]: enable led0 log output
#ifndef _SUPPORT_INFO_FROM_SYSTEM_ON_
                                            // bit[7:1]: reserved
#else
                                            // bit[1]: rsv
                                            // bit[2]: rsv
                                            // bit[3]: 0: interface info from page0, 1: from efuse, default = 0
                                            // bit[7:4] if(bit3 == 1), the 4 bits indicates chip version
#endif
#endif


    /*  Added by Wallice for HCI DMA Enhancement Function.	2012/03/20	*/
    //*********HCI DMA Enhancement Function**************
    //*     size: 2 Bytes                             					*
    //*************************************************
    0x03,                     //EFUSE[EE] UINT8    bit0 => HCI DMA Enhancement Function(ACL/LE) disable: 0, HCI DMA Enhancement Function(ACL/LE) enable: 1
    						  //		bit1 => HCI Hardware_Reset Event(ACL/LE) return disable: 0, HCI Hardware_Reset Event(ACL/LE) return enable: 1
    						  //		bit2 => HCI DMA Enhancement Function(ACL/LE) Interrupt disable: 0, HCI DMA Enhancement Function(ACL/LE) Interrupt enable: 1
    						  //		bit3 => HCI DMA Enhancement Function(CMD) Interrupt disable: 0, HCI DMA Enhancement Function(CMD) Interrupt enable: 1
    						  //		bit4 => HCI Hardware_Reset Event(CMD) return disable: 0, HCI Hardware_Reset Event(CMD) return enable: 1
    0xFF,                                   //EFUSE[EF] UINT8 reserved0
    /*  End Added by Wallice for HCI DMA Enhancement Function.		2012/03/20	*/

    /*  Added by Wallice for USB LPM Parameters.	2012/03/01	*/
    //******************for USB LPM*********************
    //*     size: 4 Bytes                             *
    //*************************************************
#ifndef _SUPPORT_POLLING_BASED_LPM_L1_
    0x00,                                   //EFUSE[F0] UINT8 bit 0 => LPM Disable: 0, LPM Enable: 1
                                            //     bit 1 => Control by F/W: 0, Control by Driver: 1
#else
    // default 0x00 (nyet, disable timer)
    // if test, set 0x13
    0x44,                                   // EFUSE[F0], for CV3 set 0x00, for normal set 0x02, 20140708, fongpin
                                            // bit 0 => LPM Disable: 0, LPM Enable: 1 (default = 0)              
                                            // bit 1 => Support USB LPM operation FSM by BT FW (default = 0)
                                            //          1: Yes, 0: NO  
                                            // bit 2 => USB LPM token response policy, (default = 1)
                                            //          0: always NYET, 
                                            //          1: return ACK when rx fifo is empty during a sample period
                                            // bit[7:3] => sample period (units: ms) default = 8 (dec)

#endif
    //0x11,                                   //EFUSE[F1] UINT8 HIRD/BESL Threshold Value. HIRD: bit7:4, BESL: bit3:0
    0xFF,                                   //EFUSE[F1] UINT8 HIRD/BESL Threshold Value. HIRD: bit7:4, BESL: bit3:0
#ifndef _SUPPORT_USB_LOG_ENABLE_
    0xFF,                                   //EFUSE[F2] UINT8 LPM reserved 0
#else
    0x10,                                   //EFUSE[F2] UINT8 usb misc
                                            // bit0: enable usb log
                                            // bit[3:1] reserved
                                            // bit[7:4]: d2 timer log, N*10sec
#endif
    /*  End Added by Wallice for USB LPM Parameters.	2012/03/01	*/

    /*  Updated by Wallice for PCM external CODEC.    2013/07/08  */
    //**************for PCM External CODEC*************
    //*     size: 8 Bytes                             *
    //*************************************************
    // new pcm default
    0x00,                                   //EFUSE[F3] PCM Convert disable
    0x00,                                   //EFUSE[F4] Bit[3:0]-> HCI:0x0, PCM External CODEC:0x1, I2S External Codec: 0x02
							 // Bit[7:4]-> Linear: 0x00, u-Law: 0x01, A-Law: 0x02, CVSD: 0x03, Transparent(mSBC): 0x04
    0x00,                                  //EFUSE[F5] UINT8 Data format, ok
    0xA001,                                //EFUSE[F6~F7] UINT16 PCM I/F Ctrl1
    0x0000,                                //EFUSE[F8~F9] UINT16 PCM I/F Ctrl2, ok
    0x8008,                                //EFUSE[FA~FB] UINT16 PCM I/F Ctrl3, ok

    0x1F00,                                //EFUSE[FC~FD], PARK_PCM_GPIO_PIN_S_TYPE
//    0x1CA0,                                //EFUSE[0xE4~0xE5], PARK_PCM_GPIO_PIN_S_TYPE

    /*  End Updated by Wallice for mSBC over PCM.    2013/11/20  */

    //**************for BW RF Settings*************
    //*     size: ?? Bytes                        *
    //*********************************************
    0x09,                                   //EFUSE[FE] UINT8 bw_rf_radio_sys_clk_sel;//RT:0x09
    0x0C,                                   //EFUSE[FF] UINT8 bw_rf_osc_delay;//0xC
#ifdef _YL_LPS_FORCE_LPO32p768K
    0x00,                                   //EFUSE[100] UINT8 bw_rf_low_clk_frac;	//0x1 //32K(1) or 32.768K(0)
#else
    0x01,                                   //EFUSE[100] UINT8 bw_rf_low_clk_frac;	//0x1 //32K(1) or 32.768K(0)
#endif
    0xFF,                                   //EFUSE[101] UINT8 bw_rf_reserved[3];
    0xFFD8,                                 //EFUSE[102~103] INT16 bw_rf_min_rssi_dbm; /-40 dBm
    0xFFE2,                                 //EFUSE[104~105] INT16 bw_rf_max_rssi_dbm;//-30 dBm

#ifndef _DAPE_TEST_NEW_HW_UPDATE_SLAVE_TIMEING_AFTER_HEC
                                                //EFUSE[106~119] UINT16 bw_rf_delay_vals[10]
    {//bw_rf_delay_vals[10]
        0xDBCE, 0xDBCE, 0x8989, 0x8989, 0x136,  //RT: 0xC0B8, 0xE0D3, 0x8989, 0x8989, 0x116,
        0x2149, 0x140, 0x146, 0x933C, 0xFFFF    //    0x140, 0x140, 0x142, 0x933C, 0xFFFF
    },
#else
    {
  #ifdef _CCH_SC_ECDH_P256_HW_TIMING
    #ifdef _BRUCE_RF_DELAY_TIMING  // early 5 us
        0xD0C3, 0xD0C3, 0x8989, 0x8989, 0x531,  //RT: 0xC0B8, 0xE0D3, 0x8989, 0x8989, 0x116,
        0x2152, 0x2B49, 0x14F, 0x933C, 0xFFFF    //    0x140, 0x140, 0x142, 0x933C, 0xFFFF
    #else
        0xA598, 0xA598, 0x8989, 0x8989, 0x506,  //RT: 0xC0B8, 0xE0D3, 0x8989, 0x8989, 0x116,
        0x217D, 0x8144, 0x17A, 0x933C, 0xFFFF    //    0x140, 0x140, 0x142, 0x933C, 0xFFFF
    #endif
  #else
        0xD5C8, 0xD5C8, 0x8989, 0x8989, 0x536,  //RT: 0xC0B8, 0xE0D3, 0x8989, 0x8989, 0x116,
        0x214D, 0x2144, 0x14A, 0x933C, 0xFFFF    //    0x140, 0x140, 0x142, 0x933C, 0xFFFF
        //0xDBCE, 0xDBCE, 0x8989, 0x8989, 0x536,  //RT: 0xC0B8, 0xE0D3, 0x8989, 0x8989, 0x116,
        //0x2149, 0x1B40, 0x146, 0x933C, 0xFFFF    //    0x140, 0x140, 0x142, 0x933C, 0xFFFF
  #endif
    },
#endif

    //********for LE Controller Timing Settings******
    //*     size: 10 Bytes                         *
    //*********************************************
#ifndef _DAPE_TEST_LE_CE_EARLY420US
    0x04AF,                                 //EFUSE[11A~11B] UINT16 le_ce_early_int_time;
#else
    //0x012C,                                 //UINT16 le_ce_early_int_time; 300us
    //0x01F4,                                 //UINT16 le_ce_early_int_time; 500us
    0x0168,                                 //EFUSE[11A~11B] UINT16 le_ce_early_int_time; 360us
    //0x0190,                                 //UINT16 le_ce_early_int_time; 400us ok
#endif

    0xDCDC,                                 //EFUSE[11C~11D]  UINT16 le_trx_on_delay;

#ifndef _NEW_MODEM_DESIGN_
    0x328A,                                 //EFUSE[11E~11F] UINT16 le_rx_turnaround_delay;
#else
    0x3286,                                 //EFUSE[11E~11F]  UINT16 le_rx_turnaround_delay;
#endif
#ifdef _DAPE_ENABLE_LE_EARLY_TX_TOGGLE
    0x5083,                                 //EFUSE[120~121] UINT16 le_tx_turnaround_delay;
#else
    0x1083,                                 //EFUSE[120~121] UINT16 le_tx_turnaround_delay;
#endif
    0x0164,                                 //EFUSE[122~123] UINT16 le_ce_rx_timeout_delay;
    0x01F0,                                 //EFUSE[124~125] UINT16 le_rx_search_timeout_value;
    0x0147,                                 //EFUSE[126~127] UINT16 le_clock_compensate;

    //********for Jr.Neil's AFH Settings*************
    //*     size: 16 Bytes                         *
    //*********************************************
    0x01,                                   //EFUSE[128] UINT8 rtk_afh_mechanism_enable;
#ifndef _NEW_MODEM_PSD_SCAN_
    0xff,                                   //EFUSE[129] UINT8 afh_reserved_00;
#else
    0x00,                                   //EFUSE[129] UINT8 rtk_afh_bt_psd_enable; (enable it until the functon is qualified)
#endif
    0x1C,                                   //EFUSE[12A] UINT8 min_channels_by_rtk;
    0x00,                                   //EFUSE[12B] UINT8 rtk_afh_using_slave_report;
#ifndef WLAN_PSD
    0x00,                                   //EFUSE[12C] UINT8 rtk_afh_wlan_psd_enable;
#else
    //0x01,                                 //UINT8 rtk_afh_wlan_psd_enable; (mark it until wifi driver ready - austin)
    0x00,                                   //EFUSE[12C] UINT8 rtk_afh_wlan_psd_enable;
#endif
    0x64,                                   //EFUSE[12D] UINT8 max_afh_execute_times;
    0x32,                                   //EFUSE[12E] UINT8 afh_execute_times_I;
    0x19,                                   //EFUSE[12F] UINT8 afh_execute_times_II;
    0x0C,                                   //EFUSE[130] UINT8 afh_execute_times_III;
    0x05,                                   //EFUSE[131] UINT8 afh_execute_times_IV;
    0x03,                                   //EFUSE[132] UINT8 min_afh_execute_times;
    0x0A,                                   //EFUSE[133] UINT8 wlan_gap_threshold;
    0x0000,                                 //EFUSE[134~135]  INT16 score_threshold;
#ifndef WLAN_PSD
    0x00,                                   //EFUSE[136]  UINT8 afh_check_wifi_alive_count
#else
    0x00,
    //0x0A,                                   //EFUSE[136] UINT8 afh_check_wifi_alive_count
#endif
    -25,                                    //EFUSE[137] INT8 trk_afh_score_thres_sco;
    -19,                                    //EFUSE[138] INT8 trk_afh_score_thres_i;
    -38,                                    //EFUSE[139] INT8 trk_afh_score_thres_ii;

    //**** for RF Settings - Tx Gain Table***********
    //*      size: 40 Bytes                        *
    //*********************************************
    {
//#if defined(_IS_ASIC_) && defined(_RTL8821A_SPECIFIC_)
#if defined(_IS_ASIC_)
    #if defined(_RTL8821B_TC_SPECIFIC_)
        0x0B, 0x0E, 0x2A, 0x2D, 0x4A, 0x4D, 0x69, 0x89,
        0x8F, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
        0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
        0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF
        //TBD
    #elif defined(_RTL8703B_SPECIFIC_) || defined(_RTL8822B_SPECIFIC_)
        0x0B, 0x0E, 0x2A, 0x2D, 0x4A, 0x4D, 0x69, 0x89,
        0x8F, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
        0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
        0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF
        //TBD
    #elif defined(_RLE0380_MP_CHIP_)
        // NOT used in FPGA (?), 8723A ASIC Old Parameters
        0x02, 0x03, 0x04, 0x05, 0x06, 0x0D, 0x0E, 0x15,
        0x16, 0x1D, 0x1E, 0x2E, 0x37, 0x3F, 0xFF, 0xFF,
        0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
        0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF
    #else
        !!!! ERROR: UNEXPECTED !!!!
    #endif
#else
        /* fpga mode dummy data */
        0x02, 0x03, 0x04, 0x05, 0x06, 0x0D, 0x0E, 0x15,
        0x16, 0x1D, 0x1E, 0x2E, 0x37, 0x3F, 0xFF, 0xFF,
        0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
        0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF
#endif
    },                                          //EFUSE[13A~159] UINT8 EFuse_PHYInit_TxGainTable[32];

#if defined(_IS_ASIC_)
    #if defined(_RTL8821B_TC_SPECIFIC_)
        0x08,                                   //EFUSE[15A] UINT8 EFuse_PHYInit_MaxTxIndex;
        0x07,                                   //EFUSE[15B] UINT8 EFuse_PHYInit_MaxTxGain1M;
        0x07,                                   //EFUSE[15C] UINT8 EFuse_PHYInit_MaxTxGain2M;
        0x07,                                   //EFUSE[15D] UINT8 EFuse_PHYInit_MaxTxGain3M;
        0x07,                                   //EFUSE[15E] UINT8 EFuse_PHYInit_MaxTxGainLE;
        //TBD
    #elif defined(_RTL8703B_SPECIFIC_) || defined (_RTL8822B_SPECIFIC_)
        0x08,                                   //EFUSE[15A] UINT8 EFuse_PHYInit_MaxTxIndex;
        0x07,                                   //EFUSE[15B] UINT8 EFuse_PHYInit_MaxTxGain1M;
        0x07,                                   //EFUSE[15C] UINT8 EFuse_PHYInit_MaxTxGain2M;
        0x07,                                   //EFUSE[15D] UINT8 EFuse_PHYInit_MaxTxGain3M;
        0x07,                                   //EFUSE[15E] UINT8 EFuse_PHYInit_MaxTxGainLE;
        //TBD
    #elif defined(_RLE0380_MP_CHIP_)
        0x0D,                                   //UINT8 EFuse_PHYInit_MaxTxIndex;
        0x0C,                                   //UINT8 EFuse_PHYInit_MaxTxGain1M;
        0x0C,                                   //UINT8 EFuse_PHYInit_MaxTxGain2M;
        0x0C,                                   //UINT8 EFuse_PHYInit_MaxTxGain3M;
        0x0B,                                   //UINT8 EFuse_PHYInit_MaxTxGainLE;
    #else
        !!!! ERROR: UNEXPECTED !!!!
    #endif
#else
        /* fpga mode dummy data */
        0x0D,                                   //UINT8 EFuse_PHYInit_MaxTxIndex;
        0x0C,                                   //UINT8 EFuse_PHYInit_MaxTxGain1M;
        0x0C,                                   //UINT8 EFuse_PHYInit_MaxTxGain2M;
        0x0C,                                   //UINT8 EFuse_PHYInit_MaxTxGain3M;
        0x0B,                                   //UINT8 EFuse_PHYInit_MaxTxGainLE;
#endif


#if defined(_IS_ASIC_)
    #if defined(_RTL8821B_TC_SPECIFIC_)
        0x01,                                  //EFUSE[15F] UINT8 EFuse_PHYInit_TxGainStep;
        EFUSE_INVALID_TXSCALEFACTOR, //0x0A,      //EFUSE[160] UINT8 EFuse_PHYInit_TxScaleFactor;
        EFUSE_INVALID_TXDAC_CURRENT, //0x0d,      //EFUSE[161] UINT8 EFuse_PHYInit_TxDACCurrent;
    #elif defined(_RTL8703B_SPECIFIC_) || defined (_RTL8822B_SPECIFIC_)
        0x01,                                  //EFUSE[15F] UINT8 EFuse_PHYInit_TxGainStep;
        EFUSE_INVALID_TXSCALEFACTOR, //0x0A,      //EFUSE[160] UINT8 EFuse_PHYInit_TxScaleFactor;
        EFUSE_INVALID_TXDAC_CURRENT, //0x0d,      //EFUSE[161] UINT8 EFuse_PHYInit_TxDACCurrent;
    #elif defined(_RLE0380_MP_CHIP_)
        0x01,                                   //UINT8 EFuse_PHYInit_TxGainStep;
        0x0A,                                   //UINT8 EFuse_PHYInit_TxScaleFactor;
        0x15,                                   //UINT8 EFuse_PHYInit_TxDACCurrent;
    #else // NOT used in FPGA, 8723A ASIC Old Parameters
        !!!! ERROR: UNEXPECTED !!!!
    #endif
#else
        /* fpga mode dummy data */
        0x01,                                   //UINT8 EFuse_PHYInit_TxGainStep;
        0x0A,                                   //UINT8 EFuse_PHYInit_TxScaleFactor;
        0x15,                                   //UINT8 EFuse_PHYInit_TxDACCurrent;
#endif

    //**** for RF Settings - Rx Gain Table***********
    //*      size: 10 Bytes                        *
    //*********************************************
#ifdef _NEW_8821A_RX_AGC_API_
//  #if defined(_IS_ASIC_) && defined(_RTL8821A_SPECIFIC_)
  #if defined(_IS_ASIC_) && (defined(_RTL8821B_TC_SPECIFIC_))
    0x2034,                                 //EFUSE[162~163] UINT16 EFuse_PHYInit_Gain0Stop;
    0x361E,                                 //EFUSE[164~165] UINT16 EFuse_PHYInit_Gain0Start;
    0x7517,                                 //EFUSE[166~167] UINT16 EFuse_PHYInit_Gain1Start;
    0x9510,                                 //EFUSE[168~169] UINT16 EFuse_PHYInit_Gain2Start;
    0xB60C,                                 //EFUSE[16A~16B] UINT16 EFuse_PHYInit_Gain3Start;
    0xDC00,                                 //EFUSE[16C~16D] UINT16 EFuse_PHYInit_Gain4Start;
    0x0000,                                 //EFUSE[16E~16F] UINT16 EFuse_PHYInit_Gain5Start;
    0x0000,                                 //EFUSE[170~171] UINT16 EFuse_PHYInit_Gain6Start;
    0x0000,                                 //EFUSE[172~173] UINT16 EFuse_PHYInit_Gain7Start;

  #elif defined(_IS_ASIC_) && (defined(_RTL8703B_SPECIFIC_) || defined(_RTL8822B_SPECIFIC_))
    0x2034,                                 //UINT16 EFuse_PHYInit_Gain0Stop;
    0x361E,                                 //UINT16 EFuse_PHYInit_Gain0Start;
    0x7517,                                 //UINT16 EFuse_PHYInit_Gain1Start;
    0x9510,                                 //UINT16 EFuse_PHYInit_Gain2Start;
    0xB60C,                                 //UINT16 EFuse_PHYInit_Gain3Start;
    0xDC00, // UINT16 EFuse_PHYInit_Gain4Start;
    0x0000, // UINT16 EFuse_PHYInit_Gain5Start;
    0x0000, // UINT16 EFuse_PHYInit_Gain6Start;
    0x0000, // UINT16 EFuse_PHYInit_Gain7Start;

    //TBD
  #elif defined(_FPGA_WITH_RLE0546_RFE_) && !defined(_IS_ASIC_)
    0x2034,                                 //UINT16 EFuse_PHYInit_Gain0Stop;
    0x361E,                                 //UINT16 EFuse_PHYInit_Gain0Start;
    0x7517,                                 //UINT16 EFuse_PHYInit_Gain1Start;
    0x9510,                                 //UINT16 EFuse_PHYInit_Gain2Start;
    0xB60C,                                 //UINT16 EFuse_PHYInit_Gain3Start;
    0xDC00, // UINT16 EFuse_PHYInit_Gain4Start;
    0x0000, // UINT16 EFuse_PHYInit_Gain5Start;
    0x0000, // UINT16 EFuse_PHYInit_Gain6Start;
    0x0000, // UINT16 EFuse_PHYInit_Gain7Start;
  #elif defined(_FPGA_WITH_RLE0608C_RFE_) && !defined(_IS_ASIC_)
    TBD
  #elif defined(_FPGA_WITH_RLE0550_RFE_) && !defined(_IS_ASIC_)
    0x2035, // UINT16 EFuse_PHYInit_Gain0Stop;
    0x701E, // UINT16 EFuse_PHYInit_Gain0Start;
    0x7717, // UINT16 EFuse_PHYInit_Gain1Start;
    0xB410, // UINT16 EFuse_PHYInit_Gain2Start;
    0xD806, // UINT16 EFuse_PHYInit_Gain3Start;
    0xF800, // UINT16 EFuse_PHYInit_Gain4Start;
    0x0000, // UINT16 EFuse_PHYInit_Gain5Start;
    0x0000, // UINT16 EFuse_PHYInit_Gain6Start;
    0x0000, // UINT16 EFuse_PHYInit_Gain7Start;
  #else /* 0379 RF*/
    0x082c,                                 //UINT16 EFuse_PHYInit_Gain0Stop;
    0x092b,                                 //UINT16 EFuse_PHYInit_Gain0Start;
//    0x052c,                                 //UINT16 EFuse_PHYInit_Gain0Stop;
//    0x062b,                                 //UINT16 EFuse_PHYInit_Gain0Start;
    0x5515,                                 //UINT16 EFuse_PHYInit_Gain1Start;
    #if 1 /* fix 0379 AGC Bug: Gap at -45dBm */
      0x7900,                                 //UINT16 EFuse_PHYInit_Gain2Start;
    #else
      0x7a00,                                 //UINT16 EFuse_PHYInit_Gain2Start;
    #endif
    0x0000,                                 //UINT16 EFuse_PHYInit_Gain3Start;
    0x0000, // UINT16 EFuse_PHYInit_Gain4Start;
    0x0000, // UINT16 EFuse_PHYInit_Gain5Start;
    0x0000, // UINT16 EFuse_PHYInit_Gain6Start;
    0x0000, // UINT16 EFuse_PHYInit_Gain7Start;
  #endif
#else /* else of _NEW_8821A_RX_AGC_API_ */
    /* For 8723A RF */
    0x7d00,                                 //UINT16 EFuse_PHYInit_HGainStart;
    0x570A,                                 //UINT16 EFuse_PHYInit_MGainStart;
    0x2F1E,                                 //UINT16 EFuse_PHYInit_LGainStart;
    0x0E28,                                 //UINT16 EFuse_PHYInit_UGainStart;
    0x0036,                                 //UINT16 EFuse_PHYInit_UGainStop;
    {0xffff, 0xffff, 0xffff, 0xffff},
#endif

#ifdef _NEW_BT_EFUSE_OPTION_FOR_TV_APP_
    //****** TV 3DD Settings **********************
    //*      size: 20 Bytes                       *
    //*********************************************
    {
                //EFuse[174 ~ 187]
        0x74,   //UINT8 enable_3dd_gpio_deboubcing : 1;
                //UINT8 enable_3d_sync_schmitt : 1;
                //UINT8 version_3DSP : 1;
                //UINT8 fast_recover_times : 5;
        0x02,   //UINT8 L_R_first : 1;
                //UINT8 calculation_window : 2;
                //UINT8 period_delay : 2;
                //UINT8 reserved : 3;
        0x1E,   //INT8  lsod;
        0xE2,   //INT8  lscd;
        0x1E,   //INT8  rsod;
        0xE2,   //INT8  rscd;
        0x4B,   //UINT8 period_diff_limit;
        0x05,   //UINT8 shutter_open_delay;
        0x05,   //UINT8 shutter_close_delay;
        0x4A,   //UINT8 rejection_times : 4;
                //UINT8 toggle_detection_window : 4;
        0x0F,   //UINT8 toggle_detection_delay;
        0x64,   //UINT8 period_significant_diff;
        0x41,   //UINT8 coeff_gain : 4;
                //UINT8 coeff_avg : 4;
        0x0F,   //UINT8 period_significant_diff_long_term;
        0xFF, 0xFF,
        0xFF, 0xFF,
        0xFF, 0xFF
    }, //UINT8 EFuse_TV_3DD_Group[20];
#else
    //********** for RF Settings - RF IQK ***********
    //*      size: 20 Bytes                        *
    //*********************************************
    0x00,                                   //UINT8 EFuse_IQK_Settings;
    0x0A,                                   //UINT8 EFuse_IQK_RFChL;
    0x46,                                   //UINT8 EFuse_IQK_RFChH;
    0x71,                                   //UINT8 EFuse_IQK_TxGainL;
    0x71,                                   //UINT8 EFuse_IQK_TxGainH;
    0x8B,                                   //UINT8 EFuse_IQK_RxGain;
    0x03,                                   //UINT8 EFuse_IQK_ErrorTh;
    0x00,                                   //UINT8 EFuse_IQK_LoadExtSetup;
    0x0040,                                 //UINT16 EFuse_IQK_TxIMRDefault;
    0xE08B,                                 //UINT16 EFuse_IQK_RxIMRDefault;
    0x9200,                                 //UINT16 EFuse_IQK_LOKLDfault;
    0x9200,                                 //UINT16 EFuse_IQK_LOKHDefault;
    0x0000,                                 //UINT16 EFuse_IQK_RxAGC;
    0x108A,                                 //UINT16 EFuse_IQK_IQKPara;
#endif

    //**** for RF Settings - Tx Power Tracking ******
    //*      size: 16 Bytes                        *
    //*********************************************
    0x00,                                   //EFUSE[188] UINT8 EFuse_TxPowerTrack_En;
    0x1C,                                   //EFUSE[189] UINT8 EFuse_ThermalDefault;
    0x05,                                   //EFUSE[18A] UINT8 EFuse_ThermalUpdateInterval;
    0x03,                                   //EFUSE[18B] UINT8 EFuse_TxPowerTrack_ThermalDelta;
    {
        0xF9, 0xFA, 0xFC, 0x06, 0x07, 0x09,
        0x0A, 0x0C, 0x0E, 0x17, 0x19, 0x1B,
        0x1D
    },                                      //EFUSE[18C~198] UINT8 EFuse_TxPowerTrack_TxGain [13];
    {0xf8, 0xf9},                           //EFUSE[199~19A] UINT8 EFuse_TxPowerTrack_TxGain_LBond[2];
    {0x1f, 0x1f},                           //EFUSE[19B~19C] UINT8 EFuse_TxPowerTrack_TxGain_HBond[2];

    //**** for RF Settings - CFO (Center Frequency Offset) ****
    //*                size: 14 Bytes                        *
    //*******************************************************
    0x00,                                   //EFUSE[19D] UINT8 EFuse_CFOTrack_En;
    0x03,                                   //EFUSE[19E] UINT8 EFuse_CFOTrack_ThermalDelta;
    {
        0x74, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x07, 0x10,
        0x1F
    },                                      //EFUSE[19F~1AB] UINT8 EFuse_CFOTrack_Cap[13];
    {0x6e, 0x74},                           //EFUSE[1AC~1AD] UINT8 EFuse_CFOTrack_Cap_LBond[2];
    {0x1f, 0x1f},                           //EFUSE[1AE~1AF] UINT8 EFuse_CFOTrack_Cap_HBond[2];

#ifdef _NEW_BT_EFUSE_OPTION_FOR_TV_APP_
    //****** TV Power On Settings *****************
    //*      size: 30 Bytes                       *
    //*********************************************
    {
                                //EFuse[1B0 ~ 1CD]
        0x28, 0x23,             //UINT16 IR_lead_high_len;
        0x94, 0x11,             //UINT16 IR_lead_low_len;
        0x17, 0x12, 0x68,       //UINT16 IR_logic1_high_len : 12;
                                //UINT16 IR_logic1_low_len : 12;
        0x17, 0x72, 0x21,       //UINT16 IR_logic0_high_len : 12;
                                //UINT16 IR_logic0_low_len : 12;
        0x20,                   //UINT8 IR_num_bits;
#ifdef EFUSE_POWERON_TV_EN_ON
        0x3D,                   //UINT8 power_on_tv_en : 1;
#else
        0x3C,                   //UINT8 power_on_tv_en : 1;
#endif
                                //UINT8 TV_default_idle_state : 1;
                                //UINT8 IR_logic1_start_level : 1;
                                //UINT8 IR_logic0_start_level : 1;
                                //UINT8 IR_has_stop_bit : 1;
                                //UINT8 IR_stop_bit_mode : 1;
                                //UINT8 has_rcuid_check : 1;
                                //UINT8 reserved0
        0xFF, 0xFF,             //UINT16 rc_uid;
        0x00,                   //UINT8 host_power_stat_mode : 3;
                                //UINT8 host_poweron_polarity : 1;
                                //UINT8 heartbeat_event_enable : 1;
                                //UINT8 reserved1 : 3;
        0x19,                   //UINT8 heartbeat_period;
        0x0A, 0x00,             //UINT16 le_scan_window;
        0x64, 0x00,             //UINT16 le_scan_interval;
        TV_POWERON_PIN_CONFIG,  //UINT8 poweron_pin_polarity : 1;
                                //UINT8 poweron_pin_type : 2;
                                //UINT8 poweron_pin_num : 5;
        0xFF,
        0xFF, 0xFF,
        0xFF, 0xFF,
        0xFF, 0xFF,
        0xFF, 0xFF
    }, //UINT8 EFuse_TV_Poweron_Group[30];
#else
    //**** for RF Settings - Rx Power Tracking ******
    //*      size: 28 Bytes                        *
    //*********************************************
    0x00,                                   //UINT8 EFuse_RxGainTrack_En;
    0x03,                                   //UINT8 EFuse_RxGainTrack_ThermalDelta;
    {
        0x04C1, 0x0441, 0x0401, 0x02C1,
        0x0241, 0x0201, 0x04C1, 0x0441,
        0x0401, 0x02C1, 0x0241, 0x0201,
        0x04C1
    },                                      //UINT16 EFuse_RxGainTrack_Para[13];

    //******* for RF Settings - Re-Calibration ******
    //*       size: 2 Bytes                        *
    //*********************************************
    0x0A,                                   //UINT8 EFuse_ReRFCal_ThermalDelta;
    0x00,                                   //UINT8 EFuse_ReRFCal_Settings;
#endif

    //********** for RF Settings - LOK Tracking *****
    //*      size: 54 Bytes                        *
    //*********************************************
    //{0xff, 0xff},                           //UINT8 EFuse_Rsvd_0[2];
#ifndef _SUPPORT_ANTENNA_SELECTION_
    {0xff, 0xff},                           //UINT8 EFuse_Rsvd_0[2];
#else
    0xF7,                                   //EFUSE[1CE] UINT8 Efuse_Ant_cfg0
    0xBE,                                   //EFUSE[1CF] UINT8 Efuse_Ant_cfg1
#endif

//#ifdef _NEW_MODEM_DESIGN_PHY_SETTING_

#ifdef _NEW_MODEM_PI_ACCESS_BY_VENDOR_REG_
    0x01, //EFUSE[1D0] UINT8 efuse_modem_setting_1_d8; (refer the structure "EFUSE_MODEM_SETTING_1_S")
#else
    0x08, //EFUSE[1D0] UINT8 efuse_modem_setting_1_d8; (refer the structure "EFUSE_MODEM_SETTING_1_S")
#endif

    //#if defined(_IS_ASIC_) && defined(_RTL8821A_SPECIFIC_)
#if defined(_IS_ASIC_) && (defined(_RTL8821B_TC_SPECIFIC_))
    -96, //EFUSE[1D1] INT8 efuse_modem_rssi0_pin_dBm
#elif defined(_IS_ASIC_) && (defined(_RTL8703B_SPECIFIC_) || defined(_RTL8822B_SPECIFIC_))
    -96, // INT8 efuse_modem_rssi0_pin_dBm
    //TBD
#elif defined(_FPGA_WITH_RLE0546_RFE_) && !defined(_IS_ASIC_)
    -96, // INT8 efuse_modem_rssi0_pin_dBm
#elif defined(_FPGA_WITH_RLE0608C_RFE_) && !defined(_IS_ASIC_)
    TBD
#elif defined(_FPGA_WITH_RLE0550_RFE_) && !defined(_IS_ASIC_)
    -96, // INT8 efuse_modem_rssi0_pin_dBm
#else /* 0379 RF*/
    -90, // INT8 efuse_modem_rssi0_pin_dBm
#endif

    0x8E4C, //EFUSE[1D2~1D3]  efuse_baud_est_setting_1_d16 (refer the structure "EFUSE_BAUD_EST_SETTING_1_S")
    0x0353, //EFUSE[1D4~1D5]  efuse_baud_est_setting_2_d16 (refer the structure "EFUSE_BAUD_EST_SETTING_2_S")
    0xCFFF, //EFUSE[1D6~1D7]  efuse_baud_est_setting_3_d16 (refer the structure "EFUSE_BAUD_EST_SETTING_3_S")
    0xE34D, //EFUSE[1D8~1D9]  efuse_baud_est_setting_4_d16 (refer the structure "EFUSE_BAUD_EST_SETTING_4_S")
    0x00B7, //EFUSE[1DA~1DB]  efuse_modem_psd_setting_1_d16 (refer the structure "EFUSE_MODEM_PSD_SETTING_1_S")
    0x0000, //EFUSE[1DC~1DD]  efuse_modem_psd_setting_2_d16 (refer the structure "EFUSE_MODEM_PSD_SETTING_2_S")

    0x000B,                                 //EFUSE[1DE~1DF] UINT16 EFuse_Rsvd_2;
    {
        0xffff,
        0xffff,
        0xffff,
        0xffff
    },                                      //EFUSE[1E0~1E7] UINT16 EFuse_Rsvd_buf[4];
    0xffff,                                //EFUSE[1E8~1E9] UINT16 efuse_host_info
    0x0003,									//EFUSE[1EA~1EB] UINT16 efuse_ft_info
    {
        0xffff,
        0xffff,
        0xffff,
        0xffff
    },                                      //EFUSE[1EC~1F3] UINT16 efuse_cal_sts[4]

    //**************for RF Parameter Settings*************
    //*     size: 104 Bytes                        *
    //*********************************************

    // for 0380 new modme parameter
#ifdef _IS_ASIC_
        //=================================================
        //============= EFuse_PHYInit_RegAddr =============
        //=================================================
//#if defined(_IS_ASIC_) && defined(_RTL8821A_SPECIFIC_)  /* RTL8821A */
#if defined(_IS_ASIC_) && (defined(_RTL8821B_TC_SPECIFIC_))
        removed for clean
#elif defined(_IS_ASIC_) && (defined(_RTL8703B_SPECIFIC_) || defined(_RTL8822B_SPECIFIC_) ||\
                             defined(_RTL8723D_SPECIFIC_) || defined(_RTL8821C_SPECIFIC_))
    {   //EFUSE[1F4~273]  UINT16 sys_init_param[64];
        0xFFFF, 0xFFFF,     0xFFFF, 0xFFFF,     0xFFFF, 0xFFFF,     0xFFFF, 0xFFFF,
        0xFFFF, 0xFFFF,     0xFFFF, 0xFFFF,     0xFFFF, 0xFFFF,     0xFFFF, 0xFFFF,
        0xFFFF, 0xFFFF,     0xFFFF, 0xFFFF,     0xFFFF, 0xFFFF,     0xFFFF, 0xFFFF,
        0xFFFF, 0xFFFF,     0xFFFF, 0xFFFF,     0xFFFF, 0xFFFF,     0xFFFF, 0xFFFF,
        0xFFFF, 0xFFFF,     0xFFFF, 0xFFFF,     0xFFFF, 0xFFFF,     0xFFFF, 0xFFFF,
        0xFFFF, 0xFFFF,     0xFFFF, 0xFFFF,     0xFFFF, 0xFFFF,     0xFFFF, 0xFFFF,
        0xFFFF, 0xFFFF,     0xFFFF, 0xFFFF,     0xFFFF, 0xFFFF,     0xFFFF, 0xFFFF,
        0xFFFF, 0xFFFF,     0xFFFF, 0xFFFF,     0xFFFF, 0xFFFF,     0xFFFF, 0xFFFF,
    },
    {
        0x203A, 0x6C51,     0x2036, 0x78db,     0x2034, 0xc311,     0x2016, 0x5a1e,
        0x8072, 0xd034,
         //----rf_parameter_adc_1

        0x2038, 0x0661,      0x2064, 0x0080,     0x8072, 0xd034,
        //----rf_parameter_modem_reg_2

        0x0021, 0xAE00,     0x002B, 0x0FA9,     0x4136, 0x0806,     0x4138, 0x3626,
        0x413A, 0x4638,     0x413C, 0x4A4A,     0x41BC, 0x0806,     0x41BE, 0x3626,
        0x41C0, 0x4638,     0x41C2, 0x4A4A,     0x41C4, 0x0806,     0x41C6, 0x3626,
        0x41C8, 0x4638, 	0x01CA, 0x4A4A,     0xE062, 0x5800,     0xE002, 0x5800,
        //----rf_parameter_setting_4

        0x0000, 0x3000,     0x003D, 0xFE40,     0x003E, 0x4000,     0x000E, 0xA420,
        0x0034, 0x2A4E,     0x0035, 0x8C50,     0x0030, 0x2c13,		0x0036, 0x4003,
        0x000E, 0x2420,		0x0024, 0x0007,     0x6000, 0x012C,     0x0024, 0x0000,
        0x003F, 0x0105,		0x000E, 0xA420,     0x003D, 0x0050,     0x003E, 0x0050,
        0x003F, 0x057E,		0x0020, 0xB811,     0x0021, 0x8889,     0x0026, 0x0808,
        0x0027, 0x8889,		0x0028, 0x8889,     0x0022, 0xC000,		0x002a, 0xFFF0,
        0x002b, 0x7800,		0x000e, 0x2420,     0x003F, 0x0103,
        //----rf_parameter_fix_sync_table_5

        0x000F, 0x0010,     0x001A, 0xD089,     0x001A, 0xC006,     0x001A, 0xB004,
        0x001A, 0xA00A,     0x001A, 0x9008,     0x001A, 0x8018,     0x001A, 0x7A40,
        0x001A, 0x6286,     0x001A, 0x528A,     0x001A, 0x45C0,     0x001A, 0x3420,
        0x000F, 0x0000,
        //----rf_parameter_Rx_table_setting_6

		0x0010,	0x1000,		0x0011,	0x0038,		0x0012,	0x0000,		0x0000, 0x1000,
		0x001C,	0x0E75,		0x6000,	0x012C,		0x001C,	0x0E74,		0x0000,	0x0000,
		//----rf_parameter_RF_setting_7

        0x0013, 0xDF00,     0x0013, 0x8200,     0x0013, 0x4100,     0x0013, 0x0000,
        0x001B, 0xFC00,     0x001B, 0xB400,     0x001B, 0x5800,     0x001B, 0x1000,
        //---rf_parameter_rx_gain_table_8

		0x000E,	0xA430,		0x002A,	0xFFF0,		0x002B,	0x7800,		0x0035,	0x8C50,
		0x000E,	0x2430,
		//----rf_parameter_wifi_spur_9

		0xFFFF, 0xFFFF,     0xFFFF, 0xFFFF,     0xFFFF, 0xFFFF,     0xFFFF, 0xFFFF,
        0xFFFF, 0xFFFF,     0xFFFF, 0xFFFF,     0xFFFF, 0xFFFF,     0xFFFF, 0xFFFF,
        0xFFFF, 0xFFFF,     0xFFFF, 0xFFFF,     0xFFFF, 0xFFFF
    }
#elif defined(_IS_ASIC_) && defined(_RLE0380_MP_CHIP_)  /* RTL8723A */
        // EXPIRED: for RTL8723A
        removed for clean
#else
        !!!! ERROR: UNEXPECTED !!!!
#endif
#endif
};


#ifndef _IS_ASIC_
const OTP_RF_STRUCT otp_rf_str_data = {
#ifdef _TWO_MAC_FPGA_RF_PARAMETER_
        //the following parameters are used for 2MAC (PI), simulated by chinwen
        //**************for Modem ini Settings********
        //*     size: ?? Bytes                        *
        //*********************************************
        0x0000,    //UINT16 mdm_ini_0x00; //Reg00 //0x6f76
        0x4145,    //UINT16 mdm_ini_0x01;Reg02 //[bit4: 1 for si, 0 for pi]
        0x94db,    //UINT16 mdm_ini_0x02; //Reg04 //0x94db
#ifndef _NEW_MODEM_DESIGN_
        0x306a,    //UINT16 mdm_ini_0x03; //Reg06 //0x309a
#else
        0x3063,    //UINT16 mdm_ini_0x03; //Reg06 //0x309a
#endif
    //    0x321e,    //UINT16 mdm_ini_0x04; //Reg08 //0x361e
        0x361e,    //UINT16 mdm_ini_0x04; //Reg08 //0x361e
    //    0x00b0,    //UINT16 mdm_ini_0x05; //Reg0A //0x00b0
        0x0063,    //UINT16 mdm_ini_0x05; //Reg0A //0x00b0
        0x3322,    //UINT16 mdm_ini_0x06; //Reg0C //0x3322
        0x0333,    //UINT16 mdm_ini_0x07; //Reg0E //0x1172
        0x569c,    //UINT16 mdm_ini_0x08; //Reg10 //0x569e
        0x0f48,    //UINT16 mdm_ini_0x09; //Reg12 //0x0f48
    //    0x0000,    //UINT16 mdm_ini_0x0A; //Reg14 //0x0000
        0x1501,    //UINT16 mdm_ini_0x0A; //Reg14 //0x0000
    //    0x5a1e,    //UINT16 mdm_ini_0x0B; //Reg16 //0x5a0e
    //    0x5b9e,    //UINT16 mdm_ini_0x0B; //Reg16 //0x5a0e
    //    0x5b8e,    //UINT16 mdm_ini_0x0B; //Reg16 //0x5a0e
        0x409e,    //UINT16 mdm_ini_0x0B; //Reg16 //0x5a0e
        0x690f,    //UINT16 mdm_ini_0x0C; //Reg18 //0x690f
        0x0000,    //UINT16 mdm_ini_0x0D; //Reg1A //0x0000

        //===Pouter===
#ifndef LE_MODE_EN
        0x4400,    //UINT16 mdm_ini_0x16; //Reg2C //0x4400
#else
    //    (LE_AA_SYNC_THRESHOLD << 13) | 0x400,  //UINT16 mdm_ini_0x16; //Reg2C
                                               //LE_AA_SYNC_THRESHOLD = bit[15:13]
        0x4400,
#endif
        0x0ec0,    //UINT16 mdm_ini_0x17; //Reg2E //0x0ec0
        0x01d4,    //UINT16 mdm_ini_0x19; //Reg32 //0x01d4

        //===AFE===
    //    0x0030,    //UINT16 mdm_ini_0x1A; //Reg34 //0x0030
        0xe0f3,    //UINT16 mdm_ini_0x1A; //Reg34 //0x0030
    //    0x0001,    //UINT16 mdm_ini_0x1B; //Reg36 //0x0001
        0x7adb,    //UINT16 mdm_ini_0x1B; //Reg36 //0x0001
    //    0x85ad,    //UINT16 mdm_ini_0x1C; //Reg38 //0x85ad
        0xc068,    //UINT16 mdm_ini_0x1C; //Reg38 //0x85ad
    //    0x0014,    //UINT16 mdm_ini_0x1D; //Reg3A //0x0014
        0x6db9,    //UINT16 mdm_ini_0x1D; //Reg3A //0x0014

        //===Modem===
    //    0x2915,    //UINT16 mdm_ini_0x1E; //Reg3C //0x2115  //BT&GPS
        0x6015,    //UINT16 mdm_ini_0x1E; //Reg3C //0x2115  //BT&GPS
        0x5693,    //UINT16 mdm_ini_0x30; //Reg60 //0x5693
    //    0x0000,    //UINT16 mdm_ini_0x31; //Reg62 //0x0000  //AFE power saving
        0x0330,    //UINT16 mdm_ini_0x31; //Reg62 //0x0000  //AFE power saving
        0x6f73,    //UINT16 mdm_ini_0x32; //Reg64 //0x6f73  //(disable AGC->0x4f76) 0x6f76->enable AGC
        0x442f,    //UINT16 mdm_ini_0x33; //Reg66 //0x004f
    //    0x004f,    //UINT16 mdm_ini_0x33; //Reg66 //0x004f
#else /* else of #ifdef _TWO_MAC_FPGA_RF_PARAMETER_ */

#ifdef _NEW_MODEM_DESIGN_
        /* following parameters are used for 1 MAC with 0360 QA board (RF only mode)
           or 0379 RF board, used SI */
        //===Modem===
        0x0000,    //UINT16 mdm_ini_0x00; //Reg00 //0x0000
        0x4155,    //UINT16 mdm_ini_0x01; //Reg02 //[bit4: 1 for si, 0 for pi]
        0x94db,    //UINT16 mdm_ini_0x02; //Reg04 //0x94db
#ifndef _NEW_MODEM_DESIGN_
        0x306a,    //UINT16 mdm_ini_0x03; //Reg06 //0x309a
#else
        0x3063,    //UINT16 mdm_ini_0x03; //Reg06 //0x309a
#endif
        0x311e,    //UINT16 mdm_ini_0x04; //Reg08 //0x361e
  #ifdef _NEW_MODEM_DESIGN_AFTER_RTL8821B_TC_
        0x00b0 | 0x1C0,    //UINT16 mdm_ini_0x05; //Reg0A //0x00b0, yilinli, 2013/07/11, set lna_constraint = 7
  #else
        0x00b0,    //UINT16 mdm_ini_0x05; //Reg0A //0x00b0
  #endif
        0x3322,    //UINT16 mdm_ini_0x06; //Reg0C //0x3322
        0x1333,    //UINT16 mdm_ini_0x07; //Reg0E //0x1172
        0x569c,    //UINT16 mdm_ini_0x08; //Reg10 //0x569e
        0x0f48,    //UINT16 mdm_ini_0x09; //Reg12 //0x0f48
        0x0000,    //UINT16 mdm_ini_0x0A; //Reg14 //0x0000
  #ifdef _NEW_MODEM_DESIGN_AFTER_RTL8821B_TC_
    #ifdef _NEW_MODEM_DESIGN_AFTER_RTL8703B_
          0x5eaa,  // set tx_scaling_factor = 22/16
    #else
          (0x5a9e & 0xC1FF) | (0x0F<<9),    //UINT16 mdm_ini_0x0B; //Reg16 (BT=0.7 (f2/f1 ratio)), 2013/07/11, set tx_scaling_factor as 0x0F
    #endif
  #else
        0x5a9e,    //UINT16 mdm_ini_0x0B; //Reg16 (BT=0.7 (f2/f1 ratio))
  #endif
        0x690f,    //UINT16 mdm_ini_0x0C; //Reg18  (PGA_settingtime=3.5 us)
        0x0000,    //UINT16 mdm_ini_0x0D; //Reg1A //0x0000

        //===Pouter===
#ifndef LE_MODE_EN
        0x4400,    //UINT16 mdm_ini_0x16; //Reg2C //0x4400
#else
        (LE_AA_SYNC_THRESHOLD << 13) | 0x400,  //UINT16 mdm_ini_0x16; //Reg2C
                                               //LE_AA_SYNC_THRESHOLD = bit[15:13]
#endif
        0x0ec0,    //UINT16 mdm_ini_0x17; //Reg2E //0x0ec0
        0x01d4,    //UINT16 mdm_ini_0x19; //Reg32 //0x01d4

        //===AFE===
  #ifdef _NEW_MODEM_DESIGN_AFTER_RTL8821B_TC_
        0x0030,    //UINT16 mdm_ini_0x1A; //Reg34 //0x0030
        0x3322,    //UINT16 mdm_ini_0x1B; //Reg36 //0x0001
        0x85ad,    //UINT16 mdm_ini_0x1C; //Reg38 //0x85ad
        0x0333,    //UINT16 mdm_ini_0x1D; //Reg3A //0x0014
  #else
        0x0030,    //UINT16 mdm_ini_0x1A; //Reg34 //0x0030
        0x3322,    //UINT16 mdm_ini_0x1B; //Reg36 //0x0001 // yilinli, 2013/07/11, to be modified for U28 RFE
        0x85ad,    //UINT16 mdm_ini_0x1C; //Reg38 //0x85ad
        0x0333,    //UINT16 mdm_ini_0x1D; //Reg3A //0x0014 // yilinli, 2013/07/11, to be modified for U28 RFE
  #endif

        //===Modem===
        // dape test for AA th
#if defined (LE_AA_SYNC_THRESHOLD_ENABLE)
        0x7900 | _LE_SET_SYNC_MF_TH,    //UINT16 mdm_ini_0x1E; //Reg3C //0x6115  //BT&GPS
#else
        // threshold:30
        0x7100 | _LE_SET_SYNC_MF_TH,    //UINT16 mdm_ini_0x1E; //Reg3C //0x6115  //BT&GPS
#endif
        0x5693,    //UINT16 mdm_ini_0x30; //Reg60 //0x5693 (notch corner = 2)
        0x0000,    //UINT16 mdm_ini_0x31; //Reg62 //0x0000  //AFE power saving
        0x6f73,    //UINT16 mdm_ini_0x32; //Reg64 //0x6f73  //(disable AGC->0x4f76) 0x6f76->enable AGC
        0x404f,    //UINT16 mdm_ini_0x33; //Reg66
#else
#if defined(_USE_RLE0380_RF_) || defined(_USE_RLE0546_RF_)
         /* following parameters are used for 0380 test chip, used SI */
         //===Modem===
         0x0000,    //Reg00 //page selection
         0x4145,    //Reg02 //SI for FPGA, DPSK delay
         0x94db,    //Reg04
#ifndef _NEW_MODEM_DESIGN_
         0x3069,    //Reg06 //DPSK_delay
#else
         0x3063,    //Reg06 //DPSK_delay
#endif
         0x311e,    //Reg08 //0x321e, 0x361e
         0x00b0,    //Reg0a //for 3-DH5 dirty TX on error floor issue, 0x0063
         0x3322,    //Reg0c
         0x0333,    //Reg0e //0x0333
         0x569c,    //Reg10
         0x0f48,    //Reg12
         0x0000,    //Reg14
         0x5a2d,    //Reg16 //0x5a0e    //TX scaling factor=4/8
         0x290f,    //Reg18 // PGA_settling time=1.5us. 0x690f
         0x0000,    //Reg1a

         //===Pouter===
         0x4a98,    //Reg2c
         0x0ec0,    //Reg2e //0xeco -> disable Pouter, 0xfc0 -> enable Pouter
         0x01d4,    //Reg32

         //===AFE===
         0x264f,    //Reg34 //increasing ADC OP and Vpp, 0x254f
         0x0004,    //Reg36
         0xc0ab,    //Reg38
         0x3646,    //Reg3a

         //===Modem===
         0x3115,    //Reg3c //0x2115, BT&GPS
         0x5693,    //Reg60
         0x03af,    //Reg62 // AFE power saving
         0x6f73,    //Reg64 //ADC_backoff=-2dB, 0x6f73
                            //(disable AGC->0x4f76) 0x6f76->enable AGC
         0x298f,    //Reg66 // GPS 1.4M /0x018f    //0x004f    //0x042f
#else
        /* following parameters are used for 1 MAC with 0360 QA board (RF only mode)
           or 0379 RF board, used SI */
        //===Modem===
        0x0000,    //UINT16 mdm_ini_0x00; //Reg00 //0x0000
        0x4155,    //UINT16 mdm_ini_0x01; //Reg02 //[bit4: 1 for si, 0 for pi]
        0x94db,    //UINT16 mdm_ini_0x02; //Reg04 //0x94db
#ifndef _NEW_MODEM_DESIGN_
        0x306a,    //UINT16 mdm_ini_0x03; //Reg06 //0x309a
#else
        0x3063,    //UINT16 mdm_ini_0x03; //Reg06 //0x309a
#endif
        0x311e,    //UINT16 mdm_ini_0x04; //Reg08 //0x361e
        0x00b0,    //UINT16 mdm_ini_0x05; //Reg0A //0x00b0
        0x3322,    //UINT16 mdm_ini_0x06; //Reg0C //0x3322
        0x1333,    //UINT16 mdm_ini_0x07; //Reg0E //0x1172
        0x569c,    //UINT16 mdm_ini_0x08; //Reg10 //0x569e
        0x0f48,    //UINT16 mdm_ini_0x09; //Reg12 //0x0f48
        0x0000,    //UINT16 mdm_ini_0x0A; //Reg14 //0x0000

#ifdef _CCH_ADC20M_IONLY_
        0x5a9e,    //UINT16 mdm_ini_0x0B; //Reg16 (BT=0.7 (f2/f1 ratio))
#else
        0x5a1e,    //UINT16 mdm_ini_0x0B; //Reg16 (BT=0.7 (f2/f1 ratio))
#endif


#if 1
        0x690f,    //UINT16 mdm_ini_0x0C; //Reg18  (PGA_settingtime=3.5 us)
#else
        0x290f,    //UINT16 mdm_ini_0x0C; //Reg18  (PGA_settingtime=1.5 us)
#endif
        0x0000,    //UINT16 mdm_ini_0x0D; //Reg1A //0x0000

        //===Pouter===
#ifndef LE_MODE_EN
        0x4400,    //UINT16 mdm_ini_0x16; //Reg2C //0x4400
#else
        (LE_AA_SYNC_THRESHOLD << 13) | 0x400,  //UINT16 mdm_ini_0x16; //Reg2C
                                               //LE_AA_SYNC_THRESHOLD = bit[15:13]
#endif

        0x0ec0,    //UINT16 mdm_ini_0x17; //Reg2E //0x0ec0
        0x01d4,    //UINT16 mdm_ini_0x19; //Reg32 //0x01d4
        //===AFE===
        0x0030,    //UINT16 mdm_ini_0x1A; //Reg34 //0x0030
        0x0001,    //UINT16 mdm_ini_0x1B; //Reg36 //0x0001
        0x85ad,    //UINT16 mdm_ini_0x1C; //Reg38 //0x85ad
        0x0014,    //UINT16 mdm_ini_0x1D; //Reg3A //0x0014

        //===Modem===
        // dape test for AA th
#if defined (LE_AA_SYNC_THRESHOLD_ENABLE)
        0x7900 | _LE_SET_SYNC_MF_TH,    //UINT16 mdm_ini_0x1E; //Reg3C //0x6115  //BT&GPS
#else
        // threshold:30
        0x7100 | _LE_SET_SYNC_MF_TH,    //UINT16 mdm_ini_0x1E; //Reg3C //0x6115  //BT&GPS
#endif

        // _CCH_CONER_0_
        0x5693,    //UINT16 mdm_ini_0x30; //Reg60 //0x5693 (notch corner = 2)
        0x0000,    //UINT16 mdm_ini_0x31; //Reg62 //0x0000  //AFE power saving
        0x6f73,    //UINT16 mdm_ini_0x32; //Reg64 //0x6f73  //(disable AGC->0x4f76) 0x6f76->enable AGC
        // _CCH_JUMP_S1_B2_
        0x418f,    //UINT16 mdm_ini_0x33; //Reg66
#endif
#endif
#endif /* end if #ifdef _TWO_MAC_FPGA_RF_PARAMETER_ */

        //********for TX Gain Step Settings********
        //*                                       *
        //*****************************************
#if defined(_USE_RLE0380_RF_)
        /* 1M TxGain Table - (one level uses 8 bits) */
        ((0x2D << 8) | (0x0E << 0)),
        ((0x50 << 8) | (0x31 << 0)),
        ((0x90 << 8) | (0x6F << 0)),
        ((0xD1 << 8) | (0xD1 << 0)),

        /* 2M TxGain Table */
        ((0x0E << 8) | (0x0A << 0)),
        ((0x31 << 8) | (0x2D << 0)),
        ((0x6F << 8) | (0x50 << 0)),
        ((0x90 << 8) | (0x90 << 0)),

        /* 3M TxGain Table */
        ((0x0E << 8) | (0x0A << 0)),
        ((0x31 << 8) | (0x2D << 0)),
        ((0x6F << 8) | (0x50 << 0)),
        ((0x90 << 8) | (0x90 << 0)),
#else
  #if defined(_FPGA_WITH_RLE0546_RFE_)
        /* 1M TxGain Table - (one level uses 8 bits) */
        ((0x4A << 8) | (0x2D << 0)),
        ((0x6A << 8) | (0x4D << 0)),
        ((0x8C << 8) | (0x89 << 0)),
        ((0x8F << 8) | (0x8F << 0)),
        /* 2M TxGain Table */
        ((0x4A << 8) | (0x2D << 0)),
        ((0x6A << 8) | (0x4D << 0)),
        ((0x8C << 8) | (0x89 << 0)),
        ((0x8F << 8) | (0x8F << 0)),
        /* 3M TxGain Table */
        ((0x4A << 8) | (0x2D << 0)),
        ((0x6A << 8) | (0x4D << 0)),
        ((0x8C << 8) | (0x89 << 0)),
        ((0x8F << 8) | (0x8F << 0)),
  #elif defined(_FPGA_WITH_RLE0608C_RFE_)
        TBD
  #elif defined(_FPGA_WITH_RLE0550_RFE_)
        /* 1M TxGain Table - (one level uses 8 bits) */
        ((0x2A << 8) | (0x2A << 0)),
        ((0x2A << 8) | (0x2A << 0)),
        ((0x2A << 8) | (0x2A << 0)),
        ((0x2A << 8) | (0x2A << 0)),
        /* 2M TxGain Table */
        ((0x2A << 8) | (0x2A << 0)),
        ((0x2A << 8) | (0x2A << 0)),
        ((0x2A << 8) | (0x2A << 0)),
        ((0x2A << 8) | (0x2A << 0)),
        /* 3M TxGain Table */
        ((0x2A << 8) | (0x2A << 0)),
        ((0x2A << 8) | (0x2A << 0)),
        ((0x2A << 8) | (0x2A << 0)),
        ((0x2A << 8) | (0x2A << 0)),
  #elif defined(_FPGA_WITH_RLE0379_RFE_)

#ifdef _BRUCE_TEST_PLC_FIX_TX_GAIN
          /* 1M TxGain Table - (one level uses 8 bits) */
          ((0x45 << 9) | (0x25 << 1)),
          ((0x45 << 9) | (0x25 << 1)),
          ((0x45 << 9) | (0x25 << 1)),
          ((0x45 << 9) | (0x25 << 1)),

          /* 2M TxGain Table */
          ((0x27 << 9) | (0x23 << 1)),
          ((0x27 << 9) | (0x23 << 1)),
          ((0x27 << 9) | (0x23 << 1)),
          ((0x27 << 9) | (0x23 << 1)),

          /* 3M TxGain Table */
          ((0x27 << 9) | (0x23 << 1)),
          ((0x27 << 9) | (0x23 << 1)),
          ((0x27 << 9) | (0x23 << 1)),
          ((0x27 << 9) | (0x23 << 1)),

#else
        /* 1M TxGain Table - (one level uses 8 bits) */
        ((0x06 << 9) | (0x02 << 1)),
        ((0x17 << 9) | (0x14 << 1)),
        ((0x45 << 9) | (0x25 << 1)),
        ((0x55 << 9) | (0x55 << 1)),

        /* 2M TxGain Table */
        ((0x04 << 9) | (0x00 << 1)),
        ((0x15 << 9) | (0x07 << 1)),
        ((0x27 << 9) | (0x23 << 1)),
        ((0x47 << 9) | (0x47 << 1)),

        /* 3M TxGain Table */
        ((0x04 << 9) | (0x00 << 1)),
        ((0x15 << 9) | (0x07 << 1)),
        ((0x27 << 9) | (0x23 << 1)),
        ((0x47 << 9) | (0x47 << 1)),

#endif

  #else
        !!! ERROR, Undefined FPGA RFE Option !!!
  #endif
#endif

        //********for Modem rx tgc tbl Settings********
        //*     size: ?? Bytes                        *
        //*********************************************
#if defined(_USE_RLE0380_RF_) || defined(_USE_RLE0546_RF_)
        0x043f,
        0x043e,
        0x043d,
        0x043c,
        0x043b,
        0x043a,
        0x0439,
        0x0438,
        0x0437,
        0x0436,
        0x0435,
        0x0534,
        0x0633,
        0x0732,
        0x0831,
        0x0930,
        0x0a2f,
        0x0b2e,
        0x0c2d,     //0dBm
        0x0d2c,
        0x0e2b,
        0x0f2a,
        0x1029,
        0x4728,     //-10dBm
        0x4827,
        0x4926,
        0x4a25,
        0x4b24,
        0x4c23,     //-20dBm
        0x4d22,
        0x4e21,
        0x4f20,
        0x501f,
        0x511e,     //-30dBm
        0x521d,
        0x531c,
        0x541b,
        0x551a,
        0x5619,     //-40dBm
        0x5718,
        0x6617,
        0x6716,
        0x6815,
        0x6914,     //-50dBm
        0x6a13,
        0x6b12,
        0x6c11,
        0x6d10,
        0x6e0f,     //-60dBm
        0x6f0e,
        0x700d,
        0x710c,
        0x720b,
        0x730a,     //-70dBm
        0x7409,
        0x7508,
        0x7607,
        0x7706,
        0x7805,     //-80dBm
        0x7904,
        0x7a03,
        0x7b02,
        0x7c01,
        0x7d00,
#else /* EXPIRED by executing rtl8723_btrf_RxAGCTableInit(); instead of rtk_modem_rx_agc_tbl_init() */
       // _CCH_AGC_DEFAULT_
        0x053f,                 //UINT16 mdm_rxagc_tbl_00;
        0x053e,                 //UINT16 mdm_rxagc_tbl_01;
        0x053d,                 //UINT16 mdm_rxagc_tbl_02;
        0x053c,                 //UINT16 mdm_rxagc_tbl_03;
        0x053b,                 //UINT16 mdm_rxagc_tbl_04;
        0x053a,                 //UINT16 mdm_rxagc_tbl_05;
        0x0539,                 //UINT16 mdm_rxagc_tbl_06;
        0x0538,                 //UINT16 mdm_rxagc_tbl_07;
        0x0537,                 //UINT16 mdm_rxagc_tbl_08;
        0x0536,                 //UINT16 mdm_rxagc_tbl_09;
        0x0535,                 //UINT16 mdm_rxagc_tbl_10;
        0x0534,                 //UINT16 mdm_rxagc_tbl_11;
        0x0533,                 //UINT16 mdm_rxagc_tbl_12;
        0x0532,                 //UINT16 mdm_rxagc_tbl_13;
        0x0531,                 //UINT16 mdm_rxagc_tbl_14;
        0x0530,                 //UINT16 mdm_rxagc_tbl_15;
        0x052f,                 //UINT16 mdm_rxagc_tbl_16;
        0x052e,                 //UINT16 mdm_rxagc_tbl_17;
        0x052d,                 //UINT16 mdm_rxagc_tbl_18;//0 dBm
        0x052c,                 //UINT16 mdm_rxagc_tbl_19;
        0x062b,                 //UINT16 mdm_rxagc_tbl_20;
        0x402a,                 //UINT16 mdm_rxagc_tbl_21;
        0x4129,                 //UINT16 mdm_rxagc_tbl_22;
        0x4228,                 //UINT16 mdm_rxagc_tbl_23;//-10dBm
        0x4327,                 //UINT16 mdm_rxagc_tbl_24;
        0x4426,                 //UINT16 mdm_rxagc_tbl_25;
        0x4525,                 //UINT16 mdm_rxagc_tbl_26;
        0x4624,                 //UINT16 mdm_rxagc_tbl_27;
        0x4723,                 //UINT16 mdm_rxagc_tbl_28;//-20dBm
        0x4822,                 //UINT16 mdm_rxagc_tbl_29;
        0x4921,                 //UINT16 mdm_rxagc_tbl_30;
        0x4a20,                 //UINT16 mdm_rxagc_tbl_31;
        0x4b1f,                 //UINT16 mdm_rxagc_tbl_32;
        0x4c1e,                 //UINT16 mdm_rxagc_tbl_33;//-30dBm
        0x4d1d,                 //UINT16 mdm_rxagc_tbl_34;
        0x4e1c,                 //UINT16 mdm_rxagc_tbl_35;
        0x4f1b,                 //UINT16 mdm_rxagc_tbl_36;
        0x501a,                 //UINT16 mdm_rxagc_tbl_37;
        0x5119,                 //UINT16 mdm_rxagc_tbl_38;//-40dBm
        0x5218,                 //UINT16 mdm_rxagc_tbl_39;
        0x5317,                 //UINT16 mdm_rxagc_tbl_40;
        0x5416,                 //UINT16 mdm_rxagc_tbl_41;
        0x5515,                 //UINT16 mdm_rxagc_tbl_42;
        0x6614,                 //UINT16 mdm_rxagc_tbl_43;//-50dBm
        0x6713,                 //UINT16 mdm_rxagc_tbl_44;
        0x6812,                 //UINT16 mdm_rxagc_tbl_45;
        0x6911,                 //UINT16 mdm_rxagc_tbl_46;
        0x6a10,                 //UINT16 mdm_rxagc_tbl_47;
        0x6b0f,                 //UINT16 mdm_rxagc_tbl_48;//-60dBm
        0x6c0e,                 //UINT16 mdm_rxagc_tbl_49;
        0x6d0d,                 //UINT16 mdm_rxagc_tbl_50;
        0x6e0c,                 //UINT16 mdm_rxagc_tbl_51;
        0x6f0b,                 //UINT16 mdm_rxagc_tbl_52;
        0x700a,                 //UINT16 mdm_rxagc_tbl_53;//-70dBm
        0x7109,                 //UINT16 mdm_rxagc_tbl_54;
        0x7208,                 //UINT16 mdm_rxagc_tbl_55;
        0x7307,                 //UINT16 mdm_rxagc_tbl_56;
        0x7506,                 //UINT16 mdm_rxagc_tbl_57;
        0x7605,                 //UINT16 mdm_rxagc_tbl_58;//-80dBm
        0x7704,                 //UINT16 mdm_rxagc_tbl_59;
        0x7803,                 //UINT16 mdm_rxagc_tbl_60;
        0x7902,                 //UINT16 mdm_rxagc_tbl_61;
        0x7a01,                 //UINT16 mdm_rxagc_tbl_62;
        0x7c00,                 //UINT16 mdm_rxagc_tbl_63;
#endif

        //*************for RF init Settings************
        //*     size: 388 Bytes                        *
        //*********************************************
#if defined(_USE_RLE0380_RF_) || defined(_USE_RLE0546_RF_)
        76,
        0x0000,                  //UINT16  rf_ini_reserved;
        {// rf_ini_offset[128]
            0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
            0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F,
            0x10, 0x11, 0x16, 0x17, 0x18, 0x19, 0x1A, 0x1B,
            0x1C, 0x1D, 0x1E, 0x1F, 0x20, 0x21, 0x22, 0x23,
            0x24, 0x25, 0x26, 0x27, 0x28, 0x29, 0x2A, 0x2B,
            0x2C, 0x2D, 0x2E, 0x2F, 0x30, 0x31, 0x32, 0x33,
            0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0x3A, 0x3B,
            0x3C, 0x3D, 0x3E, 0x3F, 0x14, 0x14, 0x14, 0x18,
            0x2F, 0xFF, 0x3A, 0xFF, 0xFF, 0x28, 0xFF, 0xFF,
            0x03, 0x3C, 0x2F, 0x31, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
        },
        {// rf_ini_value[128]
            0x1000, 0x0000, 0x1001, 0x0000, 0x0000, 0x0000, 0xB000, 0x0000,
            0x01C0, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
            0x0000, 0x0000, 0x41C0, 0x4A08, 0x1600, 0x9557, 0x0020, 0xEA00,
            0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x9200, 0x0000, 0x3D4D,
            0x7F60, 0x7C01, 0x0000, 0x4C00, 0x2488, 0x4C84, 0x00D8, 0xE08B,
            0x052D, 0x5C48, 0x4400, 0x0020, 0x1A0D, 0x69C0, 0xAD20, 0xBF0C,
            0xAC31, 0xFFC0, 0x5BE3, 0xE830, 0x0000, 0x0000, 0x4400, 0x8060,
            0x0100, 0x0140, 0x0000, 0x0000, 0x4300, 0xB700, 0xDB00, 0x1640,
            0x0021, 0x01F4, 0xC400, 0x01F4, 0x01F4, 0x4000, 0x01F4, 0x01F4,
            0xD100, 0x0600, 0x0010, 0x4380, 0x0000, 0x0000, 0x0000, 0x0000,
            0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
            0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
            0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
            0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
            0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
            0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000
        }
#else
        74,
        0x0000,                  //UINT16  rf_ini_reserved;
        {
            0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
            0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F,
            0x10, 0x11, 0x16, 0x17, 0x18, 0x19, 0x1A, 0x1B,
            0x1C, 0x1D, 0x1E, 0x1F, 0x20, 0x21, 0x22, 0x23,
            0x24, 0x25, 0x26, 0x27, 0x28, 0x29, 0x2A, 0x2B,
            0x2C, 0x2D, 0x2E, 0x2F, 0x30, 0x31, 0x32, 0x33,
            0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0x3A, 0x3B,
            0x3C, 0x3D, 0x3E, 0x3F, 0x40, 0x41, 0x42, 0x14,
            0x14, 0x14, 0x14, 0x14, 0x14, 0x18, 0x2F, 0xFF,
            0x3A, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
        },
        {
            0x1000, 0x0000, 0x1001, 0x0000, 0xBBBC, 0x0000, 0xB000, 0x0000,
            0x0380, 0x8000, 0x2000, 0x249F, 0x249E, 0x24A0, 0x0156, 0x09C4,
            0x1780, 0x0000, 0x41C0, 0x4A08, 0x5600, 0x9555, 0x0020, 0xEA00,
            0xA54A, 0xF800, 0x64A8, 0x2C46, 0x07F0, 0x83E0, 0x0000, 0x854D,
            0x6C20, 0xA001, 0x4000, 0x4C00, 0x2488, 0x00E7, 0x0958, 0xE089,
            0x052D, 0x4C48, 0x4408, 0x0020, 0x1A0D, 0x69C0, 0xACA0, 0xBE0C,
            0xAC30,
#ifdef _DAPE_TEST_BOOT_0379_SETTLING_TIME
            0x2120,
#else
            0xFFC0,
#endif
            0x63EF, 0xE830, 0x0000, 0x0000, 0x0100, 0x8040,
            0x0100, 0x0140, 0x0000, 0x0000, 0xFFF6, 0x6DB2, 0x0400, 0x4900,
            0x6B00, 0x9100, 0xB300, 0xDB00, 0xF500, 0x5640, 0x0021, 0x0064,
            0x8100, 0x01F4, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
            0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
            0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
            0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
            0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
            0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
            0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000
        }
#endif
};
#endif

