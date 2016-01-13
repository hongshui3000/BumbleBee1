#ifndef _OTP_H_
#define _OTP_H_

#include <efuse_config.h>
#include <diag.h>

typedef struct otp_struct
{
    //**************for OTP managerment***************
    //*    size: 4 Bytes                             *
    //************************************************
    uint32_t EFUSE_parameter; //EFUSE[00~03]
                            //Format: SyntaxByte_VersionByte_Reserve0Byte_Reserve1Byte
                            //SyntaxByte: [0xA5=use OTP value] [others = use default value]
                            //VersionByte: OTP version
                            //in Main(), this value will be checked first



    //**************for SYS managerment***************
    //*    size: 54 Bytes                            *
    //************************************************
    uint32_t SYS_hci_uart_baudrate;       //EFUSE[04~07] 57600
    uint32_t rtl8723_data_uart_settings;  //EFUSE[08~0B] 0x19CAC558  (refer the structure "RLT8723_DATA_UART_SETTINGS_S")
    uint32_t rtl8723_data_uart_settings_2;//EFUSE[0C~0F] (refer the structure "RLT8723_DATA_UART_SETTINGS_2_S")
    uint32_t rtl8723_data_uart_settings_3;//EFUSE[10~13]  (refer the structure "RLT8723_DATA_UART_SETTINGS_3_S")

    uint32_t efuse_lps_setting_1_d32;     //EFUSE[14~17]
    uint32_t efuse_lps_setting_2_d32;     //EFUSE[18~1B]
    uint32_t efuse_lps_setting_3_d32;	    //EFUSE[1C~1F]
    uint8_t  efuse_lps_setting_4_d8;      //EFUSE[20]
    uint8_t  efuse_lps_setting_5_d8;      //EFUSE[21](refer the structure "EFUSE_LPS_SETTING_5_S")
    uint8_t  efuse_lps_setting_rsvd_d8[4];   //EFUSE[22~25]

    uint16_t lps_mode_max_time;           //EFUSE[26~27]

    uint16_t power_option;                //EFUSE[28~29] 0x2B5
    uint16_t power_seq_param;             //EFUSE[2A~2B] 0x00000000
    uint16_t general_control;             //EFUSE[2C~2D]

    union {
        uint8_t lpm_use_gdma_d8;          //EFUSE[2E]
        EFUSE_LPM_USE_GDMA lpm_use_gdma_s;
    };

    uint8_t  control_time_parameter;      //EFUSE[2F]

    uint8_t  efuse_pow_setting_1_d8;      //EFUSE[30] (refer the structure "EFUSE_POW_SETTING_1_S")
    uint8_t  efuse_pow_setting_2_d8;      //EFUSE[31] (refer the structure "EFUSE_POW_SETTING_2_S")
    uint8_t  efuse_pow_setting_3_d8;      //EFUSE[32]  (refer the structure "EFUSE_POW_SETTING_3_S")
    uint8_t  efuse_pow_setting_4_d8;      //EFUSE[33]  (refer the structure "EFUSE_POW_SETTING_4_S")

    uint8_t  efuse_sys_rsvd_d8[6];        //EFUSE[34~39]

    //**************for Bluetooth Para*************
    //*     size: 46 Bytes                        *
    //*********************************************
    uint8_t bt_bd_addr[6];                    //EFUSE[3A~3F]  0x7c, 0x77, 0xd3, 0x72, 0x02, 0x03
    uint8_t bt_master_piconet_fea[8];         //EFUSE[40~47]  0xff,0xff,0xff,0xfe,0x9B,0xff,0x79,0x83

    uint16_t bt_manufacturer_name;            //EFUSE[48~49] 0x000a(0xffff) 0x000a is CSR's name, and this value help us be recognized by win7

    uint16_t bt_min_lpm_threshold;            //EFUSE[4A~4B] 0x80
    uint16_t bt_deep_sleep_mode_threshold;    //EFUSE[4C~4D] 0xfffe

#ifdef _SUPPORT_CSB_RECEIVER_
    uint16_t bt_page_timeout;                 //EFUSE[4E~4F] 0x2000
#else
    uint16_t bt_rsvd_d16;                     //EFUSE[4E~4F] 0x2000
#endif

    uint16_t bt_priority_low;                 //EFUSE[50~51] 0x0000
    uint16_t bt_priority_high;                //EFUSE[52~53] 0x0000
    uint16_t bt_priority_3;                   //EFUSE[54~55] 0x0000 bit[7:0]: bt_priority_3
    uint16_t bt_priority_rsvd;                //EFUSE[56~57] 0x0000 bit[7:0]: bt_priority_rsvd

    uint32_t bt_func_support_policy;          //EFUSE[58~5B] 0x0000001F
                                            /* bit[0]: reserved
                                               bit[1]: is support BT4.1
                                               bit[2]: is support BT4.2
                                               bit[3]: IS_RF_INIT
                                               bit[4]: power saving enable
                                               bit[5]: test for BQB
                                               bit[31:6]: reserved
                                            */


    uint32_t bt_le_multistate_bm_ldw;         //EFUSE[5C~5F] 0x1FFFFFFF

    uint32_t bt_le_fw_policy;                 //EFUSE[60~63] 0x469C4018
                                        /* bit[0]: IS_LE_CONN_INTV_AUTO_UPDATE_EN = 0
                                           bit[1]: IS_LE_CONN_INTV_DEFAULT_EN = 0
                                           bit[13:2]: LE_CONN_INTV_DEFAULT_VALUE = 6
                                           bit[14]: IS_LE_AUTO_CONN_UPDATE_EN = 1
                                           bit[18:15]: rsvd
                                           bit[19]: LE_LEGACY_COMPENSATE_SLOT = 1
                                           bit[24:20]: rsvd
                                           bit[25]: LE_EN_WAKE_FROM_SL_RX;
                                           bit[26]: LE_EN_WAKE_FROM_SLAVE_LATENCY;
                                           bit[28:27]: LE_MAX_NUM_OF_ADV_REPORT_OPTION
                                           bit[31:29]: LEGACY_INQUIRY_RESERVED_12SLOT
                                           */

    uint32_t bt_func_support_policy_ext;      //EFUSE[64~67]
                                        /* bit[0]: is support virtual HCI */
                                        /* bit[2:1]: support the number of le links (0~3)+1
                                           bit[3]: reserved
                                           bit[6:4]: support the number of white list unit (2^n, n <= 5)
                                           bit[9:7]: support the number of black list unit (2^n, n <= 5)
                                           bit[10]: reserved
                                           bit[11]: NO_CARE_SYNC_TRAIN_LENGTH ?
                                           bit[12]: IS_NO_LE_TX_THRESHOLD_FOR_MULTI_CE ?
                                           bit[13]: IS_NO_LE_RX_THRESHOLD_FOR_MULTI_CE ?
                                           bit[14]: reserved
                                           bit[15]: SHORTEN_MASTER_SNIFF_TIMEOUT_VAL ?
                                           bit[16]: IS_EVENT_REORDER_FOR_ANY_INTERFACE ?
                                           bit[17]: LE_AUTO_UPDT_WHEN_SLOT_CONFLICT_AFTER_CONN_UPDT ?
                                           bit[21:18]: g_le_use_interval_slot_pair
                                           bit[24:22]: support max segment number grade of bzdma le acl data for one link
                                                      (actual segment number = 2^seg_num_grade, valid range of seg_num = 0~4)
                                           bit[28:25]: support max fragment number of bzdma le acl data for one segment
                                                       (valid range = 2,4,6,8, ..., 14)
                                           bit[30:29]: reserved
                                           bit[31]: eFlash turn on/off BT_RF when writing & erasing ( 0: new way, 1: old way )
                                         */

   //*********for BW MAC/RF Timing Settings *******
   //*     size: 24 Bytes                        *
   //*********************************************
   uint8_t bw_rf_radio_sys_clk_sel;          //EFUSE[68] MT:0x17 RT:0x09
   uint8_t bw_rf_osc_delay;                  //EFUSE[69] 8
   uint8_t bw_rf_low_clk_frac;               //EFUSE[6A] 0x1 //32K(1) or 32.768K(0)
   uint8_t bw_rf_reserved[1];                //EFUSE[6B]
   uint16_t bw_rf_delay_vals[10];            //EFUSE[6C~7F] MT: 0xD6D3, 0xE0DD, 0x8989, 0x8989, 0x135, 0x140, 0x140, 0x13B, 0x933C, 0xFFFF
                                           //RT: 0xC0B8, 0xE0D3, 0x8989, 0x8989, 0x116, 0x140, 0x140, 0x142, 0x933C, 0xFFFF

   //********for LE Controller Timing Settings******
   //*     size: 14 Bytes                         *
   //*********************************************
   uint16_t le_ce_early_int_time;            //EFUSE[80~81] 0x012C (300 us)
   uint16_t le_trx_on_delay;                 //EFUSE[82~83] 0xDCDC
   uint16_t le_rx_turnaround_delay;          //EFUSE[84~85] 0x328A
   uint16_t le_tx_turnaround_delay;          //EFUSE[86~87] 0x1083 (txon_early=16 tx2rx_delay=131)
   uint16_t le_ce_rx_timeout_delay;          //EFUSE[88~89] 0x0164
   uint16_t le_rx_search_timeout_value;      //EFUSE[8A~8B] 0x01F0
   uint16_t le_clock_compensate;             //EFUSE[8C~8D] 0x0147

    //********* RF Cal Value *************
    //*     size: 18 Bytes               *
    //************************************
    uint8_t EFuse_PHYInit_TxScaleFactor;      //EFUSE[8E]
    uint8_t EFuse_PHYInit_TxDACCurrent;       //EFUSE[8F]
    uint8_t EFuse_TxPowerTrack_En;            //EFUSE[90]
    uint8_t EFuse_ThermalDefault;             //EFUSE[91]
    uint8_t EFuse_ThermalUpdateInterval;      //EFUSE[92]
    uint8_t EFuse_CFOTrack_En;                //EFUSE[93]
    uint8_t EFuse_Cap_Adj;                    //EFUSE[94]
    uint8_t EFuse_Cal_Rsvd[11];               //EFUSE[95~9F]

    //********* AON Init Value ***************
    //*     size: 20 Bytes                    *
    //****************************************
    union {
        uint32_t reg_aon_powon_d32;           //EFUSE[A0~A3]
        AON_REG_POWON reg_aon_powon_s;
    };
    union {
        uint32_t reg_aon_powdown_d32[4];      //EFUSE[A4~B3]
        AON_REG_POWDOWN reg_aon_powdown_s[4];
    };

    //********** BT_EFUSE_UPPERSTACK_CFG **********
    //*     size: 36 Bytes                         *
    //*********************************************
    union {
        uint8_t gEfuse_UpperStack_d8[36];      //EFUSE[B4~D7]
        BT_EFUSE_UPPERSTACK_CFG gEfuse_UpperStack_s;
    };

#ifdef KEEP_ACUT_EFUSE
    //********** BT_EFUSE_LTP  ********************
    //*     size: 24 Bytes                        *
    //*********************************************
    union {
        uint8_t gEfuse_Ltp_d8 [24];           //EFUSE[D8~EF]
        BT_EFUSE_LTP gEfuse_Ltp_s;
    };
#else
    //********** Reserved  ********************
    //*     size: 24 Bytes                        *
    //*********************************************
    uint8_t reserved_1[24];      //EFUSE[D8~EF]
#endif

    //********** BT_EFUSE_LOG ********************
    //*     size: 4 Byte                          *
    //*********************************************
    union {
    	uint8_t gEfuse_Log_d8[4];             //EFUSE[F0~F3]
     	BT_EFUSE_LOG gEfuse_Log_s;
     };

    //********** BT_EFUSE_STACK_BQB   *************
    //*     size: 1 Byte                          *
    //*********************************************
    union {
        uint8_t gEfuse_Bqb_d8;                //EFUSE[F4]
        BT_EFUSE_STACK_BQB gEfuse_Bqb_s;
    };

#ifdef KEEP_ACUT_EFUSE
    //********** BT_EFUSE_DTM    ******************
    //*     size: 1 Byte                          *
    //*********************************************
    union {
        uint8_t gEfuse_Dtm_d8;                //EFUSE[F5]
        BT_EFUSE_DTM gEfuse_Dtm_s ;
    };
#else
    uint8_t reserved_2;                   //EFUSE[F5]
#endif

    //********** EFUSE_OS_CONFIG  *****************
    //*     size: 2 Byte                          *
    //*********************************************
    union {
        uint8_t gEfuse_OS_d8[2];              //EFUSE[F6~F7]
        EFUSE_OS_CONFIG gEfuse_OS_s;
    };

    //****** CPU_CLK_SETTING and SWD ctrl *********
    //*     size: 1 Byte                          *
    //*********************************************
    union {
        uint8_t gEfuse_mis_d8;                //EFUSE[F8]
        EFUSE_MISC_CONFIG gEfuse_MISC_s;
    };

    //********** queue size ******************
    //      size: 9 Bytes
    //*********************************************
#ifdef KEEP_ACUT_EFUSE
    uint8_t upper_stack_to_app_queue_size;      //EFUSE[F9]
    uint8_t app_to_upper_stack_queue_size;      //EFUSE[FA]
    uint8_t IO_driver_to_app_queue_size;        //EFUSE[FB]
    uint8_t app_to_IO_driver_queue_size;        //EFUSE[FC]
    uint8_t IO_driver_queue_size;               //EFUSE[FD]
    uint8_t IO_driver_event_queue_size;         //EFUSE[FE]
    uint8_t app_event_queue_size;               //EFUSE[FF]
#else
    //********** Reserved ******************
    //      size: 7 Bytes
    //*********************************************
    uint8_t hw_timer0_period;                   //EFUSE[F9]    
    uint8_t reserved_3[6];                      //EFUSE[FA~FF]
#endif
    uint8_t timer_queue_length;                 //EFUSE[100]
    uint8_t queue_size_reserved_1;              //EFUSE[101]

    //********** task stack size ******************
    //       size: 14 Bytes
    //*********************************************
    uint16_t lower_stack_task_stack_size;     //EFUSE[102~103]
    uint16_t upper_stack_task_stack_size;     //EFUSE[104~105]
    uint16_t idle_task_stack_size;            //EFUSE[106~107]
    uint16_t timer_task_stack_size;           //EFUSE[108~109]
    uint16_t dfu_task_stack_size;             //EFUSE[10A~10B]
#ifdef KEEP_ACUT_EFUSE
    uint16_t driver_task_stack_size;          //EFUSE[10C~10D]
    uint16_t dtm_task_stack_size;             //EFUSE[10E~10F]
#else
    uint8_t  lower_stack_task_priority;       //EFUSE[10C]
    uint8_t  upper_stack_task_priority;       //EFUSE[10D]
    uint8_t  timer_task_priority;             //EFUSE[10E]
    uint8_t  dfu_task_priority;               //EFUSE[10F]
#endif

    //********** debug setting ******************
    //      size: 16 Bytes
    //*********************************************
    uint32_t debug_setting[LEVEL_NUMs];       //EFUSE[110~11F]

#ifdef CONFIG_EFUSE_VENDER_KEY
    uint8_t vender_key[32];        //EFUSE[120~13F]
    union {
        uint8_t vender_key_cfg_d8;                //EFUSE[140]
        VENDER_KEY_CFG vender_key_cfg_s;
    };

    uint8_t reserved_6[3];          //EFUSE[141~143]
#else
    uint8_t reserved_6[36];         //EFUSE[120~143]
#endif
    uint8_t gpio_pull_up[5];        //EFUSE[144~148]
    uint8_t gpio_pull_down[5];      //EFUSE[149~153]

} OTP_STRUCT;

typedef struct otp_rf_struct
{
    //**** for RF Settings - Tx Gain Table***********
    //*      size: 40 Bytes                        *
    //*********************************************
    uint8_t EFuse_PHYInit_TxGainTable[32];    //offset[00~1F]
    uint8_t EFuse_PHYInit_MaxTxIndex;         //offset[20]
    uint8_t EFuse_PHYInit_MaxTxGain1M;        //offset[21]
    uint8_t EFuse_PHYInit_MaxTxGain2M;        //offset[22]
    uint8_t EFuse_PHYInit_MaxTxGain3M;        //offset[23]
    uint8_t EFuse_PHYInit_MaxTxGainLE;        //offset[24]
    uint8_t EFuse_PHYInit_TxGainStep;         //offset[25]

    //**** for RF Settings - Rx Gain Table***********
    //*      size: 10 Bytes                        *
    //*********************************************
    uint16_t EFuse_PHYInit_Gain0Stop;         //offset[26~27]; also refer to efuse_modem_rssi0_pin_dBm //
    uint16_t EFuse_PHYInit_Gain0Start;        //offset[28~29]
    uint16_t EFuse_PHYInit_Gain1Start;        //offset[2A~2B]
    uint16_t EFuse_PHYInit_Gain2Start;        //offset[2C~2D]
    uint16_t EFuse_PHYInit_Gain3Start;        //offset[2E~2F]
    uint16_t EFuse_PHYInit_Gain4Start;        //offset[30~31]
    uint16_t EFuse_PHYInit_Gain5Start;        //offset[32~33]
    uint16_t EFuse_PHYInit_Gain6Start;        //offset[34~35]
    uint16_t EFuse_PHYInit_Gain7Start;        //offset[36~37]

    //**** for RF Settings - Tx Power Tracking ******
    //*      size: 16 Bytes                        *
    //*********************************************
    uint8_t EFuse_TxPowerTrack_ThermalDelta;  //offset[38]
    uint8_t EFuse_TxPowerTrack_TxGain[17];    //offset[39~49]

    //**** for RF Settings - CFO (Center Frequency Offset) ****
    //*                size: 14 Bytes                        *
    //*******************************************************
    uint8_t EFuse_CFOTrack_ThermalDelta;      //offset[4A]
    uint8_t EFuse_CFOTrack_Cap[17];           //offset[4B~5B]

    //********** for RF Settings - LOK Tracking *****
    //*      size: 54 Bytes                        *
    //*********************************************
    uint8_t efuse_modem_setting_1_d8;         //offset[5C]  (refer the structure "EFUSE_MODEM_SETTING_1_S")
    int8_t efuse_modem_rssi0_pin_dBm;         //offset[5D] -90 means -90dBm

#ifdef _IS_ASIC_
    //*** for PHY Init (RF/Modem/Bluewiz/Vendor)*****
    //*       size:  4N Bytes                      *
    //*********************************************
#ifdef _OTP_ADD_SYS_INIT_PARAM_
    uint16_t sys_init_param[64];             //offset[2E~AD]
    uint16_t phy_init_param[192];             //offset[AE~25D]
#else
    uint16_t phy_init_param[256];             //offset[5E~25D]
#endif
#else

    //**************for Modem ini Settings********
    //*     size: 76 Bytes                        *
    //*********************************************
    uint16_t mdm_ini_0x00;                    //0x6f76
    uint16_t mdm_ini_0x01;                    //[0x4115 for FPGA(si)]  [0x4105 for ASIC(pi)]
    uint16_t mdm_ini_0x02;                    //0x94db
    uint16_t mdm_ini_0x03;                    //0x309a
    uint16_t mdm_ini_0x04;                    //0x361f
    uint16_t mdm_ini_0x05;                    //0x00b0
    uint16_t mdm_ini_0x06;                    //0x4912
    uint16_t mdm_ini_0x07;                    //0x13f2
    uint16_t mdm_ini_0x08;                    //0x569e
    uint16_t mdm_ini_0x09;                    //0x0f48
    uint16_t mdm_ini_0x0A;                    //0x0000
    uint16_t mdm_ini_0x0B;                    //0x5a32
    uint16_t mdm_ini_0x0C;                    //0x690B
    uint16_t mdm_ini_0x0D;                    //0x0000
    uint16_t mdm_ini_0x16;                    //0x4400
    uint16_t mdm_ini_0x17;                    //0x0cc0
    uint16_t mdm_ini_0x19;                    //0x01d4
    uint16_t mdm_ini_0x1A;                    //0x0030
    uint16_t mdm_ini_0x1B;                    //0x0001
    uint16_t mdm_ini_0x1C;                    //0x034d
    uint16_t mdm_ini_0x1D;                    //0x2521
    uint16_t mdm_ini_0x1E;                    //0x00e9
    uint16_t mdm_ini_0x30;                    //0x5693
    uint16_t mdm_ini_0x31;                    //0x0000
    uint16_t mdm_ini_0x32;                    //0x6f73
    uint16_t mdm_ini_0x33;                    //0x044f

    uint16_t mdm_ini_BW_0x136;                //0x8984 (0x8404)
    uint16_t mdm_ini_BW_0x138;                //0x0628 (0x8485)
    uint16_t mdm_ini_BW_0x13a;                //0x522c (0x11B2)
    uint16_t mdm_ini_BW_0x13c;                //0x008b (0x0091)
    uint16_t mdm_ini_BW_0x1bc;                //0x8404
    uint16_t mdm_ini_BW_0x1be;                //0x8485
    uint16_t mdm_ini_BW_0x1c0;                //0x11B2
    uint16_t mdm_ini_BW_0x1c2;                //0x0091
    uint16_t mdm_ini_BW_0x1c4;                //0x8404
    uint16_t mdm_ini_BW_0x1c6;                //0x8485
    uint16_t mdm_ini_BW_0x1c8;                //0x11B2
    uint16_t mdm_ini_BW_0x1ca;                //0x0091

    //********for Modem rx tgc tbl Settings********
    //*     size: 128 Bytes                        *
    //*********************************************
    uint16_t mdm_rxagc_tbl_00;                //0x0e3f
    uint16_t mdm_rxagc_tbl_01;                //0x0e3e
    uint16_t mdm_rxagc_tbl_02;                //0x0e3d
    uint16_t mdm_rxagc_tbl_03;                //0x0e3c
    uint16_t mdm_rxagc_tbl_04;                //0x0e3b
    uint16_t mdm_rxagc_tbl_05;                //0x0e3a
    uint16_t mdm_rxagc_tbl_06;                //0x0e39
    uint16_t mdm_rxagc_tbl_07;                //0x0e38
    uint16_t mdm_rxagc_tbl_08;                //0x0e37
    uint16_t mdm_rxagc_tbl_09;                //0x0e36
    uint16_t mdm_rxagc_tbl_10;                //0x0e35
    uint16_t mdm_rxagc_tbl_11;                //0x0e34
    uint16_t mdm_rxagc_tbl_12;                //0x0e33
    uint16_t mdm_rxagc_tbl_13;                //0x0e32
    uint16_t mdm_rxagc_tbl_14;                //0x0e31
    uint16_t mdm_rxagc_tbl_15;                //0x0e30
    uint16_t mdm_rxagc_tbl_16;                //0x0f2f
    uint16_t mdm_rxagc_tbl_17;                //0x102e
    uint16_t mdm_rxagc_tbl_18;                //0x112d
    uint16_t mdm_rxagc_tbl_19;                //0x402c
    uint16_t mdm_rxagc_tbl_20;                //0x412b
    uint16_t mdm_rxagc_tbl_21;                //0x422a
    uint16_t mdm_rxagc_tbl_22;                //0x4329
    uint16_t mdm_rxagc_tbl_23;                //0x4428
    uint16_t mdm_rxagc_tbl_24;                //0x4527
    uint16_t mdm_rxagc_tbl_25;                //0x4626
    uint16_t mdm_rxagc_tbl_26;                //0x4725
    uint16_t mdm_rxagc_tbl_27;                //0x4824
    uint16_t mdm_rxagc_tbl_28;                //0x4923
    uint16_t mdm_rxagc_tbl_29;                //0x4a22
    uint16_t mdm_rxagc_tbl_30;                //0x4b21
    uint16_t mdm_rxagc_tbl_31;                //0x4c20
    uint16_t mdm_rxagc_tbl_32;                //0x4d1f
    uint16_t mdm_rxagc_tbl_33;                //0x4e1e
    uint16_t mdm_rxagc_tbl_34;                //0x4f1d
    uint16_t mdm_rxagc_tbl_35;                //0x501c
    uint16_t mdm_rxagc_tbl_36;                //0x511b
    uint16_t mdm_rxagc_tbl_37;                //0x521a
    uint16_t mdm_rxagc_tbl_38;                //0x6219
    uint16_t mdm_rxagc_tbl_39;                //0x6318
    uint16_t mdm_rxagc_tbl_40;                //0x6417
    uint16_t mdm_rxagc_tbl_41;                //0x6516
    uint16_t mdm_rxagc_tbl_42;                //0x6615
    uint16_t mdm_rxagc_tbl_43;                //0x6714
    uint16_t mdm_rxagc_tbl_44;                //0x6813
    uint16_t mdm_rxagc_tbl_45;                //0x6912
    uint16_t mdm_rxagc_tbl_46;                //0x6a11
    uint16_t mdm_rxagc_tbl_47;                //0x6b10
    uint16_t mdm_rxagc_tbl_48;                //0x6c0f
    uint16_t mdm_rxagc_tbl_49;                //0x6d0e
    uint16_t mdm_rxagc_tbl_50;                //0x6e0d
    uint16_t mdm_rxagc_tbl_51;                //0x6f0c
    uint16_t mdm_rxagc_tbl_52;                //0x700b
    uint16_t mdm_rxagc_tbl_53;                //0x710a
    uint16_t mdm_rxagc_tbl_54;                //0x7209
    uint16_t mdm_rxagc_tbl_55;                //0x7308
    uint16_t mdm_rxagc_tbl_56;                //0x7407
    uint16_t mdm_rxagc_tbl_57;                //0x7506
    uint16_t mdm_rxagc_tbl_58;                //0x7605
    uint16_t mdm_rxagc_tbl_59;                //0x7704
    uint16_t mdm_rxagc_tbl_60;                //0x7803
    uint16_t mdm_rxagc_tbl_61;                //0x7902
    uint16_t mdm_rxagc_tbl_62;                //0x7a01
    uint16_t mdm_rxagc_tbl_63;                //0x7b00

    //*************for RF init Settings************
    //*     size: 388 Bytes                        *
    //*********************************************
    uint16_t rf_ini_num;
    uint16_t rf_ini_reserved;
    uint8_t rf_ini_offset[128];
    uint16_t rf_ini_value[128];
#endif
}
OTP_RF_STRUCT;


extern OTP_STRUCT otp_str_data;
extern const OTP_RF_STRUCT otp_rf_str_data;
extern OTP_RF_STRUCT *p_otp_rf_str_data;

//20120109 morgan add
extern uint16_t fw_lmp_sub_version; /* can modify it via patch */
extern uint16_t fw_hci_sub_version; /* can modify it via patch */

/*-------------------------------------------------------------------*/
/* All Macro Definitions about "otp_str_data.bt_le_fw_policy"        */
/*-------------------------------------------------------------------*/
#define IS_LE_CONN_INTV_AUTO_UPDATE_EN   (otp_str_data.bt_le_fw_policy & 0x01)         // bit 0
#define IS_LE_CONN_INTV_DEFAULT_EN       (otp_str_data.bt_le_fw_policy & 0x02)         // bit 1
#define LE_CONN_INTV_DEFAULT_VALUE       ((otp_str_data.bt_le_fw_policy >> 2) & 0xFFF) // bit 2~13
#define IS_LE_AUTO_CONN_UPDATE_EN        (otp_str_data.bt_le_fw_policy & BIT14)        // bit 14
#define LE_LEGACY_COMPENSATE_SLOT        ((otp_str_data.bt_le_fw_policy & BIT19) >> 19)// bit 19
#define LE_EN_WAKE_FROM_SL_RX            ((otp_str_data.bt_le_fw_policy & BIT25) >> 25)// bit 25
#define LE_EN_WAKE_FROM_SLAVE_LATENCY    ((otp_str_data.bt_le_fw_policy & BIT26) >> 26)// bit 26
#define LE_MAX_NUM_OF_ADV_REPORT_OPTION  ((otp_str_data.bt_le_fw_policy >> 27) & 0x03)// bit 27~28
#ifdef _DAPE_TEST_NEW_HW_LE_UNCONN_STATE_HIGHER_THAN_LEGACY_UNCONN
/* there is not enough efuse, so a uint is 12 slot. ex: if LEGACY_INQUIRY_RESERVED_12SLOT = 2,
   then we should reserve 12*2 = 24 slots for legacy inquiry & page. */
#define LEGACY_INQUIRY_RESERVED_12SLOT        ((otp_str_data.bt_le_fw_policy >> 29) & 0x07)// bit 29~31
#else
#define LEGACY_INQUIRY_RESERVED_12SLOT         12
#endif

/*-------------------------------------------------------------------*/
/* All Macro Definitions about "otp_str_data.bt_func_support_policy" */
/*-------------------------------------------------------------------*/
#define IS_BT40                 TRUE

#define IS_BT41                 (otp_str_data.bt_func_support_policy & 0x0002)
#define IS_BT42                 (otp_str_data.bt_func_support_policy & 0x0004)

#ifdef _IS_ASIC_
#define IS_RF_INIT              (otp_str_data.bt_func_support_policy & 0x0008)
#else
#define IS_RF_INIT              TRUE
#endif

#define IS_ENABLE_POWER_SAVE    (otp_str_data.bt_func_support_policy & 0x0010)
#ifdef _IN_BQB_
#define IS_USE_FOR_BQB          TRUE
#else
#define IS_USE_FOR_BQB          (otp_str_data.bt_func_support_policy & 0x0020)
#endif

/*-------------------------------------------------------------------*/
/* All Macro Definitions about "otp_str_data.bt_priority_3"          */
/*-------------------------------------------------------------------*/
#define IS_HARDWARE_ERR_EVENT_TO_HOST_WHEN_HOST_ACL (otp_str_data.bt_priority_3 & BIT8)
#define IS_EARLY_TX_TOGGLE      (otp_str_data.bt_priority_3 & BIT9)


/*----------------------------------------------------------------------*/
/* All Macro Definitions about "otp_str_data.bt_func_support_policy_ext"*/
/*----------------------------------------------------------------------*/
#define IS_USE_VHCI         (otp_str_data.bt_func_support_policy_ext & BIT0)
#ifdef _BEE_LE_CONFIG_LOWER_STACK_RESOURCE_
#define LE_SUPPORTED_LINKS    (((otp_str_data.bt_func_support_policy_ext >> 1) & 0x03) +1)
#else
#define LE_SUPPORTED_LINKS      4
#endif
#ifdef _NEW_BZDMA_FROM_V8_
#define IS_TOGGLE_LL_DATA_CMD   FALSE
#else
#define IS_TOGGLE_LL_DATA_CMD   (otp_str_data.bt_func_support_policy_ext & BIT3)
#endif

#ifdef _BEE_LE_CONFIG_LOWER_STACK_RESOURCE_
#define LE_SUPPORTED_WHITE_LIST_ORDER     ((otp_str_data.bt_func_support_policy_ext >> 4) & 0x07)
#else
#define LE_SUPPORTED_WHITE_LIST_ORDER      5
#endif

#ifdef _BEE_LE_CONFIG_LOWER_STACK_RESOURCE_
#define LE_SUPPORTED_BLACK_LIST_ORDER     ((otp_str_data.bt_func_support_policy_ext >> 7) & 0x07)
#else
#define LE_SUPPORTED_BLACK_LIST_ORDER      5
#endif

#ifdef _SUPPORT_CSB_RECEIVER_
#ifdef _DONT_CARE_SYNC_TRAIN_LENGTH_FOR_BRCM
#define NO_CARE_SYNC_TRAIN_LENGTH        (otp_str_data.bt_func_support_policy_ext & BIT11)
#else
#define NO_CARE_SYNC_TRAIN_LENGTH        FALSE
#endif
#else
#define NO_CARE_SYNC_TRAIN_LENGTH        FALSE
#endif
#ifndef _LE_NO_TRX_THRESHOLD_INT_FOR_MULTI_CE_OPTION_
#define IS_NO_LE_TX_THRESHOLD_FOR_MULTI_CE       FALSE
#define IS_NO_LE_RX_THRESHOLD_FOR_MULTI_CE       FALSE
#else
#define IS_NO_LE_TX_THRESHOLD_FOR_MULTI_CE       (otp_str_data.bt_func_support_policy_ext & BIT12)
#define IS_NO_LE_RX_THRESHOLD_FOR_MULTI_CE       (otp_str_data.bt_func_support_policy_ext & BIT13)
#endif

#define SHORTEN_MASTER_SNIFF_TIMEOUT_VAL         (otp_str_data.bt_func_support_policy_ext & BIT15)
#ifdef _SUPPORT_CSB_RECEIVER_
#define IS_SUPPORT_3DG                           (otp_str_data.bt_func_support_policy_ext & BIT16)
#else
#define IS_SUPPORT_3DG                            FALSE
#endif

#define LE_AUTO_UPDT_WHEN_SLOT_CONFLICT_AFTER_CONN_UPDT (otp_str_data.bt_func_support_policy_ext & BIT17)

#define NEW_V8_BZDMA_SUPPORT_MAX_SEGNUM_GRADE_IN_ONE_LINK  MIN(4, ((otp_str_data.bt_func_support_policy_ext >> 22) & 0x07))
#define NEW_V8_BZDMA_SUPPORT_MAX_FRAGNUM_IN_ONE_SEGMENT ((otp_str_data.bt_func_support_policy_ext >> 25) & 0x0f)

#define IS_USE_BTON_WDG_TIMER                    FALSE


#endif//_OTP_H_

