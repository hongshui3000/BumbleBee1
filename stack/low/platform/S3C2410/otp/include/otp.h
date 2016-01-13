#ifndef _OTP_H_
#define _OTP_H_

#include "platform.h"

typedef struct otp_struct
{
    //**************for OTP managerment***************
    //*    size: 4 Bytes                             *
    //************************************************
    UINT32 EFUSE_parameter; //EFUSE[00~03] 
                            //Format: SyntaxByte_VersionByte_Reserve0Byte_Reserve1Byte
                            //SyntaxByte: [0xA5=use OTP value] [others = use default value]
                            //VersionByte: OTP version
                            //in Main(), this value will be checked first



    //**************for SYS managerment***************
    //*    size: 12 Bytes                             *
    //************************************************
    UINT32 crystal_clk;                 //EFUSE[04~07] 40000000
    UINT32 SYS_log_uart_baudrate;       //EFUSE[08~0B] 115200
    UINT32 SYS_hci_uart_baudrate;       //EFUSE[0C~0F] 57600
    UINT32 rtl8723_data_uart_settings;  //EFUSE[10~13] 0x19CAC558  (refer the structure "RLT8723_DATA_UART_SETTINGS_S")   
    UINT32 rtl8723_data_uart_settings_2;//EFUSE[14~17] (refer the structure "RLT8723_DATA_UART_SETTINGS_2_S")
    UINT32 rtl8723_data_uart_settings_3;//EFUSE[18~1B]  (refer the structure "RLT8723_DATA_UART_SETTINGS_3_S")
    UINT32 efuse_lps_setting_1_d32;     //EFUSE[1C~1F]
    UINT32 efuse_lps_setting_2_d32;     //EFUSE[20~23]
    UINT32 efuse_lps_setting_3_d32;	    //EFUSE[24~27]
    UINT8 efuse_lps_setting_4_d8;       //EFUSE[28]
    UINT8 efuse_lps_setting_5_d8;       //EFUSE[29](refer the structure "EFUSE_LPS_SETTING_5_S")  
    UINT8 efuse_lps_setting_6_d8;       //EFUSE[2A](refer the structure "EFUSE_LPS_SETTING_6_S")  
    UINT8 efuse_lps_setting_rsvd_d8;    //EFUSE[2B] 

    UINT32 lps_mode_max_time;           //EFUSE[2C~2F]
    UINT16 power_option;                //EFUSE[30~31] 0x2B5
    UINT16 power_seq_param;             //EFUSE[32~33] 0x00000000
    UINT16 general_control;             //EFUSE[34~35]

    UINT8  default_max_flush_time;      //EFUSE[36] 200 ms
    UINT8  mailbox_max_timeout;         //EFUSE[37]
    UINT8  gpio_wake_up_time;           //EFUSE[38] 1 uint:10 ms
    UINT8  pta_pkt_unit;                //EFUSE[39]
    UINT8  control_time_parameter;      //EFUSE[3A]
    UINT8  pre_detect_pkt_num;          //EFUSE[3B]
    UINT8  pdn_delay_time_b0;           //EFUSE[3C]
    UINT8  pdn_delay_time_b1;           //EFUSE[3D]
    
    UINT8  efuse_pow_setting_1_d8;      //EFUSE[3E] (refer the structure "EFUSE_POW_SETTING_1_S")
    UINT8  efuse_pow_setting_2_d8;      //EFUSE[3F] (refer the structure "EFUSE_POW_SETTING_2_S")    
    UINT8  efuse_pow_setting_3_d8;      //EFUSE[40]  (refer the structure "EFUSE_POW_SETTING_3_S")
    UINT8  efuse_pow_setting_4_d8;      //EFUSE[41]  (refer the structure "EFUSE_POW_SETTING_4_S")
    UINT8  efuse_pow_setting_5_d8;      //EFUSE[42]  (refer the structure "EFUSE_POW_SETTING_5_S")
    UINT8  efuse_pow_setting_6_d8;      //EFUSE[43]  (refer the structure "EFUSE_POW_SETTING_6_S")  

    //**************for Bluetooth Para*************
    //*     size: 184 Bytes                        *
    //*********************************************
    UINT8 bt_bd_addr[6];                    //EFUSE[44~49] 0x7c, 0x77, 0xd3, 0x72, 0x02, 0x03
    UINT8 bt_local_name[64];                //EFUSE[4A~89] 'RTK_BT_2.1'
    UINT8 bt_master_piconet_fea[8];         //EFUSE[8A~91] 0xff,0xff,0xff,0xfe,0x9B,0xff,0x79,0x83
    UINT8 bt_security_key[16];              //EFUSE[92~A1]
    UINT8 bt_country_code;                  //EFUSE[A2] 0x0

    UINT8 bt_default_tx_power_index_minus;  //EFUSE[A3]  
    UINT16 bt_manufacturer_name;            //EFUSE[A4~A5] 0x000a(0xffff) 0x000a is CSR's name, and this value help us be recognized by win7
    UINT16 bt_sleep_mode_threshold;         //EFUSE[A6~A7] 0x800
    UINT16 bt_deep_sleep_mode_threshold;    //EFUSE[A8~A9] 0xfffe
    UINT16 bt_def_link_policy_settings;     //EFUSE[AA~AB] 0
    UINT16 bt_t_poll_slave;                 //EFUSE[AC~AD] 0x24
    UINT16 bt_t_poll;                       //EFUSE[AE~AF] 0x24
    UINT16 bt_supervision_timeout;          //EFUSE[B0~B1] 0x7D00
    UINT16 bt_page_timeout;                 //EFUSE[B2~B3] 0x2000
    UINT16 bt_conn_accept_timeout;          //EFUSE[B4~B5] 0x1f40
    UINT16 bt_read_buffer_size;             //EFUSE[B6~B7] 1021
    UINT16 bt_page_scan_interval;           //EFUSE[B8~B9] 0x800
    UINT16 bt_page_scan_window;             //EFUSE[BA~BB] 0x12
    UINT16 bt_inquiry_scan_interval;        //EFUSE[BC~BD] 0x1000
    UINT16 bt_inquiry_scan_window;          //EFUSE[BE~BF] 0x12
    UINT16 bt_hold_max_interval;            //EFUSE[C0~C1] 0xffff
    UINT16 bt_hold_min_interval;            //EFUSE[C2~C3] 0x01
    UINT16 bt_sniff_max_interval;           //EFUSE[C4~C5] 0xffff
    UINT16 bt_sniff_min_interval;           //EFUSE[C6~C7] 0x01
    UINT16 bt_beacon_max_interval;          //EFUSE[C8~C9] 0x2000
    UINT16 bt_beacon_min_interval;          //EFUSE[CA~CB] 0xc0
    UINT16 bt_priority_low;                 //EFUSE[CC~CD] 0x0000    
    UINT16 bt_priority_high;                //EFUSE[CE~CF] 0x0000
    UINT16 bt_priority_3;                   //EFUSE[D0~D1] 0x0000 bit[7:0]: bt_priority_3
                                            // bit[8]: IS_HARDWARE_ERR_EVENT_TO_HOST_WHEN_HOST_ACL
                                            // bit[9]: IS_SEND_EVT_WHEN_WDG_TO
                                            // bit[11:10]: NUM_RETX_TO_PAUSE_ESCO
                                            // bit[13:12]: PAUSE_ESCO_INTERVAL
                                            // bit[14]: IS_USE_FOR_MUTE
                                            // bit[15]: EN_FW_FIX_SCO_NAK_HID/IS_BT50
    UINT16 bt_para_rsvd;                    //EFUSE[D2~D3] 
    
    UINT32 bt_class_of_device;              //EFUSE[D4~D7] 0x0    
    UINT32 bt_func_support_policy;          //EFUSE[D8~DB] 0x0000001F
                                        /* bit[0]: support BT3.0 
                                           bit[1]: support BT4.0
                                           bit[2]: support scatternet 
                                           bit[3]: allow inq scan after conn
                                           bit[4]: power saving enable
                                           bit[5]: test for BQB
                                           bit[6]: support hci log packet
                                           bit[7]: support WPAN LED ?
                                           bit[8]: decide 2Mbps packet type first ?
                                           bit[11:9]: max log event packet length (32 byte unit)
                                           bit[12]: support BT CSA4
                                           bit[13]: support BT 4.1 Secure Connection
                                           bit[14]: support BT 4.1 or RTK MWS features
                                           bit[15]: do not generate log message ?
                                           bit[26:16]: max log acl packet length (32 byte unit)
                                           bit[27]: is rf init
                                           bit[28]: usb 8703b_a_cut RF parameter 
                                           bit[29]: reserved
                                           bit[30]: use pta meter ?
                                           bit[31]: use aggresive hci rx dma ? */

    
    UINT32 bt_func_support_policy1;     //EFUSE[DC~DF] 0x00000000
    
    UINT32 bt_le_fw_policy;                 //EFUSE[E0~E3] 0x429C4018                 
                                        /* bit[0]: IS_LE_CONN_INTV_AUTO_UPDATE_EN = 0
                                           bit[1]: IS_LE_CONN_INTV_DEFAULT_EN = 0 
                                           bit[13:2]: LE_CONN_INTV_DEFAULT_VALUE = 6
                                           bit[14]: IS_LE_AUTO_CONN_UPDATE_EN = 1 
                                           bit[17:15]: LE_SLOT_AVOID_FOR_SCO = 0 
                                           bit[18]: IS_TIMER3_FOR_LE_DUAL_WHCK = 0
                                           bit[19]: LE_LEGACY_COMPENSATE_SLOT = 1
                                           bit[20]: BLOCK_LEGACY_WHEN_LE = 0
                                           bit[23:21]: g_block_legacy_for_le_slot_num
                                           bit[26:24]: BLOCK_SLOT_NUM_3SLOT
                                           bit[28:27]: LE_MAX_NUM_OF_ADV_REPORT_OPTION
                                           bit[31:29]: LEGACY_INQUIRY_RESERVED_12SLOT
                                           */      

    UINT32 bt_func_support_policy_ext;   //EFUSE[E4~E7]
                                         /* bit[0]: IS_BT42
                                           bit[1]: IS_SUPPORT_3DG_APP ?
                                           bit[2]: support enhanced cpu sleep mode ?
                                           bit[3]: support dual bzdma hci cmds to 
                                                   handle race condition when two le 
                                                   connection events are overlap
                                           bit[4]: NO_BLOCK_LEGACY_FOR_LE_WHEN_RETX ?
                                           bit[5]: NO_BLOCK_LEGACY_FOR_LE_WHEN_LEGACY_SLV ?
                                           bit[8:6]: PAUSE_LE_INTERVAL
                                           bit[10:9]: BLOCK_NUM_OF_LEGACY_WHEN_BR
                                           bit[11]: PAUSE_SCO_FOR_LE ?
                                           bit[12]: IS_NO_LE_TX_THRESHOLD_FOR_MULTI_CE ?
                                           bit[13]: IS_NO_LE_RX_THRESHOLD_FOR_MULTI_CE ?
                                           bit[14]: IS_AUTO_CHOOSE_LE_INTV ?
                                           bit[15]: SHORTEN_MASTER_SNIFF_TIMEOUT_VAL ?
                                           bit[16]: IS_EVENT_REORDER_FOR_ANY_INTERFACE ?
                                           bit[17]: LE_AUTO_UPDT_WHEN_SLOT_CONFLICT_AFTER_CONN_UPDT ?
                                           bit[21:18]: g_le_use_interval_slot_pair
                                           bit[22]: IS_USE_BTON_WDG_TIMER
                                           bit[23]: IS_WPAN_LED_USE_LED0
                                           bit[27:24]: BT_CSB_BEACON_RX_XTOL
                                           bit[28]: IS_MUTED_OVER_CODEC_ENABLE
                                           bit[29]: IS_PLC_ENABLE
                                           bit[30]: IS_BT41
                                           bit[31]: IS_USE_SCOREBOARD
                                         */
    UINT16 pta_default_setting;         //EFUSE[E8~E9] 
                                        /* pta register default settng */
                                        
    UINT8 bt_global_slot_min_sniff_intv;  //EFUSE[EA] 0x1E


/*    UINT8 rsvd_field[11];*/// original, before 8703b
#ifndef _SUPPORT_WL_BT_SCOREBOARD_
    UINT8 rsvd_field_3;                    //EFUSE[EB]
#else
    UINT8 efuse_misc_interrupt_d8;         // EFUSE[EB]
#endif    

#ifndef _DAPE_CHOOSE_LE_INTV_BY_CONTROLLER
    UINT8 rsvd_field_4;                    //EFUSE[EC] 
#else
    UINT8 bt_auto_choose_le_intv;         //EFUSE[EC] IS_AUTO_CHOOSE_LE_INTV
#endif
                                              
#ifndef _OTP_BTON_GPIO_DEF_
    UINT8 rsvd_field_2;                    // EFUSE[ED], original
#else
    UINT8 efuse_btongpio_init_d8;          //EFUSE[ED]   
                                                // bit[0]: enable led0 log output
#ifndef _SUPPORT_INFO_FROM_SYSTEM_ON_                                            
                                                // bit[7:1]: reserved
#else                                                
                                            // bit[1]: rsv
                                            // bit[2]: rsv
                                            // bit[3]: 0: interface info from page0, 1: from efuse
                                            // bit[7:4] if(bit3 == 1), the 4 bits indicates chip version
#endif                                                
                                                
#endif    
    /*  Added by Wallice for HCI DMA Enhancement Function.	2012/03/20	*/
    //*********HCI DMA Enhancement Function**************
    //*     size: 2 Bytes                             					*
    //*************************************************
    UINT8  HCI_DMA_EnhanCtrl;			//EFUSE[EE]//UINT8 bit0 => HCI DMA Enhancement Function disable: 0, HCI DMA Enhancement Function enable: 1 
    						  			//		bit1 => HCI Hardware_Reset Event return disable: 0, HCI Hardware_Reset Event return enable: 1	
                                                 //		bit2 => HCI DMA Enhancement Function Interrupt disable: 0, HCI DMA Enhancement Function Interrupt enable: 1    
    						            //		bit3 => HCI DMA Enhancement Function(CMD) Interrupt disable: 0, HCI DMA Enhancement Function(CMD) Interrupt enable: 1	
    						            //		bit4 => HCI Hardware_Reset Event(CMD) return disable: 0, HCI Hardware_Reset Event(CMD) return enable: 1	
    UINT8  HCI_DMA_EnhanRsvd0;          //EFUSE[EF] UINT8 reserved0 
    /*  End Added by Wallice for HCI DMA Enhancement Function.		2012/03/20	*/
	
    /*  Added by Wallice for USB LPM Parameters.	2012/03/01	*/
    //******************for USB LPM*********************
    //*     size: 4 Bytes                             *
    //*************************************************
#ifndef _SUPPORT_POLLING_BASED_LPM_L1_   
    UINT8  USB_LPM_Allow;                   // EFUSE[F0]	   //UINT8 bit 0 => LPM Disable: 0, LPM Enable: 1
    								        //     bit 1 => Control by F/W: 0, Control by Driver: 1
#else
    UINT8  USB_LPM_Control;                 // EFUSE[F0], for CV3 set 0x00, for normal set 0x02, 20140708, fongpin
                                            // bit 0 => LPM Disable: 0, LPM Enable: 1               
                                            // bit 1 => Support USB LPM operation FSM by BT FW
                                            //          1: Yes, 0: NO  
                                            // bit 2 => USB LPM token response policy, 
                                            //          1: always NYET, 
                                            //          0: return ACK when rx fifo is empty during a sample period
                                            // bit[7:3] => sample period (units: ms)
#endif

    UINT8  USB_LPM_HIRDBESL_Thrd;          //EFUSE[F1] HIRD/BESL Threshold Value. HIRD: bit7:4, BESL: bit3:0
                                            // it is no use for BESL
    // for hird
    // for besl?
#ifndef _SUPPORT_USB_LOG_ENABLE_     
    UINT8  USB_LPM_Rsvd0;        			//EFUSE[F2] LPM reserved0
#else
    UINT8  USB_misc;                        // EFUSE[F2] usb misc
                                            // bit0: enable usb log
                                            // bit[3:1] reserved
                                            // bit[7:4]: d2 timer log, N*10sec    
#endif
    
    /*  End Added by Wallice for USB LPM Parameters.	2012/03/01	*/

    /*  Updated by Wallice for PCM external CODEC.    2013/07/08  */
    //**************for PCM External CODEC*************
    //*     size: 8 __bytes_recd_cb                   *
    //*************************************************
    UINT8  pcm_convert;                          
                                    //EFUSE[F3] bit[7:5] -> Shift Level scale
                                   //               bit[4]-> PCM OUT Shift Type. Shift Left: 0,  Shift Right: 1
                                    //EFUSE[F3] bit[3] -> up/down sampling activate
                                   //               bit[2]-> Convert 16K to 8K Codec: 1, Convert 8K to 16K Codec:0
                                   //               bit[1:0]-> filter type. type 0: 2'b00, type 1: 2'b01, type 2: 2'b10, type3: 2'b11
    UINT8  hci_excodec_state;               
                                    //EFUSE[F4] Bit[3:0]-> HCI:0x0, PCM External CODEC:0x1, I2S External Codec: 0x02
									//		    Bit[7:4]-> Linear: 0x00, u-Law: 0x01, A-Law: 0x02, CVSD: 0x03, mSBC: 0x04		
    UINT8  scoconv;                  //EFUSE[F5] Data format

    UINT16 pcm_if_ctrl1;             //EFUSE[F6~F7] PCM I/F Ctrl1
    UINT16 pcm_if_ctrl2;             //EFUSE[F8~F9] PCM I/F Ctrl2
    UINT16 pcm_if_ctrl3;             //EFUSE[FA~FB] PCM I/F Ctrl3
    
    UINT16 efuse_park_pcm_mux_mgr;   //EFUSE[FC~FD], refer to struct PARK_PCM_GPIO_PIN_S_TYPE
                                            //bit[12]: function enabler: restore pcm mux state to "before sco connection"
                                            //bit[11:8]: gpio3~0 output value
                                            //bit[7:4]: gpio3~0 output enable
                                            //bit[3:0]: gpio3~0 on/off domain, 1: off, 0 on


    /*  End Updated by Wallice for PCM external CODEC.    2013/07/08  */

    //**************for BW RF Settings*************
    //*     size: 32 Bytes                        *
    //*********************************************
    UINT8 bw_rf_radio_sys_clk_sel;          //EFUSE[FE] MT:0x17 RT:0x09
    UINT8 bw_rf_osc_delay;                  //EFUSE[FF] 8
    UINT8 bw_rf_low_clk_frac;               //EFUSE[100] 0x1 //32K(1) or 32.768K(0)
    UINT8 bw_rf_reserved;                   //EFUSE[101]
    INT16 bw_rf_min_rssi_dbm;               //EFUSE[102~103] -40 dBm
    INT16 bw_rf_max_rssi_dbm;               //EFUSE[104~105] -30 dBm
    UINT16 bw_rf_delay_vals[10];            //EFUSE[106~119] MT: 0xD6D3, 0xE0DD, 0x8989, 0x8989, 0x135, 0x140, 0x140, 0x13B, 0x933C, 0xFFFF
                                            //RT: 0xC0B8, 0xE0D3, 0x8989, 0x8989, 0x116, 0x140, 0x140, 0x142, 0x933C, 0xFFFF

    //********for LE Controller Timing Settings******
    //*     size: 10 Bytes                         *
    //*********************************************
    UINT16 le_ce_early_int_time;            //EFUSE[11A~11B] 0x012C (300 us)
    UINT16 le_trx_on_delay;                 //EFUSE[11C~11D] 0xDCDC
    UINT16 le_rx_turnaround_delay;          //EFUSE[11E~11F] 0x328A
    UINT16 le_tx_turnaround_delay;          //EFUSE[120~121] 0x1083 (txon_early=16 tx2rx_delay=131)
    UINT16 le_ce_rx_timeout_delay;          //EFUSE[122~123] 0x0164
    UINT16 le_rx_search_timeout_value;      //EFUSE[124~125] 0x01F0
    UINT16 le_clock_compensate;             //EFUSE[126~127] 0x0147

    //********for Jr.Neil's AFH Settings*************
    //*     size: 16 Bytes                         *
    //*********************************************
    UINT8 rtk_afh_mechanism_enable;         //EFUSE[128] 0x01
#ifndef _NEW_MODEM_PSD_SCAN_        
    UINT8 afh_reserved_00;                  //EFUSE[129] 0xff
#else
    UINT8 rtk_afh_bt_psd_enable;            //EFUSE[129] 0x00   
#endif
    UINT8 min_channels_by_rtk;              //EFUSE[12A] 0x1C
    UINT8 rtk_afh_using_slave_report;       //EFUSE[12B] 0x00
    UINT8 rtk_afh_wlan_psd_enable;          //EFUSE[12C] 0x01
    UINT8 max_afh_execute_times;            //EFUSE[12D] 0xC7
    UINT8 afh_execute_times_I;              //EFUSE[12E] 0x63
    UINT8 afh_execute_times_II;             //EFUSE[12F] 0x3B
    UINT8 afh_execute_times_III;            //EFUSE[130] 0x13
    UINT8 afh_execute_times_IV;             //EFUSE[131] 0x09
    UINT8 min_afh_execute_times;            //EFUSE[132] 0x02
    UINT8 wlan_gap_threshold;               //EFUSE[133] 0x0A
    INT16 score_threshold;                  //EFUSE[134~135] 0x0000
    UINT8 afh_check_wifi_alive_count;       //EFUSE[136] 0x00
    INT8 trk_afh_score_thres_sco;           //EFUSE[137] -13
    INT8 trk_afh_score_thres_i;             //EFUSE[138] -19
    INT8 trk_afh_score_thres_ii;            //EFUSE[139] -38
    
    //**** for RF Settings - Tx Gain Table***********
    //*      size: 40 Bytes                        *
    //*********************************************
    UINT8 EFuse_PHYInit_TxGainTable[32];    //EFUSE[13A~159]
    UINT8 EFuse_PHYInit_MaxTxIndex;         //EFUSE[15A] 
    UINT8 EFuse_PHYInit_MaxTxGain1M;        //EFUSE[15B] 
    UINT8 EFuse_PHYInit_MaxTxGain2M;        //EFUSE[15C] 
    UINT8 EFuse_PHYInit_MaxTxGain3M;        //EFUSE[15D] 
    UINT8 EFuse_PHYInit_MaxTxGainLE;        //EFUSE[15E] 
    UINT8 EFuse_PHYInit_TxGainStep;         //EFUSE[15F] 
    UINT8 EFuse_PHYInit_TxScaleFactor;      //EFUSE[160] 
    UINT8 EFuse_PHYInit_TxDACCurrent;       //EFUSE[161] 

    //**** for RF Settings - Rx Gain Table***********
    //*      size: 10 Bytes                        *
    //*********************************************
#ifdef _NEW_8821A_RX_AGC_API_
    UINT16 EFuse_PHYInit_Gain0Stop;         //EFUSE[162~163]; also refer to efuse_modem_rssi0_pin_dBm //
    UINT16 EFuse_PHYInit_Gain0Start;        //EFUSE[164~165] 
    UINT16 EFuse_PHYInit_Gain1Start;        //EFUSE[166~167] 
    UINT16 EFuse_PHYInit_Gain2Start;        //EFUSE[168~169] 
    UINT16 EFuse_PHYInit_Gain3Start;        //EFUSE[16A~16B] 
    UINT16 EFuse_PHYInit_Gain4Start;        //EFUSE[16C~16D] 
    UINT16 EFuse_PHYInit_Gain5Start;        //EFUSE[16E~16F] 
    UINT16 EFuse_PHYInit_Gain6Start;        //EFUSE[170~171] 
    UINT16 EFuse_PHYInit_Gain7Start;        //EFUSE[172~173] 
#else
    UINT16 EFuse_PHYInit_HGainStart;        // also refer to efuse_modem_rssi0_pin_dBm //
    UINT16 EFuse_PHYInit_MGainStart;
    UINT16 EFuse_PHYInit_LGainStart;
    UINT16 EFuse_PHYInit_UGainStart;
    UINT16 EFuse_PHYInit_UGainStop;
    UINT16 EFuse_PHYInit_rsvd[4];
#endif

#ifdef _NEW_BT_EFUSE_OPTION_FOR_TV_APP_
    //****** TV 3DD Settings **********************
    //*      size: 20 Bytes                       *
    //*********************************************
    UINT8 EFuse_TV_3DD_Group[20];       //EFuse[174~187] /* Refer to BT_EFUSE_TV_3DD_GROUP */
#else
    //********** for RF Settings - RF IQK ***********
    //*      size: 20 Bytes                        *
    //*********************************************
    UINT8 EFuse_IQK_Settings;               //EFUSE[174]
    UINT8 EFuse_IQK_RFChL;                  //EFUSE[175]
    UINT8 EFuse_IQK_RFChH;                  //EFUSE[176]
    UINT8 EFuse_IQK_TxGainL;                //EFUSE[177]
    UINT8 EFuse_IQK_TxGainH;                //EFUSE[178]
    UINT8 EFuse_IQK_RxGain;                 //EFUSE[179]
    UINT8 EFuse_IQK_ErrorTh;                //EFUSE[17A]
    UINT8 EFuse_IQK_LoadExtSetup;           //EFUSE[17B]
    UINT16 EFuse_IQK_TxIMRDefault;          //EFUSE[17C~17D]
    UINT16 EFuse_IQK_RxIMRDefault;          //EFUSE[17E~17F]
    UINT16 EFuse_IQK_LOKLDefault;           //EFUSE[180~181]
    UINT16 EFuse_IQK_LOKHDefault;           //EFUSE[182~183]
    UINT16 EFuse_IQK_RxAGC;                 //EFUSE[184~185]
    UINT16 EFuse_IQK_IQKPara;               //EFUSE[186~187]
#endif

    //**** for RF Settings - Tx Power Tracking ******
    //*      size: 16 Bytes                        *
    //*********************************************
    UINT8 EFuse_TxPowerTrack_En;            //EFUSE[188] 
    UINT8 EFuse_ThermalDefault;             //EFUSE[189] 
    UINT8 EFuse_ThermalUpdateInterval;      //EFUSE[18A] 
    UINT8 EFuse_TxPowerTrack_ThermalDelta;  //EFUSE[18B] 
    UINT8 EFuse_TxPowerTrack_TxGain[13];    //EFUSE[18C~198] 
    UINT8 EFuse_TxPowerTrack_TxGain_LBond[2];//EFUSE[199~19A] 
    UINT8 EFuse_TxPowerTrack_TxGain_HBond[2];//EFUSE[19B~19C] 

    //**** for RF Settings - CFO (Center Frequency Offset) ****
    //*                size: 14 Bytes                        *
    //*******************************************************
    UINT8 EFuse_CFOTrack_En;                //EFUSE[19D] 
    UINT8 EFuse_CFOTrack_ThermalDelta;      //EFUSE[19E] 
    UINT8 EFuse_CFOTrack_Cap[13];           //EFUSE[19F~1AB] 
    UINT8 EFuse_CFOTrack_Cap_LBond[2];      //EFUSE[1AC~1AD] 
    UINT8 EFuse_CFOTrack_Cap_HBond[2];      //EFUSE[1AE~1AF] 

#ifdef _NEW_BT_EFUSE_OPTION_FOR_TV_APP_
    //****** TV Power On Settings *****************
    //*      size: 30 Bytes                       *
    //*********************************************
    UINT8 EFuse_TV_Poweron_Group[30];           //EFuse[1B0~1CD] /* Refer to BT_EFUSE_TV_POWERON */
#else
    //**** for RF Settings - Rx Power Tracking ******
    //*      size: 28 Bytes                        *
    //*********************************************
    UINT8 EFuse_RxGainTrack_En;             //EFUSE[1B0]
    UINT8 EFuse_RxGainTrack_ThermalDelta;   //EFUSE[1B1]
    UINT16 EFuse_RxGainTrack_Para[13];      //EFUSE[1B2~1CB]

    //******* for RF Settings - Re-Calibration ******
    //*       size: 2 Bytes                        *
    //*********************************************
    UINT8 EFuse_ReRFCal_ThermalDelta;       //EFUSE[1CC]
    UINT8 EFuse_ReRFCal_Settings;           //EFUSE[1CD]
#endif

    //********** for RF Settings - LOK Tracking *****
    //*      size: 54 Bytes                        *
    //*********************************************
    /* Do not support LOK Tracking in 8723A B-Cut */
#ifndef _SUPPORT_ANTENNA_SELECTION_
    UINT8 EFuse_Rsvd_0[2];                  //EFUSE[1CE~1CF]
#else
    UINT8 Efuse_Ant_cfg0;                   //EFUSE[1CE], compare to wifi efuse 0xC1[3], 
                                            //Bit[3]: Non-interrupt Antenna Diversity,0: disable, 1: enable
    UINT8 Efuse_Ant_cfg1;                   //EFUSE[1CF], compare to wifi efuse 0xC3[0],[6]
                                            //Bit[0]: Total antenna number, 0: 2-Antenna (default), 1: 1-Antenna
                                            //Bit[6]: Single antenna path, 
                                            //0: Single antenna use S1 (default), 1: Single antenna use S0
#endif 

    UINT8 efuse_modem_setting_1_d8;         //EFUSE[1D0]  (refer the structure "EFUSE_MODEM_SETTING_1_S")
    INT8 efuse_modem_rssi0_pin_dBm;         //EFUSE[1D1] -90 means -90dBm

    UINT16 efuse_baud_est_setting_1_d16;    //EFUSE[1D2~1D3]  (refer the structure "EFUSE_BAUD_EST_SETTING_1_S")
    UINT16 efuse_baud_est_setting_2_d16;    //EFUSE[1D4~1D5]  (refer the structure "EFUSE_BAUD_EST_SETTING_2_S")
    UINT16 efuse_baud_est_setting_3_d16;    //EFUSE[1D6~1D7]  (refer the structure "EFUSE_BAUD_EST_SETTING_3_S")
    UINT16 efuse_baud_est_setting_4_d16;    //EFUSE[1D8~1D9]  (refer the structure "EFUSE_BAUD_EST_SETTING_4_S")
    UINT16 efuse_modem_psd_setting_1_d16;   //EFUSE[1DA~1DB]  (refer the structure "EFUSE_MODEM_PSD_SETTING_1_S")
    UINT16 efuse_modem_psd_setting_2_d16;   //EFUSE[1DC~1DD]  (refer the structure "EFUSE_MODEM_PSD_SETTING_2_S")

    UINT16 EFuse_Rsvd_2;                    //EFUSE[1DE~1DF]   
    UINT16 EFuse_Rsvd_buf[4];               //EFUSE[1E0~1E7]
    UINT16 efuse_host_info;                 //EFUSE[1E8~1E9]
    UINT16 efuse_ft_info;					//EFUSE[1EA~1EB]
    										/*	BIT[0]:ignore_ft_efuse_record
												BIT[1]:use_ft_efuse_lok_iqk
												BIT[2~31]:rsvd
											*/
    UINT16 efuse_cal_sts[4];                //EFUSE[1EC~1F3]    

#ifdef _IS_ASIC_
    //*** for PHY Init (RF/Modem/Bluewiz/Vendor)*****
    //*       size:  4N Bytes                      *
    //*********************************************
  #ifdef _OTP_ADD_SYS_INIT_PARAM_
    UINT16 sys_init_param[64];             //EFUSE[1F4~273] 
    UINT16 phy_init_param[192];             //EFUSE[274~3F3] 
  #else
    UINT16 phy_init_param[256];             //EFUSE[1F4~3F3] 
  #endif  
#endif
} _PACKED_ OTP_STRUCT;


#ifndef _IS_ASIC_
typedef struct otp_rf_struct
{
    //**************for Modem ini Settings********
    //*     size: 76 Bytes                        *
    //*********************************************
    UINT16 mdm_ini_0x00;                    //0x6f76
    UINT16 mdm_ini_0x01;                    //[0x4115 for FPGA(si)]  [0x4105 for ASIC(pi)]
    UINT16 mdm_ini_0x02;                    //0x94db
    UINT16 mdm_ini_0x03;                    //0x309a
    UINT16 mdm_ini_0x04;                    //0x361f
    UINT16 mdm_ini_0x05;                    //0x00b0
    UINT16 mdm_ini_0x06;                    //0x4912
    UINT16 mdm_ini_0x07;                    //0x13f2
    UINT16 mdm_ini_0x08;                    //0x569e
    UINT16 mdm_ini_0x09;                    //0x0f48
    UINT16 mdm_ini_0x0A;                    //0x0000
    UINT16 mdm_ini_0x0B;                    //0x5a32
    UINT16 mdm_ini_0x0C;                    //0x690B
    UINT16 mdm_ini_0x0D;                    //0x0000
    UINT16 mdm_ini_0x16;                    //0x4400
    UINT16 mdm_ini_0x17;                    //0x0cc0
    UINT16 mdm_ini_0x19;                    //0x01d4
    UINT16 mdm_ini_0x1A;                    //0x0030
    UINT16 mdm_ini_0x1B;                    //0x0001
    UINT16 mdm_ini_0x1C;                    //0x034d
    UINT16 mdm_ini_0x1D;                    //0x2521
    UINT16 mdm_ini_0x1E;                    //0x00e9
    UINT16 mdm_ini_0x30;                    //0x5693
    UINT16 mdm_ini_0x31;                    //0x0000
    UINT16 mdm_ini_0x32;                    //0x6f73
    UINT16 mdm_ini_0x33;                    //0x044f

    UINT16 mdm_ini_BW_0x136;                //0x8984 (0x8404)
    UINT16 mdm_ini_BW_0x138;                //0x0628 (0x8485)
    UINT16 mdm_ini_BW_0x13a;                //0x522c (0x11B2)
    UINT16 mdm_ini_BW_0x13c;                //0x008b (0x0091)
    UINT16 mdm_ini_BW_0x1bc;                //0x8404
    UINT16 mdm_ini_BW_0x1be;                //0x8485
    UINT16 mdm_ini_BW_0x1c0;                //0x11B2
    UINT16 mdm_ini_BW_0x1c2;                //0x0091
    UINT16 mdm_ini_BW_0x1c4;                //0x8404
    UINT16 mdm_ini_BW_0x1c6;                //0x8485
    UINT16 mdm_ini_BW_0x1c8;                //0x11B2
    UINT16 mdm_ini_BW_0x1ca;                //0x0091

    //********for Modem rx tgc tbl Settings********
    //*     size: 128 Bytes                        *
    //*********************************************
    UINT16 mdm_rxagc_tbl_00;                //0x0e3f
    UINT16 mdm_rxagc_tbl_01;                //0x0e3e
    UINT16 mdm_rxagc_tbl_02;                //0x0e3d
    UINT16 mdm_rxagc_tbl_03;                //0x0e3c
    UINT16 mdm_rxagc_tbl_04;                //0x0e3b
    UINT16 mdm_rxagc_tbl_05;                //0x0e3a
    UINT16 mdm_rxagc_tbl_06;                //0x0e39
    UINT16 mdm_rxagc_tbl_07;                //0x0e38
    UINT16 mdm_rxagc_tbl_08;                //0x0e37
    UINT16 mdm_rxagc_tbl_09;                //0x0e36
    UINT16 mdm_rxagc_tbl_10;                //0x0e35
    UINT16 mdm_rxagc_tbl_11;                //0x0e34
    UINT16 mdm_rxagc_tbl_12;                //0x0e33
    UINT16 mdm_rxagc_tbl_13;                //0x0e32
    UINT16 mdm_rxagc_tbl_14;                //0x0e31
    UINT16 mdm_rxagc_tbl_15;                //0x0e30
    UINT16 mdm_rxagc_tbl_16;                //0x0f2f
    UINT16 mdm_rxagc_tbl_17;                //0x102e
    UINT16 mdm_rxagc_tbl_18;                //0x112d
    UINT16 mdm_rxagc_tbl_19;                //0x402c
    UINT16 mdm_rxagc_tbl_20;                //0x412b
    UINT16 mdm_rxagc_tbl_21;                //0x422a
    UINT16 mdm_rxagc_tbl_22;                //0x4329
    UINT16 mdm_rxagc_tbl_23;                //0x4428
    UINT16 mdm_rxagc_tbl_24;                //0x4527
    UINT16 mdm_rxagc_tbl_25;                //0x4626
    UINT16 mdm_rxagc_tbl_26;                //0x4725
    UINT16 mdm_rxagc_tbl_27;                //0x4824
    UINT16 mdm_rxagc_tbl_28;                //0x4923
    UINT16 mdm_rxagc_tbl_29;                //0x4a22
    UINT16 mdm_rxagc_tbl_30;                //0x4b21
    UINT16 mdm_rxagc_tbl_31;                //0x4c20
    UINT16 mdm_rxagc_tbl_32;                //0x4d1f
    UINT16 mdm_rxagc_tbl_33;                //0x4e1e
    UINT16 mdm_rxagc_tbl_34;                //0x4f1d
    UINT16 mdm_rxagc_tbl_35;                //0x501c
    UINT16 mdm_rxagc_tbl_36;                //0x511b
    UINT16 mdm_rxagc_tbl_37;                //0x521a
    UINT16 mdm_rxagc_tbl_38;                //0x6219
    UINT16 mdm_rxagc_tbl_39;                //0x6318
    UINT16 mdm_rxagc_tbl_40;                //0x6417
    UINT16 mdm_rxagc_tbl_41;                //0x6516
    UINT16 mdm_rxagc_tbl_42;                //0x6615
    UINT16 mdm_rxagc_tbl_43;                //0x6714
    UINT16 mdm_rxagc_tbl_44;                //0x6813
    UINT16 mdm_rxagc_tbl_45;                //0x6912
    UINT16 mdm_rxagc_tbl_46;                //0x6a11
    UINT16 mdm_rxagc_tbl_47;                //0x6b10
    UINT16 mdm_rxagc_tbl_48;                //0x6c0f
    UINT16 mdm_rxagc_tbl_49;                //0x6d0e
    UINT16 mdm_rxagc_tbl_50;                //0x6e0d
    UINT16 mdm_rxagc_tbl_51;                //0x6f0c
    UINT16 mdm_rxagc_tbl_52;                //0x700b
    UINT16 mdm_rxagc_tbl_53;                //0x710a
    UINT16 mdm_rxagc_tbl_54;                //0x7209
    UINT16 mdm_rxagc_tbl_55;                //0x7308
    UINT16 mdm_rxagc_tbl_56;                //0x7407
    UINT16 mdm_rxagc_tbl_57;                //0x7506
    UINT16 mdm_rxagc_tbl_58;                //0x7605
    UINT16 mdm_rxagc_tbl_59;                //0x7704
    UINT16 mdm_rxagc_tbl_60;                //0x7803
    UINT16 mdm_rxagc_tbl_61;                //0x7902
    UINT16 mdm_rxagc_tbl_62;                //0x7a01
    UINT16 mdm_rxagc_tbl_63;                //0x7b00

    //*************for RF init Settings************
    //*     size: 388 Bytes                        *
    //*********************************************
    UINT16 rf_ini_num;
    UINT16 rf_ini_reserved;
    UINT8 rf_ini_offset[128];
    UINT16 rf_ini_value[128];
} _PACKED_ OTP_RF_STRUCT;

extern const OTP_RF_STRUCT otp_rf_str_data;
#endif /* else of #ifndef _IS_ASIC_*/

extern OTP_STRUCT otp_str_data;

extern UINT16 fw_lmp_sub_version; /* can modify it via patch */
extern UINT16 fw_hci_sub_version; /* can modify it via patch */
extern UINT8 g_global_slot_min_allow_sniff_int;

/*----------------------------------------------------*/
/* The Macro Group of "bt_le_fw_policy field"         */
/*----------------------------------------------------*/
#define IS_LE_CONN_INTV_AUTO_UPDATE_EN   (otp_str_data.bt_le_fw_policy & 0x01)         // bit 0
#define IS_LE_CONN_INTV_DEFAULT_EN       (otp_str_data.bt_le_fw_policy & 0x02)         // bit 1
#define LE_CONN_INTV_DEFAULT_VALUE       ((otp_str_data.bt_le_fw_policy >> 2) & 0xFFF) // bit 2~13
#define IS_LE_AUTO_CONN_UPDATE_EN        (otp_str_data.bt_le_fw_policy & BIT14)        // bit 14
#define LE_SLOT_AVOID_FOR_SCO            ((otp_str_data.bt_le_fw_policy >> 15) & 0x07) // bit 15~17
#define IS_TIMER3_FOR_LE_DUAL_WHCK       (otp_str_data.bt_le_fw_policy & BIT18)       // bit 18
#define LE_LEGACY_COMPENSATE_SLOT        ((otp_str_data.bt_le_fw_policy & BIT19) >> 19)// bit 19
#define BLOCK_LEGACY_WHEN_LE              (otp_str_data.bt_le_fw_policy & BIT20)       // bit 20
//#define BLOCK_SLOT_NUM_5SLOT              ((otp_str_data.bt_le_fw_policy >> 21) & 0x07)// bit 21~23
#define BLOCK_SLOT_NUM_3SLOT              ((otp_str_data.bt_le_fw_policy >> 24) & 0x07)// bit 25~26
#define LE_MAX_NUM_OF_ADV_REPORT_OPTION   ((otp_str_data.bt_le_fw_policy >> 27) & 0x03)// bit 27~28
#ifdef _DAPE_TEST_NEW_HW_LE_UNCONN_STATE_HIGHER_THAN_LEGACY_UNCONN
/* there is not enough efuse, so a uint is 12 slot. ex: if LEGACY_INQUIRY_RESERVED_12SLOT = 2,
   then we should reserve 12*2 = 24 slots for legacy inquiry & page. */
#define LEGACY_INQUIRY_RESERVED_12SLOT        ((otp_str_data.bt_le_fw_policy >> 29) & 0x07)// bit 29~31
#endif

/*----------------------------------------------------*/
/* The Macro Group of "bt_func_support_policy1 field"  */
/*----------------------------------------------------*/
//bit[2:0] = 3
#define NEW_V8_BZDMA_SUPPORT_MAX_SEGNUM_GRADE_IN_ONE_LINK  MIN(4, (otp_str_data.bt_func_support_policy1 & 0x07))

//bit[3] = 1
#ifdef _SUPPORT_LBT_FUNC_FOR_RF_ADAPTIVITY_
#define IS_SUPPORT_RF_ADAPTIVITY    (otp_str_data.bt_func_support_policy1 & BIT3)
#else
#define IS_SUPPORT_RF_ADAPTIVITY    FALSE
#endif

//bit[7:4] = 2
#define NEW_V8_BZDMA_SUPPORT_MAX_FRAGNUM_IN_ONE_SEGMENT ((otp_str_data.bt_func_support_policy1 >> 4) & 0x0f)

//bit[31:8]

/*----------------------------------------------------*/
/* The Macro Group of "bt_func_support_policy field"  */
/*----------------------------------------------------*/
//bit[0]
#ifdef VER_3_0
#define IS_BT30                 (otp_str_data.bt_func_support_policy & BIT0)
#else
#define IS_BT30                 FALSE
#endif

//bit[1]
#ifdef LE_MODE_EN
#define IS_BT40                 (otp_str_data.bt_func_support_policy & BIT1)
#else
#define IS_BT40                 FALSE
#endif

//bit[2]
#define IS_SUPPORT_SCATTERNET   (otp_str_data.bt_func_support_policy & BIT2)

//bit[3]
#define IS_INQ_SCAN_AFTER_CONN  (otp_str_data.bt_func_support_policy & BIT3)

//bit[4]
#define IS_ENABLE_POWER_SAVE    (otp_str_data.bt_func_support_policy & BIT4)

//bit[5]
#ifdef _IN_BQB_
#define IS_USE_FOR_BQB          TRUE
#else
#define IS_USE_FOR_BQB          (otp_str_data.bt_func_support_policy & BIT5)
#endif

//bit[6]
#define IS_SUPPORT_HCI_LOG_PKT  (otp_str_data.bt_func_support_policy & BIT6)

//bit[7]
#define IS_SUPPORT_WPAN_LED     (otp_str_data.bt_func_support_policy & BIT7)

//bit[8]
#define IS_PKT_DECIDE_2M_FIRST  (otp_str_data.bt_func_support_policy & BIT8)

//bit[11:9]
#define GET_MAX_LOG_EVENT_SIZE  (((otp_str_data.bt_func_support_policy >> 9) & 0x07) << 5)

//bit[12]
#ifdef VER_CSA4
#define IS_BT_CSA4              (otp_str_data.bt_func_support_policy & BIT12)
#else
#define IS_BT_CSA4              FALSE
#endif

//bit[13]
#ifdef _SUPPORT_SECURE_CONNECTION_
#define IS_BT_SECURE_CONN       (otp_str_data.bt_func_support_policy & BIT13)
#else
#define IS_BT_SECURE_CONN       FALSE
#endif

//bit[14]
#ifdef MWS_ENABLE
#define IS_BT_MWS               (otp_str_data.bt_func_support_policy & BIT14)
#else
#define IS_BT_MWS               FALSE
#endif

//bit[15]
#define IS_NOT_GENERATE_LOG_MSG (otp_str_data.bt_func_support_policy & BIT15)

//bit[26:16]
#define GET_MAX_LOG_ACL_SIZE    (((otp_str_data.bt_func_support_policy >> 16) & 0x07FF) << 5)

#ifdef _IS_ASIC_
//bit[27]
#define IS_RF_INIT              (otp_str_data.bt_func_support_policy & BIT27)

//bit[28]
#define USE_8703B_A_CUT_RF_PARA (otp_str_data.bt_func_support_policy & BIT28)
#else
#define IS_RF_INIT              TRUE
#define USE_8703B_A_CUT_RF_PARA FALSE
#endif

//bit[30]
#define IS_USE_PTA_METER        (otp_str_data.bt_func_support_policy & BIT30)

//bit[31]
#define IS_AGGRESIVE_HCI_RX_DMA (otp_str_data.bt_func_support_policy & BIT31)

/*----------------------------------------------------*/
/* The Macro Group of "bt_priority_3 field"           */
/*----------------------------------------------------*/
#ifdef _DAPE_TEST_NEW_HW
//bit[8]
#define IS_HARDWARE_ERR_EVENT_TO_HOST_WHEN_HOST_ACL (otp_str_data.bt_priority_3 & BIT8)

//bit[9]
#define IS_SEND_EVT_WHEN_WDG_TO      (otp_str_data.bt_priority_3 & BIT9)

//bit[11:10]
#define NUM_RETX_TO_PAUSE_ESCO  ((otp_str_data.bt_priority_3 >> 10) & 0x03)

//bit[13:12]
#define PAUSE_ESCO_INTERVAL     ((otp_str_data.bt_priority_3 >> 12) & 0x03)
#endif

//bit[14]
#define IS_USE_FOR_MUTE         (otp_str_data.bt_priority_3 & BIT14)

#ifdef _DAPE_TEST_FIX_HW_SCO_NAK_SNIFF_BUG
//bit[15]
#define EN_FW_FIX_SCO_NAK_HID   (otp_str_data.bt_priority_3 & BIT15) // default:0

#else
#ifdef _SUPPORT_VER_5_0_
#define IS_BT50                 (otp_str_data.bt_priority_3 & BIT15) // default:1
#else
#define IS_BT50                 FALSE
#endif

#endif

/*---------------------------------------------------------*/
/* The Macro Group of "bt_func_support_policy_ext field"   */
/*---------------------------------------------------------*/
//bit[0]
#ifdef _SUPPORT_VER_4_2_
#define IS_BT42                 (otp_str_data.bt_func_support_policy_ext & BIT0)
#else
#define IS_BT42                 FALSE
#endif

//bit[1]
#ifndef _RTL8821A_ 
#define DONT_CHECK_WIFI_ALIVE   TRUE
#define IS_SUPPORT_3DG_APP      FALSE
#else
#define DONT_CHECK_WIFI_ALIVE   FALSE
#ifndef _3DD_FUNCTION_SUPPORT_
#define IS_SUPPORT_3DG_APP       FALSE
#else
#define IS_SUPPORT_3DG_APP      (otp_str_data.bt_func_support_policy_ext & BIT1)
#endif
#endif

//bit[2]
#define IS_ENHANCED_CPU_SLEEP_MODE  (otp_str_data.bt_func_support_policy_ext & BIT2)

//bit[3]
#ifdef _NEW_BZDMA_FROM_V8_
#define IS_TOGGLE_LL_DATA_CMD   FALSE
#else
#define IS_TOGGLE_LL_DATA_CMD   (otp_str_data.bt_func_support_policy_ext & BIT3)
#endif

//bit[4]
#define NO_BLOCK_LEGACY_FOR_LE_WHEN_RETX   (otp_str_data.bt_func_support_policy_ext & BIT4)

//bit[5]
#define NO_BLOCK_LEGACY_FOR_LE_WHEN_LEGACY_SLV (otp_str_data.bt_func_support_policy_ext & BIT5)

//bit[8:6]
#define PAUSE_LE_INTERVAL  ((otp_str_data.bt_func_support_policy_ext>> 6) & 0x07)

//bit[10:9]
#define BLOCK_NUM_OF_LEGACY_WHEN_BR      ((otp_str_data.bt_func_support_policy_ext>> 9) & 0x03)

//bit[11]
#define PAUSE_SCO_FOR_LE                 (otp_str_data.bt_func_support_policy_ext & BIT11)

//bit[12]
//bit[13]
#ifndef _LE_NO_TRX_THRESHOLD_INT_FOR_MULTI_CE_OPTION_
#define IS_NO_LE_TX_THRESHOLD_FOR_MULTI_CE       FALSE
#define IS_NO_LE_RX_THRESHOLD_FOR_MULTI_CE       FALSE
#else
#define IS_NO_LE_TX_THRESHOLD_FOR_MULTI_CE       (otp_str_data.bt_func_support_policy_ext & BIT12)
#define IS_NO_LE_RX_THRESHOLD_FOR_MULTI_CE       (otp_str_data.bt_func_support_policy_ext & BIT13)
#endif

//bit[14]
//bit[15]
#ifdef _DAPE_CHOOSE_LE_INTV_BY_CONTROLLER
#ifdef _DAPE_TEST_FOR_MUTE
#define IS_AUTO_CHOOSE_LE_INTV                  FALSE
#else
#define IS_AUTO_CHOOSE_LE_INTV                  (otp_str_data.bt_func_support_policy_ext & BIT14)
#endif
#define IS_NO_AFH_SCO_SIMULATE_ACL_PKT           FALSE
#else
#define IS_NO_AFH_SCO_SIMULATE_ACL_PKT           (otp_str_data.bt_func_support_policy_ext & BIT14)
#endif
#define SHORTEN_MASTER_SNIFF_TIMEOUT_VAL         (otp_str_data.bt_func_support_policy_ext & BIT15)

//bit[16]
#if defined(HS_USB_SEND_EVENT_REORDER_ISSUE) || defined(HS_USB_SEND_EVENT_REORDER_ISSUE_NEW)
#define IS_EVENT_REORDER_FOR_ANY_INTERFACE       (otp_str_data.bt_func_support_policy_ext & BIT16)
#else
#define IS_EVENT_REORDER_FOR_ANY_INTERFACE        FALSE
#endif

//bit[17]
#define LE_AUTO_UPDT_WHEN_SLOT_CONFLICT_AFTER_CONN_UPDT (otp_str_data.bt_func_support_policy_ext & BIT17)
/* dape: g_le_use_interval_slot_pair = ((otp_str_data.bt_func_support_policy_ext >> 18) & 0x0F);
   which is initial in ll_fw_init(). default: 5 => to choose 12 slots */

//bit[21:18]
#define LE_USE_INTERVAL_SLOT_PAIR               ((otp_str_data.bt_func_support_policy_ext >> 18) & 0x0F)

//bit[22]
#ifndef _USE_BTON_WATCHDOG_TIMER_
#define IS_USE_BTON_WDG_TIMER                    FALSE   
#else
#define IS_USE_BTIN_WDG_TIMER                   (otp_str_data.bt_func_support_policy_ext & BIT22)
#endif

//bit[23]
#ifndef MINICARD_BT_LED_CONTROL
#define IS_WPAN_LED_USE_LED0                    FALSE
#else
#define IS_WPAN_LED_USE_LED0                    (otp_str_data.bt_func_support_policy_ext & BIT23)
#endif

//bit[27:24]
#ifdef _CSB_RX_SET_XTOL_BY_EFUSE
#define BT_CSB_BEACON_RX_XTOL                   ((otp_str_data.bt_func_support_policy_ext >> 24) & 0x0F)
#endif

//bit[28]
#define IS_MUTED_OVER_CODEC_ENABLE              (otp_str_data.bt_func_support_policy_ext & BIT28)

//bit[29]
#define IS_PLC_ENABLE                           (otp_str_data.bt_func_support_policy_ext & BIT29)

//bit[30]
#ifdef _SUPPORT_VER_4_1_
#define IS_BT41                                 (otp_str_data.bt_func_support_policy_ext & BIT30)
#else
#define IS_BT41                                 FALSE
#endif

//bit[31]
#ifdef _SUPPORT_WL_BT_SCOREBOARD_
#define IS_USE_SCOREBOARD                       (otp_str_data.bt_func_support_policy_ext & BIT31)
#else
#define IS_USE_SCOREBOARD                       FALSE
#endif

/*---------------------------------------------------------*/
/* The Macro Group of "efuse_btongpio_init_d8 field"       */
/*---------------------------------------------------------*/
//bit[0]
#define IS_LOG_ENABLED_WHEN_POWER_ON            (otp_str_data.efuse_btongpio_init_d8 & BIT0)

//bit[1]
//#define IS_AUTO_SHUTDOWN_BT_LOG                 (otp_str_data.efuse_btongpio_init_d8 & BIT1)

#ifdef _SUPPORT_BTGPIO10_AS_LOG1_
//bit[1], after 8723D, there are 2 log_tx, new log is connect to bt gpio10
// original log tx is connect to btgpio8
#define SET_LOG_FROM_BTLOG1                       (otp_str_data.efuse_btongpio_init_d8 & BIT1)
#endif

//bit[2]
//#define ENABLE_OUTPUT_LED_HIGH_WHEN_LED_DISABLE       (otp_str_data.efuse_btongpio_init_d8 & BIT2)

#ifdef _SUPPORT_ENABLE_BTLOG1_WRITE_PAGE0_MUX_
//bit[2]
#define IS_ENABLE_BTLOG1_WRITE_PAGE0_MUX                (otp_str_data.efuse_btongpio_init_d8 & BIT2)
#endif

//bit[3]
// enable vendor gpio interrupt
#ifdef _ENABLE_VENDOR_GPIO_INTERRUPT_
#define IS_ENABLE_VENDOR_GPIO_INTERRUPT                (otp_str_data.efuse_btongpio_init_d8 & BIT3)
#endif

//bit[4]
#ifdef _SUPPORT_POLLING_BASED_LPM_L1_
#define IS_SUPPORT_USB_LPM_L1_FSM                       (otp_str_data.USB_LPM_Control & BIT1)
#else
#define IS_SUPPORT_USB_LPM_L1_FSM                       FALSE
#endif
#endif//_OTP_H_

