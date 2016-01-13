/***************************************************************************
 Copyright (C) MindTree Ltd.
 This module is a confidential and proprietary property of MindTree and
 a possession or use of this module requires written permission of MindTree.
 ***************************************************************************/

/**
 * \file
 *  Contains LC module initialization and shutdown routines. It also has
 *  utility functions and low power mode functions.
 */

/********************************* Logger *************************/
enum { __FILE_NUM__= 40 };
/********************************* Logger *************************/

//EFUSE: ROM bypass IQK
//EFUSE: AGC Table Initialization by PI/SI


/* ========================= Include File Section ========================= */
#include "lc_internal.h"
#include "bt_fw_os.h"
#include "vendor.h"
#include "lc.h"
#include "platform.h"
#include "bz_debug.h"
#include "led_debug.h"

#include "bt_fw_acl_q.h"

#include "lmp_pdu_q.h"

#include "bb_driver.h"

#include "mailbox.h"

#include "gpio.h"

#ifdef LE_MODE_EN
#include "le_ll_driver.h"
#endif

#include "power_control.h"
#include "h5.h"
#include "lmp_ch_assessment.h"

#ifdef _BT_ONLY_
#include "new_io.h"
#endif
#include "rlx4181.h"
#include "core_cm4.h"
#include "hci_vendor_defines.h"

#ifdef _ROM_CODE_PATCHED_
PF_ROM_CODE_PATCH_VOID rcp_lc_init_radio_phy_func = NULL;
PF_ROM_CODE_PATCH_FUNC rcp_rtk_read_modem_radio_reg = NULL;
PF_ROM_CODE_PATCH_FUNC rcp_rtk_write_modem_radio_reg = NULL;
#ifdef _NEW_MODEM_PI_ACCESS_
PF_ROM_CODE_PATCH_FUNC rcp_rtk_read_modem_radio_reg_pi = NULL;
PF_ROM_CODE_PATCH_FUNC rcp_rtk_write_modem_radio_reg_pi = NULL;
#endif
#ifdef _NEW_RFC_PI_ACCESS_
PF_ROM_CODE_PATCH_FUNC rcp_rtk_read_rfc_reg_pi = NULL;
PF_ROM_CODE_PATCH_FUNC rcp_rtk_write_rfc_reg_pi = NULL;
#endif
#ifdef _YL_MODEM_RSSI_MAPPING
PF_ROM_CODE_PATCH_FUNC rcp_lc_modem_rssi_mapping = NULL;
#endif
PF_ROM_CODE_PATCH_FUNC rcp_lc_set_modem_lna_constraint_on_off = NULL;
PF_ROM_CODE_PATCH_FUNC rcp_rtl8821_btrf_lok = NULL;
PF_ROM_CODE_PATCH_FUNC rcp_rtl8723_btfr_SetTxGainTable = NULL;
PF_ROM_CODE_PATCH_FUNC rcp_rtl8723_btrf_TxPowerTrack = NULL;
PF_ROM_CODE_PATCH_FUNC rcp_rtl8723_btrf_UpdateThermalValueTimer = NULL;
PF_ROM_CODE_PATCH_FUNC rcp_lc_psd_modem_init = NULL;
#endif

#ifdef _MODEM_LNA_CONSTRAINT_
UINT16 g_modem_reg_lna_con_on;
UINT16 g_modem_reg_lna_con_off;
UINT16 g_modem_reg_lna_con_addr;
#endif

UINT8 g_thermal_track_pause = 0;

#ifdef _BT_ONLY_
UINT8 g_cap_default = 0; /* TODO: must set default value from efuse */
#endif

extern UINT16 lc_power_ctr_config;

/* ==================== Macro declaration Section ===================== */
#define RF_OSCILLATOR_DELAY           26
#define BB_LC_RADIO_POS_OVER      0x0100

/* Hard coded value for mt_custom_radio_2 RF */
#define RSSI_MAX_VALUE 0x2D
#define RSSI_MIN_VALUE 0x00

#define RF_DELAY_H_INDEX        0x00
#define RF_DELAY_F_INDEX        0x01
#define RF_PHD_H_INDEX          0x02
#define RF_PHD_F_INDEX          0x03
#define CORR_DIS_H_INDEX        0x04
#define TRIG_1STID_INDEX        0x05
#define TRIG_RECFHS_INDEX       0x06
#define TRIG_2NDID_INDEX        0x07

#define RF_RX_BACKUP_STATUS_RPT_REG_NUM          5
const UINT16 lc_rf_rx_backup_status_rpt_reg[5] =
{
    BB_RX_BKPRT0_ADDR,
    BB_RX_BKPRT1_ADDR,
    BB_RX_BKPRT2_ADDR,
    BB_RX_BKPRT3_ADDR,
    BB_RX_BKPRT4_ADDR
};

const UINT16 g_lc_recover_reg_table[21] = {
    RF_DELAY_H_REGISTER,
    RF_DELAY_F_REGISTER,
    RF_PHD_H_REGISTER,
    RF_PHD_F_REGISTER,
    CORR_DIS_H_REGISTER,
    TRIG_1STID_REGISTER,
    TRIG_RECFHS_REGISTER,
    TRIG_2NDID_REGISTER,
    CORRELATOR_THRESOLD_REGISTER,
    0x178,
    0x17A,
    0x17C,
    0x17E,
    0x180,
    0x182,
    0x184,
    0x186,
    0x188,
    0x18A,
    0x18C,
    0x18E
};

#ifdef _IS_ASIC_
const UINT16 g_8703b_a_cut_rf_parameter_array_rx_gain_table[179] = {
    0x6015, //0
    0x1b00, 0x0041,     0x0002, 0x0043,     0x51c4, 0x08c5,     0x3106, 0x0907,
    0x55c8, 0x0409,     0x020a, 0x444b,     0x6ecc, 0x094d,     0x030e, 0x444f,
    0x0810, 0x1051,     0x6412, 0x3313,     0x0814, 0x2055,     0x7516, 0x3b97,
    0x6618, 0x0019,     0x265a, 0x111b,     0x661c, 0x001d,     0x279e, 0x111f,
    0x08e0, 0x5861,     0x1822, 0x08a3,     0x0024, 0x0025,     0x0026, 0x0027,//1~40
    0x6055, //41
    0x003f, 0x003e,     0x003d, 0x003c,     0x003b, 0x003a,     0x0039, 0x0038,
    0x0037, 0x0036,     0x0035, 0x0034,     0x0073, 0x00b2,     0x00f1, 0x0130,
    0x016f, 0x01ae,     0x01ed, 0x022c,     0x026b, 0x02aa,     0x02e9, 0x0328,
    0x0827, 0x0866,     0x08a5, 0x08e4,     0x0923, 0x0962,     0x09a1, 0x09e0,
    0x0a1f, 0x0a5e,     0x0a9d, 0x101c,     0x105b, 0x109a,     0x10d9, 0x1118,
    0x1157, 0x1196,     0x11d5, 0x1214,     0x1253, 0x1292,     0x12d1, 0x1310,
    0x134f, 0x138e,     0x13cd, 0x140c,     0x198b, 0x19ca,     0x1a09, 0x1a48,
    0x1a87, 0x1ac6,     0x1b05, 0x1b44,     0x1b83, 0x1bc2,     0x1c01, 0x1c40,//42~105
    0x6095, //106
    0x0000, 0x0001,     0x0002, 0x0003,     0x0004, 0x0005,     0x0006, 0x0007,
    0x0008, 0x0009,     0x020a, 0x050b,     0x000c, 0x000d,     0x040e, 0x000f,
    0x0010, 0x0e11,     0x0012, 0x0313,     0x0014, 0x0015,     0x0416, 0x0017,
    0x0018, 0x0b19,     0x0d1a, 0x001b,     0x001c, 0x001d,     0x011e, 0x001f,
    0x0020, 0x0021,     0x0022, 0x0023,     0x0024, 0x0025,     0x0026, 0x0027,
    0x0028, 0x0029,     0x002a, 0x002b,     0x002c, 0x002d,     0x002e, 0x002f,
    0x0030, 0x0c31,     0x0c32, 0x0f33,     0x0034, 0x0035,     0x0036, 0x0037,
    0x0038, 0x0039,     0x003a, 0x003b,     0x003c, 0x003d,     0x003e, 0x003f,//107~170
    0x60d5, //171
    0x0000, 0x0741,     0x0002, 0x0003,     0x0004, 0x0005,     0x0006  //172~178
    };
#endif

#ifdef _NEW_MODEM_DESIGN_PHY_SETTING_
// TODO:
/* 1. Default: old AGC or new AGC (OLD-like)
 * 2. For both old AGC and new AGC, it should be easy to force NO setting RxAGC
 * 3. EFUSE Table for old AGC and new AGC (OLD-like) should work for RTL8821A
 * 4. Should determined the New AGC EFUSE setting and data structure
 * 5. Can reserve new-AGC FSM table for
 *    a. use modem default value
 *    b. 0379-like mode
 *    c. RF_PD mode
 *    d. BYP_BPF mode
 *    e. others?
 *    And after writting this FSM table, it can then be modified by EFUSE (for minor change)
 * 6.
 */

#define NEW_MODEM_NEW_AGC_TABLE0_SIZE 40 /* 9-bit */
const UINT16 g_new_modem_new_agc_table0[NEW_MODEM_NEW_AGC_TABLE0_SIZE] =
#if 1
{   /*===== 0379-like =====*/
//    0x067, 0x001, 0x000, 0x111,  // state 0
//    0x06f, 0x001, 0x000, 0x001,  // state 0, reg_agc_stop is used, ini_gain wait 4.8us
    0x06c, 0x001, 0x000, 0x001,  // state 0, reg_agc_stop is used, ini_gain wait 0us
    0x147, 0x023, 0x0c4, 0x024,
    0x157, 0x010, 0x008, 0x111,
    0x1bb, 0x025, 0x00c, 0x111,
    0x020, 0x041, 0x190, 0x0cc,
    0x020, 0x081, 0x1d4, 0x0ee,
    0x198, 0x000, 0x099, 0x044,
    0x198, 0x000, 0x09e, 0x044,
    0x023, 0x161, 0x060, 0x022,
    0x000, 0x000, 0x000, 0x000   // state 9
};
#elif 0
{   /*===== RF_PD mode =====*/
//    0x067, 0x001, 0x100, 0x088, // state 0
    0x06f, 0x001, 0x100, 0x000, // state 0, reg_agc_stop is used
    0x147, 0x023, 0x0c4, 0x024,
    0x157, 0x010, 0x008, 0x111,
    0x1bb, 0x025, 0x00c, 0x111,
    0x020, 0x041, 0x190, 0x0cc,
    0x020, 0x081, 0x1d4, 0x0ee,
    0x198, 0x000, 0x099, 0x044,
    0x198, 0x000, 0x09e, 0x044,
    0x023, 0x161, 0x060, 0x022,
    0x000, 0x000, 0x000, 0x000
};
#elif 0
{   /*===== BYP_BPF =====*/
//    0x067, 0x001, 0x100, 0x088, // state 0
    0x06f, 0x001, 0x100, 0x000, // state 0, reg_agc_stop is used
    0x147, 0x023, 0x0c4, 0x024,
    0x157, 0x010, 0x008, 0x111,
    0x1bb, 0x025, 0x00c, 0x111,
    0x020, 0x041, 0x190, 0x0cc,
    0x020, 0x081, 0x1d4, 0x0ee,
    0x0c8, 0x006, 0x158, 0x0c4,
    0x088, 0x008, 0x09c, 0x044,
    0x023, 0x161, 0x060, 0x022,
    0x000, 0x000, 0x000, 0x000
}
#endif

#define NEW_MODEM_NEW_AGC_TABLE2_SIZE 64 /* asymetric, 6-bit; (32-8)=24 is required */
const UINT8 g_new_modem_new_agc_table2[NEW_MODEM_NEW_AGC_TABLE2_SIZE] =
#if 1
{   /*==== 2012/09/19 update for present RX Gain Table =====*/
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x08, 0x14, 0x00, 0x00, 0x10, 0x00,
    0x00, 0x38, 0x00, 0x0c, 0x00, 0x00, 0x10, 0x00,
    0x00, 0x2c, 0x34, 0x00, 0x00, 0x00, 0x04, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x30, 0x30, 0x3c, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};
#else
{   /*===== 0379-like =====*/
    0x00, 0x00, 0x07, 0x16, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x07, 0x16, 0x00, 0x00, 0x00, 0x00,
    0x39, 0x39, 0x00, 0x0f, 0x00, 0x00, 0x00, 0x00,
    0x2a, 0x2a, 0x31, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};
#endif
#define NEW_MODEM_NEW_AGC_TABLE3_SIZE 7 /* 10-bit */
const UINT16 g_new_modem_new_agc_table3[NEW_MODEM_NEW_AGC_TABLE3_SIZE] =
#if 1
{   /*===== 0379-like =====*/
    0x000, 0x01d, 0x000, 0x000, 0x000, 0x000, 0x000
};
#elif 0
{   /*===== BYP_BPF =====*/
    0x000, 0x01d, 0x000, 0x000, 0x000, 0x000, 0x000
};
#endif



#endif /* _NEW_MODEM_DESIGN_PHY_SETTING_ */

#ifndef RTL8723A_B_CUT
#define BT_RF_TX_POWER_TRACK_INDEX_OFFSET     6
#define BT_RF_CFO_TRACK_INDEX_OFFSET          6
#else
#define BT_RF_TX_POWER_TRACK_INDEX_OFFSET     8
#define BT_RF_CFO_TRACK_INDEX_OFFSET          8
#endif

#ifdef _RF_CONFIG_PROTECTION_
INT8 lc_rf_ctrl_stack = 0;
UINT16 lc_rf_off_mask = 0;
#ifdef _NEW_MODEM_PI_ACCESS_
UINT16 lc_host_addr_11_6_mask = 0;
#endif
#endif


#ifdef _NEW_MODEM_PI_ACCESS_
#ifdef _NEW_MODEM_PI_ACCESS_BY_VENDOR_REG_
#else
UINT8 g_efuse_modem_pi_enable = 0xff;
UINT8 g_efuse_modem_pi_addr0_convert = 1;
UINT8 g_efuse_modem_reg_rw_fast_mode = 0;
UINT8 g_modem_pi_si_sel_init_completed = 0;
#endif
UINT8 g_modem_init_completed = 0;
#endif


#ifdef _YL_TEST_NEW_MODEM_SRAM_DEBUG
#ifdef _DAPE_MODEM_SRAM_DBG_VER2
  #define MODEM_SRAM_DBG_ENTRY_NUM (256*2*8)
  SECTION_PATCH_SBSS_PRAM ALIGN(2048) MODEM_SRAM_DEBUG_ELEMENT_S_TYPE g_modem_sram_debug_array_physical[MODEM_SRAM_DBG_ENTRY_NUM];
#else
  #define MODEM_SRAM_DBG_ENTRY_NUM (256*2)
  SECTION_SRAM ALIGN(2048) MODEM_SRAM_DEBUG_ELEMENT_S_TYPE g_modem_sram_debug_array_physical[MODEM_SRAM_DBG_ENTRY_NUM];
#endif
#endif


#ifdef _YL_NEW_MODEM_SRAM_DEBUG
/* Memory Alignment Requirement:
 * PKT_SIZE 128: 128*2*8 = 2048 or 128*8 = 1024?
 * PKT_SIZE 64: 64*16 = 1024 or 512?
 * PKT_SIZE 32: 32*16 = 512 or 256?
 * PKT_SIZE 16: 16*16 = 256 or 128?
 */
MODEM_SRAM_DEBUG_ELEMENT_S_TYPE* g_modem_sram_debug_array = NULL;
UINT16 g_modem_sram_dbg_entry_num = 0;

//SECTION_SRAM ALIGN(2048) MODEM_SRAM_DEBUG_ELEMENT_S_TYPE g_modem_sram_debug_array[MODEM_SRAM_DBG_ENTRY_NUM];
UINT16 g_modem_sram_debug_captured_flag = 0;
UINT16 g_modem_sram_debug_log_count = 0;    // DBSS variables would be initialized as 0 //
UINT8 g_modem_sram_debug_en = 0;            // DBSS variables would be initialized as 0 //
UINT8 g_modem_sram_debug_log_en = 0;
UINT8 g_modem_sram_debug_xdh5_trig_crc_ok = 0;
UINT8 g_modem_sram_debug_xdh5_trig_en = 0;
UINT8 g_modem_sram_debug_xdh5_xdh3_3dh1_error_log_en = 0;
UINT8 g_modem_sram_debug_le_trig_en = 0;
UINT8 g_modem_sram_debug_le_trig_crc_ok = 0;
#ifdef _DAPE_MODEM_SRAM_DBG_VER2
UINT8 g_modem_sram_debug_modem_test = 0;
UINT8 g_modem_sram_debug_le_normal_link_trig = 0;
#endif
#endif



#ifdef _YL_MODEM_RSSI_MAPPING
/* (yilinli) this function changes the read rssi value to rssi_target
 * rssi_target: (0) -90dBm, (1) -88dBm, (2) -86dBm, ...
 * this is for AFH, LE RX Power, Inquiry with RSSI, ... etc
 */
#define MODEM_RSSI_MAPPING_TARGET_DBM (-90) // -90dBm //
UINT8 lc_modem_rssi_mapping(UINT8 rssi_with_offset)
{
    INT16 rssi_target;

#ifdef _ROM_CODE_PATCHED_
    // Reserved com code patch by yilinli //
    if (rcp_lc_modem_rssi_mapping != NULL)
    {
        return rcp_lc_modem_rssi_mapping(NULL, rssi_with_offset);
    }
#endif

    rssi_target = (INT16)(rssi_with_offset<<1)+(INT16)(otp_str_data.efuse_modem_rssi0_pin_dBm)-MODEM_RSSI_MAPPING_TARGET_DBM;
    if (rssi_target<0)
    {
        rssi_target = 0;
    }
    rssi_target = rssi_target>>1;

    return (UINT8)rssi_target;
}
#endif

#ifdef _NEW_MODEM_PSD_SCAN_
INT16 g_modem_psd_offset = MODEM_PSD_DBM_OFFSE + MODEM_PSD_AVG_DBM_OFFSET; // TODO: be tuned
MODEM_PSD_REPORT_ELEMENT_S_TYPE* g_modem_psd_report_array = NULL;
UINT16 g_modem_psd_report_entry_num = 0;

#ifdef _YL_TEST_NEW_MODEM_PSD_SCAN_FOR_AFH_
#define MODEM_PSD_REPORT_ENTRY_NUM (9)
ALIGN(16) SECTION_SRAM MODEM_PSD_REPORT_ELEMENT_S_TYPE g_modem_psd_report_array_physical[MODEM_PSD_REPORT_ENTRY_NUM];
#endif

#endif

/* ================== Static Function Prototypes Section ================== */
/**
 * Programs the radio parameters to the radio.
 *
 * \param None.
 *
 * \return None.
 */
void lc_update_radio_timings(void)
{

#ifdef _CCH_SC_ECDH_P256_HW_TIMING_TEST_ONLY
#ifdef _BRUCE_RF_DELAY_TIMING
UINT8 x = 5;   // early 5us
#else
UINT8 x = 48;
#endif
UINT32 temp1, temp2, temp3, temp4, temp5, temp6;
UINT16 load_value_conn = 0;

temp1 = otp_str_data.bw_rf_delay_vals[RF_DELAY_H_INDEX]; // C6
temp2 = otp_str_data.bw_rf_delay_vals[RF_DELAY_F_INDEX]; // C8
temp3 = otp_str_data.bw_rf_delay_vals[CORR_DIS_H_INDEX]; // CE
temp4 = otp_str_data.bw_rf_delay_vals[TRIG_1STID_INDEX]; // D0
temp5 = otp_str_data.bw_rf_delay_vals[TRIG_RECFHS_INDEX];// D2
temp6 = otp_str_data.bw_rf_delay_vals[TRIG_2NDID_INDEX]; // D4


otp_str_data.bw_rf_delay_vals[RF_DELAY_H_INDEX] = temp1 - ( (x<<8) | x);
otp_str_data.bw_rf_delay_vals[RF_DELAY_F_INDEX] = temp2 - ( (x<<8) | x);
otp_str_data.bw_rf_delay_vals[CORR_DIS_H_INDEX] = temp3 - x;
otp_str_data.bw_rf_delay_vals[TRIG_1STID_INDEX] = temp4 + x;
otp_str_data.bw_rf_delay_vals[TRIG_RECFHS_INDEX] = temp5 + ( (x<<9) | x);
otp_str_data.bw_rf_delay_vals[TRIG_2NDID_INDEX] = temp6 + x;


//load_value_conn = (((temp3>>9)&0x03)<<7) | (temp5>>9);


RT_BT_LOG(RED, YL_DBG_HEX_12, 12,
temp1 , otp_str_data.bw_rf_delay_vals[RF_DELAY_H_INDEX],
temp2 , otp_str_data.bw_rf_delay_vals[RF_DELAY_F_INDEX],
temp3 , otp_str_data.bw_rf_delay_vals[CORR_DIS_H_INDEX],
temp4 , otp_str_data.bw_rf_delay_vals[TRIG_1STID_INDEX],
temp5 , otp_str_data.bw_rf_delay_vals[TRIG_RECFHS_INDEX],
temp6 , otp_str_data.bw_rf_delay_vals[TRIG_2NDID_INDEX]);

#endif

    BB_write_baseband_register(RF_DELAY_H_REGISTER,
                               otp_str_data.bw_rf_delay_vals[RF_DELAY_H_INDEX]);
    BB_write_baseband_register(RF_DELAY_F_REGISTER,
                               otp_str_data.bw_rf_delay_vals[RF_DELAY_F_INDEX]);
    BB_write_baseband_register(RF_PHD_H_REGISTER,
                               otp_str_data.bw_rf_delay_vals[RF_PHD_H_INDEX]);
    BB_write_baseband_register(RF_PHD_F_REGISTER,
                               otp_str_data.bw_rf_delay_vals[RF_PHD_F_INDEX]);
    BB_write_baseband_register(CORR_DIS_H_REGISTER,
                               otp_str_data.bw_rf_delay_vals[CORR_DIS_H_INDEX]);
    BB_write_baseband_register(TRIG_1STID_REGISTER,
                               otp_str_data.bw_rf_delay_vals[TRIG_1STID_INDEX]);
    BB_write_baseband_register(TRIG_RECFHS_REGISTER,
                               otp_str_data.bw_rf_delay_vals[TRIG_RECFHS_INDEX]);
    BB_write_baseband_register(TRIG_2NDID_REGISTER,
                               otp_str_data.bw_rf_delay_vals[TRIG_2NDID_INDEX]);

    /* Write threshold register with the threshold values for
    correlation and sync detection in EDR packets */
    BB_write_baseband_register(CORRELATOR_THRESOLD_REGISTER,
                                otp_str_data.bw_rf_delay_vals[8]);
#ifdef _DAPE_TEST_NEW_HW
    UINT16 temp_val = BB_read_baseband_register(ENCRYPTION_ENABLE_REGISTER);
    temp_val |= BIT14;
    BB_write_baseband_register(ENCRYPTION_ENABLE_REGISTER, temp_val);
#endif

#ifndef _IS_ASIC_

    /* write RF parameters to avoid RX/TX turn on at the same time */
#if defined(_USE_RLE0380_RF_) || defined(_USE_RLE0546_RF_)
    /* 0380 test chip doesn't need Per-Packet Control RF Command (0x179~0x18A)*/
    BB_write_baseband_register(0x178, 0x0000); //DA on @ TX mode
    BB_write_baseband_register(0x17a, 0x0000);
    BB_write_baseband_register(0x17c, 0x0000); //AD off @ Tx mode
    BB_write_baseband_register(0x17e, 0x0000);
    BB_write_baseband_register(0x180, 0x0000); //RF LNA Low gain @ Tx mode
    BB_write_baseband_register(0x182, 0x0000);
    BB_write_baseband_register(0x184, 0x0000); //DA off @ Rx mode
    BB_write_baseband_register(0x186, 0x0000);
    BB_write_baseband_register(0x188, 0x0000); //AD on @ Rx mode
    BB_write_baseband_register(0x18a, 0x0000);
#elif defined(_FPGA_WITH_RLE0379_RFE_)
#ifdef _TEST_ADAPTIVITY_FUNC_2
    BB_write_baseband_register(0x178, 0x0000); //DA on @ TX mode
    BB_write_baseband_register(0x17a, 0x0000);
    BB_write_baseband_register(0x17c, 0x0000); //AD off @ Tx mode
    BB_write_baseband_register(0x17e, 0x0000);
    BB_write_baseband_register(0x180, 0x0000); //RF LNA Low gain @ Tx mode
    BB_write_baseband_register(0x182, 0x0000);
    BB_write_baseband_register(0x184, 0x0000); //DA off @ Rx mode
    BB_write_baseband_register(0x186, 0x0000);
    BB_write_baseband_register(0x188, 0x0000); //AD on @ Rx mode
    BB_write_baseband_register(0x18a, 0x0000);
#else
    BB_write_baseband_register(0x178, 0x64ab); //DA on @ TX mode
    BB_write_baseband_register(0x17a, 0x039e);
    BB_write_baseband_register(0x17c, 0xa54f); //AD off @ Tx mode
    BB_write_baseband_register(0x17e, 0x039c);
    BB_write_baseband_register(0x180, 0x0000); //RF LNA Low gain @ Tx mode
    BB_write_baseband_register(0x182, 0x0381);
    BB_write_baseband_register(0x184, 0x64a8); //DA off @ Rx mode
    BB_write_baseband_register(0x186, 0x039e);
    BB_write_baseband_register(0x188, 0xa54a); //AD on @ Rx mode
    BB_write_baseband_register(0x18a, 0x039c);
#endif
#else
   !!!!! UNDEFINED !!!!!
#endif

#endif

    return;
}

void lc_save_bluewiz_phy_settings(UINT16 *buf)
{
    UINT8 i;
    for (i = 0; i < 21; i++)
    {
        buf[i] = BB_read_baseband_register(g_lc_recover_reg_table[i]);
    }
}

void lc_load_bluewiz_phy_settings(UINT16 *buf)
{
    UINT8 i;
    for (i = 0; i < 21; i++)
    {
        BB_write_baseband_register(g_lc_recover_reg_table[i], buf[i]);
    }
}

#ifdef _NEW_MODEM_PI_ACCESS_
#ifndef _NEW_MODEM_PI_ACCESS_BY_VENDOR_REG_
// shall be executed before any modem access //
void rtk_modem_pi_enable_init(void)
{
    EFUSE_MODEM_SETTING_1_S efuse_modem_setting_1;
    *(UINT8*)&efuse_modem_setting_1 = otp_str_data.efuse_modem_setting_1_d8;
    g_efuse_modem_pi_enable = efuse_modem_setting_1.modem_pi_enable;
    g_efuse_modem_reg_rw_fast_mode = efuse_modem_setting_1.modem_reg_rw_fast_mode;
#if defined(_YL_TEST_NEW_MODEM_PI_ACCESS_) || defined(_YL_TEST_NEW_MODEM_PSD_SCAN_FOR_AFH)
    g_efuse_modem_pi_enable = 1;
#endif
    RT_BT_LOG(GREEN, MDM_LOG_011, 1, g_efuse_modem_pi_enable);
//    RT_BT_LOG(GREEN, YL_DBG_HEX_1, 1, g_efuse_modem_pi_enable);
//    g_efuse_modem_pi_enable = 1;
}
#endif
#endif


/**
 * Write rf/modem Rx backup status register. This register is used to monitor
 * rf/modem registers to the bluewiz, then the data port can be updated every
 * rx_en on to off edge
 *
 * \param: index -  the index of backup register
 *         addr - rf/modem address
 *         legacy_on - legacy mode on/off
 *         le_on - le mode on/off
 *
 * \return None.
 */
void lc_write_rf_rx_backup_status_rpt_reg(UINT8 index, UINT16 addr,
        UINT8 legacy_on, UINT8 le_on)
{
    UINT16 content;

    if (index >= RF_RX_BACKUP_STATUS_RPT_REG_NUM)
    {
        return;
    }

    content = ((le_on & 0x1) << 9) | ((legacy_on & 0x1)  << 8) | (addr & 0x7F);
    BB_write_baseband_register(lc_rf_rx_backup_status_rpt_reg[index], content);
}

/**
 * Read rf/modem Rx backup status register
 *
 * \param: index -  the index of backup register
 *
 * \return None.
 */
UINT16 lc_read_rf_rx_backup_status_rpt_reg(UINT8 index)
{
    if (index >= RF_RX_BACKUP_STATUS_RPT_REG_NUM)
    {
        return 0xDEAD;
    }
    else
    {
        return BB_read_baseband_register(lc_rf_rx_backup_status_rpt_reg[index]);
    }
}


#ifdef _MODEM_LNA_CONSTRAINT_
void lc_set_modem_lna_constraint_on_off(UINT8 is_on)
{
#ifdef _ROM_CODE_PATCHED_
    // Reserved com code patch by yilinli //
    if (rcp_lc_set_modem_lna_constraint_on_off != NULL)
    {
        if (rcp_lc_set_modem_lna_constraint_on_off((void*)&is_on))
        {
            return;
        }
    }
#endif

#ifdef _NEW_MODEM_PI_ACCESS_
    DEF_CRITICAL_SECTION_STORAGE;

    EFUSE_MODEM_SETTING_1_S efuse_modem_setting_1;
    *(UINT8*)&efuse_modem_setting_1 = otp_str_data.efuse_modem_setting_1_d8;
    UINT16 temp_modem_d16 = is_on ? g_modem_reg_lna_con_on : g_modem_reg_lna_con_off;

    MINT_OS_ENTER_CRITICAL();

    if (efuse_modem_setting_1.modem_lna_con_on_by_bk_reg)
    {
#ifndef _NEW_MODEM_DESIGN_AFTER_RTL8821B_TC_
        BB_write_baseband_register(0x18c, 0x0000);
        BB_write_baseband_register(0x18e, 0x03C0);
        BB_write_baseband_register(0x184, 0x0002);
        BB_write_baseband_register(0x186, 0x03C0);
        BB_write_baseband_register(0x188, temp_modem_d16);
        BB_write_baseband_register(0x18a, 0x03C0 | TRANS_MODEM_REG(g_modem_reg_lna_con_addr));
#else
        BB_write_baseband_register(0x188, temp_modem_d16);
        BB_write_baseband_register(0x18a, 0x03C0 | TRANS_MODEM_REG(g_modem_reg_lna_con_addr));
#endif
        bBtLNAConstraint = is_on;
    }
    else
    {
        // TODO: more options is to be implemented ...
        bBtLNAConstraint = 0;
    }

    MINT_OS_EXIT_CRITICAL();

#endif

    return;
}

void lc_force_modem_lna_constraint_off(void)
{
#ifdef _NEW_MODEM_PI_ACCESS_
    DEF_CRITICAL_SECTION_STORAGE;
    MINT_OS_ENTER_CRITICAL();

    lc_set_modem_lna_constraint_on_off(0);
    EFUSE_MODEM_SETTING_1_S efuse_modem_setting_1;
    *(UINT8*)&efuse_modem_setting_1 = otp_str_data.efuse_modem_setting_1_d8;
    efuse_modem_setting_1.modem_lna_con_on_by_bk_reg = 0;
    otp_str_data.efuse_modem_setting_1_d8 = *(UINT8*)&efuse_modem_setting_1;

    MINT_OS_EXIT_CRITICAL();
#endif
}
#endif



UINT16 cal_d_sq(INT16 i1, INT16 q1, INT16 i2, INT16 q2)
{
    INT32 di, dq;
    di = i1-i2;
    dq = q1-q2;
    di = di*di + dq*dq;
    return MIN(di, 0xFFFF);
}

UINT16 cal_sqrt_u_16_3f(UINT16 N)
{ // TODO: to be merged with patch_cal_d_sq ??
  UINT32 x,y;
  INT8 log2N;
  UINT32 Nx64 = ((UINT32)N)<<6;
  UINT16 jj;

    if (N == 0)
    {
        x = 0;
    }
    else
    {
        for (log2N = 24; log2N >=0; log2N--)
        {
            if (Nx64>=(1<<log2N))
            {
                break;
            }
        }
        x = 1<<((log2N>>1)+1);
        for (jj = 0; jj<1000; jj++)
        {
            if (x==0)
            {
                break;
            }
            y = (x + (Nx64/x))>>1;
            if (y >= x)
            {
                break;
            }
            x = y;
        }
    }

    return x;
}


// NOTE: M = 0~3
void max_min_d_u_8_Mf(INT8 X0, INT8 Y0, UINT8 array_size, INT8 *X_array, INT8 *Y_array, UINT8 M, UINT8 *dmax, UINT8 *dmin)
{
    UINT8 ii;
    UINT16 d_temp;
    UINT16 dmax_temp = 0x00;
    UINT16 dmin_temp = 0xFFFF;

    for (ii = 0; ii<array_size; ii++)
    {
        d_temp = cal_sqrt_u_16_3f(cal_d_sq(X0, Y0, X_array[ii], Y_array[ii]));
        dmax_temp = MAX(dmax_temp, d_temp);
        dmin_temp = MIN(dmin_temp, d_temp);
    }
    dmax_temp = dmax_temp>>(3-M);
    dmin_temp = dmin_temp>>(3-M);
    dmax_temp = MIN(dmax_temp, 0xFF); // Saturate to avoid overflow //
    dmin_temp = MIN(dmin_temp, 0xFF); // Saturate to avoid overflow //
    (*dmax) = dmax_temp;
    (*dmin) = dmin_temp;
}

/**
 * Selects the radio-type, and calls the appropriate function
 * to initialize the radio.
 *
 * \param None.
 *
 * \return None.
 */
#ifdef _CCH_RETENTION_FLOW_FOR_DLPS_
void lc_init_radio(UINT8 dlps_flow)
#else
void lc_init_radio(void)
#endif
{
    UINT16 data;
#ifdef _ENABLE_BTON_POWER_SAVING_
    GENERAL_CONTROL_S_TYPE bt_general_ctrl;
    BTON_POW_CTRL_REG_S_TYPE pow_ctrl;

    //the setting value loaded from efuse
    bt_general_ctrl.d16 = otp_str_data.general_control;
#endif
    lc_power_ctr_config = 0;

    /* Select sys clock */
    /* bit 2 should be 0(or with 0x0000) for 13MHz */
    /* bit 2 should be 1(or with 0x0004) for 16MHz */
    lc_power_ctr_config |= otp_str_data.bw_rf_radio_sys_clk_sel << 3;

    /* Select low power clock */
    /* bit 3 should be 1(or with 0x0008) for 32KHz */
    /* bit 3 should be 0(or with 0x0000) for 32.768KHz */
    lc_power_ctr_config |= otp_str_data.bw_rf_low_clk_frac << 2;

    /* Osc start up delay */
#ifdef _YL_LPS
    // TODO: g_efuse_lps_setting_3 had better been assigned eariler than here
    if(g_efuse_lps_setting_3.lc_init_radio_osc_delay_opt)
    {
        lc_power_ctr_config |= (otp_str_data.bw_rf_osc_delay) << 8;
    }
    else
#else
    if(1)
#endif
    {
        lc_power_ctr_config |= (RF_OSCILLATOR_DELAY) << 8;
    }

    //RT_BT_LOG(GREEN, LC_RF_245, 1, lc_power_ctr_config);

    BB_write_baseband_register(POWER_CONTROL_REGISTER, lc_power_ctr_config);

#if defined(_USE_RLE0380_RF_) || defined(_USE_RLE0546_RF_)
    /* disable AFH/RF clock gate and turn on LDO of AD in 0380 test chip */
    VENDOR_WRITE(BTON_TIMER_CTRL_REG, 0x0016d7d7);
#endif

#ifdef _ENABLE_BTON_POWER_SAVING_
    //Disable AFH and RF clock gate
    pow_ctrl.d32 = VENDOR_READ(BTON_POW_CTRL_REG);
    pow_ctrl.b.gating_xtal2afe_en = 0;
    pow_ctrl.b.gating_xtal2rf_en = 0;
    VENDOR_WRITE(BTON_POW_CTRL_REG,pow_ctrl.d32);

    //Turn on LDO of AD
#ifndef _8821A_BTON_DESIGN_
// TODO: to be reviewed
    if (bt_general_ctrl.b.afe_ldo_hw_ctrl_en)
    {
        CORE_AND_AFE_LDO_CTRL_S_TYPE bton_core_afe_ldo_ctrl_reg;

        //Enable hw control AFE LDO and isolation
        bton_core_afe_ldo_ctrl_reg.d32 = VENDOR_READ(BTON_CORE_AFE_LDO_CTRL_REG);
        bton_core_afe_ldo_ctrl_reg.b.afe_ldo_hw_ctr_en = 1;
        VENDOR_WRITE(BTON_CORE_AFE_LDO_CTRL_REG,bton_core_afe_ldo_ctrl_reg.d32);
    }
    else
    {
        trun_on_off_afe_ldo(ON);
    }
#endif
#endif

    /* Write Radio select register */
    BB_write_baseband_register(RADIO_SELECT_REGISTER, (BB_LC_RADIO_POS_OVER|7) );


#ifndef FOR_SIMULATION
#ifndef _ENABLE_BTON_POWER_SAVING_
    pf_delay(1); /* 1 msec delay */
#endif
#endif

    //RF_CTRL_SET_RF_OFF;
    //RF_CTRL_WAIT_RF_OFF_READY;


#ifdef _ROM_CODE_PATCHED_
    /* Reserved com code patch by yilinli:
      * e.g.
      *    1. new modem initialization and fine-tune
      */
    if (rcp_lc_init_radio_phy_func != NULL)
    {
        rcp_lc_init_radio_phy_func();
    }
    else
#endif
    {

#ifdef _NEW_MODEM_PI_ACCESS_
#ifndef _NEW_MODEM_PI_ACCESS_BY_VENDOR_REG_
        rtk_modem_pi_enable_init();
#endif
#endif

#ifndef _IS_ASIC_
#ifdef _CCH_RETENTION_FLOW_FOR_DLPS_
        if(dlps_flow == FALSE)
#endif
        {
            rtk_modem_init();
            rtk_rf_init();
        }
#else
        if (IS_RF_INIT)
        {
            if(USE_8703B_A_CUT_RF_PARA)
            {
                rtl8723_btrf_PHYInit();
                rtl8703b_acut_rf_parameter_parse();
            }
            else
            {
                rtl8723_btrf_PHYInit();
                rtl8723_btrf_RxAGCTableInit();
                rtl8723_btrf_TxGainTableInit();
                rtl8723_btrf_RFIQK();
                rtl8723_btrf_start_thermal_timer();
            }
        }
#endif /* #ifdef _IS_ASIC_ */

#ifdef _NEW_MODEM_PI_ACCESS_ // for warning when accessing modem address at run-time //
        g_modem_init_completed = 1;
#endif

#ifdef _YL_TEST_NEW_MODEM_SRAM_DEBUG
      g_modem_sram_debug_array = g_modem_sram_debug_array_physical;
      g_modem_sram_dbg_entry_num = MODEM_SRAM_DBG_ENTRY_NUM;
#endif

    }

#if 0
#ifdef _MODEM_HCI_VENDOR_8821A_LOK_
    // TODO: to be modified in PATCH
    {
        EFUSE_RF_SETTING_1_S efuse_rf_setting_1;
        *(UINT8*)&efuse_rf_setting_1 = otp_str_data.efuse_rf_setting_1_d8;
        if (efuse_rf_setting_1.execute_lok_at_boot)
        {
            UINT8 ii = 0;
            UINT16 rf0x21 = 0;
            UINT16 lok_i_acc = 0;
            UINT16 lok_q_acc = 0;
            UINT16 lok_avg_num = (1<<efuse_rf_setting_1.execute_lok_at_boot_avg_num);

            UINT16 rfc0x02 = RTK_READ_RF_REG((0x02));
            UINT16 rfc0x3f = RTK_READ_RF_REG((0x3f));

            if (efuse_rf_setting_1.execute_lok_at_boot_dummy_lok)
            {
                // (yilinli) DUMMY LOK to avoid Bad Result of First LOK (WHY?) //
                rf0x21 = rtl8821_btrf_lok(1, rfc0x3f&0xFF, rfc0x02>>8); // execute LOK using EFUSE CH and TXGAIN //
            }


            for (ii = 0; ii<lok_avg_num; ii++)
            {
                rf0x21 = rtl8821_btrf_lok(0, rfc0x3f&0xFF, rfc0x02>>8); // execute LOK using EFUSE CH and TXGAIN //
//                if (efuse_rf_setting_1.execute_lok_at_boot_avg_num == 0)
//                {
//                    break;
//                }
//                else
                {
                    lok_i_acc += ((rf0x21>>10)&0x1F);
                    lok_q_acc += ((rf0x21>>5)&0x1F);
                }
            }
//            if (efuse_rf_setting_1.execute_lok_at_boot_avg_num)
            {
                rf0x21 &= 0x1F;
                lok_i_acc = (  (lok_i_acc+(lok_avg_num>>1))>>efuse_rf_setting_1.execute_lok_at_boot_avg_num  ) & 0x1F;
                lok_q_acc = (  (lok_q_acc+(lok_avg_num>>1))>>efuse_rf_setting_1.execute_lok_at_boot_avg_num  ) & 0x1F;
                rf0x21 |= ((lok_i_acc)<<10);
                rf0x21 |= ((lok_q_acc)<<5);
                rf0x21 |= BIT15;
//                RT_BT_LOG(GREEN, YL_DBG_HEX_5, 5, efuse_rf_setting_1.execute_lok_at_boot_avg_num, lok_avg_num, lok_i_acc, lok_q_acc, rf0x21);
                RTK_WRITE_RF_REG((0x021),rf0x21);
            }
        }
    }
#endif
#endif

#ifdef _MODEM_LNA_CONSTRAINT_
    {
#ifdef _NEW_MODEM_PI_ACCESS_
        MODEM_REG_S_TYPE modem_reg;
        EFUSE_MODEM_SETTING_1_S efuse_modem_setting_1;
        *(UINT8*)&efuse_modem_setting_1 = otp_str_data.efuse_modem_setting_1_d8;

        /* switch to page 2 */
   #ifndef _NEW_MODEM_DESIGN_AFTER_RTL8821B_TC_
        rtk_write_modem_radio_reg(0x00, TYPE_MODEM, 0x0002);
        modem_reg.d16 = rtk_read_modem_radio_reg(TRANS_MODEM_REG(0x48), TYPE_MODEM);
        g_modem_reg_lna_con_off = modem_reg.d16;
        modem_reg.p2reg48.reg_lna_constraint = efuse_modem_setting_1.modem_lna_con_on_value;
        g_modem_reg_lna_con_on = modem_reg.d16;
        g_modem_reg_lna_con_addr = 0x48;
        /* switch to page 0 */
        rtk_write_modem_radio_reg(0x00, TYPE_MODEM, 0x0000);
   #else
        rtk_write_modem_radio_reg(0x00, TYPE_MODEM, 0x0000);
        modem_reg.d16 = rtk_read_modem_radio_reg(TRANS_MODEM_REG(0x0A), TYPE_MODEM);
        g_modem_reg_lna_con_off = modem_reg.d16;
        modem_reg.reg0a.reg_lna_constraint = efuse_modem_setting_1.modem_lna_con_on_value;
        g_modem_reg_lna_con_on = modem_reg.d16;
        g_modem_reg_lna_con_addr = 0x0A;
   #endif
#endif
    }
#endif

    //RF_CTRL_CLR_RF_OFF;

    /* add by austin */
#ifndef _NEW_MODEM_PI_ACCESS_BY_VENDOR_REG_
#if defined(_CCH_CFOE_RETRY_) || defined(_INQRES_DBG_CCH_) || defined(_YL_TEST_MODEM_RX_REPORT)
    lc_write_rf_rx_backup_status_rpt_reg(0, TRANS_MODEM_REG(0x68), 1, 0);
    lc_write_rf_rx_backup_status_rpt_reg(1, TRANS_MODEM_REG(0x6A), 1, 0);
#endif
    lc_write_rf_rx_backup_status_rpt_reg(2, TRANS_MODEM_REG(0x6C), 1, 0);
    lc_write_rf_rx_backup_status_rpt_reg(3, TRANS_MODEM_REG(0x6E), 1, 0);
    lc_write_rf_rx_backup_status_rpt_reg(4, TRANS_MODEM_REG(0x70), 1, 0);
#endif
    BB_write_baseband_register(RADIO_SELECT_REGISTER, (BB_LC_RADIO_POS_OVER|7) );

#ifdef _INFINITE_LOOP_PROTECT_
    UINT32 loop_count = 0;
#endif

    while(1)
    {
#ifndef FOR_SIMULATION
        pf_delay(1); /* 1 msec dealy */
#endif

        data = BB_read_baseband_register(RADIO_CONTROL_REGISTER);
        if((data & 0x0100) == 0x0100)
        {
            break;
        }

#ifdef _INFINITE_LOOP_PROTECT_
        if (loop_count > 100000)
        {
            RT_BT_LOG(RED, RF_MSG_INIT_ERR, 0, 0);
            break;
        }
        loop_count++;
#endif
    }

#ifdef _CCH_RETENTION_FLOW_FOR_DLPS_
    if(dlps_flow == FALSE)
#endif
    {
        /* Program Correlator. */
        BB_write_baseband_register(CORRELATOR_THRESOLD_REGISTER, 0x003C);
    }


    lc_psd_modem_init();

    return;
}

// TODO:
/*****************************************************************
  case 1: modem SPI, RFC SPI
     * set pi/si = 0
     * read/write RFC:
            As before, 不用管host_addr[11:6]
     * read/write Modem:
            As before, 不用管host_addr[11:6]
  case 2: modem PI, RFC SPI
     * set pi/si = 1
     * read/write RFC:
            As before, 不用管host_addr[11:6]
     * write MODEM:
           set 0xDA: host_addr[11:6] and host_datain[15:8]/host_dataout[15:8]
           set 0xD6 as before
           不需等0xBE[7]為1
          亦不需要額外delay?
          read 0xD6 to clear interrupt status
     * read MODEM:
           set 0xDA: host_addr[11:6]
           set 0xD6 as before
           不需等0xBE[7]為1
           亦不需要額外delay?
          read 0xDA: host_datain[15:8]/host_dataout[15:8]
          read 0xD6 as before

   Pi_addr[11:8]:
     0:     page 0
     1:     page 1
     2:     page 2
     3~7:   reserved
     8:     aagc_tab0 (new aagc)
     9:     aagc_tab1 (new aagc)
     10:    aagc_tab2 (new aagc)
     11:    aagc_tab3 (new aagc)
     12:    AGC_tab0 (old aagc)
     13~15: reserved


   另外需要修改的: (?)
      RF_CTRL_SET_RF_OFF_INVALID,
      RF_CTRL_SET_RF_OFF,
      RF_CTRL_CLR_RF_OFF,
      RF_CTRL_WAIT_RF_OFF_READY
      lc_write_radio_reg();
      lc_read_radio_reg();
      AAGC Tables
      HCI Vendor Command
      Mailbox Command
      PHY Init

******************************************************************/

/**
 * Writes a radio register through the baseband interface.
 *
 * \param addr Address of the radio register to write.
 * \param val Value to be written.
 *
 * \return None.
 */
void lc_write_radio_reg(UCHAR addr, UINT32 val)
{
    UINT16 write_val;

    if (!(addr & 0x40))
    {
        /* only check RF register */
        RF_CTRL_SET_RF_OFF;
        RF_CTRL_WAIT_RF_OFF_READY;
    }
#if 0
#ifdef _NEW_MODEM_PI_ACCESS_
    if (g_efuse_modem_pi_enable)
    {
        write_val = BB_read_baseband_register(EXTENDED_HOST_ACCESS_REGISTER);
        write_val = (write_val & 0xFF00) | ((val & 0x0000FF00) >> 8);
    }
    else
#endif
#endif
    {
#ifdef _NEW_MODEM_PI_ACCESS_
        write_val = ((val & 0x0000FF00) >> 8) | RF_CTRL_RF_OFF_BIT | RF_CTRL_HOST_ADDR_11_6_BITS;
#else
        write_val = ((val & 0x0000FF00) >> 8) | RF_CTRL_RF_OFF_BIT;
#endif
    }
    /* val[15:8] = 7:0 */
    BB_write_baseband_register(EXTENDED_HOST_ACCESS_REGISTER, write_val);

    write_val = (UINT16)(0x8000 | (addr << 8) | (val & 0x000000FF));
    /* r/w bit - 15. addr = 14:8 val[7:0] = 7:0 */
    BB_write_baseband_register(RADIO_ACCESS_REGISTER,write_val);

#ifdef _INFINITE_LOOP_PROTECT_
    UINT32 count = 0;
#endif

#ifdef _NEW_MODEM_PI_ACCESS_
#ifdef _NEW_MODEM_PI_ACCESS_BY_VENDOR_REG_
    while (1)
#else
    while (!((addr & 0x40) && g_efuse_modem_pi_enable && g_efuse_modem_reg_rw_fast_mode))
#endif
#else
    while (1)
#endif
    {
        UINT16 data;
        /* Poll the radio_control_register till the siw_host_access_dome
           bit is 1. */

        data = BB_read_baseband_register(INTERRUPT_REGISTER);
#ifdef _NEW_MODEM_PI_ACCESS_
#ifdef _NEW_MODEM_PI_ACCESS_BY_VENDOR_REG_
        if (data & 0x0080)
#else
        if ( (data & 0x0080) || ((addr & 0x40) && (g_efuse_modem_pi_enable)))
#endif
#else
        if (data & 0x0080)
#endif
        {
            BB_read_baseband_register(RADIO_CONTROL_REGISTER);
            break;
        }

#ifdef _INFINITE_LOOP_PROTECT_
        if (count > 1000000)
        {
            RT_BT_LOG(RED, RF_MSG_WRITE_REG_ERR, 2, addr, val);
            break;
        }
        count++;
#endif
    }

    if (!(addr & 0x40))
    {
        /* only check RF register */
        RF_CTRL_CLR_RF_OFF;
    }

    return;
}


/**
 * Reads a radio register through the baseband interface.
 *
 * \param addr Address of the radio register to read.
 *
 * \return return_val Value of the register read.
 */
UINT32 lc_read_radio_reg(UCHAR addr)
{
    UINT16 write_val;
    UINT32 return_val = 0;

    if (!(addr & 0x40))
    {
        /* only check RF register */
        RF_CTRL_SET_RF_OFF;
        RF_CTRL_WAIT_RF_OFF_READY;
    }

    write_val = (UINT16) ((UINT16) addr << 8);
    BB_write_baseband_register(RADIO_ACCESS_REGISTER, write_val);

#ifdef _INFINITE_LOOP_PROTECT_
    UINT32 count = 0;
#endif
#ifdef _NEW_MODEM_PI_ACCESS_
#ifdef _NEW_MODEM_PI_ACCESS_BY_VENDOR_REG_
    while (1)
#else
    while ((!(addr & 0x40)) || (!g_efuse_modem_pi_enable))
#endif
#else
    while (1)
#endif
    {
        UINT16 data;
        /* Poll the radio_control_register till the siw_host_access_done
            bit is 1. If it is 1 , then break out out of infinite loop. */
        data = BB_read_baseband_register(INTERRUPT_REGISTER);
        if (data & 0x0080)
        {
            break;
        }

#ifdef _INFINITE_LOOP_PROTECT_
        if (count > 1000000)
        {
            RT_BT_LOG(RED, RF_MSG_READ_REG_ERR, 1, addr);
            break;
        }
        count++;
#endif
    }

    write_val = BB_read_baseband_register(EXTENDED_HOST_ACCESS_REGISTER);
    return_val = (write_val & 0x00ff) << 8;
    write_val = BB_read_baseband_register(RADIO_ACCESS_REGISTER); // clear int
    return_val |= (write_val & 0x00ff);

    if (!(addr & 0x40))
    {
        /* only check RF register */
        RF_CTRL_CLR_RF_OFF;
    }

    return return_val;
}


//////////////////////////////////////////////////////////////////////////////////////////
#ifndef _IS_ASIC_
UINT8 rtk_get_rf_max_power_index(UINT32 *tx_gain_data_buf)
{
    UINT8 max_index;
    UINT8 pre_data = 0xff;
    UINT8 cur_data;
    UINT8 *ptr = (UINT8*)tx_gain_data_buf;

    for (max_index = 0; max_index < 8; max_index++)
    {
        cur_data = ptr[max_index];
        if ((cur_data == 0xff) || (cur_data == pre_data))
        {
            break;
        }
        pre_data = cur_data;
    }

    if (max_index != 0)
    {
        max_index--;
    }
    return max_index;
}

void rtk_modem_init(void)
{
#ifdef _NEW_MODEM_DESIGN_
    MODEM_REG_S_TYPE modem_reg;

    /*======================================================*/
    /*          Init Page 6 Modem Reguster Here             */
    /*======================================================*/
    /* switch to page 6 */
#ifdef _NEW_MODEM_DESIGN_AFTER_RTL8821B_TC_    
    rtk_write_modem_radio_reg(0x00, TYPE_MODEM, 0x0006);
#ifdef _FPGA_WITH_RLE0550_RFE_
    modem_reg.d16 = rtk_read_modem_radio_reg(TRANS_MODEM_REG(0x1E), TYPE_MODEM);
    modem_reg.p6reg1e.reg_if_bw_le2m = 0;
    rtk_write_modem_radio_reg(TRANS_MODEM_REG(0x1E), TYPE_MODEM, modem_reg.d16);
#endif
#endif
    /*======================================================*/
    /*          Init Page 3 Modem Reguster Here             */
    /*======================================================*/
    /* switch to page 3 */
    rtk_write_modem_radio_reg(0x00, TYPE_MODEM, 0x0003);
#ifdef _NEW_MODEM_DESIGN_AFTER_RTL8821B_TC_
    rtk_write_modem_radio_reg(TRANS_MODEM_REG(0x02), TYPE_MODEM, 0x3322);
    rtk_write_modem_radio_reg(TRANS_MODEM_REG(0x04), TYPE_MODEM, 0x0333);
    modem_reg.d16 = rtk_read_modem_radio_reg(TRANS_MODEM_REG(0x3A), TYPE_MODEM);
    modem_reg.p3reg3a.reg_adc_backoff = 6;
    modem_reg.p3reg3a.reg_if_pw_offset = 3;
    rtk_write_modem_radio_reg(TRANS_MODEM_REG(0x3A), TYPE_MODEM, modem_reg.d16);
#endif

#ifdef _NEW_MODEM_DESIGN_AFTER_RTL8703B_
#ifdef _FPGA_WITH_RLE0379_RFE_
    rtk_write_modem_radio_reg(TRANS_MODEM_REG(0x0C), TYPE_MODEM, 0x0fe0);   // set reg_adc_max=7'h3f for 0379 adc s(7,6f)    <--- 0379 ONLY !!!!!
#elif defined(_FPGA_WITH_RLE0550_RFE_)
    rtk_write_modem_radio_reg(TRANS_MODEM_REG(0x0C), TYPE_MODEM, 0x0fec);   // set reg_adc_max=7'h3f for 0379 adc s(7,6f)    <--- 0379 ONLY !!!!!
#else
    TBD
#endif
#endif
#ifdef _TEST_ADAPTIVITY_FUNC_2
    /* Page3 Reg14 = 0x3210   // reg_mp_lna_idx for RF0379 */
    rtk_write_modem_radio_reg(TRANS_MODEM_REG(0x14), TYPE_MODEM, 0x3210);
#endif

    /*======================================================*/
    /*          Init Page 2 Modem Reguster Here             */
    /*======================================================*/
    /* switch to page 2 */
    rtk_write_modem_radio_reg(0x00, TYPE_MODEM, 0x0002);

#ifdef _FPGA_WITH_RLE0550_RFE_
    modem_reg.d16 = rtk_read_modem_radio_reg(TRANS_MODEM_REG(0x32), TYPE_MODEM);
    modem_reg.p2reg32.reg_if_bw_bt2 = 7;
    modem_reg.p2reg32.reg_if_bw_bt4 = 6;
    rtk_write_modem_radio_reg(TRANS_MODEM_REG(0x32), TYPE_MODEM, modem_reg.d16);
#endif

#ifdef _BRUCE_TEST_PLC_PKT_STATUS
    UINT32 reg_temp;
    reg_temp= rtk_read_modem_radio_reg(TRANS_MODEM_REG(0x62), TYPE_MODEM);
    reg_temp |=(UINT32)BIT0;
    rtk_write_modem_radio_reg(TRANS_MODEM_REG(0x62), TYPE_MODEM, reg_temp);
#endif

    // set share_lna_en //
    modem_reg.d16 = rtk_read_modem_radio_reg(TRANS_MODEM_REG(0x34), TYPE_MODEM);
#ifdef _NEW_MODEM_SHARE_LNA_EN
    modem_reg.p2reg34.reg_share_lna_en = 1;
#else
    modem_reg.p2reg34.reg_share_lna_en = 0;
#endif
    rtk_write_modem_radio_reg(TRANS_MODEM_REG(0x34), TYPE_MODEM, modem_reg.d16);

#if defined(_FPGA_WITH_RLE0379_RFE_)
    rtk_write_modem_radio_reg(0x04, TYPE_MODEM, 0x0000);
    rtk_write_modem_radio_reg(0x05, TYPE_MODEM, 0x0264); // set BT2 RX IF as 2.5MHz

    rtk_write_modem_radio_reg(TRANS_MODEM_REG(0x30), TYPE_MODEM, 0x0000);
    UINT16 d16_temp = rtk_read_modem_radio_reg(TRANS_MODEM_REG(0x32), TYPE_MODEM);
    d16_temp &= 0xfff0;
    d16_temp += 0x0004;
    rtk_write_modem_radio_reg(TRANS_MODEM_REG(0x32), TYPE_MODEM, d16_temp); // set BT4 RX IF as 2.5MHz
#elif defined(_FPGA_WITH_RLE0546_RFE_) || defined(_FPGA_WITH_RLE0608C_RFE_) || defined(_FPGA_WITH_RLE0550_RFE_)

    UINT32 modem_if_bt2 = NEW_MODEM_IF_1P4;
    UINT32 modem_if_bt4 = NEW_MODEM_IF_1P4;
    // set BT2 IF Freq //
    modem_reg.d16 = rtk_read_modem_radio_reg(TRANS_MODEM_REG(0x08), TYPE_MODEM);
    modem_reg.p2reg08.reg_btm_if_freq_bt2_15_0 = modem_if_bt2 & 0xFFFF;
    rtk_write_modem_radio_reg(TRANS_MODEM_REG(0x08), TYPE_MODEM, modem_reg.d16);
    modem_reg.d16 = rtk_read_modem_radio_reg(TRANS_MODEM_REG(0x0A), TYPE_MODEM);
    modem_reg.p2reg0a.reg_btm_if_freq_bt2_19_16 = (modem_if_bt2>>16);
    modem_reg.p2reg0a.reg_btm_bt4_dfir_bw = 6;
    rtk_write_modem_radio_reg(TRANS_MODEM_REG(0x0A), TYPE_MODEM, modem_reg.d16);

    modem_reg.d16 = rtk_read_modem_radio_reg(TRANS_MODEM_REG(0x30), TYPE_MODEM);
    modem_reg.p2reg30.reg_btm_if_freq_bt4_15_0 = modem_if_bt4 & 0xFFFF;
    rtk_write_modem_radio_reg(TRANS_MODEM_REG(0x30), TYPE_MODEM, modem_reg.d16);
    modem_reg.d16 = rtk_read_modem_radio_reg(TRANS_MODEM_REG(0x32), TYPE_MODEM);
    modem_reg.p2reg32.reg_btm_if_freq_bt4_19_16 = (modem_if_bt4>>16);
    rtk_write_modem_radio_reg(TRANS_MODEM_REG(0x32), TYPE_MODEM, modem_reg.d16);


//    modem_reg.d16 = rtk_read_modem_radio_reg(TRANS_MODEM_REG(0x3C), TYPE_MODEM);
//    modem_reg.p2reg3c.reg_if_bw_bt2=0; /* IF 1.4MHz, BW 1.3MHz */
//    modem_reg.p2reg3c.reg_if_bw_bt4=1; /* IF 1.4MHz, BW 1.6MHz */
//    rtk_write_modem_radio_reg(TRANS_MODEM_REG(0x3C), TYPE_MODEM, modem_reg.d16);
#else
    !!!!! ERROR !!!!!
#endif


#ifdef _FPGA_WITH_RLE0379_RFE_
    rtk_write_modem_radio_reg(TRANS_MODEM_REG(0x0a), TYPE_MODEM, 0x0664);   // set srrc555 for dpsk DFIR        <--- 0379 ONLY !!!!!
#else
    rtk_write_modem_radio_reg(TRANS_MODEM_REG(0x0a), TYPE_MODEM, 0x0264);   // set srrc500 for dpsk DFIR
#endif

#if defined(_NEW_MODEM_AAGC_SET1_) /* RX AGC SETTING 1, for RLE0546 */
//    rtk_write_modem_radio_reg(TRANS_MODEM_REG(0x0a), TYPE_MODEM, 0x0244);
    rtk_write_modem_radio_reg(TRANS_MODEM_REG(0x34), TYPE_MODEM, 0x0492);
    rtk_write_modem_radio_reg(TRANS_MODEM_REG(0x40), TYPE_MODEM, 0x3fa5);
    rtk_write_modem_radio_reg(TRANS_MODEM_REG(0x42), TYPE_MODEM, 0x07ff);
    rtk_write_modem_radio_reg(TRANS_MODEM_REG(0x44), TYPE_MODEM, 0x403f);
    rtk_write_modem_radio_reg(TRANS_MODEM_REG(0x46), TYPE_MODEM, 0x3514);
    rtk_write_modem_radio_reg(TRANS_MODEM_REG(0x48), TYPE_MODEM, 0x3514);
    rtk_write_modem_radio_reg(TRANS_MODEM_REG(0x4e), TYPE_MODEM, 0x0514);
    rtk_write_modem_radio_reg(TRANS_MODEM_REG(0x50), TYPE_MODEM, 0x0514);
//    rtk_write_modem_radio_reg(TRANS_MODEM_REG(0x40), TYPE_MODEM, 0x3450);
//    rtk_write_modem_radio_reg(TRANS_MODEM_REG(0x4e), TYPE_MODEM, 0x0618);
//    rtk_write_modem_radio_reg(TRANS_MODEM_REG(0x50), TYPE_MODEM, 0x0618);
#elif defined(_NEW_MODEM_AAGC_SET2_) /* RX AGC SETTING 2, for RLE0546(LE<1%) and RLE0379 Legacy */
    rtk_write_modem_radio_reg(TRANS_MODEM_REG(0x40), TYPE_MODEM, 0x3fa5);
    rtk_write_modem_radio_reg(TRANS_MODEM_REG(0x42), TYPE_MODEM, 0x07ff);
#elif defined(_NEW_MODEM_AAGC_SET3_) /* RX AGC SETTING 3, for RLE0546/RLE0379, AGC_SET2 + "big_jump = 16dB" */
    rtk_write_modem_radio_reg(TRANS_MODEM_REG(0x40), TYPE_MODEM, 0x3fa5);
    rtk_write_modem_radio_reg(TRANS_MODEM_REG(0x42), TYPE_MODEM, 0x07ff);
    rtk_write_modem_radio_reg(TRANS_MODEM_REG(0x44), TYPE_MODEM, 0x403f);
#elif defined(_NEW_MODEM_AAGC_SET4_) /* RX AGC SETTING 4, for RLE0546/RLE0379, AGC_SET3 + "clip ratio 2/4,1/4" + "Fast Settling 3.0us" */

  #ifdef _NEW_MODEM_DESIGN_AFTER_RTL8703B_
    rtk_write_modem_radio_reg(TRANS_MODEM_REG(0x40), TYPE_MODEM, 0x3250);   // set bb_pw_time_clip, BR clip2
  #elif _NEW_MODEM_DESIGN_AFTER_RTL8821B_TC_
    rtk_write_modem_radio_reg(TRANS_MODEM_REG(0x40), TYPE_MODEM, 0x3fa5);   // to fix AGC bug
  #else
    rtk_write_modem_radio_reg(TRANS_MODEM_REG(0x40), TYPE_MODEM, 0x3f50);
  #endif

    rtk_write_modem_radio_reg(TRANS_MODEM_REG(0x42), TYPE_MODEM, 0x07ff);
    rtk_write_modem_radio_reg(TRANS_MODEM_REG(0x44), TYPE_MODEM, 0x403f);
    rtk_write_modem_radio_reg(TRANS_MODEM_REG(0x46), TYPE_MODEM, 0x379e);
    rtk_write_modem_radio_reg(TRANS_MODEM_REG(0x48), TYPE_MODEM, 0x779e);

  #ifdef _NEW_MODEM_DESIGN_AFTER_RTL8703B_
    rtk_write_modem_radio_reg(TRANS_MODEM_REG(0x4e), TYPE_MODEM, 0x069a);   // set LE settling time = 2.6us
    rtk_write_modem_radio_reg(TRANS_MODEM_REG(0x50), TYPE_MODEM, 0x069a);   // set LE settling time = 2.6us
    rtk_write_modem_radio_reg(TRANS_MODEM_REG(0x60), TYPE_MODEM, 0x1934);   // for NEW BT4 settings // set LE clip2
  #else
    rtk_write_modem_radio_reg(TRANS_MODEM_REG(0x4e), TYPE_MODEM, 0x079e);
    rtk_write_modem_radio_reg(TRANS_MODEM_REG(0x50), TYPE_MODEM, 0x079e);
    rtk_write_modem_radio_reg(TRANS_MODEM_REG(0x60), TYPE_MODEM, 0x1fb4);  // for NEW BT4 settings
  #endif


#elif defined(_NEW_MODEM_AAGC_SETTEMP_) /* RX AGC SETTING for UPF TEST */
	{
        UINT16 BIG_JUMP_BT2 = 4; // unit: 4dB, 4b
        UINT16 BIG_JUMP_BT4 = 4; // unit: 4dB, 4b
        UINT16 SMALL_JUMP_BT2 = 3; // unit: 2dB, 3b
        UINT16 SMALL_JUMP_BT4 = 3; // unit: 2dB, 3b
        UINT16 SETTLING_TIME_BT2_X =  25; // uint: 0.1us
        UINT16 SETTLING_TIME_BT4_X =  25; // uint: 0.1us
        UINT16 MP_GAIN_UP_BOUND = 63; // -90dBm + x*2dB  limit the minimum gain
        UINT16 MP_GAIN_LOW_BOUND = 0; // -90dBm + x*2dB  limit the maximum gain (改這個來做限制AAGC可調範圍，比之前講的方法(改AGC table)簡單)
                                      //                   e.g. MP_GAIN_LOW_BOUND = 15: Input Power限制在-60dBm以上，對High Power對好一點
        rtk_write_modem_radio_reg(TRANS_MODEM_REG(0x40), TYPE_MODEM, 0x3f50);
        rtk_write_modem_radio_reg(TRANS_MODEM_REG(0x42), TYPE_MODEM, 0x07ff);
        rtk_write_modem_radio_reg(TRANS_MODEM_REG(0x44), TYPE_MODEM, (BIG_JUMP_BT2<<12) | (MP_GAIN_LOW_BOUND<<6) | (MP_GAIN_UP_BOUND));
        rtk_write_modem_radio_reg(TRANS_MODEM_REG(0x46), TYPE_MODEM, (SMALL_JUMP_BT2<<12) | (SETTLING_TIME_BT2_X<<6) | SETTLING_TIME_BT2_X);
        rtk_write_modem_radio_reg(TRANS_MODEM_REG(0x48), TYPE_MODEM, 0x6000 | (SETTLING_TIME_BT2_X<<6) | SETTLING_TIME_BT2_X));
        rtk_write_modem_radio_reg(TRANS_MODEM_REG(0x4e), TYPE_MODEM, (SETTLING_TIME_BT4_X<<6) | SETTLING_TIME_BT4_X));
        rtk_write_modem_radio_reg(TRANS_MODEM_REG(0x50), TYPE_MODEM, (SETTLING_TIME_BT4_X<<6) | SETTLING_TIME_BT4_X));
        rtk_write_modem_radio_reg(TRANS_MODEM_REG(0x60), TYPE_MODEM, 0x1f80 | (SMALL_JUMP_BT4<<4) | BIG_JUMP_BT4); // for NEW BT4 settings
    }
#endif



#ifdef _DAPE_TEST_CHG_MODEM_TX_FREQ_FOR_IOT
////////// 0x0147 : ?kHz
//    rtk_write_modem_radio_reg(TRANS_MODEM_REG(0x2E), TYPE_MODEM, 0x0147);
////////// 0x028E : ?kHz
//    rtk_write_modem_radio_reg(TRANS_MODEM_REG(0x2E), TYPE_MODEM, 0x028E);
////////// 0xFEB7 : ?kHz
    rtk_write_modem_radio_reg(TRANS_MODEM_REG(0x2E), TYPE_MODEM, 0xFEB7);
	d16_temp = rtk_read_modem_radio_reg(TRANS_MODEM_REG(0x2E), TYPE_MODEM);
    RT_BT_LOG(YELLOW, DAPE_TEST_LOG213, 2,0x2E, d16_temp);
#endif
#endif // end of #ifdef _NEW_MODEM_DESIGN_ //
    /*======================================================*/
    /*          End of Setting Page 2 Modem Reguster        */
    /*======================================================*/

    /*======================================================*/
    /*          Init Page 1 Modem Reguster Here             */
    /*======================================================*/
    /* switch to page 1 */
    rtk_write_modem_radio_reg(0x00, TYPE_MODEM, 0x0001);

#if defined(LE_MODE_EN) && (defined(LE_ACC_CHK2_EN) || defined(LE_SET_AA_OPT_EN))
    UINT16 read_val;
    read_val = rtk_read_modem_radio_reg(0x02, TYPE_MODEM);
    read_val &= 0x803f;

#ifdef LE_ACC_CHK2_EN
    read_val |= BIT14;
    /* NOTE: LE_ACC_CHK2_VALUE is expired after 04/25; replaced by LE_ACC_CHK2_TH32 */
    // read_val |= (LE_ACC_CHK2_VALUE << 6);
#endif
#ifdef LE_SET_AA_OPT_EN
    read_val |= (LE_SET_AA_OPT_VALUE << 12);
#endif
    rtk_write_modem_radio_reg(0x02, TYPE_MODEM, read_val);
#endif
    /* NOTE: Valid after 04/25 */
    rtk_write_modem_radio_reg(0x1A, TYPE_MODEM, LE_ACC_CHK2_TH32&0xFFFF);
    rtk_write_modem_radio_reg(0x1B, TYPE_MODEM, (LE_ACC_CHK2_TH32>>16)&0xFFFF);

#ifdef _YL_NEW_MODEM_PGA_SETTLINGTIME_
    /* NEW Setting, To be added to efuse & patch/vendor command(desired) */
    /* valid after 2011/03/16 modem update */
    UINT32 mdm_page1_0x18 = (PGA_SETTLINGTIME_UP) | (PGA_SETTLINGTIME_BIG<<4) |
                            (PGA_SETTLINGTIME_SML<<8) | (PGA_SETTLINGTIME_DN<<12);
    UINT32 mdm_page1_0x19 = (PGA_SETTLINGTIME_UP_BT4) | (PGA_SETTLINGTIME_BIG_BT4<<4) |
                            (PGA_SETTLINGTIME_SML_BT4<<8) | (PGA_SETTLINGTIME_DN_BT4<<12);
    rtk_write_modem_radio_reg(0x18, TYPE_MODEM, mdm_page1_0x18);
    rtk_write_modem_radio_reg(0x19, TYPE_MODEM, mdm_page1_0x19);
    //RT_BT_LOG(YELLOW, CCH_DBG_200, 4, mdm_page1_0x18, mdm_page1_0x19);
#endif

    /*======================================================*/
    /*          Init Page 0 Modem Reguster Here             */
    /*======================================================*/
    /* switch to page 0 */
    rtk_write_modem_radio_reg(0x00, TYPE_MODEM, otp_rf_str_data.mdm_ini_0x00);//0x6f76);//(disable AGC) 0x6f76
    rtk_write_modem_radio_reg(0x01, TYPE_MODEM, otp_rf_str_data.mdm_ini_0x01);//0x4115);
    rtk_write_modem_radio_reg(0x02, TYPE_MODEM, otp_rf_str_data.mdm_ini_0x02);//0x94db);//PGA_settling=2.0us
    rtk_write_modem_radio_reg(0x03, TYPE_MODEM, otp_rf_str_data.mdm_ini_0x03);//0x309a);//BB_PW_time=1.6us, freeze_th=1.5dB
    rtk_write_modem_radio_reg(0x04, TYPE_MODEM, otp_rf_str_data.mdm_ini_0x04);//0x361f);//DPSK_sync_MF_TH=9, AGC_sat_num=2, diable all RF-mode, AGC_PW_mom_th=2dB
    rtk_write_modem_radio_reg(0x05, TYPE_MODEM, otp_rf_str_data.mdm_ini_0x05);//0x00b0);
    rtk_write_modem_radio_reg(0x06, TYPE_MODEM, otp_rf_str_data.mdm_ini_0x06);//0x4912);
    rtk_write_modem_radio_reg(0x07, TYPE_MODEM, otp_rf_str_data.mdm_ini_0x07);//0x13f2);//big_jump=24dB, small_jump=6dB.(0917)0x63f2
    rtk_write_modem_radio_reg(0x08, TYPE_MODEM, otp_rf_str_data.mdm_ini_0x08);//0x569e);
    rtk_write_modem_radio_reg(0x09, TYPE_MODEM, otp_rf_str_data.mdm_ini_0x09);//0x0f48);//(0917)0x0198, EDR_AGC_offset=0dB
    rtk_write_modem_radio_reg(0x0A, TYPE_MODEM, otp_rf_str_data.mdm_ini_0x0A);//0x0000);
    rtk_write_modem_radio_reg(0x0B, TYPE_MODEM, otp_rf_str_data.mdm_ini_0x0B);//0x5a32);//clip_ratio=2/4
    rtk_write_modem_radio_reg(0x0C, TYPE_MODEM, otp_rf_str_data.mdm_ini_0x0C);//0x690B);//PGA_settlingtime=3.5us, sync_MF_TH=42

    //marc: for 0380 MODEM
    rtk_write_modem_radio_reg(0x0D, TYPE_MODEM, otp_rf_str_data.mdm_ini_0x0D); //GPS

    //===Pouter===
    rtk_write_modem_radio_reg(0x16, TYPE_MODEM, otp_rf_str_data.mdm_ini_0x16);//0x4400);
    rtk_write_modem_radio_reg(0x17, TYPE_MODEM, otp_rf_str_data.mdm_ini_0x17);//0x0cc0); //bypass payload header for ACL
    rtk_write_modem_radio_reg(0x19, TYPE_MODEM, otp_rf_str_data.mdm_ini_0x19);//0x01d4);

    //===AFE===
    rtk_write_modem_radio_reg(0x1A, TYPE_MODEM, otp_rf_str_data.mdm_ini_0x1A);//0x0030);
    rtk_write_modem_radio_reg(0x1B, TYPE_MODEM, otp_rf_str_data.mdm_ini_0x1B);//0x0001);
    rtk_write_modem_radio_reg(0x1C, TYPE_MODEM, otp_rf_str_data.mdm_ini_0x1C);//0x034d);
    rtk_write_modem_radio_reg(0x1D, TYPE_MODEM, otp_rf_str_data.mdm_ini_0x1D);//0x2521);
    rtk_write_modem_radio_reg(0x1E, TYPE_MODEM, otp_rf_str_data.mdm_ini_0x1E);//0x00e9);

    rtk_write_modem_radio_reg(0x30, TYPE_MODEM, otp_rf_str_data.mdm_ini_0x30);
    rtk_write_modem_radio_reg(0x31, TYPE_MODEM, otp_rf_str_data.mdm_ini_0x31);
    rtk_write_modem_radio_reg(0x32, TYPE_MODEM, otp_rf_str_data.mdm_ini_0x32);
    rtk_write_modem_radio_reg(0x33, TYPE_MODEM, otp_rf_str_data.mdm_ini_0x33);

#ifdef _NEW_MODEM_DESIGN_
    /* set used old/new AGC */
    modem_reg.d16 = rtk_read_modem_radio_reg(TRANS_MODEM_REG(0x3c), TYPE_MODEM);
#ifdef _NEW_MODEM_OLD_AAGC
    modem_reg.reg3c.r_aagc_8723A_en = 1;
#else
    modem_reg.reg3c.r_aagc_8723A_en = 0;
#endif
    rtk_write_modem_radio_reg(TRANS_MODEM_REG(0x3c), TYPE_MODEM, modem_reg.d16);
#endif

    //===Rx AGC===
    #ifdef _NEW_MODEM_DESIGN_PHY_SETTING_
        rtl8723_btrf_RxAGCTableInit();
    #else
        rtk_modem_rx_agc_tbl_init();
    #endif

    //===Bluewiz Tx AGC===
    BB_write_baseband_register(0x136, otp_rf_str_data.mdm_ini_BW_0x136);//0x8984);
    BB_write_baseband_register(0x138, otp_rf_str_data.mdm_ini_BW_0x138);//0x0628);
    BB_write_baseband_register(0x13a, otp_rf_str_data.mdm_ini_BW_0x13a);//0x522c);
    BB_write_baseband_register(0x13c, otp_rf_str_data.mdm_ini_BW_0x13c);//0x00b1);

    /* add by austin */
    BB_write_baseband_register(TX_AGC_2M_LUT0_REG, otp_rf_str_data.mdm_ini_BW_0x1bc);
    BB_write_baseband_register(TX_AGC_2M_LUT1_REG, otp_rf_str_data.mdm_ini_BW_0x1be);
    BB_write_baseband_register(TX_AGC_2M_LUT2_REG, otp_rf_str_data.mdm_ini_BW_0x1c0);
    BB_write_baseband_register(TX_AGC_2M_LUT3_REG, otp_rf_str_data.mdm_ini_BW_0x1c2);

    BB_write_baseband_register(TX_AGC_3M_LUT0_REG, otp_rf_str_data.mdm_ini_BW_0x1c4);
    BB_write_baseband_register(TX_AGC_3M_LUT1_REG, otp_rf_str_data.mdm_ini_BW_0x1c6);
    BB_write_baseband_register(TX_AGC_3M_LUT2_REG, otp_rf_str_data.mdm_ini_BW_0x1c8);
    BB_write_baseband_register(TX_AGC_3M_LUT3_REG, otp_rf_str_data.mdm_ini_BW_0x1ca);

    /* init GFSK SBD (GIAC) registers */
    rtk_write_modem_radio_reg(0x12, TYPE_MODEM, 0x5E72);
    rtk_write_modem_radio_reg(0x13, TYPE_MODEM, 0x7334);
    rtk_write_modem_radio_reg(0x14, TYPE_MODEM, 0x58CC);
    rtk_write_modem_radio_reg(0x15, TYPE_MODEM, 0x475C);

    UINT32 buf[2];
    buf[0] = (otp_rf_str_data.mdm_ini_BW_0x138 << 16) |
              otp_rf_str_data.mdm_ini_BW_0x136;
    buf[1] = (otp_rf_str_data.mdm_ini_BW_0x13c << 16) |
              otp_rf_str_data.mdm_ini_BW_0x13a;
    max_rtk_radio_tx_step_index = rtk_get_rf_max_power_index(buf);

#ifdef LE_MODE_EN
    le_tx_power_max_index = max_rtk_radio_tx_step_index;
    le_tx_power_max_value = *((UINT8*)buf + le_tx_power_max_index);
#endif

#if 0
#ifdef _YL_NEW_MODEM_SRAM_DEBUG
    rtl8821_btrf_modem_sram_debug_init();
#endif
#endif

#ifdef _TEST_ADAPTIVITY_FUNC_
{
    /********************************************************/
    /***************** Debug Port Setting Step 2 ************/
    /* Reg_0a[15:11] of page0 <= 11 for lbt_debug signal ****/
    /********************************************************/
    /* switch to page 0 */
    rtk_write_modem_radio_reg(0x00, TYPE_MODEM, 0x0000);

    UINT16 read_val;
    read_val = rtk_read_modem_radio_reg(TRANS_MODEM_REG(0x0A), TYPE_MODEM);
    RT_BT_LOG(GREEN, DAPE_TEST_LOG213, 2, read_val, rtk_read_modem_radio_reg(TRANS_MODEM_REG(0x7E), TYPE_MODEM));
    read_val &= 0x07FF;
    read_val |= 0x5800;
    rtk_write_modem_radio_reg(TRANS_MODEM_REG(0xA), TYPE_MODEM, read_val);
    RT_BT_LOG(BLUE, DAPE_TEST_LOG213, 2, read_val, rtk_read_modem_radio_reg(TRANS_MODEM_REG(0x0A), TYPE_MODEM));


    /*****************************************************************/
    /******************** Debug Port Setting Step 2 ******************/
    /* Reg_18[2:0] of page6 <= 0/1/2/3 for lbt debug port selection ****/
    /*****************************************************************/
    /* switch to page 6 */
    rtk_write_modem_radio_reg(0x00, TYPE_MODEM, 0x0006);
    read_val = rtk_read_modem_radio_reg(TRANS_MODEM_REG(0x18), TYPE_MODEM);
    RT_BT_LOG(GREEN, DAPE_TEST_LOG207, 1, read_val);
    read_val &= 0xFFF8;

   /*****************************************************************/
   /***************|= 0~3 for LBT debug port selection.  ************/
#ifdef _TEST_ADAPTIVITY_FUNC_2
    read_val |= 0x0001;
#else
    read_val |= 0x0002;
#endif
   /*****************************************************************/
   /*****************************************************************/
    rtk_write_modem_radio_reg(TRANS_MODEM_REG(0x18), TYPE_MODEM, read_val);

    RT_BT_LOG(BLUE, DAPE_TEST_LOG213, 2, read_val, rtk_read_modem_radio_reg(TRANS_MODEM_REG(0x18), TYPE_MODEM));

#ifdef _TEST_ADAPTIVITY_FUNC_2
    /* Step 2 */
    /* Page6 Reg0a[15:10] reg_lbt_rxg_idx_start = 9 */
    read_val = rtk_read_modem_radio_reg(TRANS_MODEM_REG(0x0A), TYPE_MODEM);
    read_val &= 0x03FF;
    read_val |= 0x2400;
    rtk_write_modem_radio_reg(TRANS_MODEM_REG(0x0A), TYPE_MODEM, read_val);

    /* Page6 Reg10[13:7] reg_lbt_txpwr_offset = 40 */
    read_val = rtk_read_modem_radio_reg(TRANS_MODEM_REG(0x10), TYPE_MODEM);
    read_val &= 0xC07F;
    read_val |= 0x1400;
    rtk_write_modem_radio_reg(TRANS_MODEM_REG(0x10), TYPE_MODEM, read_val);

    /* Page6 Reg14[3:0] reg_lbt_rf_mode = 3
       Page6 Reg14[15:13] reg_lbt_lpf_bw = 1*/
    read_val = rtk_read_modem_radio_reg(TRANS_MODEM_REG(0x14), TYPE_MODEM);
    read_val &= 0x1FF0;
    read_val |= 0x2003;
    rtk_write_modem_radio_reg(TRANS_MODEM_REG(0x14), TYPE_MODEM, read_val);

#endif

    /* switch to page 0, set value to default. */
    rtk_write_modem_radio_reg(0x00, TYPE_MODEM, 0x0000);
    //rtk_write_modem_radio_reg(0x0A, TYPE_MODEM, otp_rf_str_data.mdm_ini_0x0A);//0x0000);

}
#endif

}


#if 0
//////////////////////////////////////////////////////////////////////////////////////////
#ifndef _IS_ASIC_
UINT8 rtk_get_rf_max_power_index(UINT32 *tx_gain_data_buf)
{
    UINT8 max_index;
    UINT8 pre_data = 0xff;
    UINT8 cur_data;
    UINT8 *ptr = (UINT8*)tx_gain_data_buf;

    for (max_index = 0; max_index < 8; max_index++)
    {
        cur_data = ptr[max_index];
        if ((cur_data == 0xff) || (cur_data == pre_data))
        {
            break;
        }
        pre_data = cur_data;
    }

    if (max_index != 0)
    {
        max_index--;
    }
    return max_index;
}
#endif
#endif

void rtk_modem_rx_agc_tbl_init()
{
#ifndef _NEW_MODEM_DESIGN_
    rtk_write_modem_radio_reg(0x1f, TYPE_MODEM, otp_rf_str_data.mdm_rxagc_tbl_00);//0x0e3f);
    rtk_write_modem_radio_reg(0x1f, TYPE_MODEM, otp_rf_str_data.mdm_rxagc_tbl_01);//0x0e3e);
    rtk_write_modem_radio_reg(0x1f, TYPE_MODEM, otp_rf_str_data.mdm_rxagc_tbl_02);//0x0e3d);
    rtk_write_modem_radio_reg(0x1f, TYPE_MODEM, otp_rf_str_data.mdm_rxagc_tbl_03);//0x0e3c);
    rtk_write_modem_radio_reg(0x1f, TYPE_MODEM, otp_rf_str_data.mdm_rxagc_tbl_04);//0x0e3b);
    rtk_write_modem_radio_reg(0x1f, TYPE_MODEM, otp_rf_str_data.mdm_rxagc_tbl_05);//0x0e3a);
    rtk_write_modem_radio_reg(0x1f, TYPE_MODEM, otp_rf_str_data.mdm_rxagc_tbl_06);//0x0e39);
    rtk_write_modem_radio_reg(0x1f, TYPE_MODEM, otp_rf_str_data.mdm_rxagc_tbl_07);//0x0e38);
    rtk_write_modem_radio_reg(0x1f, TYPE_MODEM, otp_rf_str_data.mdm_rxagc_tbl_08);//0x0e37);
    rtk_write_modem_radio_reg(0x1f, TYPE_MODEM, otp_rf_str_data.mdm_rxagc_tbl_09);//0x0e36);
    rtk_write_modem_radio_reg(0x1f, TYPE_MODEM, otp_rf_str_data.mdm_rxagc_tbl_10);//0x0e35);
    rtk_write_modem_radio_reg(0x1f, TYPE_MODEM, otp_rf_str_data.mdm_rxagc_tbl_11);//0x0e34);
    rtk_write_modem_radio_reg(0x1f, TYPE_MODEM, otp_rf_str_data.mdm_rxagc_tbl_12);//0x0e33);
    rtk_write_modem_radio_reg(0x1f, TYPE_MODEM, otp_rf_str_data.mdm_rxagc_tbl_13);//0x0e32);
    rtk_write_modem_radio_reg(0x1f, TYPE_MODEM, otp_rf_str_data.mdm_rxagc_tbl_14);//0x0e31);
    rtk_write_modem_radio_reg(0x1f, TYPE_MODEM, otp_rf_str_data.mdm_rxagc_tbl_15);//0x0e30);
    rtk_write_modem_radio_reg(0x1f, TYPE_MODEM, otp_rf_str_data.mdm_rxagc_tbl_16);//0x0f2f);
    rtk_write_modem_radio_reg(0x1f, TYPE_MODEM, otp_rf_str_data.mdm_rxagc_tbl_17);//0x102e);
    rtk_write_modem_radio_reg(0x1f, TYPE_MODEM, otp_rf_str_data.mdm_rxagc_tbl_18);//0x112d);
    rtk_write_modem_radio_reg(0x1f, TYPE_MODEM, otp_rf_str_data.mdm_rxagc_tbl_19);//0x402c);
    rtk_write_modem_radio_reg(0x1f, TYPE_MODEM, otp_rf_str_data.mdm_rxagc_tbl_20);//0x412b);
    rtk_write_modem_radio_reg(0x1f, TYPE_MODEM, otp_rf_str_data.mdm_rxagc_tbl_21);//0x422a);
    rtk_write_modem_radio_reg(0x1f, TYPE_MODEM, otp_rf_str_data.mdm_rxagc_tbl_22);//0x4329);
    rtk_write_modem_radio_reg(0x1f, TYPE_MODEM, otp_rf_str_data.mdm_rxagc_tbl_23);//0x4428);
    rtk_write_modem_radio_reg(0x1f, TYPE_MODEM, otp_rf_str_data.mdm_rxagc_tbl_24);//0x4527);
    rtk_write_modem_radio_reg(0x1f, TYPE_MODEM, otp_rf_str_data.mdm_rxagc_tbl_25);//0x4626);
    rtk_write_modem_radio_reg(0x1f, TYPE_MODEM, otp_rf_str_data.mdm_rxagc_tbl_26);//0x4725);
    rtk_write_modem_radio_reg(0x1f, TYPE_MODEM, otp_rf_str_data.mdm_rxagc_tbl_27);//0x4824);
    rtk_write_modem_radio_reg(0x1f, TYPE_MODEM, otp_rf_str_data.mdm_rxagc_tbl_28);//0x4923);
    rtk_write_modem_radio_reg(0x1f, TYPE_MODEM, otp_rf_str_data.mdm_rxagc_tbl_29);//0x4a22);
    rtk_write_modem_radio_reg(0x1f, TYPE_MODEM, otp_rf_str_data.mdm_rxagc_tbl_30);//0x4b21);
    rtk_write_modem_radio_reg(0x1f, TYPE_MODEM, otp_rf_str_data.mdm_rxagc_tbl_31);//0x4c20);
    rtk_write_modem_radio_reg(0x1f, TYPE_MODEM, otp_rf_str_data.mdm_rxagc_tbl_32);//0x4d1f);
    rtk_write_modem_radio_reg(0x1f, TYPE_MODEM, otp_rf_str_data.mdm_rxagc_tbl_33);//0x4e1e);
    rtk_write_modem_radio_reg(0x1f, TYPE_MODEM, otp_rf_str_data.mdm_rxagc_tbl_34);//0x4f1d);
    rtk_write_modem_radio_reg(0x1f, TYPE_MODEM, otp_rf_str_data.mdm_rxagc_tbl_35);//0x501c);
    rtk_write_modem_radio_reg(0x1f, TYPE_MODEM, otp_rf_str_data.mdm_rxagc_tbl_36);//0x511b);
    rtk_write_modem_radio_reg(0x1f, TYPE_MODEM, otp_rf_str_data.mdm_rxagc_tbl_37);//0x521a);
    rtk_write_modem_radio_reg(0x1f, TYPE_MODEM, otp_rf_str_data.mdm_rxagc_tbl_38);//0x6219);
    rtk_write_modem_radio_reg(0x1f, TYPE_MODEM, otp_rf_str_data.mdm_rxagc_tbl_39);//0x6318);
    rtk_write_modem_radio_reg(0x1f, TYPE_MODEM, otp_rf_str_data.mdm_rxagc_tbl_40);//0x6417);
    rtk_write_modem_radio_reg(0x1f, TYPE_MODEM, otp_rf_str_data.mdm_rxagc_tbl_41);//0x6516);
    rtk_write_modem_radio_reg(0x1f, TYPE_MODEM, otp_rf_str_data.mdm_rxagc_tbl_42);//0x6615);
    rtk_write_modem_radio_reg(0x1f, TYPE_MODEM, otp_rf_str_data.mdm_rxagc_tbl_43);//0x6714);
    rtk_write_modem_radio_reg(0x1f, TYPE_MODEM, otp_rf_str_data.mdm_rxagc_tbl_44);//0x6813);
    rtk_write_modem_radio_reg(0x1f, TYPE_MODEM, otp_rf_str_data.mdm_rxagc_tbl_45);//0x6912);
    rtk_write_modem_radio_reg(0x1f, TYPE_MODEM, otp_rf_str_data.mdm_rxagc_tbl_46);//0x6a11);
    rtk_write_modem_radio_reg(0x1f, TYPE_MODEM, otp_rf_str_data.mdm_rxagc_tbl_47);//0x6b10);
    rtk_write_modem_radio_reg(0x1f, TYPE_MODEM, otp_rf_str_data.mdm_rxagc_tbl_48);//0x6c0f);
    rtk_write_modem_radio_reg(0x1f, TYPE_MODEM, otp_rf_str_data.mdm_rxagc_tbl_49);//0x6d0e);
    rtk_write_modem_radio_reg(0x1f, TYPE_MODEM, otp_rf_str_data.mdm_rxagc_tbl_50);//0x6e0d);
    rtk_write_modem_radio_reg(0x1f, TYPE_MODEM, otp_rf_str_data.mdm_rxagc_tbl_51);//0x6f0c);
    rtk_write_modem_radio_reg(0x1f, TYPE_MODEM, otp_rf_str_data.mdm_rxagc_tbl_52);//0x700b);
    rtk_write_modem_radio_reg(0x1f, TYPE_MODEM, otp_rf_str_data.mdm_rxagc_tbl_53);//0x710a);
    rtk_write_modem_radio_reg(0x1f, TYPE_MODEM, otp_rf_str_data.mdm_rxagc_tbl_54);//0x7209);
    rtk_write_modem_radio_reg(0x1f, TYPE_MODEM, otp_rf_str_data.mdm_rxagc_tbl_55);//0x7308);
    rtk_write_modem_radio_reg(0x1f, TYPE_MODEM, otp_rf_str_data.mdm_rxagc_tbl_56);//0x7407);
    rtk_write_modem_radio_reg(0x1f, TYPE_MODEM, otp_rf_str_data.mdm_rxagc_tbl_57);//0x7506);
    rtk_write_modem_radio_reg(0x1f, TYPE_MODEM, otp_rf_str_data.mdm_rxagc_tbl_58);//0x7605);
    rtk_write_modem_radio_reg(0x1f, TYPE_MODEM, otp_rf_str_data.mdm_rxagc_tbl_59);//0x7704);
    rtk_write_modem_radio_reg(0x1f, TYPE_MODEM, otp_rf_str_data.mdm_rxagc_tbl_60);//0x7803);
    rtk_write_modem_radio_reg(0x1f, TYPE_MODEM, otp_rf_str_data.mdm_rxagc_tbl_61);//0x7902);
    rtk_write_modem_radio_reg(0x1f, TYPE_MODEM, otp_rf_str_data.mdm_rxagc_tbl_62);//0x7a01);
    rtk_write_modem_radio_reg(0x1f, TYPE_MODEM, otp_rf_str_data.mdm_rxagc_tbl_63);//0x7b00);
#else
    /* In RTL8821A:
     *    Old AGC table and New AGC tables are separate ones.
     *    In other words, there are totally 5 physical tables in the BT Modem
     *    One can seperately write the 5 tables, and then set r_aagc_8723A_en
     *    to select New/Old AGC (and the corresponding tables ...)
     */

    UINT16 ii;
    MODEM_REG_S_TYPE modem_reg;
    MODEM_AGC_TABLE_S_TYPE modem_agc_table;
    OTP_OLD_AGC_TABLE_S_TYPE otp_old_agc_table;
    modem_reg.d16 = rtk_read_modem_radio_reg(TRANS_MODEM_REG(0x3c), TYPE_MODEM);
  #ifdef _NEW_MODEM_OLD_AAGC
    modem_reg.reg3c.r_aagc_8723A_en = 1;
  #else
    modem_reg.reg3c.r_aagc_8723A_en = 0;
  #endif

/***** Table 0 *****/
  #ifndef _NEW_MODEM_OLD_AAGC
    modem_reg.reg3c.r_table_sel = 0;
    rtk_write_modem_radio_reg(TRANS_MODEM_REG(0x3c), TYPE_MODEM, modem_reg.d16);
  #ifdef _NEW_MODEM_DESIGN_PHY_SETTING_
    for (ii = 0; ii<NEW_MODEM_NEW_AGC_TABLE0_SIZE; ii++)
    {
#ifdef _NEW_MODEM_PI_ACCESS_
#ifdef _NEW_MODEM_PI_ACCESS_BY_VENDOR_REG_
        if (1)
#else
        if (g_efuse_modem_pi_enable)
#endif

        {
            RTK_WRITE_MODEM_REG_PI(MODEM_PI_PAGE_NEW_AGC_TAB0, ii, g_new_modem_new_agc_table0[ii]);
        }
        else
#endif
        {
            modem_agc_table.addr = ii;
            modem_agc_table.value = g_new_modem_new_agc_table0[ii];
            rtk_write_modem_radio_reg(0x1f, TYPE_MODEM, modem_agc_table.d16);
        }
    }
  #endif
  #endif
/***** Table 1 *****/
  #ifdef _NEW_MODEM_OLD_AAGC
    modem_reg.reg3c.r_table_sel = 0;
  #else
    modem_reg.reg3c.r_table_sel = 1;
  #endif
    rtk_write_modem_radio_reg(TRANS_MODEM_REG(0x3c), TYPE_MODEM, modem_reg.d16);

    for (ii = 0; ii<0x40; ii++)
    {
        otp_old_agc_table.d16 = *((UINT16 *)(((UINT8 *)&otp_rf_str_data.mdm_rxagc_tbl_00)+ii*2));
//        RT_BT_LOG(YELLOW, DAPE_TEST_LOG213, 2, ii, otp_old_agc_table.d16);

#ifdef _NEW_MODEM_PI_ACCESS_
#ifdef _NEW_MODEM_PI_ACCESS_BY_VENDOR_REG_
        if (1)
#else
        if (g_efuse_modem_pi_enable)
#endif
        {
  #ifdef _NEW_MODEM_OLD_AAGC
            RTK_WRITE_MODEM_REG_PI(MODEM_PI_PAGE_OLD_AGC_TAB0, otp_old_agc_table.addr, otp_old_agc_table.value);
  #else
            RTK_WRITE_MODEM_REG_PI(MODEM_PI_PAGE_NEW_AGC_TAB1, otp_old_agc_table.addr, otp_old_agc_table.value);
  #endif
        }
        else
#endif
        {
            modem_agc_table.addr = otp_old_agc_table.addr;
            modem_agc_table.value = otp_old_agc_table.value;
            rtk_write_modem_radio_reg(0x1f, TYPE_MODEM, modem_agc_table.d16);
        }
    }
/***** Table 2 *****/
  #ifndef _NEW_MODEM_OLD_AAGC
    modem_reg.reg3c.r_table_sel = 2;
    rtk_write_modem_radio_reg(TRANS_MODEM_REG(0x3c), TYPE_MODEM, modem_reg.d16);
  #ifdef _NEW_MODEM_DESIGN_PHY_SETTING_
    for (ii = 0; ii<NEW_MODEM_NEW_AGC_TABLE2_SIZE; ii++)
    {
#ifdef _NEW_MODEM_PI_ACCESS_
#ifdef _NEW_MODEM_PI_ACCESS_BY_VENDOR_REG_
        if (1)
#else
        if (g_efuse_modem_pi_enable)
#endif
        {
            RTK_WRITE_MODEM_REG_PI(MODEM_PI_PAGE_NEW_AGC_TAB2, ii, g_new_modem_new_agc_table2[ii]);
        }
        else
#endif
        {
            modem_agc_table.addr = ii;
            modem_agc_table.value = g_new_modem_new_agc_table2[ii];
            rtk_write_modem_radio_reg(0x1f, TYPE_MODEM, modem_agc_table.d16);
        }
    }
  #endif
  #endif
/***** Table 3 *****/
  #ifndef _NEW_MODEM_OLD_AAGC
    modem_reg.reg3c.r_table_sel = 3;
    rtk_write_modem_radio_reg(TRANS_MODEM_REG(0x3c), TYPE_MODEM, modem_reg.d16);
  #ifdef _NEW_MODEM_DESIGN_PHY_SETTING_
    for (ii = 0; ii<NEW_MODEM_NEW_AGC_TABLE3_SIZE; ii++)
    {
#ifdef _NEW_MODEM_PI_ACCESS_
#ifdef _NEW_MODEM_PI_ACCESS_BY_VENDOR_REG_
        if (1)
#else
        if (g_efuse_modem_pi_enable)
#endif
        {
            RTK_WRITE_MODEM_REG_PI(MODEM_PI_PAGE_NEW_AGC_TAB3, ii, g_new_modem_new_agc_table3[ii]);
        }
        else
#endif
        {
            modem_agc_table.addr = ii;
            modem_agc_table.value = g_new_modem_new_agc_table3[ii];
            rtk_write_modem_radio_reg(0x1f, TYPE_MODEM, modem_agc_table.d16);
        }
    }
  #endif
  #endif
#endif
}


#if defined(_FPGA_WITH_RLE0546_RFE_)||defined(_FPGA_WITH_RLE0608C_RFE_)||defined(_FPGA_WITH_RLE0550_RFE_)
typedef union _RFE_TABLE_FORMAT_S_{
    UINT32 d32;
    struct
    {
        UINT32 data_lsb:16;     //[15:0]
        UINT32 data_msb:4;      //[19:16]
        UINT32 addr:8;          //[27:20]
        UINT32 si_sel:4;        //[31:28]
    };
    struct /* valid when si_sel = 0xF */
    {
        UINT32 delay_time:28;   //[27:0]
        UINT32 rsvd:4;          //[31:28]
    };
}RFE_TABLE_FORMAT_S_TYPE;
#define RFE_TABLE_ENDING_PATTERN 0xFFFFFFFF
#endif // #if defined(_FPGA_WITH_RLE0546_RFE_)||defined(_FPGA_WITH_RLE0608C_RFE_)||defined(_FPGA_WITH_RLE0550_RFE_)

#if defined(_FPGA_WITH_RLE0546_RFE_)
/* {SI_SEL[3:0], ADDR[7:0], DATA[19:0]} */
const UINT32 RFE_20BIT_INIT_TABLE[] =
    {
        0x00000C1C,
        0x0015830A,
        0x002D0000,
        0x00319791,
        0x004FB484,
        0x00500031,
        0x006053C1,
        0x00707C43,
        0x008808E0,
//        0x00777C43,  // only for test
//        0x0088081C,  // only for test, inc. cap
        0x00900002,
        0x00A9FF00,
        0x00B8D971,
        0x00C2AA7F,
        0x00D2418A,
        0x00EEA555,
        0x00F00603,
        0xF0001000, // Addition Delay for XTAL ON
        // WLAN      'SEL[1:0]=01
        0x1010002F,
        0x10200023,
        //0x00A9EF00,  //wlan pll off test by yichang
        //BT_AFE    'SEL[1:0]=10
#if 1   /* External ADC Clock */
        0x20002A54,  //[11]: ADC external clock, [14:13]: SH gain(2/1/0.7/0.7)
#else   /* Internal ADC Clock */
        0x20002254,  //[11]: ADC external clock, [14:13]: SH gain(2/1/0.7/0.7)
#endif
        0x2010C06B,
        0x202065A7,
        0x20303002,
#if 1   /* External DAC Clock */
        0x2010C46B,  //[10]: DAC external clock
#else
        0x2010C06B,  //[10]: DAC external clock
#endif
//        0x20404E2B,  //'DAC CLK 8M, ADC CLK 20M
        0x20404B2B,  //'DAC CLK 10M, ADC CLK 20M
#ifdef _RLE0546_RFE_EXT_26M_
        0x2050140B,  // 26MHz XTAL ([4:1]=5)
#else
        0x20501401, // 40MHz XTAL
#endif
        0x20600000,
        0x207FEFE0,
        0x20805963,
        0x20924082,
        0x20A80000,
#ifdef _RLE0546_RFE_EXT_26M_
        0x20B8D951, // 26MHz XTAL  ([7:4]=5)
#else
        0x20B8D901, // 40MHz XTAL
#endif
        0x20C2AA7F,
        0x20D2418A,
        0x20EEA555,
#ifdef _RLE0546_RFE_EXT_26M_
        0x20FA0622, // [17], RFC clock from PLL
        0x202065A6, // BT PLL OFF
        0x202065A7, // BT PLL ON
#else
        0x20F80622, // [17]
#endif

        0xF0000100,
        RFE_TABLE_ENDING_PATTERN   // ENDING PATTERN
    };

/* {SI_SEL[3:0], ADDR[7:0], DATA[19:0]}, only DATA[15:0] is used*/
const UINT32 RFE_16BIT_INIT_TABLE[] =
    {
        //BT        'SEL[1:0]=11
        0x30001006,
#if 1
        0x30E0102A,  //'enable TX IDAC
        0x31602400,  //'IF/BW Table 1 ==> 1.4M/1.6M
        0x31600000,  //'IF/BW Table 0 ==> 1.4M/1.3M
        0x31C08120,  //'RXBB RCK -2bits
        0x31F0ADC3,  //'PA bias current=001
        0x3210A793,  //'TX PAD LC TANK=011
//        0x32307980,  //'TX IDAC current for LO Leakage
        0x323079C0,  //'TX IDAC current for LO Leakage
        0x3240FE00,  //'EN_ACC OFF
        0x32600425,  //'SYN CP
        0x32700060,  //'CP_RSW[2:0]=000
        0x3280C1A8,  //'SYN LPF, C1A8 for TX, 80C2 for RX
        0x3290FA48,  //'SYN LPF, FA48 for TX, 47C8 for RX
        0x32A040C0,  //'SYN
        0x32D00008,  //'VCO_ISW[3:0]=0100
        //Page3
        0x3370C004,
        //Page4     '0E=0902A
        0x30E0902A,
        0x33105001,
        0x33200071,
        0x33306F70,  // error?
        // restore Page
        0x30E0102A,  //'enable TX IDAC
        // 1. LCK
        0x32A046C0,  // 2A[10:9]=11 --> 24[0]=1 --> 2A[10:9]=00      error? 046C0
        0x3240FE01,
        0xF0040000,  // 256ms delay
        0x32A040C0,
        // 2. RCK
        0x31C08121,  // 1C[0]=1
        0xF0010000,  // 64ms delay or 16us?
        // 3. 改寫TXRF Gain Table
        0x31500000,  // 15[15:0]=0000
        0x31502080,  // 15[15:0]=2080
        0x31504880,  // 15[15:0]=4880
        0x31506980,  // 15[15:0]=6980
        0x31509180,  // 15[15:0]=9180
        0x3150B980,  // 15[15:0]=B980
        0x3150DA00,  // 15[15:0]=DA00
        0x3150FA80,  // 15[15:0]=FA80
        // (FOR TEST) initialized RXAGC as 0xB3
        0x3010B300,
#if 1
        0x33F00000,  // change channel to latch the new LCK results
        0x33F00001,
#endif
#endif
        RFE_TABLE_ENDING_PATTERN   // ENDING PATTERN
    };

#elif defined(_FPGA_WITH_RLE0608C_RFE_)
    TBD
#elif defined(_FPGA_WITH_RLE0550_RFE_)
/* {SI_SEL[3:0], ADDR[7:0], DATA[19:0]} */
const UINT32 RFE_20BIT_INIT_TABLE[] =
    {
        // WLAN_AFE
        0x00000C1C,
        0x00101B20,
        0x00280000,
        0x00300004,
        0x004B6E10,
        0x005BE1A1,
        0x00675011,
        0x007C2CB1,
        0x00803820,
        0x00940000,
        0x00A9FFF0,
        0x00B8D971,
        0x00C2AA7F,
        0x00D2418A,
        0x00EEA555,
        0x00F00003,
        0x01000000,
        0x01100000,
        0x01280000,
        0x01300004,
        0x014B6E10,
        0x015BE1A1,
        0x01675011,
        0x017C2CB1,
        0x01803820,
        0x01940000,
        0x01A9FFF0,
        0x01B8D971,
        0x01C2AA7F,
        0x01D2418A,
        0x01EEA555,
        0x01F00003,

        // WLAN      'SEL[1:0]=01
        //TBD!!!

        // BT_AFE    'SEL[1:0]=10
        0x20000A54,
        0x2010C46B,
        0x202065A7,
        0x20303002,
        0x20404E2B,
        0x20500000,
        0x20600000,
        0x20700000,
        0x20800000,
        0x20900000,
        0x20A00000,
        0x20B00000,
        0x20C00000,
        0x20D00000,
        0x20E00000,
        0x20FA0000,

        0xF0000100,
        RFE_TABLE_ENDING_PATTERN   // ENDING PATTERN
    };

/* {SI_SEL[3:0], ADDR[7:0], DATA[19:0]}, only DATA[15:0] is used*/
const UINT32 RFE_16BIT_INIT_TABLE[] =
    {
        //BT        'SEL[1:0]=11
        0x32E0A795,
        0x3320A795,
        0x33F00001,
        0x30E0800A,
        0x33007813,
        0x33103C9F,
        0x33200060,
        0x3330FB0A,
        0x3340E330,
        0x335040D0,
        0x33604000,
        0x33700690,
        0x33800006,
        0x33900270,
        0x33A00000,
        0x33B01400,
        0x33C00007,
        0x33D00000,
        0x33E00000,
        0x33F0C000,
        0x30E0000A,
//        0x31600000,
//        0x31602400,
        0x3130788F,
        0x3190788F,
        0x30001006,
        0x31C00021,

        // LCK
        0x30001006,  // set RF standby mode
        0xF0000400,  //delay 1ms
        0x30E0800A,
        0x335046D0,
        0x30E0000A,
        0x32400001,
        0xF0050000,  //delay 300ms
        0x30E0800A,
        0x335040D0,
        0x30E0000A,
        0x33F00003,

        // RCK
        0x30001006,  // set RF standby mode
        0xF0000400,  //delay 1ms
        0x31C00021,
        0xF0001000,  //delay 4ms
//        0x33F00000,  // change channel to latch the new LCK results
//        0x33F00001,

        0xF0000400,  //delay 1ms
//        0x30000000,  // set RF shutdown mode

        RFE_TABLE_ENDING_PATTERN   // ENDING PATTERN
    };
#endif //#if defined(_FPGA_WITH_RLE0546_RFE_) #elif ...


#if defined(_FPGA_WITH_RLE0546_RFE_)||defined(_FPGA_WITH_RLE0608C_RFE_)||defined(_FPGA_WITH_RLE0550_RFE_)
void rtk_rf_set_si_sel(UINT8 si_sel)
{

    /* DPDT_P: si_sel[1], BT-ONLY V5 FPGA: AM29
     * DPDT_N: si_sel[0], BT-ONLY V5 FPGA: AN29 */

#if defined(_FPGA_WITH_RLE0550_RFE_)
    UINT32 u32_tmp;
    u32_tmp = VENDOR_READ(0x3F8);
    u32_tmp = u32_tmp & 0xFFFFFFFC;
    u32_tmp = u32_tmp + si_sel;
    VENDOR_WRITE(0x3F8, u32_tmp);
#else
  #ifdef _NEW_BTON_DESIGN_AFTER_RTL8821B_TC_
    BTON_AC_REG_S_TYPE bton_0xac;
    bton_0xac.d32 = VENDOR_READ(BTON_AC_REG);
    bton_0xac.BT_RFE_OE = 1;
    bton_0xac.BT_RFE_CTRL &= (~(BIT1|BIT0));
    bton_0xac.BT_RFE_CTRL |= (si_sel&(BIT1|BIT0));
    VENDOR_WRITE(BTON_AC_REG, bton_0xac.d32);
  #else
    BTON_BT_RFE_PAD_CTRL_REG_S_TYPE bton_0xac;
    bton_0xac.d32 = VENDOR_READ(BTON_BT_RFE_PAD_CTRL_REG);
    bton_0xac.BT_RFE_OE = 1;
    bton_0xac.BT_RFE_CTRL &= (~(BIT1|BIT0));
    bton_0xac.BT_RFE_CTRL |= (si_sel&(BIT1|BIT0));
    VENDOR_WRITE(BTON_BT_RFE_PAD_CTRL_REG, bton_0xac.d32);
  #endif
#endif

#if 1   // add delay to asure the settling of SI_SEL[1:0] //
    pf_delay_us(10);
#endif
}


void rtk_rf_init_combo_wifi(void)
{
    /* Write vendor register 0x150[2] to 0 for bluewiz to switch to wifi*/
    /* Write modem reg0a[0] to 1 for modem to switch to wifi*/
    /* rega[4:1] is for wifi data[19:16]*/
    // example:
    //    UINT32 modem_reg_a = rtk_read_modem_radio_reg(TRANS_MODEM_REG(0xa), TYPE_MODEM);
    //    modem_reg_a |= BIT0;
    //    rtk_write_modem_radio_reg(TRANS_MODEM_REG(0xa), TYPE_MODEM, modem_reg_a);
    //    UINT32 modem_reg_a_tmp = rtk_read_modem_radio_reg(TRANS_MODEM_REG(0xa), TYPE_MODEM);
    //    rtk_write_modem_radio_reg(TRANS_MODEM_REG(0xa), TYPE_MODEM, ((7<<1)|BIT0));
    //    rtk_write_modem_radio_reg(0x02, TYPE_RF,0x0D5D);

    /* switch to modem page 0*/
    RTK_WRITE_MODEM_REG(0x00, 0x0000);

    /* switch to WiFi SI mode */
    MODEM_REG_S_TYPE modem_reg_0a;
    modem_reg_0a.d16 = RTK_READ_MODEM_REG(TRANS_MODEM_REG(0xa));
    modem_reg_0a.reg0a.r_wifi_si_en_fpga = 1;

    /* start writing to RFE */
    UINT16 ii = 0;
    RFE_TABLE_FORMAT_S_TYPE rfe_table;

    for (ii = 0;;ii++)
    {
        if (RFE_20BIT_INIT_TABLE[ii] == RFE_TABLE_ENDING_PATTERN)
            break;
        rfe_table.d32 = RFE_20BIT_INIT_TABLE[ii];

        if (rfe_table.si_sel == 0xF)
        {
            pf_delay_us(rfe_table.delay_time);
        }
        else
        {
            rtk_rf_set_si_sel(rfe_table.si_sel);
            modem_reg_0a.reg0a.r_wifi_msb_fpga = rfe_table.data_msb;
            RTK_WRITE_MODEM_REG(TRANS_MODEM_REG(0xa), modem_reg_0a.d16);
            RTK_WRITE_RF_REG(rfe_table.addr, rfe_table.data_lsb);
        }
    }


    /* switch back to BT SI mode */
    /* Write modem reg0a[0] to 0 for modem to switch back to BT*/
    modem_reg_0a.reg0a.r_wifi_si_en_fpga = 0;
    RTK_WRITE_MODEM_REG(TRANS_MODEM_REG(0xa), modem_reg_0a.d16);

}


void rtk_rf_init_combo_bt(void)
{

    /* switch to modem page 0*/
    RTK_WRITE_MODEM_REG(0x00, 0x0000);

    /* switch to BT SI mode */
    MODEM_REG_S_TYPE modem_reg_0a;
    modem_reg_0a.d16 = RTK_READ_MODEM_REG(TRANS_MODEM_REG(0xa));
    modem_reg_0a.reg0a.r_wifi_si_en_fpga = 0;
    RTK_WRITE_MODEM_REG(TRANS_MODEM_REG(0xa), modem_reg_0a.d16);

    /* start writing to RFE */
    UINT16 ii = 0;
    RFE_TABLE_FORMAT_S_TYPE rfe_table;

    for (ii = 0;;ii++)
    {
        if (RFE_16BIT_INIT_TABLE[ii] == RFE_TABLE_ENDING_PATTERN)
            break;
        rfe_table.d32 = RFE_16BIT_INIT_TABLE[ii];
        if (rfe_table.si_sel == 0xF)
        {
            pf_delay_us(rfe_table.delay_time);
//            RT_BT_LOG(BLUE, YL_DBG_HEX_2, 2, ii, rfe_table.delay_time);
        }
        else
        {
            rtk_rf_set_si_sel(rfe_table.si_sel);
            RTK_WRITE_RF_REG(rfe_table.addr, rfe_table.data_lsb);
//            RT_BT_LOG(GRAY, YL_DBG_HEX_5, 5, ii,
//                        rfe_table.addr,
//                        rfe_table.data_lsb,
//                        RTK_READ_RF_REG(rfe_table.addr),
//                        RTK_READ_RF_REG(0x00));
        }
    }

}
#endif // #if defined(_FPGA_WITH_RLE0546_RFE_)||defined(_FPGA_WITH_RLE0608C_RFE_)||defined(_FPGA_WITH_RLE0550_RFE_)


#ifdef _TEST_DUMP_RLE0550_RFE_REGISTERS_
void rtk_rf_dump_rle0550_rfe(void)
{
    UINT16 dump[16];
    UINT8 ii;
    
    for (ii = 0; ii <16; ii ++)
    {
        dump[ii] = RTK_READ_RF_REG(0x00+ii);
    }
    RT_BT_LOG(YELLOW, YL_DUMP_REG_16B, 17, 0x00, dump[0], dump[1], dump[2], dump[3], dump[4], dump[5], dump[6], dump[7], 
                                                      dump[8], dump[9], dump[10], dump[11], dump[12], dump[13], dump[14], dump[15]);
    
    for (ii = 0; ii <16; ii ++)
    {
        dump[ii] = RTK_READ_RF_REG(0x10+ii);
    }
    RT_BT_LOG(YELLOW, YL_DUMP_REG_16B, 17, 0x10, dump[0], dump[1], dump[2], dump[3], dump[4], dump[5], dump[6], dump[7], 
                                                      dump[8], dump[9], dump[10], dump[11], dump[12], dump[13], dump[14], dump[15]);

    for (ii = 0; ii <16; ii ++)
    {
        dump[ii] = RTK_READ_RF_REG(0x20+ii);
    }
    RT_BT_LOG(YELLOW, YL_DUMP_REG_16B, 17, 0x20, dump[0], dump[1], dump[2], dump[3], dump[4], dump[5], dump[6], dump[7], 
                                                      dump[8], dump[9], dump[10], dump[11], dump[12], dump[13], dump[14], dump[15]);

    for (ii = 0; ii <16; ii ++)
    {
        dump[ii] = RTK_READ_RF_REG(0x30+ii);
    }
    RT_BT_LOG(YELLOW, YL_DUMP_REG_16B, 17, 0x30, dump[0], dump[1], dump[2], dump[3], dump[4], dump[5], dump[6], dump[7], 
                                                      dump[8], dump[9], dump[10], dump[11], dump[12], dump[13], dump[14], dump[15]);

    RTK_UPDATE_RF_REG(0x0E, BIT15, BIT15);
    
    for (ii = 0; ii <16; ii ++)
    {
        dump[ii] = RTK_READ_RF_REG(0x30+ii);
    }
    RT_BT_LOG(YELLOW, YL_DUMP_REG_16B, 17, 0x40, dump[0], dump[1], dump[2], dump[3], dump[4], dump[5], dump[6], dump[7], 
                                                          dump[8], dump[9], dump[10], dump[11], dump[12], dump[13], dump[14], dump[15]);
    for (ii = 0; ii <16; ii ++)
    {
        dump[ii] = RTK_READ_RF_REG(0x20+ii);
    }
    RT_BT_LOG(YELLOW, YL_DUMP_REG_16B, 17, 0x50, dump[0], dump[1], dump[2], dump[3], dump[4], dump[5], dump[6], dump[7], 
                                                          dump[8], dump[9], dump[10], dump[11], dump[12], dump[13], dump[14], dump[15]);
    RTK_UPDATE_RF_REG(0x0E, BIT15, 0);
    
}
#endif

void rtk_rf_init(void)
{

#if defined(_FPGA_WITH_RLE0546_RFE_)||defined(_FPGA_WITH_RLE0608C_RFE_)||defined(_FPGA_WITH_RLE0550_RFE_)
    rtk_rf_init_combo_wifi();
    rtk_rf_init_combo_bt();
 #ifdef _TEST_DUMP_RLE0550_RFE_REGISTERS_
    rtk_rf_dump_rle0550_rfe();
 #endif
#elif defined(_FPGA_WITH_RLE0379_RFE_)

    UINT16 ini_num, i;
    ini_num = otp_rf_str_data.rf_ini_num;
    for(i=0; i<ini_num; i++)
    {
        if ((otp_rf_str_data.rf_ini_offset[i] != 0xff) &&
            (otp_rf_str_data.rf_ini_offset[i] != 0xfe))
        {
            rtk_write_modem_radio_reg(otp_rf_str_data.rf_ini_offset[i], TYPE_RF,
                                      otp_rf_str_data.rf_ini_value[i]);
        }
        else
        {
#ifndef FOR_SIMULATION
            pf_delay(otp_rf_str_data.rf_ini_value[i]);
#endif
        }
    }
#ifdef _TEST_ADAPTIVITY_FUNC_2
    // AD/DA always on
    /* RF Reg1e = 0x64ab  //DA on */
    /* RF Reg1c = 0xa54a  //AD on */

    UINT32 reg1e = rtk_read_modem_radio_reg(0x1E, TYPE_RF);
    UINT32 reg1c = rtk_read_modem_radio_reg(0x1C, TYPE_RF);

    rtk_write_modem_radio_reg(0x1E, TYPE_RF, 0x64AB);
    rtk_write_modem_radio_reg(0x1C, TYPE_RF, 0xA54A);

    RT_BT_LOG(WHITE, YL_DBG_HEX_4, 4, reg1e, reg1c, rtk_read_modem_radio_reg(0x1E, TYPE_RF),
        rtk_read_modem_radio_reg(0x1C, TYPE_RF));
#endif
#else
    UNEXPECTED COMPILE OPTION
#endif
}
#endif


#ifdef _NEW_MODEM_PI_ACCESS_
UCHAR lc_read_modem_pi_page(void)
{
    UINT16 read_val;
    read_val = BB_read_baseband_register(EXTENDED_HOST_ACCESS_REGISTER);
    read_val = read_val>>12;
    return read_val;
}

void lc_write_modem_pi_page(UCHAR modem_page)
{
    UINT16 read_val;
    UINT16 write_val;
    UINT16 addr_7_0 = 0;
    //DEF_CRITICAL_SECTION_STORAGE;

    //MINT_OS_ENTER_CRITICAL();

#ifdef _RF_CONFIG_PROTECTION_
    lc_host_addr_11_6_mask = (modem_page<<12) | ((addr_7_0 & 0xC0)<<4);
#endif

    read_val = BB_read_baseband_register(EXTENDED_HOST_ACCESS_REGISTER);
    write_val = read_val & 0x03FF;
    write_val = write_val | (modem_page<<12) | ((addr_7_0 & 0xC0)<<4);
    BB_write_baseband_register(EXTENDED_HOST_ACCESS_REGISTER, write_val);

    //MINT_OS_EXIT_CRITICAL();

#ifdef _YL_TEST_NEW_MODEM_PI_ACCESS_
    RT_BT_LOG(YELLOW, YL_DBG_HEX_2, 2, EXTENDED_HOST_ACCESS_REGISTER, BB_read_baseband_register(EXTENDED_HOST_ACCESS_REGISTER));
#endif

}


#ifdef _NEW_RFC_PI_ACCESS_
UINT16 rtk_read_rfc_reg_pi(UINT8 rfc_addr)
{
    UINT16 read_value;
#ifdef _ROM_CODE_PATCHED_
    if (rcp_rtk_read_rfc_reg_pi != NULL)
    {
        UINT16 patch_data;
        if (rcp_rtk_read_rfc_reg_pi((void*)&patch_data, rfc_addr))
        {
            return patch_data;
        }
    }
#endif

    VENDOR_PI_ACCESS_RFC_STR pi_rfc_access;

    pi_rfc_access.rfc_pi_sel = 0;
    pi_rfc_access.rf_iow = PI_RFC_READ;
    pi_rfc_access.bt_ioad_rfc = rfc_addr;
    VENDOR_WRITE(REG_BTRFC_PI_IF, pi_rfc_access.d32);
    read_value = VENDOR_READ(REG_BTRFC_PI_IF);

    return read_value;
}



void rtk_write_rfc_reg_pi(UINT8 u8rfc_addr, UINT16 assignvalue,  UINT8 park_rfc_pi_sel)
{

#ifdef _ROM_CODE_PATCHED_
    if (rcp_rtk_write_rfc_reg_pi != NULL)
    {

        if (rcp_rtk_write_rfc_reg_pi((void *)&u8rfc_addr, assignvalue, park_rfc_pi_sel))
        {
            return ;
        }
    }
#endif

    VENDOR_PI_ACCESS_RFC_STR pi_rfc_access;
    pi_rfc_access.rfc_pi_sel = PI_RFC_ARBITER_CPU;
    pi_rfc_access.rf_iow = PI_RFC_WRITE;
    pi_rfc_access.bt_ioad_rfc = u8rfc_addr;
    pi_rfc_access.bt_ioq_rfc = assignvalue;
    VENDOR_WRITE(REG_BTRFC_PI_IF, pi_rfc_access.d32);

    if(park_rfc_pi_sel == 0)
    {
        return;
    }
    // park rfc_pi_sel to modem access
    pi_rfc_access.rfc_pi_sel = PI_RFC_ARBITER_MODEM;
    pi_rfc_access.rf_iow = PI_RFC_READ;
    pi_rfc_access.bt_ioad_rfc = u8rfc_addr;
    VENDOR_WRITE(REG_BTRFC_PI_IF, pi_rfc_access.d32);
    //read_value = VENDOR_READ(REG_BTRFC_PI_IF);

    //return;
}
#endif /* endof #ifdef _NEW_RFC_PI_ACCESS_ */


UINT32 rtk_read_modem_radio_reg_pi(UCHAR modem_page, UCHAR addr, UCHAR type)
{
    UINT32 read_value = 0;
#ifdef _ROM_CODE_PATCHED_
    // Reserved com code patch by yilinli //
    if (rcp_rtk_read_modem_radio_reg_pi != NULL)
    {
        UINT32 data;
        if (rcp_rtk_read_modem_radio_reg_pi((void *)&data, modem_page, addr, type))
        {
            return data;
        }
    }
#endif

    //DEF_CRITICAL_SECTION_STORAGE;

#ifdef _NEW_MODEM_PI_ACCESS_BY_VENDOR_REG_
// TODO:        To be finished
    UINT32 u32PiAddr;
    //u32PiAddr = ((UINT32)modem_page<<24)|(UINT32)addr<<16;
    u32PiAddr = ((UINT32)modem_page<<22)|(UINT32)(addr&0x3F)<<16;
    u32PiAddr = u32PiAddr&0xEFFFFFFF;// clear bit 28, for read
    VENDOR_WRITE(REG_BTPHY_FPI_IF, u32PiAddr);
    __NOP();
    read_value = VENDOR_READ(REG_BTPHY_FPI_IF);
    read_value = read_value&0xFFFF;
#else
    if( (type == TYPE_MODEM) && (g_efuse_modem_pi_enable))
    {
        lc_write_modem_pi_page(modem_page);
        if ((modem_page>=MODEM_PI_PAGE_W_ADDR0_START) && (modem_page<=MODEM_PI_PAGE_W_ADDR0_END))
        {
            //MINT_OS_ENTER_CRITICAL();
            g_efuse_modem_pi_addr0_convert = 0;
            read_value = rtk_read_modem_radio_reg(addr, type);
            g_efuse_modem_pi_addr0_convert = 1;
            //MINT_OS_EXIT_CRITICAL();
        }
        else
        {
            read_value = rtk_read_modem_radio_reg(addr, type);
        }
        // switch back to page 0, to avoid wrong access or original FW codes //
        lc_write_modem_pi_page(MODEM_PI_PAGE_0);
    }
    else
    {
        read_value = rtk_read_modem_radio_reg(addr, type);
    }
#endif

    return read_value;
}



//void rtk_write_modem_radio_reg_pi(UCHAR modem_page, UCHAR addr, UCHAR type, UINT32 val)
void rtk_write_modem_radio_reg_pi(UCHAR modem_page, UCHAR addr, UCHAR type, UINT16 val)
{

#ifdef _ROM_CODE_PATCHED_
    // Reserved com code patch by yilinli //
    if (rcp_rtk_write_modem_radio_reg_pi != NULL)
    {
        rcp_rtk_write_modem_radio_reg_pi(NULL, modem_page, addr, type, val);
        return;
    }
#endif

    //DEF_CRITICAL_SECTION_STORAGE;

#ifdef _FORBID_WRITE_MODEM_PAGE0_ADDR0_VIA_PI_
    if ((modem_page == 0) && (addr == 0))
    {
        /* our modem does not ALLOW to write modem page 0 and address 0
           via PI (austin) */
        return;
    }
#endif

#ifdef _NEW_MODEM_PI_ACCESS_BY_VENDOR_REG_
// TODO:         To be finished
    UINT32 u32PiAddrData;
    //u32PiAddrData = ((UINT32)modem_page<<24)|((UINT32)addr<<16)|val;
    u32PiAddrData = ((UINT32)modem_page<<22)|((UINT32)(addr&0x3F)<<16)|val;
    u32PiAddrData = u32PiAddrData|0x10000000;// enable bit 28, for write, hard code for speed up
    VENDOR_WRITE(REG_BTPHY_FPI_IF, u32PiAddrData);
#else
    if( (type == TYPE_MODEM) && (g_efuse_modem_pi_enable))
    {
        lc_write_modem_pi_page(modem_page);
        if ((modem_page>=MODEM_PI_PAGE_W_ADDR0_START) && (modem_page<=MODEM_PI_PAGE_W_ADDR0_END))
        {
            //MINT_OS_ENTER_CRITICAL();
            g_efuse_modem_pi_addr0_convert = 0;
            rtk_write_modem_radio_reg(addr, type, val);
            g_efuse_modem_pi_addr0_convert = 1;
            //MINT_OS_EXIT_CRITICAL();
        }
        else
        {
            rtk_write_modem_radio_reg(addr, type, val);
        }
        // switch back to page 0, to avoid wrong access or original FW codes //
        lc_write_modem_pi_page(MODEM_PI_PAGE_0);
    }
    else
    {
        rtk_write_modem_radio_reg(addr, type, val);
    }
#endif
}

#ifndef _NEW_MODEM_PI_ACCESS_BY_VENDOR_REG_
// NOTE: TRIG_2NDID_REGISTER should avoid to be modify during run-time except lc_write_modem_pi_si_sel() //
void lc_write_modem_pi_si_sel(UINT8 modem_pi_enable)
{
    // set the pi/si according to g_efuse_modem_pi_enable //
    UINT16 temp_0xd4 = BB_read_baseband_register(TRIG_2NDID_REGISTER);
    if (modem_pi_enable)
    {
        temp_0xd4 |= BIT15;
    }
    else
    {
        temp_0xd4 &= (~BIT15);
    }
    BB_write_baseband_register(TRIG_2NDID_REGISTER, temp_0xd4);
#ifdef _YL_TEST_NEW_MODEM_PI_ACCESS_
//    RT_BT_LOG(YELLOW, YL_DBG_HEX_4, 4, modem_pi_enable, temp_0xd4, TRIG_2NDID_REGISTER, BB_read_baseband_register(TRIG_2NDID_REGISTER));
#endif
    g_modem_pi_si_sel_init_completed = 1;
}
#endif

#ifndef _NEW_MODEM_PI_ACCESS_BY_VENDOR_REG_
void rtk_modem_access_warning_message(UCHAR addr, UCHAR type, UINT32 val, UINT8 read1_write0)
{
    // warning message for un-initialized access //
    if (g_efuse_modem_pi_enable == 0xff)
    {
        RT_BT_LOG(RED, MDM_LOG_012, 0, 0);
    }
    // warning message for run-time writing of non-zero pages //
    if ( (type == TYPE_MODEM) && (addr == 0) && (!g_efuse_modem_pi_enable) && (g_modem_init_completed) )
    {
        RT_BT_LOG(RED, MDM_LOG_013, 2, val, read1_write0);
    }
}
#endif
#endif



UINT32 rtk_read_modem_radio_reg(UCHAR addr, UCHAR type)
{
    UINT32 reg_val;
    UCHAR rep;

    reg_val = 0;


#ifdef _ROM_CODE_PATCHED_
    // Reserved com code patch by yilinli //
    if (rcp_rtk_read_modem_radio_reg != NULL)
    {
        UINT32 data;
        if (rcp_rtk_read_modem_radio_reg((UINT8*)&data, addr, type))
        {
            return data;
        }
    }
#endif

#if !defined(_USE_RLE0380_RF_) && !defined(_USE_RLE0546_RF_)
    UINT32 rvalue = 0;
    UINT8 addr_tmp = addr;
#endif

    if(type == TYPE_RF)   //SI, read twice.
    {
        rep = 2;

#if !defined(_USE_RLE0380_RF_) && !defined(_USE_RLE0546_RF_)
        if (addr_tmp > 0x3F) //For 0379, the extended register 0x40,0x41,0x42
        {
            /* select RF page */
            rvalue = lc_read_radio_reg(0x71);
            lc_write_radio_reg(0x71, rvalue | BIT12);
            addr &= 0x3f;
        }
#endif
    }
    else
    {
#ifdef _YL_TEST_MODEM_PI_TIMING_BY_BT_GPIO
       SET_BT_GPIO_OUTPUT_3_0(3);
#endif

        rep = 1;

        addr &= 0x3F;

#ifdef _NEW_MODEM_PI_ACCESS_
#ifndef _NEW_MODEM_PI_ACCESS_BY_VENDOR_REG_
        rtk_modem_access_warning_message(addr, type, 0xffffffff, 1);
    //    if (g_efuse_modem_pi_enable == 0xff)
    //    {
    //        RT_BT_LOG(RED, MDM_LOG_012, 0, 0);
    //    }
    //    if ((type == TYPE_MODEM) && (addr == 0) && (!g_efuse_modem_pi_enable) && (g_modem_init_completed))
    //    {
    //        RT_BT_LOG(RED, MDM_LOG_008, 0, 0);
    //    }

        // switch page in different ways according to PI or SI //
        if ((addr == 0) && g_efuse_modem_pi_enable && g_efuse_modem_pi_addr0_convert)
        {
            return lc_read_modem_pi_page();
        }

        lc_write_modem_pi_si_sel(g_efuse_modem_pi_enable);
#endif
#endif
    }

    while (rep--)
    {
        if (type == TYPE_MODEM)
        {
            reg_val = lc_read_radio_reg(0x40 | addr );
        }
        else if (type == TYPE_RF)
        {
            reg_val = lc_read_radio_reg(addr );
        }
    }

#if !defined(_USE_RLE0380_RF_) && !defined(_USE_RLE0546_RF_)
    if ((type == TYPE_RF) && (addr_tmp > 0x3F))
    {
        /* select original page */
        lc_write_radio_reg(0x71, rvalue);
    }
#endif

    return (reg_val);
}

void rtk_write_modem_radio_reg(UCHAR addr, UCHAR type, UINT32 val)
{

#ifdef _ROM_CODE_PATCHED_
    // Reserved com code patch by yilinli //
    if (rcp_rtk_write_modem_radio_reg != NULL)
    {
        if (rcp_rtk_write_modem_radio_reg(NULL, addr, type, val))
        {
            return;
        }
    }
#endif

    if (type == TYPE_MODEM)
    {
        addr &= 0x3F;

#ifdef _NEW_MODEM_PI_ACCESS_
#ifndef _NEW_MODEM_PI_ACCESS_BY_VENDOR_REG_
        if (!g_efuse_modem_reg_rw_fast_mode)
        {
            rtk_modem_access_warning_message(addr, type, val, 0);
        }
    //    if (g_efuse_modem_pi_enable == 0xff)
    //    {
    //        RT_BT_LOG(RED, MDM_LOG_012, 0, 0);
    //    }
    //    // warning message for run-time writing of non-zero pages //
    //    if ( (type == TYPE_MODEM) && (addr == 0) && (!g_efuse_modem_pi_enable) && (g_modem_init_completed) )
    //    {
    //        RT_BT_LOG(RED, MDM_LOG_007, 1, val);
    //    }

        // switch page in different ways according to PI or SI //
        if ((addr == 0) && g_efuse_modem_pi_enable && g_efuse_modem_pi_addr0_convert)
        {
            lc_write_modem_pi_page(val);
            return;
        }

        if (!(g_efuse_modem_reg_rw_fast_mode && g_modem_pi_si_sel_init_completed))
        {
            lc_write_modem_pi_si_sel(g_efuse_modem_pi_enable);
        }
#endif
#endif

        lc_write_radio_reg(0x40 | addr, val);
    }
    else if (type == TYPE_RF)
    {
#if !defined(_USE_RLE0380_RF_) && !defined(_USE_RLE0546_RF_)
        UINT32 rvalue = 0;
        UINT8 addr_tmp = addr;

        if (addr_tmp > 0x3F) //For 0379, the extended register 0x40,0x41,0x42
        {
            /* select RF page */
            rvalue = lc_read_radio_reg(0x71);
            lc_write_radio_reg(0x71, rvalue | BIT12);
            addr &= 0x3f;
        }
#endif
        lc_write_radio_reg(addr, val);

#if !defined(_USE_RLE0380_RF_) && !defined(_USE_RLE0546_RF_)
        if (addr_tmp > 0x3F)
        {
            /* select original page */
            lc_write_radio_reg(0x71, rvalue);
        }
#endif
    }
}


/* yilinli, 2014/02/07, to facillate RFC/MODEM retention restore
 * RF_IO [31:0]: RFC Direct Read register
 * initial value: 0
 * REG II.7 (Offset 0x348h) RF_IO Register Definition
 * Bit	R/W	Symbol	Description
 * 31_26		reserved
 * 25	R/W	RFC_PI_SEL	1: pi mux to cpu.
 * 0:pi mux to modem. (default)
 * 24	W	RF_IOW	Assert as write RFC register is requested
 * 23_16	R/W	BT_IOAD_RFC	RFC address.
 * 15_0	R/W	BT_IOQ_RFC	Read:The read data of RFC
 * Write: Write data to RFC. Translate to PI interface as
 *
 * BTPHY_FPI_IF [31:0]: BT MODEM fast interface
 * initial value: 0
 * REG II.10 (Offset 0x354h) Fast MODEM PI Register Definition
 * Bit	R/W	Symbol	Description
 * 31_29		reserved
 * 28	W	Pi_wr	1; write modem register
 * 0: read modem register
 * 27_16	R/W	Pi_addr	PI address
 * b27~22: page
 * 15_0	R/W	Pi_wdata/pi_rdata	PI write data  */
void __attribute__ ((noinline)) write_table_to_io(UINT32 addr, UINT32* table, UINT32 table_size)
{
    UINT32 ii;
    for (ii = 0; ii < table_size; ii++)
    {
        WR_REG_MMIO(UINT32, addr, table[ii]);
    }
}



#ifdef _YL_NEW_MODEM_SRAM_DEBUG
#ifdef _DAPE_MODEM_SRAM_DBG_VER2
void rtl8821_btrf_modem_sram_debug_init(MODEM_SRAM_DEBUG_ELEMENT_S_TYPE* modem_sram_debug_array, UINT8 mode)
#else
void rtl8821_btrf_modem_sram_debug_init(MODEM_SRAM_DEBUG_ELEMENT_S_TYPE* modem_sram_debug_array)
#endif
{
    if (modem_sram_debug_array == NULL)
    {
        return;
    }

    MODEM_REG_S_TYPE modem_reg;
    UINT32 addr_temp;
    addr_temp = (UINT32)(modem_sram_debug_array);
    addr_temp &= 0xFFFF;
    addr_temp = (addr_temp>>3);

    g_modem_sram_debug_en = 1;
    g_modem_sram_debug_log_en = 1;
    g_modem_sram_debug_captured_flag = 0;
    g_modem_sram_debug_xdh5_trig_en = 1;
    g_modem_sram_debug_xdh5_trig_crc_ok = 0;
    g_modem_sram_debug_le_trig_en = 1;
    g_modem_sram_debug_le_trig_crc_ok = 0;
    g_modem_sram_debug_xdh5_xdh3_3dh1_error_log_en = 1;
#ifdef _DAPE_MODEM_SRAM_DBG_VER2
    g_modem_sram_debug_log_count = 0;
#endif

    VENDOR_WRITE(0x30, VENDOR_READ(0x30) | BIT31);

    /* switch to modem page 2 */
    rtk_write_modem_radio_reg(TRANS_MODEM_REG(0x00), TYPE_MODEM, 0x02);

    modem_reg.d16 = rtk_read_modem_radio_reg(TRANS_MODEM_REG(0x52), TYPE_MODEM);
    modem_reg.p2reg52.reg_sram_start_addr = addr_temp&0x1FFF;
    rtk_write_modem_radio_reg(TRANS_MODEM_REG(0x52), TYPE_MODEM, modem_reg.d16);

    modem_reg.d16 = rtk_read_modem_radio_reg(TRANS_MODEM_REG(0x54), TYPE_MODEM);
    modem_reg.p2reg54.reg_sram_end_addr = (addr_temp+(g_modem_sram_dbg_entry_num-2))&0x1FFF;
    modem_reg.p2reg54.reg_sram_adc_rate = MODEM_SRAM_DEBUG_ADC_RATE_SEL;
#if _DAPE_MODEM_SRAM_DBG_VER2
    if (g_modem_sram_debug_modem_test)
    {
        modem_reg.p2reg54.reg_sram_debug_pattern = 1;
    }
    else
    {
        modem_reg.p2reg54.reg_sram_debug_pattern = 0;
    }
#else
#if 0 // TODO: FOR TEST ONLY
    modem_reg.p2reg54.reg_sram_debug_pattern = 1;
#endif
#endif
    rtk_write_modem_radio_reg(TRANS_MODEM_REG(0x54), TYPE_MODEM, modem_reg.d16);
    
#ifdef _YL_EXTEND_MODEM_SRAM_ADDR_14BIT_
    modem_reg.d16 = rtk_read_modem_radio_reg(TRANS_MODEM_REG(0x5e), TYPE_MODEM);
    modem_reg.p2reg5e.reg_sram_start_addr_msb = (addr_temp>>13)&BIT0;
    modem_reg.p2reg5e.reg_sram_end_addr_msb = ((addr_temp+(g_modem_sram_dbg_entry_num-2))>>13)&BIT0;
    rtk_write_modem_radio_reg(TRANS_MODEM_REG(0x5e), TYPE_MODEM, modem_reg.d16);    
#endif    
    RT_BT_LOG(YELLOW, MDM_LOG_001, 3,
                                    (UINT32)modem_sram_debug_array,
                                    rtk_read_modem_radio_reg(TRANS_MODEM_REG(0x52), TYPE_MODEM),
                                    rtk_read_modem_radio_reg(TRANS_MODEM_REG(0x54), TYPE_MODEM) );


#if 0 // TODO: to be removed
    modem_reg.d16 = rtk_read_modem_radio_reg(TRANS_MODEM_REG(0x56), TYPE_MODEM);
    modem_reg.p2reg56.reg_sram_pkt_num = 3;
    rtk_write_modem_radio_reg(TRANS_MODEM_REG(0x56), TYPE_MODEM, modem_reg.d16);
#endif

    /* switch to modem page 0 */
    rtk_write_modem_radio_reg(TRANS_MODEM_REG(0x00), TYPE_MODEM, 0x00);
#ifdef _DAPE_MODEM_SRAM_DBG_VER2
    rtl8821_btrf_modem_sram_debug_set_mode(mode);
#else
    rtl8821_btrf_modem_sram_debug_set_mode(MODEM_SRAM_DEBUG_MODE_SEL);
#endif
    rtl8821_btrf_modem_sram_debug_set_size(MODEM_SRAM_DEBUG_PKT_SIZE_1PKT);
    rtl8821_btrf_modem_sram_debug_set_en(1);
    rtl8821_btrf_modem_sram_debug_set_rst(1);
    modem_reg.d16 = rtk_read_modem_radio_reg(TRANS_MODEM_REG(0x08), TYPE_MODEM);
    modem_reg.reg08.reg_sram_debug_filter_en = 1;
    rtk_write_modem_radio_reg(TRANS_MODEM_REG(0x08), TYPE_MODEM, modem_reg.d16);
#ifdef _DAPE_MODEM_SRAM_DBG_VER2
    RT_BT_LOG(YELLOW, YL_DBG_HEX_2, 2, rtk_read_modem_radio_reg(TRANS_MODEM_REG(0x08), TYPE_MODEM), mode);
#else
    RT_BT_LOG(YELLOW, DMA_UART_003_DBG_HEX, 1, rtk_read_modem_radio_reg(TRANS_MODEM_REG(0x08), TYPE_MODEM));
#endif
}

/* Note: this function should be called when modem page is 0 */
void rtl8821_btrf_modem_sram_debug_set_mode(UINT8 value)
{
    MODEM_REG_S_TYPE modem_reg;
#ifdef _NEW_MODEM_PI_ACCESS_  // to avoid run-time access page-conflicts //
    modem_reg.d16 = RTK_READ_MODEM_REG_PI(MODEM_PI_PAGE_0, TRANS_MODEM_REG(0x08));
    modem_reg.reg08.reg_sram_debug_mode = value;
    RTK_WRITE_MODEM_REG_PI(MODEM_PI_PAGE_0, TRANS_MODEM_REG(0x08), modem_reg.d16);
#else
    modem_reg.d16 = rtk_read_modem_radio_reg(TRANS_MODEM_REG(0x08), TYPE_MODEM);
    modem_reg.reg08.reg_sram_debug_mode = value;
    rtk_write_modem_radio_reg(TRANS_MODEM_REG(0x08), TYPE_MODEM, modem_reg.d16);
#endif
}
void rtl8821_btrf_modem_sram_debug_set_size(UINT8 value)
{
    MODEM_REG_S_TYPE modem_reg;
#ifdef _NEW_MODEM_PI_ACCESS_  // to avoid run-time access page-conflicts //
    modem_reg.d16 = RTK_READ_MODEM_REG_PI(MODEM_PI_PAGE_0, TRANS_MODEM_REG(0x08));
    modem_reg.reg08.reg_sram_pktg_size = value;
    RTK_WRITE_MODEM_REG_PI(MODEM_PI_PAGE_0, TRANS_MODEM_REG(0x08), modem_reg.d16);
#else
    modem_reg.d16 = rtk_read_modem_radio_reg(TRANS_MODEM_REG(0x08), TYPE_MODEM);
    modem_reg.reg08.reg_sram_pktg_size = value;
    rtk_write_modem_radio_reg(TRANS_MODEM_REG(0x08), TYPE_MODEM, modem_reg.d16);
#endif
}

void rtl8821_btrf_modem_sram_debug_set_en(UINT8 value)
{
    MODEM_REG_S_TYPE modem_reg;
#ifdef _NEW_MODEM_PI_ACCESS_  // to avoid run-time access page-conflicts //
    modem_reg.d16 = RTK_READ_MODEM_REG_PI(MODEM_PI_PAGE_0, TRANS_MODEM_REG(0x08));
    modem_reg.reg08.reg_sram_debug_en = value;
    RTK_WRITE_MODEM_REG_PI(MODEM_PI_PAGE_0, TRANS_MODEM_REG(0x08), modem_reg.d16);
#else
    modem_reg.d16 = rtk_read_modem_radio_reg(TRANS_MODEM_REG(0x08), TYPE_MODEM);
    modem_reg.reg08.reg_sram_debug_en = value;
    rtk_write_modem_radio_reg(TRANS_MODEM_REG(0x08), TYPE_MODEM, modem_reg.d16);
#endif
}
/* Note: this function should be called when modem page is 0 */
UINT8 rtl8821_btrf_modem_sram_debug_get_en(void)
{
    MODEM_REG_S_TYPE modem_reg;
#ifdef _NEW_MODEM_PI_ACCESS_  // to avoid run-time access page-conflicts //
    modem_reg.d16 = RTK_READ_MODEM_REG_PI(MODEM_PI_PAGE_0, TRANS_MODEM_REG(0x08));
    return (UINT8)modem_reg.reg08.reg_sram_debug_en;
#else
    modem_reg.d16 = rtk_read_modem_radio_reg(TRANS_MODEM_REG(0x08), TYPE_MODEM);
    return (UINT8)modem_reg.reg08.reg_sram_debug_en;
#endif
}
/* Note: this function should be called when modem page is 0 */
void rtl8821_btrf_modem_sram_debug_set_rst(UINT8 value)
{
    MODEM_REG_S_TYPE modem_reg;
#ifdef _NEW_MODEM_PI_ACCESS_  // to avoid run-time access page-conflicts //
    modem_reg.d16 = RTK_READ_MODEM_REG_PI(MODEM_PI_PAGE_0, TRANS_MODEM_REG(0x08));
    modem_reg.reg08.reg_sram_soft_rst = value;
    RTK_WRITE_MODEM_REG_PI(MODEM_PI_PAGE_0, TRANS_MODEM_REG(0x08), modem_reg.d16);
#else
    modem_reg.d16 = rtk_read_modem_radio_reg(TRANS_MODEM_REG(0x08), TYPE_MODEM);
    modem_reg.reg08.reg_sram_soft_rst = value;
    rtk_write_modem_radio_reg(TRANS_MODEM_REG(0x08), TYPE_MODEM, modem_reg.d16);
#endif
}

void rtl8821_btrf_modem_sram_debug_test_log(MODEM_SRAM_DEBUG_ELEMENT_S_TYPE* modem_sram_debug_array)
{
    UINT16 ii,jj,iijj;
//    RT_BT_LOG(YELLOW, DMA_UART_003_DBG_HEX, 1, VENDOR_READ(0x30));
//    RT_BT_LOG(YELLOW, DMA_UART_003_DBG_HEX, 1, rtk_read_modem_radio_reg(TRANS_MODEM_REG(0x08), TYPE_MODEM));
    UINT16 LOG_NUM_1TIME;

    if (modem_sram_debug_array == NULL)
    {
        return;
    }
#ifdef _DAPE_MODEM_SRAM_DBG_VER2
    LOG_NUM_1TIME = 32;
#else
    LOG_NUM_1TIME = 64;
#endif

#if 1   /* printf RAW Data, to be parsed in MATLAB */
//    if (1)
    if (g_modem_sram_debug_en && g_modem_sram_debug_log_en)
    {
        if (g_modem_sram_debug_captured_flag)
        {
            jj = (g_modem_sram_debug_log_count*LOG_NUM_1TIME)%g_modem_sram_dbg_entry_num;
            for (ii = 0; ii<LOG_NUM_1TIME; ii=ii+8)
            {
                iijj=ii+jj;
                RT_BT_LOG(GREEN, MDM_LOG_006, 18,
                    g_modem_sram_debug_captured_flag,
                    iijj,
                    modem_sram_debug_array[iijj].d32_array[1], modem_sram_debug_array[iijj].d32_array[0],
                    modem_sram_debug_array[iijj+1].d32_array[1], modem_sram_debug_array[iijj+1].d32_array[0],
                    modem_sram_debug_array[iijj+2].d32_array[1], modem_sram_debug_array[iijj+2].d32_array[0],
                    modem_sram_debug_array[iijj+3].d32_array[1], modem_sram_debug_array[iijj+3].d32_array[0],
                    modem_sram_debug_array[iijj+4].d32_array[1], modem_sram_debug_array[iijj+4].d32_array[0],
                    modem_sram_debug_array[iijj+5].d32_array[1], modem_sram_debug_array[iijj+5].d32_array[0],
                    modem_sram_debug_array[iijj+6].d32_array[1], modem_sram_debug_array[iijj+6].d32_array[0],
                    modem_sram_debug_array[iijj+7].d32_array[1], modem_sram_debug_array[iijj+7].d32_array[0]);
#ifdef _DAPE_MODEM_SRAM_DBG_VER2
               {
                    HCI_EVENT_PKT *hci_event_pkt_ptr;
                    OS_SIGNAL signal_send;
            
                    if (OS_ALLOC_BUFFER(tx_table[HCI_EVENT_HANDLER_TASK].pool_handle,
                                       (OS_ADDRESS *)&hci_event_pkt_ptr) == BT_ERROR_OK)
                    {
                        UINT8 i;

                        hci_event_pkt_ptr->event_opcode = HCI_VENDOR_SPECIFIC_EVENT;
                        hci_event_pkt_ptr->param_total_length = 70; // length of event_parameter[xx]
                        hci_event_pkt_ptr->event_parameter[0] = 0x34;
                        hci_event_pkt_ptr->event_parameter[1] = 0x00;
                        hci_event_pkt_ptr->event_parameter[2] = g_modem_sram_debug_captured_flag;
                        hci_event_pkt_ptr->event_parameter[4] = iijj;
                        hci_event_pkt_ptr->event_parameter[5] = iijj >> 8;
                        for (i = 0; i < 8; i++)
                        {
                            hci_event_pkt_ptr->event_parameter[6+ i*8] = modem_sram_debug_array[iijj+i].d32_array[1];
                            hci_event_pkt_ptr->event_parameter[7+ i*8] = modem_sram_debug_array[iijj+i].d32_array[1] >> 8;
                            hci_event_pkt_ptr->event_parameter[8+ i*8] = modem_sram_debug_array[iijj+i].d32_array[1] >> 16;
                            hci_event_pkt_ptr->event_parameter[9+ i*8] = modem_sram_debug_array[iijj+i].d32_array[1] >> 24;
                            hci_event_pkt_ptr->event_parameter[10+ i*8] = modem_sram_debug_array[iijj+i].d32_array[0];
                            hci_event_pkt_ptr->event_parameter[11+ i*8] = modem_sram_debug_array[iijj+i].d32_array[0] >> 8;
                            hci_event_pkt_ptr->event_parameter[12+ i*8] = modem_sram_debug_array[iijj+i].d32_array[0] >> 16;
                            hci_event_pkt_ptr->event_parameter[13+ i*8] = modem_sram_debug_array[iijj+i].d32_array[0] >> 24;
                        }

                        signal_send.type  = HCI_TD_HCI_EVENT_TO_HOST_SIGNAL;
                        signal_send.param = (OS_ADDRESS *)hci_event_pkt_ptr;
            
                        /* Total length of the event packet =
                         * param_length (2nd  byte)+ Event type (1) + length field(1) */
                         signal_send.length = hci_event_pkt_ptr->param_total_length + 2;
                        if (OS_SEND_SIGNAL_TO_TASK(hci_td_task_handle,
                                                   signal_send) != BT_ERROR_OK)
                        {
                            OS_FREE_BUFFER(tx_table[HCI_EVENT_HANDLER_TASK].pool_handle,
                                           hci_event_pkt_ptr);            
                        }
                    }
                }
#endif
            }
            g_modem_sram_debug_log_count++;
        }
    }
#endif
}


#endif


#ifdef _YL_TEST_NEW_MODEM_PI_ACCESS_
UINT16 g_modem_pi_access_test_page = 9999;
void lc_modem_pi_access_test_log(UINT32 dbg_count)
{
    UINT8 pp, aa;
#if 1 // test 0xD4, 0xDA read/write //
    if (g_modem_pi_access_test_page == 9999)
    {
#if 1
        UINT16 read_value1 = BB_read_baseband_register(TRIG_2NDID_REGISTER);
        BB_write_baseband_register(TRIG_2NDID_REGISTER, 0xffff);
        UINT16 read_value2 = BB_read_baseband_register(TRIG_2NDID_REGISTER);
        BB_write_baseband_register(TRIG_2NDID_REGISTER, 0x0000);
        UINT16 read_value3 = BB_read_baseband_register(TRIG_2NDID_REGISTER);
        BB_write_baseband_register(TRIG_2NDID_REGISTER, read_value1);
        RT_BT_LOG(BLUE, YL_DBG_HEX_4, 4, TRIG_2NDID_REGISTER, read_value1, read_value2, read_value3);

        read_value1 = BB_read_baseband_register(EXTENDED_HOST_ACCESS_REGISTER);
        BB_write_baseband_register(EXTENDED_HOST_ACCESS_REGISTER, 0xffff);
        read_value2 = BB_read_baseband_register(EXTENDED_HOST_ACCESS_REGISTER);
        BB_write_baseband_register(EXTENDED_HOST_ACCESS_REGISTER, 0x0000);
        read_value3 = BB_read_baseband_register(EXTENDED_HOST_ACCESS_REGISTER);
        BB_write_baseband_register(EXTENDED_HOST_ACCESS_REGISTER, read_value1);
        RT_BT_LOG(BLUE, YL_DBG_HEX_4, 4, EXTENDED_HOST_ACCESS_REGISTER, read_value1, read_value2, read_value3);

        // Bluewiz PSD Registers //
        read_value1 = BB_read_baseband_register(0x222);
        BB_write_baseband_register(0x222, 0xffff);
        read_value2 = BB_read_baseband_register(0x222);
        BB_write_baseband_register(0x222, 0x0000);
        read_value3 = BB_read_baseband_register(0x222);
        BB_write_baseband_register(0x222, read_value1);
        RT_BT_LOG(BLUE, YL_DBG_HEX_4, 4, 0x222, read_value1, read_value2, read_value3);

        read_value1 = BB_read_baseband_register(0x220);
        BB_write_baseband_register(0x220, 0xffff);
        read_value2 = BB_read_baseband_register(0x220);
        BB_write_baseband_register(0x220, 0x0000);
        read_value3 = BB_read_baseband_register(0x220);
        BB_write_baseband_register(0x220, read_value1);
        RT_BT_LOG(BLUE, YL_DBG_HEX_4, 4, 0x220, read_value1, read_value2, read_value3);

        read_value1 = BB_read_baseband_register(0x224);
        BB_write_baseband_register(0x224, 0xffff);
        read_value2 = BB_read_baseband_register(0x224);
        BB_write_baseband_register(0x224, 0x0000);
        read_value3 = BB_read_baseband_register(0x224);
        BB_write_baseband_register(0x224, read_value1);
        RT_BT_LOG(BLUE, YL_DBG_HEX_4, 4, 0x224, read_value1, read_value2, read_value3);

        // Modem PSD Registers //
        RT_BT_LOG(BLUE, YL_DBG_HEX_1, 1, lc_read_modem_pi_page());
        read_value1 = RTK_READ_MODEM_REG(TRANS_MODEM_REG(0x5A));
        RTK_WRITE_MODEM_REG(TRANS_MODEM_REG(0x5A), 0xffff);
        read_value2 = RTK_READ_MODEM_REG(TRANS_MODEM_REG(0x5A));
        RTK_WRITE_MODEM_REG(TRANS_MODEM_REG(0x5A), 0x0000);
        read_value3 = RTK_READ_MODEM_REG(TRANS_MODEM_REG(0x5A));
        RTK_WRITE_MODEM_REG(TRANS_MODEM_REG(0x5A), read_value1);
        RT_BT_LOG(BLUE, YL_DBG_HEX_4, 4, 0x5A, read_value1, read_value2, read_value3);

        read_value1 = RTK_READ_MODEM_REG(TRANS_MODEM_REG(0x02));
        RTK_WRITE_MODEM_REG(TRANS_MODEM_REG(0x02), 0xffff);
        read_value2 = RTK_READ_MODEM_REG(TRANS_MODEM_REG(0x02));
        RTK_WRITE_MODEM_REG(TRANS_MODEM_REG(0x02), 0x0000);
        read_value3 = RTK_READ_MODEM_REG(TRANS_MODEM_REG(0x02));
        RTK_WRITE_MODEM_REG(TRANS_MODEM_REG(0x02), read_value1);
        RT_BT_LOG(BLUE, YL_DBG_HEX_4, 4, 0x02, read_value1, read_value2, read_value3);

#elif 0
        for (pp = 0; pp<=2; pp++)
        {
            RTK_WRITE_MODEM_REG(0x00, pp);
            for (aa = 1; aa<0x40; aa++)
            {
                RTK_WRITE_MODEM_REG(aa, ~(((pp+aa)<<8) | (pp+aa)));
            }
        }
        RTK_WRITE_MODEM_REG(0x00, 0x00);
#endif


#if 1
        // g_data_uart_settings.wr_bton_en = 1;
        RT_BT_LOG(YELLOW, DMA_UART_128, 1, hci_uart_read_bton_baud_record());
        hci_uart_baud_record_to_bton(hci_uart_read_bton_baud_record() + 0xA6A62323);
        RT_BT_LOG(YELLOW, DMA_UART_128, 1, hci_uart_read_bton_baud_record());
#endif

    }
#endif



    if (dbg_count>=10)
    {
        // enable_uart_suspend();
        // WDG_TIMER_TIMEOUT_SOON;
        pf_delay(1000);
    }


    if (g_modem_pi_access_test_page == 9999)
        g_modem_pi_access_test_page = 0;
    else
        g_modem_pi_access_test_page ++;

#ifdef _NEW_MODEM_PI_ACCESS_BY_VENDOR_REG_
    if ( ((g_modem_pi_access_test_page == 13)) )
#else
    if ( ((g_modem_pi_access_test_page == 4) && (g_efuse_modem_pi_enable == 0)) || ((g_modem_pi_access_test_page == 13)) )
#endif
    {
        // toggle g_efuse_modem_pi_enable //
//        g_efuse_modem_pi_enable = (g_efuse_modem_pi_enable == 0);
        g_modem_pi_access_test_page = 0;
    }
    else if (g_modem_pi_access_test_page == 4)
    {
        g_modem_pi_access_test_page = 8;
    }


    if (g_modem_pi_access_test_page == 3) // Print RF Registers
    {
        UINT16 start_addr = 0;
#ifdef _NEW_MODEM_PI_ACCESS_BY_VENDOR_REG_
        RT_BT_LOG(GREEN, MDM_LOG_010, 2, 1, start_addr);
#else
        RT_BT_LOG(GREEN, MDM_LOG_010, 2, g_efuse_modem_pi_enable, start_addr);
#endif
        for (start_addr = 0; start_addr<0x3F;)
        {

            RT_BT_LOG(GREEN, YL_DBG_HEX_16, 16,
                        RTK_READ_RF_REG(start_addr), RTK_READ_RF_REG(start_addr+1),
                        RTK_READ_RF_REG(start_addr+2), RTK_READ_RF_REG(start_addr+3),
                        RTK_READ_RF_REG(start_addr+4), RTK_READ_RF_REG(start_addr+5),
                        RTK_READ_RF_REG(start_addr+6), RTK_READ_RF_REG(start_addr+7),
                        RTK_READ_RF_REG(start_addr+8), RTK_READ_RF_REG(start_addr+9),
                        RTK_READ_RF_REG(start_addr+10), RTK_READ_RF_REG(start_addr+11),
                        RTK_READ_RF_REG(start_addr+12), RTK_READ_RF_REG(start_addr+13),
                        RTK_READ_RF_REG(start_addr+14), RTK_READ_RF_REG(start_addr+15) );
            start_addr = start_addr + 16;
        }
    }
    else  // Print RF Registers
    {
        RTK_WRITE_MODEM_REG(0, g_modem_pi_access_test_page);
        UINT16 start_addr = 0;
#ifdef _NEW_MODEM_PI_ACCESS_BY_VENDOR_REG_
        RT_BT_LOG(GREEN, MDM_LOG_009, 3, 1, g_modem_pi_access_test_page, start_addr);
#else
        RT_BT_LOG(GREEN, MDM_LOG_009, 3, g_efuse_modem_pi_enable, g_modem_pi_access_test_page, start_addr);
#endif
        for (start_addr = 0; start_addr<0x3F;)
        {
            RT_BT_LOG(GREEN, YL_DBG_HEX_16, 16,
                        RTK_READ_MODEM_REG(start_addr), RTK_READ_MODEM_REG(start_addr+1),
                        RTK_READ_MODEM_REG(start_addr+2), RTK_READ_MODEM_REG(start_addr+3),
                        RTK_READ_MODEM_REG(start_addr+4), RTK_READ_MODEM_REG(start_addr+5),
                        RTK_READ_MODEM_REG(start_addr+6), RTK_READ_MODEM_REG(start_addr+7),
                        RTK_READ_MODEM_REG(start_addr+8), RTK_READ_MODEM_REG(start_addr+9),
                        RTK_READ_MODEM_REG(start_addr+10), RTK_READ_MODEM_REG(start_addr+11),
                        RTK_READ_MODEM_REG(start_addr+12), RTK_READ_MODEM_REG(start_addr+13),
                        RTK_READ_MODEM_REG(start_addr+14), RTK_READ_MODEM_REG(start_addr+15) );
            start_addr = start_addr + 16;
        }
        RTK_WRITE_MODEM_REG(0, 0);

        RT_BT_LOG(YELLOW, YL_DBG_HEX_2, 2, TRIG_2NDID_REGISTER, BB_read_baseband_register(TRIG_2NDID_REGISTER));
        RT_BT_LOG(YELLOW, YL_DBG_HEX_2, 2, EXTENDED_HOST_ACCESS_REGISTER, BB_read_baseband_register(EXTENDED_HOST_ACCESS_REGISTER));

    }
}
#endif

#ifdef _NEW_MODEM_DESIGN_
void pf_sram_modem_access_enable(void)
{
    VENDOR_WRITE(0x30, VENDOR_READ(0x30) | BIT31);
}
void pf_sram_modem_access_disable(void)
{
    VENDOR_WRITE(0x30, VENDOR_READ(0x30) & (~BIT31));
}
#endif

#ifdef _NEW_MODEM_PSD_SCAN_
/* a. setting psd_range0, psd_range1, psd_timeout
 * b. 下set psd_en instruction
 * c. process psd interrupt:
 *      baseband_interrupt_handler()
 *      {
 *         ...
 *         if (0xBE[2] = 1)
 *         {
 *             BB_handle_imode_interrupt();
 *         }
 *         ...
 *      }
 *
 *       BB_handle_imode_interrupt()
 *       {
 *         ...
 *         if (mode_sts_reg&BIT3)
 *         {
 *             lc_handle_psd_en_interrupt();
 *         }
 *       }
 *
 *  1. 如果在psd_en過程中，下clear psd_en，是否會有任何interrupt?
 *      最久是不是一個slot過後就會停下來?
 *       A: 不會有任何 interrupt.  clr_psd 後就你 page_scan inq_scan 一樣會馬上停下來.
 *  2. 如果在psd_en過程中，再set psd_en，會有問題嗎?  是否會整個重頭來過(包含offsetx2)?
 *       A: 不有任何作用.
 *  3. 如果在psd_en過程中，下clear psd_en，再set psd_en，會有問題嗎?  是否會整個重頭來過(包含offsetx2)?
 *       A: 可以, 會重頭來過.
 */

#ifndef _NEW_MODEM_PI_ACCESS_
    !!!! ERROR: Unexpected Compile Flags: _NEW_MODEM_PSD_SCAN_ w/o _NEW_MODEM_PI_ACCESS_ !!!!
#endif


// TODO: "VENDOR_WRITE(0x30, VENDOR_READ(0x30) | BIT31);" is also required
void lc_psd_modem_init(void)
{
    UINT32 addr_temp;
//    UINT32 ii;
#ifdef _ROM_CODE_PATCHED_
    if (rcp_lc_psd_modem_init != NULL)
    {
        if (rcp_lc_psd_modem_init(NULL))
        {
            return;
        }
    }
#endif

    g_efuse_modem_psd_setting_1.d16 = otp_str_data.efuse_modem_psd_setting_1_d16;
    g_efuse_modem_psd_setting_2.d16 = otp_str_data.efuse_modem_psd_setting_2_d16;

#ifdef _YL_TEST_NEW_MODEM_PSD_SCAN_FOR_AFH_
    g_modem_psd_report_array = (MODEM_PSD_REPORT_ELEMENT_S_TYPE*)((void*)g_modem_psd_report_array_physical);
    g_modem_psd_report_entry_num = MODEM_PSD_REPORT_ENTRY_NUM;
#endif
#ifdef _YL_TEST_NEW_MODEM_PSD_SCAN_FOR_AFH_
    otp_str_data.rtk_afh_bt_psd_enable = 1;
    g_efuse_modem_psd_setting_1.rom_code_init_en = 1;
    g_efuse_modem_psd_setting_1.rtk_afh_modem_psd_enable = 1;
    g_efuse_modem_psd_setting_1.afh_map_gen_instant_opt = 1;
    g_efuse_modem_psd_setting_1.psd_timers_start_opt = 3; // ???    [3:2] (1) 0: no start, 1: as old afh, 2: if any active link with AFH, 3: execute anyway
    g_efuse_modem_psd_setting_1.psd_scan_3ch_mode = 0;
    g_efuse_modem_psd_setting_1.rom_code_init_en = 1;
    g_efuse_modem_psd_setting_1.force_channel_classify_on = 0;
    g_efuse_modem_psd_setting_1.lmp_gen_afh_map_use_modem_psd_only = 1;
    g_efuse_modem_psd_setting_1.lmp_gen_afh_map_at_other_case = 0;
#endif


    if (!otp_str_data.rtk_afh_bt_psd_enable)
    {
        return;
    }

    if (!g_efuse_modem_psd_setting_1.rom_code_init_en)
    {
        return;
    }

    pf_sram_modem_access_enable();

    addr_temp = (UINT32)(g_modem_psd_report_array);
    addr_temp &= 0xFFFF;
    addr_temp = (addr_temp>>3);

    MODEM_REG_S_TYPE modem_reg;

//    if (g_efuse_modem_pi_enable)
    if (1)
    {

        RTK_WRITE_MODEM_REG(TRANS_MODEM_REG(0x00), 0x02);
        modem_reg.d16 = RTK_READ_MODEM_REG(TRANS_MODEM_REG(0x56));
        modem_reg.p2reg56.reg_psd_sram_report_addr_base = addr_temp&0x1FFF;
        RTK_WRITE_MODEM_REG(TRANS_MODEM_REG(0x56), modem_reg.d16);

#ifdef _YL_EXTEND_MODEM_SRAM_ADDR_14BIT_
        modem_reg.d16 = RTK_READ_MODEM_REG(TRANS_MODEM_REG(0x50));
        modem_reg.p2reg50.reg_psd_sram_report_addr_base_msb = (addr_temp>>13)&BIT0;
        RTK_WRITE_MODEM_REG(TRANS_MODEM_REG(0x50), modem_reg.d16);
        RTK_WRITE_MODEM_REG(TRANS_MODEM_REG(0x00), 0x0);
#endif

#ifdef _YL_TEST_NEW_MODEM_PSD_SCAN_FOR_AFH_
        RTK_WRITE_MODEM_REG(TRANS_MODEM_REG(0x00), 0x02);
        
        modem_reg.d16 = RTK_READ_MODEM_REG(TRANS_MODEM_REG(0x58));
        modem_reg.p2reg58.reg_psd_iq2pwr_opt = 0;
        RTK_WRITE_MODEM_REG(TRANS_MODEM_REG(0x58), modem_reg.d16);

        modem_reg.d16 = RTK_READ_MODEM_REG(TRANS_MODEM_REG(0x5a));
  #if (!defined(_IS_ASIC_)) && defined(_FPGA_WITH_RLE0379_RFE_)
        modem_reg.p2reg5a.reg_btm_if_freq_psd = MODEM_PSD_IF_2P5;
  #elif (!defined(_IS_ASIC_)) && defined(_FPGA_WITH_RLE0546_RFE_)
        modem_reg.p2reg5a.reg_btm_if_freq_psd = MODEM_PSD_IF_1P4;
  #elif (!defined(_IS_ASIC_)) && defined(_FPGA_WITH_RLE0608C_RFE_)
        modem_reg.p2reg5a.reg_btm_if_freq_psd = MODEM_PSD_IF_1P4;
  #elif (!defined(_IS_ASIC_)) && defined(_FPGA_WITH_RLE0550_RFE_)
        modem_reg.p2reg5a.reg_btm_if_freq_psd = MODEM_PSD_IF_1P4;
  #else
        !!! UNEXPECTED !!!
  #endif
        RTK_WRITE_MODEM_REG(TRANS_MODEM_REG(0x5a), modem_reg.d16);

        RTK_WRITE_MODEM_REG(TRANS_MODEM_REG(0x00), 0x00);


        lc_psd_modem_set_agc(3, 3, 0, 15);

#endif
    }

#ifndef _NEW_MODEM_PI_ACCESS_BY_VENDOR_REG_
    if (!g_efuse_modem_pi_enable)
    {
        RT_BT_LOG(RED, MDM_LOG_017, 0, 0);
    }
#endif

}


// Note: This functi may be executed in run-time, Modem PI Access is MUST!!! //
void lc_psd_modem_set_agc(UINT8 psd_cal_dly, UINT8 psd_agc_time, UINT8 psd_fix_gain, UINT8 psd_ini_gain_mp)
{
    MODEM_REG_S_TYPE modem_reg;

    RTK_WRITE_MODEM_REG(TRANS_MODEM_REG(0x00), 0x02);

    modem_reg.d16 = RTK_READ_MODEM_REG(TRANS_MODEM_REG(0x56));
    modem_reg.p2reg56.reg_psd_agc_time = psd_agc_time;
    RTK_WRITE_MODEM_REG(TRANS_MODEM_REG(0x56), modem_reg.d16);

    modem_reg.d16 = RTK_READ_MODEM_REG(TRANS_MODEM_REG(0x58));
    modem_reg.p2reg58.reg_psd_cal_dly = psd_cal_dly;
    modem_reg.p2reg58.reg_psd_fix_gain = psd_fix_gain;
    RTK_WRITE_MODEM_REG(TRANS_MODEM_REG(0x58), modem_reg.d16);
    RTK_WRITE_MODEM_REG(TRANS_MODEM_REG(0x00), 0x00);

#ifdef _YL_NEW_PSD_REG_AFTER_8703B_
    RTK_WRITE_MODEM_REG(TRANS_MODEM_REG(0x00), 0x03);
    modem_reg.d16 = RTK_READ_MODEM_REG(TRANS_MODEM_REG(0x06));
    modem_reg.p3reg06.reg_psd_ini_gain_mp = psd_ini_gain_mp;
    RTK_WRITE_MODEM_REG(TRANS_MODEM_REG(0x06), modem_reg.d16);
    RTK_WRITE_MODEM_REG(TRANS_MODEM_REG(0x00), 0x00);
#else
    RTK_WRITE_MODEM_REG(TRANS_MODEM_REG(0x00), 0x02);
    modem_reg.d16 = RTK_READ_MODEM_REG(TRANS_MODEM_REG(0x5e));
    modem_reg.p2reg5e.reg_psd_ini_gain_mp = psd_ini_gain_mp;
    RTK_WRITE_MODEM_REG(TRANS_MODEM_REG(0x5e), modem_reg.d16);
    RTK_WRITE_MODEM_REG(TRANS_MODEM_REG(0x00), 0x00);
#endif
}

void lc_psd_bluewiz_set_parameters(UINT8 psd_ch_start, UINT8 psd_ch_step, UINT8 psd_ch_end, UINT8 psd_mode, UINT16 psd_timeout_us)
{
    UINT16 read_value;
    read_value = BB_read_baseband_register(BB_PSD_END_START);
    read_value &= 0x8080;
    read_value = read_value | psd_ch_start | (psd_ch_end<<8);
    BB_write_baseband_register(BB_PSD_END_START, read_value);

    read_value = BB_read_baseband_register(BB_PSD_STEP_MODE);
    read_value &= 0xfe80;
    read_value = read_value | psd_ch_step | ((psd_mode&0x01)<<8); // mode = 0: half-slot mode, mode = 1: full-slot mode
    BB_write_baseband_register(BB_PSD_STEP_MODE, read_value);

    read_value = BB_read_baseband_register(BB_PSD_TIMEOUT);
    read_value &= 0xfc00;
    read_value = read_value | psd_timeout_us;
    BB_write_baseband_register(BB_PSD_TIMEOUT, read_value);
}

void lc_psd_set_psd_en(UINT8 psd_en)
{
    UINT16 inst_code;
    if (psd_en)
    {
        inst_code = BB_SET_PSD_EN;
    }
    else
    {
        inst_code = BB_CLR_PSD_EN;
    }
    BB_write_baseband_register(INSTRUCTION_REGISTER, inst_code);
}


INT8 log10x10_table[16] = {-30, -6, -3, -1, 0, 1, 2, 2, 3, 4, 4, 4, 5, 5, 5, 6};
INT8 log10x10(UINT16 xin)
{
    INT8 xout = 0;
    UINT8 ii;
    UINT32 xinx4;
    xinx4 = ((UINT32)xin)<<2;
    for (ii=16; ii>=2; ii-=2)
    {
        if (xinx4>>ii)
        {
            xout = (ii-2)*3;
            xinx4 = xinx4>>(ii-2);
            break;
        }
    }
    xout += log10x10_table[MIN((UINT8)xinx4,15)];
    return xout;
}

UINT8 modem_psd_psdavg2dbm(UINT16 psd_avg, INT8 mp_gain_idx)
{
    INT32 output_temp;
    output_temp = ( (INT32)log10x10(psd_avg) +
                    (INT32)otp_str_data.efuse_modem_rssi0_pin_dBm +
                    (INT32)(mp_gain_idx<<1) +
                    (INT32)g_modem_psd_offset );
    output_temp = MIN(MAX(output_temp, 0), 255);
    return (UINT8)output_temp;
}

#endif



#ifdef _NEW_8821A_RX_AGC_API_
/********************************************************************************
 * In RTL8821A:
 *    Old AGC table and New AGC tables are separate ones.
 *    In other words, there are totally 5 physical tables in the BT Modem
 *    One can seperately write the 5 tables, and then set r_aagc_8723A_en
 *    to select New/Old AGC (and the corresponding tables ...)
 * Example:
 *    EFuse_PHYInit_Gain0Stop = 0x2034;
 *    EFuse_PHYInit_Gain0Start = 0x361E;
 *    EFuse_PHYInit_Gain1Start = 0x7517;
 *    EFuse_PHYInit_Gain2Start = 0x9510;
 *    EFuse_PHYInit_Gain3Start = 0xB60C;
 *    EFuse_PHYInit_Gain4Start = 0xDC00;
 *    EFuse_PHYInit_Gain5Start = EFuse_PHYInit_Gain6Start = EFuse_PHYInit_Gain7Start = 0;
 *
 *     [INDEX] [RX GAIN]
 *     0x3F:   0x20
 *       |       |
 *     0x34:   0x20
 *     0x33:   0x21
 *       |       |
 *     0x1E:   0x36
 *     0x1D:   0x6F
 *       |       |
 *     0x17:   0x75
 *     0x16:   0x8F
 *       |       |
 *     0x10:   0x95
 *     0x0F:   0xB3
 *       |       |
 *     0x0C:   0xB6
 *     0x0B:   0xD1
 *       |       |
 *     0x00:   0xDC
 *******************************************************************************/
void rtl8723_btrf_RxAGCTableInit(void)
{

    UINT16 Gain_LNA = 0;
    UINT16 Gain_PGA = 0;
    UINT16 Gain_All;
#ifdef _NEW_MODEM_DESIGN_PHY_SETTING_
    UINT16 StartGainIndex_gain[8];
    UINT16 StopGainIndex_gain0;
#else
    UINT8 StartIndex_Hgain;
    UINT8 StartIndex_Mgain;
    UINT8 StartIndex_Lgain;
    UINT8 StartIndex_Ugain;
    UINT8 StopIndex_Ugain;
#endif
    UINT8 n; /* gain table index */
    UINT8 GainStep = 0;

#ifdef _NEW_MODEM_DESIGN_PHY_SETTING_

    UINT8 lna_gain_idx_now;
    UINT8 k; /* LNA index */
    UINT16 Gain_All_shift8;
    MODEM_REG_S_TYPE modem_reg_3c;
    MODEM_AGC_TABLE_S_TYPE modem_agc_table_3e;
    UINT16 r_aagc_8723A_en_record;

    UINT8 trans_modem_reg_0x3c = TRANS_MODEM_REG(0x3c);
    UINT8 trans_modem_reg_0x3e = TRANS_MODEM_REG(0x3e);

    EFUSE_MODEM_SETTING_1_S efuse_modem_setting_1;
    *(UINT8*)&efuse_modem_setting_1 = otp_str_data.efuse_modem_setting_1_d8;


    /***** switch modem to page 0 *****/
    RTK_WRITE_MODEM_REG(TRANS_MODEM_REG(0x00), 0);

    /***** record the r_aagc_8723A_en *****/
    /* note: r_aagc_8723A_en should be restored after filling the tables */
    modem_reg_3c.d16 = RTK_READ_MODEM_REG(trans_modem_reg_0x3c);
    r_aagc_8723A_en_record = modem_reg_3c.reg3c.r_aagc_8723A_en;

    /***** write tables *****/

    /* Old RX AGC Table 0 and New RX AGC Table 1 Initialization */
    /* Note: gain[7] corresponds to highest LNA gain */
    StartGainIndex_gain[7] =  otp_str_data.EFuse_PHYInit_Gain7Start;
    StartGainIndex_gain[6] =  otp_str_data.EFuse_PHYInit_Gain6Start;
    StartGainIndex_gain[5] =  otp_str_data.EFuse_PHYInit_Gain5Start;
    StartGainIndex_gain[4] =  otp_str_data.EFuse_PHYInit_Gain4Start;
    StartGainIndex_gain[3] =  otp_str_data.EFuse_PHYInit_Gain3Start;
    StartGainIndex_gain[2] =  otp_str_data.EFuse_PHYInit_Gain2Start;
    StartGainIndex_gain[1] =  otp_str_data.EFuse_PHYInit_Gain1Start;
    StartGainIndex_gain[0] =  otp_str_data.EFuse_PHYInit_Gain0Start;
    StopGainIndex_gain0 =  otp_str_data.EFuse_PHYInit_Gain0Stop;

    for (n = 0; n < 64; n++)
    {
        lna_gain_idx_now = 0;
        for (k = 7; k > 0; k--)
        {
            if (n < (StartGainIndex_gain[k-1] & 0xff))
            {
                lna_gain_idx_now = k;
                break;
            }
        }
        /* (k == 0) means n is at the last segment of the table */

        if ( (k != 0) || (n < (StopGainIndex_gain0 & 0xff)) )
        {
            if (n == (StartGainIndex_gain[lna_gain_idx_now] & 0xff))
            {
                Gain_LNA = StartGainIndex_gain[lna_gain_idx_now] & 0xe000;
                Gain_PGA = StartGainIndex_gain[lna_gain_idx_now] & 0x1f00;
#if 0
            #ifdef _RTL8821A_RF_RXAGC_FORMAT_ // RTL8821A, RTL8723B, BT-ONLY
                Gain_LNA = StartGainIndex_gain[lna_gain_idx_now] & 0xe000;
                Gain_PGA = StartGainIndex_gain[lna_gain_idx_now] & 0x1f00;
            #else
              #ifdef _RTL8723A_RF_RXAGC_FORMAT_ // RLE0379, RLE0380, RTL8723A
                Gain_LNA = StartGainIndex_gain[lna_gain_idx_now] & 0x6000;
                Gain_PGA = StartGainIndex_gain[lna_gain_idx_now] & 0x1f00;
              #else
                ** yilinli: unexpected compiler options **
                ** yilinli: need to be customized for each RF **
              #endif
            #endif
#endif
                GainStep = 0;
            }
            else
            {
                GainStep++;
            }
        }
        else
        {
            if (n == (StopGainIndex_gain0 & 0xff))
            {
                Gain_LNA = StopGainIndex_gain0 & 0xe000;
                Gain_PGA = StopGainIndex_gain0 & 0x1f00;
#if 0
            #ifdef _RTL8821A_RF_RXAGC_FORMAT_ // RTL8821A, RTL8723B, BT-ONLY
                Gain_LNA = StopGainIndex_gain0 & 0xe000;
                Gain_PGA = StopGainIndex_gain0 & 0x1f00;
            #else
              #ifdef _RTL8723A_RF_RXAGC_FORMAT_ // RLE0379, RLE0380, RTL8723A
                Gain_LNA = StopGainIndex_gain0 & 0x6000;
                Gain_PGA = StopGainIndex_gain0 & 0x1f00;
              #else
                ** yilinli: unexpected compiler options **
                ** yilinli: need to be customized for each RF **
              #endif
            #endif
#endif
            }
            GainStep = 0;
        }

        Gain_All = Gain_LNA;
        if (Gain_PGA > (GainStep << 8))
        {
           Gain_All |= Gain_PGA - (GainStep << 8);
        }
        Gain_All_shift8 = Gain_All >> 8;



        /* write to modem */
        EFUSE_MODEM_SETTING_1_S efuse_modem_setting_1;
        *(UINT8*)&efuse_modem_setting_1 = otp_str_data.efuse_modem_setting_1_d8;

        modem_agc_table_3e.d16 = 0;
        modem_agc_table_3e.addr = n;
        modem_agc_table_3e.value = Gain_All_shift8;
        modem_reg_3c.d16 = RTK_READ_MODEM_REG(trans_modem_reg_0x3c);
        if (!efuse_modem_setting_1.not_init_old_agc_table)
        {
            modem_reg_3c.reg3c.r_aagc_8723A_en = 1;
            modem_reg_3c.reg3c.r_table_sel = 0;
            RTK_WRITE_MODEM_REG(trans_modem_reg_0x3c, modem_reg_3c.d16);
#ifdef _NEW_MODEM_PI_ACCESS_
  #ifdef _NEW_MODEM_PI_ACCESS_BY_VENDOR_REG_
            if (efuse_modem_setting_1.modem_agc_init_by_pi)
  #else
            if (g_efuse_modem_pi_enable)
  #endif
            {
                RTK_WRITE_MODEM_REG_PI(MODEM_PI_PAGE_OLD_AGC_TAB0, n, Gain_All_shift8);
            }
            else
#endif
            {
                RTK_WRITE_MODEM_REG(trans_modem_reg_0x3e, modem_agc_table_3e.d16);
            }
        }
        if (!efuse_modem_setting_1.not_init_new_agc_table)
        {
            modem_reg_3c.reg3c.r_aagc_8723A_en = 0;
            modem_reg_3c.reg3c.r_table_sel = 1;
            RTK_WRITE_MODEM_REG(trans_modem_reg_0x3c, modem_reg_3c.d16);
#ifdef _NEW_MODEM_PI_ACCESS_
  #ifdef _NEW_MODEM_PI_ACCESS_BY_VENDOR_REG_
            if (efuse_modem_setting_1.modem_agc_init_by_pi)
  #else
            if (g_efuse_modem_pi_enable)
  #endif
            {
                RTK_WRITE_MODEM_REG_PI(MODEM_PI_PAGE_NEW_AGC_TAB1, n, Gain_All_shift8);
            }
            else
#endif
            {
                RTK_WRITE_MODEM_REG(trans_modem_reg_0x3e, modem_agc_table_3e.d16);
            }
        }
        RF0380_LOG(GRAY, MSG_RT8723_NEW_RF_RXAGC_2, 4, n, modem_agc_table_3e.value, lna_gain_idx_now, modem_agc_table_3e.d16);
//        RTK_WRITE_MODEM_REG(TRANS_MODEM_REG(0x3E), Gain_All + n);
//        RF0380_LOG(GRAY, MSG_RT8723_NEW_RF_RXAGC, 2, n, Gain_All + n);
    }

    /* New RX AGC Table 0, 2, 3 Initialization */
    if (!efuse_modem_setting_1.not_init_new_agc_table)
    {
        /* New RX AGC Table 0 Initialization */
        modem_reg_3c.d16 = RTK_READ_MODEM_REG(trans_modem_reg_0x3c);
        modem_reg_3c.reg3c.r_table_sel = 0;
        RTK_WRITE_MODEM_REG(trans_modem_reg_0x3c, modem_reg_3c.d16);
        for (n = 0; n<NEW_MODEM_NEW_AGC_TABLE0_SIZE; n++)
        {
            modem_agc_table_3e.addr = n;
            modem_agc_table_3e.value = g_new_modem_new_agc_table0[n];
#ifdef _NEW_MODEM_PI_ACCESS_
  #ifdef _NEW_MODEM_PI_ACCESS_BY_VENDOR_REG_
            if (efuse_modem_setting_1.modem_agc_init_by_pi)
  #else
            if (g_efuse_modem_pi_enable)
  #endif
            {
                RTK_WRITE_MODEM_REG_PI(MODEM_PI_PAGE_NEW_AGC_TAB0, n, g_new_modem_new_agc_table0[n]);
            }
            else
#endif
            {
                RTK_WRITE_MODEM_REG(trans_modem_reg_0x3e, modem_agc_table_3e.d16);
            }
            RF0380_LOG(GRAY, MSG_RT8723_NEW_RF_RXAGC_2, 4, n, modem_agc_table_3e.value, lna_gain_idx_now, modem_agc_table_3e.d16);
        }

        /* New RX AGC Table 2 Initialization */
        modem_reg_3c.reg3c.r_table_sel = 2;
        RTK_WRITE_MODEM_REG(trans_modem_reg_0x3c, modem_reg_3c.d16);
        for (n = 0; n<NEW_MODEM_NEW_AGC_TABLE2_SIZE; n++)
        {
            modem_agc_table_3e.addr = n;
            modem_agc_table_3e.value = g_new_modem_new_agc_table2[n];
#ifdef _NEW_MODEM_PI_ACCESS_
  #ifdef _NEW_MODEM_PI_ACCESS_BY_VENDOR_REG_
            if (efuse_modem_setting_1.modem_agc_init_by_pi)
  #else
            if (g_efuse_modem_pi_enable)
  #endif
            {
                RTK_WRITE_MODEM_REG_PI(MODEM_PI_PAGE_NEW_AGC_TAB2, n, g_new_modem_new_agc_table2[n]);
            }
            else
#endif
            {
                RTK_WRITE_MODEM_REG(trans_modem_reg_0x3e, modem_agc_table_3e.d16);
            }
            RF0380_LOG(GRAY, MSG_RT8723_NEW_RF_RXAGC_2, 4, n, modem_agc_table_3e.value, lna_gain_idx_now, modem_agc_table_3e.d16);
        }

        /* New RX AGC Table 3 Initialization */
        modem_reg_3c.reg3c.r_table_sel = 3;
        RTK_WRITE_MODEM_REG(trans_modem_reg_0x3c, modem_reg_3c.d16);
        for (n = 0; n<NEW_MODEM_NEW_AGC_TABLE3_SIZE; n++)
        {
            modem_agc_table_3e.addr = n;
            modem_agc_table_3e.value = g_new_modem_new_agc_table3[n];
#ifdef _NEW_MODEM_PI_ACCESS_
  #ifdef _NEW_MODEM_PI_ACCESS_BY_VENDOR_REG_
            if (efuse_modem_setting_1.modem_agc_init_by_pi)
  #else
            if (g_efuse_modem_pi_enable)
  #endif
            {
                RTK_WRITE_MODEM_REG_PI(MODEM_PI_PAGE_NEW_AGC_TAB3, n, g_new_modem_new_agc_table3[n]);
            }
            else
#endif
            {
                RTK_WRITE_MODEM_REG(trans_modem_reg_0x3e, modem_agc_table_3e.d16);
            }
            RF0380_LOG(GRAY, MSG_RT8723_NEW_RF_RXAGC_2, 4, n, modem_agc_table_3e.value, lna_gain_idx_now, modem_agc_table_3e.d16);
        }

    }

    /***** restore the r_aagc_8723A_en *****/
    modem_reg_3c.d16 = RTK_READ_MODEM_REG(trans_modem_reg_0x3c);
    modem_reg_3c.reg3c.r_aagc_8723A_en = r_aagc_8723A_en_record;
    RTK_WRITE_MODEM_REG(trans_modem_reg_0x3c, modem_reg_3c.d16);

#else
    StartIndex_Hgain =  otp_str_data.EFuse_PHYInit_HGainStart & 0xff;
    StartIndex_Mgain =  otp_str_data.EFuse_PHYInit_MGainStart & 0xff;
    StartIndex_Lgain =  otp_str_data.EFuse_PHYInit_LGainStart & 0xff;
    StartIndex_Ugain =  otp_str_data.EFuse_PHYInit_UGainStart & 0xff;
    StopIndex_Ugain =  otp_str_data.EFuse_PHYInit_UGainStop & 0xff;
    for (n = 0; n < 64; n++)
    {
        if (n < StartIndex_Mgain)  //high-gain
        {
            if (n == StartIndex_Hgain)
            {
                Gain_LNA = otp_str_data.EFuse_PHYInit_HGainStart & 0x6000;
                Gain_PGA = otp_str_data.EFuse_PHYInit_HGainStart & 0x1f00;
                GainStep = 0;
            }
            else
            {
                GainStep++;
            }
        }
        else if (n < StartIndex_Lgain)  //middle gain
        {
            if (n == StartIndex_Mgain)
            {
                Gain_LNA = otp_str_data.EFuse_PHYInit_MGainStart & 0x6000;
                Gain_PGA = otp_str_data.EFuse_PHYInit_MGainStart & 0x1f00;
                GainStep = 0;
            }
            else
            {
                GainStep++;
            }
        }
        else if (n < StartIndex_Ugain)  //low gain
        {
            if (n == StartIndex_Lgain)
            {
                Gain_LNA = otp_str_data.EFuse_PHYInit_LGainStart & 0x6000;
                Gain_PGA = otp_str_data.EFuse_PHYInit_LGainStart & 0x1f00;
                GainStep = 0;
            }
            else
            {
                GainStep++;
            }
        }
        else if (n < StopIndex_Ugain)  //Ultra-low gain
        {
            if (n == StartIndex_Ugain)
            {
                Gain_LNA = otp_str_data.EFuse_PHYInit_UGainStart & 0x6000;
                Gain_PGA = otp_str_data.EFuse_PHYInit_UGainStart & 0x1f00;
                GainStep = 0;
            }
            else
            {
                GainStep++;
            }
        }
        else      //ultra low gain, redundance
        {
            if (n == StopIndex_Ugain)
            {
              Gain_LNA = otp_str_data.EFuse_PHYInit_UGainStop & 0x6000;
              Gain_PGA = otp_str_data.EFuse_PHYInit_UGainStop & 0x1f00;
            }
            GainStep = 0;
        }

        Gain_All = Gain_LNA;
        if (Gain_PGA > (GainStep << 8))
        {
           Gain_All |= Gain_PGA - (GainStep << 8);
        }

        RTK_WRITE_MODEM_REG(TRANS_MODEM_REG(0x3E), Gain_All + n);
        RF0380_LOG(GRAY, MSG_RT8723_NEW_RF_RXAGC, 2, n, Gain_All + n);
    }
#endif

}
#endif

#ifdef _SUPPORT_LBT_FUNC_FOR_RF_ADAPTIVITY_
UINT8 g_tx_power_over_10dbm_index = 7;
#endif

#ifdef _IS_ASIC_
INT8 g_rtl8723_btrf_txpower_track_n_old = 0;
INT8 g_rtl8723_btrf_cfo_track_n_old = 0;
INT8 g_rtl8723_btrf_rxgain_track_n_old = 0;
UINT8 g_rtl8723_btrf_thermal_old = 0xFF;
#ifndef RTL8723A_B_CUT
INT8 g_rtl8723_btrf_lok_track_n_old = 0;
#endif
TIMER_ID g_rtl8723_btrf_thermal_value_timer = OS_INVALID_HANDLE;

INT8 g_rtl8723_btrf_txpower_track_extra_delta = 0; /* use for new
                                                      tx_table power tracking */
UINT8 g_max_rtl8723rf_tx_power_index = 6;

#ifdef _ROM_CODE_PATCHED_
PF_ROM_CODE_PATCH_VOID rcp_rf_iqk_func = NULL;
#endif

#ifdef _OTP_ADD_SYS_INIT_PARAM_
void sys_init_by_otp(void)
{
    rtl8723_btrf_PHYInit_partial(otp_str_data.sys_init_param, 0, 64);
}
#endif

void rtl8723_btrf_PHYInit(void)
{
#ifdef _OTP_ADD_SYS_INIT_PARAM_
    rtl8723_btrf_PHYInit_partial(otp_str_data.phy_init_param, 0, 192);
#else
    rtl8723_btrf_PHYInit_partial(otp_str_data.phy_init_param, 0, 256);
#endif
}

void rtl8723_btrf_PHYInit_partial(UINT16 *buf, UINT16 start, UINT16 end)
{
    UINT16 RegType;
    UINT16 RegAddr;
    UINT16 RegValue;
#if defined (_NEW_MODEM_PI_ACCESS_) || defined(_NEW_RFC_PI_ACCESS_)
	UINT16 RegPi;
#endif
    UINT16 i;
    UINT8 cur_page = 0xFF;
    UINT8 new_page;
    UINT8 error;

    for (i = start; i < end; i += 2)
    {
        /* no valid content */
        if (buf[i] == 0xFFFF)
        {
            break;
        }
		/*
		BIT[0~9]	:	RegAddr
		BIT[10]		:	Pi access
		BIT[11~12]  :   Reserved
		BIT[13~15]	:	RegType*/
        RegType = buf[i] >> 13;
        RegAddr = buf[i] & 0x3FF;
        RegValue = buf[i + 1];
#if defined (_NEW_MODEM_PI_ACCESS_) || defined(_NEW_RFC_PI_ACCESS_)
		RegPi = (buf[i] >> 10) & BIT0;
#endif
        error = FALSE;

        switch(RegType)
        {
        case 0:
            /* RF (only 6-bit address valid) */
            if (RegAddr & RF_INIT_AD_BIT_MODE)
            {   /*  RF bit-mode
                      RegAddr[9]: bit-mode enable
                      RegAddr[8]: bit-value to write
                      RegAddr[5:0]: RF RFC address
                      RegValue[15:12]: RF RFC bit location */
                UINT16 bit_val = (RegAddr & RF_INIT_AD_BIT_VAL_1);
                UINT32 bit_mask = GEN_BIT_MASK((RegValue >> 12));
                if (bit_val)
                {
                    RTK_UPDATE_RF_REG(RegAddr & 0x3F, bit_mask, bit_mask);
                }
                else
                {
                    RTK_UPDATE_RF_REG(RegAddr & 0x3F, bit_mask, 0x0000);
                }
            }
            else
            {
#ifdef _NEW_RFC_PI_ACCESS_
				if(RegPi)
				{
					rtk_write_rfc_reg_pi(RegAddr & 0x3F, RegValue, 0x1);
				}
				else
#endif
				{
            		/* word-mode */
                	RTK_WRITE_RF_REG(RegAddr & 0x3F, RegValue);
				}
            }
            break;

        case 1:
            new_page = RegAddr >> 8;
#ifdef _NEW_MODEM_PI_ACCESS_
			if(RegPi)
			{
				RTK_WRITE_MODEM_REG_PI(new_page, RegAddr, RegValue);
			}
			else
#endif
			{
            	if (cur_page != new_page)
	            {
	                cur_page = new_page;
	                /* switch page */
	                RTK_WRITE_MODEM_REG(TRANS_MODEM_REG(0), new_page);
	            }
			    /* Modem,AFE (only 7-bit address valid) */
			    RTK_WRITE_MODEM_REG(TRANS_MODEM_REG(RegAddr & 0x7F), RegValue);
			}
			break;

        case 3:
            /* delay how many ms */
            if (RegAddr & RF_INIT_AD_BIT_MODE)
            {   /*  RF bit-mode wait OR timeout
                      RegAddr[9]: bit-mode enable
                      RegAddr[8]: bit-value to check
                      RegAddr[5:0]: RF RFC address
                      RegValue[15:12]: RF RFC bit location
                      RegValue[11:0]: timeout delay (ms) */
                UINT16 bit_val = (RegAddr & RF_INIT_AD_BIT_VAL_1);
                UINT32 bit_mask = GEN_BIT_MASK((RegValue >> 12));
                UINT16 ii = RegValue & 0x0FFF;
                UINT16 cmp_eq;
                for (;ii>0; ii--)
                {
                    /* check if escape-condition holds */
                    cmp_eq = ((RTK_READ_RF_REG(RegAddr & 0x3F) & bit_mask) == 0) == (bit_val == 0);
                    if (cmp_eq)
                    {
                        break;
                    }
                    pf_delay(1);
                }
                if (ii == 0)
                {
                    error = TRUE;
                }
            }
            else
            {   /* simple delay */
                pf_delay(RegValue);
            }
            break;

        case 4:
            /* Vendor Regsiter */
            if (RegAddr & 0x01)
            {
                error = TRUE;
                break;
            }
            WR_16BIT_IO(VENDOR_BASE_ADDRESS, RegAddr, RegValue);
            break;

#ifdef _BT_ONLY_
        case 5:
            /* Page 0 system Register */
            if (RegAddr & 0x01)
            {
                error = TRUE;
                break;
            }
            WR_16BIT_SYSON_IO(RegAddr, RegValue);
            break;
#endif

#ifdef _8821A_BTON_DESIGN_
        case 6:
            /* Page 0 system Register */
            if (RegAddr & 0x01)
            {
                error = TRUE;
                break;
            }
            WR_16BIT_IO(BT_DMA_UART_BASE_ADDRESS, RegAddr, RegValue);
            break;
#endif
#ifdef LE_MODE_EN
        case 7:
            /* LE Register */
            if (RegAddr & 0x01)
            {
                error = TRUE;
                break;
            }
            WR_LE_REG(RegAddr, RegValue);
            break;
#endif
        default:
            /* Bluewize Regsiter */
            if (RegAddr & 0x01)
            {
                error = TRUE;
                break;
            }
            BB_write_baseband_register(RegAddr, RegValue);
            break;
        }


        if (error)
        {
            RT_BT_LOG(RED, MSG_RT8723_NEW_RF_INIT_ERR, 3,
                                        RegType, RegAddr, RegValue);
        }

        //RF0380_LOG(GRAY, MSG_RT8723_NEW_RF_PHY_INIT, 4,
        //                 i, RegType, RegAddr, RegValue);

    }
}
void rtl8703b_acut_rf_parameter_parse(void)
{
    RTK_WRITE_MODEM_REG(TRANS_MODEM_REG(0x00),0x0003);
	RTK_WRITE_MODEM_REG(TRANS_MODEM_REG(0x14),0xfa50);
	RTK_WRITE_MODEM_REG(TRANS_MODEM_REG(0x00),0x0000);
    UINT8 i;
    for( i = 0 ; i < 179 ; i++)
    {
        if( (i == 0) || (i == 41) || (i == 106) || (i == 171))
        {
            RTK_WRITE_MODEM_REG(TRANS_MODEM_REG(0x3c),g_8703b_a_cut_rf_parameter_array_rx_gain_table[i]);
        }
        else
        {
            RTK_WRITE_MODEM_REG(TRANS_MODEM_REG(0x3e),g_8703b_a_cut_rf_parameter_array_rx_gain_table[i]);
        }
    }
}



void rtl8723_btfr_SetTxGainTable(INT8 RFGainDiff)
{
    INT8 TxGainIdx[3];
    UINT8 GainStep;
    INT8 n;
//    UINT8 gain[3][MAX_RTL8723RF_TX_POWER_INDEX + 1];
    UINT8 gain[3][7];
    UINT8 early_exit = FALSE;
    UINT8 min_n = 0;
    UINT16 tx_agc_reg_base[3] = {TX_AGC_1M_LUT0_REG,
                                 TX_AGC_2M_LUT0_REG,
                                 TX_AGC_3M_LUT0_REG};
    UINT16 temp[4];
    UINT8 i;
    UINT8 j;

#ifdef _ROM_CODE_PATCHED_
    // Reserved com code patch by yilinli //
    if (rcp_rtl8723_btfr_SetTxGainTable != NULL)
    {
        if (rcp_rtl8723_btfr_SetTxGainTable((void*)&RFGainDiff))
        {
            return;
        }
    }
#endif


    GainStep = otp_str_data.EFuse_PHYInit_TxGainStep;

    /* get maximum tx index of 1M/2M/3M from efuse */
    TxGainIdx[0] = otp_str_data.EFuse_PHYInit_MaxTxGain1M + RFGainDiff;
    TxGainIdx[1] = otp_str_data.EFuse_PHYInit_MaxTxGain2M + RFGainDiff;
    TxGainIdx[2] = otp_str_data.EFuse_PHYInit_MaxTxGain3M + RFGainDiff;

    for (i = 0; i < 3; i++)
    {
        if (TxGainIdx[i] < 0)
        {
            TxGainIdx[i] = 0;

        }
        else if (TxGainIdx[i] > otp_str_data.EFuse_PHYInit_MaxTxIndex)
        {
            //can't exceed the max Tx index
            TxGainIdx[i] = otp_str_data.EFuse_PHYInit_MaxTxIndex;
        }
    }

    /* get tx gain table of 1M/2M/3M from efuse */
    for (n = g_max_rtl8723rf_tx_power_index; n >= 0; n--)
    {
        for (i = 0; i < 3; i++)
        {
            gain[i][n] = otp_str_data.EFuse_PHYInit_TxGainTable[TxGainIdx[i]];

            if (TxGainIdx[i] == 0)
            {
                early_exit = TRUE;
                break;
            }

            RF0380_LOG(GRAY, MSG_RT8723_NEW_RF_TXGAIN, 4,
                             i+1, TxGainIdx[i], GainStep, gain[i][n]);

            if (TxGainIdx[i] > GainStep)
            {
                TxGainIdx[i] -= GainStep;
            }
            else
            {
                TxGainIdx[i] = 0;
            }
        }

        if (early_exit)
        {
            min_n = (n == g_max_rtl8723rf_tx_power_index) ?
                    g_max_rtl8723rf_tx_power_index : (n + 1);
            break;
        }
    }

    max_rtk_radio_tx_step_index = g_max_rtl8723rf_tx_power_index - min_n;

    if (max_rtk_radio_tx_step_index > otp_str_data.bt_default_tx_power_index_minus)
    {
        default_rtk_radio_tx_step_index = max_rtk_radio_tx_step_index -
                                otp_str_data.bt_default_tx_power_index_minus;
    }
    else
    {
        default_rtk_radio_tx_step_index = 0;
    }

    for (i = 0; i < 3; i++)
    {
        n = min_n;
        for (j = 0; j < 8; j++)
        {
            *((UINT8*)temp + j) = gain[i][n]; /* direct to map rf
                                                 register 0x03[15:8] */
            if (n != g_max_rtl8723rf_tx_power_index)
            {
                n++;
            }
        }
        BB_write_baseband_register(tx_agc_reg_base[i] + 0, temp[0]);
        BB_write_baseband_register(tx_agc_reg_base[i] + 2, temp[1]);
        BB_write_baseband_register(tx_agc_reg_base[i] + 4, temp[2]);
        BB_write_baseband_register(tx_agc_reg_base[i] + 6, temp[3]);
    }

#ifdef LE_MODE_EN
    le_tx_power_max_index = otp_str_data.EFuse_PHYInit_MaxTxGainLE + RFGainDiff;
    le_tx_power_max_value = otp_str_data.EFuse_PHYInit_TxGainTable[le_tx_power_max_index];

    LE_REG_S_SET ll_reg;

    /* set default power to maximum */
    ll_reg.value = RD_LE_REG(LE_REG_SLAVE_CONN_WIN_SIZE);
    ll_reg.slave_win_size.slave_tx_power = LE_DEAULT_HW_TX_POWER;
    WR_LE_REG(LE_REG_SLAVE_CONN_WIN_SIZE, ll_reg.value);

    /* set advertising channel tx power */
    ll_reg.value = RD_LE_REG(LE_REG_CBK_CONTROL);
    ll_reg.cbk_ctrl.adv_tx_power = LE_DEAULT_ADV_TX_POWER;
    WR_LE_REG(LE_REG_CBK_CONTROL, ll_reg.value);

    RF0380_LOG(GRAY, MSG_RT8723_NEW_RF_LE_TXGAIN, 2,
                 le_tx_power_max_index,
                 otp_str_data.EFuse_PHYInit_TxGainTable[le_tx_power_max_index]);
#endif
}

void rtl8723_btrf_TxGainTableInit(void)
{
    rtl8723_btfr_SetTxGainTable(0);

    /* Tx Scaling Factor */
    if (otp_str_data.EFuse_PHYInit_TxScaleFactor != EFUSE_INVALID_TXSCALEFACTOR)
    {
#ifdef _NEW_MODEM_PI_ACCESS_  // to avoid run-time access page-conflicts //
        RTK_UPDATE_MODEM_REG_PI(MODEM_PI_PAGE_0, TRANS_MODEM_REG(0x16), 0x3c,
                         otp_str_data.EFuse_PHYInit_TxScaleFactor << 2);
#else
        RTK_UPDATE_MODEM_REG(TRANS_MODEM_REG(0x16), 0x3c,
                            otp_str_data.EFuse_PHYInit_TxScaleFactor << 2);
#endif
    }

    if (otp_str_data.EFuse_PHYInit_TxDACCurrent != EFUSE_INVALID_TXDAC_CURRENT)
    {
#ifdef _NEW_MODEM_PI_ACCESS_  // to avoid run-time access page-conflicts //
        /* DAC current */
        RTK_UPDATE_MODEM_REG_PI(MODEM_PI_PAGE_0, TRANS_MODEM_REG(0x38), 0xf8,
                         otp_str_data.EFuse_PHYInit_TxDACCurrent << 3);
#else
        /* DAC current */
        RTK_UPDATE_MODEM_REG(TRANS_MODEM_REG(0x38), 0xf8,
                             otp_str_data.EFuse_PHYInit_TxDACCurrent << 3);
#endif
    }
}


void rtl8723_btrf_RFIQK(void)
{
#ifdef _ROM_CODE_PATCHED_
    //Insert ROM code patch for IQK Pre-setup here...
    if (rcp_rf_iqk_func != NULL)
    {
        rcp_rf_iqk_func();
    }
#endif
}

void rtl8723_btrf_TxPowerTrack(INT8 N_new)
{
    INT8 index;
    INT8 RFGainDiff;
    UINT8 ScalingFactorIndex = 0;


#ifdef _ROM_CODE_PATCHED_
    // Reserved com code patch by yilinli //
    if (rcp_rtl8723_btrf_TxPowerTrack != NULL)
    {
        if (rcp_rtl8723_btrf_TxPowerTrack((void*)&N_new))
        {
            return;
        }
    }
#endif

    if (N_new != g_rtl8723_btrf_txpower_track_n_old)
    {
        index = N_new + BT_RF_TX_POWER_TRACK_INDEX_OFFSET;

        if (index < 0)
        {
            index = 0;
        }
        else if (index > (BT_RF_TX_POWER_TRACK_INDEX_OFFSET*2))
        {
            index = BT_RF_TX_POWER_TRACK_INDEX_OFFSET*2;
        }

        //Update Tx Gain Table
#ifndef RTL8723A_B_CUT
        RFGainDiff = (otp_str_data.EFuse_TxPowerTrack_TxGain[index] & 0xf0) >> 4;
#else
        UINT8 temp;
        if (index <= 1)
        {
            /* index 0 and 1 */
            temp = otp_str_data.EFuse_TxPowerTrack_TxGain_LBond[index];
        }
        else if (index >= 15)
        {
            /* index 15 and 16 */
            temp = otp_str_data.EFuse_TxPowerTrack_TxGain_HBond[index - 15];
        }
        else
        {
            /* index 2 ~ 14 */
            temp = otp_str_data.EFuse_TxPowerTrack_TxGain[index - 2];
        }
        RFGainDiff = temp >> 4;
#endif

        //unsigned to signed
        if (RFGainDiff > 7)
        {
            RFGainDiff = RFGainDiff - 16;
        }

        /* wifi driver can adjust the gain index via mailbox command */
        RFGainDiff -= g_rtl8723_btrf_txpower_track_extra_delta;

        rtl8723_btfr_SetTxGainTable(RFGainDiff);

        //Update Modem Tx Scaling Factor
#ifndef RTL8723A_B_CUT
        ScalingFactorIndex = otp_str_data.EFuse_TxPowerTrack_TxGain[index] & 0xf;
#else
        ScalingFactorIndex = temp & 0x0f;
#endif

#ifdef _NEW_MODEM_PI_ACCESS_  // to avoid run-time access page-conflicts //
        RTK_UPDATE_MODEM_REG_PI(MODEM_PI_PAGE_0, TRANS_MODEM_REG(0x16), 0x3C, ScalingFactorIndex << 2); // to be confirmed //
#else
        RTK_UPDATE_MODEM_REG(TRANS_MODEM_REG(0x16), 0x3C, ScalingFactorIndex << 2);
#endif

        RF0380_LOG(GRAY, MSG_RT8723_RF_TXPOWER_TRACK, 3,
                 g_rtl8723_btrf_txpower_track_n_old, N_new, ScalingFactorIndex);

        g_rtl8723_btrf_txpower_track_n_old = N_new;  //uodate N_old value
    }
}

void rtl8723_btrf_CFOTrack(INT8 N_new)
{
    INT8 index;
    INT8 CapOffset = 0;

    if (N_new != g_rtl8723_btrf_cfo_track_n_old)
    {
        index = N_new + BT_RF_CFO_TRACK_INDEX_OFFSET;
        if (index < 0)
        {
            index = 0;
        }
        else if (index > (BT_RF_CFO_TRACK_INDEX_OFFSET*2))
        {
            index = BT_RF_CFO_TRACK_INDEX_OFFSET*2;
        }

        //7 bits signed to unsigned format
#ifndef RTL8723A_B_CUT
        CapOffset = otp_str_data.EFuse_CFOTrack_Cap[index];
#else
        if (index <= 1)
        {
            /* index 0 and 1 */
            CapOffset = otp_str_data.EFuse_CFOTrack_Cap_LBond[index];
        }
        else if (index >= 15)
        {
            /* index 15 and 16 */
            CapOffset = otp_str_data.EFuse_CFOTrack_Cap_HBond[index - 15];
        }
        else
        {
            /* index 2 ~ 14 */
            CapOffset = otp_str_data.EFuse_CFOTrack_Cap[index - 2];
        }
#endif

        if (CapOffset > 63)
        {
            CapOffset = CapOffset - 128;
        }

#ifndef _BT_ONLY_
        /* TODO: Must use mailbox to do this */
#ifdef _ENABLE_MAILBOX_
        UINT8 mailbox_in[4];
        mailbox_in[0] = WIFI_CL_EN;
        mailbox_in[1] = 1;
        mailbox_in[2] = CapOffset;
        mailbox_in[3] = 0;

        RT_BT_LOG(BLUE,NOTIFY_WIFI_CALIBRATION,0,0);
        pf_os_trigger_mailbox_task(MAILBOX_WRITE, (UINT32)
                                    *((UINT32 *)mailbox_in), 0);
#endif

#else
        UINT32 temp = RD_32BIT_SYSON_IO(0x2C);
        UINT8 cap = (g_cap_default + CapOffset) & 0x3f;
        temp = (temp & 0xff000fff) | (cap << 12) | (cap << 18);
        WR_32BIT_SYSON_IO(0x2c, temp);
#endif

        RF0380_LOG(GRAY, MSG_RT8723_RF_CFO_TRACK, 3,
                        g_rtl8723_btrf_cfo_track_n_old, N_new, CapOffset);

        g_rtl8723_btrf_cfo_track_n_old = N_new;  //uodate N_old value
    }
}

void rtl8723_btrf_UpdateThermalValueTimer(void)   //update Thermal Value periodically
{
    UINT8 Thermal_Now;
    INT8 Thermal_Diff;

#ifndef _NEW_THERMAL_TRACK_
    BB_write_baseband_register(0x1ba, 0x800f); //enable Thermal meter after each Tx_ON
    Thermal_Now = (BB_read_baseband_register(0x1ba) >> 8) & 0x3F;
#else
	/*new style thermal parser from 8703b a cut*/
	VENDOR_WRITE(0xBC, VENDOR_READ(0xBC) & (~BIT4));
	RTK_WRITE_RF_REG(0x00, 0x1000);
	RTK_WRITE_RF_REG(0x04, 0x0060);
	pf_delay_us(100);
	RTK_WRITE_RF_REG(0x0E,RTK_READ_RF_REG(0x0E)|BIT15);
	RTK_WRITE_RF_REG(0x4B,RTK_READ_RF_REG(0x3B));
	Thermal_Now = RTK_READ_RF_REG(0x4B)&0x3f;
	RTK_WRITE_RF_REG(0x0E,RTK_READ_RF_REG(0x0E) & (~BIT15));
#endif

    Thermal_Diff = Thermal_Now - otp_str_data.EFuse_ThermalDefault;
#ifdef _ROM_CODE_PATCHED_
    if (rcp_rtl8723_btrf_UpdateThermalValueTimer != NULL)
    {
        if (rcp_rtl8723_btrf_UpdateThermalValueTimer((UINT8*)&Thermal_Now, &Thermal_Diff))
        {
            return;
        }
    }
#endif

    RF0380_LOG(GRAY, MSG_RT8723_RF_THERMAL_TIMER, 7,
                                Thermal_Now, Thermal_Diff,
                                g_rtl8723_btrf_thermal_old,
                               otp_str_data.EFuse_TxPowerTrack_En,
                               otp_str_data.EFuse_CFOTrack_En,
                               otp_str_data.EFuse_RxGainTrack_En,
                               otp_str_data.EFuse_LOKTrack_En);

    if (Thermal_Now == 0)
    {
        return;
    }

    if (g_thermal_track_pause)
    {
        return;
    }

    /* We allow TxPower Tracking and CFO Tracking in Scan State */

    /*==========================*/
    /* update TxPower Tracking  */
    /*==========================*/
    if (otp_str_data.EFuse_TxPowerTrack_En)
    {
        if (otp_str_data.EFuse_TxPowerTrack_ThermalDelta == 0)
        {
            /* avoid exception */
            otp_str_data.EFuse_TxPowerTrack_ThermalDelta = 1;
        }
        rtl8723_btrf_TxPowerTrack(Thermal_Diff / otp_str_data.EFuse_TxPowerTrack_ThermalDelta);
    }

    /*=========================*/
    /* update CFO Tracking     */
    /*=========================*/
    if (otp_str_data.EFuse_CFOTrack_En)
    {
        if (otp_str_data.EFuse_CFOTrack_ThermalDelta == 0)
        {
            /* avoid exception */
            otp_str_data.EFuse_CFOTrack_ThermalDelta = 1;
        }
        rtl8723_btrf_CFOTrack(Thermal_Diff / otp_str_data.EFuse_CFOTrack_ThermalDelta);
    }
}

void rtl8723_btrf_start_thermal_timer(void)
{
#ifdef _BT_ONLY_
    g_cap_default = (RD_32BIT_SYSON_IO(0x2C) & 0x00fc0000) >> 18;
#endif

    if (otp_str_data.EFuse_ThermalUpdateInterval != 0)
    {
        if (g_rtl8723_btrf_thermal_value_timer == OS_INVALID_HANDLE)
        {
            OS_CREATE_TIMER((UCHAR)PERIODIC_TIMER,
                     &g_rtl8723_btrf_thermal_value_timer,
                     (OS_TIMEOUT_HANDLER)rtl8723_btrf_UpdateThermalValueTimer,
                     (void *)NULL, 0);

            OS_START_TIMER(g_rtl8723_btrf_thermal_value_timer,
                           otp_str_data.EFuse_ThermalUpdateInterval * 1000);
        }
    }
}

#endif

#ifdef _SUPPORT_LBT_FUNC_FOR_RF_ADAPTIVITY_
UINT8 rtl8723_btrf_check_and_enable_lbt(UINT8 power_index)
{
#ifdef _TEST_ADAPTIVITY_FUNC_
    return TRUE;
#endif

    if (IS_SUPPORT_RF_ADAPTIVITY && (power_index >= g_tx_power_over_10dbm_index))
    {
        return TRUE;
    }
    else
    {
        return FALSE;
    }
}
#endif


