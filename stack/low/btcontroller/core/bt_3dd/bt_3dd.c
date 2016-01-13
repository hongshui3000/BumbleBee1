/***************************************************************************
 Copyright (C) Realtek
 This module is a confidential and proprietary property of Realtek and
 a possession or use of this module requires written permission of Realtek.
 ***************************************************************************/

/**
 * \file
 *  3dd.c (3D Dislay)
 *
 * \author
 *  austin <austin_chen@realtek.com>, (C) 2012
 */

/** \addtogroup 3D Display Driver Module
 *  @{ */

enum { __FILE_NUM__= 80 };

#include "bt_3dd.h"
#include "mem.h"
#include "bt_fw_hci_external_defines.h"
#include "bb_driver.h"
#include "lc_internal.h"
#include "lmp_ch_assessment.h"
#include "lmp_internal.h"
#include "bt_fw_hci_csa4_cmds_evts.h"
#include "hci_vendor_defines.h"
#include "bt_fw_hci_internal.h"
#ifdef CONFIG_TV_POWERON
#include "tv.h"
#endif

extern UCHAR hci_handle_write_eir_data(HCI_CMD_PKT *hci_cmd_ptr);
extern SECTION_ISR_LOW void BB_handle_hlc_interrupt_in_scatternet(UCHAR phy_piconet_id);

BT_3DD_MANAGER_S bt_3dd_var;

#ifdef _SUPPORT_CSB_TRANSMITTER_
/* dm1 < dh1 < 2dh1 < 3dh1 < dm3 < dh3 <
   dm5 < dh5 < 2dh3 < 3dh3 < 2dh5 < 3dh5 */
const UINT8 g_host_hci_packet_type_bitnum_sort_by_maxlen[12] = {
    3, 4, 1, 2, 10, 11, 14, 15, 8, 9, 12, 13};

const UINT8 g_host_hci_packet_type_ptt_sort_by_maxlen[12] = {
    0, 0, 1, 1, 0, 0, 0, 0, 1, 1, 1, 1};
#endif
extern UINT16 g_beacon_rx_xtol;
#ifdef _CSB_RX_DBG_LOG
extern UINT8 g_csb_rx_dbg_log;
#endif
#ifdef _DAPE_TEST_CSB_RX_SELF_CALC_CLK_OFST_BY_VENDOR_CMD
extern UINT8 g_self_calc_clk_ofst;
#endif
#ifdef _CSB_TEST_RX_BEACON_AFTER_RX_SYNC_TRAIN
extern UCHAR g_ptt_enabled;
#endif

//
// 20140417 morgan add from 8761dcut//
#ifdef _ROM_CODE_PATCHED_
PF_ROM_CODE_PATCH_FUNC rcp_generate_3dd_gpio_sync_toggle_response_func = NULL;
PF_ROM_CODE_PATCH_VOID rcp_bt_3dd_driver_handle_ext_clock_capture_by_edge_trigger = NULL;
PF_ROM_CODE_PATCH_VOID rcp_bt_3dd_driver_fn_set_shutter_delay = NULL;
PF_ROM_CODE_PATCH_VOID rcp_bt_3dd_driver_fn_set_host_shutter_delay = NULL;
#endif

//20131206 morgan add for 3DD FPGA test

#define PERIOD_60HZ 0
#define PERIOD_120HZ 1
#define PERIOD_50HZ 2
#define INITIAL_PERIOD_STATE 0
#define CALCULATE_PERIOD_STATE 1
UINT8 long_term_period_window = 30;

INT16 bt_3dd_shutter_lsod = 50;
INT16 bt_3dd_shutter_lscd = -50;
INT16 bt_3dd_shutter_rsod = 50;
INT16 bt_3dd_shutter_rscd = -50;

UINT8 rejection_cnt =0;
UINT8 long_term_state = 0;
UINT8 flag_rejection_end = 0;
UINT32 init_ext_frame_period_us = 0;
UINT8 toggle_delay_check_cnt = 0;
UINT8 enable_toggle_delay_check = 0;
UINT8 period_significant_diff_cnt = 0;
UINT8 period_significant_diff_long_term_cnt =0;
UINT8 state_period_detection = 0;

UINT8 ext_clk_edge_counts_length = 0xf;
UINT8 ext_clk_edge_counts_period = 16;
UINT8 ext_clk_edge_counts_avg =4;

#ifdef _3DD_ONLY_SUPPORT
UINT8 efuse_3dd_only = 0;
UINT8 efuse_sync_train_always_on = 1;
UINT16 efuse_inquiry_scan_window = 36;
UINT16 efuse_inquiry_scan_interval = 256;
UINT16 efuse_page_scan_window = 18;
UINT16 efuse_page_scan_interval = 256;
#endif

#define DEFAULT_PERIOD_60HZ

UINT8 cur_coeff_gain = 3;
UINT8 cur_coeff_avg = 4;
UINT8 cur_coeff_q_gain =1;
// for 3DD initial value setting
UINT8 flag_toggle_restart =0;
//UINT8 d3d_sync_toggle_restart_cnt = 0;
UINT16 BT_EXT_FRAME_PERIOD_120HZ_US_DEFAULT = 8333;
UINT16 BT_EXT_FRAME_PERIOD_50HZ_US_DEFAULT = 20000;
UINT16 BT_EXT_FRAME_PERIOD_60HZ_US_DEFAULT= 16666;  /* unit: us, default 60 hz */
#ifdef DEFAULT_PERIOD_60HZ
INT32 ext_frame_period_us_q = 16666;
INT32 long_term_avg_period = 16666;
UINT8 period_mode = 0;
#endif

#ifdef DEFAULT_PERIOD_50HZ
INT32 ext_frame_period_us_q = 20000;
INT32 long_term_avg_period = 20000;
UINT8 period_mode = 2;
#endif

#ifdef DEFAULT_PERIOD_120HZ
INT32 ext_frame_period_us_q = 8333;
INT32 long_term_avg_period = 8333;
UINT8 period_mode = 1;
#endif

//UINT8 period_mode = 0;

UINT8 avg_cnt = 0;
// declare for 3DD toggle detection
//20131106 morgan add for 3dd toggle report
UINT8 gpio_3dd_sync_toggle_cnt;
UINT8 d3dd_cnt = 0;
UINT8 flag_3dd_toggle_q;

TimerHandle_t hTimerID3DsyncMeter = NULL;
TimerHandle_t stp_timer = NULL;  /* synchronization train timer */
TimerHandle_t csb_trx_supervision_timer = NULL; /* csb trx supervision timer */

// for 3DD define function
void bt_3dd_driver_fn3DSyncToggleTimer(TimerHandle_t timer);
void bt_3dd_driver_generate_3dd_gpio_sync_toggle_response(UINT8 status);
void bt_3dd_driver_write_period_to_reg(UINT32 period_us);
extern UCHAR hci_handle_write_eir_data(HCI_CMD_PKT *hci_cmd_ptr);
BT_3DD_MANAGER_S bt_3dd_var;

////////////////////////////////////////////////////////

/**************************************************************************
 * Function     : bt_3dd_driver_init
 *
 * Description  : This function is used to initiate bluetooth 3dd and csb
 *                function
 *
 * Parameters   : None
 *
 * Returns      : None
 *
 *************************************************************************/
void bt_3dd_driver_init_imp(void)
{
    memset(&bt_3dd_var, 0, sizeof(BT_3DD_MANAGER_S));
    bt_3dd_var.synctrain.interval = BT3DD_SYNC_TRAIN_PERIOD_DEFAULT;
    bt_3dd_var.beacon.interval = BT3DD_BEACON_PERIOD_DEFAULT;

#ifdef _SUPPORT_CSB_TRANSMITTER_
    bt_3dd_var.is_csb_mode = TRUE; // TODO: come from efuse

    /* fill synchronization train non-zero default parameters */
    memcpy(bt_3dd_var.stp_tx_param.ch_map, afh_ahs79_channel_map, LMP_AFH_MAP_SIZE);
    bt_3dd_var.stp_tx_param.interval_min = BT_CSB_SYNC_TRAIN_INTERVAL_DEFAULT;
    bt_3dd_var.stp_tx_param.interval_max = BT_CSB_SYNC_TRAIN_INTERVAL_DEFAULT;
    bt_3dd_var.stp_tx_param.interval = BT_CSB_SYNC_TRAIN_INTERVAL_DEFAULT;
    bt_3dd_var.stp_tx_param.timeout = BT_CSB_SYNC_TRAIN_TIMEOUT_DEFAULT;
    bt_3dd_var.stp_tx_param.num_of_next_beacon_inst = BT_CSB_BEACON_OFFSET;
    bt_3dd_var.stp_tx_param.stp2stp_duration = BT_CSB_SYNC_TRAIN_PACKET_PERIOD;

    /* fill connectionless slave broadcast non-zero default parameters */
    bt_3dd_var.csb_tx_param.lt_addr = 1;
    bt_3dd_var.csb_tx_param.fragment = 3;
    bt_3dd_var.csb_tx_param.support_packet_type_bm = 0xCC18;
    bt_3dd_var.csb_tx_param.interval_min = BT_CSB_BEACON_INTERVAL_DEFAULT;
    bt_3dd_var.csb_tx_param.interval_max = BT_CSB_BEACON_INTERVAL_DEFAULT;
    bt_3dd_var.csb_tx_param.interval = BT_CSB_BEACON_INTERVAL_DEFAULT;
    bt_3dd_var.csb_tx_param.supervision_timeout = BT_CSB_BEACON_SUPERVISION_TIMEOUT_DEFAULT;

    if (stp_timer != NULL)
    {
        /* stop the timer if we have the body of sync train protocol timer */
        OS_DELETE_TIMER(&stp_timer);
    }

    if (csb_trx_supervision_timer != NULL)
    {
        /* stop the timer if we have the body of supervision timer */
        OS_DELETE_TIMER(&csb_trx_supervision_timer);
    }
#endif

#ifdef _NEW_3DD_HW_
    /* set frame clock 60hz be default (16666.66~67 us)
     => period 16666 us, fraction = 170 */
    bt_3dd_var.ext_clk_period = 16666;
    bt_3dd_var.ext_clk_fraction = 170;

    /* select CSB or Reference Protocol mode */
    BB_write_baseband_register(BB_3DD_SYNC_BB_TIME_US_REG,
            ((!bt_3dd_var.is_csb_mode) << 9)
            | (bt_3dd_var.stp_tx_param.num_of_next_beacon_inst << 10)
            | (bt_3dd_var.stp_tx_param.stp2stp_duration << 12));
#endif
#ifdef _CSB_RX_SET_XTOL_BY_EFUSE
    g_beacon_rx_xtol = ((otp_str_data.bt_func_support_policy_ext >> 24) & 0x0F);
    RT_BT_LOG(RED, DAPE_TEST_LOG293, 1, g_beacon_rx_xtol);
#endif

#ifdef _3DD_ONLY_SUPPORT
    RT_BT_LOG(GREEN, DD_MSG_CMD2, 1, efuse_3dd_only);
#endif
#ifdef _8761_D_CUT_3D_SYNC_SCHMITT_EN
    CORE_AND_AFE_LDO_CTRL_S_TYPE core_ctrl_reg;
    core_ctrl_reg.d32 = VENDOR_READ(BTON_CORE_AFE_LDO_CTRL_REG);
    core_ctrl_reg.b.en_3d_sync_schmitt = efuse_tv_3dd->enable_3d_sync_schmitt;
    VENDOR_WRITE(BTON_CORE_AFE_LDO_CTRL_REG,core_ctrl_reg.d32);
#endif
    /* fill synchronization train non-zero default parameters */
    // 20140417 morgan : the structure modified
    /*
    memcpy(bt_3dd_var.stp_tx_param.ch_map, afh_ahs79_channel_map, LMP_AFH_MAP_SIZE);
    bt_3dd_var.stp_tx_param.ch_map[9] = 0x3F;
    */
    /* workaround hw issue */

    /*
    bt_3dd_var.stp_tx_param.interval = BT_CSB_SYNC_TRAIN_INTERVAL_DEFAULT;
    bt_3dd_var.stp_tx_param.timeout = BT_CSB_SYNC_TRAIN_TIMEOUT_DEFAULT;
    bt_3dd_var.stp_tx_param.stp_timer = OS_INVALID_HANDLE;
    */

    /* fill connectionless slave broadcast non-zero default parameters */
    //bt_3dd_var.csb_tx_param.lt_addr = 1;
    //bt_3dd_var.csb_tx_param.interval = BT_CSB_BEACON_INTERVAL_DEFAULT;

    // @@@@ morgan : no ext_frame_period_us element
    /*
#ifdef DEFAULT_PERIOD_60HZ
     bt_3dd_var.csb_tx_param.ext_frame_period_us = BT_EXT_FRAME_PERIOD_60HZ_US_DEFAULT;
#else
     bt_3dd_var.csb_tx_param.ext_frame_period_us = long_term_avg_period;
#endif
    */
    RT_BT_LOG(GREEN, DD_MSG_CMD14, 1, long_term_avg_period);

    fn_set_shutter_delay();

    // 20131107 morgan add for toggle detection
    gpio_3dd_sync_toggle_cnt = 0;
    flag_3dd_toggle_q = 0;

    /* GPIO12 = 3DG_SYNC  set reg 0xAC[27:26] = [01] */
    {
        UINT32 val = VENDOR_READ(0xAC); //BTON_BT_RFE_PAD_CTRL_REG
        val &= ~BIT27;
        val |= BIT26;
        VENDOR_WRITE(0xAC, val);
    }

    /* 20131217 morgan add for 3DSP version setting */

    BZ_REG_S_OPT_REG opt_reg;
    *(UINT16*) &opt_reg = BB_read_baseband_register(OPT_REGISTER);

    // @@@@ morgan : en_3dd_sync_mask, en_3dd_clk_shift element.  need wait for dape commit
    if (efuse_tv_3dd->version_3DSP) // 1 = v1.01 0=v1.0
    {
        //opt_reg.en_3dd_sync_mask = FALSE;
        opt_reg.en_3dd_clk_shift = TRUE;
    }
    else
    {
        opt_reg.en_3dd_clk_shift = FALSE;
        //opt_reg.en_3dd_sync_mask = FALSE;  // ???
    }
    BB_write_baseband_register(OPT_REGISTER, *(UINT16*) &opt_reg);

    if (hTimerID3DsyncMeter != NULL)
    {
        OS_DELETE_TIMER(&hTimerID3DsyncMeter);
    }
    OS_CREATE_TIMER(PERIODIC_TIMER, &hTimerID3DsyncMeter,
            bt_3dd_driver_fn3DSyncToggleTimer, NULL, 0);

    //RT_BT_LOG(YELLOW, PTA_DISPLAY_MESSAGE3, 2, hTimerID3DsyncMeter, 0);
    OS_START_TIMER(hTimerID3DsyncMeter, efuse_tv_3dd->toggle_detection_window * 250);

    /* 20131128 morgan add for 3D FPGA test : enable scan and modify parameter */
    /* dape: change scan interval setting for 3DD implementation. can be set by host. */

#ifdef _3DD_ONLY_SUPPORT
    lmp_self_device_data.inquiry_scan_window = efuse_inquiry_scan_window;
    lmp_self_device_data.inquiry_scan_interval = efuse_inquiry_scan_interval;
    lmp_self_device_data.page_scan_window = efuse_page_scan_window;
    lmp_self_device_data.page_scan_interval = efuse_page_scan_interval;
#endif
}
void (*bt_3dd_driver_init)(void) = bt_3dd_driver_init_imp;

/**
* Retrieves 3dd-inquiry-scan after being disabled by the Controller.
*
* \param None.
*
* \return None.
*/
void bt_3dd_driver_retrieve_inq_scan(void)
{
    if (lmp_self_device_data.scan_enable & 0x01)
    {
        /* FIXME: HW seems to flush the broadcast FIFO after the first EIR
         * packet is sent, so we have to write EIR data every time to prevent
         * incorrect EIR packets. This action won't be needed until HW fixes
         * this issue.
         */
        hci_check_and_enable_eir_trans();

        BB_write_baseband_register(INSTRUCTION_REGISTER,
                BB_INQUIRY_SCAN_3DD);
    }
}

/**************************************************************************
 * Function     : bt_3dd_driver_get_sync_native_clock
 *
 * Description  : This function is used to get the capture value of
 *                BT native clock (BB_clock[27:0] and usec offset) at the
 *                rising edge external frame sync signal.
 *
 * Parameters   : offset - the clock offset in usec unit
 *
 * Returns      : BB native clock[27:0]
 *
 *************************************************************************/
UINT32 bt_3dd_driver_get_sync_native_clock(UINT16 *offset)
{
    UINT32 clk;

#ifdef _NEW_3DD_HW_
    /* select native clock (bit[10:8]) be default capture clock type */
    BB_write_baseband_register(BB_3DD_SYNC_FRAME_PERIOD_FRACTION_REG,
                                            bt_3dd_var.ext_clk_fraction);
#endif

    /* capture native clock */
    clk = BB_read_baseband_register(BB_3DD_SYNC_BB_TIME1_REG) & 0x0fff;
    clk <<= 16;
    clk |= BB_read_baseband_register(BB_3DD_SYNC_BB_TIME0_REG);

    if (offset != NULL)
    {
        /* capture us clock */
        *offset = BB_read_baseband_register(BB_3DD_SYNC_BB_TIME_US_REG) & 0x1ff;
    }
    return clk;
}

/**************************************************************************
 * Function     : bt_3dd_driver_update_transmit_beacon_parameters
 *
 * Description  : This function is used to update the registers to fill some
 *                field (RSOD, RSCD, LSOD, LSCD, dual vedio stream mode) of
 *                beacon payload body in reference protocol.
 *
 * Parameters   : None
 *
 * Returns      : None
 *
 *************************************************************************/
void bt_3dd_driver_update_transmit_beacon_parameters(void)
{
    if (bt_3dd_var.beacon.is_2d_mode)
    {
        /* set right shutter open delay */
        BB_write_baseband_register(BB_3DD_RSOD_REG, 0xffff);

        /* set right shutter close delay */
        BB_write_baseband_register(BB_3DD_RSCD_REG, 0xffff);

        /* set left shutter open delay */
        BB_write_baseband_register(BB_3DD_LSOD_REG, 0xffff);

        /* set left shutter close delay */
        BB_write_baseband_register(BB_3DD_LSCD_REG, 0xffff);
    }
    else
    {
        /* set right shutter open delay */
        BB_write_baseband_register(BB_3DD_RSOD_REG, bt_3dd_var.beacon.rsod);

        /* set right shutter close delay */
        BB_write_baseband_register(BB_3DD_RSCD_REG, bt_3dd_var.beacon.rscd);

        /* set left shutter open delay */
        BB_write_baseband_register(BB_3DD_LSOD_REG, bt_3dd_var.beacon.lsod);

        /* set left shutter close delay */
        BB_write_baseband_register(BB_3DD_LSCD_REG, bt_3dd_var.beacon.lscd);
    }

    /* set dual video stream mode setting */
    {
        BZ_REG_CON_3DD reg = bz_reg_con_3dd_read();
        reg.stream_mode = bt_3dd_var.beacon.dual_video_stream_mode;
        bz_reg_con_3dd_write(reg);
    }
}

void bt_3dd_driver_set_3d_sync_debouncing(UINT16 window)
{
    CO_EXIST_CTRL_REG reg = co_exist_ctrl_reg_read();
    reg.deb_3d_sync_en = 1;
    reg.deb_win = window;
    co_exist_ctrl_reg_write(reg);
}

void bt_3dd_driver_unset_3d_sync_debouncing()
{
    CO_EXIST_CTRL_REG reg = co_exist_ctrl_reg_read();
    reg.deb_3d_sync_en = 0;
    reg.deb_win = 0;
    co_exist_ctrl_reg_write(reg);
}

#ifdef _NEW_3DD_HW_
/**************************************************************************
 * Function     : bt_3dd_driver_set_ext_frame_sync_period
 *
 * Description  : This function is used to update the registers to fill some
 *                field (frame sync period, frame sync fraction) of
 *                beacon payload body in reference protocol.
 *                Note : in old design, these two field are generated by hw.
 *                In new design, these two field are generated by fw because
 *                it shall have high accuracy.
 *
 * Parameters   : None
 *
 * Returns      : None
 *
 *************************************************************************/
void bt_3dd_driver_set_ext_frame_sync_period(void)
{
    /* set the period of external frame sync (units: us) */
    BB_write_baseband_register(BB_3DD_SYNC_FRAME_PERIOD_REG,
                                bt_3dd_var.ext_clk_period);

    /* set the fraction of external frame sync (units: 1/256 us) */
    BB_write_baseband_register(BB_3DD_SYNC_FRAME_PERIOD_FRACTION_REG,
                                bt_3dd_var.ext_clk_fraction);
}
#endif

#if !defined(_SUPPORT_RP_VIA_CSB_SPEC_) && !defined(_DO_NOT_SUPPORT_RP_)
#ifdef _3DD_FUNCTION_TEST_
/**************************************************************************
 * Function     : bt_3dd_driver_generate_eir_payload
 *
 * Description  : This function is used to generate EIR payload of RP mode.
 *                (for firmware test only)
 *
 * Parameters   : None
 *
 * Returns      : None
 *
 *************************************************************************/
void bt_3dd_driver_generate_eir_payload(void)
{
    HCI_CMD_PKT hci_cmd_tmp;
    UINT8 *data = &hci_cmd_tmp.cmd_parameter[1];

    hci_cmd_tmp.cmd_opcode = HCI_WRITE_EXTENDED_INQUIRY_RESPONSE_OPCODE;
    hci_cmd_tmp.param_total_length = 241;
    hci_cmd_tmp.cmd_parameter[0] = 1;
    memset(data, 0, 240);

    /* parameters in the EIR packet */

    /**************************/
    /* manufacturer spec part */
    /**************************/
    data[0] = 0x06; /* manufacturer spec section length */
    data[1] = 0xff; /* manufacturer specific */
    data[2] = 0x0f; /* fixed id (lsb) */
    data[3] = 0x00; /* fixed id (msb) */
    data[4] = 0x00; /* reserved (1) */
    data[5] = 0x11; /* bit[0]: multicast capable
                       bit[1]: reserved
                       bit[2]: sending beacon
                       bit[3]: reserved (remote paired)
                       bit[4]: showroom mode
                       bit[5]: reserved (remote paired mode)
                       bit[6]: reserved
                       bit[7]: test mode for BQB test */
    data[6] = 0x3C; /* path loss threshold (dB) */

    /**************************/
    /* TX power part          */
    /**************************/
    data[7] = 0x02; /* tx power section length */
    data[8] = 0x0a; /* tx power level */
    data[9] = 0x00; /* tx power value (dBm) */

    hci_handle_write_eir_data(&hci_cmd_tmp);
}
#endif

/**************************************************************************
 * Function     : bt_3dd_driver_association_enable
 *
 * Description  : This function is used to enable association procedure of RP
 *                mode (for firmware test only)
 *
 * Parameters   : None
 *
 * Returns      : None
 *
 *************************************************************************/
void bt_3dd_driver_association_enable(void)
{
    UINT32 cod;

#ifdef _3DD_FUNCTION_TEST_
    bt_3dd_driver_generate_eir_payload();
#endif

    cod = BT3DD_COD_DWORD_I;
    lmp_self_device_data.class_of_device = cod;
    BB_write_baseband_register(CLASS_OF_DEVICE_REGISTER1, cod);
    BB_write_baseband_register_lower_octet(CLASS_OF_DEVICE_REGISTER2, cod >> 16);

    lmp_self_device_data.inquiry_scan_type = HCI_3DD_INQ_SCAN;
    lmp_self_device_data.scan_enable = 0x01;

    lc_start_write_scan_mode(1);
}

/**************************************************************************
 * Function     : bt_3dd_driver_association_disable
 *
 * Description  : This function is used to disable association procedure of RP
 *                mode (for firmware test only)
 *
 * Parameters   : None
 *
 * Returns      : None
 *
 *************************************************************************/
void bt_3dd_driver_association_disable(void)
{
    UINT32 cod;

    cod = otp_str_data.bt_class_of_device;
    lmp_self_device_data.class_of_device = cod;
    BB_write_baseband_register(CLASS_OF_DEVICE_REGISTER1, cod);
    BB_write_baseband_register_lower_octet(CLASS_OF_DEVICE_REGISTER2, cod >> 16);

    lc_start_write_scan_mode(0);
}

/**************************************************************************
 * Function     : bt_3dd_driver_start_sync_train
 *
 * Description  : This function is used to start the sync train tx of RP mode
 *
 * Parameters   : None
 *
 * Returns      : None
 *
 *************************************************************************/
void bt_3dd_driver_start_sync_train(void)
{
    /* set version field */
    BB_write_baseband_register(BB_3DD_CON_REG, 0x01);

    /* set tsync train period */
    BB_write_baseband_register(BB_3DD_SYNC_ITV_REG, bt_3dd_var.synctrain.interval);

    /* set afh channel map */
#ifdef _3DD_FUNCTION_TEST_
//    memcpy(afh_map_last, afh_ahs79_channel_map, 10);
    memset(afh_map_last, 0x33, 10);
#endif

    BB_write_afh_map(0, 0, afh_map_last);

    /* start the sync train */
    BB_write_baseband_register(INSTRUCTION_REGISTER, BB_START_SYNC_3DD);
}

/**************************************************************************
 * Function     : bt_3dd_driver_stop_sync_train
 *
 * Description  : This function is used to stop the sync train tx of RP mode
 *
 * Parameters   : None
 *
 * Returns      : None
 *
 *************************************************************************/
void bt_3dd_driver_stop_sync_train(void)
{
    /* kill the sync train */
    BB_write_baseband_register(INSTRUCTION_REGISTER, BB_KILL_SYNC_3DD);
}

/**************************************************************************
 * Function     : bt_3dd_driver_start_transmit_beacon
 *
 * Description  : This function is used to start the beacon tx of RP mode
 *
 * Parameters   : None
 *
 * Returns      : None
 *
 *************************************************************************/
void bt_3dd_driver_start_transmit_beacon(void)
{
    /* update 3dd beacon tx parameters */
    bt_3dd_driver_update_transmit_beacon_parameters();

    /* set beacon interval */
    BB_write_baseband_register(BB_3DD_BCN_ITV_REG, bt_3dd_var.beacon.interval);

    /* start the beacon (linkless broadcast) */
    BB_write_baseband_register(INSTRUCTION_REGISTER, BB_START_BEACON_3DD);
}

/**************************************************************************
 * Function     : bt_3dd_driver_stop_transmit_beacon
 *
 * Description  : This function is used to stop the beacon tx of RP mode
 *
 * Parameters   : None
 *
 * Returns      : None
 *
 *************************************************************************/
void bt_3dd_driver_stop_transmit_beacon(void)
{
    /* kill the beacon */
    BB_write_baseband_register(INSTRUCTION_REGISTER, BB_KILL_BEACON_3DD);
}

#ifdef _3DD_FUNCTION_TEST_
/**************************************************************************
 * Function     : bt_3dd_test_func
 *
 * Description  : This function is used to test 3dd function
 *
 * Parameters   : None
 *
 * Returns      : None
 *
 *************************************************************************/
void bt_3dd_test_func(void)
{
    bt_3dd_driver_association_enable();
    bt_3dd_driver_start_sync_train();
    bt_3dd_driver_start_transmit_beacon();
}
#endif
#endif

#if !defined(_DO_NOT_SUPPORT_RP_)
/**************************************************************************
 * Function     : bt_3dd_checkin_association_notification_packet
 *
 * Description  : This function is used to check-in association notification
 *                packet from BR's 3DG during association procedure
 *
 * Parameters   : pparam - the Pointer of BB Rx Parameters
 *
 * Returns      : None
 *
 *************************************************************************/
void bt_3dd_checkin_association_notification_packet(BB_RX_PARAMS *pparam)
{
    PKT_HEADER *ppkt;
    UINT32 rsvd_buf = FALSE;
    OS_SIGNAL rx_sig_send;
    UINT8 wptr;

    /* parser potential association notification packet */
    wptr = lc_rx_pkt_header_q.write_pointer;

    ppkt = &lc_rx_pkt_header_q.pkt_header_queue[wptr];

    /* Queue the packet in the headers queue */
    ppkt->packet_header = pparam->packet_header;
    ppkt->payload_header = pparam->payload_header;

    /* queue packet and update write pointer of queue */
    lc_rx_pkt_header_q.write_pointer =
                    (wptr + 1) & (LC_MAX_NUMBER_OF_HEADERS - 1);

    /* Send the signal to lc_rx_task to pick up the packet*/
    rx_sig_send.type = LC_HANDLE_RECD_PACKET_SIGNAL;
    rx_sig_send.param = (void*)(rsvd_buf);
    OS_ISR_SEND_SIGNAL_TO_TASK ( lc_rx_task_handle, rx_sig_send );
}

/**************************************************************************
 * Function     : bt_3dd_handle_received_assocation_notification_packet
 *
 * Description  : This function is used to handle received association
 *                notification packet from BR's 3DG during association
 *                procedure. FW can generate one hci vendor event to notify
 *                hci host.
 *
 * Parameters   : None
 *
 * Returns      : TRUE or FALSE
 *
 *************************************************************************/
UINT8 bt_3dd_handle_received_assocation_notification_packet(void)
{
    /* send AN packet indication from 3DG to host */
    HCI_EVENT_PKT *hci_event_pkt_ptr;
    OS_SIGNAL signal_send;

    RT_BT_LOG(GREEN, MSG_3DD_RP_RX_AN, 1, BB_read_native_clock());

    // here Host receive a AN packet ,
#ifdef _3DD_ONLY_SUPPORT
    if( efuse_3dd_only )
    {
        // here add sync train and beacon train enable
        UCHAR ret_error;
        UCHAR an_status = 1;

        // set flow
        // 0x0c78 -> 0x0441 -> 0x0443 - 0x0c76
        // sync train has no parameter
        //case HCI_WRITE_SYNCHRONIZATION_TRAIN_PARAMETERS_OPCODE:      // 0x0C78
        //break;
        RT_BT_LOG(GREEN, DD_MSG_CMD, 1, 0x0c78);

        //case HCI_SET_CONNECTIONLESS_SLAVE_BROADCAST_OPCODE:      // 0441
        // here write a register to set period :
        // bit file Beacon sync frame period : control by  0x230[15:0]
        // 16666us = 0x411A
        // BB_write_baseband_register(0x230,  0x4128); //411a
        ret_error = pat3d_sim_hci_handle_set_connectionless_slave_broadcast(an_status);

        //case HCI_START_SYNCHRONIZATION_TRAIN_OPCODE:            //0443
        ret_error = pat3d_sim_hci_handle_start_synchronization_train_command(an_status);

        //case HCI_SET_CONNECTIONLESS_SLAVE_BROADCAST_DATA_OPCODE:            // 0x0c76
        ret_error = pat3d_sim_hci_handle_set_connectionless_slave_broadcast_data(NULL,efuse_3dd_only);//(hci_cmd_ptr);

        // 20131107 morgan add for measure 3dd sync toggle
        // create a new timer => move to initialize
#ifndef BZDMA_DONT_WAIT_RXCMD_COMPLETE
        BB_read_baseband_RX_FIFO_and_flush(17, TRUE);
#else
        BB_read_baseband_RX_FIFO_and_flush(17, FALSE);
#endif
    } /* if( efuse_3dd_only ) */
    else
#endif /* _3DD_ONLY_SUPPORT */
    {
        // ==========================================================================
        // here is used to allocate buffer for event report
        if (OS_ALLOC_BUFFER(tx_table[HCI_EVENT_HANDLER_TASK].pool_handle,
                (OS_ADDRESS *)&hci_event_pkt_ptr) != BT_ERROR_OK)
        {
            return FALSE;
        }

        hci_event_pkt_ptr->event_opcode = HCI_VENDOR_SPECIFIC_EVENT;
        BB_read_baseband_RX_FIFO(&hci_event_pkt_ptr->event_parameter[2], 17,
                TRUE);
        hci_event_pkt_ptr->event_parameter[0] = 0x23;
        hci_event_pkt_ptr->event_parameter[1] = 0x00;
        hci_event_pkt_ptr->param_total_length = 19;

        signal_send.type = HCI_TD_HCI_EVENT_TO_HOST_SIGNAL;
        signal_send.param = (OS_ADDRESS *) hci_event_pkt_ptr;

        // Total length of the event packet = param_length (2nd  byte)+ Event type (1) + length field(1)
        signal_send.length = hci_event_pkt_ptr->param_total_length + 2;

        if (OS_SEND_SIGNAL_TO_TASK(hci_td_task_handle,
                signal_send) != BT_ERROR_OK)
        {
            OS_FREE_BUFFER(tx_table[HCI_EVENT_HANDLER_TASK].pool_handle,
                    hci_event_pkt_ptr);
        }
    }

    return TRUE;
}

#if 0
UINT8 bt_3dd_handle_received_assocation_notification_packet(void)
{
    /* send AN packet indication from 3DG to host */
    HCI_EVENT_PKT *hci_event_pkt_ptr;
    OS_SIGNAL signal_send;

    if (OS_ALLOC_BUFFER(tx_table[HCI_EVENT_HANDLER_TASK].pool_handle,
                    (OS_ADDRESS *)&hci_event_pkt_ptr) != BT_ERROR_OK)
    {
#ifndef BZDMA_DONT_WAIT_RXCMD_COMPLETE
        BB_read_baseband_RX_FIFO_and_flush(17, TRUE);
#else
        BB_read_baseband_RX_FIFO_and_flush(17, FALSE);
#endif
        return FALSE;
    }

    hci_event_pkt_ptr->event_opcode = HCI_VENDOR_SPECIFIC_EVENT;
    BB_read_baseband_RX_FIFO(&hci_event_pkt_ptr->event_parameter[2], 17, TRUE);
    hci_event_pkt_ptr->event_parameter[0] = 0x23;
    hci_event_pkt_ptr->event_parameter[1] = 0x00;
    hci_event_pkt_ptr->param_total_length = 19;

    signal_send.type = HCI_TD_HCI_EVENT_TO_HOST_SIGNAL;
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

    return TRUE;
}
#endif // #if 0
#endif

#ifdef _SUPPORT_CSB_TRANSMITTER_
/**************************************************************************
 * Function     : bt_csb_driver_get_ext_frame_sync_clock
 *
 * Description  : This function is used to get the capture value of
 *                BT native clock (BB_clock[27:0] and usec offset) or Piconet
 *                clock (piconet clock[27:0] and usec offset) at the
 *                rising edge external frame sync signal.
 *
 * Parameters   : native_clk - native clock (TRUE) or piconet clock (FALSE)
 *                piconet - the piconet number (invalid for native clock)
 *                offset - the clock offset in usec unit
 *
 * Returns      : BB native or piconet clock[27:0]
 *
 *************************************************************************/
UINT32 bt_csb_driver_get_ext_frame_sync_clock(UINT8 native_clk,
                                              UINT8 piconet,
                                              UINT16 *offset)
{
    UINT32 clk;
    UINT8 mod;
    UINT32 new_clk;
    UINT8 clk_id;

    if (native_clk)
    {
        clk_id = 0;
    }
    else
    {
        clk_id = piconet + 1;
    }

    /* select native clock (bit[10:8]) be default capture clock type */
    BB_write_baseband_register(BB_3DD_SYNC_FRAME_PERIOD_FRACTION_REG,
                                (clk_id << 8) |
                                 bt_3dd_var.ext_clk_fraction);

    clk = bt_3dd_driver_get_sync_native_clock(offset);

    mod = clk & 0x03;
    new_clk = clk & ~0x03;

    *offset += ((625 * mod) >> 1); /* plus (312.5 usec x mod) */

    return new_clk;
}

/**************************************************************************
 * Function     : bt_csb_sync_train_timeout_callback
 *
 * Description  : This function is a callback function of the one shot software
 *                timer for synchronization train transmit timeout. It shall
 *                stop the synchronization train transmit then send hci
 *                syncrhonization train complete event to hci host.
 *
 * Parameters   : timer_handle - the handle of software timer
 *                index - argument
 *
 * Returns      : None
 *
 *************************************************************************/
void bt_csb_sync_train_timeout_callback(TimerHandle_t timer_handle)
{
    if (timer_handle == NULL)
    {
        return;
    }

	DEF_CRITICAL_SECTION_STORAGE;
    MINT_OS_ENTER_CRITICAL();

    /* delete timer in invalid argument */
    if ((!bt_3dd_var.stp_tx_param.enable) ||
            (stp_timer != timer_handle))
    {
        OS_DELETE_TIMER(&timer_handle);
        MINT_OS_EXIT_CRITICAL();
        return;
    }

    /* kill the sync train */
    bt_csb_driver_stop_sync_train();

    MINT_OS_EXIT_CRITICAL();

    /* generate Synchronization Train Complete event */
    hci_generate_csa4_sychronization_train_complete_event(HCI_COMMAND_SUCCEEDED);
}

/**************************************************************************
 * Function     : bt_csb_control_sync_train_timer
 *
 * Description  : This function is used to enable or disable the software timer
 *                of synchronization train timeout.
 *
 * Parameters   : enable - TRUE is enable and FALSE is disable
 *
 * Returns      : None
 *
 *************************************************************************/
void bt_csb_control_sync_train_timer(UINT8 enable)
{
    if (!bt_3dd_var.stp_tx_param.enable)
    {
        return;
    }

    if (stp_timer != NULL)
    {
        /* stop the timer if we have the body of supervision timer */
        OS_DELETE_TIMER(&stp_timer);
    }

    if (enable)
    {
        /* create a supervision timer and install the callback function */
        OS_CREATE_TIMER(ONESHOT_TIMER, &stp_timer,
                bt_csb_sync_train_timeout_callback, NULL, 0);

        OS_START_TIMER(stp_timer,
                        SLOT_VAL_TO_TIMER_VAL(bt_3dd_var.stp_tx_param.timeout));
    }
}

/**************************************************************************
 * Function     : bt_csb_driver_start_sync_train
 *
 * Description  : This function is used to start the synchronization train
 *                transmit. It can set corresponding hw register and initate
 *                the software timer for synchronization train transmit timeout.
 *
 * Parameters   : None
 *
 * Returns      : None
 *
 *************************************************************************/
void bt_csb_driver_start_sync_train(void)
{
    UINT8 lt_addr = bt_3dd_var.csb_tx_param.lt_addr;

    /* set service data field and reserved lt address field */
    {
        BZ_REG_CON_3DD reg = bz_reg_con_3dd_read();
        if (bt_3dd_var.is_csb_mode)
        {
            reg.service_data = bt_3dd_var.stp_tx_param.service_data;
        }
        else
        {
            /* RP mode: set 3DD version to 1 */
            reg.service_data = 0x01;
        }
        reg.rsvd_am_addr = lt_addr;
        reg.max_random_delay = BT_CSB_SYNC_TRAIN_MAX_RANDOM_DELAY;
        bz_reg_con_3dd_write(reg);
    }

    /* set tsync train period */
    BB_write_baseband_register(BB_3DD_SYNC_ITV_REG,
                                (bt_3dd_var.stp_tx_param.interval >> 1));
#ifdef _DAPE_TEST_SYNC_TRAIN_FORMAT2
    /* select native clock (bit[10:8]) be default capture clock type */
    BB_write_baseband_register(BB_3DD_SYNC_FRAME_PERIOD_FRACTION_REG,
                                (0 << 8) | (BIT11) |
                                 bt_3dd_var.ext_clk_fraction);
RT_BT_LOG(GREEN, DAPE_TEST_LOG213, 2, BB_3DD_SYNC_FRAME_PERIOD_FRACTION_REG,
    BB_read_baseband_register(BB_3DD_SYNC_FRAME_PERIOD_FRACTION_REG));
#endif

#ifdef _DAPE_TEST_CHK_CSB_SYNC_TX
RT_BT_LOG(WHITE, DAPE_TEST_LOG522, 2,
              lt_addr, bt_3dd_var.csb_tx_param.piconet_id);
#endif
    /* set afh channel map */
    BB_write_afh_map(lt_addr, bt_3dd_var.csb_tx_param.piconet_id,
                                        bt_3dd_var.stp_tx_param.ch_map);

    /* start the sync train */
    BB_write_baseband_register(INSTRUCTION_REGISTER, BB_START_SYNC_3DD);

    /* enable sync train sw flag */
    bt_3dd_var.stp_tx_param.enable = TRUE;

    /* start the sync train timer */
    bt_csb_control_sync_train_timer(TRUE);

    CSB_LOG(GREEN, LOG_LEVEL_HIGH, MSG_CSB_STP_TX_ENABLE, 14,
                            lt_addr,
                            bt_3dd_var.stp_tx_param.service_data,
                            bt_3dd_var.stp_tx_param.ch_map[0],
                            bt_3dd_var.stp_tx_param.ch_map[1],
                            bt_3dd_var.stp_tx_param.ch_map[2],
                            bt_3dd_var.stp_tx_param.ch_map[3],
                            bt_3dd_var.stp_tx_param.ch_map[4],
                            bt_3dd_var.stp_tx_param.ch_map[5],
                            bt_3dd_var.stp_tx_param.ch_map[6],
                            bt_3dd_var.stp_tx_param.ch_map[7],
                            bt_3dd_var.stp_tx_param.ch_map[8],
                            bt_3dd_var.stp_tx_param.ch_map[9],
                            bt_3dd_var.stp_tx_param.interval,
                            bt_3dd_var.stp_tx_param.timeout);
}

/**************************************************************************
 * Function     : bt_csb_driver_stop_sync_train
 *
 * Description  : This function is used to stop the synchronization train
 *                transmit. It can set corresponding hw register and kill
 *                the software timer for synchronization train transmit timeout.
 *
 * Parameters   : None
 *
 * Returns      : None
 *
 *************************************************************************/
void bt_csb_driver_stop_sync_train(void)
{
    /* kill the sync train sw timer */
    bt_csb_control_sync_train_timer(FALSE);

    /* kill the sync train */
    BB_write_baseband_register(INSTRUCTION_REGISTER, BB_KILL_SYNC_3DD);

    /* disable sync train sw flag */
    bt_3dd_var.stp_tx_param.enable = FALSE;

    CSB_LOG(GREEN, LOG_LEVEL_HIGH, MSG_CSB_STP_TX_DISABLE, 0, 0);
}

/**************************************************************************
 * Function     : bt_csb_tx_supervision_callback
 *
 * Description  : This function is a callback function of the one shot software
 *                timer for CSB transmit supervision timeout. It shall
 *                stop the connectionless slave broadcast transmit then send hci
 *                connectionless slave broadcast timeout event to hci host.
 *
 * Parameters   : timer_handle - the handle of software timer
 *                index - argument
 *
 * Returns      : None
 *
 *************************************************************************/
void bt_csb_tx_supervision_callback(TimerHandle_t timer_handle)
{
    if (timer_handle == NULL)
    {
        return;
    }

	DEF_CRITICAL_SECTION_STORAGE;
    MINT_OS_ENTER_CRITICAL();

    /* delete timer in invalid argument */
    if ((!bt_3dd_var.csb_tx_param.enable) ||
                (csb_trx_supervision_timer != timer_handle))
    {
        OS_DELETE_TIMER(&timer_handle);
        MINT_OS_EXIT_CRITICAL();
        return;
    }

    /* kill the broadcast slave data */
    bt_csb_driver_stop_transmit_beacon();

    MINT_OS_EXIT_CRITICAL();

    /* generate Connectionless Slave Broadcast Timeout event */
    hci_generate_csa4_connectionless_slave_broadcast_timeout_event(0);
}

/**************************************************************************
 * Function     : bt_csb_control_supervision_timer
 *
 * Description  : This function is used to enable or disable the software timer
 *                of connectionless slave broadcast transmit supervision.
 *
 * Parameters   : enable - TRUE is enable and FALSE is disable
 *
 * Returns      : None
 *
 *************************************************************************/
void bt_csb_control_supervision_timer(UINT8 enable)
{
    if (!bt_3dd_var.csb_tx_param.enable)
    {
        return;
    }

    if (csb_trx_supervision_timer != NULL)
    {
        /* stop the timer if we have the body of supervision timer */
        OS_DELETE_TIMER(&csb_trx_supervision_timer);
    }

    if (enable)
    {
        /* create a supervision timer and install the callback function */
        OS_CREATE_TIMER(ONESHOT_TIMER, &csb_trx_supervision_timer,
                bt_csb_tx_supervision_callback, NULL, 0);

        OS_START_TIMER(csb_trx_supervision_timer,
            SLOT_VAL_TO_TIMER_VAL(bt_3dd_var.csb_tx_param.supervision_timeout));
    }
}

/**************************************************************************
 * Function     : bt_csb_decide_broadcast_packet_type
 *
 * Description  : This function is used to decide the BB packet type of CSB
 *                data. It is based on the supported packet type and CSB
 *                data length to select one less slot number and non-fragment
 *                packet type.
 *
 * Parameters   : length - the total data length from hci command packets
 *                pkt_type_lut - the packet type field of LUT register
 *                pkt_type_table - the packet type table setting
 *
 * Returns      : TRUE or FALSE (FALSE means overflow)
 *
 *************************************************************************/
UINT8 bt_csb_decide_broadcast_packet_type(UINT16 *length,
                                                     UINT16 *pkt_type_lut,
                                                     UINT8 *pkt_type_table)
{
    UINT8 i;
    UINT16 type;
    UINT8 bit_num = 3;
    UINT8 ptt = 0;

    type = bt_3dd_var.csb_tx_param.support_packet_type_bm;

    /* search packet type from small to large packet size */
    for (i = 0; i < 12; i++)
    {
        if (type & (1 << g_host_hci_packet_type_bitnum_sort_by_maxlen[i]))
        {
            bit_num = g_host_hci_packet_type_bitnum_sort_by_maxlen[i];
            ptt = g_host_hci_packet_type_ptt_sort_by_maxlen[i];

            if (*length <= g_lc_pkt_type_max_len[bit_num])
            {
                *pkt_type_lut = g_lc_pkt_type_lut[bit_num];
                *pkt_type_table = ptt;
                return FALSE;
            }
        }
    }

    /* warnning : pending packet length is larger than valid packet type!! */
    *length = g_lc_pkt_type_max_len[bit_num];
    *pkt_type_lut = g_lc_pkt_type_lut[bit_num];
    *pkt_type_table = ptt;
    return TRUE;
}

/**************************************************************************
 * Function     : bt_csb_driver_prepare_beacon_data
 *
 * Description  : This function is used to prepare the data packet of
 *                connectionless slave broadcast transmit. For RP mode,
 *                fw only fetch some fields from hci commands to fill some
 *                registers in old design. In new design. In new design,
 *                the payload body is come from the host and fw can schedule it
 *                via hci dma.
 *
 * Parameters   : None
 *
 * Returns      : BT_ERROR_OK or BT_ERROR
 *
 *************************************************************************/

UINT8 bt_csb_driver_prepare_beacon_data(void)
{
    BT_CSA4_BEACON_TX_UINT_S *csb = &bt_3dd_var.csb_tx_param;
    BZDMA_TX_DESC_ENTRY_STATUS *pTxEnt;
    UINT8 lt_addr = bt_3dd_var.csb_tx_param.lt_addr;
    UINT16 len = csb->data_len;
    UINT16 pkt_type_lut;
    UINT8 ptt = 0;
    LUT_EXTENSION_TABLE *ex_lut = &lut_ex_table[lt_addr];
    UINT16 lut_contents;
    HCI_ACL_DATA_PKT *ppkt = (HCI_ACL_DATA_PKT *) csb->pData;
    UINT8 status = BT_ERROR;

    do
    {
        if (len > 0)
        {
            if (ppkt == NULL)
            {
                break;
            }

            if (csb->fragment < 2)
            {
                /* only start or continuous fragment */
                break;
            }

#ifndef _DO_NOT_SUPPORT_RP_
            if (!bt_3dd_var.is_csb_mode)
            {
                status = BT_ERROR_OK;
                break;
            }
#endif

            if (bt_csb_decide_broadcast_packet_type(&len, &pkt_type_lut, &ptt))
            {
                // TODO: Log Something
                break;
            }

            if (!(Bzdma_Manager.bmFreeTxEnt & BIT(BZDMA_TX_ENTRY_TYPE_CSB)))
            {
                /* invalid bzdma tx command */
                bzdma_invalid_txcmd(BZDMA_TX_ENTRY_TYPE_CSB,
                        bt_3dd_var.csb_tx_param.lt_addr, 0);
            }

            bzdma_get_dedicated_free_tx_entry(BZDMA_TX_ENTRY_TYPE_CSB);

            pTxEnt = &Bzdma_Manager.TxEntSta[BZDMA_TX_ENTRY_TYPE_CSB];

            /* init tx descriptors */
            pTxEnt->pTxDesc[0].DWord0 = 0;
            pTxEnt->pTxDesc[0].DWord1 = 0;
            pTxEnt->pTxDesc[0].len = len;
            pTxEnt->pTxDesc[0].start_addr = (UINT32) ppkt->hci_acl_data_pkt;

            /* set last flag */
            pTxEnt->pTxDesc[0].isLast = TRUE;

            /* send tx command */
            bzdma_send_txcmd(BZDMA_TX_ENTRY_TYPE_CSB);

            AND_val_with_bb_reg_macro(ex_lut->upper_lut_address,
                    ~(BIT(BB_L2CAP_FLOW_BIT_POS)));

            if (ptt)
            {
                LC_SET_PTT_BIT_IN_LUT(ex_lut->upper_lut_address);
            }
            else
            {
                LC_RESET_PTT_BIT_IN_LUT(ex_lut->upper_lut_address);
            }

            lut_contents = BB_PBD_PKT | len | pkt_type_lut;
            BB_write_baseband_register(ex_lut->lower_lut_address, lut_contents);
            csb->select_tx_packet_type = pkt_type_lut >> 12;

            CSB_LOG(GREEN, LOG_LEVEL_HIGH, MSG_CSB_TX_NON_NULL_PKT, 7,
                    csb->select_tx_packet_type, len, ppkt,
                    ppkt->hci_acl_data_pkt[0], ppkt->hci_acl_data_pkt[1],
                    ppkt->hci_acl_data_pkt[2], ppkt->hci_acl_data_pkt[3]);

            return BT_ERROR_OK;
        }
    } while (0);

#ifndef _DO_NOT_SUPPORT_RP_
    if ((!bt_3dd_var.is_csb_mode) && (status == BT_ERROR_OK))
    {
        /* it is RP mode now */
        bt_3dd_var.beacon.dual_video_stream_mode =
                (ppkt->hci_acl_data_pkt[3] >> 6) & 0x01;
        bt_3dd_var.beacon.lsod = *(UINT16*) &ppkt->hci_acl_data_pkt[6];
        bt_3dd_var.beacon.lscd = *(UINT16*) &ppkt->hci_acl_data_pkt[8];
        bt_3dd_var.beacon.rsod = *(UINT16*) &ppkt->hci_acl_data_pkt[10];
        bt_3dd_var.beacon.rscd = *(UINT16*) &ppkt->hci_acl_data_pkt[12];

        bt_3dd_driver_update_transmit_beacon_parameters();
#ifdef _NEW_3DD_HW_
        bt_3dd_driver_set_ext_frame_sync_period();
#endif
        csb->select_tx_packet_type = BB_DM1;

        return BT_ERROR_OK;
    }
#endif

    /* if csb data is zero length or the combined segments are not ready from
     host, we can send NULL packet for CSB tx data */

    /* set NULL packet */
    BB_write_baseband_register(ex_lut->lower_lut_address,
            LC_SLAVE_DEFAULT_PACKET_TYPE);

    /* invalid bzdma tx command */
    bzdma_invalid_txcmd(BZDMA_TX_ENTRY_TYPE_CSB,
            bt_3dd_var.csb_tx_param.lt_addr, 0);

    csb->select_tx_packet_type = BB_NULL;

    if (len == 0)
    {
        if (ppkt != NULL)
        {
            /* free unused pending buffer */
            if (OS_FREE_BUFFER(tx_table[ACL_DATA_HANDLER_TASK].pool_handle,
                    ppkt) == BT_ERROR_OK)
            {
                os_free_reserved_buffer();
                ppkt = NULL;
            }
        }
        status = BT_ERROR_OK;
    }

    CSB_LOG(GREEN, LOG_LEVEL_HIGH, MSG_CSB_TX_NULL_PKT, 1, status);

    return status;
}

/**************************************************************************
 * Function     : bt_csb_driver_start_transmit_beacon
 *
 * Description  : This function is used to start the connectionless slave
 *                broadcast transmit. It can set corresponding hw register and
 *                initate the CSB trasmit supervision timer.
 *
 * Parameters   : None
 *
 * Returns      : None
 *
 *************************************************************************/
void bt_csb_driver_start_transmit_beacon(void)
{
    /* set beacon interval */
    BB_write_baseband_register(BB_3DD_BCN_ITV_REG,
                                (bt_3dd_var.csb_tx_param.interval >> 1));

    /* prepare beacon data */
    bt_csb_driver_prepare_beacon_data();

    /* start the beacon (linkless broadcast) */
    BB_write_baseband_register(INSTRUCTION_REGISTER, BB_START_BEACON_3DD);

    /* enable broadcast sw flag */
    bt_3dd_var.csb_tx_param.enable = TRUE;

    /* start the beacon tx supervision timer */
    bt_csb_control_supervision_timer(TRUE);

    CSB_LOG(GREEN, LOG_LEVEL_HIGH, MSG_CSB_BEACON_TX_ENABLE, 2,
                                  bt_3dd_var.csb_tx_param.interval,
                                  bt_3dd_var.csb_tx_param.supervision_timeout);
}

/**************************************************************************
 * Function     : bt_csb_driver_stop_transmit_beacon
 *
 * Description  : This function is used to stop the connectionless slave
 *                broadcast transmit. It can set corresponding hw register and
 *                kill the CSB trasmit supervision timer.
 *
 * Parameters   : None
 *
 * Returns      : None
 *
 *************************************************************************/
void bt_csb_driver_stop_transmit_beacon(void)
{
    /* kill the supervision sw timer */
    bt_csb_control_supervision_timer(FALSE);

    /* kill the beacon */
    BB_write_baseband_register(INSTRUCTION_REGISTER, BB_KILL_BEACON_3DD);

    /* release corresponding bzdma tx command */
    bzdma_invalid_txcmd(BZDMA_TX_ENTRY_TYPE_CSB,
                        bt_3dd_var.csb_tx_param.lt_addr, 0);

    /* disble broadcast sw flag */
    bt_3dd_var.csb_tx_param.enable = FALSE;

    CSB_LOG(GREEN, LOG_LEVEL_HIGH, MSG_CSB_BEACON_TX_DISABLE, 0, 0);
}

/**************************************************************************
 * Function     : bt_csb_driver_handle_ext_clock_capture_by_edge_trigger
 *
 * Description  : This function is used to handle the capture event and
 *                external clock capture spec of CSA4 at the  rising edge of
 *                the external frame sync or clock. For RP application,
 *                FW must calculate the frame sync period and frame sync
 *                fraction to fill registers in the new design, too.
 *
 * Parameters   : None
 *
 * Returns      : None
 *
 *************************************************************************/
const UINT8 c_ext_clk_edge_counts_length_table[4] = {0xf, 0x1f, 0x3f, 0x7f};
const UINT8 c_ext_clk_edge_counts_period_table[4] = {16, 32, 64, 128};
const UINT8 c_ext_clk_edge_counts_avg_table[4] = {4, 5, 6, 7};

/* (austin) Note : this function is in ISR and noisy triggered .
   Do not print too much log function in ASIC */
void bt_csb_driver_handle_ext_clock_capture_by_edge_trigger(void)
{
#ifdef _ROM_CODE_PATCHED_
    if (rcp_bt_3dd_driver_handle_ext_clock_capture_by_edge_trigger != NULL)
    {
        rcp_bt_3dd_driver_handle_ext_clock_capture_by_edge_trigger();
        return;
    }
#endif

    //20131224 morgan add for calculation detection
    //d3dd_cnt ++;

    // 20140117 : add a state machine
    UINT32 clk_diff;
    INT16 clk_us_diff;
    INT32 clk_sum_us;

    UINT32 cur_clk;
    UINT16 cur_clk_us;
    INT32 period_diff = 0;

    if (state_period_detection == INITIAL_PERIOD_STATE)
    {
        bt_3dd_var.ext_clk_edge_counts = 0;
        state_period_detection = CALCULATE_PERIOD_STATE;
        ext_clk_edge_counts_length = 0xf;
        ext_clk_edge_counts_period = 16;
        ext_clk_edge_counts_avg = 4;
        RT_BT_LOG(YELLOW, D3D_MSG_24, 0, 0);
        return;
    }

    //========================================================
    if ((bt_3dd_var.ext_clk_edge_counts & ext_clk_edge_counts_length) == 0) //0x0f // use 128
    {
        //============
        // obtain current clk
        //============
        cur_clk = bt_3dd_driver_get_sync_native_clock(&cur_clk_us);
        cur_clk_us += ((625 * (cur_clk & 0x03)) >> 1);
        cur_clk &= ~0x03;

        if (bt_3dd_var.ext_clk_edge_counts == ext_clk_edge_counts_period) //efuse default:32
        {
            /* calculate average frame sync period and offset */
            clk_diff = lc_get_clock_diff(cur_clk, bt_3dd_var.bbclk, 0);
            clk_diff >>= 2; /* bit [1:0] are zero */
            clk_us_diff = (INT16) (cur_clk_us - bt_3dd_var.bbclk_us);
            clk_sum_us = (1250 * clk_diff) + clk_us_diff;
            bt_3dd_var.csb_tx_param.ext_frame_period_us = clk_sum_us >> ext_clk_edge_counts_avg; //>>4

            if (flag_toggle_restart)
            {
#ifdef _ENABLE_LOG_OF_3DD_FOR_TV_APP_
                RT_BT_LOG(YELLOW,D3D_MSG_11,1,long_term_avg_period);
#endif
                bt_3dd_var.csb_tx_param.ext_frame_period_us = long_term_avg_period;
                ext_frame_period_us_q = bt_3dd_var.csb_tx_param.ext_frame_period_us;
                long_term_state = 0;
                flag_toggle_restart = 0;
                flag_rejection_end = 1;
                // modify tracking gain
                cur_coeff_gain = 1;
                cur_coeff_q_gain = 1;
                cur_coeff_avg = 2;
                // initialize rejection time calculation coeff
                rejection_cnt = 0;
                init_ext_frame_period_us = 0;
                period_significant_diff_cnt = 0;
                period_significant_diff_long_term_cnt = 0;
            } //20130113 add

            // 20140108 morgan add for rejection time and initial period calculation
            if (flag_rejection_end)
            {
                // rejection end point : calculate the initial period value
                // reset tracking coeff
                // reset calculation window
                // modify tracking gain
                if (rejection_cnt >= efuse_tv_3dd->fast_recover_times)
                {
                    flag_rejection_end = 0;
                    cur_coeff_gain = efuse_tv_3dd->coeff_gain;
                    cur_coeff_avg = efuse_tv_3dd->coeff_avg;
                    if (cur_coeff_avg > cur_coeff_gain)
                    {
                        cur_coeff_q_gain = cur_coeff_avg - cur_coeff_gain;
                    }
                    else
                    {
                        cur_coeff_gain = 1;
                        cur_coeff_q_gain = 3;
                        cur_coeff_avg = 4;
                    }
                    init_ext_frame_period_us = init_ext_frame_period_us / (efuse_tv_3dd->fast_recover_times - efuse_tv_3dd->rejection_times);
                    bt_3dd_var.csb_tx_param.ext_frame_period_us = init_ext_frame_period_us;
                    ext_frame_period_us_q = init_ext_frame_period_us;
                    RT_BT_LOG(BLUE, D3D_MSG_17, 3, rejection_cnt, bt_3dd_var.csb_tx_param.ext_frame_period_us, long_term_avg_period);
                    rejection_cnt = 0;

                    UINT8 calc_win = efuse_tv_3dd->calculation_window;
                    ext_clk_edge_counts_length = c_ext_clk_edge_counts_length_table[calc_win];
                    ext_clk_edge_counts_period = c_ext_clk_edge_counts_period_table[calc_win];
                    ext_clk_edge_counts_avg = c_ext_clk_edge_counts_avg_table[calc_win];
                }
                // ignore the first period value ( it's long term period )
                if ((rejection_cnt >= efuse_tv_3dd->rejection_times) && (rejection_cnt < efuse_tv_3dd->fast_recover_times))
                {
                    // calculate average period
                    init_ext_frame_period_us += bt_3dd_var.csb_tx_param.ext_frame_period_us;
                    //RT_BT_LOG(YELLOW,D3D_MSG_18,2,rejection_cnt,init_ext_frame_period_us);
                }
                rejection_cnt++;
            }
            //20140108 morgan add for check period has a siginficant diff with long term value
            // 20140217 morgan add second compare condition to satisfy
            // bt_3dd_var.csb_tx_param.ext_frame_period_us < long_term_avg_period condition
            if (((bt_3dd_var.csb_tx_param.ext_frame_period_us - long_term_avg_period) > (efuse_tv_3dd->period_significant_diff * 2))
                    || ((long_term_avg_period - bt_3dd_var.csb_tx_param.ext_frame_period_us) > (efuse_tv_3dd->period_significant_diff * 2)))
            {
                if (period_significant_diff_cnt <= 1)
                {
                    RT_BT_LOG(YELLOW, D3D_MSG_19, 2, bt_3dd_var.csb_tx_param.ext_frame_period_us, period_significant_diff_cnt);
                    bt_3dd_var.csb_tx_param.ext_frame_period_us = long_term_avg_period;
                }
                else
                {
                    flag_toggle_restart = 1;
                    long_term_avg_period = bt_3dd_var.csb_tx_param.ext_frame_period_us; // if initial mode is wrong
#ifdef _3DD_ONLY_SUPPORT
                    if (efuse_3dd_only)
                    {
                        fn_set_shutter_delay();
                    }
                    else // host mode need to update lsod lscd rsod rscd here
#endif
                    {
                        fn_host_set_shutter_delay(bt_3dd_shutter_lsod, bt_3dd_shutter_lscd, bt_3dd_shutter_rsod, bt_3dd_shutter_rscd);
                    }
                    // bt_3dd_driver_prepare_beacon_data();
                    bt_3dd_driver_update_transmit_beacon_parameters();
                    RT_BT_LOG(YELLOW, D3D_MSG_20, 2, bt_3dd_var.csb_tx_param.ext_frame_period_us, long_term_avg_period);
                    state_period_detection = 0;
                    period_significant_diff_cnt = 0;
                    return; //20140214 add
                }
                period_significant_diff_cnt++;
            }
            //20140108 morgan add for check period has slight diff with long term value
            // it is active when entering long term state
#ifdef _ENABLE_LOG_OF_3DD_FOR_TV_APP_
            if (efuse_tv_3dd->enable_3dd_log)
            {
                RT_BT_LOG(GREEN,DD_MSG_CMD12,3,bt_3dd_var.ext_clk_edge_counts,bt_3dd_var.csb_tx_param.ext_frame_period_us,long_term_avg_period);
            }
#endif
            if (long_term_state)
            {
                period_diff = bt_3dd_var.csb_tx_param.ext_frame_period_us - long_term_avg_period;
                if (period_diff < 0)
                {
                    period_diff = long_term_avg_period - bt_3dd_var.csb_tx_param.ext_frame_period_us;
                }
                if (period_diff > (efuse_tv_3dd->period_significant_diff_long_term * 2)) // ignore the detection // period_diff > 300 => restart
                {
                    //RT_BT_LOG(GREEN,D3D_MSG_19,1,bt_3dd_var.csb_tx_param.ext_frame_period_us);

                    if (period_significant_diff_long_term_cnt <= 1)
                    {
                        RT_BT_LOG(BLUE, D3D_MSG_19, 2, bt_3dd_var.csb_tx_param.ext_frame_period_us, period_significant_diff_long_term_cnt);
                        bt_3dd_var.csb_tx_param.ext_frame_period_us = long_term_avg_period;
                    }
                    else
                    {
                        flag_toggle_restart = 1;
                        long_term_avg_period = bt_3dd_var.csb_tx_param.ext_frame_period_us; // if initial mode is wrong
#ifdef _3DD_ONLY_SUPPORT
                        if (efuse_3dd_only)
                        {
                            fn_set_shutter_delay();
                        }
                        else // host mode need to update lsod lscd rsod rscd here
#endif
                        {
                            fn_host_set_shutter_delay(bt_3dd_shutter_lsod, bt_3dd_shutter_lscd, bt_3dd_shutter_rsod, bt_3dd_shutter_rscd);
                        }
                        //bt_3dd_driver_prepare_beacon_data();
                        bt_3dd_driver_update_transmit_beacon_parameters();
                        RT_BT_LOG(BLUE, D3D_MSG_20, 2, bt_3dd_var.csb_tx_param.ext_frame_period_us, long_term_avg_period);
                        state_period_detection = 0;
                        period_significant_diff_long_term_cnt = 0;
                        return; //20140214 add
                    }
                    period_significant_diff_long_term_cnt++;
                }
            }
            // calucalute period
            bt_3dd_var.csb_tx_param.ext_frame_period_us = ((bt_3dd_var.csb_tx_param.ext_frame_period_us * cur_coeff_gain) + (ext_frame_period_us_q * cur_coeff_q_gain)) / cur_coeff_avg;
            ext_frame_period_us_q = bt_3dd_var.csb_tx_param.ext_frame_period_us;

            // here calculate long term period and period mode
            avg_cnt++;
            if (avg_cnt >= long_term_period_window)
            {
                avg_cnt = 0;
                period_significant_diff_cnt = 0;
                period_significant_diff_long_term_cnt = 0;
                long_term_state = 1;
                long_term_avg_period = bt_3dd_var.csb_tx_param.ext_frame_period_us;
#ifdef _3DD_ONLY_SUPPORT
                if (efuse_3dd_only)
                {
                    fn_set_shutter_delay();
                }
                else // host mode need to update lsod lscd rsod rscd here
#endif
                {
                    fn_host_set_shutter_delay(bt_3dd_shutter_lsod, bt_3dd_shutter_lscd, bt_3dd_shutter_rsod, bt_3dd_shutter_rscd);
                }
                //bt_3dd_driver_prepare_beacon_data();
                bt_3dd_driver_update_transmit_beacon_parameters();
                /* note period mode */

                period_diff = BT_EXT_FRAME_PERIOD_60HZ_US_DEFAULT - long_term_avg_period;
#ifdef _ENABLE_LOG_OF_3DD_FOR_TV_APP_
                //RT_BT_LOG(BLUE,D3D_MSG_14,1,period_diff);
#endif
                if (period_diff < 0)
                {
                    period_diff = long_term_avg_period - BT_EXT_FRAME_PERIOD_60HZ_US_DEFAULT;
                }
#ifdef _ENABLE_LOG_OF_3DD_FOR_TV_APP_
                //RT_BT_LOG(BLUE,D3D_MSG_14,1,period_diff);
#endif
                if (period_diff < (efuse_tv_3dd->period_diff_limit * 4))
                {
                    period_mode = 0;
                }
                // check mode 1
                period_diff = BT_EXT_FRAME_PERIOD_120HZ_US_DEFAULT - long_term_avg_period;
#ifdef _ENABLE_LOG_OF_3DD_FOR_TV_APP_
                //RT_BT_LOG(BLUE,D3D_MSG_14,1,period_diff);
#endif
                if (period_diff < 0)
                {
                    period_diff = long_term_avg_period - BT_EXT_FRAME_PERIOD_120HZ_US_DEFAULT;
                }
#ifdef _ENABLE_LOG_OF_3DD_FOR_TV_APP_
                //RT_BT_LOG(BLUE,D3D_MSG_14,1,period_diff);
#endif
                if (period_diff < (efuse_tv_3dd->period_diff_limit * 4))
                {
                    period_mode = 1;
                }
                // check mode 2
                period_diff = BT_EXT_FRAME_PERIOD_50HZ_US_DEFAULT - long_term_avg_period;
#ifdef _ENABLE_LOG_OF_3DD_FOR_TV_APP_
                //RT_BT_LOG(BLUE,D3D_MSG_14,1,period_diff);
#endif
                if (period_diff < 0)
                {
                    period_diff = long_term_avg_period - BT_EXT_FRAME_PERIOD_50HZ_US_DEFAULT;
                }
#ifdef _ENABLE_LOG_OF_3DD_FOR_TV_APP_
                //RT_BT_LOG(BLUE,D3D_MSG_14,1,period_diff);
#endif
                if (period_diff < (efuse_tv_3dd->period_diff_limit * 4))
                {
                    period_mode = 2;
                }
#ifdef _ENABLE_LOG_OF_3DD_FOR_TV_APP_
                RT_BT_LOG(BLUE,D3D_MSG_13,2,long_term_avg_period,period_mode);
#endif

            }
            // if ( long_term_state && (period_diff > 1) )
            // 20140421 morgan modify :
            // @@@@ need to rewrite :  bt_3dd_var.csb_tx_param.ext_frame_period_us
            // @@@@  t_3dd_var.ext_clk_period
            //bt_3dd_driver_write_period_to_reg(bt_3dd_var.csb_tx_param.ext_frame_period_us);
            bt_3dd_var.ext_clk_period = bt_3dd_var.csb_tx_param.ext_frame_period_us;
            bt_3dd_var.ext_clk_fraction = 170;
            bt_3dd_driver_set_ext_frame_sync_period();
#ifdef _ENABLE_LOG_OF_3DD_FOR_TV_APP_
            if (efuse_tv_3dd->enable_3dd_log)
            {
                RT_BT_LOG(GREEN,DD_MSG_CMD5,2,bt_3dd_var.ext_clk_edge_counts,bt_3dd_var.csb_tx_param.ext_frame_period_us);
            }
#endif
            bt_3dd_var.ext_clk_edge_counts = 0;
        }
        bt_3dd_var.bbclk = cur_clk;
        bt_3dd_var.bbclk_us = cur_clk_us;
    }
    bt_3dd_var.ext_clk_edge_counts++;

    // TODO: Node B here
}

/**************************************************************************
 * Function     : bt_csb_driver_handle_ext_clock_capture_by_edge_trigger
 *
 * Description  : This function is used to handle the capture event and
 *                external clock capture spec of CSA4 at the  rising edge of
 *                the external frame sync or clock. For RP application,
 *                FW must calculate the frame sync period and frame sync
 *                fraction to fill registers in the new design, too.
 *
 * Parameters   : None
 *
 * Returns      : None
 *
 *************************************************************************/
#if 0
void bt_csb_driver_handle_ext_clock_capture_by_edge_trigger(void)
{
    BT_CSA4_CLOCK_CAPTURE_UNIT_S *pclk_cap;
    UINT32 cur_clk;
    UINT16 cur_clk_us;

    pclk_cap = (BT_CSA4_CLOCK_CAPTURE_UNIT_S *)&bt_3dd_var.clk_cap;

    if (pclk_cap->enable)
    {
        pclk_cap->cur_ext_clk_captures++;

        /* when satisfy number of clocks for capture, we shall send event to
           host */
        if ((pclk_cap->num_ext_clk_captures == 0) ||
            (pclk_cap->num_ext_clk_captures ==
                            pclk_cap->cur_ext_clk_captures))
        {
            cur_clk = bt_csb_driver_get_ext_frame_sync_clock(
                                    !bt_3dd_var.use_piconet_clk,
                                    pclk_cap->piconet_id,
                                    &cur_clk_us);

            pclk_cap->pre_piconet_clk = cur_clk;
            pclk_cap->pre_piconet_clk_us = cur_clk_us;
            hci_generate_csa4_triggered_clock_capture_event();
            pclk_cap->cur_ext_clk_captures = 0;
        }
    }

#ifndef _DO_NOT_SUPPORT_RP_
    if (bt_3dd_var.is_csb_mode)
    {
        return;
    }

    /* calculate the duration and average every 16 ticks of external
       frame sync */
    if ((bt_3dd_var.ext_clk_edge_counts & 0x0f) == 0)
    {
        cur_clk = bt_csb_driver_get_ext_frame_sync_clock(0, 0, &cur_clk_us);

        if (bt_3dd_var.ext_clk_edge_counts == 16)
        {
            UINT32 clk_diff;
            INT16 clk_us_diff;
            INT32 clk_sum_us;

            clk_diff = lc_get_clock_diff(cur_clk, bt_3dd_var.bbclk, 0);
            clk_diff >>= 2; /* bit [1:0] are zero */
            clk_us_diff = (INT16)(cur_clk_us - bt_3dd_var.bbclk_us);
            clk_sum_us = (1250 * clk_diff) + clk_us_diff;

            /* calculate the average of clk_sum_us (clk_sum_us / 16) */
            bt_3dd_var.ext_clk_period = clk_sum_us >> 4;

            /* calculate the mod of clk_sum_us (clk_sum_us % 16),
               but we need to get 1/256 us in unit,
               so the value become ((mod/16) * 256) = (mod * 16) */
            bt_3dd_var.ext_clk_fraction = (clk_sum_us & 0x0F) << 4;

#ifdef _NEW_3DD_HW_
            bt_3dd_driver_set_ext_frame_sync_period();
#endif
            bt_3dd_var.ext_clk_edge_counts = 0;
        }

        bt_3dd_var.bbclk = cur_clk;
        bt_3dd_var.bbclk_us = cur_clk_us;
    }

    bt_3dd_var.ext_clk_edge_counts++;
#endif
}
#endif
/**************************************************************************
 * Function     : bt_csb_update_host_afh_ch_classification
 *
 * Description  : This function is used to check and update the afh channel map
 *                when receive hci host afh channel classification command.
 *                When the channel map is updated, fw can send a connectionless
 *                slave broadcast channel map change event to hci host.
 *
 * Parameters   : map - the new afh map
 *
 * Returns      : None
 *
 *************************************************************************/
void bt_csb_update_host_afh_ch_classification(UINT8 *map)
{
    if (memcmp(bt_3dd_var.stp_tx_param.ch_map, map, 10) == 0)
    {
        /* quit now because no different */
        return;
    }

    /* Connectionless Slave Broadcast Channel Map Change event shall also
       be sent if the Host issues a Set_AFH_Channel_Classification command
       which causes the Connectionless Slave Broadcast Channel Map to change.*/

    memcpy(bt_3dd_var.stp_tx_param.ch_map, map, 10);
    hci_generate_csa4_connectionless_slave_broadcast_channel_map_change_event();
}

BOOLEAN bt_csb_is_rp_mode()
{
    if ((lmp_self_device_data.class_of_device != BT3DD_COD_DWORD_I) \
            && (lmp_self_device_data.class_of_device != BT3DD_COD_DWORD_II))
    {
        return FALSE;
    }
    EIR_HEADER *eir_header = eir_header_find(lmp_self_device_data.eir_data.data,
            lmp_self_device_data.eir_data.length, EIR_TYPE_MANUFACTURER_DATA);
    if (eir_header == NULL)
    {
        return FALSE;
    }
    EIR_MANUFACTURER_DATA_HEADER *manu_header = (EIR_MANUFACTURER_DATA_HEADER *) eir_header;
    if (manu_header->head.len != 6
            || EIR_MANUFACTURER_DATA_GET_COMPANY_ID(manu_header) != BROADCOM_COMPANY_ID)
    {
        return FALSE;
    }
    EIR_LEGACY_3D_INFO *legacy_3d_info = (EIR_LEGACY_3D_INFO *) eir_header;
    if (legacy_3d_info->fixed0 != 0)
    {
        return FALSE;
    }
    return TRUE;
}

/**************************************************************************
 * Function     : bt_csb_check_and_set_csb_or_rp_mode
 *
 * Description  : This function is used to check and set current is
 *                CSB or RP mode.
 *
 * Parameters   : None
 *
 * Returns      : None
 *
 *************************************************************************/
void bt_csb_check_and_set_csb_or_rp_mode(void)
{
#ifndef _DO_NOT_SUPPORT_RP_

    UINT8 temp;

    if (!IS_SUPPORT_3DG_APP)
    {
        return;
    }

    temp = bt_3dd_var.is_csb_mode;

    /* compare the EIR pattern of 3DD reference protocol */
    /*
    if ((lmp_self_device_data.eir_data.length == 10) &&
       ((lmp_self_device_data.class_of_device == BT3DD_COD_DWORD_I) ||
       (lmp_self_device_data.class_of_device == BT3DD_COD_DWORD_II)) &&
       (*(UINT32*)lmp_self_device_data.eir_data.data == 0x000FFF06))
    // */
    if (bt_csb_is_rp_mode())
    {
        bt_3dd_var.is_csb_mode = FALSE;
        lmp_self_device_data.inquiry_scan_type = HCI_3DD_INQ_SCAN;
    }
    else
    {
        bt_3dd_var.is_csb_mode = TRUE;
        lmp_self_device_data.inquiry_scan_type = HCI_STANDARD_INQ_SCAN;
    }

#ifdef _NEW_3DD_HW_
    if (temp != bt_3dd_var.is_csb_mode)
    {
        /* select CSB or Reference Protocol mode */
        BB_write_baseband_register(BB_3DD_SYNC_BB_TIME_US_REG,
                  ((!bt_3dd_var.is_csb_mode) << 9) |
                  (bt_3dd_var.stp_tx_param.num_of_next_beacon_inst << 10) |
                  (bt_3dd_var.stp_tx_param.stp2stp_duration << 12));
    }
#endif

    CSB_LOG(GREEN, LOG_LEVEL_HIGH, MSG_CSB_MODE_CHANGE,
                                   1, bt_3dd_var.is_csb_mode);
#endif
}

#endif
#ifdef _SUPPORT_CSB_RECEIVER_
UCHAR hci_validate_truncated_page_cmd_params(HCI_CMD_PKT *hci_cmd_ptr)
{
    UINT16 ce_index ;
    UCHAR index;
    UCHAR temp_bd_addr[LMP_BD_ADDR_SIZE];
    UINT16 temp_page_scan_repetition_mode;
//    UINT16 clock_offset ; (dape : not checked, so not needed)

    memcpy(temp_bd_addr, &hci_cmd_ptr->cmd_parameter[0], LMP_BD_ADDR_SIZE);
    index = LMP_BD_ADDR_SIZE ;

    if(LMP_GET_CE_INDEX_FROM_BD_ADDR(temp_bd_addr, &ce_index) == API_SUCCESS)
    {
        return ACL_CONNECTION_EXISTS_ERROR ;
    }

    if (hci_is_full_bandwidth() == TRUE)
    {
        return HOST_REJECTED_LIMITED_RESOURCES_ERROR;
    }

    /* Page scan repetition mode */
    temp_page_scan_repetition_mode = (UCHAR) hci_cmd_ptr->cmd_parameter[index];
    index++;

    if(temp_page_scan_repetition_mode > 2)
    {
        return INVALID_HCI_COMMAND_PARAMETERS_ERROR;
    }

    /* dape : not checked, so not needed/ */
    /*
    BT_FW_EXTRACT_16_BITS(clock_offset, &(hci_cmd_ptr->cmd_parameter[index]));
    index += 2;
    */
    return HCI_COMMAND_SUCCEEDED;
}
/**************************************************************************
 * Function     : bt_csb_driver_start_sync_scan
 *
 * Description  : This function is used to start the synchronization scan.
 *                It can set corresponding hw register and initate
 *                the software timer for synchronization train scan timeout.
 *
 * Parameters   : None
 *
 * Returns      : None
 *
 *************************************************************************/
void bt_csb_driver_start_sync_scan(void)
{
	UINT32 lap;
	UINT32 uap;
    UCHAR parity_bits[5];

    /* set scan interval */
    BB_write_baseband_register(BB_CSB_RX_SYNC_SCAN_INTV_REG,
                               bt_3dd_var.stp_rx_param.scan_interval);

    /* set scan window */
    BB_write_baseband_register(BB_CSB_RX_SYNC_SCAN_WINDOW_REG,
                               bt_3dd_var.stp_rx_param.scan_window);

    /********************************************************/
    /***  this should be modified once this register  *******/
    /**** has other parameter.                        *******/
    /********************************************************/
    /* set scan channel map */
    BB_write_baseband_register(BB_CSB_RX_SYNC_SCAN_MAP_REG,
                               bt_3dd_var.stp_rx_param.scan_map);

    /********************************************************/
    /********************************************************/
    /********************************************************/

	/* Extract the LAP */
    lap = (bt_3dd_var.stp_rx_param.bd_addr[2] << 16) |
          (bt_3dd_var.stp_rx_param.bd_addr[1] << 8) |
          (bt_3dd_var.stp_rx_param.bd_addr[0]);
    /* Generate parity bits corresponding to lap */
	lc_generate_parity_bits(lap, &parity_bits[0]);
    /* Extract the UAP */
    uap = bt_3dd_var.stp_rx_param.bd_addr[3];

    /* Configure lap, uap registers */
    BB_write_baseband_register(BB_CSB_RX_SYNC_LAP0_REG, lap & 0xFFFF);
    BB_write_baseband_register_lower_octet(BB_CSB_RX_SYNC_LAP1_REG, lap >> 16);
    BB_write_baseband_register_upper_octet(BB_CSB_RX_SYNC_LAP1_REG, uap);

    /* Configure Calculated value of Parity bits */
    BB_write_baseband_register(BB_CSB_RX_SYNC_PARITY0_REG,
            (*((UINT16 *) &parity_bits[0])));
    BB_write_baseband_register(BB_CSB_RX_SYNC_PARITY1_REG,
            (*((UINT16 *) &parity_bits[2])));
#ifdef _CSB_RX_DBG_LOG
    if (g_csb_rx_dbg_log)
    {
        RT_BT_LOG(WHITE, YL_DBG_HEX_7, 7,
        BB_read_baseband_register(BB_CSB_RX_SYNC_SCAN_INTV_REG),
        BB_read_baseband_register(BB_CSB_RX_SYNC_SCAN_WINDOW_REG),
        BB_read_baseband_register(BB_CSB_RX_SYNC_LAP0_REG),
        BB_read_baseband_register(BB_CSB_RX_SYNC_LAP1_REG),
        BB_read_baseband_register(BB_CSB_RX_SYNC_PARITY0_REG),
        BB_read_baseband_register(BB_CSB_RX_SYNC_PARITY1_REG),
        BB_read_baseband_register(BB_CSB_RX_SYNC_SCAN_MAP_REG));
    }
#endif
    /* start the sync scan */
    BB_write_baseband_register(INSTRUCTION_REGISTER, BB_START_SYNC_SCAN_3DD);

    /* enable sync train sw flag */
    bt_3dd_var.stp_rx_param.enable = TRUE;

    /* start the sync train timer */
    bt_csb_control_sync_scan_timer(TRUE);

    CSB_LOG(GREEN, LOG_LEVEL_HIGH, MSG_CSB_STP_RX_ENABLE, 11,
                            bt_3dd_var.stp_rx_param.enable,
                            bt_3dd_var.stp_rx_param.bd_addr[5],
                            bt_3dd_var.stp_rx_param.bd_addr[4],
                            bt_3dd_var.stp_rx_param.bd_addr[3],
                            bt_3dd_var.stp_rx_param.bd_addr[2],
                            bt_3dd_var.stp_rx_param.bd_addr[1],
                            bt_3dd_var.stp_rx_param.bd_addr[0],
                            bt_3dd_var.stp_rx_param.scan_window,
                            bt_3dd_var.stp_rx_param.scan_interval,
                            bt_3dd_var.stp_rx_param.scan_map,
                            bt_3dd_var.stp_rx_param.timeout);
}
/**************************************************************************
 * Function     : bt_csb_control_sync_scan_timer
 *
 * Description  : This function is used to enable or disable the software timer
 *                of synchronization scan timeout.
 *
 * Parameters   : enable - TRUE is enable and FALSE is disable
 *
 * Returns      : None
 *
 *************************************************************************/
void bt_csb_control_sync_scan_timer(UINT8 enable)
{
    if (!bt_3dd_var.stp_rx_param.enable)
    {
        return;
    }

    if (stp_timer != NULL)
    {
        /* stop the timer if we have the body of supervision timer */
        OS_DELETE_TIMER(&stp_timer);
    }

    if (enable)
    {
        /* create a supervision timer and install the callback function */
        OS_CREATE_TIMER(ONESHOT_TIMER, &stp_timer,
                bt_csb_sync_scan_timeout_callback, NULL, 0);

        OS_START_TIMER(stp_timer,
                        SLOT_VAL_TO_TIMER_VAL(bt_3dd_var.stp_rx_param.timeout));
    }
}
/**************************************************************************
 * Function     : bt_csb_sync_scan_timeout_callback
 *
 * Description  : This function is a callback function of the one shot software
 *                timer for synchronization train scan timeout. It shall
 *                stop the synchronization train scan then send hci
 *                syncrhonization scan complete event to hci host.
 *
 * Parameters   : timer_handle - the handle of software timer
 *                index - argument
 *
 * Returns      : None
 *
 *************************************************************************/
void bt_csb_sync_scan_timeout_callback(TimerHandle_t timer_handle)
{
    if (timer_handle == NULL)
    {
        return;
    }

	DEF_CRITICAL_SECTION_STORAGE;
    MINT_OS_ENTER_CRITICAL();

    /* delete timer in invalid argument */
    if ((!bt_3dd_var.stp_rx_param.enable) ||
            (stp_timer != timer_handle))
    {
        OS_DELETE_TIMER(&timer_handle);
        MINT_OS_EXIT_CRITICAL();
        return;
    }

    /* kill the sync train */
    bt_csb_driver_stop_sync_scan();

    MINT_OS_EXIT_CRITICAL();

    /* generate Synchronization Train Complete event */
    hci_generate_csa4_sychronization_train_received_event(HCI_COMMAND_SUCCEEDED);
}
/**************************************************************************
 * Function     : bt_csb_driver_stop_sync_scan
 *
 * Description  : This function is used to stop the synchronization train
 *                scan. It can set corresponding hw register and kill
 *                the software timer for synchronization train scan timeout.
 *
 * Parameters   : None
 *
 * Returns      : None
 *
 *************************************************************************/
void bt_csb_driver_stop_sync_scan(void)
{
    /* kill the sync train sw timer */
    bt_csb_control_sync_scan_timer(FALSE);

    /* kill the sync train */
    BB_write_baseband_register(INSTRUCTION_REGISTER, BB_KILL_SYNC_SCAN_3DD);

    /* disable sync train sw flag */
    bt_3dd_var.stp_rx_param.enable = FALSE;

    CSB_LOG(GREEN, LOG_LEVEL_HIGH, MSG_CSB_STP_RX_DISABLE, 0, 0);
}
/**************************************************************************
 * Function     : bt_csb_driver_start_rx_beacon
 *
 * Description  : This function is used to start the connectionless slave
 *                broadcast transmit. It can set corresponding hw register and
 *                initate the CSB trasmit supervision timer.
 *
 * Parameters   : None
 *
 * Returns      : None
 *
 *************************************************************************/
void bt_csb_driver_start_rx_beacon(void)
{
    UCHAR parity_bits[6];    /* So, do not move these down ! */
    UINT32 lap;

    UCHAR pid;
    UINT16 temp;
    UCHAR lcl_lut_index;
    UINT16 ce_index;
    LMP_CONNECTION_ENTITY *ce_ptr;
    UINT8 am_addr;

    DEF_CRITICAL_SECTION_STORAGE;

    MINT_OS_ENTER_CRITICAL();

    pid = lc_get_piconet_id_for_pagescan();

    am_addr = (bt_3dd_var.csb_rx_param.lt_addr);

    /*****************************************/
    /* set this for BB_handle_hlc_interrupt. */
    /*****************************************/
    lc_set_lc_cur_connecting_am_addr(am_addr);
    lc_pagescan_piconet_id = pid;
    /*****************************************/
    /*****************************************/
    /*****************************************/

    temp = (UINT16)((am_addr << 1) | SCA_PICONET_SLAVE);
    OR_val_with_bb_reg(reg_PICONET_INFO[pid], temp);

    lcl_lut_index = LC_SCA_SLAVE_1_LUT + pid;

    lut_ex_table[lcl_lut_index].lower_lut_address = reg_SCA_SLAVE_LOWER_LUT[pid];
    lut_ex_table[lcl_lut_index].upper_lut_address = reg_SCA_SLAVE_UPPER_LUT[pid];

    /* Arqn = 0, Seqn = 0, Flow = 1 */
    temp = BB_read_baseband_register(reg_SCA_SLAVE_UPPER_LUT[pid]);
    temp &= 0xFFF9;
    temp |= 0x0001;


    /* Set LUT unACTIVE!!!!!!!!!!!!!!!*/
    temp &= (~BIT8);


    BB_write_baseband_register(reg_SCA_SLAVE_UPPER_LUT[pid], temp);

    /*****************************************************/
    /*****************************************************/
    /***   lmp_handle_page_scan_fhs_pkt  ***/
    /*****************************************************/
    /* Get one ce_index for csb rx */
    if ((ce_index = lmp_allocate_entity_from_ce_database()) == BT_FW_ERROR)
    {
        MINT_OS_EXIT_CRITICAL();
        LMP_ERR(CONNECTION_ENTITY_ALLOCATION_FAILED,0,0);
        return;
    }

    bt_3dd_var.csb_rx_param.ce_index = ce_index;

    ce_ptr = &lmp_connection_entity[ce_index];

    ce_ptr->remote_dev_role = MASTER;

    /* Remote device as a master gives am_addr. use that am_Addr */
    ce_ptr->am_addr = am_addr;

    if (pid <= SCA_PICONET_MAX)
    {
        ce_ptr->phy_piconet_id = pid;
    }
    else
    {
        //RT_BT_LOG(RED, PICONET_ID_CHECK_MSG, 1, pid);
        ce_ptr->phy_piconet_id = SCA_PICONET_SECOND;
    }

    ce_ptr->Tpoll = bt_3dd_var.csb_rx_param.interval;

#ifdef COMPILE_AFH_HOP_KERNEL
    /* Disable AFH - clean up */
    BB_disable_afh(am_addr, ce_ptr->phy_piconet_id);
#endif /* COMPILE_AFH_HOP_KERNEL */

    lmp_slave_use_am_addr_ppi(am_addr, ce_ptr->phy_piconet_id, ce_index);

    ce_ptr->remote_dev_role = (UCHAR)MASTER;

    /* We do not have to set lmp status for csb slave since the lmp
    procedure are not necessary for csb slave. */
    //    lmp_set_ce_status(ce_index, LMP_PAGE_SCAN);

    lc_update_ce_index_to_lut_extn_tbl(ce_index, lcl_lut_index);

/*****************************************************/
/*****************************************************/
/*****************************************************/

    /* Configure connector register and remote parity bits */
    temp = (am_addr << 5) | (pid << 11) ;
    BB_write_baseband_register(CONNECTOR_REGISTER, temp);

    /* Configure remote device BD_ADDR */
    lc_slave_bd_addr[0] = *(UINT16*)&(bt_3dd_var.csb_rx_param.bd_addr[0]);
    BB_write_baseband_register(reg_PICONET_BD_ADDR1[pid], lc_slave_bd_addr[0]);

    lc_slave_bd_addr[1] = *(UINT16*)&(bt_3dd_var.csb_rx_param.bd_addr[2]);
    BB_write_baseband_register(reg_PICONET_BD_ADDR2[pid], lc_slave_bd_addr[1]);

    lc_slave_bd_addr[2] = *(UINT16*)&(bt_3dd_var.csb_rx_param.bd_addr[4]);
    BB_write_baseband_register(reg_PICONET_BD_ADDR3[pid], lc_slave_bd_addr[2]);

    lap = (bt_3dd_var.csb_rx_param.bd_addr[2] << 16) | lc_slave_bd_addr[0];
    lc_generate_parity_bits(lap, &parity_bits[0]);
    lc_slave_parity_bits[0] = *(UINT16*)&parity_bits[0];
    lc_slave_parity_bits[1] = *(UINT16*)&parity_bits[2];
    lc_slave_parity_bits[2] = (*(UINT16*)&parity_bits[4]) & 0x00ff;

    BB_write_baseband_register(
        reg_PICONET_PARITY_BITS1[pid], lc_slave_parity_bits[0]);
    BB_write_baseband_register(
        reg_PICONET_PARITY_BITS2[pid], lc_slave_parity_bits[1]);

    BB_WRITE_REMOTE_PARITY_BITS(parity_bits[4], (UCHAR) pid);

    BB_handle_hlc_interrupt_in_scatternet(pid);

    /* set beacon interval. */
    BB_write_baseband_register(TPOLL_HOLD_SNIFF_INTERVAL_REGISTER, ce_ptr->Tpoll);

#ifdef _CSB_RX_DBG_LOG
    if (g_csb_rx_dbg_log)
    {
        RT_BT_LOG(WHITE, DAPE_TEST_LOG213, 2,TPOLL_HOLD_SNIFF_INTERVAL_REGISTER,
            BB_read_baseband_register(TPOLL_HOLD_SNIFF_INTERVAL_REGISTER));
    }
#endif


    /* Program Clock */
    UINT32 cur_clk = BB_read_native_clock();
    UINT32 clk_offset = bt_3dd_var.csb_rx_param.clk_offset;

    UINT32 master_clk = cur_clk - clk_offset;
    master_clk &= (UINT32)(0x0FFFFFFF);

    UINT32 next_instant = bt_3dd_var.csb_rx_param.next_instant;
#ifdef _CSB_RX_DBG_LOG
    if (g_csb_rx_dbg_log)
    {
        RT_BT_LOG(WHITE, DAPE_TEST_LOG525, 6, cur_clk, clk_offset,
        master_clk, next_instant,0,0);
    }
#endif
    while (master_clk > next_instant)
    {
        next_instant += (bt_3dd_var.csb_rx_param.interval<<1);
        cur_clk = BB_read_native_clock();
        master_clk = cur_clk - clk_offset;
        master_clk &= (UINT32)(0x0FFFFFFF);

    }
    next_instant &= (UINT32)(0x0FFFFFFF);
    BB_write_baseband_register(SNIFF_SLOT_OFFSET_INTERVAL_REGISTER,
                               (UINT16) (next_instant >> 1));
#ifdef _CSB_RX_DBG_LOG
    if (g_csb_rx_dbg_log)
    {
        RT_BT_LOG(WHITE, DAPE_TEST_LOG525, 6, cur_clk, clk_offset,
        master_clk, next_instant,0,0);
    }
#endif
#ifndef _DAPE_TEST_CSB_RX_NO_SET_AFH
    BB_write_afh_map(am_addr, ce_ptr->phy_piconet_id,
                                            bt_3dd_var.csb_rx_param.ch_map);

#else
    BB_write_baseband_register(AFH_CHANNEL_MAP_EN_REG, 0);
#endif
    /*****************************************************/
    /*************** program clock ofst !! ****************/
    /*****************************************************/
    BB_write_baseband_register(BB_CSB_RX_SYNC_CLK_OFST0_REG,
           ((bt_3dd_var.stp_rx_param.clock_offset) & ((UINT16)(0xFFFF))));
    BB_write_baseband_register(BB_CSB_RX_SYNC_CLK_OFST1_REG,
                            (bt_3dd_var.stp_rx_param.clock_offset >> 16));


    /*****************************************************/
    /*************** program tolerance !! ****************/
    /******* not sure what value to fill in yet !!********/
    /*****************************************************/
    ce_ptr->sniff_xtol = g_beacon_rx_xtol;
    BB_modify_xtol_in_scatternet(ce_ptr->sniff_xtol, ce_ptr->phy_piconet_id);

    /*****************************************************/
    /* Set LUT unACTIVE!!!!!!!!!!!!!!!*/
    /*****************************************************/
    temp = BB_read_baseband_register(reg_SCA_SLAVE_UPPER_LUT[pid]);
    temp &= (~BIT8);
    /*****************************************************/
    /********** packet type not yet !!! ******************/
    /*****************************************************/
    if (bt_3dd_var.csb_rx_param.support_packet_type_bm & ALL_EDR_PACKETS)
    {
        temp |= (BIT15);
    }
    else
    {
        temp &= (~BIT15);
    }

    BB_write_baseband_register(reg_SCA_SLAVE_UPPER_LUT[pid], temp);


    /* start the beacon rx */
    BB_write_baseband_register(INSTRUCTION_REGISTER, BB_START_CSB_SCAN_3DD);
    MINT_OS_EXIT_CRITICAL();

    /* enable broadcast sw flag */
    bt_3dd_var.csb_rx_param.enable = TRUE;

    /* start the beacon tx supervision timer */
    /*****************************************************/
    /*************** Do not start this timer now!!! ********************/
    /*****************************************************/
    bt_csb_rx_control_supervision_timer(TRUE);

    /*****************************************************/
    /*************** skip not yet !!! ********************/
    /*****************************************************/
    /*****************************************************/
    /*************** skip not yet !!! ********************/
    /*****************************************************/
    ce_ptr->sniff_interval = bt_3dd_var.csb_rx_param.interval;

#ifdef _CSB_RX_SKIP_ENABLE
    ce_ptr->sniff_interval = bt_3dd_var.csb_rx_param.interval;

    //BB_write_baseband_register(BB_CSB_RX_SKIP_REG,
    //bt_3dd_var.csb_rx_param.skip | ce_ptr->phy_piconet_id << 8 |
    //7 << 10);

//    RT_BT_LOG(WHITE, DAPE_TEST_LOG213, 2, BB_CSB_RX_SKIP_REG,
//    BB_read_baseband_register(BB_CSB_RX_SKIP_REG));
#endif



#ifdef _CSB_RX_DBG_LOG
    if (g_csb_rx_dbg_log)
    {
        RT_BT_LOG(BLUE, YL_DBG_HEX_18, 18,
        BB_read_native_clock(),
        pid,
        BB_read_baseband_register(reg_PICONET_BD_ADDR1[pid]),
        BB_read_baseband_register(reg_PICONET_BD_ADDR2[pid]),
        BB_read_baseband_register(reg_PICONET_BD_ADDR3[pid]),
        BB_read_baseband_register(0x16),
        BB_read_baseband_register(reg_PICONET_PARITY_BITS1[pid]),
        BB_read_baseband_register(reg_PICONET_PARITY_BITS2[pid]),
        BB_read_baseband_register(TPOLL_HOLD_SNIFF_INTERVAL_REGISTER),
        BB_read_baseband_register(SNIFF_SLOT_OFFSET_INTERVAL_REGISTER),
        BB_read_baseband_register(X_VALUE_FOR_TOLERANCE_REGISTER),
        BB_read_baseband_register(reg_PICONET_INFO[pid]),
        ((bt_3dd_var.stp_rx_param.clock_offset) & ((UINT16)(0xFFFF))),
        (bt_3dd_var.stp_rx_param.clock_offset >> 16),
        BB_read_baseband_register(reg_SCA_SLAVE_UPPER_LUT[pid]),
        BB_read_baseband_register(reg_SCA_SLAVE_LOWER_LUT[pid]),
        bt_3dd_var.stp_rx_param.clock_offset,
        BB_read_native_clock() - (bt_3dd_var.stp_rx_param.clock_offset)

        );
    }
#endif

    CSB_LOG(GREEN, LOG_LEVEL_HIGH, MSG_CSB_BEACON_RX_ENABLE1,13,
            g_beacon_rx_xtol,
            bt_3dd_var.csb_rx_param.lt_addr,
            bt_3dd_var.csb_rx_param.bd_addr[5],
            bt_3dd_var.csb_rx_param.bd_addr[4],
            bt_3dd_var.csb_rx_param.bd_addr[3],
            bt_3dd_var.csb_rx_param.bd_addr[2],
            bt_3dd_var.csb_rx_param.bd_addr[1],
            bt_3dd_var.csb_rx_param.bd_addr[0],
            bt_3dd_var.csb_rx_param.interval,
            bt_3dd_var.csb_rx_param.clk_offset,
            bt_3dd_var.csb_rx_param.next_instant,
            bt_3dd_var.csb_rx_param.supervision_timeout,
            bt_3dd_var.csb_rx_param.remote_timging_accuracy);

    CSB_LOG(GREEN, LOG_LEVEL_HIGH, MSG_CSB_BEACON_RX_ENABLE2, 12,
          bt_3dd_var.csb_rx_param.skip,
          bt_3dd_var.csb_rx_param.support_packet_type_bm,
          bt_3dd_var.csb_rx_param.ch_map[9],
          bt_3dd_var.csb_rx_param.ch_map[8],
          bt_3dd_var.csb_rx_param.ch_map[7],
          bt_3dd_var.csb_rx_param.ch_map[6],
          bt_3dd_var.csb_rx_param.ch_map[5],
          bt_3dd_var.csb_rx_param.ch_map[4],
          bt_3dd_var.csb_rx_param.ch_map[3],
          bt_3dd_var.csb_rx_param.ch_map[2],
          bt_3dd_var.csb_rx_param.ch_map[1],
          bt_3dd_var.csb_rx_param.ch_map[0]);
}

/**************************************************************************
 * Function     : bt_csb_driver_stop_rx_beacon
 *
 * Description  : This function is used to stop the connectionless slave
 *                broadcast transmit. It can set corresponding hw register and
 *                kill the CSB trasmit supervision timer.
 *
 * Parameters   : None
 *
 * Returns      : None
 *
 *************************************************************************/
void bt_csb_driver_stop_rx_beacon(void)
{
    /* kill the supervision sw timer */
    bt_csb_rx_control_supervision_timer(FALSE);

    /* kill the csb scan */
    BB_write_baseband_register(INSTRUCTION_REGISTER, BB_KILL_CSB_SCAN_3DD);

    /* disble broadcast sw flag */
    bt_3dd_var.csb_rx_param.enable = FALSE;

    CSB_LOG(GREEN, LOG_LEVEL_HIGH, MSG_CSB_BEACON_RX_DISABLE, 0, 0);
}
/**************************************************************************
 * Function     : bt_csb_rx_control_supervision_timer
 *
 * Description  : This function is used to enable or disable the software timer
 *                of connectionless slave broadcast transmit supervision.
 *
 * Parameters   : enable - TRUE is enable and FALSE is disable
 *
 * Returns      : None
 *
 *************************************************************************/
void bt_csb_rx_control_supervision_timer(UINT8 enable)
{
    if (!bt_3dd_var.csb_rx_param.enable)
    {
        return;
    }

    if (csb_trx_supervision_timer != NULL)
    {
        /* stop the timer if we have the body of supervision timer */
        OS_DELETE_TIMER(&csb_trx_supervision_timer);
    }

    if (enable)
    {
        /* create a supervision timer and install the callback function */
        OS_CREATE_TIMER(ONESHOT_TIMER, &csb_trx_supervision_timer,
                bt_csb_rx_supervision_callback, NULL, 0);

        OS_START_TIMER(csb_trx_supervision_timer,
            SLOT_VAL_TO_TIMER_VAL(bt_3dd_var.csb_rx_param.supervision_timeout));
    }
}
/**************************************************************************
 * Function     : bt_csb_rx_supervision_callback
 *
 * Description  : This function is a callback function of the one shot software
 *                timer for CSB transmit supervision timeout. It shall
 *                stop the connectionless slave broadcast transmit then send hci
 *                connectionless slave broadcast timeout event to hci host.
 *
 * Parameters   : timer_handle - the handle of software timer
 *                index - argument
 *
 * Returns      : None
 *
 *************************************************************************/
void bt_csb_rx_supervision_callback(TimerHandle_t timer_handle)
{
    if (timer_handle == NULL)
    {
        return;
    }

	DEF_CRITICAL_SECTION_STORAGE;
    MINT_OS_ENTER_CRITICAL();

    /* delete timer in invalid argument */
    if ((!bt_3dd_var.csb_rx_param.enable) ||
                (csb_trx_supervision_timer != timer_handle))
    {
        OS_DELETE_TIMER(&timer_handle);
        MINT_OS_EXIT_CRITICAL();
        return;
    }

    /* kill the broadcast slave data */
    bt_csb_driver_stop_rx_beacon();

    lmp_cleanup_after_acl_detach_if_no_scheduled_pkt(bt_3dd_var.csb_rx_param.ce_index);

    MINT_OS_EXIT_CRITICAL();

    /* generate Connectionless Slave Broadcast Timeout event */
    hci_generate_csa4_connectionless_slave_broadcast_timeout_event(1);
}
/**************************************************************************
 * Function     : bt_3dd_checkin_sync_train_packet
 *
 * Description  : This function is used to check-in 3dd sync train
 *                packet during sync scan state
 *
 * Parameters   : pparam - the Pointer of BB Rx Parameters
 *
 * Returns      : None
 *
 *************************************************************************/
void bt_3dd_checkin_sync_train_packet(BB_RX_PARAMS *pparam)
{
    PKT_HEADER *ppkt;
    UINT32 rsvd_buf = FALSE;
    OS_SIGNAL rx_sig_send;
    UINT8 wptr;

    /* parser potential association notification packet */
    wptr = lc_rx_pkt_header_q.write_pointer;

    ppkt = &lc_rx_pkt_header_q.pkt_header_queue[wptr];

    /* Queue the packet in the headers queue */
    ppkt->packet_header = pparam->packet_header;
    ppkt->payload_header = pparam->payload_header;

    /* queue packet and update write pointer of queue */
    lc_rx_pkt_header_q.write_pointer =
                    (wptr + 1) & (LC_MAX_NUMBER_OF_HEADERS - 1);

    /* Send the signal to lc_rx_task to pick up the packet*/
    rx_sig_send.type = LC_HANDLE_RECD_PACKET_SIGNAL;
    rx_sig_send.param = (void*)(rsvd_buf);
    OS_ISR_SEND_SIGNAL_TO_TASK ( lc_rx_task_handle, rx_sig_send );
}


/**************************************************************************
 * Function     : bt_3dd_handle_received_sync_train_packet
 *
 * Description  : This function is used to handle received sync train packet
 *                during sync scan state. FW will generate sync_train_
 *                received_event (0x50 )to notify hci host.
 *                HCI_SYNCHRONIZATION_TRAIN_RECEIVED_EVENT
 * Parameters   : None
 *
 * Returns      : TRUE or FALSE
 *
 *************************************************************************/
UINT8 bt_3dd_handle_received_sync_train_packet(void)
{
    HCI_EVENT_PKT   *hci_event_pkt_ptr ;
    //OS_SIGNAL signal_send;
    UINT32 next_instant;

    if (OS_ALLOC_BUFFER(tx_table[HCI_EVENT_HANDLER_TASK].pool_handle,
                       (OS_ADDRESS *)&hci_event_pkt_ptr) != BT_ERROR_OK)
    {
#ifndef BZDMA_DONT_WAIT_RXCMD_COMPLETE
        BB_read_baseband_RX_FIFO_and_flush(28, TRUE);
#else
        BB_read_baseband_RX_FIFO_and_flush(28, FALSE);
#endif
        return FALSE;
    }

    hci_event_pkt_ptr->event_opcode = HCI_VENDOR_SPECIFIC_EVENT;
    BB_read_baseband_RX_FIFO(&hci_event_pkt_ptr->event_parameter[0], 28, TRUE);
#ifdef _DAPE_TEST_CSB_RX_SELF_CALC_CLK_OFST_BY_VENDOR_CMD
    if (g_self_calc_clk_ofst)
    {
        bt_3dd_var.stp_rx_param.master_clk =
        hci_event_pkt_ptr->event_parameter[0] |
        (hci_event_pkt_ptr->event_parameter[1] << 8)|
        (hci_event_pkt_ptr->event_parameter[2] << 16)|
        (hci_event_pkt_ptr->event_parameter[3] << 24);

        bt_3dd_var.stp_rx_param.clock_offset =
        BB_read_native_clock() - bt_3dd_var.stp_rx_param.master_clk;

        //RT_BT_LOG(WHITE, DAPE_TEST_LOG207, 1,bt_3dd_var.stp_rx_param.master_clk);
    }
#endif
#ifdef _CSB_RX_DBG_LOG
    if (g_csb_rx_dbg_log)
    {
        RT_BT_LOG(WHITE, YL_DBG_HEX_20, 20,
        hci_event_pkt_ptr->event_parameter[0],
        hci_event_pkt_ptr->event_parameter[1],
        hci_event_pkt_ptr->event_parameter[2],
        hci_event_pkt_ptr->event_parameter[3],
        hci_event_pkt_ptr->event_parameter[4],
        hci_event_pkt_ptr->event_parameter[5],
        hci_event_pkt_ptr->event_parameter[6],
        hci_event_pkt_ptr->event_parameter[7],
        hci_event_pkt_ptr->event_parameter[8],
        hci_event_pkt_ptr->event_parameter[9],
        hci_event_pkt_ptr->event_parameter[10],
        hci_event_pkt_ptr->event_parameter[11],
        hci_event_pkt_ptr->event_parameter[12],
        hci_event_pkt_ptr->event_parameter[13],
        hci_event_pkt_ptr->event_parameter[14],
        hci_event_pkt_ptr->event_parameter[15],
        hci_event_pkt_ptr->event_parameter[16],
        hci_event_pkt_ptr->event_parameter[17],
        hci_event_pkt_ptr->event_parameter[18],
        hci_event_pkt_ptr->event_parameter[19]);
        RT_BT_LOG(WHITE, YL_DBG_HEX_8, 8,
        hci_event_pkt_ptr->event_parameter[20],
        hci_event_pkt_ptr->event_parameter[21],
        hci_event_pkt_ptr->event_parameter[22],
        hci_event_pkt_ptr->event_parameter[23],
        hci_event_pkt_ptr->event_parameter[24],
        hci_event_pkt_ptr->event_parameter[25],
        hci_event_pkt_ptr->event_parameter[26],
        hci_event_pkt_ptr->event_parameter[27]);
    }
#endif
    bt_3dd_var.stp_rx_param.csb_lt_addr = hci_event_pkt_ptr->event_parameter[26];
    memcpy(&bt_3dd_var.stp_rx_param.ch_map[0],
                                  &hci_event_pkt_ptr->event_parameter[8], 10);
    // memcpy(&bt_3dd_var.stp_rx_param.next_instant[0],
    //                               &hci_event_pkt_ptr->event_parameter[4], 4);
    next_instant = hci_event_pkt_ptr->event_parameter[4] |
            (hci_event_pkt_ptr->event_parameter[5] << 8) |
            (hci_event_pkt_ptr->event_parameter[6] << 16) |
            (hci_event_pkt_ptr->event_parameter[7] << 24);
    next_instant <<= 1;
    bt_3dd_var.stp_rx_param.next_instant[0] = next_instant & 0xFF;
    bt_3dd_var.stp_rx_param.next_instant[1] = next_instant >> 8;
    bt_3dd_var.stp_rx_param.next_instant[2] = next_instant >> 16;
    bt_3dd_var.stp_rx_param.next_instant[3] = next_instant >> 24;

    memcpy(&bt_3dd_var.stp_rx_param.csb_interval[0],
                                  &hci_event_pkt_ptr->event_parameter[24], 4);

    bt_3dd_var.stp_rx_param.service_data = hci_event_pkt_ptr->event_parameter[27];

    OS_FREE_BUFFER(tx_table[HCI_EVENT_HANDLER_TASK].pool_handle,
                    hci_event_pkt_ptr);

    CSB_LOG(GREEN, LOG_LEVEL_HIGH, MSG_CSB_STP_RECEIVED1, 17,
                            bt_3dd_var.stp_rx_param.bd_addr[5],
                            bt_3dd_var.stp_rx_param.bd_addr[4],
                            bt_3dd_var.stp_rx_param.bd_addr[3],
                            bt_3dd_var.stp_rx_param.bd_addr[2],
                            bt_3dd_var.stp_rx_param.bd_addr[1],
                            bt_3dd_var.stp_rx_param.bd_addr[0],
                            bt_3dd_var.stp_rx_param.clock_offset,
                            bt_3dd_var.stp_rx_param.ch_map[9],
                            bt_3dd_var.stp_rx_param.ch_map[8],
                            bt_3dd_var.stp_rx_param.ch_map[7],
                            bt_3dd_var.stp_rx_param.ch_map[6],
                            bt_3dd_var.stp_rx_param.ch_map[5],
                            bt_3dd_var.stp_rx_param.ch_map[4],
                            bt_3dd_var.stp_rx_param.ch_map[3],
                            bt_3dd_var.stp_rx_param.ch_map[2],
                            bt_3dd_var.stp_rx_param.ch_map[1],
                            bt_3dd_var.stp_rx_param.ch_map[0]);
    CSB_LOG(GREEN, LOG_LEVEL_HIGH, MSG_CSB_STP_RECEIVED2, 8,
                            bt_3dd_var.stp_rx_param.csb_lt_addr,
                            bt_3dd_var.stp_rx_param.next_instant[3],
                            bt_3dd_var.stp_rx_param.next_instant[2],
                            bt_3dd_var.stp_rx_param.next_instant[1],
                            bt_3dd_var.stp_rx_param.next_instant[0],
                            bt_3dd_var.stp_rx_param.csb_interval[1],
                            bt_3dd_var.stp_rx_param.csb_interval[0],
                            bt_3dd_var.stp_rx_param.service_data);
    hci_generate_csa4_sychronization_train_received_event(HCI_COMMAND_SUCCEEDED);
#ifdef _CSB_TEST_RX_BEACON_AFTER_RX_SYNC_TRAIN

    bt_csb_driver_stop_sync_scan();

    pf_delay(500);
/*
    bt_3dd_var.csb_rx_param.enable = 1;
    bt_3dd_var.csb_rx_param.lt_addr = bt_3dd_var.stp_rx_param.csb_lt_addr;
    memcpy(&bt_3dd_var.csb_rx_param.bd_addr[0],
             &bt_3dd_var.stp_rx_param.bd_addr[0], 6);
    bt_3dd_var.csb_rx_param.interval =
        bt_3dd_var.stp_rx_param.csb_interval[0]|(bt_3dd_var.stp_rx_param.csb_interval[1]<<8);
    bt_3dd_var.csb_rx_param.clk_offset = bt_3dd_var.stp_rx_param.clock_offset;
    bt_3dd_var.csb_rx_param.next_instant =
        bt_3dd_var.stp_rx_param.next_instant[0] |
        (bt_3dd_var.stp_rx_param.next_instant[1] << 8) |
        (bt_3dd_var.stp_rx_param.next_instant[2] << 16) |
        (bt_3dd_var.stp_rx_param.next_instant[3] << 24);
    memcpy(&bt_3dd_var.csb_rx_param.ch_map[0],
             &bt_3dd_var.stp_rx_param.ch_map[0], 10);

    bt_3dd_var.csb_rx_param.skip = 3;
    bt_3dd_var.csb_rx_param.supervision_timeout = 0x2000;

    bt_csb_driver_start_rx_beacon();
*/
    HCI_CMD_PKT temp_hci_cmd_pkt;

    temp_hci_cmd_pkt.cmd_parameter[0] = 1;
    memcpy(&temp_hci_cmd_pkt.cmd_parameter[1],
         &bt_3dd_var.stp_rx_param.bd_addr[0], 6);
    temp_hci_cmd_pkt.cmd_parameter[7] = bt_3dd_var.stp_rx_param.csb_lt_addr;

    temp_hci_cmd_pkt.cmd_parameter[8] = bt_3dd_var.stp_rx_param.csb_interval[0];
    temp_hci_cmd_pkt.cmd_parameter[9] = bt_3dd_var.stp_rx_param.csb_interval[1];
    temp_hci_cmd_pkt.cmd_parameter[10] = bt_3dd_var.stp_rx_param.clock_offset;
    temp_hci_cmd_pkt.cmd_parameter[11] = bt_3dd_var.stp_rx_param.clock_offset >> 8;
    temp_hci_cmd_pkt.cmd_parameter[12] = bt_3dd_var.stp_rx_param.clock_offset >> 16;
    temp_hci_cmd_pkt.cmd_parameter[13] = bt_3dd_var.stp_rx_param.clock_offset >> 24;

    temp_hci_cmd_pkt.cmd_parameter[14] =
                    bt_3dd_var.stp_rx_param.next_instant[0];
    temp_hci_cmd_pkt.cmd_parameter[15] =
                    bt_3dd_var.stp_rx_param.next_instant[1];
    temp_hci_cmd_pkt.cmd_parameter[16] =
                    bt_3dd_var.stp_rx_param.next_instant[2];
    temp_hci_cmd_pkt.cmd_parameter[17] =
                    bt_3dd_var.stp_rx_param.next_instant[3];

    temp_hci_cmd_pkt.cmd_parameter[18] = 00;
    temp_hci_cmd_pkt.cmd_parameter[19] = 0x20;
    temp_hci_cmd_pkt.cmd_parameter[20] = 01;

    /* skip */
    temp_hci_cmd_pkt.cmd_parameter[21] = 03;

    /* packet type */
    if (!g_ptt_enabled)
    {
    /* Basic Rate */
    temp_hci_cmd_pkt.cmd_parameter[22] = 0xFF;
    temp_hci_cmd_pkt.cmd_parameter[23] = 0xFF;
    }
    else
    {
    /* EDR */
        temp_hci_cmd_pkt.cmd_parameter[22] = 0x00;
        temp_hci_cmd_pkt.cmd_parameter[23] = 0x00;
    }
    memcpy(&temp_hci_cmd_pkt.cmd_parameter[24],
             &bt_3dd_var.stp_rx_param.ch_map[0], 10);

hci_handle_set_connectionless_slave_broadcast_receive(&temp_hci_cmd_pkt);
#endif
#ifdef _CSB_RX_DBG_LOG
/*RT_BT_LOG(WHITE, YL_DBG_HEX_18, 18,
bt_3dd_var.stp_rx_param.csb_lt_addr,
bt_3dd_var.stp_rx_param.ch_map[0],
bt_3dd_var.stp_rx_param.ch_map[1],
bt_3dd_var.stp_rx_param.ch_map[2],
bt_3dd_var.stp_rx_param.ch_map[3],
bt_3dd_var.stp_rx_param.ch_map[4],
bt_3dd_var.stp_rx_param.ch_map[5],
bt_3dd_var.stp_rx_param.ch_map[6],
bt_3dd_var.stp_rx_param.ch_map[7],
bt_3dd_var.stp_rx_param.ch_map[8],
bt_3dd_var.stp_rx_param.ch_map[9],
bt_3dd_var.stp_rx_param.next_instant[0],
bt_3dd_var.stp_rx_param.next_instant[1],
bt_3dd_var.stp_rx_param.next_instant[2],
bt_3dd_var.stp_rx_param.next_instant[3],
bt_3dd_var.stp_rx_param.csb_interval[0],
bt_3dd_var.stp_rx_param.csb_interval[1],
bt_3dd_var.stp_rx_param.service_data);
RT_BT_LOG(WHITE, YL_DBG_HEX_6, 6,
bt_3dd_var.stp_rx_param.bd_addr[5],
bt_3dd_var.stp_rx_param.bd_addr[4],
bt_3dd_var.stp_rx_param.bd_addr[3],
bt_3dd_var.stp_rx_param.bd_addr[2],
bt_3dd_var.stp_rx_param.bd_addr[1],
bt_3dd_var.stp_rx_param.bd_addr[0]);*/
#endif
    return TRUE;
}
/**************************************************************************
 * Function     : bt_3dd_handle_received_beacon_packet
 *
 * Description  : This function is used to handle received beacon packet
 *                during csb scan state. FW will generate sync_train_
 *                received_event (0x52 )to notify hci host.
 *
 * Parameters   : None
 *
 * Returns      : TRUE or FALSE
 *
 *************************************************************************/
void bt_3dd_handle_received_beacon_packet(UINT16 payload_length, UINT8 am_addr,
    UINT8 phy_piconet_id, UCHAR pkt_type, UCHAR llid)
{
    HCI_ACL_RX_DATA_PKT *acl_pkt;

    if ( (OS_ALLOC_BUFFER(tx_table[ACL_DATA_HANDLER_TASK].pool_handle,
                      (void **)&acl_pkt))!= BT_ERROR_OK)
    {
#ifndef BZDMA_DONT_WAIT_RXCMD_COMPLETE
        BB_read_baseband_RX_FIFO_and_flush(payload_length, TRUE);
#else
        BB_read_baseband_RX_FIFO_and_flush(payload_length, FALSE);
#endif
        return;
    }
#ifndef BZDMA_DONT_WAIT_RXCMD_COMPLETE
    BB_read_baseband_RX_FIFO(acl_pkt->hci_acl_data_pkt, payload_length, TRUE);
#else
    BB_read_baseband_RX_FIFO(acl_pkt->hci_acl_data_pkt, payload_length, FALSE);
#endif
    {
        UINT32 cur_clk;
        lc_get_clock_in_scatternet(&cur_clk, phy_piconet_id);
        RT_BT_LOG(WHITE, MSG_CSB_BEACON_RECEIVED, 24 ,
            cur_clk, llid, pkt_type, payload_length,
            acl_pkt->hci_acl_data_pkt[0],
            acl_pkt->hci_acl_data_pkt[1],
            acl_pkt->hci_acl_data_pkt[2],
            acl_pkt->hci_acl_data_pkt[3],
            acl_pkt->hci_acl_data_pkt[4],
            acl_pkt->hci_acl_data_pkt[5],
            acl_pkt->hci_acl_data_pkt[6],
            acl_pkt->hci_acl_data_pkt[7],
            acl_pkt->hci_acl_data_pkt[8],
            acl_pkt->hci_acl_data_pkt[9],
            acl_pkt->hci_acl_data_pkt[10],
            acl_pkt->hci_acl_data_pkt[11],
            acl_pkt->hci_acl_data_pkt[12],
            acl_pkt->hci_acl_data_pkt[13],
            acl_pkt->hci_acl_data_pkt[14],
            acl_pkt->hci_acl_data_pkt[15],
            acl_pkt->hci_acl_data_pkt[16],
            acl_pkt->hci_acl_data_pkt[17],
            bt_3dd_var.csb_rx_param.clk_offset,
            bt_3dd_var.csb_rx_param.clk_rx);
    }
    if(OS_FREE_BUFFER(tx_table[ACL_DATA_HANDLER_TASK].pool_handle,acl_pkt)
    == BT_ERROR_OK)
    {
        os_free_reserved_buffer();
    }
    /******************************************************/
    /******************************************************/
    /******** To Do: Send Event To Host!!!!****************/
    /******************************************************/
    /******************************************************/

}
#endif


// 20140212 morgan add host set shutter delay
void fn_host_set_shutter_delay(INT16 bt_shutter_lsod, INT16 bt_shutter_lscd,
        INT16 bt_shutter_rsod, INT16 bt_shutter_rscd)
{
    UINT16 temp_period = 0;

    bt_3dd_var.beacon.lsod = bt_shutter_lsod;
    bt_3dd_var.beacon.lscd = (long_term_avg_period >> 1) + bt_shutter_lscd;
    bt_3dd_var.beacon.rsod = ((long_term_avg_period + 1) >> 1) + bt_shutter_rsod;
    bt_3dd_var.beacon.rscd = long_term_avg_period + bt_shutter_rscd;

    RT_BT_LOG(GREEN, MSG_3DD_SHOW_BEACON, 4,
            bt_3dd_var.beacon.lsod, bt_3dd_var.beacon.lscd,
            bt_3dd_var.beacon.rsod, bt_3dd_var.beacon.rscd);

    if (efuse_tv_3dd->period_delay == 0)
    {
        if (bt_shutter_lsod < 0)
        {
            bt_3dd_var.beacon.lsod = 0;
        }
        temp_period = 0;
    }
    else if (efuse_tv_3dd->period_delay == 1)
    {
        temp_period = (long_term_avg_period >> 1); //BT_EXT_FRAME_PERIOD_60HZ_US_DEFAULT/2;
    }
    else if (efuse_tv_3dd->period_delay == 2)
    {
        temp_period = long_term_avg_period; //BT_EXT_FRAME_PERIOD_60HZ_US_DEFAULT;
    }
    bt_3dd_var.beacon.lsod += temp_period;
    bt_3dd_var.beacon.lscd += temp_period;
    bt_3dd_var.beacon.rsod += temp_period;
    bt_3dd_var.beacon.rscd += temp_period;

    if (efuse_tv_3dd->L_R_first)
    {
        temp_period = bt_3dd_var.beacon.lsod;
        bt_3dd_var.beacon.lsod = bt_3dd_var.beacon.rsod;
        bt_3dd_var.beacon.rsod = temp_period;
        temp_period = bt_3dd_var.beacon.lscd;
        bt_3dd_var.beacon.lscd = bt_3dd_var.beacon.rscd;
        bt_3dd_var.beacon.rscd = temp_period;
    }

#ifdef _ROM_CODE_PATCHED_
    if (rcp_bt_3dd_driver_fn_set_host_shutter_delay != NULL)
    {
        rcp_bt_3dd_driver_fn_set_host_shutter_delay();
    }
#endif

    RT_BT_LOG(GREEN, DD_MSG_CMD13, 4,
            bt_3dd_var.beacon.lsod, bt_3dd_var.beacon.lscd,
            bt_3dd_var.beacon.rsod, bt_3dd_var.beacon.rscd);
}

// 20140123 morgan modify half period value = long_term_avg_period
void fn_set_shutter_delay(void)
{
    UINT16 temp_period = 0;
    UINT16 shutter_open_delay_term;
    UINT16 shutter_close_delay_term;

    shutter_open_delay_term = (efuse_tv_3dd->shutter_open_delay * 20);
    shutter_close_delay_term = (efuse_tv_3dd->shutter_close_delay * 20);

#if 0
    bt_3dd_var.beacon.lsod = efuse_tv_3dd->lsod - shutter_open_delay_term;
    bt_3dd_var.beacon.lscd = BT_EXT_FRAME_PERIOD_60HZ_US_DEFAULT/2 + efuse_tv_3dd->lscd -shutter_close_delay_term;
    bt_3dd_var.beacon.rsod = (BT_EXT_FRAME_PERIOD_60HZ_US_DEFAULT+1)/2 + efuse_tv_3dd->rsod -shutter_close_delay_term;
    bt_3dd_var.beacon.rscd = BT_EXT_FRAME_PERIOD_60HZ_US_DEFAULT + efuse_tv_3dd->rscd - shutter_open_delay_term;
#endif

    bt_3dd_var.beacon.lsod = (efuse_tv_3dd->lsod * 4) - shutter_open_delay_term;
    bt_3dd_var.beacon.lscd = (long_term_avg_period / 2) + (efuse_tv_3dd->lscd * 4) - shutter_close_delay_term;
    bt_3dd_var.beacon.rsod = ((long_term_avg_period + 1) / 2) + (efuse_tv_3dd->rsod * 4) - shutter_close_delay_term;
    bt_3dd_var.beacon.rscd = long_term_avg_period + (efuse_tv_3dd->rscd * 4) - shutter_open_delay_term;

    if (efuse_tv_3dd->period_delay == 0)
    {
        if (shutter_open_delay_term > (efuse_tv_3dd->lsod * 4))
        {
            bt_3dd_var.beacon.lsod = 0;
        }
        temp_period = 0;
    }
    else if (efuse_tv_3dd->period_delay == 1)
    {
        temp_period = (long_term_avg_period / 2); //BT_EXT_FRAME_PERIOD_60HZ_US_DEFAULT/2;
    }
    else if (efuse_tv_3dd->period_delay == 2)
    {
        temp_period = long_term_avg_period; //BT_EXT_FRAME_PERIOD_60HZ_US_DEFAULT;
    }
    bt_3dd_var.beacon.lsod += temp_period;
    bt_3dd_var.beacon.lscd += temp_period;
    bt_3dd_var.beacon.rsod += temp_period;
    bt_3dd_var.beacon.rscd += temp_period;

    if (efuse_tv_3dd->L_R_first)
    {
        temp_period = bt_3dd_var.beacon.lsod;
        bt_3dd_var.beacon.lsod = bt_3dd_var.beacon.rsod;
        bt_3dd_var.beacon.rsod = temp_period;
        temp_period = bt_3dd_var.beacon.lscd;
        bt_3dd_var.beacon.lscd = bt_3dd_var.beacon.rscd;
        bt_3dd_var.beacon.rscd = temp_period;
    }

#ifdef _ROM_CODE_PATCHED_
    if (rcp_bt_3dd_driver_fn_set_shutter_delay != NULL)
    {
        rcp_bt_3dd_driver_fn_set_shutter_delay();
    }
#endif

    RT_BT_LOG(GREEN, DD_MSG_CMD8, 4,
            bt_3dd_var.beacon.lsod, bt_3dd_var.beacon.lscd,
            bt_3dd_var.beacon.rsod, bt_3dd_var.beacon.rscd);
}

#ifdef ENABLE_REPORT_3DD_SYNC_TOGGLE
void bt_3dd_driver_fn3DSyncToggleTimer(TimerHandle_t timer)
{
	if ( gpio_3dd_sync_toggle_cnt == 1 )
	{
		gpio_3dd_sync_toggle_cnt = 0;
		bt_3dd_driver_generate_3dd_gpio_sync_toggle_response(1);
	}
	else
	{
		bt_3dd_driver_generate_3dd_gpio_sync_toggle_response(0);
	}
 }
#endif


// 20131106 morgan add for report 3dd sync toggle
// 20140123 morgan modify for 3DD + HOGP mode
// 20140417 morgan copy to 8821
#ifdef ENABLE_REPORT_3DD_SYNC_TOGGLE
const UINT16 c_3dd_period_init_table[3] = {16666,8333,20000};

void bt_3dd_driver_generate_3dd_gpio_sync_toggle_response(UINT8 status)
{
#ifdef _ROM_CODE_PATCHED_
    if (rcp_generate_3dd_gpio_sync_toggle_response_func != NULL)
    {
        if (rcp_generate_3dd_gpio_sync_toggle_response_func((void*)(&status)))
        {
            return;
        }
    }
#endif

    /* send toggle event from 3DG to host or directly response to status change in 3DD mode */

    if ((flag_3dd_toggle_q == 1) && (status == 0)) /* busy to idle */
    {
        /* wait for efuse_tv_3dd->toggle_detection_delay * 2 sec to report event */
        toggle_delay_check_cnt = 0;
        enable_toggle_delay_check = 1;

#ifdef _3DD_ONLY_SUPPORT
        if (efuse_3dd_only)
        {
            RT_BT_LOG(YELLOW, D3D_MSG_22, 2, status, period_mode);
        }
        else /* host mode */
#endif
        {
            //pat3d_generate_3dd_gpio_sync_toggle_event(status);
            //RT_BT_LOG(YELLOW,BT_3DD_MSG_SEND_VENDOR_EVT_SYNC_TOGGLE, 1, status);

        }

        flag_3dd_toggle_q = status;
        return;
    }
    else if ((flag_3dd_toggle_q == 0) && (status == 1)) /* idle to busy */
    {
        /* reset period calculation */
        flag_toggle_restart = 1;
        //d3d_sync_toggle_restart_cnt = 0;
        avg_cnt = 0;
        state_period_detection = 0;
        enable_toggle_delay_check = 0; /* disable enable_toggle_delay_check timer */
        toggle_delay_check_cnt = 0; /* reset toggle delay check */
        long_term_avg_period = c_3dd_period_init_table[period_mode];

#ifdef _3DD_ONLY_SUPPORT
        if (efuse_3dd_only)
        {
            /* enable synchronization train transmitter */
            bt_3dd_driver_start_sync_train();
            /* start transmit beacon = TRUE */
            pat3d_sim_hci_handle_set_connectionless_slave_broadcast(TRUE);
            RT_BT_LOG(YELLOW,D3D_MSG_22, 2, status,period_mode);
        }
        else
#endif
        {
            /* report event to host */
            pat3d_generate_3dd_gpio_sync_toggle_event(status);
            // dape move log to pat3d_generate_3dd_gpio_sync_toggle_event
            //RT_BT_LOG(YELLOW,BT_3DD_MSG_SEND_VENDOR_EVT_SYNC_TOGGLE, 1, status);
            flag_3dd_toggle_q = status;
            return;
        }
    }

    /* check if need to start sync train immediatelly after toggle turn busy */
    if (status)
    {
#ifdef _3DD_ONLY_SUPPORT
        if (efuse_sync_train_always_on && (!bt_3dd_var.stp_tx_param.enable))
        {
            /* enable synchronization train transmitter */
            //bt_3dd_driver_start_sync_train();
            bt_csb_driver_start_sync_train();
            /* start transmit beacon = TRUE */
            //pat3d_sim_hci_handle_set_connectionless_slave_broadcast(TRUE);
            /* enable connectionless slave broadcast transmitter */
            bt_csb_driver_start_transmit_beacon();
        }
#endif
    }
    else /* start delay timer if toggle is idle */
    {
        if (enable_toggle_delay_check)
        {
            if (toggle_delay_check_cnt >= (efuse_tv_3dd->toggle_detection_delay * 2))
            {
#ifdef _3DD_ONLY_SUPPORT
                if (efuse_3dd_only) /* 3DD only mode:stop sync and beacon train*/
                {
                    pat3d_sim_hci_handle_set_connectionless_slave_broadcast(FALSE);
                }
                else /* host mode : report event to host */
#endif
                {
                    pat3d_generate_3dd_gpio_sync_toggle_event(status);
                    RT_BT_LOG(GREEN, BT_3DD_MSG_SEND_VENDOR_EVT_SYNC_TOGGLE, 1, status);
                }
                toggle_delay_check_cnt = 0;
                enable_toggle_delay_check = 0;
            }
            toggle_delay_check_cnt++;
        }
    }
    flag_3dd_toggle_q = status;
}
#endif

// register setting
//BB_START_SYNC_3DD      0x14
//BB_START_BEACON_3DD   0x15
// 20140417 morgan copy to 8821
#ifdef _3DD_ONLY_SUPPORT
UCHAR pat3d_sim_hci_handle_set_connectionless_slave_broadcast(UCHAR an_status)
{
    UCHAR ret_error = HCI_COMMAND_SUCCEEDED;

    if (an_status == 0)
    {
        /* If this command is issued with Enable = 0, and the
           synchronization train substate is active, then the Controller
           shall also exit the synchronization train substate.*/

        /* disable synchronization train transmitter */
        bt_3dd_driver_stop_sync_train();
	    //bt_csb_driver_stop_sync_train();

        /* disable connectionless slave broadcast transmitter */
        bt_3dd_driver_stop_transmit_beacon();
    }
    else
    {
        /* enable connectionless slave broadcast transmitter */
        bt_3dd_driver_start_transmit_beacon();
    }

    return ret_error;
}
#endif

// 20140122 morgan add for report 3dd sync toggle
// 20140123 morgan modify for 3DD + HOGP mode
// 20140417 morgan copy to 8821
void pat3d_generate_3dd_gpio_sync_toggle_event(UINT8 status)
{
#if 0
    HCI_EVENT_PKT   *hci_event_pkt_ptr ;
    OS_SIGNAL signal_send;

	   // allocate buffer
    if (OS_ALLOC_BUFFER(tx_table[HCI_EVENT_HANDLER_TASK].pool_handle,
                       (OS_ADDRESS *)&hci_event_pkt_ptr) != BT_ERROR_OK)
    {
        // allocate buffer fail
        RT_BT_LOG(RED,BT_3DD_MSG_SEND_VENDOR_EVT_SYNC_TOGGLE, 1, 9);
        return;
    }

    hci_event_pkt_ptr->event_opcode = HCI_VENDOR_SPECIFIC_EVENT;
    hci_event_pkt_ptr->event_parameter[0] = 0x28; // sub code
    hci_event_pkt_ptr->event_parameter[1] = status; //0x00;
    hci_event_pkt_ptr->param_total_length = 2;//19;

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
    RT_BT_LOG(YELLOW,BT_3DD_MSG_SEND_VENDOR_EVT_SYNC_TOGGLE, 1, status);
#endif

#ifdef CONFIG_TV_POWERON
    // FIXME: g_host_wakeup & g_host_state duplicate??
    if (!g_host_wakeup)
    {
        RT_BT_LOG(RED, D3D_MSG_23, 1, status);
    }
    else
#endif
    {
        UCHAR event_parameter[2];
        event_parameter[0] = 0x28; // sub code
        event_parameter[1] = status;
        hci_generate_event(HCI_VENDOR_SPECIFIC_EVENT, event_parameter, 2);
        RT_BT_LOG(YELLOW,BT_3DD_MSG_SEND_VENDOR_EVT_SYNC_TOGGLE, 1, status);
    }
}


