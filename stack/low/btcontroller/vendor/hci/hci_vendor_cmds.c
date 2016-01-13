/***************************************************************************
 Copyright (C) MindTree Ltd.
 This module is a confidential and proprietary property of MindTree and
 a possession or use of this module requires written permission of MindTree.
 ***************************************************************************/

/********************************* Logger *************************/
enum { __FILE_NUM__= 34 };
/********************************* Logger *************************/

#include "bt_fw_os.h"
#include "bt_fw_hci_defines.h"
#include "hci_vendor_internal.h"
#include "hci_vendor_defines.h"
#include "lmp_vendor_internal.h"
#include "mpal.h"
#include "crypto.h"
#include "vendor.h"
#include "platform.h"
#include "bz_nvconfig.h"
#include "bz_auth.h"
#include "mem.h"
#include "pta.h"
#include "mailbox.h"
#ifdef MWS_VERIFICATION
#include "mws.h"
#include "mws_imp.h"
#include "new_io.h"
#include "bb_driver.h"
#endif
#ifdef _DAPE_SEND_PCA_ADJ_BY_VENDOR_CMD
#include "lmp_4_1.h"
#endif

#ifdef _DAPE_TEST_DISABLE_AFH_POWER_CTRL_FOR_PING_TEST_BY_VENDOR_CMD
#include "lmp_3_0.h"
#endif
#ifdef _SUPPORT_CSB_RECEIVER_
#include "bt_3dd.h"
#endif
#ifdef _ENABLE_BTON_POWER_SAVING_
#include "power_control.h"
#include "dma_usb.h"
#endif
#include "pta.h"
#include "h5.h"
#include "spi.h"
#include "timer.h"
#include "gpio.h"
#ifdef _DAPE_TEST_KILL_LE_CE_BY_VENDOR_CMD
#include "le_hw_reg.h"
#endif
#ifdef _VENDOR_RESET_BB_
#include "le_ll.h"
#endif

#ifdef _BT_ONLY_
#include "new_io.h"
#include "efuse.h"
#endif

#ifdef _SUPPORT_COMBO_VENDER_READ_PAGE0_
#include "new_io.h"
#endif

#ifdef CONFIG_TV_POWERON
#include "tv.h"
#endif

#include "lmp_vendor_defines.h"
#include "lc_internal.h"

#ifdef _PLC_TEST_MODE_ENABLE_
#include "plc.h"
#endif

#include "fm.h"

#include "le_hci_4_0.h"
#include "uart.h"

#if defined (_TEST_PI_WRITE_PAGE_3_BY_VENDOR_CMD) || defined (_TEST_ADAPTIVITY_FUNC_2)
UINT8 g_enable_pi_write_page_3 = 0;
UINT8 g_enable_pi_read_page_3 = 0;
UINT8 g_enable_pi_read_after_tx = 0;
UINT8 g_enable_change_tx_gain = 0;
UINT8 g_set_no_tx = 0;
#endif

#ifdef _DAPE_TEST_SEND_PING_RES_BY_VENDOR_CMD
UINT8 g_send_ping_response = 1;
#endif
#ifdef _DAPE_TEST_ALWAYZ_NAK_ESCO_BY_VENDOR_CMD
extern UINT8 g_nak_esco;
#endif
#ifdef _DAPE_TEST_GEN_FAKE_ESCO_DATA
extern UINT16 g_handle;
UINT8 g_change = 0;
HCI_SYNC_DATA_PKT *pgen_synchronous_pkt = NULL;
extern UINT8 g_change_flag;
UINT8 g_gen_fake_esco_data = 0;
#endif
#ifdef _SECURE_CONN_REFRESH_KEY_BY_VENDOR_CMD
UINT8 g_refresh_key_en = 0;
#endif
#ifdef _SECURE_CONN_REFRESH_KEY_WHEN_CONTINUOUS_MIC
extern UINT8 g_sc_refresh_key_when_continuous_mic_err;
#endif
#ifdef _DAPE_TEST_FRAME_SYNC_UPDATE_CHOOSE_WAY
extern UINT8 g_modify_way;
#endif

#ifdef _DAPE_TEST_SEND_ZERO_LEN_PKT_BY_VENDOR_CMD
UINT8 g_send_zero_len = 0;
#endif
#ifdef _DAPE_TEST_CHK_MWS_FRAME_SYNC
extern UINT8 g_gen_frame_sync;
extern UINT8 g_modify_clk;
extern UINT8 g_modify_clk_value;
extern UINT8 g_modify_clk_cnt;
#endif

#ifndef _SUPPORT_MSFT_BT_HCI_EXTENSION_FINAL_SPEC_
#ifdef _SUPPORT_MSFT_BT_HCI_EXTENSION_FIX_
UINT8 filter_backup;
#endif
#endif

#if (defined(_SUPPORT_SET_PCM_DISC_IO_VENDOR_CMD_)&& defined(_SUPPORT_BT_CONTROL_PCM_MUX_))
extern UINT32 u32BtGpioStateBkup;
extern UINT8 u8BtVenBtGpioCtrlBkup;
#endif

extern API_RESULT lc_baseband_reset(void);
#ifdef VER_3_0
API_RESULT lmp_generate_power_ctrl_req_pdu(UINT16 ce_index, UCHAR power_adjustment);
#endif
#ifdef COMPILE_PARK_MODE
extern UCHAR lmp_validate_beacon_parameters(LMP_PDU_PKT *lmp_pdu_ptr);
#endif
#ifdef _DAPE_TEST_AUTO_CONN
extern SECTION_SRAM LEGACY_WHITE_LIST conn_white_list[LEGACY_MAX_WHITE_LIST];
extern SECTION_SRAM UINT8 num_of_white_list_device;
#endif

#ifdef _UART_H5
extern UINT16 hci_uart_h5_read_ln_state(void);
#endif
#ifdef _CSB_RX_SET_XTOL_BY_VENDOR_CMD
extern UINT16 g_beacon_rx_xtol;
#endif

#ifdef _IS_ASIC_
#ifdef _RTK_8723A_B_CUT_NEW_HCI_VENDOR_CMD_
extern TIMER_ID g_rtl8723_btrf_thermal_value_timer;
extern void rtl8723_btrf_UpdateThermalValueTimer(void);
#endif
#endif
#ifdef _CSB_RX_DBG_LOG
UINT8 g_csb_rx_dbg_log = 0;
#endif
#ifdef _DAPE_TEST_CSB_RX_SELF_CALC_CLK_OFST_BY_VENDOR_CMD
UINT8 g_self_calc_clk_ofst = 0;
#endif
#ifdef _DAPE_TEST_DISABLE_DM1_FOR_CCPT_BY_VENDOR_CMD
UINT8 g_disable_dm1 = 0;
#endif
#ifdef _SECURE_CONN_TEST_CHK_ACLU_DATA_BY_VENDOR_CMD
UINT8 g_chk_secure_conn_data = 0;
#endif

/********************************* Logger *************************/
/********************************* Logger *************************/

#if defined(RT_VENDOR_CMDS)
UINT16 hci_vendor_g_read_address;
UINT8  hci_vendor_g_read_len;
UINT32 hci_vendor_spi_address = 0;
UINT16 hci_vendor_spi_len = 0;

UINT8 dbg_vendor_log_interface = VENDOR_LOG_PACKET_TYPE_INVALID;
UINT16 dbg_vendor_log_conn_handle = 0;
UINT8 *dbg_vendor_log_buf = NULL;
UINT8 *dbg_vendor_set_log_complete_event_buf = (UINT8*)0xDEADBEEF;

/*Connection entity database */
extern LMP_CONNECTION_ENTITY  lmp_connection_entity[LMP_MAX_CE_DATABASE_ENTRIES] ;

#ifdef USE_BB_SLOT_TIMER
UCHAR bb_slot_timer_enable = 0x0;
#endif
#ifdef _DAPE_TEST_DISABLE_EDR_BY_VENDOR_CMD_FOR_BR_PKT
UCHAR g_ptt_enabled = 0x01;
#endif

UCHAR rf_transmit_test_params[11];
UCHAR hci_vendor_g_read_radio_address;
#ifdef _NEW_MODEM_PI_ACCESS_
UCHAR hci_vendor_g_read_radio_modem_page;
#endif

#if (defined(_ENABLE_32K_CLK_WAKE_UP_ISR_) || defined(_GPIO_POWER_SEQUENCE_ENABLE))
UINT16 power_ctr_reg = 0;
UINT32 sys_clk_reg = 0;
#endif

#ifndef IS_BTSOC
extern UINT32 rom_code_patch_start_address;
extern UINT8 rom_code_patch_block_index;
#endif

#ifdef _ROM_CODE_PATCHED_
PF_ROM_CODE_PATCH_FUNC rcp_hci_vendor_cmd_func = NULL;
//PF_ROM_CODE_PATCH_FUNC rcp_hci_vendor_host_enter_sleep_mode = NULL;
PF_ROM_CODE_PATCH_FUNC rcp_hci_handle_vendor_write_bb_reg_cmd = NULL;
#endif

#ifdef POWER_SAVE_FEATURE
extern UINT16 lc_power_ctr_config;
#endif

UINT32 g_hci_vendor_read_data;
UINT8 g_hci_vendor_read_data_len;

#ifdef _SUPPORT_VENDER_READ_SIE_
UINT16 g_hci_vendor_read_sie_data;
UINT8 g_hci_vendor_read_sie_data_len;
#endif

#ifdef _CCH_RTL8723A_B_CUT
// _CCH_ISSC_VENDOR_
extern ISSC_SEC_STRUCT issc_sec;
#endif

#ifdef _8821A_BTON_DESIGN_
HCI_CMD_PKT *g_hci_cmd_ptr_for_evt_gen;
#endif

#ifdef _SUPPORT_VENDER_READ_SIE_
UCHAR hci_handle_vendor_read_sie_reg_cmd(HCI_CMD_PKT *hci_cmd_ptr)
{
    UINT8 type = hci_cmd_ptr->cmd_parameter[0];
    UINT8 addr = hci_cmd_ptr->cmd_parameter[1];

    if ((type!=READ_SIE_BYTE)&&(type!=READ_SIE_WORD))
    {
        return INVALID_HCI_COMMAND_PARAMETERS_ERROR;
    }
    g_hci_vendor_read_sie_data_len = (1<<type);
    g_hci_vendor_read_sie_data = safe_indirect_read_sie(type, addr);
    //g_hci_vendor_read_data_len = (1<<type);
    //g_hci_vendor_read_data = indirect_read_sie(type, addr);

    return HCI_COMMAND_SUCCEEDED;
}
#endif

UCHAR hci_handle_vendor_read_write_reg_cmd(HCI_CMD_PKT *hci_cmd_ptr)
{
    UINT8 num_byte_grade = (hci_cmd_ptr->cmd_parameter[0] >> 4) & 0x03;
    UINT8 type = hci_cmd_ptr->cmd_parameter[0] & 0x03;
    UINT32 addr = *(UINT32*)&hci_cmd_ptr->cmd_parameter[1];
    UINT8 *data = &hci_cmd_ptr->cmd_parameter[5];

    if (num_byte_grade == 3)
    {
        num_byte_grade = 2;
    }

    g_hci_vendor_read_data_len = 1 << num_byte_grade;

    if (addr & (g_hci_vendor_read_data_len - 1))
    {
        return INVALID_HCI_COMMAND_PARAMETERS_ERROR;
    }

    switch (type)
    {
#if defined(_BT_ONLY_) || defined(_SUPPORT_COMBO_RW_PAGE0_)
    case 2:
        if (HCI_VENDOR_WRITE == hci_cmd_ptr->cmd_opcode)
        {
            /* write system register */
            if (num_byte_grade == 0)
            {
                WR_8BIT_SYSON_IO(addr, *data);
            }
            else if (num_byte_grade == 1)
            {
                WR_16BIT_SYSON_IO(addr, *(UINT16*)data);
            }
            else
            {
                WR_32BIT_SYSON_IO(addr, *(UINT32*)data);
            }
        }
        else
        {
            /* read system register */
            if (num_byte_grade == 0)
            {
                g_hci_vendor_read_data = RD_8BIT_SYSON_IO(addr);
            }
            else if (num_byte_grade == 1)
            {
                g_hci_vendor_read_data = RD_16BIT_SYSON_IO(addr);
            }
            else
            {
                g_hci_vendor_read_data = RD_32BIT_SYSON_IO(addr);
            }
        }
        break;
#else
#ifndef _REMOVE_INDIRECT_READ_PAGE0_
#if _SUPPORT_COMBO_VENDER_READ_PAGE0_
    case 2:
        if (HCI_VENDOR_READ == hci_cmd_ptr->cmd_opcode)
        {

            /* read system register */
            if (num_byte_grade == 0)
            {
                g_hci_vendor_read_data = RD_8BIT_COMBO_SYSON_IO(addr);
            }
            else if (num_byte_grade == 1)
            {
                g_hci_vendor_read_data = RD_16BIT_COMBO_SYSON_IO(addr);
            }
            else
            {
                g_hci_vendor_read_data = RD_32BIT_COMBO_SYSON_IO(addr);
            }
            //RT_BT_LOG(WHITE,YL_DBG_HEX_3,3, num_byte_grade, addr, g_hci_vendor_read_data);

        }
    break;
#endif /* #if _SUPPORT_COMBO_VENDER_READ_PAGE0_ */

#endif /* #ifndef _REMOVE_COMBO_INDIRECT_READ_PAGE0_ */
#endif /* #if defined(_BT_ONLY_) || defined(_SUPPORT_COMBO_RW_PAGE0_)*/

    case 1:
        if (HCI_VENDOR_WRITE == hci_cmd_ptr->cmd_opcode)
        {
            /* write memory */
            if (num_byte_grade == 0)
            {
               *(UINT8*)addr =  *data;
            }
            else if (num_byte_grade == 1)
            {
                *(UINT16*)addr =  *(UINT16*)data;
            }
            else
            {
                *(UINT32*)addr =  *(UINT32*)data;
            }
        }
        else
        {
            /* read memory */
            if (num_byte_grade == 0)
            {
                g_hci_vendor_read_data = *(UINT8*)addr;
            }
            else if (num_byte_grade == 1)
            {
                g_hci_vendor_read_data = *(UINT16*)addr;
            }
            else
            {
                g_hci_vendor_read_data = *(UINT32*)addr;
            }
        }
        break;

    case 0:
        if (HCI_VENDOR_WRITE == hci_cmd_ptr->cmd_opcode)
        {
            /* write IO */
            if (num_byte_grade == 0)
            {
                WR_8BIT_IO(addr, 0, *data);
            }
            else if (num_byte_grade == 1)
            {
                WR_16BIT_IO(addr, 0, *(UINT16*)data);
            }
            else
            {
                WR_32BIT_IO(addr, 0, *(UINT32*)data);
            }
        }
        else
        {
            /* read IO */
            if (num_byte_grade == 0)
            {
                g_hci_vendor_read_data = RD_8BIT_IO(addr, 0);
            }
            else if (num_byte_grade == 1)
            {
                g_hci_vendor_read_data = RD_16BIT_IO(addr, 0);
            }
            else
            {
                g_hci_vendor_read_data = RD_32BIT_IO(addr, 0);
            }
        }
        break;

    default:
        return INVALID_HCI_COMMAND_PARAMETERS_ERROR;
    }

    return HCI_COMMAND_SUCCEEDED;
}

UCHAR hci_handle_vendor_write_bb_reg_cmd(HCI_CMD_PKT *hci_cmd_ptr)
{
    UCHAR write_address;
    UINT16 temp;
    UINT16 write_val;

#ifdef _ROM_CODE_PATCHED_
    if (rcp_hci_handle_vendor_write_bb_reg_cmd != NULL)
    {
        if (rcp_hci_handle_vendor_write_bb_reg_cmd((void*)hci_cmd_ptr))
        {
            return HCI_COMMAND_SUCCEEDED;
        }
    }
#endif

    write_address = (UINT8)hci_cmd_ptr->cmd_parameter[0];
    temp = (hci_cmd_ptr->cmd_parameter[1]<<8) | hci_cmd_ptr->cmd_parameter[0];
    write_val = (hci_cmd_ptr->cmd_parameter[3]<<8) | hci_cmd_ptr->cmd_parameter[2];

    /* page 0: Bluewiz (0xb6000000)
       page 1: HCI DMA (0xb4000000)
       page 2: Timer (0xb0004000)
       page 3: GPIO (0xb0006000)
       page 4: UART (0xb0000000)
       page 5: H-UART (0xb5000000)
       page 6: SPIC (0xb0008000)
       page 7: LE (0xb6001000)
       page 8: Vendor/BZDMA (0xb000a000) */

    UINT8 page = temp >> 12;
    UINT16 reg_offset = temp & 0xFFF;
    UINT32 base;

    if (reg_offset & 0x01)
    {
#ifndef _ROM_CODE_PATCHED_REDUCE_
        RT_BT_LOG(RED, MSG_HCI_VENDOR_CMDS_ERR, 2, page, reg_offset);
#endif
        return INVALID_HCI_COMMAND_PARAMETERS_ERROR;
    }

    switch (page)
    {
    case 1:
        base = BT_DMA_REG_BASE_ADDR;
        break;
    case 2:
        base = TIMER_BASE_ADDR;
        break;
    case 3:
        base = GPIO_BASE_ADDRESS;
        break;
    case 4:
        base = UART_BASE_ADDR;
        break;
    case 5:
        base = BT_DMA_UART_BASE_ADDRESS;
        break;
    case 6:
        base = SPI_REG_BASE;
        break;
    case 8:
        base = VENDOR_BASE_ADDRESS;
        break;
    case 7:
        base = LE_REG_BASE;
        break;
#ifdef _YL_NEW_MODEM_SRAM_DEBUG
    case 0xF:
        base = 0xffffffff;
        switch (reg_offset)
        {
            case 0x00: // init as g_modem_sram_debug_xdh5_trig_crc_ok = 0//
#ifdef _DAPE_MODEM_SRAM_DBG_VER2
                {
                    UINT8 mode;
                    mode = hci_cmd_ptr->cmd_parameter[2];
                    g_modem_sram_debug_le_trig_crc_ok = (hci_cmd_ptr->cmd_parameter[3] & BIT0);
                    g_modem_sram_debug_modem_test = (hci_cmd_ptr->cmd_parameter[3] & BIT1);              
                    g_modem_sram_debug_le_normal_link_trig = (hci_cmd_ptr->cmd_parameter[3] & BIT7);
                    rtl8821_btrf_modem_sram_debug_init(g_modem_sram_debug_array, mode);
                }
#else
                rtl8821_btrf_modem_sram_debug_init(g_modem_sram_debug_array);
#endif
                break;
            case 0x02: // re-init (re-capture) //
                g_modem_sram_debug_en = 1;
                g_modem_sram_debug_captured_flag = 0;
                rtl8821_btrf_modem_sram_debug_set_en(1);
                break;
            case 0x04: // stop //
                g_modem_sram_debug_en = 0;
                g_modem_sram_debug_captured_flag = 1;
                rtl8821_btrf_modem_sram_debug_set_en(0);
                break;
            case 0x06:
                g_modem_sram_debug_log_en = (write_val!=0); // default: 1 //
                break;
            case 0x08:
                g_modem_sram_debug_xdh5_trig_en = (write_val!=0); // default: 1 //
                break;
            case 0x0a:
                g_modem_sram_debug_xdh5_trig_crc_ok = (write_val!=0); // default: 0 //
                break;
            case 0x0c:
                g_modem_sram_debug_le_trig_en = (write_val!=0); // default: 1 //
                break;
            case 0x0e:
                g_modem_sram_debug_le_trig_crc_ok = (write_val!=0); // default: 0 //
                break;
            case 0x10:
                g_modem_sram_debug_xdh5_xdh3_3dh1_error_log_en = (write_val!=0); // default: 1 //
                break;
            case 0x12:
                g_modem_sram_debug_en = (write_val!=0); // default: 0 //
                break;
            case 0x14:
                g_modem_sram_debug_captured_flag = (write_val!=0); // default: 0 //
                break;
            default:
                break;
        }
        break;
#endif
    default:
        base = BB_BASE_ADDR;
        break;
    }

    if (base!=0xffffffff)
    {
        WR_16BIT_IO(base, reg_offset, write_val);
    }
#ifndef _ROM_CODE_PATCHED_REDUCE_
    RT_BT_LOG(WHITE, MSG_HCI_VENDOR_CMDS_WR, 3,
                    base, reg_offset, write_val);
#endif
    return HCI_COMMAND_SUCCEEDED;
}

UCHAR hci_handle_vendor_set_log_enable_cmd(HCI_CMD_PKT *hci_cmd_ptr)
{
    if (hci_cmd_ptr->param_total_length == 4)
    {
        UINT8 type;
        UINT16 conn_handle;
        UINT8 enable;

        type = hci_cmd_ptr->cmd_parameter[0];
        conn_handle = hci_cmd_ptr->cmd_parameter[1] |
                      (hci_cmd_ptr->cmd_parameter[2] << 8);
        enable = hci_cmd_ptr->cmd_parameter[3];
        if ((type > 1) || (enable > 1))
        {
            return INVALID_HCI_COMMAND_PARAMETERS_ERROR;
        }

        DEF_CRITICAL_SECTION_STORAGE;
        MINT_OS_ENTER_CRITICAL();

        dbg_vendor_set_log_complete_event_buf = (UINT8*)0xDEADBEEF;

        if (dbg_vendor_log_buf != NULL)
        {
            if (dbg_vendor_log_interface == VENDOR_LOG_PACKET_TYPE_EVENT)
            {
                OS_FREE_BUFFER(tx_table[HCI_EVENT_HANDLER_TASK].pool_handle,
                               dbg_vendor_log_buf);
            }
            else if (dbg_vendor_log_interface == VENDOR_LOG_PACKET_TYPE_ACL)
            {
                OS_FREE_BUFFER(tx_table[ACL_DATA_HANDLER_TASK].pool_handle,
                               dbg_vendor_log_buf);
                os_free_reserved_buffer();
            }
            dbg_vendor_log_buf = NULL;
        }

        if (enable)
        {
            dbg_vendor_log_interface = type;

            if (type == VENDOR_LOG_PACKET_TYPE_ACL)
            {
                dbg_vendor_log_conn_handle = conn_handle;
            }
        }
        else
        {
            dbg_vendor_log_interface = VENDOR_LOG_PACKET_TYPE_INVALID;
            dbg_vendor_log_conn_handle = 0;
        }

        MINT_OS_EXIT_CRITICAL();

        return HCI_COMMAND_SUCCEEDED;
    }

    return INVALID_HCI_COMMAND_PARAMETERS_ERROR;
}

UCHAR hci_handle_vendor_set_mute_enable_cmd(HCI_CMD_PKT *hci_cmd_ptr)
{
    UINT8 enable;

    enable = hci_cmd_ptr->cmd_parameter[0];
    if (hci_cmd_ptr->param_total_length == 1)
    {
        if (enable > 1)
        {
            return INVALID_HCI_COMMAND_PARAMETERS_ERROR;
        }
        DEF_CRITICAL_SECTION_STORAGE;
        MINT_OS_ENTER_CRITICAL();

        if (enable == 1)
        {
            otp_str_data.bt_auto_choose_le_intv = 0x00;
            otp_str_data.bt_master_piconet_fea[1] = (((FMD_PAGE0_BYTE1) & (~(1<<2))) | (LMP_FEATURE_MASK_BIT(0, 1, 2, OFF))); //EFUSE[83]

#ifdef _DAPE_TEST_NEW_HW
            otp_str_data.bt_priority_3 = (BIT11|BIT12|BIT14); //0x5800
#endif
        }
        else
        {
            otp_str_data.bt_auto_choose_le_intv = (otp_str_data.bt_func_support_policy_ext & BIT14);
            otp_str_data.bt_master_piconet_fea[1] = (((FMD_PAGE0_BYTE1) & (~(1<<2))) | (LMP_FEATURE_MASK_BIT(0, 1, 2, ON))); //EFUSE[83]

#ifdef _DAPE_TEST_NEW_HW
            otp_str_data.bt_priority_3 = (BIT11|BIT12); //0x1800
#endif
        }
        MINT_OS_EXIT_CRITICAL();
        return HCI_COMMAND_SUCCEEDED;
    }

    return INVALID_HCI_COMMAND_PARAMETERS_ERROR;
}

#if defined(_BT_ONLY_) && defined(_PG_EFUSE_VIA_HCI_VENDOR_COMMAND_)
UCHAR hci_handle_read_write_efuse_data_cmd(HCI_CMD_PKT *hci_cmd_ptr)
{
    UINT8 bank;
    UINT16 addr;
    UINT16 len;
    UINT8 *data;
    UINT16 i;
    UINT8 status;

    status = INVALID_HCI_COMMAND_PARAMETERS_ERROR;

    bank = hci_cmd_ptr->cmd_parameter[0];
    addr = hci_cmd_ptr->cmd_parameter[1] |
          (hci_cmd_ptr->cmd_parameter[2] << 8);
    len = hci_cmd_ptr->cmd_parameter[3] |
         (hci_cmd_ptr->cmd_parameter[4] << 8);

    do
    {

        if ((bank > 2) || (len > 250))
        {
            /* check efuse bank and valid length */
            break;
        }

        /* check RLE0557 bt only efuse configuration */
        if (bank == 0)
        {
            if (addr > 0xff)
            {
                break;
            }

            if ((addr + len) > 0x100)
            {
                break;
            }
        }
        else
        {
            if (addr > 0x1ff)
            {
                break;
            }

            if ((addr + len) > 0x200)
            {
                break;
            }
        }

        efuse_manger.app_bank = bank;
        efuse_manger.app_start_addr = addr;
        efuse_manger.app_len = len;

        status = HCI_COMMAND_SUCCEEDED;

        if (len  == 0)
        {
            /* no any data so only select efuse bank ... */
            efuse_page0_set_bank(bank);
            break;
        }

        if (hci_cmd_ptr->cmd_opcode == HCI_VENDOR_WRITE_EFUSE_DATA)
        {
            data = &hci_cmd_ptr->cmd_parameter[5];

            for (i = 0; i < len; i++)
            {
                efuse_page0_write(bank, addr, data[i]);

                if (efuse_manger.cur_status != EFUSE_PG_STATUS_OK)
                {
                    status = HARDWARE_FAILURE_ERROR;
                    break;
                }
                addr++;
            }
            efuse_manger.app_len = i;
        }
    }
    while (0);

    return status;
}

#endif

#ifdef _SUPPORT_MSFT_BT_HCI_EXTENSION_
UINT16 g_hci_vs_msft_sub_opcode = 0;
UINT16 g_hci_vs_msft_le_monitor_handle = 0;
UINT8 g_msft_event_prefix[8] = {0x23, 0x79, 0x54, 0x33, 0x77, 0x88, 0x97, 0x68};

/**************************************************************************
 * Function     : hci_handle_vs_msft_read_supported_features_cmd
 *
 * Description  : This function is used to handle
 *                HCI_VS_MSFT_Read_Supported_Features.
 *                Please refer chap 3.1 of MS final BT HCI extensions
 *                spec to know the detailed definition.
 *
 * Parameters   : hci_cmd_ptr Pointer to the command packet.
 *
 * Returns      : HCI_COMMAND_SUCCEEDED or HCI error code
 *
 *************************************************************************/
UCHAR hci_handle_vs_msft_read_supported_features_cmd(HCI_CMD_PKT *hci_cmd_ptr)
{
    UINT8 status = HCI_COMMAND_SUCCEEDED;

    if (hci_cmd_ptr->param_total_length != 1)
    {
        return INVALID_HCI_COMMAND_PARAMETERS_ERROR;
    }

    return status;
}

/**************************************************************************
 * Function     : hci_handle_vs_msft_monitor_rssi_cmd
 *
 * Description  : This function is used to handle HCI_VS_MSFT_Monitor_RSSI.
 *                Please refer chap 3.2 of MS final BT HCI extensions
 *                spec to know the detailed definition.
 *
 * Parameters   : hci_cmd_ptr Pointer to the command packet.
 *
 * Returns      : HCI_COMMAND_SUCCEEDED or HCI error code
 *
 *************************************************************************/
UCHAR hci_handle_vs_msft_monitor_rssi_cmd(HCI_CMD_PKT *hci_cmd_ptr)
{
    UINT8 status = HCI_COMMAND_SUCCEEDED;
    UINT16 conn_handle;
    INT8 rssi_threshold_high;
    INT8 rssi_threshold_low;
    UINT8 rssi_threshold_low_intv;
    UINT8 rssi_sample_period;
    UINT8 index;

    if (hci_cmd_ptr->param_total_length != 7)
    {
        return INVALID_HCI_COMMAND_PARAMETERS_ERROR;
    }

    conn_handle = hci_cmd_ptr->cmd_parameter[1] |
                  (hci_cmd_ptr->cmd_parameter[2] << 8);
    rssi_threshold_high = (INT8)hci_cmd_ptr->cmd_parameter[3];
    rssi_threshold_low = (INT8)hci_cmd_ptr->cmd_parameter[4];
    rssi_threshold_low_intv = hci_cmd_ptr->cmd_parameter[5];
    rssi_sample_period = hci_cmd_ptr->cmd_parameter[6];

    if ((rssi_threshold_low_intv > 0x3c) || (rssi_threshold_low_intv == 0))
    {
        return INVALID_HCI_COMMAND_PARAMETERS_ERROR;
    }

    if (conn_handle > LL_HCI_MAX_CONN_HANDLE)
    {
        return INVALID_HCI_COMMAND_PARAMETERS_ERROR;
    }
    else if (conn_handle >= LL_HCI_MIN_CONN_HANDLE)
    {
        LL_CONN_HANDLE_UNIT *phandle;
        phandle = ll_fw_search_handle_unit_via_conn_handle(conn_handle);
        if (phandle == NULL)
        {
            return NO_CONNECTION_ERROR;
        }

        if (((rssi_threshold_high < -127) || (rssi_threshold_high > 20)) ||
            ((rssi_threshold_low < -127) || (rssi_threshold_low > 20)) ||
            (rssi_threshold_low > rssi_threshold_high))
        {
            return INVALID_HCI_COMMAND_PARAMETERS_ERROR;
        }

        if (ll_rssi_manager.bm_used_handle & (1 << phandle->unit_id))
        {
            return COMMAND_DISALLOWED_ERROR;
        }

        ll_rssi_manager.bm_used_handle |= 1 << phandle->unit_id;
        index = 16 + phandle->unit_id;
    }
    else
    {
        UINT16 ce_index;
        if (LMP_GET_CE_INDEX_FROM_CONN_HANDLE(conn_handle,
                                               &ce_index) != API_SUCCESS)
        {
            return NO_CONNECTION_ERROR;
        }

        if (rssi_threshold_low > rssi_threshold_high)
        {
            return INVALID_HCI_COMMAND_PARAMETERS_ERROR;
        }

        if (ll_rssi_manager.bm_used_legacy_handle & (1 << ce_index))
        {
            return COMMAND_DISALLOWED_ERROR;
        }

        ll_rssi_manager.bm_used_legacy_handle |= 1 << ce_index;
        index = ce_index;
    }

    if (status == HCI_COMMAND_SUCCEEDED)
    {
        ll_rssi_manager.entry[index].min_rssi_thres = rssi_threshold_low;
        ll_rssi_manager.entry[index].max_rssi_thres = rssi_threshold_high;
        ll_rssi_manager.entry[index].min_rssi_thres_intv = rssi_threshold_low_intv * 10;

        ll_rssi_manager.entry[index].notified = 0;
        ll_rssi_manager.entry[index].start_periodic_timer = FALSE;
        ll_rssi_manager.entry[index].sample_period = rssi_sample_period;
        ll_rssi_manager.entry[index].sample_counter_of_tick = 0;
        ll_rssi_manager.entry[index].rssi_count = 0;
        ll_rssi_manager.entry[index].rssi_sum = 0;

        ll_rssi_manager.entry[index].conn_handle = conn_handle;
        ll_rssi_manager.entry[index].rssi_thres_counter = 0;
    }
    return status;
}

/**************************************************************************
 * Function     : hci_handle_vs_msft_cancel_monitor_rssi_cmd
 *
 * Description  : This function is used to handle HCI_VS_MSFT_Cancel_Monitor_RSSI.
 *                Please refer chap 3.3 of MS final BT HCI extensions
 *                spec to know the detailed definition.
 *
 * Parameters   : hci_cmd_ptr Pointer to the command packet.
 *
 * Returns      : HCI_COMMAND_SUCCEEDED or HCI error code
 *
 *************************************************************************/
UCHAR hci_handle_vs_msft_cancel_monitor_rssi_cmd(HCI_CMD_PKT *hci_cmd_ptr)
{
    UINT8 status = HCI_COMMAND_SUCCEEDED;
    UINT16 conn_handle;

    if (hci_cmd_ptr->param_total_length != 3)
    {
        return INVALID_HCI_COMMAND_PARAMETERS_ERROR;
    }

    conn_handle = hci_cmd_ptr->cmd_parameter[1] |
                  (hci_cmd_ptr->cmd_parameter[2] << 8);


    if (conn_handle > LL_HCI_MAX_CONN_HANDLE)
    {
        return INVALID_HCI_COMMAND_PARAMETERS_ERROR;
    }
    else if (conn_handle >= LL_HCI_MIN_CONN_HANDLE)
    {
        LL_CONN_HANDLE_UNIT *phandle;
        phandle = ll_fw_search_handle_unit_via_conn_handle(conn_handle);
        if (phandle == NULL)
        {
            return NO_CONNECTION_ERROR;
        }

        ll_rssi_manager.bm_used_handle &=  ~(1 << phandle->unit_id);
    }
    else
    {
        UINT16 ce_index;
        if (LMP_GET_CE_INDEX_FROM_CONN_HANDLE(conn_handle,
                                               &ce_index) != API_SUCCESS)
        {
            return NO_CONNECTION_ERROR;
        }

        ll_rssi_manager.bm_used_legacy_handle &=  ~(1 << ce_index);
    }

    return status;
}

/**************************************************************************
 * Function     : hci_handle_vs_msft_le_monitor_adv_cmd
 *
 * Description  : This function is used to handle HCI_VS_MSFT_LE_Monitor_Advertisement.
 *                Please refer chap 3.4 of MS final BT HCI extensions
 *                spec to know the detailed definition.
 *
 * Parameters   : hci_cmd_ptr Pointer to the command packet.
 *
 * Returns      : HCI_COMMAND_SUCCEEDED or HCI error code
 *
 *************************************************************************/
UCHAR hci_handle_vs_msft_le_monitor_adv_cmd(HCI_CMD_PKT *hci_cmd_ptr)
{
    UINT8 status = HCI_COMMAND_SUCCEEDED;
    INT8 rssi_threshold_high;
    INT8 rssi_threshold_low;
    UINT8 rssi_threshold_low_intv;
    UINT8 rssi_sample_period;
    UINT8 flags;
    UINT8 handle = 0;
    UINT8 i;


    if (hci_cmd_ptr->param_total_length < 7)
    {
        return INVALID_HCI_COMMAND_PARAMETERS_ERROR;
    }

    rssi_threshold_high = (INT8)hci_cmd_ptr->cmd_parameter[1];
    rssi_threshold_low = (INT8)hci_cmd_ptr->cmd_parameter[2];
    rssi_threshold_low_intv = hci_cmd_ptr->cmd_parameter[3];
    rssi_sample_period = hci_cmd_ptr->cmd_parameter[4];
    flags = hci_cmd_ptr->cmd_parameter[5];

    if (((rssi_threshold_high < -127) || (rssi_threshold_high > 20)) ||
        ((rssi_threshold_low < -127) || (rssi_threshold_low > 20)) ||
        (rssi_threshold_low > rssi_threshold_high))
    {
        return INVALID_HCI_COMMAND_PARAMETERS_ERROR;
    }

    if ((rssi_threshold_low_intv > 0x3c) || (rssi_threshold_low_intv == 0))
    {
        return INVALID_HCI_COMMAND_PARAMETERS_ERROR;
    }

    if ((flags == 0) || (flags > LE_MSFT_MON_RSSI_FLAG_BDADDR))
    {
        return INVALID_HCI_COMMAND_PARAMETERS_ERROR;
    }

#ifndef _SUPPORT_MSFT_BT_HCI_EXTENSION_FINAL_SPEC_
    if ((ll_rssi_manager.le_adv_monitor_type != LE_MSFT_MON_RSSI_FLAG_NONE) &&
        (ll_rssi_manager.le_adv_monitor_type != flags))
    {
        return COMMAND_DISALLOWED_ERROR;
    }

#ifdef _SUPPORT_MSFT_BT_HCI_EXTENSION_FIX_
	filter_backup = ll_manager.scan_unit.filter_policy;
	ll_manager.scan_unit.filter_policy = 0;
	LE_REG_S_SET reg;
	reg.value = RD_LE_REG(LE_REG_SCAN_CONTROL);
	reg.scan_ctrl.scan_filter = ll_manager.scan_unit.filter_policy;
	WR_LE_REG(LE_REG_SCAN_CONTROL, reg.value);
#endif
#endif /* end of #ifndef _SUPPORT_MSFT_BT_HCI_EXTENSION_FINAL_SPEC_ */

	switch(flags)
    {
    case LE_MSFT_MON_RSSI_FLAG_UUID:
        {
            UINT8 uuid_type;
            const UINT8 uuid_len_array[4] = {0, 2, 4, 16};

            uuid_type = hci_cmd_ptr->cmd_parameter[6];
            if ((uuid_type == 0) || (uuid_type > 3))
            {
                return INVALID_HCI_COMMAND_PARAMETERS_ERROR;
            }

            if (hci_cmd_ptr->param_total_length !=
                                    (7 + uuid_len_array[uuid_type]))
            {
                return INVALID_HCI_COMMAND_PARAMETERS_ERROR;
            }

            handle = rssi_app_msft_get_rssi_monitor_entry(flags, 1);
            if (handle == 0xff)
            {
                return MEMORY_FULL_ERROR;
            }

#ifdef _SUPPORT_MSFT_BT_HCI_EXTENSION_FINAL_SPEC_
            ll_rssi_manager.ms_adv_unit[handle].le_adv_uuid.type = uuid_type;
            memcpy(ll_rssi_manager.ms_adv_unit[handle].le_adv_uuid.data,
                   &hci_cmd_ptr->cmd_parameter[7], uuid_len_array[uuid_type]);
#else
            ll_rssi_manager.le_adv_uuid[handle].type = uuid_type;
            memcpy(ll_rssi_manager.le_adv_uuid[handle].data,
                   &hci_cmd_ptr->cmd_parameter[7], uuid_len_array[uuid_type]);
#endif
        }
        break;

    case LE_MSFT_MON_RSSI_FLAG_IRK:
        if (hci_cmd_ptr->param_total_length != 22)
        {
            return INVALID_HCI_COMMAND_PARAMETERS_ERROR;
        }

        handle = rssi_app_msft_get_rssi_monitor_entry(flags, 1);
        if (handle == 0xff)
        {
            return MEMORY_FULL_ERROR;
        }

#ifdef _SUPPORT_MSFT_BT_HCI_EXTENSION_FINAL_SPEC_
        memcpy(&ll_rssi_manager.ms_adv_unit[handle].le_adv_irk,
               &hci_cmd_ptr->cmd_parameter[6], 16);
#else
        memcpy(&ll_rssi_manager.le_adv_irk[handle][0],
               &hci_cmd_ptr->cmd_parameter[6], 16);
#endif
        break;

    case LE_MSFT_MON_RSSI_FLAG_PATTERN_DATA:
        {
            UINT8 num_of_patterns;
            UINT8 length;
            UINT8 ad_type;
            UINT8 cmd_param_offset;
            UINT8 pattern_offset;
            UINT8 loops;
            UINT8 head = 0;

            num_of_patterns = hci_cmd_ptr->cmd_parameter[6];

            if (num_of_patterns == 0)
            {
                return INVALID_HCI_COMMAND_PARAMETERS_ERROR;
            }

            for (loops = 0; loops < 2; loops++)
            {
                cmd_param_offset = 7;

                for (i = 0; i < num_of_patterns; i++)
                {
                    length = hci_cmd_ptr->cmd_parameter[cmd_param_offset];

                    if (loops == 0)
                    {
                        if (length < 2)
                        {
                            /* invalid length field */
                            break;
                        }

                        cmd_param_offset += (length + 1);
                    }
                    else
                    {
                        cmd_param_offset++;
                        ad_type = hci_cmd_ptr->cmd_parameter[cmd_param_offset];
                        cmd_param_offset++;
                        pattern_offset = hci_cmd_ptr->cmd_parameter[cmd_param_offset];

#ifdef _SUPPORT_MSFT_BT_HCI_EXTENSION_FINAL_SPEC_
                        ll_rssi_manager.ms_adv_unit[handle].le_adv_pat.ad_type = ad_type;
                        ll_rssi_manager.ms_adv_unit[handle].le_adv_pat.offset = pattern_offset;
                        ll_rssi_manager.ms_adv_unit[handle].le_adv_pat.length = length - 2;
                        cmd_param_offset++;
                        memcpy(ll_rssi_manager.ms_adv_unit[handle].le_adv_pat.data,
                               &hci_cmd_ptr->cmd_parameter[cmd_param_offset],
                               (length - 2));

                        /* get the next node from linked list */
                        handle = ll_rssi_manager.ms_adv_unit[handle].next_id;
#else
                        ll_rssi_manager.le_adv_pat[handle].ad_type = ad_type;
                        ll_rssi_manager.le_adv_pat[handle].offset = pattern_offset;
                        ll_rssi_manager.le_adv_pat[handle].length = length - 2;
                        cmd_param_offset++;
                        memcpy(ll_rssi_manager.le_adv_pat[handle].data,
                               &hci_cmd_ptr->cmd_parameter[cmd_param_offset],
                               (length - 2));
#endif
                        cmd_param_offset += (length - 2);
                    }
                }

                if (loops == 0)
                {
                    if (i < num_of_patterns)
                    {
                        return INVALID_HCI_COMMAND_PARAMETERS_ERROR;
                    }
                    else
                    {
                        /* allocate a linked-list from the free list to fill
                           all patterns (note : one node of linked-list
                           only fill one pattern)  */
                        handle = rssi_app_msft_get_rssi_monitor_entry(flags,
                                                            num_of_patterns);

                        if (handle == 0xff)
                        {
                            return MEMORY_FULL_ERROR;
                        }

                        /* store the head of handle id */
                        head = handle;
                    }
                }
            }

            /* restore the head of handle id */
            handle = head;
        }
        break;

    case LE_MSFT_MON_RSSI_FLAG_BDADDR:
        {
            if ((hci_cmd_ptr->param_total_length != 13) ||
                 (hci_cmd_ptr->cmd_parameter[6] > 1))

            {
                return INVALID_HCI_COMMAND_PARAMETERS_ERROR;
            }

            UINT8 addr_type = hci_cmd_ptr->cmd_parameter[6];
            handle = rssi_app_msft_get_rssi_monitor_entry(flags, 1);
            if (handle == 0xff)
            {
                return MEMORY_FULL_ERROR;
            }

#ifdef _SUPPORT_MSFT_BT_HCI_EXTENSION_FINAL_SPEC_
            ll_rssi_manager.ms_adv_unit[handle].le_adv_bd_addr.type = addr_type;
            memcpy(ll_rssi_manager.ms_adv_unit[handle].le_adv_bd_addr.addr,
                    &hci_cmd_ptr->cmd_parameter[7], 6);
#else
            ll_rssi_manager.le_adv_bd_addr[handle].type = addr_type;
            memcpy(ll_rssi_manager.le_adv_bd_addr[handle].addr,
                    &hci_cmd_ptr->cmd_parameter[7], 6);
#endif
        }
        break;

    default:
        return INVALID_HCI_COMMAND_PARAMETERS_ERROR;
    }

    if (status == HCI_COMMAND_SUCCEEDED)
    {
        ll_rssi_manager.adv_entry[handle].max_rssi_thres = rssi_threshold_high;
        ll_rssi_manager.adv_entry[handle].min_rssi_thres = rssi_threshold_low;
        ll_rssi_manager.adv_entry[handle].min_rssi_thres_intv = rssi_threshold_low_intv * 10;
        ll_rssi_manager.adv_entry[handle].rssi_thres_counter = 0;

        ll_rssi_manager.adv_entry[handle].notified = 0;
        ll_rssi_manager.adv_entry[handle].start_periodic_timer = FALSE;
        ll_rssi_manager.adv_entry[handle].sample_period = rssi_sample_period;
        ll_rssi_manager.adv_entry[handle].sample_counter_of_tick = 0;
        ll_rssi_manager.adv_entry[handle].pkt_cnts = 0;
        ll_rssi_manager.adv_entry[handle].pkt_rssi_sum = 0;
        ll_rssi_manager.adv_entry[handle].mon_state = LE_MSFT_MONITOR_RSSI_STATE_OUT_RANGE;

#ifdef _SUPPORT_MSFT_BT_HCI_EXTENSION_FINAL_SPEC_
        ll_rssi_manager.adv_entry[handle].condition_type = flags;

        /* check and disable scanner hw filter policy, then
           enable sw filter policy */
        if (ll_manager.scan_unit.filter_policy &&
              (ll_manager.scan_unit.sw_filter_policy == 0))
        {
            ll_manager.scan_unit.sw_filter_policy = 1;

            ll_driver_disable_scanning();

            ll_manager.scan_unit.filter_policy = 0;
#ifdef _CCH_RETENTION_FLOW_FOR_DLPS_LE_
            ll_driver_enable_scanning(FALSE);
#else
            ll_driver_enable_scanning();
#endif
            ll_manager.scan_unit.filter_policy = 1;
        }
#endif
    }

    g_hci_vs_msft_le_monitor_handle = handle;

    return status;
}

/**************************************************************************
 * Function     : hci_handle_vs_msft_le_cancel_monitor_adv_cmd
 *
 * Description  : This function is used to handle
 *                HCI_VS_MSFT_Cancel_Monitor_Advertisement.
 *                Please refer chap 3.5 of MS final BT HCI extensions
 *                spec to know the detailed definition.
 *
 * Parameters   : hci_cmd_ptr Pointer to the command packet.
 *
 * Returns      : HCI_COMMAND_SUCCEEDED or HCI error code
 *
 *************************************************************************/
UCHAR hci_handle_vs_msft_le_cancel_monitor_adv_cmd(HCI_CMD_PKT *hci_cmd_ptr)
{
    UINT8 handle;
#ifndef _SUPPORT_MSFT_BT_HCI_EXTENSION_FINAL_SPEC_
    UINT8 status = HCI_COMMAND_SUCCEEDED;
    UINT8 type;
#endif

    if (hci_cmd_ptr->param_total_length != 2)
    {
        return INVALID_HCI_COMMAND_PARAMETERS_ERROR;
    }

    handle = hci_cmd_ptr->cmd_parameter[1];

#ifndef _SUPPORT_MSFT_BT_HCI_EXTENSION_FINAL_SPEC_
    type = ll_rssi_manager.le_adv_monitor_type;

#ifdef _SUPPORT_MSFT_BT_HCI_EXTENSION_FIX_
	ll_manager.scan_unit.filter_policy = filter_backup;
	LE_REG_S_SET reg;
	reg.value = RD_LE_REG(LE_REG_SCAN_CONTROL);
	reg.scan_ctrl.scan_filter = ll_manager.scan_unit.filter_policy;
	WR_LE_REG(LE_REG_SCAN_CONTROL, reg.value);
#endif

    switch (type)
    {
    case LE_MSFT_MON_RSSI_FLAG_IRK:
    case LE_MSFT_MON_RSSI_FLAG_PATTERN_DATA:
    case LE_MSFT_MON_RSSI_FLAG_BDADDR:
    case LE_MSFT_MON_RSSI_FLAG_UUID:
        if ((ll_rssi_manager.le_adv_monitor_entry_count == 0) ||
            !(ll_rssi_manager.bm_le_adv_monitor_entry & (1 << handle)))
        {
            /* invalid handle filed */
            return INVALID_HCI_COMMAND_PARAMETERS_ERROR;
        }
        rssi_app_msft_free_rssi_monitor_entry(type, handle);
        break;

    default:
        return INVALID_HCI_COMMAND_PARAMETERS_ERROR;
    }

    return status;

#else /* else of #ifndef _SUPPORT_MSFT_BT_HCI_EXTENSION_FINAL_SPEC_	 */

    if ((ll_rssi_manager.le_adv_monitor_entry_count == 0) ||
        !(ll_rssi_manager.bm_le_adv_monitor_entry[0] & (1 << handle)))
    {
        /* invalid handle filed */
        return INVALID_HCI_COMMAND_PARAMETERS_ERROR;
    }

    rssi_app_msft_free_rssi_monitor_entry(
        ll_rssi_manager.adv_entry[handle].condition_type,
        handle);

    /* check and disable scanner hw filter policy, then
       enable sw filter policy */
    if ((ll_rssi_manager.le_adv_monitor_entry_count == 0) &&
                        ll_manager.scan_unit.sw_filter_policy)
    {
        ll_manager.scan_unit.sw_filter_policy = 0;

        if (ll_manager.scan_unit.filter_policy)
        {
            ll_driver_disable_scanning();
#ifdef _CCH_RETENTION_FLOW_FOR_DLPS_LE_
            ll_driver_enable_scanning(FALSE);
#else
            ll_driver_enable_scanning();
#endif
        }
    }

    return HCI_COMMAND_SUCCEEDED;
#endif /* end of #ifndef _SUPPORT_MSFT_BT_HCI_EXTENSION_FINAL_SPEC_	 */
}

/**************************************************************************
 * Function     : hci_handle_vs_msft_le_set_adv_filter_enable_cmd
 *
 * Description  : This function is used to handle
 *                HCI_VS_MSFT_LE_Set_Advertisement_Filter_Enable.
 *                Please refer chap 3.6 of MS final BT HCI extensions
 *                spec to know the detailed definition.
 *
 * Parameters   : hci_cmd_ptr Pointer to the command packet.
 *
 * Returns      : HCI_COMMAND_SUCCEEDED or HCI error code
 *
 *************************************************************************/
UCHAR hci_handle_vs_msft_le_set_adv_filter_enable_cmd(HCI_CMD_PKT *hci_cmd_ptr)
{
    UINT8 status = HCI_COMMAND_SUCCEEDED;
    UINT8 enable;

    if ((hci_cmd_ptr->param_total_length != 2) ||
        (hci_cmd_ptr->cmd_parameter[1] > 1))
    {
        return INVALID_HCI_COMMAND_PARAMETERS_ERROR;
    }

    enable = hci_cmd_ptr->cmd_parameter[1];

    /* The controller shall reject an
        HCI_VS_MSFT_LE_Set_Advertisement_Filter_Enable command if it does
        not toggle the filter state. */

    if (enable == ll_rssi_manager.le_adv_monttor_filter_enable)
    {
        return COMMAND_DISALLOWED_ERROR;
    }

    ll_rssi_manager.le_adv_monttor_filter_enable = enable;

    return status;
}


UCHAR hci_handle_vs_msft_read_absolute_rssi_cmd(HCI_CMD_PKT *hci_cmd_ptr)
{
    UINT8 status = HCI_COMMAND_SUCCEEDED;
    UINT16 ce_index;

    if (hci_cmd_ptr->param_total_length != 3)
    {
        return INVALID_HCI_COMMAND_PARAMETERS_ERROR;
    }

    g_hci_vs_msft_le_monitor_handle = hci_cmd_ptr->cmd_parameter[1] |
                                    (hci_cmd_ptr->cmd_parameter[2] << 8);

    if (LMP_GET_CE_INDEX_FROM_CONN_HANDLE(g_hci_vs_msft_le_monitor_handle, &ce_index)
            == API_FAILURE)
    {
        status = NO_CONNECTION_ERROR ;
    }

    return status;
}

/*************************************************************************
* The function table of HCI VS MSFT Command Handling
*************************************************************************/
UINT8 (*(hci_handle_vs_msft_commands[])) (HCI_CMD_PKT *hci_cmd_ptr) =
{
    hci_handle_vs_msft_read_supported_features_cmd,
    hci_handle_vs_msft_monitor_rssi_cmd,
    hci_handle_vs_msft_cancel_monitor_rssi_cmd,
    hci_handle_vs_msft_le_monitor_adv_cmd,
    hci_handle_vs_msft_le_cancel_monitor_adv_cmd,
    hci_handle_vs_msft_le_set_adv_filter_enable_cmd,
    hci_handle_vs_msft_read_absolute_rssi_cmd
};

/**************************************************************************
 * Function     : hci_handle_vs_msft_cmd_set
 *
 * Description  : This function is used to handle all microsoft-defined
 *                hci commands (sub-commands).
 *                Please refer chap 3 of MS final BT HCI extensions
 *                spec to know the detailed definition.
 *
 * Parameters   : hci_cmd_ptr Pointer to the command packet.
 *
 * Returns      : HCI_COMMAND_SUCCEEDED or HCI error code
 *
 *************************************************************************/
UCHAR hci_handle_vs_msft_cmd_set(HCI_CMD_PKT *hci_cmd_ptr,
                              UINT8 *send_cmd_comp_event,
                              UINT8 *send_cmd_status_event)
{
    UINT8 status = HCI_COMMAND_SUCCEEDED;
    UINT8 opcode = hci_cmd_ptr->cmd_parameter[0];

    *send_cmd_comp_event = 1;
    *send_cmd_status_event = 0;

    if (hci_cmd_ptr->param_total_length == 0)
    {
        g_hci_vs_msft_sub_opcode = 0xffff;
        return INVALID_HCI_COMMAND_PARAMETERS_ERROR;
    }

    g_hci_vs_msft_sub_opcode = opcode;

    if (opcode > HCI_SUB_VENDOR_MSFT_MAX_OPCODE)
    {
        g_hci_vs_msft_sub_opcode = 0xffff;
        return INVALID_HCI_COMMAND_PARAMETERS_ERROR;
    }
    else
    {
        if (ll_rssi_manager.msft_monitor_rssi_mode !=
                                LE_MSFT_MONITOR_RSSI_VIA_CONN_HANDLE_MODE_MSFT)
        {
            ll_rssi_manager.max_indivadual_cnt = 0;
            ll_rssi_manager.max_bredr_cnt = 0;
            ll_rssi_manager.bm_used_handle = 0;
            ll_rssi_manager.bm_used_legacy_handle = 0;
            ll_rssi_manager.msft_monitor_rssi_mode =
                            LE_MSFT_MONITOR_RSSI_VIA_CONN_HANDLE_MODE_MSFT;
        }

        status = hci_handle_vs_msft_commands[opcode](hci_cmd_ptr);
    }

    return status;
}

#endif /* end of #ifdef _SUPPORT_MSFT_BT_HCI_EXTENSION_ */

#if defined(_LE_AUTO_REPORT_RSSI_AND_LOGIN_INOUT_FUNC_) && defined(LE_MODE_EN)
UCHAR hci_handle_set_le_rssi_threshold_cmd(HCI_CMD_PKT *hci_cmd_ptr)
{
    memset(&ll_rssi_manager, 0, sizeof(LL_CONN_HANDLE_RSSI_MANAGER));

    if (hci_cmd_ptr->cmd_parameter[0] != 0)
    {
        /* enable le rssi threshold */

        UINT8 num_of_links;
        UINT8 addr_type;
        UINT8 *addr;
        UINT8 offset;
        UINT8 i;
        INT8 rssi_min;
        INT8 rssi_max;

        num_of_links = hci_cmd_ptr->cmd_parameter[1];

        if (hci_cmd_ptr->param_total_length != ((num_of_links * 9) + 2))
        {
            return INVALID_HCI_COMMAND_PARAMETERS_ERROR;
        }

        offset = 2;

#ifdef _SUPPORT_MSFT_BT_HCI_EXTENSION_
        if (num_of_links > 0)
        {
            ll_rssi_manager.msft_monitor_rssi_mode =
                                LE_MSFT_MONITOR_RSSI_VIA_CONN_HANDLE_MODE_RTK;
        }
        else
        {
            ll_rssi_manager.msft_monitor_rssi_mode =
                                LE_MSFT_MONITOR_RSSI_VIA_CONN_HANDLE_MODE_NONE;
        }
#endif

        while (ll_rssi_manager.max_indivadual_cnt != num_of_links)
        {
            /* record every rssi info to the entry of catch */

            i = ll_rssi_manager.max_indivadual_cnt;

            addr_type = hci_cmd_ptr->cmd_parameter[offset];
            addr = &hci_cmd_ptr->cmd_parameter[offset + 1];
            rssi_min = (INT8)hci_cmd_ptr->cmd_parameter[offset + 7];
            rssi_max = (INT8)hci_cmd_ptr->cmd_parameter[offset + 8];

            ll_rssi_manager.entry[i].addr_type = addr_type;
            memcpy(ll_rssi_manager.entry[i].addr, addr, 6);
            ll_rssi_manager.entry[i].min_rssi_thres = rssi_min;
            ll_rssi_manager.entry[i].max_rssi_thres = rssi_max;
            ll_rssi_manager.max_indivadual_cnt++;

            if (addr_type == 2)
            {
                ll_rssi_manager.max_bredr_cnt++;
            }

            offset += 9;
#ifndef _ROM_CODE_PATCHED_REDUCE_
            RT_BT_LOG(YELLOW, MSG_LE_RSSI_RPT_LOG, 10,
                            num_of_links, addr_type, addr[0], addr[1],
                            addr[2], addr[3], addr[4], addr[5],
                            rssi_min, rssi_max);
#endif
        }

        rssi_app_le_bind_rssi_info();
    }

    return HCI_COMMAND_SUCCEEDED;
}
#endif

#ifndef IS_BTSOC
#ifdef _ROM_CODE_PATCHED_
UCHAR hci_handle_vendor_download_cmd(HCI_CMD_PKT *hci_cmd_ptr)
{
    if (hci_cmd_ptr->param_total_length > 0)
    {
        UINT8 index;
        index = hci_cmd_ptr->cmd_parameter[0];
        rom_code_patch_block_index = index;

#ifndef SPI_FLASH_BOOT
        if (!IS_ROM_CODE_PATCHED) /* do not accpet patch again after POR */
        {
            UINT8 len;
            UINT8 last;

            last = index >> 7;
            len = hci_cmd_ptr->param_total_length - 1;

            if ((index & 0x7F) == 0)
            {
                rom_code_patch_start_address = ROM_CODE_PAGE_START_ADDRESS;
            }

            if (len > 0)
            {
                memcpy((UINT8*)rom_code_patch_start_address,
                       &hci_cmd_ptr->cmd_parameter[1], len);
                rom_code_patch_start_address += len;
            }

            if (last)
            {
                SIGN_ROM_CODE_PATCH_SIGNATURE;
                SIGN_PATCH_END_EVENT_SIGNATURE(index);
                SIGN_FW_TRIG_WDG_TO_SIGNATURE;
#ifdef _UART_H5
#ifdef _YL_RTL8723A_B_CUT
                if (g_fun_interface_info.b.bt_interface == UART_INTERFACE)
                {
                    UINT32 uart_status_d32 = (UINT32)UART_DWORD_READ(DMA_UART_H5_INTSTS);
                    SIGN_PATCH_UART_STATUS(uart_status_d32);
                }
#endif
#endif

                WDG_TIMER_TIMEOUT_SOON;
                while (1); /* infinite loop here to wait watch dog timeout */
            }
        }
        else
        {
            rom_code_patch_block_index |= BIT7;
        }
#endif
        return HCI_COMMAND_SUCCEEDED;
    }

    return COMMAND_DISALLOWED_ERROR;
}
#endif
#endif

#ifdef _SPIC_FUNC_ENABLE_
UCHAR hci_handle_spi_erase_cmd(HCI_CMD_PKT *hci_cmd_ptr)
{
    UINT8 status;

    do
    {
        status = COMMAND_DISALLOWED_ERROR;
        if ((hci_cmd_ptr->param_total_length == 7) && is_spi_init)
        {
            UINT8 type;
            UINT32 start_addr;
            UINT32 end_addr;

            type = hci_cmd_ptr->cmd_parameter[0];
            start_addr = hci_cmd_ptr->cmd_parameter[1] |
                        (hci_cmd_ptr->cmd_parameter[2] << 8) |
                        (hci_cmd_ptr->cmd_parameter[3] << 16);
            end_addr =   hci_cmd_ptr->cmd_parameter[4] |
                        (hci_cmd_ptr->cmd_parameter[5] << 8) |
                        (hci_cmd_ptr->cmd_parameter[6] << 16);

            if ((type > SPI_FLASH_ERASE_TYPE_SECTOR) ||
                (start_addr > end_addr))
            {
                break;
            }

            WDG_TIMER_DISABLE; /* avoid watch dog timer reset */

            BTON_WDG_TIMER_DISABLE; /* avoid bt on watch dog timer reset */

            spi_erase(type, start_addr, end_addr);
            status = HCI_COMMAND_SUCCEEDED;
        }
    }
    while (0);

    return status;
}

UCHAR hci_handle_spi_write_cmd(HCI_CMD_PKT *hci_cmd_ptr)
{
    UINT8 status;

    do
    {
        status = COMMAND_DISALLOWED_ERROR;
        if ((hci_cmd_ptr->param_total_length > 5) && is_spi_init)
        {
            UINT32 start_addr;
            UINT16 immediate;
            UINT16 data_len;
            UINT16 offset;

            start_addr = hci_cmd_ptr->cmd_parameter[0] |
                        (hci_cmd_ptr->cmd_parameter[1] << 8) |
                        (hci_cmd_ptr->cmd_parameter[2] << 16);
            immediate =  hci_cmd_ptr->cmd_parameter[3] |
                        (hci_cmd_ptr->cmd_parameter[4] << 8);
            data_len = hci_cmd_ptr->param_total_length - 5;

            if ((immediate > 1) || (data_len & 0x03))
            {
                break;
            }

            WDG_TIMER_DISABLE; /* avoid watch dog timer reset */

            BTON_WDG_TIMER_DISABLE; /* avoid bt on watch dog timer reset */

            offset = start_addr & 0xFF;

            if ((offset + data_len) > 0x100)
            {
                /* cross the page boundary */
                UINT16 cur_page_len = 0x100 - offset;
                UINT16 next_page_len = data_len - cur_page_len;
                offset = 5;
                while (cur_page_len >= 12)
                {
                    spi_write_data(start_addr, 12,
                                   &hci_cmd_ptr->cmd_parameter[offset]);
                    cur_page_len -= 12;
                    start_addr += 12;
                    offset += 12;
                }

                if (cur_page_len > 0)
                {
                    spi_write_data(start_addr, cur_page_len,
                                   &hci_cmd_ptr->cmd_parameter[offset]);
                    start_addr += cur_page_len;
                    offset += cur_page_len;
                }

                while (next_page_len >= 12)
                {
                    spi_write_data(start_addr, 12,
                                   &hci_cmd_ptr->cmd_parameter[offset]);
                    next_page_len -= 12;
                    start_addr += 12;
                    offset += 12;
                }

                if (next_page_len > 0)
                {
                    spi_write_data(start_addr, next_page_len,
                                   &hci_cmd_ptr->cmd_parameter[offset]);
                }
            }
            else
            {
                /* no cross the page boundary */
                offset = 5;
                while (data_len >= 12)
                {
                    spi_write_data(start_addr, 12,
                                   &hci_cmd_ptr->cmd_parameter[offset]);
                    data_len -= 12;
                    start_addr += 12;
                    offset += 12;
                }

                if (data_len > 0)
                {
                    spi_write_data(start_addr, data_len,
                                   &hci_cmd_ptr->cmd_parameter[offset]);
                }
            }
            status = HCI_COMMAND_SUCCEEDED;
        }
    }
    while (0);

    return status;
}

#endif

#ifdef BT_DRIVER_PROVIDE_PROFILE_INFO
/*===NEW PACKET FORMAT===(Baron , 2014.02.19)
?Parameter Total Length: 1 + (3*n) + 1 Octets

Number_Of_Handles (1 octet) : the number of connection handle
Connection_Hanlde[n] (2 octet): mapped connection handle value
Profile [n] (1 octet) : the bit map of Profile Information(Connection/Disconnection)
                    bit[0]: SCO
                    bit[1]: HID
                    bit[2]: A2DP
                    bit[3]: FTP/PAN/OPP
                    bit[7:4]: reserved, TBD
State (1 octet) : the bit map of Status Information(busy/idle)
                    bit[0]: SCO
                    bit[1]: HID
                    bit[2]: A2DP
                    bit[3]: FTP/PAN/OPP
                    bit[7:4]: reserved, TBD
*/

UCHAR hci_handle_set_profile_info_cmd(HCI_CMD_PKT *hci_cmd_ptr)
{
  //(2014.02.19 BaRon mask because NEW FUNCTION of this cmd had be implemented)
    /*UINT8 num_of_links;
    UINT8 len;
    UINT16 conn_handle;
    UINT16 bit_map;
    UINT8 offset;

    num_of_links = hci_cmd_ptr->cmd_parameter[0];

    if (hci_cmd_ptr->param_total_length != ((num_of_links * 3) + 2))
    {
        return INVALID_HCI_COMMAND_PARAMETERS_ERROR;
    }

    len  = hci_cmd_ptr->param_total_length - 1;
    offset = 1;

    g_host_set_profile = TRUE;
    g_driver_report_hid_state = 0;
    g_driver_report_ftp_state = 0;
    g_driver_report_a2dp_state = 0;

    while (len)
    {
        conn_handle = hci_cmd_ptr->cmd_parameter[offset] |
                      (hci_cmd_ptr->cmd_parameter[offset + 1] << 8);
        bit_map = hci_cmd_ptr->cmd_parameter[offset + 2] |
                      (hci_cmd_ptr->cmd_parameter[offset + 3] << 8);

        if (bit_map & BIT0)
        {
            g_driver_report_hid_state = 1;
        }
        if (bit_map & BIT1)
        {
            g_driver_report_a2dp_state = 1;
        }
        if (bit_map & BIT2)
        {
            g_driver_report_ftp_state = 1;
        }
        offset += 4;
        len -= 4;
    }
    */

    //(BaRoN) NEW STYLE 2014.02.19
    UCHAR param_length;
    UCHAR profile_status;
    UINT8 i;
    UINT16 conn_handle;
    UINT8 bit_map;
    param_length = hci_cmd_ptr->param_total_length;
    profile_status = hci_cmd_ptr->cmd_parameter[ (param_length-1) ] ;
    //num_links = hci_cmd_ptr->cmd_parameter[0];
    for( i = 0 ; i < (param_length-2) ; i += 3 )
    {
        conn_handle = hci_cmd_ptr->cmd_parameter[ i + 1 ] | (hci_cmd_ptr->cmd_parameter[ i + 2] << 8);
        bit_map = hci_cmd_ptr->cmd_parameter[ i + 3 ];
		g_driver_sco_connection_handle =  (  bit_map        & BIT0) ? conn_handle : 0xffff ;
		g_driver_hid_connection_handle =  (( bit_map >> 1 ) & BIT0) ? conn_handle : 0xffff ;
		g_driver_a2dp_connection_handle = (( bit_map >> 2 ) & BIT0) ? conn_handle : 0xffff ;
		g_driver_ftp_connection_handle =  (( bit_map >> 3 ) & BIT0) ? conn_handle : 0xffff ;
    }

    g_driver_report_sco_state = 	(  profile_status       & BIT0 );
    g_driver_report_hid_state = 	( (profile_status >> 1) & BIT0 );
    g_driver_report_a2dp_state = 	( (profile_status >> 2) & BIT0 );
    g_driver_report_ftp_state = 	( (profile_status >> 3) & BIT0 );

    return HCI_COMMAND_SUCCEEDED;
}

#endif

#ifdef _DAPE_TEST_AUTO_CONN
UCHAR hci_handle_add_dev_to_white_list_cmd(HCI_CMD_PKT *hci_cmd_ptr)
{
    UINT8 index;
    UINT8 find_result = 1;
	    LEGACY_WHITE_LIST *white_list = NULL;
    UINT8 state = 0;

    if (num_of_white_list_device)
    {
        for (index = 0; index < LEGACY_MAX_WHITE_LIST_SIZE; index++ )
        {
            white_list = &conn_white_list[index];
            find_result = memcmp(white_list->bd_addr, &hci_cmd_ptr->cmd_parameter[0], 6);
            if ((find_result == 0) && (white_list->enabled))
            {
                break;
            }
       	}
    }
    if ((num_of_white_list_device)
		 && (find_result == 0) && (white_list->enabled))
    {
/*                RT_BT_LOG(GREEN, DAPE_TEST_LOG533, 7,
                white_list->bd_addr[5], white_list->bd_addr[4],
                white_list->bd_addr[3], white_list->bd_addr[2],
                white_list->bd_addr[1], white_list->bd_addr[0],
                num_of_white_list_device);*/
        state |= BIT0;
    }
    else
    {
        if (num_of_white_list_device < LEGACY_MAX_WHITE_LIST_SIZE)
        {
            LEGACY_WHITE_LIST *white_list;
            for (index = 0; index < LEGACY_MAX_WHITE_LIST_SIZE; index++ )
            {
                white_list = &conn_white_list[index];
                if (!white_list->enabled)
                {
                    memcpy(white_list->bd_addr, &hci_cmd_ptr->cmd_parameter[0], 6);
                    white_list->enabled = 1;
                    num_of_white_list_device ++;
                    break;
                }
            }
            /*RT_BT_LOG(GREEN, DAPE_TEST_LOG528, 7,
                white_list->bd_addr[5], white_list->bd_addr[4],
                white_list->bd_addr[3], white_list->bd_addr[2],
                white_list->bd_addr[1], white_list->bd_addr[0],
                num_of_white_list_device);*/
            state |= BIT1;
            //RT_BT_LOG(GREEN, DAPE_TEST_LOG528, 0,0);
        }
        else
        {
            /*RT_BT_LOG(RED, DAPE_TEST_LOG529, 7,
                num_of_white_list_device,
                hci_cmd_ptr->cmd_parameter[5], hci_cmd_ptr->cmd_parameter[4],
                hci_cmd_ptr->cmd_parameter[3], hci_cmd_ptr->cmd_parameter[2],
                hci_cmd_ptr->cmd_parameter[1], hci_cmd_ptr->cmd_parameter[0]);*/
            state |= BIT2;
            //RT_BT_LOG(RED, DAPE_TEST_LOG529, 0,0);
        }
    }

    RT_BT_LOG(GREEN, MSG_HCI_VENDOR_ADD_DEVICE_TO_WHITE_LIST, 1, state);

    return HCI_COMMAND_SUCCEEDED;
}


UCHAR hci_handle_rem_dev_from_white_list_cmd(HCI_CMD_PKT *hci_cmd_ptr)
{
    UINT8 state = 0;
    if (num_of_white_list_device)
    {
        UINT8 index;
        UINT8 find_result = 1;
       	LEGACY_WHITE_LIST *white_list;
        UINT8 state = 0;

       	for (index = 0; index < LEGACY_MAX_WHITE_LIST_SIZE; index++ )
        {
            white_list = &conn_white_list[index];
            find_result = memcmp(white_list->bd_addr, &hci_cmd_ptr->cmd_parameter[0], 6);
            if ((find_result == 0) && (white_list->enabled))
            {
                memset(&conn_white_list[index], 0, sizeof(conn_white_list[index]));
                num_of_white_list_device --;

                /*RT_BT_LOG(GREEN, DAPE_TEST_LOG530, 7,
                hci_cmd_ptr->cmd_parameter[5], hci_cmd_ptr->cmd_parameter[4],
                hci_cmd_ptr->cmd_parameter[3], hci_cmd_ptr->cmd_parameter[2],
                hci_cmd_ptr->cmd_parameter[1], hci_cmd_ptr->cmd_parameter[0],
			    num_of_white_list_device);*/
			    state |= BIT0;
                //RT_BT_LOG(GREEN, DAPE_TEST_LOG530, 0,0);
                break;
           }
       	}
        if(find_result != 0)
        {
            /*RT_BT_LOG(RED, DAPE_TEST_LOG531, 7,
                      hci_cmd_ptr->cmd_parameter[5], hci_cmd_ptr->cmd_parameter[4],
                      hci_cmd_ptr->cmd_parameter[3], hci_cmd_ptr->cmd_parameter[2],
                      hci_cmd_ptr->cmd_parameter[1], hci_cmd_ptr->cmd_parameter[0],
                      num_of_white_list_device);*/
            state |= BIT1;
            //RT_BT_LOG(RED, DAPE_TEST_LOG531, 0,0);
        }
    }
    else
    {
        state |= BIT2;
        //RT_BT_LOG(RED, DAPE_TEST_LOG532, 0, 0);
    }
    RT_BT_LOG(RED, MSG_HCI_VENDOR_REMOVE_DEVICE_FROM_WHITE_LIST, 1, state);

    return HCI_COMMAND_SUCCEEDED;
}



#endif

#ifdef _DAPE_TEST_GEN_FAKE_ESCO_DATA
void gen_esco_data_packet(UINT8 change)
{
    HCI_SYNC_DATA_PKT *synchronous_pkt;

    if (pgen_synchronous_pkt == NULL)
    {
        OS_RESET_POOL(&synchronous_data_pool_id);
        OS_ALLOC_BUFFER(synchronous_data_pool_id, (OS_ADDRESS *)&pgen_synchronous_pkt);
    }

    synchronous_pkt = pgen_synchronous_pkt;

    synchronous_pkt->connection_handle = g_handle;
    if (change)
    {
        g_change++;
        RT_BT_LOG(GREEN, DAPE_TEST_LOG522, 2,g_change, (g_change<<4));
        synchronous_pkt->packet_length = 60;
        UINT8 i = 0;
        for (i = 0; i <synchronous_pkt->packet_length; i++)
        {
            synchronous_pkt->hci_sync_data_packet[i] = (g_change<<4)+(i%10);
        }
        g_change_flag = 1;
    }
    lc_handle_synchronous_data_pkt(synchronous_pkt);
}
#endif

#ifdef BT2FM_INDIRECT_ACCESS
UCHAR hci_vendor_fm_switch_hdlr(HCI_CMD_PKT *hci_cmd_ptr)
{
    UINT8 state = hci_cmd_ptr->cmd_parameter[0];
    RT_BT_LOG(GREEN, LOG_FM_SWITCH, 1, state);
    fm_switch(state & BIT0);
    fm_i2s_switch(state & BIT1);
    return HCI_COMMAND_SUCCEEDED;
}

UINT16 g_fm_rd_addr;
UINT8 g_fm_rd_len;
UINT8 hci_vendor_fm_read_hdlr(HCI_CMD_PKT *hci_cmd_ptr)
{
    UINT16 addr = (hci_cmd_ptr->cmd_parameter[0] |
            (hci_cmd_ptr->cmd_parameter[1] << 8));
    UINT8 len = hci_cmd_ptr->cmd_parameter[2];

    if (len > FM_READ_MAX_LEN)
    {
        return INVALID_HCI_COMMAND_PARAMETERS_ERROR;
    }
    RT_BT_LOG(GREEN, LOG_FM_READ, 2, addr, len);
    g_fm_rd_addr = addr;
    g_fm_rd_len = len;
    /* Defer real read operation until generating command complete event to
     * save temporary buffer size.
     */
    return HCI_COMMAND_SUCCEEDED;
}

UCHAR hci_vendor_fm_write_hdlr(HCI_CMD_PKT *hci_cmd_ptr)
{
    UINT16 addr = (hci_cmd_ptr->cmd_parameter[0] |
            (hci_cmd_ptr->cmd_parameter[1] << 8));
    UINT8 len = hci_cmd_ptr->param_total_length - 2;
    UINT8 i;
    for (i = 0; i < len; ++i)
    {
        if (!fm_write_register(addr, hci_cmd_ptr->cmd_parameter[2 + i]))
        {
            return HARDWARE_FAILURE_ERROR;
        }
        RT_BT_LOG(GREEN, LOG_FM_WRITE, 2, addr,
                hci_cmd_ptr->cmd_parameter[2 + i]);
        ++addr;
    }
    return HCI_COMMAND_SUCCEEDED;
}

UCHAR hci_vendor_fm_set_hdlr(HCI_CMD_PKT *cmd_pkt)
{
    UINT16 addr = (cmd_pkt->cmd_parameter[0] |
            (cmd_pkt->cmd_parameter[1] << 8));
    UINT8 i, len;
    switch (cmd_pkt->param_total_length)
    {
    case 4:
    case 6:
    case 10:
        len = (cmd_pkt->param_total_length - 2) / 2;
        break;
    default:
        return INVALID_HCI_COMMAND_PARAMETERS_ERROR;
    }

    UINT32 mask = 0;
    UINT32 val = 0;
    UINT32 data = 0;
    for (i = 0; i < len; ++i)
    {
        UINT8 t;
        mask |= cmd_pkt->cmd_parameter[2 + i];
        val |= cmd_pkt->cmd_parameter[2 + len + i];
        if (!fm_read_register(addr + i, &t))
        {
            return HARDWARE_FAILURE_ERROR;
        }
        data |= t;
        if (i < len - 1)
        {
            mask <<= 8;
            val <<= 8;
            data <<= 8;
        }
    }
    UINT32 old_data = data;
    data ^= ((data & mask) ^ (val & mask));
    RT_BT_LOG(GREEN, LOG_FM_SET, 5, addr, mask, val, old_data, data);

    for (i = 0; i < len; ++i)
    {
        UINT8 octet = (data >> ((len - 1 - i) * 8));
        if (!fm_write_register(addr + i, octet))
        {
            return HARDWARE_FAILURE_ERROR;
        }
        RT_BT_LOG(GREEN, LOG_FM_WRITE, 2, addr + i, octet);
    }
    return HCI_COMMAND_SUCCEEDED;
}

#ifdef FM_DEBUG
UCHAR hci_vendor_fm_test_status_hdlr(HCI_CMD_PKT *cmd_pkt)
{
    UINT8 ready = fm_get_ready();
    RT_BT_LOG(GREEN, LOG_FM_GET_READY, 1, ready);

    UINT32 reg = fm_get_access_reg_raw();
    RT_BT_LOG(GREEN, LOG_FM_GET_ACCESS_REG, 1, reg);
    return HCI_COMMAND_SUCCEEDED;
}
#endif
#endif /* BT2FM_INDIRECT_ACCESS */

/************************************************************************
 * Function : hci_handle_vendor_cmds
 *
 * Description:
 *          This function verifies the vendor group of
 *          commands received from the host. It extracts the command opcode
 *          from the command packet and analyzes the  command packet depending
 *          on the group of command it calls the corresponding routines.
 *
 *
 * Parameters :
 *      HCI_CMD_PKT *hci_cmd_ptr : Pointer to the command buffer.
 *
 * Returns    : SUCCESS or FAILURE
 *
 *************************************************************************/
UCHAR hci_handle_vendor_cmds(HCI_CMD_PKT *hci_cmd_ptr)
{
    UCHAR           status = HCI_COMMAND_SUCCEEDED;
    UCHAR           send_cmd_comp_event_flag = 1;
    UCHAR           send_cmd_status_event_flag = 0;
    void *arg = NULL;

    DEF_CRITICAL_SECTION_STORAGE;


#ifdef _ROM_CODE_PATCHED_
    /* TODO: ROM CODE PATCH HERE if required */
    if (rcp_hci_vendor_cmd_func != NULL)
    {
        if (rcp_hci_vendor_cmd_func((void*)hci_cmd_ptr,
                                &send_cmd_comp_event_flag,
                                &status))
        {
            return status;
        }
    }
#endif

    switch (hci_cmd_ptr->cmd_opcode)
    {
#ifdef _REPORT_CRC_ERR_PKT_IN_LE_RX_TEST_MODE
        case HCI_VENDOR_READ_LE_RX_TEST_PKT:
            send_cmd_comp_event_flag = 1;
            status = HCI_COMMAND_SUCCEEDED;

            break;
#endif

#if defined (_TEST_PI_WRITE_PAGE_3_BY_VENDOR_CMD) || defined (_TEST_ADAPTIVITY_FUNC_2)
        case HCI_VENDOR_ENABLE_WRITE_PI_PAGE_3:
            g_enable_pi_write_page_3 = hci_cmd_ptr->cmd_parameter[0];
            g_enable_pi_read_page_3 = hci_cmd_ptr->cmd_parameter[1];
            g_enable_pi_read_after_tx = hci_cmd_ptr->cmd_parameter[2];
            g_enable_change_tx_gain = hci_cmd_ptr->cmd_parameter[3];
            g_set_no_tx = hci_cmd_ptr->cmd_parameter[4];

            RT_BT_LOG(WHITE, YL_DBG_DEC_4, 4,
                g_enable_pi_write_page_3, g_enable_pi_read_page_3, g_enable_pi_read_after_tx,
                g_enable_change_tx_gain);

            UINT32 reg5C = RTK_READ_MODEM_REG_PI(MODEM_PI_PAGE_3, TRANS_MODEM_REG(0x5C));
            UINT8 reg_tx_gain_set_manu_en = (reg5C >> 6) & BIT0;
            UINT8 reg_tx_gain_set_manu_val_7_0 = (reg5C >> 7) & 0xFF;

            if (g_enable_change_tx_gain)
            {
                /* 0x5C[14:6] all 1*/
                reg5C |= 0x7FC0;
                RTK_WRITE_MODEM_REG_PI(MODEM_PI_PAGE_3, TRANS_MODEM_REG(0x5C), reg5C);

            }
            else
            {
                /* 0x5C[6] = 0 (manual_en = 0)*/
                reg5C &= (~BIT6);
                RTK_WRITE_MODEM_REG_PI(MODEM_PI_PAGE_3, TRANS_MODEM_REG(0x5C), reg5C);
            }

            UINT16 reg14 = RTK_READ_MODEM_REG_PI(MODEM_PI_PAGE_6, TRANS_MODEM_REG(0x14));
            if (g_set_no_tx)
            {
                RTK_WRITE_MODEM_REG_PI(MODEM_PI_PAGE_6, TRANS_MODEM_REG(0x14), reg14 & (0xFF0F));
            }
            else
            {
                UINT16 temp_reg14= reg14 & (0xFF0F);
                temp_reg14 |= 0x0020;
                RTK_WRITE_MODEM_REG_PI(MODEM_PI_PAGE_6, TRANS_MODEM_REG(0x14), temp_reg14);

            }
            RT_BT_LOG(BLUE, YL_DBG_HEX_5, 5,
                RTK_READ_MODEM_REG_PI(MODEM_PI_PAGE_6, TRANS_MODEM_REG(0x14)),
                RTK_READ_MODEM_REG_PI(MODEM_PI_PAGE_0, TRANS_MODEM_REG(0x7E)),
                RTK_READ_MODEM_REG_PI(MODEM_PI_PAGE_3, TRANS_MODEM_REG(0x5C)),
                reg_tx_gain_set_manu_en, reg_tx_gain_set_manu_val_7_0);

            break;
#endif

#ifdef _DAPE_SUPPORT_LE_TX_PKT_MODE
        case HCI_VENDOR_ENABLE_LE_TX_PKT_MODE:
            ll_manager.test_unit.tx_cnt_mode_en = hci_cmd_ptr->cmd_parameter[0];
            ll_manager.test_unit.tx_pkt_cnt = hci_cmd_ptr->cmd_parameter[1];
#ifdef _DAPE_SUPPORT_CHG_AA_IN_LE_TEST_MODE
            ll_manager.test_unit.chg_aa = hci_cmd_ptr->cmd_parameter[2];
            ll_manager.test_unit.access_addr_l = (hci_cmd_ptr->cmd_parameter[3] |
                                                   (hci_cmd_ptr->cmd_parameter[4] << 8));

            ll_manager.test_unit.access_addr_h = (hci_cmd_ptr->cmd_parameter[5] |
                                                   (hci_cmd_ptr->cmd_parameter[6] << 8));

#endif
#ifdef _DAPE_SUPPORT_LE_TEST_MODE_TX_RANDOM
            ll_manager.test_unit.prbs_fix = hci_cmd_ptr->cmd_parameter[7];
#endif

            break;
#endif

#ifdef _DAPE_TEST_FRAME_SYNC_UPDATE_CHOOSE_WAY
        case HCI_VENDOR_CHOOSE_MODIFY_WAY:
            g_modify_way = hci_cmd_ptr->cmd_parameter[0];
            break;
#endif

#ifdef _DAPE_SEND_PCA_ADJ_BY_VENDOR_CMD
        case HCI_VENDOR_SEND_CLK_ADJ_REQ:
        {
            UINT16 ce_index = (UINT16)hci_cmd_ptr->cmd_parameter[0];
            INT16 clk_adj_us;
            UINT8 clk_adj_slot;
            UINT8 clk_adj_period;

            clk_adj_us = hci_cmd_ptr->cmd_parameter[1]|
                               ((hci_cmd_ptr->cmd_parameter[2]& 0x7F)<< 8);
            if (hci_cmd_ptr->cmd_parameter[2]&BIT7)
            {
                clk_adj_us = 0 - clk_adj_us;
            }
            clk_adj_slot = hci_cmd_ptr->cmd_parameter[3];
            clk_adj_period = hci_cmd_ptr->cmd_parameter[4];

            lmp_generate_clk_adj_req_pdu(ce_index, clk_adj_us, clk_adj_slot, clk_adj_period);
        }
        break;
        case HCI_VENDOR_SEND_CLK_ADJ:
        {
            UINT32 clk = BB_read_native_clock();
            UINT32 instant = (clk>>1) + hci_cmd_ptr->cmd_parameter[0] +
            (hci_cmd_ptr->cmd_parameter[1]<<8) + (hci_cmd_ptr->cmd_parameter[2]<<16);
            instant &= ((UINT32)(0x3FFFFF)); // 22 bits
            UINT8 clk_adj_slots = hci_cmd_ptr->cmd_parameter[3];
            INT16 clk_adj_us =
            hci_cmd_ptr->cmd_parameter[4]| ((hci_cmd_ptr->cmd_parameter[5]& 0x7F)<< 8);
            if (hci_cmd_ptr->cmd_parameter[5]&BIT7)
            {
                clk_adj_us = 0 - clk_adj_us;
            }
            UINT8 clk_adj_mode = hci_cmd_ptr->cmd_parameter[6];
            RT_BT_LOG(BLUE, DAPE_TEST_LOG576, 5,
            clk, instant<<1, clk_adj_slots, clk_adj_us, clk_adj_mode);
            //RT_BT_LOG(WHITE, DAPE_TEST_LOG293, 1, clk_adj_us);

            UINT16 result = lmp_generate_clk_adj_pdu(instant, clk_adj_slots,
            clk_adj_us, clk_adj_mode);
            if (result != API_SUCCESS)
            {
                RT_BT_LOG(RED, DAPE_TEST_LOG293, 1, result);
            }
        }
        break;
#endif
#ifdef _DAPE_TEST_CHK_MWS_FRAME_SYNC
        case HCI_VENDOR_GEN_FAKE_FRAME_SYNC:
        {
            g_gen_frame_sync = hci_cmd_ptr->cmd_parameter[0];
        }
        break;
        case HCI_VENDOR_READ_FRAME_SYNC:
        {
            UINT16 reg276 = BB_read_baseband_register(MWS_CLKADJ_RPT0);
            UINT16 reg26a = BB_read_baseband_register(MWS_CLKADJ_RPT2);
            UINT8 sync_clk10 = (reg276 & ((UINT16)(0x3)));
        	   RT_BT_LOG(WHITE, DAPE_TEST_LOG561, 6,
        	   	BB_read_native_clock(),
        	   	BB_read_baseband_register(MWS_CLKADJ_CTRL4),
        	   	reg276,sync_clk10,
        	   	reg26a, reg26a);
        }
        break;


        case HCI_VENDOR_WRITE_FRAME_SYNC:
        {
            UINT16 clk_cnt_diff = hci_cmd_ptr->cmd_parameter[0]|
        	((hci_cmd_ptr->cmd_parameter[1] & (UINT8)(0x7F))<<8 );
            UINT16 clk_cnt_read = BB_read_baseband_register(MWS_CLKADJ_RPT2);
            UINT16 clk_cnt;
            if (hci_cmd_ptr->cmd_parameter[1] & BIT7)
            {
                clk_cnt = clk_cnt_read - clk_cnt_diff;
            }
            else
            {
                clk_cnt = clk_cnt_read + clk_cnt_diff;
            }
            UINT8 sync_clk10_write = hci_cmd_ptr->cmd_parameter[2] & ((UINT16)(0x3));
            UINT32 pre_clk = BB_read_native_clock();
            UINT16 reg276 = BB_read_baseband_register(MWS_CLKADJ_RPT0);
            UINT8 sync_clk10 = (reg276 & ((UINT16)(0x3)));

            UINT16 reg274_write = (BIT15|(sync_clk10_write<<10))|clk_cnt;
            BB_write_baseband_register(MWS_CLKADJ_CTRL4, reg274_write);
            RT_BT_LOG(GREEN, DAPE_TEST_LOG560, 11,
                BB_read_native_clock(), pre_clk,
                reg274_write,
                BB_read_baseband_register(MWS_CLKADJ_CTRL4),
                sync_clk10, sync_clk10_write,
                BB_read_baseband_register(MWS_CLKADJ_RPT2),
               (hci_cmd_ptr->cmd_parameter[1] & BIT8),
        	       clk_cnt_read, clk_cnt_diff, clk_cnt);
           g_modify_clk = 1;
        }
        break;

#endif
#ifdef _DAPE_SET_PCA_CLK_US_BY_VENDOR_CMD
        case HCI_VENDOR_SET_PCA_PARAM:
        {
        UINT16 piconet = 0;
        UINT16 adj_instant = hci_cmd_ptr->cmd_parameter[0] | (hci_cmd_ptr->cmd_parameter[1]<<8);
        UINT32 clk_adj_val_lsb = hci_cmd_ptr->cmd_parameter[2] |
                                           (hci_cmd_ptr->cmd_parameter[3]<<8);
        UINT16 clk_adj_cnt = ((hci_cmd_ptr->cmd_parameter[4] | (hci_cmd_ptr->cmd_parameter[5]<<8))
                             & (UINT16)(0x03FF));
        UINT32 pn_clk;
        lc_get_clock_in_scatternet(&pn_clk, piconet);

        BB_write_baseband_register(MWS_CLKADJ_CTRL2, adj_instant);
        BB_write_baseband_register(MWS_CLKADJ_CTRL0, clk_adj_val_lsb);
        BB_write_baseband_register(MWS_CLKADJ_CTRL1, (pn_clk>>16) | (piconet <<12) | BIT15);

        BZ_REG_S_MWS_CLKADJ_CTRL3 reg272;
        *(UINT16*) &reg272 = BB_read_baseband_register(MWS_CLKADJ_CTRL3);
        reg272.clk_adj_cnt9_0 = clk_adj_cnt;
        reg272.clk_adj_inst21_16 = ((adj_instant)>> 16);
        BB_write_baseband_register(MWS_CLKADJ_CTRL3, *(UINT16*)&reg272);

        RT_BT_LOG(GREEN, DAPE_TEST_LOG562, 9, pn_clk,
        adj_instant, BB_read_baseband_register(MWS_CLKADJ_CTRL2)| ((BB_read_baseband_register(MWS_CLKADJ_CTRL3)>>10)<<16),
        ((pn_clk>>16) | (piconet <<12) | BIT15),
        clk_adj_val_lsb, clk_adj_cnt,
        BB_read_baseband_register(MWS_CLKADJ_CTRL1),
        BB_read_baseband_register(MWS_CLKADJ_CTRL0),
        BB_read_baseband_register(MWS_CLKADJ_CTRL3)& (0x3FF));
        }
        break;
#endif

#ifdef _DAPE_TEST_ALWAYZ_NAK_ESCO_BY_VENDOR_CMD
        case HCI_VENDOR_ALWAYS_NAK_ESCO:
        {
            g_nak_esco = hci_cmd_ptr->cmd_parameter[0];
        }
        break;
#endif

#ifdef _DAPE_CHANGE_ESCO_RETRY_PRIORITY_BY_VENDOR_CMD
        case HCI_VENDOR_CHANGE_ESCO_RETRY_PRI_CMD:
        {
            UINT8 priority = hci_cmd_ptr->cmd_parameter[0];
            if (priority)
            {
            UINT16 reg = BB_read_baseband_register(0x5E);
            BB_write_baseband_register(0x5E, reg|BIT14);
            reg = BB_read_baseband_register(0x120);
            BB_write_baseband_register(0x120, reg | BIT8);
            }
            else
            {
            UINT16 reg = BB_read_baseband_register(0x5E);
            BB_write_baseband_register(0x5E, reg&(~BIT14));
            reg = BB_read_baseband_register(0x120);
            BB_write_baseband_register(0x120, reg & (~BIT8));
            }

            RT_BT_LOG(WHITE, DAPE_TEST_LOG207, 2, BB_read_baseband_register(0x5E),
            BB_read_baseband_register(0x120));
        }
        break;
#endif
#ifdef _DAPE_TEST_KILL_LE_CE_BY_VENDOR_CMD
        case HCI_VENDOR_KILL_LE_CE:
        {
            UINT8 ce_entry;
            ce_entry = hci_cmd_ptr->cmd_parameter[0];
            UINT16 reg_val = RD_LE_REG(0xD2);
            RT_BT_LOG(WHITE, DAPE_TEST_LOG213, 2, reg_val, (BIT0<<ce_entry));

            WR_LE_REG(0xD2, reg_val& (~(BIT0<<ce_entry)));
            RT_BT_LOG(GREEN, DAPE_TEST_LOG213, 2, RD_LE_REG(0xD2), (BIT0<<ce_entry));

        }
        break;
#endif

#ifdef _DAPE_TEST_SEND_PING_RES_BY_VENDOR_CMD
        case HCI_VENDOR_RESPOND_TO_PING_REQ:
        {
            g_send_ping_response = hci_cmd_ptr->cmd_parameter[0];
        }
        break;
#endif
#ifdef _DAPE_TEST_SEND_PTT_REQ_BY_VENDOR_CMD
        case HCI_VENDOR_SEND_PTT_REQ:
        {
            UINT8 ptt;
            UINT8 i;
            ptt = hci_cmd_ptr->cmd_parameter[0];
            for (i = 0; i < LMP_MAX_CE_DATABASE_ENTRIES; i++)
            {
                if (lmp_connection_entity[i].entity_status == ASSIGNED)
                {
                    lmp_generate_lmp_ptt_req_pdu(i, ptt);
                }
            }
        }
        break;

#endif
#ifdef _DAPE_TEST_SEND_ZERO_LEN_PKT_BY_VENDOR_CMD
        case HCI_VENDOR_SEND_ZERO_LEN_PKT:
        {
            g_send_zero_len = hci_cmd_ptr->cmd_parameter[0];
        }
        break;
#endif

#ifdef _SECURE_CONN_REFRESH_KEY_WHEN_CONTINUOUS_MIC
        case HCI_VENDOR_REFRESH_KEY_WHEN_CONTINUOUS_MIC_ERR:
        {
            g_sc_refresh_key_when_continuous_mic_err = hci_cmd_ptr->cmd_parameter[0];
        }
        break;
#endif

#ifdef _SECURE_CONN_REFRESH_KEY_BY_VENDOR_CMD
        case HCI_VENDOR_REFRESH_KEY:
        {
            g_refresh_key_en = hci_cmd_ptr->cmd_parameter[0];
        }
        break;
#endif

#ifdef _SECURE_CONN_TEST_CHK_ACLU_DATA_BY_VENDOR_CMD
        case HCI_VENDOR_CHK_SECURE_CONN_DATA:
        {
            g_chk_secure_conn_data = hci_cmd_ptr->cmd_parameter[0];
        }
        break;
#endif

#ifdef _DAPE_TEST_DISABLE_DM1_FOR_CCPT_BY_VENDOR_CMD
        case HCI_VENDOR_DISABLE_DM1:
        {
            g_disable_dm1 = hci_cmd_ptr->cmd_parameter[0];
        }
        break;
#endif
#ifdef _DAPE_TEST_CSB_RX_SELF_CALC_CLK_OFST_BY_VENDOR_CMD
        case HCI_VENDOR_CSB_RX_SELF_CALC_CLK_OFST:
        {
            g_self_calc_clk_ofst = hci_cmd_ptr->cmd_parameter[0];
        }
        break;
#endif
#ifdef _DAPE_TEST_DISABLE_EDR_BY_VENDOR_CMD_FOR_BR_PKT
        case HCI_VENDOR_SET_PTT:
        {
            g_ptt_enabled = hci_cmd_ptr->cmd_parameter[0];
        }
        break;
#endif
#ifdef _CSB_RX_SET_PTT_BY_VENDOR_CMD
        case HCI_VENDOR_SET_CSB_RX_PTT:
        {
            UINT16 ce_index;
            UINT16 temp;
            LMP_CONNECTION_ENTITY *ce_ptr;

            ce_index = bt_3dd_var.csb_rx_param.ce_index;
            ce_ptr = &lmp_connection_entity[ce_index];
            temp = BB_read_baseband_register(reg_SCA_SLAVE_UPPER_LUT[ce_ptr->phy_piconet_id]);

            if (hci_cmd_ptr->cmd_parameter[0])
            {
                temp |= (BIT15);
            }
            else
            {
                temp &= (~BIT15);
            }
            BB_write_baseband_register(reg_SCA_SLAVE_UPPER_LUT[ce_ptr->phy_piconet_id], temp);
#ifndef _ROM_CODE_PATCHED_REDUCE_
            RT_BT_LOG(RED, DAPE_TEST_LOG213, 2, reg_SCA_SLAVE_UPPER_LUT[ce_ptr->phy_piconet_id],
            BB_read_baseband_register(reg_SCA_SLAVE_UPPER_LUT[ce_ptr->phy_piconet_id]));
#endif
        }
        break;
#endif
#ifdef _CSB_RX_SET_XTOL_BY_VENDOR_CMD
        case HCI_VENDOR_SET_CSB_RX_XTOL:
        {
            g_beacon_rx_xtol = (hci_cmd_ptr->cmd_parameter[0]);
        }
        break;
#endif

#ifdef _CSB_RX_DBG_LOG
    case HCI_VENDOR_ENABLE_CSB_RX_LOG:
    {
        g_csb_rx_dbg_log = (hci_cmd_ptr->cmd_parameter[0]);
    }
    break;
#endif
#ifdef _DAPE_TEST_DISABLE_AFH_POWER_CTRL_FOR_PING_TEST_BY_VENDOR_CMD
    case HCI_VENDOR_DISALE_AFH_N_POWER_CTRL:
        if (hci_cmd_ptr->cmd_parameter[0])
        {
            lmp_feature_data.feat_page0[4] &= (~AFH_CAPABLE_SLAVE);
            lmp_feature_data.feat_page0[4] &= (~AFH_CLASSIFICATION_SLAVE);
            lmp_feature_data.feat_page0[5] &= (~AFH_CAPABLE_MASTER);
            lmp_feature_data.feat_page0[5] &= (~AFH_CLASSIFICATION_MASTER);
            lmp_feature_data.feat_page0[7] &= (~LMP_EPC_FEATURE);
            lmp_feature_data.feat_page0[2] &= (~LMP_POWER_CONTROL_FEATURE);
        }
        else
        {
            lmp_feature_data.feat_page0[4] |= (AFH_CAPABLE_SLAVE);
            lmp_feature_data.feat_page0[4] |= (AFH_CLASSIFICATION_SLAVE);
            lmp_feature_data.feat_page0[5] |= (AFH_CAPABLE_MASTER);
            lmp_feature_data.feat_page0[5] |= (AFH_CLASSIFICATION_MASTER);
            lmp_feature_data.feat_page0[7] |= (LMP_EPC_FEATURE);
            lmp_feature_data.feat_page0[2] |= (LMP_POWER_CONTROL_FEATURE);

        }
        break;
#endif
#ifdef _DAPE_TEST_CHG_NATIVE_CLK_FOR_ESCO_DAY_COUNTER
    case HCI_VENDOR_CHG_NATIVE_CLOCK:
{
    UINT32 set_time;
    UINT32 set_time_msb;
    UINT32 set_time_lsb;
    //hci_cmd_ptr->cmd_parameter[0] * 60 * 1000;
    set_time = TIMER_VAL_TO_SLOT_VAL(hci_cmd_ptr->cmd_parameter[0] * 60 * 1000) * 2;
    set_time_msb = ((0xFFFFFFF - set_time) >> 16 ) & 0xFFFF;
    set_time_lsb = ((0xFFFFFFF - set_time)) & 0xFFFF;
#ifndef _ROM_CODE_PATCHED_REDUCE_
    RT_BT_LOG(GREEN, DAPE_TEST_LOG525, 6,
    TIMER_VAL_TO_SLOT_VAL(180000),
    TIMER_VAL_TO_SLOT_VAL(180000) * 2,
    0xFFFFFFF - set_time,
    set_time,
    set_time_msb,
    set_time_lsb);
#endif
    BB_write_baseband_register(NATIVE_CLOCK2_REGISTER, set_time_msb);
    BB_write_baseband_register(NATIVE_CLOCK1_REGISTER, set_time_lsb);

    //BB_write_baseband_register(NATIVE_CLOCK2_REGISTER, 0x0FF7);
    //BB_write_baseband_register(NATIVE_CLOCK1_REGISTER, 0x35FF);
#ifndef _ROM_CODE_PATCHED_REDUCE_
    RT_BT_LOG(RED, DAPE_TEST_LOG207, 1, BB_read_native_clock());
#endif
}
    break;
#endif
#ifdef _DAPE_TEST_GEN_FAKE_ESCO_DATA
    case HCI_VENDOR_GEN_ESCO_DATA:
        {
            g_gen_fake_esco_data = hci_cmd_ptr->cmd_parameter[0];
            if (g_gen_fake_esco_data)
            {
                DEF_CRITICAL_SECTION_STORAGE;

                if (lmp_self_device_data.test_mode == HCI_REMOTE_LOOPBACK_MODE)
                {
                    if (lc_sca_manager.master_cnt == 0)
                    {
                        /* do not generate fake esco data when we are slave in
                           remote wakeup mode (austin) */
                        return status;
                    }
                }

                MINT_OS_ENTER_CRITICAL();

                gen_esco_data_packet(TRUE);

                MINT_OS_EXIT_CRITICAL();
            }
        }
        break;
#endif

#ifdef _CCH_TEST_DISABLE_EIR_BY_VENDOR_CMD
    case HCI_VENDOR_DISABLE_EIR:
        {
            bb_enable_eir(hci_cmd_ptr->cmd_parameter[0]);
        }
        break;
#endif

#ifdef _CCH_VENDOR_CMD_ESCO_DAYCOUNTER
    case HCI_VENDOR_WRITE_ESCO_DAYCOUNTER:
        {
            UINT32 daycounter;

            daycounter = hci_cmd_ptr->cmd_parameter[1] |
                        (hci_cmd_ptr->cmd_parameter[2] << 8) |
                        (hci_cmd_ptr->cmd_parameter[3] << 16)|
                        (hci_cmd_ptr->cmd_parameter[4] << 24);

            BB_write_sc_esco_first_flag(hci_cmd_ptr->cmd_parameter[0], daycounter);
        }
        break;

    case HCI_VENDOR_READ_ESCO_DAYCOUNTER:
        {
            UINT32 daycounter;

            daycounter = hci_cmd_ptr->cmd_parameter[1] |
                        (hci_cmd_ptr->cmd_parameter[2] << 8) |
                        (hci_cmd_ptr->cmd_parameter[3] << 16)|
                        (hci_cmd_ptr->cmd_parameter[4] << 24);

            BB_read_sc_esco_daycounter(hci_cmd_ptr->cmd_parameter[0], &daycounter);
#ifndef _ROM_CODE_PATCHED_REDUCE_
			    RT_BT_LOG(YELLOW, CCH_DBG_167, 1,daycounter);
#endif
        }
        break;

#endif


#ifdef BT_DRIVER_PROVIDE_PROFILE_INFO
    case HCI_VENDOR_ENABLE_PROFILE_RPT:
        if (hci_cmd_ptr->cmd_parameter[0] > 0)
        {
            g_host_set_profile = TRUE;
        }
        else
        {
            g_host_set_profile = FALSE;
        }
        break;

    case HCI_VENDOR_SET_PROFILE_RPT:
        status = hci_handle_set_profile_info_cmd(hci_cmd_ptr);
        break;
#endif

#ifdef CONFIG_TV_POWERON
    case HCI_VENDOR_TV_POWER_ON_OFF:
        status = hci_vendor_handle_tv_power_on_off_cmd(hci_cmd_ptr,
                &send_cmd_comp_event_flag);
        break;

    case HCI_VENDOR_TV_SET_LE_SCAN_PARAM:
        status = hci_vendor_handle_tv_set_le_scan_param_cmd(hci_cmd_ptr,
                &send_cmd_comp_event_flag);
        break;
#endif

#ifndef IS_BTSOC
#ifdef _ROM_CODE_PATCHED_
    case HCI_VENDOR_DOWNLOAD:
        status = hci_handle_vendor_download_cmd(hci_cmd_ptr);
        break;
#endif
#endif

    case HCI_VENDOR_SET_ASSOC:
        status = COMMAND_DISALLOWED_ERROR;
        if (hci_cmd_ptr->param_total_length == 17)
        {
            UINT8 index_h;
            UINT8 i;
            UINT8 *ptr = lmp_self_device_data.rtk_assoc_data + 6;

            /* copy index and random array from hci command */
            index_h = hci_cmd_ptr->cmd_parameter[0];
            index_h ^= 0x55;

            if (index_h > 15)
            {
                /* invalid range */
                break;
            }

            for (i = 0; i < 6; i++)
            {
                ptr[i] = hci_cmd_ptr->cmd_parameter[index_h + 1] ^ 0x55;
                ptr[i] ^= otp_str_data.bt_bd_addr[i];
                index_h = (index_h + 1) & 0x0F;
            }

            status = HCI_COMMAND_SUCCEEDED;
        }
        break;

    case HCI_VENDOR_GET_ASSOC:
        break;

    case HCI_VENDOR_CTRL_BTON_GPIO:
        gpio_ctrl_set_in_out(GEN_BIT_MASK(hci_cmd_ptr->cmd_parameter[0]),
                            (hci_cmd_ptr->cmd_parameter[1] == OUTPUT_HI_Z ? 0 : 0x1FFFFF));
        if(hci_cmd_ptr->cmd_parameter[1]<2)// if output mode, set H/L
            gpio_ctrl_write_gpio(hci_cmd_ptr->cmd_parameter[0], hci_cmd_ptr->cmd_parameter[1]);
        break;

/*	Added by Wallice for USB LPM HCI Vendor command.	2012/03/19	*/
	case HCI_VENDOR_USB_LPM_CTRL:
        status = COMMAND_DISALLOWED_ERROR;

#ifndef  _SUPPORT_POLLING_BASED_LPM_L1_
        if (hci_cmd_ptr->param_total_length == 2)
        {
            BTON_USB_LPM_REG_S_TYPE bton_USB_LPM_reg;

            bton_USB_LPM_reg.d32 = VENDOR_READ(BTON_USB_LPM);

            if (hci_cmd_ptr->cmd_parameter[0] == 0x01)
            {
                bton_USB_LPM_reg.b.LPM_Allow = 1;
                *((unsigned char *)&bton_USB_LPM_reg) = hci_cmd_ptr->cmd_parameter[1];
            }
            else if (hci_cmd_ptr->cmd_parameter[0] == 0x00)
            {
                bton_USB_LPM_reg.b.LPM_Allow = 1;
            }

            VENDOR_WRITE(BTON_USB_LPM, bton_USB_LPM_reg.d32);
            status = HCI_COMMAND_SUCCEEDED;
        }
#else
        if (hci_cmd_ptr->param_total_length == 1)
        {

            if (hci_cmd_ptr->cmd_parameter[0] & BIT0)
            {
                set_lpm_l1_token_reponse(L1_ACK);
            }
            else
            {
                set_lpm_l1_token_reponse(L1_NYET);
            }

            status = HCI_COMMAND_SUCCEEDED;
        }
#endif

		break;
/*	End Added by Wallice for USB LPM HCI Vendor command.	2012/03/19	*/
#ifdef _SUPPORT_POWERON_ENABLE_LOG_MUX_
    case HCI_VENDOR_8703B_BTGPIO_LOG_ENABLE:
        status = COMMAND_DISALLOWED_ERROR;
        if (hci_cmd_ptr->param_total_length == 1)
        {
            if (hci_cmd_ptr->cmd_parameter[0] & BIT0)
            {
#ifdef _SUPPORT_BTGPIO10_AS_LOG1_
                if(hci_cmd_ptr->cmd_parameter[0] & BIT1)
                {
                    SWITCH_BTON_GPIO_TO_OFF_DOMAIN(BTON_GPIOMAP_10);
                    off_domain_log_control(OUTPUT_LOG1);
                }
                else
#endif
                {
                    SWITCH_BTON_GPIO_TO_OFF_DOMAIN(BTON_GPIOMAP_08);
                    off_domain_log_control(OUTPUT_LOG0);
                }

                //SWITCH_BTON_GPIO_TO_OFF_DOMAIN(BTON_GPIOMAP_08);
                //off_domain_log_control(OUTPUT_LOG0);
            }
            else
            {
                SWITCH_BTON_GPIO_TO_ON_DOMAIN(BTON_GPIOMAP_08|BTON_GPIOMAP_10);
                off_domain_log_control(LOG_OFF);
            }
            status = HCI_COMMAND_SUCCEEDED;
        }

        //status = HCI_COMMAND_SUCCEEDED;
        break;
#endif

#ifdef _SPIC_FUNC_ENABLE_
    case HCI_VENDOR_SPI_INIT:
        spi_init();
        break;

    case HCI_VENDOR_SPI_ERASE:
        status = hci_handle_spi_erase_cmd(hci_cmd_ptr);
        break;

    case HCI_VENDOR_SPI_WRITE:
        status = hci_handle_spi_write_cmd(hci_cmd_ptr);
        break;

    case HCI_VENDOR_SPI_READ:
        status = COMMAND_DISALLOWED_ERROR;
        if ((hci_cmd_ptr->param_total_length == 4) && is_spi_init)
        {
            if (hci_cmd_ptr->cmd_parameter[3] == 0)
            {
                break;
            }

            hci_vendor_spi_address = hci_cmd_ptr->cmd_parameter[0] |
                                    (hci_cmd_ptr->cmd_parameter[1] << 8) |
                                    (hci_cmd_ptr->cmd_parameter[2] << 16);
            hci_vendor_spi_len =  hci_cmd_ptr->cmd_parameter[3];

            status = HCI_COMMAND_SUCCEEDED;
        }
        break;
#endif

    case HCI_VENDOR_SET_LOG_ENABLE:
        status = hci_handle_vendor_set_log_enable_cmd(hci_cmd_ptr);
        break;

#ifdef _DAPE_TEST_NEW_HW
    case HCI_VENDOR_SET_MUTE_ENABLE:
        status = hci_handle_vendor_set_mute_enable_cmd(hci_cmd_ptr);
        break;
#endif

    case HCI_VENDOR_WRITE_BB_REGISTER:
        status = hci_handle_vendor_write_bb_reg_cmd(hci_cmd_ptr);
        break;

    case HCI_VENDOR_WRITE:
    case HCI_VENDOR_READ:
        status = hci_handle_vendor_read_write_reg_cmd(hci_cmd_ptr);
        break;

#ifdef _SUPPORT_VENDER_READ_SIE_
    case HCI_VENDOR_READ_SIE:
        status =hci_handle_vendor_read_sie_reg_cmd(hci_cmd_ptr);
        break;
#endif
    case HCI_VENDOR_READ_BB_REGISTER:
        hci_vendor_g_read_address = (hci_cmd_ptr->cmd_parameter[1]<<8)
                                    | hci_cmd_ptr->cmd_parameter[0];
        break;

#ifdef BT2FM_INDIRECT_ACCESS
    case HCI_VENDOR_FM_SWITCH:
        status = hci_vendor_fm_switch_hdlr(hci_cmd_ptr);
        break;
    case HCI_VENDOR_FM_READ:
        status = hci_vendor_fm_read_hdlr(hci_cmd_ptr);
        break;
    case HCI_VENDOR_FM_WRITE:
        status = hci_vendor_fm_write_hdlr(hci_cmd_ptr);
        break;
    case HCI_VENDOR_FM_SET:
        status = hci_vendor_fm_set_hdlr(hci_cmd_ptr);
        break;
#ifdef FM_DEBUG
    case HCI_VENDOR_FM_TEST_STATUS:
        status = hci_vendor_fm_test_status_hdlr(hci_cmd_ptr);
        break;
#endif
#endif/* BT2FM_INDIRECT_ACCESS */

#ifdef MWS_ENABLE
     case HCI_VENDOR_MWS_INDIRECT_READ:
        {
            UINT16 reg_addr;
            reg_addr = hci_cmd_ptr->cmd_parameter[0]|(hci_cmd_ptr->cmd_parameter[1]<<8);
            RT_BT_LOG(BLUE,YL_DBG_HEX_2,2,reg_addr,mws_read_register_new(reg_addr));
        }
     break;

     case HCI_VENDOR_MWS_INDIRECT_WRITE:
        {
            UINT32 reg_write;
            UINT16 reg_addr;
            reg_addr = hci_cmd_ptr->cmd_parameter[0]|(hci_cmd_ptr->cmd_parameter[1]<<8);

            reg_write = hci_cmd_ptr->cmd_parameter[2] | (hci_cmd_ptr->cmd_parameter[3]<<8) |
                        (hci_cmd_ptr->cmd_parameter[4]<<16) | (hci_cmd_ptr->cmd_parameter[5]<<24) ;
            mws_write_register(reg_addr,reg_write,0xf);
            RT_BT_LOG(BLUE,YL_DBG_HEX_2,2,reg_addr,mws_read_register_new(reg_addr));
        }
     break;

#endif

    case HCI_VENDOR_WRITE_RF_TRANSMIT_TEST_CONFIG:
        {
            memcpy(rf_transmit_test_params, hci_cmd_ptr->cmd_parameter, 11);
            hci_set_vendor_rf_transmit_test_config(hci_cmd_ptr->cmd_parameter);
        }
        break;

    case HCI_VENDOR_SET_PCMI2S_PARAM:
        status = hci_handle_vendor_set_PCMI2SParam_cmd(hci_cmd_ptr);

        break;

    case HCI_VENDOR_START_RF_TRANSMIT_TEST:
        hci_start_rf_transmit_test();
        break;

    case HCI_VENDOR_RF_RADIO_INIT:
        if(hci_cmd_ptr->param_total_length != 0)
        {
            return INVALID_HCI_COMMAND_PARAMETERS_ERROR;
        }
#ifdef _CCH_RETENTION_FLOW_FOR_DLPS_
        lc_init_radio(FALSE);
#else
        lc_init_radio();
#endif
        break;

    case HCI_VENDOR_READ_RF_TRANSMIT_TEST_CONFIG:
        /* Generate command complete */
        send_cmd_comp_event_flag = 1;
        break;

    case HCI_VENDOR_STOP_RF_TRANSMIT_TEST:
        hci_stop_rf_transmit_test();
        break;

    case HCI_VENDOR_RF_RADIO_REG_READ:
        if(hci_cmd_ptr->param_total_length != 1)
        {
            return INVALID_HCI_COMMAND_PARAMETERS_ERROR;
        }
        hci_vendor_g_read_radio_address=hci_cmd_ptr->cmd_parameter[0];
        send_cmd_comp_event_flag = 1;
        break;

    case HCI_VENDOR_RF_RADIO_REG_WRITE:
        {
            UCHAR reg_offset;
            UCHAR bModem;
            UINT32 data;

            if(hci_cmd_ptr->param_total_length != 4)
            {
                return INVALID_HCI_COMMAND_PARAMETERS_ERROR;
            }

            /* p0 is the reg_address */
            reg_offset = hci_cmd_ptr->cmd_parameter[0];
            bModem = reg_offset >> 7;
            reg_offset &= 0x7f;

            /* [MSB] p3 p2 p1 [LSB] is the data */
            data = hci_cmd_ptr->cmd_parameter[1] |
                  (hci_cmd_ptr->cmd_parameter[2] << 8) |
                  (hci_cmd_ptr->cmd_parameter[3] << 16);
            rtk_write_modem_radio_reg(reg_offset, bModem, data);
        }
        break;

#ifdef _MODEM_LNA_CONSTRAINT_
    case HCI_VENDOR_FORCE_MODEM_LNA_CONST_OFF:
        {
          lc_force_modem_lna_constraint_off();
          send_cmd_comp_event_flag = 1;
        }
        break;
#endif

#if 0
#ifdef _MODEM_HCI_VENDOR_8821A_LOK_
    case HCI_VENDOR_EXECUTE_8821A_LOK:
        {
            UINT8 delay_ms = hci_cmd_ptr->cmd_parameter[0];
            UINT8 lok_rf_ch = hci_cmd_ptr->cmd_parameter[1];
            UINT8 lok_tx_gain = hci_cmd_ptr->cmd_parameter[2];
            //UINT8 psd_cycle_no_idac = hci_cmd_ptr->cmd_parameter[3];
            //UINT8 tx_psd_pts = hci_cmd_ptr->cmd_parameter[4];
            g_hci_vendor_read_data = rtl8821_btrf_lok(delay_ms, lok_rf_ch, lok_tx_gain);
            send_cmd_comp_event_flag = 1;
        }
        break;
#endif
#endif

#ifdef _NEW_MODEM_PI_ACCESS_
    case HCI_VENDOR_RF_RADIO_REG_READ_PI:
        if(hci_cmd_ptr->param_total_length != 2) // length = 2; {addr, page}
        {
            return INVALID_HCI_COMMAND_PARAMETERS_ERROR;
        }
        hci_vendor_g_read_radio_address=hci_cmd_ptr->cmd_parameter[0];
        {
            hci_vendor_g_read_radio_modem_page = hci_cmd_ptr->cmd_parameter[1];
        }
        send_cmd_comp_event_flag = 1;
        break;

    case HCI_VENDOR_RF_RADIO_REG_WRITE_PI:
        {
            UCHAR reg_offset;
            UCHAR bModem;
            UINT32 data;

            if(hci_cmd_ptr->param_total_length != 4)
            {
                return INVALID_HCI_COMMAND_PARAMETERS_ERROR;
            }

            /* p0 is the reg_address */
            reg_offset = hci_cmd_ptr->cmd_parameter[0];
            bModem = reg_offset >> 7;
            reg_offset &= 0x7f;

            UINT8 modem_page = hci_cmd_ptr->cmd_parameter[1];
            /* [MSB] p3 p2 p1 [LSB] is the data */
            data = hci_cmd_ptr->cmd_parameter[2] |
                  (hci_cmd_ptr->cmd_parameter[3] << 8);

            if (bModem == TYPE_MODEM)
            {
//                rtk_write_modem_radio_reg_pi(modem_page, reg_offset, bModem, data);
                RTK_WRITE_MODEM_REG_PI(modem_page, reg_offset, (UINT16)data);

            }
            else
            {
                rtk_write_modem_radio_reg(reg_offset, bModem, data);
            }
        }
        break;
#endif

#ifdef _ENABLE_BTON_POWER_SAVING_
    case HCI_VENDOR_UART_MODE_SUSPEND:
        {
            send_cmd_comp_event_flag = 0;
            MINT_OS_ENTER_CRITICAL();

#ifdef _UART_H5
#ifdef _8821A_BTON_DESIGN_
//            if (hci_uart_h5_go_sleep() == API_SUCCESS)
            if (1)
#else
            if (!hci_uart_h5_go_sleep())
#endif
#endif
            {
#ifdef _8821A_BTON_DESIGN_
                BTON_UART_INFO_REG_S_TYPE bton_uart_info;
                EFUSE_POW_SETTING_3_S efuse_pow_setting_3;
                *(UINT8*)&efuse_pow_setting_3 = otp_str_data.efuse_pow_setting_3_d8;

                bton_uart_info.d32 = VENDOR_READ(BTON_UART_INFO_REG);
                bton_uart_info.b.bton_pre_boot_state = BIT1;
                if (efuse_pow_setting_3.record_bton_pre_boot_state)
                {
                    VENDOR_WRITE(BTON_UART_INFO_REG, bton_uart_info.d32);
                }
                bton_set_pcm_pull_low(PCM_PULL_OPTION_PDN_SUS_ST);
                pf_delay_us((hci_cmd_ptr->cmd_parameter[0]<<4));
#endif
                enable_uart_suspend();
            }
#ifndef _8821A_BTON_DESIGN_
            trun_on_off_afe_ldo(OFF);
#endif
            MINT_OS_EXIT_CRITICAL();
        }
        break;

#ifndef IS_BTSOC
    case HCI_VENDOR_EXECUTE_LPS:
        send_cmd_comp_event_flag = 0;
        if (hci_cmd_ptr->cmd_parameter[0] == 0)
        {
            execute_lps_mode_procedure(0,0);
        }
        else if (hci_cmd_ptr->cmd_parameter[0] >= 1)
        {  /* new */
            UINT32 clock, wakeup_instant;
            UINT8 bz_wake_en = (hci_cmd_ptr->cmd_parameter[0] == 1);
            bton_clr_wakeup_sts();
            DEF_CRITICAL_SECTION_STORAGE;
            MINT_OS_ENTER_CRITICAL();
            clock = BB_read_native_clock();
            clock = clock >> 1;
            wakeup_instant = clock  + hci_cmd_ptr->cmd_parameter[1] + (hci_cmd_ptr->cmd_parameter[2]<<8);
            wakeup_instant = wakeup_instant & 0xFFFF;
            pf_delay_us((hci_cmd_ptr->cmd_parameter[3]<<4));
            execute_lps_mode_procedure_6128(bz_wake_en, wakeup_instant);
            MINT_OS_EXIT_CRITICAL();
#ifdef LPS_PROTECT_DSM_BB_ACCESS // YL TEST 2
            {
                while(sleep_mode_param.bb_sm_sts != BB_NORMAL_MODE)
                {
                    //pf_delay_us(1);
                    rlx4081sleep(); /* do not busy wait and let cpu sleep */
                }
            }
#endif
        }

        break;
#endif
#endif

    case HCI_VENDOR_PROFILE_REPORT:
        send_cmd_comp_event_flag = 0;
#ifdef _ENABLE_RTK_PTA_
        pta_profile_manage(hci_cmd_ptr->cmd_parameter);
#endif
        break;

    case HCI_VENDOR_WIFI_CL_EN:
#ifdef _ENABLE_MAILBOX_
        {
            send_cmd_comp_event_flag = 0;
            UINT32_S mailbox_in;

            mailbox_in.u1Byte[0] = WIFI_CL_EN;
            mailbox_in.u1Byte[1] = 1;
            mailbox_in.u1Byte[2] = (hci_cmd_ptr->cmd_parameter[0] | 0x80);
            mailbox_in.u1Byte[3] = 0;

            RT_BT_LOG(BLUE,NOTIFY_WIFI_CALIBRATION,0,0);
            pf_os_trigger_mailbox_task(MAILBOX_WRITE, mailbox_in.u4Byte, 0);
        }
#endif
        break;

    case HCI_VENDOR_HOST_ENTER_SLEEP_MODE:
        /* we can use rcp_hci_vendor_cmd_func to replace it to reduce mem size */

        //if (rcp_hci_vendor_host_enter_sleep_mode != NULL)
        //{
        //    rcp_hci_vendor_host_enter_sleep_mode((void*)hci_cmd_ptr, &send_cmd_comp_event_flag);
        //}
        //else
        {
            MINT_OS_ENTER_CRITICAL();
            g_host_state = 1;
            MINT_OS_EXIT_CRITICAL();
        }
        break;

#ifdef _YL_RTL8273A_B_CUT
    case HCI_VENDOR_SET_HOST_WAKE_BT:
        MINT_OS_ENTER_CRITICAL();
        g_host_wake_bt = 1;
        MINT_OS_EXIT_CRITICAL();
        break;

    case HCI_VENDOR_CLR_HOST_WAKE_BT:
        MINT_OS_ENTER_CRITICAL();
        g_host_wake_bt = 0;
        MINT_OS_EXIT_CRITICAL();
        break;
#endif

#ifdef _UART_BAUD_ESTIMATE_
    case HCI_VENDOR_UART_SYNC:
        send_cmd_comp_event_flag = 1;
        MINT_OS_ENTER_CRITICAL();
        hci_uart_man_sram.hci_vendor_uart_sync_received = 1;
        if (g_data_uart_settings.baud_est_stop_at_rx_vendor_sync)
        {
            hci_uart_man_sram.hci_uart_baud_est_state = HCI_UART_BAUD_EST_ST_STOP;
            if (g_data_uart_settings.baud_record_at_rx_vendor_sync)
            {
                // for original BuadDet to avoid baudrate un-sync-ed //
                hci_uart_man_sram.baud_current_setting = hci_uart_read_hw_setting();
            }
            if (g_data_uart_settings.baud_est_wr_bton_en_at_h5link_uartsync)
            {
                BAUDRATE_FORMAT_TYPE baud_hw = hci_uart_read_hw_setting();
                hci_uart_baud_record_to_bton(baud_hw.d32);
            }
        }
        MINT_OS_EXIT_CRITICAL();
        break;
#endif

#ifdef _8821A_BTON_DESIGN_
    case HCI_VENDOR_UART_PARA_CHANGE:
        {
            if (hci_cmd_ptr->param_total_length != 13)
            {
                return INVALID_HCI_COMMAND_PARAMETERS_ERROR;
            }
            g_hci_cmd_ptr_for_evt_gen = hci_cmd_ptr;
            send_cmd_comp_event_flag = 1;
            hci_uart_man.chg_para_valid_flag = 1;
        }
        break;
#endif

    case HCI_VENDOR_SET_BAUDRATE:
#ifdef _UART_H5
        DMA_UART_DBG_LOG(RED, DMA_UART_003_DBG_HEX, 1, 0xffff);
        send_cmd_comp_event_flag = 1;
        MINT_OS_ENTER_CRITICAL();
        hci_uart_man.baud_new.d32 = (UINT32)(hci_cmd_ptr->cmd_parameter[0] |
                                            (hci_cmd_ptr->cmd_parameter[1]<<8) |
                                            (hci_cmd_ptr->cmd_parameter[2]<<16) |
                                            (hci_cmd_ptr->cmd_parameter[3]<<24));
        hci_uart_man.baud_new_valid_flag = 1;
        hci_uart_man_sram.hci_vendor_set_baud_received = 1;
        MINT_OS_EXIT_CRITICAL();
#endif
        break;

#ifdef _DAPE_TEST_AUTO_CONN
    case HCI_VENDOR_CLEAR_WHITE_LIST:
        {
            UINT8 i = 0;
            for (i = 0; i < LEGACY_MAX_WHITE_LIST_SIZE ; i++)
            {
                memset(&conn_white_list[i], 0, sizeof(conn_white_list[i]));
            }
            num_of_white_list_device = 0;
    	}
        break;

    case HCI_VENDOR_ADD_DEVICE_TO_WHITE_LIST:
        status = hci_handle_add_dev_to_white_list_cmd(hci_cmd_ptr);
        break;

    case HCI_VENDOR_REMOVE_DEVICE_FROM_WHITE_LIST:
        status = hci_handle_rem_dev_from_white_list_cmd(hci_cmd_ptr);
        break;
#endif

#ifdef MINICARD_BT_LED_CONTROL
    case HCI_VENDOR_BT_DEV_MGR_ENABLE:
        if (hci_cmd_ptr->cmd_parameter[0] == 0)
        {
            /* Turn Off BT LED */
            BT_LED_WPAN_OFF();
        }
        break;
#endif

#ifdef _RTK_8723A_B_CUT_NEW_HCI_VENDOR_CMD_
    case HCI_VENDOR_FORCE_RESET_AND_ALLOW_PATCH:
    case HCI_VENDOR_FORCE_RESET:
            if (hci_cmd_ptr->cmd_opcode == HCI_VENDOR_FORCE_RESET_AND_ALLOW_PATCH)
            {
                ERASE_ROM_CODE_PATCH_SIGNATURE;
            }

#ifdef _UART_H5
#ifdef _YL_RTL8723A_B_CUT
            if (g_fun_interface_info.b.bt_interface == UART_INTERFACE)
            {
                UINT32 uart_status_d32 = (UINT32)UART_DWORD_READ(DMA_UART_H5_INTSTS);
                SIGN_PATCH_UART_STATUS(uart_status_d32);
            }
#endif
#endif
            SIGN_FW_TRIG_WDG_TO_SIGNATURE;
            WDG_TIMER_TIMEOUT_SOON;
            while (1); /* infinite loop here to wait watch dog timeout */

#ifdef _IS_ASIC_
        case HCI_VENDOR_SET_THERMAL_DEFAULT:
            /* update the default value of referenced thermal */
            otp_str_data.EFuse_ThermalDefault = hci_cmd_ptr->cmd_parameter[0];
            break;

    case HCI_VENDOR_SET_THERMAL_TIMER_INTERVAL:
        {
            /* update the thermal meter timer interval */
            UINT16 interval = hci_cmd_ptr->cmd_parameter[0] * 100; /* unit: 100ms */

            if (g_rtl8723_btrf_thermal_value_timer != OS_INVALID_HANDLE)
            {
                OS_DELETE_TIMER(&g_rtl8723_btrf_thermal_value_timer);
            }

            if (interval != 0)
            {
                /* call the callback function immediately */
                rtl8723_btrf_UpdateThermalValueTimer();

                OS_CREATE_TIMER((UCHAR)PERIODIC_TIMER,
                     &g_rtl8723_btrf_thermal_value_timer,
                     (OS_TIMEOUT_HANDLER)rtl8723_btrf_UpdateThermalValueTimer,
                     (void *)NULL, 0);
                OS_START_TIMER(g_rtl8723_btrf_thermal_value_timer, interval);
            }
        }
        break;

    case HCI_VENDOR_SET_CFO_TRACK_ENABLE:
        /* update the CFO tracking enable bit */
        otp_str_data.EFuse_CFOTrack_En =
                            hci_cmd_ptr->cmd_parameter[0] ? TRUE : FALSE;
        break;
#endif
#endif

#ifdef _BT_ONLY_
#ifdef _PG_EFUSE_VIA_HCI_VENDOR_COMMAND_
    case HCI_VENDOR_WRITE_EFUSE_DATA:
    case HCI_VENDOR_READ_EFUSE_DATA:
        status = hci_handle_read_write_efuse_data_cmd(hci_cmd_ptr);
        break;
#endif
#endif

#if defined(_LE_AUTO_REPORT_RSSI_AND_LOGIN_INOUT_FUNC_) && defined(LE_MODE_EN)
    case HCI_VENDOR_SET_LE_RSSI_THRESHOLD:
        status = hci_handle_set_le_rssi_threshold_cmd(hci_cmd_ptr);
        break;
#endif

#ifdef MINICARD_BT_LED_CONTROL
    case HCI_VENDOR_SET_BT_LED_MODE:
        bt_led_control(hci_cmd_ptr->cmd_parameter[0], hci_cmd_ptr->cmd_parameter[1]);
        break;
#endif

    case HCI_VENDOR_LPS_SETTING:
    {
		// 00: No LPS
		// 03: LPS
		// 07: DLPS
		// 0B: LPS no wakeup
		// 0F: DLPS no wakeup

        g_efuse_lps_setting_5.disable_never_enter_lps_when_usb_active =
                            (hci_cmd_ptr->cmd_parameter[0])&BIT0;

        g_efuse_lps_setting_2.timer2_lps_on =
                            (hci_cmd_ptr->cmd_parameter[0]>>1)&BIT0;
        UINT8 hci_lps_timer_en = (hci_cmd_ptr->cmd_parameter[0]>>1)&BIT0;

#ifdef _CCH_RETENTION_FLOW_FOR_DLPS_
        g_efuse_rsvd_2.enable_deep_lps_mode =
                            (hci_cmd_ptr->cmd_parameter[0]>>2)&BIT0;
#endif

#ifdef _MODI_LPS_AFTER_RTL8821B_TC_
        g_efuse_lps_setting_2.lps_nowakeup_when_no_scan =
                            (hci_cmd_ptr->cmd_parameter[0]>>3)&BIT0;

        g_efuse_lps_setting_2.le_lps_enable =
                            (hci_cmd_ptr->cmd_parameter[0]>>4)&BIT0;

        g_efuse_lps_setting_2.adjust_lps_scan_interval =
                            (hci_cmd_ptr->cmd_parameter[0]>>5)&BIT0;
#endif
#ifdef _CCH_RETENTION_FLOW_FOR_DLPS_
        g_efuse_rsvd_2.le_link_lps_enable =
                            (hci_cmd_ptr->cmd_parameter[0]>>6)&BIT0;
#endif

#ifdef _MODI_LPS_STATE_WITH_INTR_
        g_efuse_rsvd_2.lps_state_with_intr =
                            (hci_cmd_ptr->cmd_parameter[0]>>7)&BIT0;
#endif

        otp_str_data.power_seq_param &= (~BIT0);
        otp_str_data.power_seq_param |= hci_lps_timer_en;

#ifdef _CCH_RETENTION_FLOW_FOR_DLPS_
        sleep_mode_param.run_dlps_flow = g_efuse_rsvd_2.enable_deep_lps_mode;
#endif

        sleep_mode_param.lps_log_enable =
                            (hci_cmd_ptr->cmd_parameter[1])&BIT0;

        break;
    }

    case HCI_VENDOR_READ_RTK_ROM_VERSION:
        break;


#ifdef _PLC_TEST_USED_VENDOR_COMMAND_
    case HCI_VENDOR_SEND_TEST_DATA:
        status = hci_vendor_handle_send_test_data_cmd(hci_cmd_ptr);
        break;
#endif

    case HCI_VENDOR_SET_THERMAL_TRACK_PAUSE:
        send_cmd_comp_event_flag = 1;
        status = HCI_COMMAND_SUCCEEDED;
        g_thermal_track_pause = (hci_cmd_ptr->cmd_parameter[0]!=0);
        break;

#ifdef _SUPPORT_MSFT_BT_HCI_EXTENSION_
    case HCI_VENDOR_MSFT_EXTENSION:
        status = hci_handle_vs_msft_cmd_set(hci_cmd_ptr,
                                            &send_cmd_comp_event_flag,
                                            &send_cmd_status_event_flag);
        break;
#endif

#ifdef _BT4_2_PRIVACY_FEATURE_SUPPORT_

    case HCI_VENDOR_PRIVACY_ENABLE_OPTION:
        send_cmd_comp_event_flag = 1;
        if (hci_cmd_ptr->cmd_parameter[0] > HCI_SUB_VENDOR_PRIVACY_MAX)
        {
            status = UNKNOWN_HCI_COMMAND_ERROR;
            break;
        }

        status = HCI_COMMAND_SUCCEEDED;

        switch (hci_cmd_ptr->cmd_parameter[0])
        {
            case HCI_SUB_VENDOR_PRIVACY_ENABLE_LOG:
                ll_manager.show_resolvable_addr = (hci_cmd_ptr->cmd_parameter[1] > 0);
                break;

            case HCI_SUB_VENDOR_PRIVACY_ENABLE_DBG:
                ll_manager.debug_privacy_enable = (hci_cmd_ptr->cmd_parameter[1] > 0);
                break;

            case HCI_SUB_VENDOR_PRIVACY_ENABLE_COMPARE_RPA:
                ll_manager.comp_rpa_lbd_opt_enable = (hci_cmd_ptr->cmd_parameter[1] > 0);
                break;

            case HCI_SUB_VENDOR_PRIVACY_ENABLE_FORCE_BT40_OPT:
                ll_manager.force_bt40_opt = (hci_cmd_ptr->cmd_parameter[1] > 0);
                break;

            default:
                status = UNKNOWN_HCI_COMMAND_ERROR;
                break;
        }

        break;

#endif /* _BT4_2_PRIVACY_FEATURE_SUPPORT_ */

    default:
        status = UNKNOWN_HCI_COMMAND_ERROR;
        break;
    }

    if ( send_cmd_comp_event_flag == 1 )
    {
        hci_vendor_cmd_generate_event(hci_cmd_ptr->cmd_opcode, status, arg);
    }
    else if ( send_cmd_status_event_flag == 1 )
    {
        hci_generate_command_status_event(hci_cmd_ptr->cmd_opcode, status);
    }

    return status;
}


/**************************************************************************
 * Function   : hci_vendor_cmd_generate_event
 *
 * Description: This function is used to generate event.
 *
 * Parameters : cmd_opcode: event opcode
 *              status: success or failure
 *              arg: extra argument
 *
 * Returns    : None.
 *
 *************************************************************************/
BOOLEAN hci_vendor_cmd_generate_event(UINT16 cmd_opcode, UINT8 status, void *arg)
{
    HCI_EVENT_PKT   *hci_event_pkt_ptr ;
    OS_SIGNAL        signal_send;

    if ((hci_event_pkt_ptr = hci_vendor_generate_command_complete_event(
                                                   cmd_opcode, status, arg)) == NULL)
    {
        BT_FW_HCI_ERR(GENERATION_OF_COMMAND_COMPLETE_EVENT_FAILED,0,0);
        return FALSE;
    }

    /* Signal to event task for delivering the event */
    signal_send.type = HCI_DELIVER_HCI_EVENT_SIGNAL ;
    signal_send.param =(OS_ADDRESS) hci_event_pkt_ptr ;

    if (OS_SEND_SIGNAL_TO_TASK(hci_event_task_handle, signal_send)
            !=BT_ERROR_OK)
    {
        BT_FW_HCI_ERR(LOG_LEVEL_HIGH, OS_SEND_SIGNAL_TO_TASK_FAILED,0,0);
    }

    return TRUE;

}

/**************************************************************************
 * Function   : hci_set_vendor_rf_transmit_test_config
 *
 * Description: This function is used to set rf test config.
 *
 * Parameters : params: parameter point
 *
 * Returns    : None.
 *
 *************************************************************************/
void hci_set_vendor_rf_transmit_test_config(UCHAR *params)
{
    UINT16 read;
    UINT16 length;
    UCHAR am_addr;
    UINT16 radio_sel_val;
    UINT8 tx_gain_idx;

    am_addr = params[8];

    read = (UINT16) ((params[1] & 0x7F) | ((params[1] & 0x7F) << 8) );

    BB_write_baseband_register(CHANNEL_REGISTER,read);
    OR_val_with_bb_reg(SCO_PACKET_TYPE_REGISTER,0x3 << 5);  /* b11 */

    radio_sel_val = 0x07;
    radio_sel_val = (UINT16) (radio_sel_val | (params[7] & 1) << 12);
    radio_sel_val = (UINT16) ((radio_sel_val | ((params[2] & 0x7) << 9) | 0x100 ) );

    /* Check if the pattern is burst. */
    if (params[7] == 0x2) /* burst mode is selected. */
    {
        radio_sel_val |= (0x1 << 4); /* Set bit-4. */
    }

    BB_write_baseband_register(RADIO_SELECT_REGISTER,radio_sel_val); /* pattern */

    /* Packet type  & length */
    read = BB_read_baseband_register(reg_MASTER_LOWER_LUT[am_addr]);
    read = (UINT16) (read & ~0xF3FF);  /* Clear length also */
    read = (UINT16) ( read | ((params[3] & 0xF) << 12) );
    length = (UINT16) ((params[4]) | (params[5] << 8));
    read = (UINT16) (read | length );
    BB_write_baseband_register(reg_MASTER_LOWER_LUT[am_addr],read);

    /* byte 3: bit 4 will be PTT. already bits 0-3 are used for the packet type. */
    read = params[3];
    read >>= 4;
    read &= 0x01;
    {
        /* read modify write, the upper lut with the PTT. */
        UINT16 lcl_upper_lut;
        lcl_upper_lut = BB_read_baseband_register(reg_MASTER_UPPER_LUT[am_addr]);
        lcl_upper_lut = (UINT16) (lcl_upper_lut & 0x7FFF);
        lcl_upper_lut = (UINT16) (lcl_upper_lut | (read << 15));
        BB_write_baseband_register(reg_MASTER_UPPER_LUT[am_addr],lcl_upper_lut);
    }

    tx_gain_idx = params[0]& 0x7;

    /* Byte 9 is the num of packets. */
    read = BB_read_baseband_register(reg_MASTER_UPPER_LUT[am_addr]) ;
    read = (UINT16) (read & 0xE1EF);
    read = (UINT16) (read | (tx_gain_idx << 9) ); /* Transmit power */

#ifdef _SUPPORT_LBT_FUNC_FOR_RF_ADAPTIVITY_
    if (rtl8723_btrf_check_and_enable_lbt(tx_gain_idx))
    {
        /* enable lbt_en_lut bit */
        read |= BIT12;
    }
#endif

    BB_write_baseband_register(reg_MASTER_UPPER_LUT[am_addr],read);

    LC_SET_2M_TX_POWER(am_addr, tx_gain_idx);
    LC_SET_3M_TX_POWER(am_addr, tx_gain_idx);
}

/**************************************************************************
 * Function   : hci_start_rf_transmit_test
 *
 * Description: This function is used to start rf test.
 *
 * Parameters : None.
 *
 * Returns    : None.
 *
 *************************************************************************/
void hci_start_rf_transmit_test(void)
{
    UINT32 radio_sel_val;

    OR_val_with_bb_reg(SCO_PACKET_TYPE_REGISTER, (BIT4 | BIT7));//disable whitening
    OR_val_with_bb_reg(reg_MASTER_UPPER_LUT[rf_transmit_test_params[8] & 7], BIT8 );

    radio_sel_val = 0x07;
    radio_sel_val = radio_sel_val | (rf_transmit_test_params[7] & 1) << 12;
    radio_sel_val |= ( ((rf_transmit_test_params[2] & 0x7) << 9) | 0x100);

    /* Check if the pattern is burst. */
    if (rf_transmit_test_params[7] == 0x2) /* burst mode is selected. */
    {
        radio_sel_val |= (0x1 << 4); /* Set bit-4. */

    }
    BB_write_baseband_register(PICONET1_INFO_REGISTER, 0x03);
    BB_write_baseband_register(RADIO_SELECT_REGISTER, (UINT16) radio_sel_val); /* pattern */

    AND_val_with_bb_reg(SCO_PACKET_TYPE_REGISTER, 0xf7ff);
    OR_val_with_bb_reg(PICONET1_INFO_REGISTER, 0x03);
}

/**************************************************************************
 * Function   : hci_stop_rf_transmit_test
 *
 * Description: This function is used to stop rf test.
 *
 * Parameters : None.
 *
 * Returns    : None.
 *
 *************************************************************************/
void hci_stop_rf_transmit_test(void)
{
    AND_val_with_bb_reg(SCO_PACKET_TYPE_REGISTER,(UINT16) ~(BIT4 | BIT7)); // enable whitening
    AND_val_with_bb_reg(reg_MASTER_UPPER_LUT[rf_transmit_test_params[8] & 7],
                       (UINT16) ~(1 << 8));

    BB_write_baseband_register(RADIO_SELECT_REGISTER, 0x107); /* pattern */
    BB_write_baseband_register(PICONET1_INFO_REGISTER, 0x00);
}

#ifdef _VENDOR_RESET_BB_
void hci_vendor_reset_bb(void)
{
    UINT16 temp1;
    UINT16 temp2;
#ifdef _DAPE_TEST_NEW_HW
    UINT16 temp3;
#endif
#ifndef _ROM_CODE_PATCHED_REDUCE_
#ifdef LE_MODE_EN
    //RT_BT_LOG(BLUE, CCH_DBG_146, 0,0);
    RT_BT_LOG(GRAY, CCH_DBG_147, 13, lmp_self_device_data.number_of_hlc, ll_manager.conn_unit.enable, ll_manager.conn_unit.connection_cnts,
       lmp_connection_entity[0].ce_status, lmp_connection_entity[1].ce_status, lmp_connection_entity[2].ce_status, lmp_connection_entity[3].ce_status,
       lmp_connection_entity[4].ce_status, lmp_connection_entity[5].ce_status, lmp_connection_entity[6].ce_status, lmp_connection_entity[7].ce_status,
       lmp_connection_entity[8].ce_status, lmp_connection_entity[9].ce_status);
#endif
#endif
    /* Disable All interrupts */
    BB_write_baseband_register(INTERRUPT_MASK_REGISTER, 0xFFFF);
    BB_write_baseband_register(INTERRUPT_MASK_REGISTER2, 0xFFFF);

    temp1 = BB_read_baseband_register(SCA_PRIORITY_REGISTER);
    temp2 = BB_read_baseband_register(SCA_PRIORITY_REGISTER2);
#ifdef _DAPE_TEST_NEW_HW
    temp3 = BB_read_baseband_register(SCA_PRIORITY_REGISTER3);
#endif

#ifdef _CCH_RETENTION_FLOW_FOR_DLPS_
    lc_baseband_partial_reset(FALSE);
#else
    lc_baseband_partial_reset();
#endif

    BB_write_baseband_register(SCA_PRIORITY_REGISTER, temp1);
    BB_write_baseband_register(SCA_PRIORITY_REGISTER2, temp2);
#ifdef _DAPE_TEST_NEW_HW
    BB_write_baseband_register(SCA_PRIORITY_REGISTER3, temp3);
#endif
}
#endif

#if 1
UCHAR hci_handle_vendor_set_PCMI2SParam_cmd(HCI_CMD_PKT *hci_cmd_ptr)
{
    if (hci_cmd_ptr->param_total_length == 0x09)
    {
        pcmifctrl1 = hci_cmd_ptr->cmd_parameter[0] |  (hci_cmd_ptr->cmd_parameter[1] << 8);
        pcmifctrl2 = hci_cmd_ptr->cmd_parameter[2] |  (hci_cmd_ptr->cmd_parameter[3] << 8);
        pcmifctrl3 = hci_cmd_ptr->cmd_parameter[4] |  (hci_cmd_ptr->cmd_parameter[5] << 8);
        pcmconvert = hci_cmd_ptr->cmd_parameter[6];
        scoconv = hci_cmd_ptr->cmd_parameter[7];
        hci_excodec_state = hci_cmd_ptr->cmd_parameter[8];

        pcm_ex_codec_format_8bit = (pcmifctrl1 >> 0x06) & 0x01;
        pcm_ex_codec_format = (hci_excodec_state >> 0x04) & 0xF;

        return HCI_COMMAND_SUCCEEDED;
    }

    return INVALID_HCI_COMMAND_PARAMETERS_ERROR;
}
#endif
#endif//(RT_VENDOR_CMDS)

