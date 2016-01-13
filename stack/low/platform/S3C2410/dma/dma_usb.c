/***************************************************************************
 Copyright (C) RealTek Ltd.
 This module is a confidential and proprietary property of RealTek and
 a possession or use of this module requires written permission of RealTek.
 ***************************************************************************/

enum { __FILE_NUM__= 120 };

/* ========================= Include File Section ========================= */
#include "dma_usb.h"
#include "logger.h"
#include "UartPrintf.h"
#include "bt_fw_os.h"
#include "S3C2410_task.h"
#include "bb_driver.h"
#include "hci_td.h"
#include "bt_fw_hci.h"
#include "lc.h"
#include "uart.h"
#include "mem.h"
#ifdef LE_MODE_EN
#include "le_ll.h"
#include "le_hci_4_0.h"
#endif
#include "gpio.h"
#include "bb_driver.h"
#include "otp.h"
#include "hci_vendor_defines.h"
#include "bz_fw_isoch.h"
#include "bt_fw_hci_internal.h"
#include "h5.h"
#ifdef CONFIG_TV_POWERON
#include "tv.h"
#endif

#ifdef _SUPPORT_USB_TIMEOUT_INTERRUPT_
#include "scoreboard.h"
#include "new_io.h"
#endif
//20120109 morgan
#ifdef PTA_EXTENSION
#include "pta_meter.h"
#endif
#ifdef _SUPPORT_INFO_FROM_SYSTEM_ON_    
#include "system_on.h"
#endif

UINT32 *dma_acl_tx_des;
UINT32 *dma_sco_tx_des;
UINT32 *dma_cmd_tx_des;
#ifdef LE_MODE_EN
UINT32 *dma_le_acl_tx_des;
#endif
DMA_NEW_RX_DES_TYPE *dma_acl_rx_des;
DMA_NEW_RX_DES_TYPE *dma_sco_rx_des;
DMA_NEW_RX_DES_TYPE *dma_evt_rx_des;

USB_RX_PKT_FUNC_PTR RxPktFunc;
USB_TX_COMP_FUNC_PTR TxCompFunc;
DMA_MANAGER_TYPE dma_man;

UINT8 cmd_fifo_index[EVT_RX_DES_NUM];
UINT8 acl_fifo_index[ACL_RX_DES_NUM];
UINT8 sco_fifo_index[SCO_RX_DES_NUM];

CHIP_FUN_AND_INTERFACE_S_TYPE g_fun_interface_info;
UINT32 g_baudRate; 
UINT32 g_uart_clk; /* never needed ?? */
UINT8  g_buffer_free_type = 0;
UINT8  g_hci_reset_buf_allcate = 0;

UINT8 g_usb_iso_alternate_value = 2;  /* record current alternate setting */
UINT8 g_usb_iso_alternate_change[ISOCH_SCO_MAX_CONNS] = {FALSE,FALSE,FALSE};

extern BOOLEAN bz_isoch_is_usb_ready;
extern volatile UCHAR bz_sco_pkt_size;
static UINT8 sco_pkt_size[6] = {0, 24, 48, 72, 96, 144};

HCI_ACL_DATA_PKT_WS acl_h2c_data_ws[BT_FW_TOTAL_ACL_PKTS_FROM_HOST];

UINT8 first_iso_in = 0;
UINT8 dectect_pkt_flag = 0;
UINT8 temp_check_value = 0;
UINT8 wakeup_isr_en = 0;
#ifdef _FORCE_ENTER_HCI_LOOPBACK_MODE_
UINT8 enable_hci_loopback_mode = 1;
#else
UINT8 enable_hci_loopback_mode = 0;
#endif
#ifdef _SUPPORT_INFO_FROM_SYSTEM_ON_    
extern UINT8 g_chip_id;
#endif            

#ifdef _SUPPORT_POLLING_BASED_LPM_L1_
UINT32 g_check_lpm_timer_cnt = 0;
LPM_POLL_CONTROL g_lpm_poll_control_s;
UINT32 g_print_d2_timer_cnt = 0;
#endif

#ifdef _SUPPORT_USB_LOG_ENABLE_
UINT8 g_usb_misc;
#endif

UINT8 g_rx_interrupt_happened = 0;
//UINT32 g_wake_count = 0;
UINT32 g_dbginfo_timer_L1_wake_count = 0;

#ifdef _DAPE_TEST_CHECK_SCO_TX_INDEX
UINT32 dape_sco_tx_index = 0;
#endif
#ifdef _SCO_SEND_SINE_TONE_PKT_
#ifdef _SCO_SEND_SINE_TONE_PKT_AND_COMPARE
UINT32 sco_num_in_tx_pkt = 0;
#endif
#endif
UINT8 g_uart_h5_fw_allow = 0;
UINT8 g_wake_first_time = 1;
UINT8 g_host_state = 0;
#ifdef _YL_RTL8723A_B_CUT
UINT8 g_host_wake_bt = 1; // 1: HOST expects BT alive (not in LPS mode); ??? Q: should be ignored in USB/PCI-e cases???
#endif
//alternating seting clear index
UINT8 alternatset_change_tx_clear = 0;
UINT8 alternatset_change_rx_clear = 0;

#ifdef _SUPPORT_USB_TIMEOUT_INTERRUPT_
extern UINT16 g_u16usb_token_timeout_timer_cnt;
enum{USB_TO_DONOTHOIN = 0, USB_TO_DIS, USB_TO_DISEN, USB_TO_TIMER};
#endif

/* the rx descriptor entity of hci dma in SRAM */
ALIGN(8) SECTION_SRAM DMA_NEW_RX_DES_TYPE g_dma_rx_des_entity[DMA_RX_ENTRY_NUM];

/* the tx descriptor entity of hci dma in SRAM */
ALIGN(8) SECTION_SRAM UINT32 g_dma_tx_des_entity[DMA_TX_ENTRY_NUM];

/* the current read pointer of tx descriptor entity */
UINT8 g_dma_tx_des_entity_rptr = 0;

/* the current read pointer of rx descriptor entity */
UINT8 g_dma_rx_des_entity_rptr = 0;

#ifdef _SUPPORT_NEW_USB_LPM_L1_DATA_PATH_150629_
UINT8 g_usb_lpm_l1_hci_dma_notification_queue_bm = 0;
#ifdef _ALLOW_ENTER_USB_LPM_L1_VIA_HW_TIMER2_
UINT8 g_usb_lpm_l1_hw_timer_is_running = 0;
#endif
#endif

extern UINT32 g_lps_timer_counter;
extern UINT32 g_timer2_init_value;
#ifdef _SECURE_CONN_TEST_CHK_ACLU_DATA_BY_VENDOR_CMD
extern UINT8 g_chk_secure_conn_data;
#endif
/* ================== For rom code patch function point ================== */
#ifdef _ROM_CODE_PATCHED_
#ifdef _CCH_8723_RCP_
PF_ROM_CODE_PATCH_FUNC rcp_USB_DMA_IntrHandler_lps_reset = NULL;
#endif
PF_ROM_CODE_PATCH_FUNC rcp_hci_dma_intr_h4_error_func = NULL;
PF_ROM_CODE_PATCH_FUNC rcp_usb_dma_intrhandler_func = NULL;
PF_ROM_CODE_PATCH_FUNC rcp_S3C2410Usb_Dma_wake_host = NULL;
#ifdef _YL_RTL8723A_B_CUT
//PF_ROM_CODE_PATCH_FUNC rcp_S3C2410Usb_Dma_func = NULL;
#endif

PF_ROM_CODE_PATCH_FUNC rcp_usb_req_rx_dma_func = NULL;

#ifdef _SUPPORT_NEW_USB_LPM_L1_DATA_PATH_150629_
PF_ROM_CODE_PATCH_FUNC rcp_hci_dma_usb_check_usb_lpm_l1_then_queue_notification = NULL;
PF_ROM_CODE_PATCH_FUNC rcp_hci_dma_usb_check_and_allow_enter_usb_lpm_l1 = NULL;
PF_ROM_CODE_PATCH_FUNC rcp_hci_dma_usb_check_and_dequeue_notification = NULL;
#endif

#endif

/**
 * Function     : dma_init
 *
 * Description  : This function initialises "USB DMA Interface". It primarily
 *              allocates start address for command, acl sco fifo and
 *              initialises hardware setting.
 *
 * Parameters   : None.
 *
 * Returns      : OK
 *
 * Side Effects : None
 *
*/
BOOLEAN dma_init(UINT8 init_type)
{
    UINT8 i;
    DMA_ADDR_REG_TYPE           gen_dma_address_reg;
    ACL_TX_BUF_SIZE_TYPE        acl_tx_buf_size_reg;
    SCO_TX_BUF_SIZE_TYPE        sco_tx_buf_size_reg;
    CMD_TX_BUF_SIZE_TYPE        cmd_tx_buf_size_reg;
#ifdef LE_MODE_EN
    LE_ACL_TX_BUF_SIZE_TYPE     le_acl_tx_buf_size_reg;
#endif
    ISR_CONTRL_TYPE             isr_control_reg;
    DMA_CONTRL_TYPE             dma_control_reg;
    //SCO_CON_HANDLE_1_AND_2_TYPE sco_con_handle_1_and_2_reg;
    SCO_CON_HANDLE_3_TYPE       sco_con_handle_3_reg;

//    RT_BT_LOG(GREEN, DMA_INIT, 0, 0);

#ifdef _CCH_RETENTION_FLOW_FOR_DLPS_
if (init_type != INIT_FROM_DLPS)
#endif
{

#ifdef _ENABLE_BTON_POWER_SAVING_
    BTON_POW_CTRL_REG_S_TYPE pow_ctrl;

    pow_ctrl.d32 = VENDOR_READ(BTON_POW_CTRL_REG);
    pow_ctrl.b.gating_hci_clk_en = 1;
//    RT_BT_LOG(BLUE,POWER_CTRL_REG_VALUE,1,pow_ctrl.d32);
    VENDOR_WRITE(BTON_POW_CTRL_REG,pow_ctrl.d32);

    EFUSE_POW_PARA_S_TYPE efuse_pow_para;
    efuse_pow_para.d16 = otp_str_data.power_seq_param;
    if (efuse_pow_para.b.usb_sie_ny_en)
    {
//        RT_BT_LOG(BLUE,USB_SIE_NY_ENABLE,0,0);
#ifdef _NEW_BTON_DESIGN_AFTER_RTL8821B_TC_
        BTON_74_REG_S_TYPE bton_74;
        bton_74.d32 = VENDOR_READ(BTON_74_REG);
        bton_74.Reg_BT_USB_NYEN = 1;
        VENDOR_WRITE(BTON_74_REG,bton_74.d32);
#else
        CORE_AND_AFE_LDO_CTRL_S_TYPE core_ctrl_reg;
        core_ctrl_reg.d32 = VENDOR_READ(BTON_CORE_AFE_LDO_CTRL_REG);
        core_ctrl_reg.b.usb_lpm_ny_en = 1;
        VENDOR_WRITE(BTON_CORE_AFE_LDO_CTRL_REG,core_ctrl_reg.d32);
#endif  
    }
    RT_BT_LOG(GREEN, DMA_INIT_2, 2, pow_ctrl.d32, efuse_pow_para.b.usb_sie_ny_en);
#else
    RT_BT_LOG(GREEN, DMA_INIT, 0, 0);
#endif
}

    g_hci_reset_buf_allcate = 1;
#ifndef _DAPE_TEST_FIX_DMA_HANG
    g_usb_iso_alternate_value = 0;
#else
    g_usb_iso_alternate_value = 2;
#endif

    for (i = 0; i < ISOCH_SCO_MAX_CONNS; i++)
    {
        g_usb_iso_alternate_change[i] = FALSE;
    }

    if (init_type == INIT_FIRST)
    {
        switch (g_fun_interface_info.b.bt_interface)
        {
            case USB_INTERFACE:
                RT_BT_LOG(GREEN, DMA_IS_USB_INTERFACE, 0, 0);
                break;
#ifndef _REMOVE_HCI_PCIE_                
            case PCIE_INTERFACE:
            {
                UINT8 temp_pkt_len = 0;
                RT_BT_LOG(GREEN, DMA_IS_PCIE_INTERFACE, 0, 0);

                //Set PCIE Configration packet length
/*  Fixed by Wallice for ACL Pkt Length error.  2011/02/10  */
//                DMA_WORD_WRITE(PCIE_DMA_ACL_MAX_PKT_LEN_REG, HCI_ACL_DATA_PKT_SIZE);
                DMA_WORD_WRITE(PCIE_DMA_ACL_MAX_PKT_LEN_REG, otp_str_data.bt_read_buffer_size);
/*  End Fixed by Wallice for ACL Pkt Length error.  2011/02/10  */
                temp_pkt_len = (HCI_SYNCHRONOUS_DATA_PKT_SIZE > 0xFF)? 0xFF:(HCI_SYNCHRONOUS_DATA_PKT_SIZE);
                DMA_BYTE_WRITE(PCIE_DMA_SCO_MAX_PKT_LEN_REG, temp_pkt_len);
                temp_pkt_len = (HCI_CMD_BUFFER_SIZE > 0xFF)? 0xFF:(HCI_CMD_BUFFER_SIZE);
                DMA_BYTE_WRITE(PCIE_DMA_CMD_MAX_PKT_LEN_REG, temp_pkt_len);

                DMA_DBG_LOG(GREEN, PCIE_DMA_ACL_MAX_LEN, 1, DMA_WORD_READ(PCIE_DMA_ACL_MAX_PKT_LEN_REG));
                DMA_DBG_LOG(GREEN, PCIE_DMA_SCO_MAX_LEN, 1, DMA_BYTE_READ(PCIE_DMA_SCO_MAX_PKT_LEN_REG));
                DMA_DBG_LOG(GREEN, PCIE_DMA_CMD_MAX_LEN, 1, DMA_BYTE_READ(PCIE_DMA_CMD_MAX_PKT_LEN_REG));
                break;
            }
#endif            
            case UART_INTERFACE:
            {
                
/* [20110310, yilinli] move the following to hci_uart_reset_init() for RTL8723*/

#ifdef _RTK8723_UART_INIT_ // for test only
//               hci_uart_reset_init_RTL8723(0);  note: move to other place, after interrupt enable
               break;
#else 
                UINT32 divisor;
                UINT32 dlh;
                UINT32 dll;

                RT_BT_LOG(GREEN, DMA_IS_UART_INTERFACE, 0, 0);
                DMA_DBG_LOG(GREEN, DMA_UART_CLK, 1, g_uart_clk);
                DMA_DBG_LOG(GREEN, DMA_UART_BAUDRATE, 1, g_baudRate);

                // set DLAB bit to 1
                UART_DWORD_WRITE(UART_LINE_CTL_REG_OFF, 0x83);

                // set up buad rate division (avoid rounding)
                //divisor = (g_uart_clk + (g_baudRate << 3))/ (g_baudRate << 4);
                divisor = g_uart_clk / (g_baudRate << 4);

                dll = divisor & 0xff;
                dlh = (divisor & 0xff00)>>8;
                UART_DWORD_WRITE(UART_DLL_OFF, dll);
                UART_DWORD_WRITE(UART_DLH_OFF, dlh);

                // clear DLAB bit
                UART_DWORD_WRITE(UART_LINE_CTL_REG_OFF, 0x03);

                //Turn on flow control
                UART_DWORD_WRITE(UART_MODEM_CTL_REG_OFF, 0xa0);               

                break;
#endif    
            }
            default:
                RT_BT_LOG(RED, DMA_DO_NOT_SUPPORT_THIS_INTERFACE, 1, 
                                           g_fun_interface_info.b.bt_interface);
                break;
        }

        dma_des_init();
        g_hci_reset_buf_allcate = 0;
    }

#ifdef _SUPPORT_POLLING_BASED_LPM_L1_
#ifdef _SUPPORT_NEW_USB_LPM_L1_DATA_PATH_150629_
    g_usb_lpm_l1_hci_dma_notification_queue_bm = 0;
#ifdef _ALLOW_ENTER_USB_LPM_L1_VIA_HW_TIMER2_
    g_usb_lpm_l1_hw_timer_is_running = 0;
#endif
#endif
#endif

    if (init_type == INIT_FROM_DLPS)
    {
        UINT8 i;
        
        //initial rx des and set end bit
        //ACL
        for (i=0; i<ACL_RX_DES_NUM; i++)
        {
            dma_acl_rx_des[i].rx_des_dw1_u32 = 0;
            dma_acl_rx_des[i].rx_des_dw2_u32 = 0;
        }
        dma_acl_rx_des[ACL_RX_DES_NUM - 1].rx_des_dw1.end = 1;
        dma_acl_rx_des[ACL_RX_DES_NUM - 1].rx_des_dw1.payload_start_add = (UINT16)(0xFFFF & ((UINT32)dma_acl_rx_des));

        //SCO
        for (i=0; i<SCO_RX_DES_NUM; i++)
        {
            dma_sco_rx_des[i].rx_des_dw1_u32 = 0;
            dma_sco_rx_des[i].rx_des_dw2_u32 = 0;
        }
        dma_sco_rx_des[SCO_RX_DES_NUM - 1].rx_des_dw1.end = 1;
        dma_sco_rx_des[SCO_RX_DES_NUM - 1].rx_des_dw1.payload_start_add = (UINT16)(0xFFFF & ((UINT32)dma_sco_rx_des));

        //Event
        for (i=0; i<EVT_RX_DES_NUM; i++)
        {
            dma_evt_rx_des[i].rx_des_dw1_u32 = 0;
            dma_evt_rx_des[i].rx_des_dw2_u32 = 0;
        }
        dma_evt_rx_des[EVT_RX_DES_NUM - 1].rx_des_dw1.end= 1;
        dma_evt_rx_des[EVT_RX_DES_NUM - 1].rx_des_dw1.payload_start_add = (UINT16)(0xFFFF & ((UINT32)dma_evt_rx_des));

        g_hci_reset_buf_allcate = 0;
    }

    /* bind work space (dmem) with acl tx pkts (sram) */
    for (i = 0; i < BT_FW_TOTAL_ACL_PKTS_FROM_HOST; i++)
    {
        /* bind the ws in the dmem to the HCI ACL Data Pkt Body */
        HCI_ACL_DATA_PKT *pkt = (HCI_ACL_DATA_PKT *)(*(dma_acl_tx_des + i));
        memset(&acl_h2c_data_ws[i], 0, sizeof(HCI_ACL_DATA_PKT_WS));
        pkt->ws = &acl_h2c_data_ws[i];                
    }

    //================== TX Start Address Register Initial ========================
    //Initial ACL TX buffer start address
    DMA_DWORD_WRITE(ACL_TX_BUF_START_ADDR_REG, (0xFFFF & dma_acl_tx_des[0]));

    //Initial SCO TX buffer start address
    DMA_DWORD_WRITE(SCO_TX_BUF_START_ADDR_REG, (0xFFFF & dma_sco_tx_des[0]));

    //Initial Command TX buffer start address
    DMA_DWORD_WRITE(CMD_TX_BUF_START_ADDR_REG, (0xFFFF & dma_cmd_tx_des[0]));

#ifdef LE_MODE_EN
    //Initial LE ACL TX buffer start address
    DMA_DWORD_WRITE(LE_ACL_TX_BUF_START_ADDR_REG, (0xFFFF & dma_le_acl_tx_des[0]));
#endif

//================== TX Buffer Size Register Initial ========================
    //Intial ACL TX Buffer Size Register
    acl_tx_buf_size_reg.d32 = 0;
#ifdef _NEW_HCIDMA_FROM_V1_3_
    acl_tx_buf_size_reg.b.acl_tx_pkt_offset = 0;
#endif
    acl_tx_buf_size_reg.b.acl_tx_buf_entry_size =  HCI_ACL_DATA_PKT_SIZE;
    acl_tx_buf_size_reg.b.acl_tx_pkt_num = BT_FW_TOTAL_ACL_PKTS_FROM_HOST;
    DMA_DWORD_WRITE(ACL_TX_BUF_SIZE_REG, acl_tx_buf_size_reg.d32);

    //Initial SCO TX Buffer Size Register
    sco_tx_buf_size_reg.d32 = 0;
#ifdef _NEW_HCIDMA_FROM_V1_3_
    sco_tx_buf_size_reg.b.sco_tx_pkt_offset = 0;
#endif
    sco_tx_buf_size_reg.b.sco_tx_buf_entry_size =  HCI_SYNCHRONOUS_DATA_PKT_SIZE;
    sco_tx_buf_size_reg.b.sco_tx_pkt_num = BT_FW_TOTAL_SYNCHRONOUS_PKTS_FROM_HOST;
    DMA_DWORD_WRITE(SCO_TX_BUF_SIZE_REG, sco_tx_buf_size_reg.d32);

    //Initial Command TX Buffer Size Register
    cmd_tx_buf_size_reg.d32 = 0;
#ifdef _NEW_HCIDMA_FROM_V1_3_
    cmd_tx_buf_size_reg.b.cmd_tx_pkt_offset = 0;
#endif
    cmd_tx_buf_size_reg.b.cmd_tx_buf_entry_size =  HCI_CMD_BUFFER_SIZE;
    cmd_tx_buf_size_reg.b.cmd_tx_pkt_num = BT_FW_CMD_BUFFERS;
    DMA_DWORD_WRITE(CMD_BUF_SIZE_REG, cmd_tx_buf_size_reg.d32);

#ifdef LE_MODE_EN
    //Initial Command TX Buffer Size Register
    le_acl_tx_buf_size_reg.d32 = 0;
#ifdef _NEW_HCIDMA_FROM_V1_3_
    le_acl_tx_buf_size_reg.b.le_tx_pkt_offset = LL_POLL_HCI_TX_ACL_PKT_OFFSET;
#endif
    le_acl_tx_buf_size_reg.b.le_tx_buf_entry_size =  LL_POLL_HCI_MAX_TX_ACL_PKT_SIZE;
    le_acl_tx_buf_size_reg.b.le_acl_tx_pkt_num = LL_POLL_HCI_MAX_TX_ACL_PKT_CNT;
    DMA_DWORD_WRITE(LE_BUF_SIZE_REG, le_acl_tx_buf_size_reg.d32);
#endif

#ifdef _NEW_HCIDMA_FROM_V1_3_
    /* modify le tx size limit */
    DMA_DWORD_WRITE(LE_TX_HCI_SIZE_LIMIT_REG, (LL_POLL_HCI_MAX_TX_ACL_PKT_SIZE - LL_POLL_HCI_TX_ACL_PKT_OFFSET));
#endif

//================== RX Dexcription Ring Base Register Initial ========================

    //Initial ACL RX
    gen_dma_address_reg.d32 = 0;
    gen_dma_address_reg.b.addr = 0xFFFF & (UINT32) dma_acl_rx_des;
    DMA_DWORD_WRITE(ACL_RX_DES_RING_BASE_REG, gen_dma_address_reg.d32);

    //Initial SCO RX
    gen_dma_address_reg.d32 = 0;
    gen_dma_address_reg.b.addr = 0xFFFF & (UINT32) dma_sco_rx_des;
    DMA_DWORD_WRITE(SCO_RX_DES_RING_BASE_REG, gen_dma_address_reg.d32);

    //Initial Event RX
    gen_dma_address_reg.d32 = 0;
    gen_dma_address_reg.b.addr = 0xFFFF & (UINT32) dma_evt_rx_des;
    DMA_DWORD_WRITE(EVENT_DES_RING_BASE_REG, gen_dma_address_reg.d32);

    if(init_type != INIT_FROM_DLPS)
    {        
    //  USB_DMA_WRITE(EVENT_DES_RING_BASE_REG, 0xc530);
    //=================== DMA RX own bit clear ===========================================
#ifdef _DAPE_TEST_FIX_DMA_HANG
        for (i = 0; i < ACL_RX_DES_NUM; i++)
        {
            dma_acl_rx_des[i].rx_des_dw1.own = 0;
        }
        for (i = 0; i < SCO_RX_DES_NUM; i++)
        {
            dma_sco_rx_des[i].rx_des_dw1.own = 0;
        }
        for (i = 0; i < EVT_RX_DES_NUM; i++)
        {
            dma_evt_rx_des[i].rx_des_dw1.own = 0;
        }
#endif    
    }

    //=================== DMA SCO setting ================================================
    ////// alternate setting is read-only, so no use.
    //    sco_con_handle_1_and_2_reg.d32 = 0;
    //    sco_con_handle_1_and_2_reg.b.iso_al_setting = 2;
    //    DMA_DWORD_WRITE(SCO_CONNECTION_HANDLE_1_AND_2_REG, sco_con_handle_1_and_2_reg.d32);

    sco_con_handle_3_reg.d32 = 0;
#ifdef LE_MODE_EN
    if (IS_BT40)
    {
        sco_con_handle_3_reg.b.le_con_handle = LL_HCI_MIN_CONN_HANDLE;
        sco_con_handle_3_reg.b.le_con_han_value = 1;
    }
#endif
    DMA_DWORD_WRITE(SCO_CONNECTION_HANDLE_3_REG, sco_con_handle_3_reg.d32);

//if(init_type != INIT_FROM_DLPS) 
{
//=================== DMA Control Setting ============================================

    isr_control_reg.d32 = 0;
    isr_control_reg.b.cmd_tx_int_en = 1;
    isr_control_reg.b.acl_tx_int_en = 1;
    isr_control_reg.b.sco_tx_int_en = 1;
    isr_control_reg.b.evt_rx_int_en = 1;
    isr_control_reg.b.acl_rx_int_en = 1;
    isr_control_reg.b.sco_rx_int_en = 1;
    isr_control_reg.b.iso_alt_ch_int_en = 1;
#ifdef LE_MODE_EN
    isr_control_reg.b.le_acl_tx_int_en = 1;
#endif

#ifdef _NEW_HCIDMA_DESIGN_
    if ((otp_str_data.HCI_DMA_EnhanCtrl & 0x08))
    {
        isr_control_reg.b.USB_CMD_Tx_PktErr_int_en = 1; 
    }

    if ((otp_str_data.HCI_DMA_EnhanCtrl & 0x04))
    {
        isr_control_reg.b.USB_ACL_Tx_LongPktErr_int_en = 1; 
        isr_control_reg.b.USB_LE_Tx_LongPktErr_int_en = 1;  
    }
#endif

#ifdef _SUPPORT_USB_TIMEOUT_INTERRUPT_ // for debug only
    if(otp_str_data.efuse_misc_interrupt_d8 & 0x04)
    {
        isr_control_reg.b.evt_timeout_int_en = 1;
    }
    if(otp_str_data.efuse_misc_interrupt_d8 & 0x08)
    {
        isr_control_reg.b.iso_in_timeout_int_en = 1;
    }
    if(otp_str_data.efuse_misc_interrupt_d8 & 0x10)
    {
        isr_control_reg.b.iso_out_timeout_int_en = 1;
    }
    if(otp_str_data.efuse_misc_interrupt_d8 & 0x20)
    {
        UINT8 tmp = 0;
        tmp = DMA_BYTE_READ(USB_TOKEN_TIMEOUT_REG);
        tmp |= BIT0;
        DMA_BYTE_WRITE(USB_TOKEN_TIMEOUT_REG, tmp);
    }    
#endif

    DMA_DWORD_WRITE(ISR_ENABLE_REG, isr_control_reg.d32);


    dma_control_reg.d32 = 0;
    //Soft reset, do we need to hardware notify firmware that it had finished reset.
    dma_control_reg.b.dma_soft_rst = 1;
    DMA_DWORD_WRITE(DMA_CTL_REG, dma_control_reg.d32);

    while(dma_control_reg.b.dma_soft_rst == 1)
    {
        dma_control_reg.d32 = DMA_DWORD_READ(DMA_CTL_REG);
        //wait reset complete
        if (dma_control_reg.b.dma_soft_rst == 0)
        {
            if(init_type != INIT_FROM_DLPS)
            {        
                RT_BT_LOG(RED, DMA_SOFT_TEST_COMPLETE, 0, 0);
            }           
        }
    };

    dma_control_reg.b.sco_rx_int_mode = 0;
    dma_control_reg.b.acl_rx_int_mode = 0;
    dma_control_reg.b.evt_rx_int_mode = 0;
    dma_control_reg.b.sco_tx_int_thr = 1;
    dma_control_reg.b.sco_on = 1;
    dma_control_reg.b.dma_tx_en = 1;
    dma_control_reg.b.dma_rx_en = 1;
#ifdef _NEW_HCIDMA_FROM_V1_3_
    dma_control_reg.b.le_data_ext_en = 1;
#endif
    
#ifndef _REMOVE_HCI_PCIE_     
    //notify PCIE driver, device is ready.
    if ((g_fun_interface_info.b.bt_interface == PCIE_INTERFACE)&&(init_type == INIT_FIRST))
    {
        dma_control_reg.b.fw_set_ready = 1;
    }
#endif
    
#ifdef LE_MODE_EN
    dma_control_reg.b.le_acl_tx_pkt_buf_en = 1;
#endif
    DMA_DWORD_WRITE(DMA_CTL_REG, dma_control_reg.d32);

#ifdef _USB_DUMP_REG_
    dump_usb_dma_register();
#endif

}
    
    //sco_tx_data.buf_size = (bz_sco_pkt_size+3)*(SCO_TX_BUF_SIZE/(bz_sco_pkt_size+3));

    /*
     * Initial global variable
     */
    memset(&dma_man, 0, sizeof(DMA_MANAGER_TYPE));
    dma_man.evt_available_num = (EVT_RX_DES_NUM - 1);
    dma_man.acl_available_num = (ACL_RX_DES_NUM - 1);
    dma_man.sco_available_num = (SCO_RX_DES_NUM - 1);

    if(init_type != INIT_FROM_DLPS)
    {
        //For Test mode, check which type free
        switch_hci_dma_setting(LEAVE_TEST_MODE);
    }
    
#ifdef _ENABLE_32K_CLK_WAKE_UP_ISR_
    wake_up_interrupt_switch(DISABLE_WAKE_UP_ISR);
#endif

    /*------------------------------------------------------------*/
    /*
     * Initial variable for verifing DMA hardware
     */
#ifdef _RX_ACL_DBG_
    acl_rx_test_step_1();
#endif
#ifdef _RX_CMD_DBG_
    cmd_rx_test_step_1();
#endif
#ifdef _RX_SCO_DBG_
    sco_rx_test_step_1();
#endif
    /*------------------------------------------------------------*/

    return TRUE;

}

/**
 * Function     : hci_dma_isr_handle_cmd_tx
 *
 * Description  : This function is used to handle hci cmd tx interrupt service
 *                routine. We can follow the order fifo register to pop all 
 *                received hci command packets in sequence. It has supported
 *                loopback mode for testing.
 *
 * Parameters   : status: the isr status from hci dma hw
 *
 * Returns      : None
 */
void hci_dma_isr_handle_cmd_tx(UINT32 status)
{
    UINT8 isr_num;
    UINT8 *in_put_ptr;
    UINT16 pkt_len;
    UINT8 cmd_tx_index;
    ISR_STATUS_TYPE *pisr_status = (ISR_STATUS_TYPE *)&status;
    
    DMA_DBG_LOG(WHITE, HCI_CMD, 0, 0);
    //===================================================
    //  	Command Packet Format
    // __________ _______________________ ____________
    //|  2 bytes |       1 byte          |  Max:0xFF  |
    //|  Opcode  | Paramter Total Length |  Paramter  |
    // ---------- ----------------------- ------------
    //===================================================

    for (isr_num = pisr_status->b.cmd_num_in_tx_pkt; isr_num>0; isr_num--)
    {            
        //reset lps mode timer
#ifndef _CCH_LPS_
        g_lps_timer_counter = 0;            //clear ISR
#endif

        cmd_tx_index= (UINT8) DMA_DWORD_READ(CMD_TX_ORDER_FIFO_REG) & 0x3;
        in_put_ptr = (UINT8 *) dma_cmd_tx_des[cmd_tx_index];
        // parameter total length + 2 bytes(Opcode)
        pkt_len = (UINT16)(in_put_ptr[2] + 3);

#ifdef _YL_H5_DBG // for detailed debug message            
        DMA_UART_DBG_LOG(BLUE, DMA_UART_053, 11, 
                         pkt_len, 
                         in_put_ptr[0], in_put_ptr[1], in_put_ptr[2], in_put_ptr[3], in_put_ptr[4],
                         in_put_ptr[5], in_put_ptr[6], in_put_ptr[7], in_put_ptr[8], in_put_ptr[9]);
        DMA_UART_DBG_LOG(BLUE, DMA_UART_003_DBG_HEX, 1, DMA_DWORD_READ(  CMD_TX_BUF_FREE_RX_START_CTL_REG));                     
#endif

#ifdef _MODIFY_LOOPBACK_CODE_
        UINT16 opcode;
        opcode = (in_put_ptr[1]<<8) | in_put_ptr[0];

#ifdef _MODIFY_OLD_LOOPBACK_COMMAND_
        if (opcode == HCI_VENDOR_HCI_LOOPBACK_MODE)
        {
            enable_hci_loopback_mode = in_put_ptr[3] & 0x01;
            dma_tx_fifo_order_free(cmd_tx_index, HCI_TRANSPORT_CMD_PKT_TYPE);
            continue;
        }
#endif  
        if (enable_hci_loopback_mode) //RTK Loopback Mode
        {
            if (opcode == HCI_WRITE_LOOPBACK_MODE_OPCODE) //Normal Mode
            {
            	if (in_put_ptr[3] == 0x00)
            	{
                    RxPktFunc(HCI_TRANSPORT_CMD_PKT_TYPE, in_put_ptr, pkt_len, cmd_tx_index);
                    continue;						                    	
            	}
            }
            
            in_put_ptr++;
            in_put_ptr[0] = 0xFC;
            pkt_len -= 1;
            pf_hci_transport_write(HCI_TRANSPORT_EVENT_PKT_TYPE, in_put_ptr, pkt_len, cmd_tx_index);
        }
        else
        {
            RxPktFunc(HCI_TRANSPORT_CMD_PKT_TYPE,
                      in_put_ptr, pkt_len, cmd_tx_index); 
        }
#else

        RxPktFunc(HCI_TRANSPORT_CMD_PKT_TYPE,
                  in_put_ptr, pkt_len, cmd_tx_index);
#endif
    }

#ifdef CONFIG_TV_POWERON
    if (efuse_tv_poweron->power_on_tv_en)
    {
        if (g_host_wakeup == 0)
        {
            g_host_wakeup = 1;
            /* set event mask to default.*/
            UINT8 index;
            /* Default event_mask is 0x 00 00 1F FF FF FF FF FF */
            for (index = 0; index < 5; index++)
            {
                lmp_self_device_data.event_mask[index] = 0xFF;
            }
            lmp_self_device_data.event_mask[5] = 0x1F;
            lmp_self_device_data.event_mask[6] = 0x00;
            lmp_self_device_data.event_mask[7] = 0x00;
        }
    }
#endif
}    

/**
 * Function     : hci_dma_isr_handle_sco_tx
 *
 * Description  : This function is used to handle hci sco tx interrupt service
 *                routine. We can follow the order fifo register to pop all 
 *                received hci sco packets in sequence. It has supported
 *                loopback mode for testing.
 *
 * Parameters   : status: the isr status from hci dma hw
 *
 * Returns      : None
 */
void hci_dma_isr_handle_sco_tx(UINT32 status)
{
    UINT8 isr_num;
    UINT8 *in_put_ptr;
    UINT16 pkt_len;
    UINT8 sco_tx_index;
    ISR_STATUS_TYPE *pisr_status = (ISR_STATUS_TYPE *)&status;


#ifdef _SCO_SEND_SINE_TONE_PKT_
#ifdef _SCO_SEND_SINE_TONE_PKT_AND_COMPARE
    sco_num_in_tx_pkt = isr_status.b.sco_num_in_tx_pkt;
#endif
#endif
    for (isr_num = pisr_status->b.sco_num_in_tx_pkt; isr_num>0; isr_num--)
    {
        //reset lps mode timer
#ifndef _CCH_LPS_
        g_lps_timer_counter = 0;            //clear ISR
#endif
        DMA_DBG_LOG(YELLOW, SCO_ISR, 0, 0);                
        //===================================================
        //      Command Packet Format
        //__________ ___________ ___________________ __________ ___________
        //  1 byte  |  12 bites |       2 bites     | 2 bites  |  1 byte   |
        //  DMA HW  | Connection|                   | Reserved |   Total   |
        // reserved | handle    | Packet status flag|          |   length  |
        //---------- ----------- ------------------- ---------- -----------
        //===================================================
        //clear ISR
        sco_tx_index= DMA_DWORD_READ(SCO_TX_ORDER_FIFO_REG) & 0xF;
        
#ifdef _DAPE_TEST_CHECK_SCO_TX_INDEX
        dape_sco_tx_index = sco_tx_index;
#endif
//                sco_tx_index = sco_tx_index & 0xF;

        DMA_DBG_LOG(WHITE, TX_DES_INDEX, 1, sco_tx_index);
        in_put_ptr = (UINT8 *) dma_sco_tx_des[sco_tx_index];
        pkt_len = (UINT16)(in_put_ptr[3] + 3);//need to reduce 1 byte hw added

#ifdef _YL_H5_DBG  // for detailed debug message
            DMA_UART_DBG_LOG(BLUE, DMA_UART_052, 11, 
                                pkt_len, 
                                in_put_ptr[0], in_put_ptr[1], in_put_ptr[2], in_put_ptr[3], in_put_ptr[4],
                                in_put_ptr[5], in_put_ptr[6], in_put_ptr[7], in_put_ptr[8], in_put_ptr[9]);
            DMA_UART_DBG_LOG(BLUE, DMA_UART_003_DBG_HEX, 1, DMA_DWORD_READ(  SCO_TX_BUF_FREE_REG));                     
#endif
        
        if (enable_hci_loopback_mode)
        {
            
#ifdef _TX_CHECK_
            //Debug
            UINT32 i = 0, diff = 0, j =0;
            if ((pkt_len-3)>1)
            {
                for (i=0;i<((pkt_len-3)-1);i++)
                {
                    if ((in_put_ptr[4+i]+1) != in_put_ptr[5+i])
                    {
                        RT_BT_LOG(RED, SCO_TX_ERROR, 1, (pkt_len-3));
                        for (j=i;j<pkt_len;j++)
                            RT_BT_LOG(RED, LMP_PAYLOAD_INFO, 1, in_put_ptr[j]);
                        break;
                    }
                }
            }
#endif                
            pf_hci_transport_write(HCI_TRANSPORT_SYNC_DATA_PKT_TYPE, in_put_ptr, 
                                    pkt_len, sco_tx_index);
        }
        else
        {
            RxPktFunc(HCI_TRANSPORT_SYNC_DATA_PKT_TYPE,
                    (in_put_ptr), pkt_len, sco_tx_index);
        }
    }
}

/**
 * Function     : hci_dma_isr_handle_acl_tx
 *
 * Description  : This function is used to handle hci acl tx interrupt service
 *                routine. We can follow the order fifo register to pop all 
 *                received hci acl packets in sequence. It has supported
 *                loopback mode for testing.
 *
 * Parameters   : status: the isr status from hci dma hw
 *
 * Returns      : None
 */
void hci_dma_isr_handle_acl_tx(UINT32 status)    
{
    UINT8 isr_num;
    UINT8 *in_put_ptr;
    UINT16 pkt_len;
    UINT8 acl_tx_index;
    ACL_TX_PKT_ORDER_FIFO_TYPE      ACL_TxPkt_OrderFIFO;
    ISR_STATUS_TYPE *pisr_status = (ISR_STATUS_TYPE *)&status;

    for (isr_num = pisr_status->b.acl_num_in_tx_pkt; isr_num>0; isr_num--)
    {
        //reset lps mode timer
#ifndef _CCH_LPS_
        g_lps_timer_counter = 0;            //clear ISR
#endif

        DMA_DBG_LOG(YELLOW, ACL_ISR, 0, 0);
        //===================================================
        //      Command Packet Format
        // ___________ __________ __________ ___________
        //|  12 bites |  2 bites | 2 bites  |  2 bytes  |
        //| Connection|    PB    |    BC    |   Total   |
        //| handle    |   Flag   |   Flag   |   length  |
        // ---------- ----------- ---------- ------------
        //===================================================
        //clear ISR
       ACL_TxPkt_OrderFIFO.d32 = DMA_DWORD_READ(ACL_TX_ORDER_FIFO_REG);
       acl_tx_index = ACL_TxPkt_OrderFIFO.b.acl_tx_order_fifo_port;   
       in_put_ptr = (UINT8 *) dma_acl_tx_des[acl_tx_index];               

#ifdef _NEW_HCIDMA_DESIGN_
        if ((otp_str_data.HCI_DMA_EnhanCtrl & 0x01))
        {
//         RT_BT_LOG(YELLOW, MSG_ACL_TX_PKT_ORDER_FIFO, 1, acl_tx_index);
           if (ACL_TxPkt_OrderFIFO.b.USB_ACL_HdrLen_Shorter_F || 
               ACL_TxPkt_OrderFIFO.b.USB_ACL_HdrLen_Longer_F)
           {
               //UINT16                          acl_connectionhdl;
                //acl_connectionhdl = (UINT16)((in_put_ptr[1]<<8) + in_put_ptr[0]);     
                //if (ACL_TxPkt_OrderFIFO.b.USB_ACL_HdrLen_Shorter_F == 1)
                //{
                    //DMA_DBG_LOG(YELLOW, MSG_ACL_HDR_LEN_SHORTER, 2, acl_connectionhdl, acl_tx_index);
                //    RT_BT_LOG(YELLOW, MSG_ACL_HDR_LEN_SHORTER, 2, acl_connectionhdl, acl_tx_index);
                //}

                //if (ACL_TxPkt_OrderFIFO.b.USB_ACL_HdrLen_Longer_F == 1)
                //{
                    //DMA_DBG_LOG(YELLOW, MSG_ACL_HDR_LEN_LONGER, 2, acl_connectionhdl, acl_tx_index);
                //    RT_BT_LOG(YELLOW, MSG_ACL_HDR_LEN_LONGER, 2, acl_connectionhdl, acl_tx_index);
                //}

               RT_BT_LOG(YELLOW, DMA_UART_074, 2, status, ACL_TxPkt_OrderFIFO.d32);

                if ((otp_str_data.HCI_DMA_EnhanCtrl & 0x02))
                {
                    dma_tx_fifo_order_free(acl_tx_index, HCI_TRANSPORT_ACL_DATA_PKT_TYPE);                    
                    hci_generate_hw_error_event(HARDWARE_FAILURE_ERROR);
                    break;
                }
            }                
        }           
#endif

        /* bind the ws in the dmem to the HCI ACL Data Pkt Body */
        HCI_ACL_DATA_PKT *acl_pkt = (HCI_ACL_DATA_PKT *)in_put_ptr;
        acl_pkt->ws = &acl_h2c_data_ws[acl_tx_index];

        // parameter total length + 2 bytes(Opcode)
        pkt_len = (UINT16)((in_put_ptr[3]<<8) + in_put_ptr[2] + 4);

#if 0
        payload_len = pkt_len - 0x4;

   RT_BT_LOG(YELLOW, MSG_ACL_TXADDR_INPTR, 1, in_put_ptr);
   RT_BT_LOG(YELLOW, MSG_ACL_PKT_LEN, 1, payload_len);
   RT_BT_LOG(BLUE, DMA_UART_051, 11, 
                                pkt_len, 
                                in_put_ptr[0], in_put_ptr[1], in_put_ptr[2], in_put_ptr[3], in_put_ptr[4],
                                in_put_ptr[5], in_put_ptr[6], in_put_ptr[7], in_put_ptr[8], in_put_ptr[9]);
#endif

#ifdef _YL_H5_DBG // for detailed debug message
            DMA_UART_DBG_LOG(BLUE, DMA_UART_051, 11, 
                                pkt_len, 
                                in_put_ptr[0], in_put_ptr[1], in_put_ptr[2], in_put_ptr[3], in_put_ptr[4],
                                in_put_ptr[5], in_put_ptr[6], in_put_ptr[7], in_put_ptr[8], in_put_ptr[9]);
            DMA_UART_DBG_LOG(BLUE, DMA_UART_003_DBG_HEX, 1, DMA_DWORD_READ(ACL_TX_BUF_FREE_REG));                     
#endif
#ifdef _SECURE_CONN_TEST_CHK_ACLU_DATA_BY_VENDOR_CMD
        if (g_chk_secure_conn_data)
        {
            RT_BT_LOG(GREEN, DAPE_TEST_LOG557, 11,
            pkt_len-4, in_put_ptr[0], in_put_ptr[1],in_put_ptr[2],in_put_ptr[3],
            in_put_ptr[4], in_put_ptr[5],in_put_ptr[6],in_put_ptr[7],in_put_ptr[8], in_put_ptr[9]);
        }
#endif
        if (enable_hci_loopback_mode)
        {
#ifdef _TX_CHECK_
            //Debug
            UINT32 i = 0, diff = 0, j =0;
            if ((pkt_len-4)>1)
            {
                for (i=0;i<((pkt_len-4)-1);i++)
                {
                    if ((in_put_ptr[4+i]+1) != in_put_ptr[5+i])
                    {
                        RT_BT_LOG(RED, ACL_TX_ERROR, 1, (pkt_len-4));
                        for (j=i;j<pkt_len;j++)
                            RT_BT_LOG(RED, LMP_PAYLOAD_INFO, 1, in_put_ptr[j]);
                        break;
                    }
                }
            }
#endif 

            pf_hci_transport_write(HCI_TRANSPORT_ACL_DATA_PKT_TYPE, in_put_ptr, pkt_len, acl_tx_index);   
        }
        else
        {
            //DMA_DBG_LOG(WHITE, TX_PKT_LEN, 1, pkt_len);
            RxPktFunc(HCI_TRANSPORT_ACL_DATA_PKT_TYPE,
                    in_put_ptr, pkt_len, acl_tx_index);
        }
    }
}

#ifdef LE_MODE_EN
/**
 * Function     : hci_dma_isr_handle_le_acl_tx
 *
 * Description  : This function is used to handle hci le acl tx interrupt 
 *                service routine. We can follow the order fifo register to 
 *                pop all  received hci le acl packets in sequence.
 *
 * Parameters   : status: the isr status from hci dma hw
 *
 * Returns      : None
 */
void hci_dma_isr_handle_le_acl_tx(UINT32 status)     
{
    UINT8 isr_num;
    UINT8 *in_put_ptr;
    UINT16 pkt_len;
    UINT8 le_acl_tx_index;
    LE_ACL_TX_PKT_ORDER_FIFO_TYPE	LE_ACL_TxPkt_OrderFIFO;
    ISR_STATUS_TYPE *pisr_status = (ISR_STATUS_TYPE *)&status;

    for (isr_num = pisr_status->b.le_acl_num_in_tx_pkt; isr_num>0; isr_num--)
    {
        //reset lps mode timer
#ifndef _CCH_LPS_
        g_lps_timer_counter = 0;            //clear ISR
#endif

        DMA_DBG_LOG(YELLOW, LE_ACL_ISR, 0, 0);
        //clear ISR

        LE_ACL_TxPkt_OrderFIFO.d32 = DMA_DWORD_READ(LE_ACLT_TX_ORDER_FIFO_REG);
        le_acl_tx_index = LE_ACL_TxPkt_OrderFIFO.b.LE_tx_order_fifo_port;
        in_put_ptr = (UINT8 *) dma_le_acl_tx_des[le_acl_tx_index]; 

#ifdef _NEW_HCIDMA_DESIGN_
        if ((otp_str_data.HCI_DMA_EnhanCtrl & 0x01))
        {        
            DMA_DBG_LOG(YELLOW, MSG_LE_TX_PKT_ORDER_FIFO, 1, le_acl_tx_index);

            if (LE_ACL_TxPkt_OrderFIFO.b.USB_LE_HdrLen_Shorter_F || 
                LE_ACL_TxPkt_OrderFIFO.b.USB_LE_HdrLen_Longer_F)
            {
#if 0
                UINT16                          le_connectionhdl;   
                le_connectionhdl = (UINT16)((in_put_ptr[1]<<8) + in_put_ptr[0]);                            
                if (ACL_TxPkt_OrderFIFO.b.USB_ACL_HdrLen_Shorter_F)
                    DMA_DBG_LOG(YELLOW, MSG_LE_HDR_LEN_SHORTER, 2, le_connectionhdl, le_acl_tx_index);

                if (ACL_TxPkt_OrderFIFO.b.USB_ACL_HdrLen_Longer_F)
                    DMA_DBG_LOG(YELLOW, MSG_LE_HDR_LEN_LONGER, 2, le_connectionhdl, le_acl_tx_index);
#endif
                RT_BT_LOG(YELLOW, DMA_UART_074, 2, status, LE_ACL_TxPkt_OrderFIFO.d32);

                if ((otp_str_data.HCI_DMA_EnhanCtrl & 0x02))
                {
                    dma_tx_fifo_order_free(le_acl_tx_index, HCI_TRANSPORT_LE_ACL_DATA_PKT_TYPE);
                    hci_generate_hw_error_event(HARDWARE_FAILURE_ERROR);
                    break;
                }
            }
        }
#endif

        // parameter total length + 2 bytes(Opcode)
        pkt_len = (UINT16)((in_put_ptr[3]<<8) + in_put_ptr[2] + 4);
#ifdef _UPF_DBG
        RT_BT_LOG(BLUE,YL_DBG_HEX_15,15,5566,pkt_len,le_acl_tx_index,
            in_put_ptr[0],in_put_ptr[1],in_put_ptr[2],in_put_ptr[3],in_put_ptr[4],in_put_ptr[5],in_put_ptr[6],
            in_put_ptr[7],in_put_ptr[8],in_put_ptr[9],in_put_ptr[10],in_put_ptr[11]);
#endif
        //DMA_DBG_LOG(WHITE, TX_PKT_LEN, 1, pkt_len);
        RxPktFunc(HCI_TRANSPORT_LE_ACL_DATA_PKT_TYPE,
                  in_put_ptr, pkt_len, le_acl_tx_index);

    }
}
#endif

/**
 * Function     : hci_dma_isr_handle_event_rx
 *
 * Description  : This function is used to handle hci event rx interrupt 
 *                service routine. We can follow the rx descriptor ring to 
 *                capture the completion indication of sent event packets. 
 *                Then we can start to do package free procedure.
 *                It has supported loopback mode for testing.
 *
 * Parameters   : status: the isr status from hci dma hw
 *
 * Returns      : None
 */
void hci_dma_isr_handle_event_rx(UINT32 status)  
{
    UINT8 i;
    UINT32 temp_addr;

    DMA_DBG_LOG(RED, EVT_SEND_FINISH, 0, 0);
#ifdef _YL_H5_DBG // more detailed debug message
    DMA_UART_DBG_LOG(RED, EVT_SEND_FINISH, 0, 0);
#endif        
    DMA_DWORD_WRITE(ISR_STATUS_REG, CLEAN_EVT_RXDONE);

    for (i=0; i<(EVT_RX_DES_NUM-1); i++ )            
    {
        if (dma_man.evt_available_num == (EVT_RX_DES_NUM-1))
        {
            break;
        }
        
        if (dma_evt_rx_des[dma_man.evt_rx_r_ind].rx_des_dw1.own != 0)
        {
            //if (i == 0)
            //    RT_BT_LOG(RED, EVT_RX_OWN_ERROR, 1, 0);
            break;
        }
        
        if (enable_hci_loopback_mode)
        {
            UINT8 bitmap = 0;
            bitmap = cmd_fifo_index[dma_man.evt_rx_r_ind];
            dma_tx_fifo_order_free(bitmap, HCI_TRANSPORT_CMD_PKT_TYPE);
        }
        else
        {
            temp_addr = dma_man.dma_evt_rx_free_addr[dma_man.evt_rx_r_ind];
            DMA_DBG_LOG(BLUE, RX_FREE_ADDR, 1, temp_addr);
            DMA_DBG_LOG(BLUE, EVT_RX_FREE_FIFO_INDEX, 1, dma_man.evt_rx_r_ind);
            TxCompFunc((UCHAR *)temp_addr, 0, HCI_TRANSPORT_EVENT_PKT_TYPE);
        }

        dma_man.evt_rx_r_ind = (dma_man.evt_rx_r_ind+1) & (EVT_RX_DES_NUM-2);
        dma_man.evt_available_num++;                
    }

#ifdef _SUPPORT_NEW_USB_LPM_L1_DATA_PATH_150629_
    if (IS_SUPPORT_USB_LPM_L1_FSM && 
                (g_fun_interface_info.b.bt_interface == USB_INTERFACE)) 
    {
        hci_dma_usb_check_and_allow_enter_usb_lpm_l1();
    }
#endif /* end of #ifdef _SUPPORT_NEW_USB_LPM_L1_DATA_PATH_150629_ */   
}

/**
 * Function     : hci_dma_isr_handle_acl_rx
 *
 * Description  : This function is used to handle hci acl rx interrupt 
 *                service routine. We can follow the rx descriptor ring to 
 *                capture the completion indication of sent acl packets. 
 *                Then we can start to do package free procedure.
 *                It has supported loopback mode for testing.
 *
 * Parameters   : status: the isr status from hci dma hw
 *
 * Returns      : None
 */
void hci_dma_isr_handle_acl_rx(UINT32 status)  
{
    UINT8 i;
    UINT32 temp_addr;
    HCI_TRANSPORT_PKT_TYPE type;

    DMA_DBG_LOG(RED, ACL_SEND_FINISH, 0, 0);
#ifdef _YL_H5_DBG // more detailed debug message
    DMA_UART_DBG_LOG(RED, ACL_SEND_FINISH, 0, 0);
#endif
    DMA_DWORD_WRITE(ISR_STATUS_REG, CLEAN_ACL_RXDONE);

    for (i=0; i<ACL_RX_DES_NUM; i++)
    {
        if (dma_man.acl_available_num == (ACL_RX_DES_NUM-1))
        {
            break;
        }
        if (dma_acl_rx_des[dma_man.acl_rx_r_ind].rx_des_dw1.own != 0)
        {
            //if (i == 0)
            //    RT_BT_LOG(RED, EVT_RX_OWN_ERROR, 1, 0);
            break;
        }

        if (enable_hci_loopback_mode)
        {
            UINT8 bitmap = 0;
            bitmap = acl_fifo_index[dma_man.acl_rx_r_ind];
            dma_tx_fifo_order_free(bitmap, HCI_TRANSPORT_ACL_DATA_PKT_TYPE);           
        }
        else
        {
            temp_addr = dma_man.dma_acl_rx_free_addr[dma_man.acl_rx_r_ind];
            DMA_DBG_LOG(BLUE, RX_FREE_ADDR, 1, temp_addr);
            DMA_DBG_LOG(BLUE, EVT_RX_FREE_FIFO_INDEX, 1, dma_man.acl_rx_r_ind);
            type = HCI_TRANSPORT_ACL_DATA_PKT_TYPE;
#ifdef LE_MODE_EN
            if ((temp_addr & 0xF000000) == 0x6000000)
            {
                DMA_DBG_LOG(BLUE, RX_LE_ACL_ISR, 0, 0);
                temp_addr &= ~0x6000000;
                type = HCI_TRANSPORT_LE_ACL_DATA_PKT_TYPE;
            }
#endif
            TxCompFunc((UCHAR *)temp_addr, 0, type);
        }
        dma_man.acl_rx_r_ind = (dma_man.acl_rx_r_ind+1) & (ACL_RX_DES_NUM-2);
        dma_man.acl_available_num++;
    }

#ifdef _SUPPORT_NEW_USB_LPM_L1_DATA_PATH_150629_
    if (IS_SUPPORT_USB_LPM_L1_FSM && 
                (g_fun_interface_info.b.bt_interface == USB_INTERFACE)) 
    {
        hci_dma_usb_check_and_allow_enter_usb_lpm_l1();
    }
#endif /* end of #ifdef _SUPPORT_NEW_USB_LPM_L1_DATA_PATH_150629_ */
}

/**
 * Function     : hci_dma_isr_handle_sco_rx
 *
 * Description  : This function is used to handle hci sco rx interrupt 
 *                service routine. We can follow the rx descriptor ring to 
 *                capture the completion indication of sent sco packets. 
 *                Then we can start to do package free procedure.
 *                It has supported loopback mode for testing.
 *
 * Parameters   : status: the isr status from hci dma hw
 *
 * Returns      : None
 */
void hci_dma_isr_handle_sco_rx(UINT32 status)  
{
    UINT8 i;
    UINT32 temp_addr;

    first_iso_in = 1;
    DMA_DBG_LOG(RED, SCO_SEND_FINISH, 0, 0);
#ifdef _YL_H5_DBG // more detailed debug message
    DMA_UART_DBG_LOG(RED, SCO_SEND_FINISH, 0, 0);
#endif        
    DMA_DWORD_WRITE(ISR_STATUS_REG, CLEAN_SCO_RXDONE);

    for (i=0; i<SCO_RX_DES_NUM; i++)
    {
        if (dma_man.sco_available_num == (SCO_RX_DES_NUM-1))
        {
            break;
        }
        if (dma_sco_rx_des[dma_man.sco_rx_r_ind].rx_des_dw1.own != 0)
        {
//                if (i == 0)
//                    RT_BT_LOG(RED, EVT_RX_OWN_ERROR, 1, i);
            break;
        }

        if (enable_hci_loopback_mode)
        {
            UINT8 bitmap = 0;
            bitmap = sco_fifo_index[dma_man.sco_rx_r_ind];
            dma_tx_fifo_order_free(bitmap, HCI_TRANSPORT_SYNC_DATA_PKT_TYPE);
     
        }
        else
        {
            temp_addr = dma_man.dma_sco_rx_free_addr[dma_man.sco_rx_r_ind];
            dma_man.dma_sco_rx_free_addr[dma_man.sco_rx_r_ind] = 0;
            DMA_DBG_LOG(BLUE, RX_FREE_ADDR, 1, temp_addr);
            DMA_DBG_LOG(BLUE, EVT_RX_FREE_FIFO_INDEX, 1, dma_man.sco_rx_r_ind);
            TxCompFunc((UCHAR *)temp_addr, 0, HCI_TRANSPORT_SYNC_DATA_PKT_TYPE);
        }
        dma_man.sco_rx_r_ind = (dma_man.sco_rx_r_ind+1) & (SCO_RX_DES_NUM-2);
        dma_man.sco_available_num++;                   
    }
}

/**
 * Function     : hci_dma_isr_handle_iso_inf_alt_change
 *
 * Description  : This function is used to handle the interrupt service routine
 *                about the usb alternate setting change indication of usb 
 *                isochronous interface. We need to do some synchronization 
 *                procedure for fw and hw here. And we can decide the actual 
 *                sco packet size via this indication.
 *
 * Parameters   : status: the isr status from hci dma hw
 *
 * Returns      : None
 */
void hci_dma_isr_handle_iso_inf_alt_change(UINT32 status)  
{
    SCO_CON_HANDLE_1_AND_2_TYPE  con_handle_1_and_2_reg;
    UINT8 usb_iso_pkt_size;
    UINT8 i;

    // TODO: Need to modify SCO function
    con_handle_1_and_2_reg.d32 = DMA_DWORD_READ(SCO_CONNECTION_HANDLE_1_AND_2_REG);
    DMA_DWORD_WRITE(ISR_STATUS_REG,CLEAN_ALTERNATING_SETTING);
    usb_iso_pkt_size = sco_pkt_size[con_handle_1_and_2_reg.b.iso_al_setting];

    /* record the alternate setting info to help pkt handling - austin*/
    if (g_usb_iso_alternate_value != con_handle_1_and_2_reg.b.iso_al_setting)
    {
        for (i = 0; i < ISOCH_SCO_MAX_CONNS; i++)
        {
            g_usb_iso_alternate_change[i] = TRUE;
        }
        g_usb_iso_alternate_value = con_handle_1_and_2_reg.b.iso_al_setting;
#ifdef _DAPE_TEST_FIX_DMA_HANG
        
//=================== DMA RX own bit clear ===========================================
                    
        SCO_TX_BUF_SIZE_TYPE        sco_tx_buf_size_reg;
        sco_tx_buf_size_reg.d32 = DMA_DWORD_READ(SCO_TX_BUF_SIZE_REG);
        sco_tx_buf_size_reg.b.sco_tx_pkt_num = 0;
        DMA_DWORD_WRITE(SCO_TX_BUF_SIZE_REG, sco_tx_buf_size_reg.d32);
        
        //set flag BB need to clear SCO TX 
        alternatset_change_tx_clear = 1;
        alternatset_change_rx_clear = 1;
                    
        for (i = 0; i < SCO_RX_DES_NUM; i++)
        {
            dma_sco_rx_des[i].rx_des_dw1.own = 0;
        }

        //Free RX buffer
        if (!enable_hci_loopback_mode)
        {
            for (i = 0; i < (SCO_RX_DES_NUM-1); i++)
            {
                if (dma_man.dma_sco_rx_free_addr[i] != 0)
                {
                    if (OS_FREE_BUFFER(synchronous_data_to_host_pool_id, 
                          (void*)dma_man.dma_sco_rx_free_addr[i]) != BT_ERROR_OK)
                    {
                        RT_BT_LOG(GRAY, HCI_TD_893, 0, 0);
                    }
                    dma_man.dma_sco_rx_free_addr[i] = 0;
                }               
            }
        }
        dma_man.sco_rx_w_ind = 0;
        dma_man.sco_rx_r_ind = 0;            
        dma_man.sco_available_num = (SCO_RX_DES_NUM - 1);
       
        //When fw receive alternetseting isr, hw will stop operation.
        // if hw wants to start operation again, fw need to initial rx register again.
        //Initial SCO RX
        DMA_ADDR_REG_TYPE           gen_dma_address_reg;
        gen_dma_address_reg.d32 = 0;
        gen_dma_address_reg.b.addr = 0xFFFF & (UINT32) dma_sco_rx_des;
        DMA_DWORD_WRITE(SCO_RX_DES_RING_BASE_REG, gen_dma_address_reg.d32);
        if (enable_hci_loopback_mode)
        {
            alternatset_change_rx_clear = 0;
            clear_sco_buffer();
        }
                    
//====================================================================================
#endif			

    }
    else
    {
        for (i = 0; i < ISOCH_SCO_MAX_CONNS; i++)
        {
            g_usb_iso_alternate_change[i] = FALSE;
        }
    }
    
    if (con_handle_1_and_2_reg.b.iso_al_setting != 0)
    {
        if (bz_sco_pkt_size != usb_iso_pkt_size)
        {
            bz_sco_pkt_size = usb_iso_pkt_size;
            //sco_tx_data.buf_size = (bz_sco_pkt_size+3)*(SCO_TX_BUF_SIZE/(bz_sco_pkt_size+3));
        }
        bz_isoch_is_usb_ready = TRUE;
        //RT_BT_LOG(GRAY, USB_1574, 1, sco_tx_data.buf_size);
    }
    else
    {
        bz_isoch_is_usb_ready = FALSE;
    }
    
    RT_BT_LOG(RED, ALTERNATING_SETTING_NUMBER, 1, con_handle_1_and_2_reg.b.iso_al_setting);
    //RT_BT_LOG(GREEN, USB_1583, 1, bz_sco_pkt_size);
}


/**
 * Function     : hci_dma_isr_handle_32k_clk_wakeup
 *
 * Description  : This function is used to handle the interrupt service routine
 *                about the wakeup event from 32K clock.
 *
 * Parameters   : status: the isr status from hci dma hw
 *
 * Returns      : None
 */
void hci_dma_isr_handle_32k_clk_wakeup(UINT32 status)  
{
    //sys_clk_reg = sys_clk_reg & 0x600;
    //VENDOR_WRITE(0,sys_clk_reg);
    DMA_DWORD_WRITE(ISR_STATUS_REG,CLEAN_WAKE_UP_ISR);

    if (wakeup_isr_en)
    {
        switch_hci_dma_setting(LEAVE_SUSPEND_MODE);
        wake_up_interrupt_switch(DISABLE_WAKE_UP_ISR);
        RT_BT_LOG(YELLOW,LEAVE_32_K_MODE,0,0);            
    }
}

void hci_dma_isr_handle_usb_token_timeout(UINT32 status)
{
#if defined(_SUPPORT_USB_TIMEOUT_INTERRUPT_)&&defined(_SUPPORT_FW_INDIRECT_READ_SIE_)
    ISR_STATUS_TYPE isr_status;
    UINT8 u8ep_sts = 0;
    
    isr_status.d32= DMA_DWORD_READ(ISR_STATUS_REG);
    RT_BT_LOG(YELLOW,FONGPIN_USB_TIMEOUT_OCCUR,2,DMA_DWORD_READ(ISR_ENABLE_REG), isr_status.d32);
   
    u8ep_sts = safe_indirect_read_sie(READ_SIE_BYTE, SIE_BUFF_OK_REG);
    if(isr_status.b.iso_out_timeout_int_sts)
    {
        //RT_BT_LOG(YELLOW, FONGPIN_ISO_OUT_TOKEN_TO, 0,0);
        DMA_DWORD_WRITE(ISR_STATUS_REG,0x80000000);// clear interrupt
    }
    
    if(isr_status.b.iso_in_timeout_int_sts)
    {
#if defined (_SUPPORT_INFO_FROM_SYSTEM_ON_) && !defined(_BT_ONLY_)
        if (g_chip_id == CHIP_ID_8822B)
        {
            UINT16 u16reqlen = safe_indirect_read_sie(READ_SIE_WORD, RX_SCO_REQ_LEN_REG);
            UINT16 u16buf_addr = safe_indirect_read_sie(READ_SIE_WORD, RX_SCO_BUF_W_ADDR_LOW_REG);
            RT_BT_LOG(YELLOW, FONGPIN_ISO_IN_TOKEN_TO_22B, 3,u16reqlen,u8ep_sts,(u16buf_addr&0x1FF));
        }
        //else
        //{
            //RT_BT_LOG(YELLOW, FONGPIN_ISO_IN_TOKEN_TO, 0,0);
        //}
#endif
        
        DMA_DWORD_WRITE(ISR_STATUS_REG,0x40000000);// clear interrupt
    }
    
    if(isr_status.b.evt_timeout_int_sts)
    {
       
        UINT16 u16reqlen = safe_indirect_read_sie(READ_SIE_WORD, RX_EVT_REQ_LEN_REG);
        UINT8 u8buf_addr = safe_indirect_read_sie(READ_SIE_BYTE, RX_EVT_BUF_W_ADDR_REG);
        RT_BT_LOG(YELLOW, FONGPIN_EVT_IN_TOKEN_TO, 3,u16reqlen,u8ep_sts,u8buf_addr);
        
        DMA_DWORD_WRITE(ISR_STATUS_REG,0x04000000);// clear interrupt
    }

    EFUSE_SCOREBOARD_AND_USB_TO_INIT_S efuse_usb_token_timeout;
    *(UINT8*)&efuse_usb_token_timeout = otp_str_data.efuse_misc_interrupt_d8;


    UINT8 u8Value = DMA_BYTE_READ(USB_TOKEN_TIMEOUT_REG);
    switch (efuse_usb_token_timeout.usb_timeout_fun_policy_sel)
    {
       case USB_TO_DONOTHOIN:
            // do nothing
       break;
       case USB_TO_DIS:
            DMA_BYTE_WRITE(USB_TOKEN_TIMEOUT_REG,u8Value&0xFE);// disbale timeout function 
       break;
       case USB_TO_DISEN:
            DMA_BYTE_WRITE(USB_TOKEN_TIMEOUT_REG,u8Value&0xFE);// disbale timeout function
            DMA_BYTE_WRITE(USB_TOKEN_TIMEOUT_REG,u8Value|BIT0);// enable timeout function             
       break;
       case USB_TO_TIMER:
            if(g_u16usb_token_timeout_timer_cnt == 128)// 1sec
            {
                RT_BT_LOG(RED, YL_DBG_HEX_2, 2,g_u16usb_token_timeout_timer_cnt, u8Value&0xFE);
                DMA_BYTE_WRITE(USB_TOKEN_TIMEOUT_REG,u8Value&0xFE);// disbale timeout function
            }
       break;
       default:
            // error case
       break;
    }

#endif    
}

void hci_dma_isr_handle_invalid(UINT32 status)
{
    return;
}

/*********************************************************/
/* The function table of USB DMA ontrol Command Handling */
/*********************************************************/
void (*(hci_dma_isr_sub_handler[])) (UINT32) =
{
    hci_dma_isr_handle_cmd_tx,
    hci_dma_isr_handle_sco_tx,
    hci_dma_isr_handle_acl_tx,
#ifdef LE_MODE_EN
    hci_dma_isr_handle_le_acl_tx, 
#endif
    hci_dma_isr_handle_event_rx,
    hci_dma_isr_handle_acl_rx,
    hci_dma_isr_handle_sco_rx,  
    hci_dma_isr_handle_iso_inf_alt_change,
#ifdef _ENABLE_32K_CLK_WAKE_UP_ISR_    
    hci_dma_isr_handle_32k_clk_wakeup,
#endif    
#ifdef _SUPPORT_USB_TIMEOUT_INTERRUPT_
    hci_dma_isr_handle_usb_token_timeout,
#endif
    hci_dma_isr_handle_invalid
};

/**
 * Function     : USB_DMA_IntrHandler
 *
 * Description  : This function handle USB DMA event.
 *                1) SCO RX Interrupt
 *                2) ACL RX Interrupt
 *				  3) EVT RX Interrupt
 *                4) SCO TX Interrupt
 *                5) ACL TX Interrupt
 *				  6) EVT TX Interrupt
 *                =============== Clear TX ISR Note ==============================
 *                 example:
 *                       If there are 4 packet in fifo, firmware needs to read
 *                       CMD_TX_ORDER_FIFO_REG 4 times. After that, hardware
 *                       will clear isr bit automatically.
 *                       When firmware was in ISR state and another isr event
 *                       happened at the same time, firmware still need to read
 *                       more one time. Then hardware can clear ISR bit.
 *                =================================================================
 *
 * Parameters   : None.
 *
 * Returns      : None
 *
 * Side Effects : None
 */
SECTION_ISR_LOW void USB_DMA_IntrHandler(void)
{
    ISR_STATUS_TYPE isr_status;
    UINT32 isr_mask = 0xBF;

#ifdef _YL_H5_DBG // for detailed debug message
    UINT32 cmd_free, acl_free, sco_free;
    cmd_free = DMA_DWORD_READ(  CMD_TX_BUF_FREE_RX_START_CTL_REG);
    acl_free = DMA_DWORD_READ(  ACL_TX_BUF_FREE_REG);
    sco_free = DMA_DWORD_READ(  SCO_TX_BUF_FREE_REG);
    if(cmd_free && acl_free && sco_free )
        DMA_UART_DBG_LOG(WHITE,DMA_UART_072,3, cmd_free, acl_free, sco_free);
    else
        DMA_UART_DBG_LOG(RED,DMA_UART_072,3, cmd_free, acl_free, sco_free);
//    DMA_DBG_LOG(YELLOW, DMA_UART_074, 1, isr_status.d32);            
#endif	
     
    isr_status.d32= DMA_DWORD_READ(ISR_STATUS_REG);  

#ifdef _ROM_CODE_PATCHED_
    /* reserved rom code patch by yilinli, for
      * e.g.
      *    1. may block interrupts when sleep_mode_param.bb_sm_sts != NORMAL
      *    2. etc.
      */
    if (rcp_usb_dma_intrhandler_func != NULL)
    {
        if(rcp_usb_dma_intrhandler_func((void*)&isr_status.d32))
        {
            return;
        }
    }    
#endif

//    DMA_DBG_LOG(YELLOW, CMD_ISR, 1, isr_status.d32);    
    DMA_DBG_LOG(YELLOW, DMA_UART_074, 2, isr_status.d32, isr_status.d32);

#ifdef LE_MODE_EN
    if (IS_BT40)
    {
        isr_mask = 0xFF;
    }
#endif

#ifdef _SUPPORT_USB_TIMEOUT_INTERRUPT_
    isr_mask |= BIT26 | BIT30 | BIT31;
#endif

    while (1) 
    {
#ifdef _CCH_LPS_
#ifdef _ROM_CODE_PATCHED_
#ifdef _CCH_8723_RCP_
        if (rcp_USB_DMA_IntrHandler_lps_reset != NULL)
        {
            rcp_USB_DMA_IntrHandler_lps_reset((void*)(&isr_status.d32), isr_mask);
        }
        else
#endif
#endif
        {
            if (isr_status.d32 & isr_mask)
            {
                g_lps_timer_counter = g_timer2_init_value;
            } 
        }     
#endif

#ifdef _NEW_HCIDMA_DESIGN_
        if ((otp_str_data.HCI_DMA_EnhanCtrl & 0x08))
        {
            if (isr_status.b.USB_CMD_Tx_PktErr_int_sts)
            {
                DMA_DWORD_WRITE(ISR_STATUS_REG, 0x2000);

                RT_BT_LOG(YELLOW, INTR_CMD_PKT_ERR, 1, isr_status.d32);

                if ((otp_str_data.HCI_DMA_EnhanCtrl & 0x10))
                {
            	    hci_generate_hw_error_event(HARDWARE_FAILURE_ERROR);        	   	     	     
                    return;       
                }                
            }            
        }

        if ((otp_str_data.HCI_DMA_EnhanCtrl & 0x04))
        {
            if (isr_status.b.USB_ACL_Tx_LongPktErr_int_sts || 
                isr_status.b.USB_LE_Tx_LongPktErr_int_sts)
            {
                UINT32 tmpData;

                tmpData = DMA_DWORD_READ(USB_ACL_TX_LENERR_EARLY_PKTINDEX_REG);
                
                if (isr_status.b.USB_ACL_Tx_LongPktErr_int_sts)
                {
                    //RT_BT_LOG(YELLOW, INTR_ACL_HDR_LEN_LONGER, 1, tmpData);
                    DMA_DWORD_WRITE(ISR_STATUS_REG, 0x400);
                }

                if (isr_status.b.USB_LE_Tx_LongPktErr_int_sts)
                {
                    //RT_BT_LOG(YELLOW, INTR_LE_HDR_LEN_LONGER, 1, tmpData);
                    DMA_DWORD_WRITE(ISR_STATUS_REG, 0x800);
                }

                RT_BT_LOG(YELLOW, DMA_UART_074, 2, isr_status.d32, tmpData);
        		
            	if ((otp_str_data.HCI_DMA_EnhanCtrl & 0x02))
            	{
        	        hci_generate_hw_error_event(HARDWARE_FAILURE_ERROR);        	   	     	     
                    return;       
                }
            }	
        }
#endif

        //2 Handle CMD TX
        if (isr_status.b.cmd_num_in_tx_pkt!= 0)
        {
            (*(hci_dma_isr_sub_handler[HCID_ISR_OP_CMD_TX]))(isr_status.d32);
        }
        
        //2 Handle SCO TX
        if (isr_status.b.sco_num_in_tx_pkt!= 0)
        {          
            (*(hci_dma_isr_sub_handler[HCID_ISR_OP_SCO_TX]))(isr_status.d32);

        }
        
        //2 Handle ACL TX
        if (isr_status.b.acl_num_in_tx_pkt!= 0)
        {                  
            (*(hci_dma_isr_sub_handler[HCID_ISR_OP_ACL_TX]))(isr_status.d32);

        }

#ifdef LE_MODE_EN
        //2 Handle LE ACL TX
        if (isr_status.b.le_acl_num_in_tx_pkt!= 0)
        {
            (*(hci_dma_isr_sub_handler[HCID_ISR_OP_LE_ACL_TX]))(isr_status.d32);

        }
#endif
        //2 Handle EVT RX
        if (isr_status.b.evt_rx_int_sts)
        {
            (*(hci_dma_isr_sub_handler[HCID_ISR_OP_EVENT_RX]))(isr_status.d32);
        }

        //2 Handle ACL RX
        if (isr_status.b.acl_rx_int_sts)
        {
            (*(hci_dma_isr_sub_handler[HCID_ISR_OP_ACL_RX]))(isr_status.d32);
        }

        //2 Handle SCO RX
        if (isr_status.b.sco_rx_int_sts)
        {
            (*(hci_dma_isr_sub_handler[HCID_ISR_OP_SCO_RX]))(isr_status.d32);
        }

        //2 Handle USB Alternating Setting Number
        if (isr_status.b.iso_alt_ch_int_sts)
        {
            (*(hci_dma_isr_sub_handler[HCID_ISR_OP_ISO_ALT_CHG]))(isr_status.d32);

        }

#ifdef _ENABLE_32K_CLK_WAKE_UP_ISR_
        //2 Handle 32K clock wake up interupt
        if (isr_status.b.wake_up_int_sts)
        {
            (*(hci_dma_isr_sub_handler[HCID_ISR_OP_32K_CLK_WKUP]))(isr_status.d32);
        }
#endif

#ifdef _SUPPORT_USB_TIMEOUT_INTERRUPT_
        //2 Handle usb token timeout
        if ((isr_status.b.iso_out_timeout_int_sts) ||
            (isr_status.b.iso_in_timeout_int_sts) ||
            (isr_status.b.evt_timeout_int_sts))
        {
            (*(hci_dma_isr_sub_handler[HCID_ISR_OP_TIMEOUT_HANDLE]))(isr_status.d32);
        }
#endif

#ifdef _ENABLE_UART_DMA_ERROR_REPORT_
        //2 Handle DMA Uart error status
        if (g_fun_interface_info.b.bt_interface == UART_INTERFACE)
        {
            UINT32 uart_reg_status = 0;

            // TODO: To Confirm the logic        
            if (isr_status.d32 == 0)
            {
                /* YL20110328, modify the procedure for UART Error Recovery */
                uart_reg_status = UART_DWORD_READ(DMA_UART_STS_REG);
                if (uart_reg_status & BIT0)
                {
#ifdef _ROM_CODE_PATCHED_
#ifdef _YL_RTL8723A_B_CUT
                    /* Reserved com code patch by yilinli:
                      *    1. clear g_lps_timer_counter 
                      */    
#endif                      
                    if (rcp_hci_dma_intr_h4_error_func != NULL)
                    {
                        if (rcp_hci_dma_intr_h4_error_func((void*)(&uart_reg_status)))
                        {
                            return; /* if return TRUE: force stop DMA loop */
                        }
                    }
#endif                                        
                
    //                RT_BT_LOG(RED, ISR_INDEX_CHECK, 1, BIT0);
                    RT_BT_LOG(RED, DMA_UART_040, 1, uart_reg_status);

#ifndef _RTK8723_UART_INIT_
    //                uart_reg_status = uart_reg_status | BIT0;
                    UART_DWORD_WRITE(DMA_UART_STS_REG,uart_reg_status);
#else
                    /* YL20110328, write 1 to clear error interrupt flag */

#ifdef _UART_H5 /* for H4 error w1c after event RX */
                    if (g_data_uart_settings_3.err_send_event_delayw1c_opt <= 1)
                    {
                        UART_DWORD_WRITE(DMA_UART_STS_REG,uart_reg_status);
                    }
#else
                        UART_DWORD_WRITE(DMA_UART_STS_REG,uart_reg_status);
#endif

                    if(g_data_uart_settings.err_recov_en)
                    {
                        DMA_UART_LOG(RED, DMA_UART_032, 0, 0);
                        hci_uart_reset_init_RTL8723(1);
                    }
                    // TODO: UART Reset?? DMA Reset??                

#ifdef _UART_H5 /* for H4 error w1c after event RX */
                    if (g_data_uart_settings_3.err_send_event_delayw1c_opt >= 1)
                    {
                        DMA_UART_LOG(RED, DMA_UART_033, 0, 0);
                        hci_generate_hw_error_event(hci_uart_man.h4_hw_err_event_code);
                            
                        if (g_data_uart_settings_3.err_send_event_delayw1c_opt >= 2)
                        {
                            hci_uart_set_h4_error_interrupt_en(0);
                            hci_uart_man.h4_err_event_wait_rx_state = 1;
                            hci_uart_man.h4_err_event_wait_recov_timer_state = 1;
                        }
                    }
#else
                    DMA_UART_LOG(RED, DMA_UART_033, 0, 0);
                    hci_generate_hw_error_event(HARDWARE_FAILURE_ERROR);
#endif
                    
#endif
                }
            }
        }
#endif

#ifdef _UART_H5
        // TODO: Priority and Excution Order??? 
        // TODO: check g_fun_interface_info.b.bt_interface ??
        if (g_fun_interface_info.b.bt_interface == UART_INTERFACE)
        {
//            if (1) // TODO: to be reviewed
            if (g_data_uart_settings_2.h5_en)
            {        
                hci_uart_h5_isr(isr_status.d32);            
            }
#ifdef _UART_BAUD_ESTIMATE_
            hci_uart_falling_cnt_isr(isr_status.d32);
#endif        
        }
#endif

        isr_status.d32 = DMA_DWORD_READ(ISR_STATUS_REG);

        if (!(isr_status.d32 & isr_mask))
        {
            /* check any new pending interrupt flags */
            break;
        }
    }
    return;
}

BOOLEAN S3C2410Usb_Dma(HCI_TRANSPORT_PKT_TYPE pkt_type,
                       UCHAR *buf, UINT16 len, UINT32 free_index)
{
    UINT32 temp_addr = (UINT32) buf;
    UINT8 wptr;  
    GENERAL_CONTROL_S_TYPE bt_general_ctrl;
    UINT16 conn_handle;

/*	Added by Wallice for USB2.0 Debug.	2011/12/16	*/
    BZDMA_REG_S Reg;
    UINT32 ticks;
/*	End Added by Wallice for USB2.0 Debug.	2011/12/16	*/
        
#ifdef _UART_H5
    OS_BOOL is_h5_sco_flow_control;
#endif


    DEF_CRITICAL_SECTION_STORAGE;

    //the setting value loaded from efuse

#ifdef _ROM_CODE_PATCHED_
    if (rcp_S3C2410Usb_Dma_wake_host != NULL)
    {
        rcp_S3C2410Usb_Dma_wake_host((void*)buf, pkt_type, len, free_index);
    }
    else
#endif        
    {
        bt_general_ctrl.d16 = otp_str_data.general_control;
        if (g_fun_interface_info.b.bt_interface == USB_INTERFACE)    
        {
            MINT_OS_ENTER_CRITICAL();
            if (g_host_state)
            {
#ifdef _ENABLE_BTON_POWER_SAVING_
                if (usb_read_d2_remote_wakeup_feature())
                {
                    usb_remote_wakeup();   
                }

#endif
            	g_host_state = 0;            
            }
            MINT_OS_EXIT_CRITICAL();    
        }
        else
        {
#ifdef _ENABLE_BTON_POWER_SAVING_
            //When efuse enable wake up scheme and host is sleep mode, 
            //controller need to wake up the host and queue rx packet.
            if (bt_general_ctrl.b.gpio_wake_up_fun_en || bt_general_ctrl.b.uart_h5_wakeup_en)
            {
                if(!execute_wakeup_procedure(pkt_type, buf, len))
                {
                    //wake up host and queue packet
                    return TRUE;
                }
            }
#endif
        }
    }
    
#ifdef _CCH_LPS_
    g_lps_timer_counter =  g_timer2_init_value;
#endif

    switch (pkt_type)
    {
    case HCI_TRANSPORT_EVENT_PKT_TYPE:        	
        MINT_OS_ENTER_CRITICAL();
        if (!dma_man.evt_available_num || (len < 2))
        {
            RT_BT_LOG(RED, RX_FIFO_FULL, 2, dma_man.evt_rx_w_ind,dma_man.evt_rx_r_ind);
            OS_FREE_BUFFER(tx_table[HCI_EVENT_HANDLER_TASK].pool_handle,buf);
            hci_vendor_check_log_data_packet(VENDOR_LOG_PACKET_TYPE_EVENT, 
                                             buf, TRUE);
            goto error;
        }                

        /* get current write pointer */
        wptr = dma_man.evt_rx_w_ind;

        if (enable_hci_loopback_mode)
            cmd_fifo_index[wptr] = free_index;

        // EVT_DES_NUM is even.
        dma_man.evt_rx_w_ind = ((wptr + 1) & (EVT_RX_DES_NUM-2));
        dma_man.evt_available_num--;   
        dma_man.dma_evt_rx_free_addr[wptr] = temp_addr;

        dma_evt_rx_des[wptr].rx_des_dw1.payload_start_add = (0xFFFF & (temp_addr + 2));
        dma_evt_rx_des[wptr].rx_des_evt_dw2.event_pkt_length = len-2;
        dma_evt_rx_des[wptr].rx_des_evt_dw2.event_code = buf[0];
        dma_evt_rx_des[wptr].rx_des_dw1.own = 1;
        
#ifdef _ROM_CODE_PATCHED_
        if (rcp_usb_req_rx_dma_func != NULL)
        {
            /* because usb lpm_l1 has dirty design, if bt needs to support this 
               feature and fw may does additional queue and wakeup flow (austin) */
            if (!rcp_usb_req_rx_dma_func(&pkt_type))
            {
                goto error;
            }
        }
        else
#endif
        {
#ifdef _SUPPORT_NEW_USB_LPM_L1_DATA_PATH_150629_
            /* check interface is usb or not */
            if (IS_SUPPORT_USB_LPM_L1_FSM &&
                (g_fun_interface_info.b.bt_interface == USB_INTERFACE))
            {
                if (hci_dma_usb_check_usb_lpm_l1_then_queue_notification(pkt_type))
                {
                    goto error;
                }
            }
#endif

            //notify DMA that there is a valid event packet
            DMA_DWORD_WRITE(CMD_TX_BUF_FREE_RX_START_CTL_REG, EVT_NOTIFY_HW);
        }

        MINT_OS_EXIT_CRITICAL();  
        break;

    case HCI_TRANSPORT_ACL_DATA_PKT_TYPE:
        /*	Added by Wallice for USB2.0 Debug.	2011/12/16	*/
        if (!IS_AGGRESIVE_HCI_RX_DMA || 
//#ifndef _RTL8822B_SPECIFIC_
#ifndef _SUPPORT_3BITS_HCI_SELECTION_
#ifndef _SUPPORT_2BIT_HCI_AND_PCIE_UART_
            ((g_fun_interface_info.b.hci_sel == RTK8723_U) ||
            (g_fun_interface_info.b.hci_sel == RTK8723_EE)))
#else
            (g_fun_interface_info.b.hci_sel == PACKAGE_USB_USB))
#endif
#else
             (g_fun_interface_info.b.hci_sel == PACKAGE_USB_USB))
#endif

        {
            ticks = 0x00;
            while (1)
            {
                Reg.DWord = RD_U32_BZDMA_REG(BZDMA_REG_INT_STATUS); 
                if (!Reg.IntStatus.rx_busy)
                {
                    break;
                }
        
                ticks++; 
        
                if (ticks > 100000)
                {
                    RT_BT_LOG(RED, MSG_BZDMA_RXCMD_TIMEOUT, 0, 0); 
                    break;
                }
            }
        }
        /*  End Added by Wallice for USB2.0 Debug.  2011/12/16  */

        MINT_OS_ENTER_CRITICAL();

        /* get current write pointer */
        wptr = dma_man.acl_rx_w_ind;

        if (!dma_man.acl_available_num || (len < 4))
        {
            RT_BT_LOG(RED, RX_FIFO_FULL, 2, dma_man.acl_rx_w_ind,dma_man.acl_rx_r_ind);                

            if (hci_vendor_check_log_data_packet(VENDOR_LOG_PACKET_TYPE_ACL, 
                                                 buf, TRUE))
            {
                OS_FREE_BUFFER(tx_table[ACL_DATA_HANDLER_TASK].pool_handle,buf);
                os_free_reserved_buffer();
                goto error;                    
            }

#ifdef LE_MODE_EN
            if (dma_acl_rx_des[wptr].rx_des_acl_dw2.handle >= LL_HCI_MIN_CONN_HANDLE)
            {
                /* drop le packets */
                OS_FREE_BUFFER(tx_table[LE_ACL_DATA_HANDLER_TASK].pool_handle,buf);
#ifdef _ENABLE_BLE_LL_FLOW_CONTROL_
                os_release_one_le_reserved_buffer();
#endif    
                goto error;                    
            }
#endif                    

            OS_FREE_BUFFER(tx_table[ACL_DATA_HANDLER_TASK].pool_handle,buf);
            os_free_reserved_buffer();
            goto error;
        }


        if (enable_hci_loopback_mode)
        {
            acl_fifo_index[wptr] = free_index;
        }
        
        dma_man.acl_rx_w_ind = ((wptr + 1) & (ACL_RX_DES_NUM-2));
        dma_man.acl_available_num--;

        conn_handle = ((buf[1] & 0xF)<<8) | buf[0];

        dma_acl_rx_des[wptr].rx_des_dw1.payload_start_add = (0xFFFF & (UINT32) (temp_addr + 4));
        dma_acl_rx_des[wptr].rx_des_acl_dw2.data_total_length = len-4;
        dma_acl_rx_des[wptr].rx_des_acl_dw2.handle = conn_handle;
        dma_acl_rx_des[wptr].rx_des_acl_dw2.pb_flag = (buf[1] & 0x30)>>4;
        dma_acl_rx_des[wptr].rx_des_acl_dw2.bc_flag = (buf[1] & 0xc0)>>6;
        dma_acl_rx_des[wptr].rx_des_dw1.own = 1;
       
#ifdef LE_MODE_EN
        //4 Distinguish  Legacy and LE
        if (dma_acl_rx_des[wptr].rx_des_acl_dw2.handle >= LL_HCI_MIN_CONN_HANDLE)
        {
            if (!hci_vendor_check_log_data_packet(VENDOR_LOG_PACKET_TYPE_ACL, 
                                                 buf, FALSE))
            {
                temp_addr |= 0x6000000;
            }
        }
#endif

        dma_man.dma_acl_rx_free_addr[wptr] = temp_addr;

#ifdef _ROM_CODE_PATCHED_
        if (rcp_usb_req_rx_dma_func != NULL)
        {
            /* because usb lpm_l1 has dirty design, if bt needs to support this 
               feature and fw may does additional queue and wakeup flow (austin) */
            if (!rcp_usb_req_rx_dma_func(&pkt_type))
            {
                goto error;
            }
        }
        else
#endif
        {            
#ifdef _SUPPORT_NEW_USB_LPM_L1_DATA_PATH_150629_
            /* check interface is usb or not */
            if (IS_SUPPORT_USB_LPM_L1_FSM &&
                (g_fun_interface_info.b.bt_interface == USB_INTERFACE))
            {
                if (hci_dma_usb_check_usb_lpm_l1_then_queue_notification(pkt_type))
                {
                    goto error;
                }
            }
#endif

            //notify DMA that there is a valid event packet
            DMA_DWORD_WRITE(CMD_TX_BUF_FREE_RX_START_CTL_REG, ACL_NOTIFY_HW);
        }

        MINT_OS_EXIT_CRITICAL(); 

#ifdef SECURE_CONN_BROADCAST
#if 0
        if (dma_acl_rx_des[wptr].rx_des_acl_dw2.bc_flag == 1)
        {
            RT_BT_LOG(GREEN, DAPE_TEST_LOG551, 11,
            len-4, buf[0], buf[1],buf[2],buf[3],
            buf[4], buf[5],buf[6],buf[7],buf[8], buf[9]);
        }
#endif 
#endif 
#ifdef _SECURE_CONN_TEST_CHK_ACLU_DATA_BY_VENDOR_CMD
        if (g_chk_secure_conn_data)
        {
            RT_BT_LOG(GREEN, DAPE_TEST_LOG555, 11,
            len-4, buf[0], buf[1],buf[2],buf[3],
            buf[4], buf[5],buf[6],buf[7],buf[8], buf[9]);
        }
#endif
#ifdef PTA_EXTENSION
        if (is_wifi_alive) /* do not use it in bt only */
        {
            if( pta_meter_var.bPtaMeterSwitch )
            {
                if(conn_handle != 0x123)  //(BaRoN)
                {
                    pta_meter_var.dwPtaACLRxCnt += (len - 4);
                }
            }

            // 20111122 morgan for sniffer mode check
            // here is for check sniffer mode working  
#ifdef HID_ESTIMATION
            pta_hid_estimation(conn_handle);            
#endif
        }
#endif /* endif of PATCH_PTA_EXTENSION */
        
        break;

    case HCI_TRANSPORT_SYNC_DATA_PKT_TYPE:
        MINT_OS_ENTER_CRITICAL();
#ifndef _DMA_LOOPBACK_TEST_
        if (!enable_hci_loopback_mode)
        {
#ifndef _REMOVE_HCI_PCIE_        
            if ((g_fun_interface_info.b.bt_interface == USB_INTERFACE) ||
                (g_fun_interface_info.b.bt_interface == PCIE_INTERFACE)||
                (IS_USE_FOR_MUTE))         
#else
            if ((g_fun_interface_info.b.bt_interface == USB_INTERFACE) ||
                (IS_USE_FOR_MUTE))

#endif         
            {
                if (!bz_isoch_is_usb_ready || (len < 3))
                {
                        OS_FREE_BUFFER(synchronous_data_to_host_pool_id, buf);
                    goto error;
                }

                if (first_iso_in)
                {
                    dectect_pkt_flag = 0;
                }                    
                if ((dectect_pkt_flag > 6) || (IS_USE_FOR_MUTE))
                {
                    OS_FREE_BUFFER(synchronous_data_to_host_pool_id, buf);
                    goto error;
                }
            }
            
            if (!dma_man.sco_available_num)
            {
                RT_BT_LOG(RED, RX_FIFO_FULL, 2, dma_man.sco_rx_w_ind,dma_man.sco_rx_r_ind);
                OS_FREE_BUFFER(synchronous_data_to_host_pool_id,buf);
                goto error;
            }
        }
        //RT_BT_LOG(YELLOW, DBG_RX_INDEX, 1, dma_man.sco_rx_w_ind);
#ifdef _DAPE_TEST_FIX_DMA_HANG
        if (alternatset_change_rx_clear)
        {
           alternatset_change_rx_clear = 0;
           // Secnario: When fw had prepared the sco packet and didn't set to HCI DMA
           // if fw just cleared the packet in dma hardware, it still missed this error packet
           // So , if the len didn't equal to the present bz_sco_pkt_size, we need to free this packet
           if (len != bz_sco_pkt_size)
           {
              DMA_DBG_LOG(BLUE, RX_FREE_ADDR, 1, temp_addr);
              if(OS_FREE_BUFFER(synchronous_data_to_host_pool_id, (void*)temp_addr) != BT_ERROR_OK)
              {
                 RT_BT_LOG(GRAY, HCI_TD_893, 0, 0);
              }
              goto error;
           }
        }
#endif

        wptr = dma_man.sco_rx_w_ind;

        if (enable_hci_loopback_mode)
        {
            sco_fifo_index[wptr] = free_index;
        }
        dma_man.sco_rx_w_ind = ((wptr + 1) & (SCO_RX_DES_NUM-2));
        dma_man.sco_available_num--;

        if (!enable_hci_loopback_mode)
        {
#ifndef _REMOVE_HCI_PCIE_         
            if ((g_fun_interface_info.b.bt_interface == USB_INTERFACE) ||
                (g_fun_interface_info.b.bt_interface == PCIE_INTERFACE))
#else
            if(g_fun_interface_info.b.bt_interface == USB_INTERFACE)
#endif

            {
                if (!first_iso_in)
                {
                    dectect_pkt_flag++;
                }
            }                        
        }
#else
        wptr = dma_man.sco_rx_w_ind;
        dma_man.sco_rx_w_ind = ((wptr + 1) & (SCO_RX_DES_NUM-2));
        dma_man.sco_available_num--;
#endif
        dma_man.dma_sco_rx_free_addr[wptr] = temp_addr;

        dma_sco_rx_des[wptr].rx_des_dw1.payload_start_add= (0xFFFF & (UINT32) (temp_addr + 4));
        dma_sco_rx_des[wptr].rx_des_sco_dw2.data_total_length = len-3;
        dma_sco_rx_des[wptr].rx_des_sco_dw2.handle = ((buf[2] & 0xF)<<8) | buf[1];
        dma_sco_rx_des[wptr].rx_des_sco_dw2.pkt_sta_flag = (buf[2] & 0x30)>>4;;
#ifdef _UART_H5
        // TODO: make sure lmp_self_device_data.sco_flow_control_enable would be store/restore when entering/leaving LowPower
        // TODO: need to check interface type (HOW?)

        if ((g_fun_interface_info.b.bt_interface == UART_INTERFACE) && g_data_uart_settings_2.h5_en)
        {
            if (g_data_uart_settings_3.h5_scounrel_force_en)
            {
                dma_sco_rx_des[wptr].rx_des_dw1.h5_unrel=g_data_uart_settings_3.h5_scounrel_force_value;
            }
            else
            {
                is_h5_sco_flow_control = ((lmp_self_device_data.sco_flow_control_enable == 1) &&
                                                        ((lmp_self_device_data.flow_control_hc_to_host == 2) ||
                                                            (lmp_self_device_data.flow_control_hc_to_host == 3))) ? TRUE : FALSE;            
                if(is_h5_sco_flow_control)                        
                    dma_sco_rx_des[wptr].rx_des_dw1.h5_unrel = 0;
                else
                    dma_sco_rx_des[wptr].rx_des_dw1.h5_unrel = 1;
            }
        }
        else
        {
            dma_sco_rx_des[wptr].rx_des_dw1.h5_unrel = 0;
        }
            
#endif 
        
        dma_sco_rx_des[wptr].rx_des_dw1.own = 1;

#ifdef _ROM_CODE_PATCHED_
        if (rcp_usb_req_rx_dma_func != NULL)
        {
            /* because usb lpm_l1 has dirty design, if bt needs to support this 
               feature and fw may does additional queue and wakeup flow (austin) */
            if (!rcp_usb_req_rx_dma_func(&pkt_type))
            {
                goto error;
            }
        }
        else
#endif
        {            
#ifdef _SUPPORT_NEW_USB_LPM_L1_DATA_PATH_150629_
            /* check interface is usb or not */
            if (IS_SUPPORT_USB_LPM_L1_FSM &&
                (g_fun_interface_info.b.bt_interface == USB_INTERFACE))
            {
                if (hci_dma_usb_check_usb_lpm_l1_then_queue_notification(pkt_type))
                {
                    goto error;
                }
            }
#endif

            DMA_DWORD_WRITE(CMD_TX_BUF_FREE_RX_START_CTL_REG, SCO_NOTIFY_HW);           
        }         

        MINT_OS_EXIT_CRITICAL();  

#ifdef PTA_EXTENSION
        if (is_wifi_alive) /* do not use it in bt only */
        {
            //20111130 morgan add for SCO traffic measure
            if( pta_meter_var.bPtaMeterSwitch )
            {
                pta_meter_var.dwPtaSCORxCnt += (len - 3);
            }
        }
#endif
        break;

    default:
        RT_BT_LOG(RED, UNKONW_PKT_TYPE, 0, 0);
        break;

    }

    return TRUE;

error:

    MINT_OS_EXIT_CRITICAL();
    
    return FALSE;
}

/**
 * Function     : dma_des_init
 *
 * Description  : This function initialises dma descriptor
 *
 * Parameters   : None
 *
 * Returns      : None
 *
 * Side Effects : None
 *
 */
void dma_des_init()
{
    UINT8 i;
    RT_BT_LOG(RED, DMA_DES_INIT, 0, 0);
    MEM_DBG_LOG(WHITE, DMA_ACL_TX_DES, 0, 0);

    g_dma_tx_des_entity_rptr = 0;
    g_dma_rx_des_entity_rptr = 0;

//4 -----------------------------------------  Initial DMA TX  ------------------------------------------------------
    //4 Initial ACL DMA setting
    dma_des_allocate(dma_tx_pool_id, acl_data_pool_id,
                     IS_TX, ((void**)&dma_acl_tx_des), BT_FW_TOTAL_ACL_PKTS_FROM_HOST);
    for (i=0; i<BT_FW_TOTAL_ACL_PKTS_FROM_HOST; i++)
    {
        MEM_DBG_LOG(BLUE, ALLOCATE_TX_DMA_ADDRESS, 2, (dma_acl_tx_des+i), (*(dma_acl_tx_des+i)));
    }

    //4 Initial SCO DMA setting
    MEM_DBG_LOG(WHITE, DMA_SCO_TX_DES, 0, 0);
    MEM_DBG_LOG(BLUE, SAPARATE_LINE, 0, 0);
    dma_des_allocate(dma_tx_pool_id, synchronous_data_pool_id,
                     IS_TX, ((void**)&dma_sco_tx_des), BT_FW_TOTAL_SYNCHRONOUS_PKTS_FROM_HOST);
    for (i=0; i<BT_FW_TOTAL_SYNCHRONOUS_PKTS_FROM_HOST; i++)
    {
        MEM_DBG_LOG(BLUE, ALLOCATE_TX_DMA_ADDRESS, 2, (dma_sco_tx_des+i), (*(dma_sco_tx_des+i)));
    }

    //4 Initial CMD DMA setting
    MEM_DBG_LOG(WHITE, DMA_CMD_TX_DES, 0, 0);
    MEM_DBG_LOG(BLUE, SAPARATE_LINE, 0, 0);
    dma_des_allocate(dma_tx_pool_id, hci_cmd_buffer_pool_handle,
                     IS_TX, ((void**)&dma_cmd_tx_des), BT_FW_CMD_BUFFERS);
    for (i=0; i<BT_FW_CMD_BUFFERS; i++)
    {
        MEM_DBG_LOG(BLUE, ALLOCATE_TX_DMA_ADDRESS, 2, (dma_cmd_tx_des+i), (*(dma_cmd_tx_des+i)));
    }

#ifdef LE_MODE_EN
    //4 Initial LE ACL DMA setting
    MEM_DBG_LOG(WHITE, DMA_LE_ACL_TX_DES, 0, 0);
    MEM_DBG_LOG(BLUE, SAPARATE_LINE, 0, 0);
#ifndef LE_HW_TEST
    dma_des_allocate(dma_tx_pool_id, le_acl_data_to_ll_pool_id,
                     IS_TX, ((void**)&dma_le_acl_tx_des), LL_POLL_HCI_MAX_TX_ACL_PKT_CNT);
#endif
    for (i=0; i<LL_POLL_HCI_MAX_TX_ACL_PKT_CNT; i++)
    {
        MEM_DBG_LOG(BLUE, ALLOCATE_TX_DMA_ADDRESS, 2, (dma_le_acl_tx_des+i), (*(dma_le_acl_tx_des+i)));
    }
#endif
//4 -----------------------------------------  Initial DMA RX  ------------------------------------------------------
    //set memory address to segment 1 (no-cache able), 0xa000 0000 start
    //4 Initial ACL DMA setting
    dma_des_allocate(dma_rx_pool_id, NO_NEED_POOL_ID,
                     IS_RX, ((void**)&dma_acl_rx_des), ACL_RX_DES_NUM);
    for (i=0; i<ACL_RX_DES_NUM; i++)
    {
        MEM_DBG_LOG(BLUE, ALLOCATE_RX_DMA_ADDRESS, 1, (dma_acl_rx_des+i));
    }
    //4 Initial SCO DMA setting
    MEM_DBG_LOG(BLUE, SAPARATE_LINE, 0, 0);
    dma_des_allocate(dma_rx_pool_id, NO_NEED_POOL_ID,
                     IS_RX, ((void**)&dma_sco_rx_des), SCO_RX_DES_NUM);
    for (i=0; i<SCO_RX_DES_NUM; i++)
    {
        MEM_DBG_LOG(BLUE, ALLOCATE_RX_DMA_ADDRESS, 1, (dma_sco_rx_des+i));
    }
    //4 Initial CMD DMA setting
    MEM_DBG_LOG(BLUE, SAPARATE_LINE, 0, 0);
    dma_des_allocate(dma_rx_pool_id, NO_NEED_POOL_ID,
                     IS_RX,( (void**)&dma_evt_rx_des), EVT_RX_DES_NUM);
    for (i=0; i<EVT_RX_DES_NUM; i++)
    {
        MEM_DBG_LOG(BLUE, ALLOCATE_RX_DMA_ADDRESS, 1, (dma_evt_rx_des+i));
    }

    //initial rx des and set end bit
    //ACL
    for (i=0; i<ACL_RX_DES_NUM; i++)
    {
        dma_acl_rx_des[i].rx_des_dw1_u32 = 0;
        dma_acl_rx_des[i].rx_des_dw2_u32 = 0;
    }
    dma_acl_rx_des[ACL_RX_DES_NUM - 1].rx_des_dw1.end = 1;
    dma_acl_rx_des[ACL_RX_DES_NUM - 1].rx_des_dw1.payload_start_add = (UINT16)(0xFFFF & ((UINT32)dma_acl_rx_des));
    //SCO
    for (i=0; i<SCO_RX_DES_NUM; i++)
    {
        dma_sco_rx_des[i].rx_des_dw1_u32 = 0;
        dma_sco_rx_des[i].rx_des_dw2_u32 = 0;
    }
    dma_sco_rx_des[SCO_RX_DES_NUM - 1].rx_des_dw1.end = 1;
    dma_sco_rx_des[SCO_RX_DES_NUM - 1].rx_des_dw1.payload_start_add = (UINT16)(0xFFFF & ((UINT32)dma_sco_rx_des));
    //Event
    for (i=0; i<EVT_RX_DES_NUM; i++)
    {
        dma_evt_rx_des[i].rx_des_dw1_u32 = 0;
        dma_evt_rx_des[i].rx_des_dw2_u32 = 0;
    }
    dma_evt_rx_des[EVT_RX_DES_NUM - 1].rx_des_dw1.end= 1;
    dma_evt_rx_des[EVT_RX_DES_NUM - 1].rx_des_dw1.payload_start_add = (UINT16)(0xFFFF & ((UINT32)dma_evt_rx_des));

#ifdef _USB_DUMP_REG_
    dump_dma_tx_des();
#endif
    return;
}

/**
 * Function     : dma_des_allocate
 *
 * Description  : This function handle memory allocated which dma descriptor required
 *
 * Parameters   : pkt_num    : Packet number
 *                entry_size : Entry size of every packet
 *                des_p      : TX descriptor point
 *                is_tx      : TX or RX
 *
 * Returns      : None
 *
 * Side Effects : None
 *
 */
void dma_des_allocate(POOL_ID dma_trx_pool_id, POOL_ID tx_pool_id,
                      UINT8 is_tx, void**des_p, UINT32 buf_num)
{
    UINT8 i;
    UINT32 *temp_addr,*temp;
    UINT32 address = 0;

    if (is_tx)
    {
        address = (UINT32)&g_dma_tx_des_entity[g_dma_tx_des_entity_rptr];
        g_dma_tx_des_entity_rptr += buf_num;
    }
    else
    {
        address = (UINT32)&g_dma_rx_des_entity[g_dma_rx_des_entity_rptr];  
        g_dma_rx_des_entity_rptr += buf_num;
    }
    address |= NO_CACHE_ABLE_MASK;

    *des_p = (UINT32 *) address;
    temp = (UINT32 *) address;
    if (is_tx)
    {
        for (i=(buf_num); i>0; i--)
        {
            OS_ALLOC_BUFFER(tx_pool_id, (void **)(&temp_addr));
            address = (UINT32) temp_addr;
            address = ((UINT32)address | NO_CACHE_ABLE_MASK);
            temp[i-1] = address;
        }
    }

}


/**
 * Function     : dma_tx_fifo_order_free
 *
 * Description  : This function provide the lower layer to free hardware
 *                dma tx fifo oredr
 *
 * Parameters   : free_bitmap: Which fifo order will be freed
 *                type       : ACL buffer, SCO buffer, or CMD buffer
 *
 * Returns      : None
 *
 * Side Effects : None
 *
 */
void dma_tx_fifo_order_free(UINT16 free_bitmap, UINT8 type)
{
#ifdef _YL_H5_DBG // more detailed debug message
//        DMA_UART_DBG_LOG(GRAY, DMA_UART_082, 1, free_bitmap); // Note: is would produce error message...WHY???
#endif        
    free_bitmap = ((1 << free_bitmap) & 0xFFFF);
    DMA_DBG_LOG(WHITE, FREE_DMA_TX_FIFO, 0, 0);
    switch (type)
    {
        case HCI_TRANSPORT_CMD_PKT_TYPE:
            DMA_DWORD_WRITE(CMD_TX_BUF_FREE_RX_START_CTL_REG, free_bitmap);
            break;
        case HCI_TRANSPORT_ACL_DATA_PKT_TYPE:
            DMA_DWORD_WRITE(ACL_TX_BUF_FREE_REG, free_bitmap);
            break;
        case HCI_TRANSPORT_SYNC_DATA_PKT_TYPE:
            DMA_DWORD_WRITE(SCO_TX_BUF_FREE_REG, free_bitmap);
            break;
        case HCI_TRANSPORT_LE_ACL_DATA_PKT_TYPE:
            DMA_DWORD_WRITE(LE_TX_BUF_FREE_REG, free_bitmap);
            break;
        default:
            RT_BT_LOG(RED, DMA_TX_FIFO_FREE_TYPE_ERROR, 0, 0);
            break;
    }
    return;
}


/**
 * Function     : dma_tx_fifo_pkt_free
 *
 * Description  : This function provide the lower layer to free usb tx fifo pkt
 *                and hardware dma tx fifo oredr - added by austin
 *
 * Parameters   : ppkt: the packet pointer
 *                type: ACL buffer, SCO buffer, or CMD buffer
 *
 * Returns      : BT_ERROR or BT_ERROR_OK
 *
 * Side Effects : None
 *
 */
UINT16 dma_tx_fifo_pkt_free(void *ppkt, UINT8 type)
{
    UINT16 reg_offset = 0;
    UINT32 idx = 0;
    UINT16 status = BT_ERROR;
    UINT32 u32pkt = (UINT32)ppkt;
    POOL_ID pool_index = 0;
    DEF_CRITICAL_SECTION_STORAGE;
    
    DMA_DBG_LOG(WHITE, FREE_DMA_TX_FIFO, 0, 0);

    switch (type)
    {
        case HCI_TRANSPORT_CMD_PKT_TYPE:
#if (BT_FW_CMD_BUFFERS > 0) && (HCI_CMD_BUFFER_SIZE > 0)
            if ((u32pkt < dma_cmd_tx_des[0]) ||
                    (u32pkt > dma_cmd_tx_des[BT_FW_CMD_BUFFERS-1]))
            {
                break;
            }
            status = BT_ERROR_OK;
            reg_offset = CMD_TX_BUF_FREE_RX_START_CTL_REG;
            idx = (u32pkt - dma_cmd_tx_des[0]) / HCI_CMD_BUFFER_SIZE;
#endif
            break;
#ifdef LE_MODE_EN
        case HCI_TRANSPORT_LE_ACL_DATA_PKT_TYPE:
#if (LL_POLL_HCI_MAX_TX_ACL_PKT_CNT > 0) && (LL_POLL_HCI_MAX_TX_ACL_PKT_SIZE > 0)
            if ((u32pkt < dma_le_acl_tx_des[0]) ||
                    (u32pkt > dma_le_acl_tx_des[LL_POLL_HCI_MAX_TX_ACL_PKT_CNT-1]))
            {
                break;
            }
            status = BT_ERROR_OK;
            reg_offset = LE_TX_BUF_FREE_REG;
            idx = (u32pkt - dma_le_acl_tx_des[0]) / LL_POLL_HCI_MAX_TX_ACL_PKT_SIZE;
            pool_index = le_acl_data_to_ll_pool_id;
#endif
            break;
#endif
        case HCI_TRANSPORT_ACL_DATA_PKT_TYPE:
#if (BT_FW_TOTAL_ACL_PKTS_FROM_HOST > 0) && (HCI_ACL_DATA_PKT_SIZE > 0)
            if ((u32pkt < dma_acl_tx_des[0]) ||
                    (u32pkt > dma_acl_tx_des[BT_FW_TOTAL_ACL_PKTS_FROM_HOST-1]))
            {
                break;
            }
            status = BT_ERROR_OK;
            reg_offset = ACL_TX_BUF_FREE_REG;
            idx = (u32pkt - dma_acl_tx_des[0]) / HCI_ACL_DATA_PKT_SIZE;
            pool_index = acl_data_pool_id;
#endif
            break;

        case HCI_TRANSPORT_SYNC_DATA_PKT_TYPE:
#if (BT_FW_TOTAL_SYNCHRONOUS_PKTS_FROM_HOST > 0) && (HCI_SYNCHRONOUS_DATA_PKT_SIZE > 0)
            if ((u32pkt < dma_sco_tx_des[0]) ||
                    (u32pkt > dma_sco_tx_des[BT_FW_TOTAL_SYNCHRONOUS_PKTS_FROM_HOST-1]))
            {
                break;
            }
            status = BT_ERROR_OK;
            reg_offset = SCO_TX_BUF_FREE_REG;
            idx = (u32pkt - dma_sco_tx_des[0]) / HCI_SYNCHRONOUS_DATA_PKT_SIZE;
            pool_index = synchronous_data_pool_id;
#ifdef _DAPE_TEST_CHECK_SCO_TX_INDEX
//RT_BT_LOG(WHITE, DAPE_TEST_LOG525, 6,BB_read_native_clock(), 
//          dape_sco_tx_index, idx, 0,0,0);
#endif
			
#endif
            break;

        default:
            RT_BT_LOG(RED, DMA_TX_FIFO_FREE_TYPE_ERROR, 0, 0);
            return status;
    }

    MINT_OS_ENTER_CRITICAL();    
    if ((!g_buffer_free_type) || (type == HCI_TRANSPORT_CMD_PKT_TYPE))
    {
        if (status == BT_ERROR_OK)
        {
            DMA_DWORD_WRITE(reg_offset, 1 << idx);
        }
    }
    else
    {
        OS_FREE_BUFFER(pool_index,(void *)u32pkt);
    }
    MINT_OS_EXIT_CRITICAL();       
    
    return status;
}

void switch_hci_dma_setting(UINT8 mode)
{
    UINT8 i = 0;
    UINT32 temp_addr = 0;
    DMA_CONTRL_TYPE             dma_control_reg;
    ACL_TX_BUF_SIZE_TYPE        acl_tx_buf_size_reg;
    SCO_TX_BUF_SIZE_TYPE        sco_tx_buf_size_reg;
    CMD_TX_BUF_SIZE_TYPE        cmd_tx_buf_size_reg;

    DEF_CRITICAL_SECTION_STORAGE;
    
    dma_control_reg.d32 = DMA_DWORD_READ(DMA_CTL_REG);
    dma_control_reg.b.dma_rx_en = 0;
    dma_control_reg.b.dma_tx_en = 0;
    DMA_DWORD_WRITE(DMA_CTL_REG, dma_control_reg.d32);

    switch (mode)
    {
        case ENTER_TEST_MODE:
            acl_tx_buf_size_reg.d32 = DMA_DWORD_READ(ACL_TX_BUF_SIZE_REG);
            acl_tx_buf_size_reg.b.acl_tx_pkt_num = 0;
            DMA_DWORD_WRITE(ACL_TX_BUF_SIZE_REG, acl_tx_buf_size_reg.d32);

            sco_tx_buf_size_reg.d32 = DMA_DWORD_READ(SCO_TX_BUF_SIZE_REG);
            sco_tx_buf_size_reg.b.sco_tx_pkt_num = 0;
            DMA_DWORD_WRITE(SCO_TX_BUF_SIZE_REG, sco_tx_buf_size_reg.d32);

            MINT_OS_ENTER_CRITICAL();
            if (!g_buffer_free_type)
            {
                OS_RESET_POOL(&acl_data_pool_id);
                OS_RESET_POOL(&synchronous_data_pool_id);                
                g_buffer_free_type = 1;
            }

            OS_RESET_POOL(&acl_data_to_host_pool_id);
            
            MINT_OS_EXIT_CRITICAL();
            break;
            
        case LEAVE_TEST_MODE:
            acl_tx_buf_size_reg.d32 = DMA_DWORD_READ(ACL_TX_BUF_SIZE_REG);
            acl_tx_buf_size_reg.b.acl_tx_pkt_num = BT_FW_TOTAL_ACL_PKTS_FROM_HOST;
            DMA_DWORD_WRITE(ACL_TX_BUF_SIZE_REG, acl_tx_buf_size_reg.d32);

            sco_tx_buf_size_reg.d32 = DMA_DWORD_READ(SCO_TX_BUF_SIZE_REG);
            sco_tx_buf_size_reg.b.sco_tx_pkt_num = BT_FW_TOTAL_SYNCHRONOUS_PKTS_FROM_HOST;
            DMA_DWORD_WRITE(SCO_TX_BUF_SIZE_REG, sco_tx_buf_size_reg.d32);

            MINT_OS_ENTER_CRITICAL();
            if (g_buffer_free_type || g_hci_reset_buf_allcate)
            {
                for (i=0; i<BT_FW_TOTAL_ACL_PKTS_FROM_HOST; i++)
                {
                    OS_ALLOC_BUFFER(acl_data_pool_id, (void **)(&temp_addr));
                }
                for (i=0; i<BT_FW_TOTAL_SYNCHRONOUS_PKTS_FROM_HOST; i++)
                {
                    OS_ALLOC_BUFFER(synchronous_data_pool_id, (void **)(&temp_addr));
                }
                g_buffer_free_type = 0;
                g_hci_reset_buf_allcate = 0;
            }
            MINT_OS_EXIT_CRITICAL();
            break;
            
        case DUT_TEST_MODE_FIFO_FREE:
            OS_RESET_POOL(&acl_data_pool_id);
            OS_RESET_POOL(&synchronous_data_pool_id);  
            OS_RESET_POOL(&acl_data_to_host_pool_id);
            break;

        case ENTER_SUSPEND_MODE:
            acl_tx_buf_size_reg.d32 = DMA_DWORD_READ(ACL_TX_BUF_SIZE_REG);
            acl_tx_buf_size_reg.b.acl_tx_pkt_num = 0;
            DMA_DWORD_WRITE(ACL_TX_BUF_SIZE_REG, acl_tx_buf_size_reg.d32);

            sco_tx_buf_size_reg.d32 = DMA_DWORD_READ(SCO_TX_BUF_SIZE_REG);
            sco_tx_buf_size_reg.b.sco_tx_pkt_num = 0;
            DMA_DWORD_WRITE(SCO_TX_BUF_SIZE_REG, sco_tx_buf_size_reg.d32);

            cmd_tx_buf_size_reg.d32 = DMA_DWORD_READ(CMD_BUF_SIZE_REG);
            cmd_tx_buf_size_reg.b.cmd_tx_pkt_num = 0;
            DMA_DWORD_WRITE(CMD_BUF_SIZE_REG, cmd_tx_buf_size_reg.d32);
            break;

        case LEAVE_SUSPEND_MODE:
            acl_tx_buf_size_reg.d32 = DMA_DWORD_READ(ACL_TX_BUF_SIZE_REG);
            acl_tx_buf_size_reg.b.acl_tx_pkt_num = BT_FW_TOTAL_ACL_PKTS_FROM_HOST;
            DMA_DWORD_WRITE(ACL_TX_BUF_SIZE_REG, acl_tx_buf_size_reg.d32);

            sco_tx_buf_size_reg.d32 = DMA_DWORD_READ(SCO_TX_BUF_SIZE_REG);
            sco_tx_buf_size_reg.b.sco_tx_pkt_num = BT_FW_TOTAL_SYNCHRONOUS_PKTS_FROM_HOST;
            DMA_DWORD_WRITE(SCO_TX_BUF_SIZE_REG, sco_tx_buf_size_reg.d32);

            cmd_tx_buf_size_reg.d32 = DMA_DWORD_READ(CMD_BUF_SIZE_REG);
            cmd_tx_buf_size_reg.b.cmd_tx_pkt_num = BT_FW_CMD_BUFFERS;
            DMA_DWORD_WRITE(CMD_BUF_SIZE_REG, cmd_tx_buf_size_reg.d32);
            break;

        default:
            break;
    }

    if (mode != ENTER_SUSPEND_MODE)
    {
        dma_control_reg.d32 = DMA_DWORD_READ(DMA_CTL_REG);
        dma_control_reg.b.dma_rx_en = 1;
        dma_control_reg.b.dma_tx_en = 1;
        DMA_DWORD_WRITE(DMA_CTL_REG, dma_control_reg.d32);
    }

//    RT_BT_LOG(YELLOW,MSG_HCI_DMA_SET_MODE,1,mode);    
}

void wake_up_interrupt_switch(UINT8 mode)
{
    ISR_CONTRL_TYPE    isr_control_reg;
    isr_control_reg.d32 = DMA_DWORD_READ(ISR_ENABLE_REG);
    switch (mode)
    {
        case ENABLE_WAKE_UP_ISR:
            isr_control_reg.b.wake_up_int_en = 1;
            wakeup_isr_en = 1;
            break;
        case DISABLE_WAKE_UP_ISR:
            isr_control_reg.b.wake_up_int_en = 0;
            wakeup_isr_en = 0;
            break;
        default:
            RT_BT_LOG(RED,DO_NOT_SUPPORT_MODE,0,0);
            break;
    }
    DMA_DWORD_WRITE(ISR_ENABLE_REG, isr_control_reg.d32);
}

void clear_sco_buffer(void)
{
    SCO_TX_BUF_SIZE_TYPE        sco_tx_buf_size_reg;
    UINT8 i = 0;
    DEF_CRITICAL_SECTION_STORAGE;

    MINT_OS_ENTER_CRITICAL();

    sco_tx_buf_size_reg.d32 = 0;

    if (!enable_hci_loopback_mode)    
    {
        for (i=0;i<BT_FW_TOTAL_SYNCHRONOUS_PKTS_FROM_HOST;i++)
        {                
            dma_tx_fifo_order_free(i, HCI_TRANSPORT_SYNC_DATA_PKT_TYPE);
        }
    }
    alternatset_change_tx_clear = 0;
    
    sco_tx_buf_size_reg.d32 = DMA_DWORD_READ(SCO_TX_BUF_SIZE_REG);
    sco_tx_buf_size_reg.b.sco_tx_pkt_num = BT_FW_TOTAL_SYNCHRONOUS_PKTS_FROM_HOST;
    DMA_DWORD_WRITE(SCO_TX_BUF_SIZE_REG, sco_tx_buf_size_reg.d32);

    MINT_OS_EXIT_CRITICAL();
    
}

 /**
  * Function     : enable_filter_sco_connection
  *
  * Description  : This function provide the lower layer to enable hardware
  *                      filter the sco connection handle that firmware doesn't designate
  *
  * Parameters   : is_enable: more than 1: enable; 0:disable
  *                        con_handle: allow connection handle
  *                        mode: 0: connection handle match
  *                              1: connection handle and length match
  *                              2: connection handle and length match, and time base data lost chek
  *
  * Returns      : None
  *
  * Side Effects : None
  *
  */
void enable_filter_sco_connection(UINT8 sco_index, 
              UINT8 is_enable, UINT8 con_handle, UINT8 mode)
{
    SCO_CON_HANDLE_1_AND_2_TYPE     sco_con_handle_1_and_2_reg;
    SCO_CON_HANDLE_3_TYPE           sco_con_handle_3_reg;
    UINT8 mode_array[3] = {1, 3, 7};

    if ((mode > 2) || (sco_index > CON_HANDLE_3))
    {
        RT_BT_LOG(RED,DOES_NOT_SUPPORT_PARAMETER,0,0);
        return;
    }
    
    sco_con_handle_1_and_2_reg.d32 = DMA_DWORD_READ(SCO_CONNECTION_HANDLE_1_AND_2_REG);
    sco_con_handle_3_reg.d32 = DMA_DWORD_READ(SCO_CONNECTION_HANDLE_3_REG);

    switch (sco_index) {
        case CON_HANDLE_1:
            sco_con_handle_1_and_2_reg.b.con_handle1 = con_handle;

            if (is_enable)
                sco_con_handle_1_and_2_reg.b.con_handle1_vld = 1;
            else    
                sco_con_handle_1_and_2_reg.b.con_handle1_vld = 0;            

        case CON_HANDLE_2:
            sco_con_handle_1_and_2_reg.b.con_handle2 = con_handle;

            if (is_enable)
                sco_con_handle_1_and_2_reg.b.con_handle2_vld = 1;
            else    
                sco_con_handle_1_and_2_reg.b.con_handle2_vld = 0;            

            break;
        case CON_HANDLE_3:
            sco_con_handle_3_reg.b.con_handle3 = con_handle;
            if (is_enable)
                sco_con_handle_3_reg.b.con_handle3_vld= 1;
            else    
                sco_con_handle_3_reg.b.con_handle3_vld = 0;            
            
            break;
        default:        
            break;
    }

    if (g_fun_interface_info.b.bt_interface == USB_INTERFACE)
    {    
        /* for usb, we can filter connection handle or packet size or 
           time stamp */              
        sco_con_handle_1_and_2_reg.b.recover_mode = mode_array[mode];
    }
    else
    {
        /* for pci-e or uart, we only can filter connection handle */        
        sco_con_handle_1_and_2_reg.b.recover_mode = 1;
    }
    
    DMA_DWORD_WRITE(SCO_CONNECTION_HANDLE_3_REG, sco_con_handle_3_reg.d32);
    DMA_DWORD_WRITE(SCO_CONNECTION_HANDLE_1_AND_2_REG, sco_con_handle_1_and_2_reg.d32);

}



#ifdef _USB_DUMP_REG_
/**
 * Function     : Dump_USB_DMA_register
 *
 * Description  : This function print debug information for DMA register
 *
 * Parameters   : None.
 *
 * Returns      : None
 *
 * Side Effects : None
 *
 */
void dump_usb_dma_register()
{
    ACL_TX_BUF_SIZE_TYPE	acl_tx_buf_size_reg;
    SCO_TX_BUF_SIZE_TYPE	sco_tx_buf_size_reg;
    CMD_TX_BUF_SIZE_TYPE	cmd_tx_buf_size_reg;
#ifdef LE_MODE_EN
    LE_ACL_TX_BUF_SIZE_TYPE	le_acl_tx_buf_size_reg;
#endif

    DMA_DBG_LOG(YELLOW, ACL_TX_BUF_START_ADDR_LOG, 1, DMA_DWORD_READ(ACL_TX_BUF_START_ADDR_REG));
    DMA_DBG_LOG(YELLOW, SCO_TX_BUF_START_ADDR_LOG, 1, DMA_DWORD_READ(SCO_TX_BUF_START_ADDR_REG));
    DMA_DBG_LOG(YELLOW, CMD_TX_BUF_START_ADDR_LOG, 1, DMA_DWORD_READ(CMD_TX_BUF_START_ADDR_REG));
#ifdef LE_MODE_EN
    DMA_DBG_LOG(YELLOW, LE_ACL_TX_BUF_START_ADDR_LOG, 1, DMA_DWORD_READ(LE_ACL_TX_BUF_START_ADDR_REG));
#endif

    acl_tx_buf_size_reg.d32 = DMA_DWORD_READ(ACL_TX_BUF_SIZE_REG);
    DMA_DBG_LOG(YELLOW, ACL_TX_BUF_SIZE_LOG, 2, acl_tx_buf_size_reg.b.acl_tx_buf_entry_size, acl_tx_buf_size_reg.b.acl_tx_pkt_num);
    sco_tx_buf_size_reg.d32 = DMA_DWORD_READ(SCO_TX_BUF_SIZE_REG);
    DMA_DBG_LOG(YELLOW, SCO_TX_BUF_SIZE_LOG, 2, sco_tx_buf_size_reg.b.sco_tx_buf_entry_size, sco_tx_buf_size_reg.b.sco_tx_pkt_num);
    cmd_tx_buf_size_reg.d32 = DMA_DWORD_READ(CMD_BUF_SIZE_REG);
    DMA_DBG_LOG(YELLOW, CMD_TX_BUF_SIZE_LOG, 2, cmd_tx_buf_size_reg.b.cmd_tx_buf_entry_size, cmd_tx_buf_size_reg.b.cmd_tx_pkt_num);
#ifdef LE_MODE_EN
    le_acl_tx_buf_size_reg.d32 = DMA_DWORD_READ(LE_BUF_SIZE_REG);
    DMA_DBG_LOG(YELLOW, ACL_TX_BUF_SIZE_LOG, 2, le_acl_tx_buf_size_reg.b.le_tx_buf_entry_size, le_acl_tx_buf_size_reg.b.le_acl_tx_pkt_num);
#endif

    DMA_DBG_LOG(YELLOW, ACL_RX_DES_RING_BASE_LOG, 1, DMA_DWORD_READ(ACL_RX_DES_RING_BASE_REG));
    DMA_DBG_LOG(YELLOW, SCO_RX_DES_RING_BASE_LOG, 1, DMA_DWORD_READ(SCO_RX_DES_RING_BASE_REG));
    DMA_DBG_LOG(YELLOW, EVENT_RX_DES_RING_BASE_LOG, 1, DMA_DWORD_READ(EVENT_DES_RING_BASE_REG));

    DMA_DBG_LOG(YELLOW, ISR_ENABLE_LOG, 1, DMA_DWORD_READ(ISR_ENABLE_REG));
    DMA_DBG_LOG(YELLOW, DMA_CTL_LOG, 1, DMA_DWORD_READ(DMA_CTL_REG));

    return;
}

void dump_dma_tx_des()
{
    UINT8 i;
    DMA_DBG_LOG(YELLOW, DMA_ACL_TX_DES_LOG, 0, 0);
    for (i=0; i<BT_FW_TOTAL_ACL_PKTS_FROM_HOST; i++)
        DMA_DBG_LOG(YELLOW, DMA_TX_ADDRESS, 1, dma_acl_tx_des[i]);

    DMA_DBG_LOG(YELLOW, DMA_SCO_TX_DES_LOG, 0, 0);
    for (i=0; i<BT_FW_TOTAL_SYNCHRONOUS_PKTS_FROM_HOST; i++)
        DMA_DBG_LOG(YELLOW, DMA_TX_ADDRESS, 1, dma_sco_tx_des[i]);

    DMA_DBG_LOG(YELLOW, DMA_CMD_TX_DES_LOG, 0, 0);
    for (i=0; i<BT_FW_CMD_BUFFERS; i++)
        DMA_DBG_LOG(YELLOW, DMA_TX_ADDRESS, 1, dma_cmd_tx_des[i]);
#ifdef LE_MODE_EN
    DMA_DBG_LOG(YELLOW, DMA_LE_ACL_TX_DES_LOG, 0, 0);
    for (i=0; i<LL_POLL_HCI_MAX_TX_ACL_PKT_CNT; i++)
        DMA_DBG_LOG(YELLOW, DMA_TX_ADDRESS, 1, dma_le_acl_tx_des[i]);
#endif

}
#endif

#ifdef _RTL_DBG_UTILITY_
BOOLEAN rtk_vender_command(UINT8* in_buf)
{
    UINT8 i,vender_cmd_len,opcode,reg_8_value,*ptr_buf,send_packet = 0;
    UINT16 reg_16_value, bitmap;
    UINT32 reg_context,reg_address = 0,reg_32_value = 0;
    SCO_CON_HANDLE_1_AND_2_TYPE sco_con_handle_1_and_2_reg;

    ptr_buf = (UINT8 *)(((UINT32)pbuf) | NO_CACHE_ABLE_MASK);

    //byte[3]: opcode length
    //byte[4]: opcode
    //byte[5]-byte[16]: context

    if ((in_buf[0] == 0xFF) & (in_buf[1] == 0xFF))
    {
        RT_BT_LOG(BLUE, RTL_VENDER_COMMAND, 0, 0);
        vender_cmd_len = (UINT8)in_buf[3];
        opcode = (UINT8) in_buf[4];
        for (i=0; i<(vender_cmd_len+5); i++)
        {
            RT_BT_LOG(WHITE, _DEBUG_, 2, i, in_buf[i]);
        }
        switch(opcode)
        {
            case 1:
                reg_context = 1<<((UINT8)in_buf[5]);
                RT_BT_LOG(BLUE, RTL_VENDER_COMMAND_OPCODE_1, 1, in_buf[5]);
                RT_BT_LOG(BLUE, RTL_VENDER_COMMAND_OPCODE_1, 1, reg_context);
                //notify DMA that there is a valid sco packet
                DMA_DWORD_WRITE(CMD_TX_BUF_FREE_RX_START_CTL_REG, reg_context);
                bitmap = cmd_tx_index;
                dma_tx_fifo_order_free(bitmap ,HCI_TRANSPORT_CMD_PKT_TYPE);
                break;
            case 2:
                ptr_buf[0] = 0xFE;
                ptr_buf[1] = 0x9;
                ptr_buf[2] = BT_FW_CMD_BUFFERS;
                ptr_buf[3] = HCI_CMD_BUFFER_SIZE & 0xFF;
                ptr_buf[4] = (HCI_CMD_BUFFER_SIZE & 0xFF00) >> 8;
                ptr_buf[5] = BT_FW_TOTAL_ACL_PKTS_FROM_HOST;
                ptr_buf[6] = HCI_ACL_DATA_PKT_SIZE & 0xFF;
                ptr_buf[7] = (HCI_ACL_DATA_PKT_SIZE & 0xFF00) >> 8;
                ptr_buf[8] = BT_FW_TOTAL_SYNCHRONOUS_PKTS_FROM_HOST;
                ptr_buf[9] = HCI_SYNCHRONOUS_DATA_PKT_SIZE & 0xFF;
                ptr_buf[10] = (HCI_SYNCHRONOUS_DATA_PKT_SIZE & 0xFF00) >> 8;
                send_packet = 1;
                break;
            case 3:
                sco_con_handle_1_and_2_reg.d32 = 0;
                sco_con_handle_1_and_2_reg.b.iso_al_setting = (UINT8)in_buf[5];
                //DMA_DWORD_WRITE(SCO_CONNECTION_HANDLE_1_AND_2_REG, sco_con_handle_1_and_2_reg.d32);
                RT_BT_LOG(BLUE, SCO_HANDLE_REG_CONTEXT, 1, DMA_DWORD_READ(SCO_CONNECTION_HANDLE_1_AND_2_REG));
                ptr_buf[0] = 0xFE;
                ptr_buf[1] = 0x1;
                ptr_buf[2] = 0x1; // Success
                //send_packet = 1;
                bitmap = cmd_tx_index;
                dma_tx_fifo_order_free(bitmap ,HCI_TRANSPORT_CMD_PKT_TYPE);
                break;
            case 4:
                reg_address = in_buf[7] + (in_buf[8]<<8) + (in_buf[9]<<16) + (in_buf[10]<<24);
                RT_BT_LOG(BLUE, READ_WRITE_COMMAND, 0, 0);
                RT_BT_LOG(BLUE, TX_DES_CONTEXT, 1, reg_address);

                if (in_buf[5])
                {
                    switch(in_buf[6])
                    {
                        case 1:
                            reg_8_value = in_buf[11];
                            RT_BT_LOG(BLUE, REG_WRITE_VALUE, 1, reg_8_value);
                            ((*((volatile UINT8*)(reg_address))) = reg_8_value);
                            break;
                        case 2:
                            reg_16_value = in_buf[11] + (in_buf[12]<<8);
                            RT_BT_LOG(BLUE, REG_WRITE_VALUE, 1, reg_16_value);
                            ((*((volatile UINT16*)(reg_address))) = reg_16_value);
                            break;
                        case 3:
                            reg_32_value = in_buf[11] + (in_buf[12]<<8) + (in_buf[13]<<16) + (in_buf[14]<<24);
                            RT_BT_LOG(BLUE, REG_WRITE_VALUE, 1, reg_32_value);
                            ((*((volatile UINT32*)(reg_address))) = reg_32_value);
                            break;
                        default:

                            break;
                    }
                    ptr_buf[1] = 0x1;
                    ptr_buf[2] = 0x1;
                }
                else
                {

                    switch(in_buf[6])
                    {
                        case 1:
                            reg_8_value = (*((volatile UINT8*)(reg_address)));
                            RT_BT_LOG(BLUE, REG_READ_VALUE, 1, reg_8_value);
                            ptr_buf[2] = reg_8_value;
                            break;
                        case 2:
                            reg_16_value = (*((volatile UINT16*)(reg_address)));
                            RT_BT_LOG(BLUE, REG_READ_VALUE, 1, reg_16_value);
                            ptr_buf[2] = (reg_16_value & 0xFF);
                            ptr_buf[3] = ((reg_16_value>>8) & 0xFF);
                            break;
                        case 3:
                            reg_32_value = (*((volatile UINT32*)(reg_address)));
                            RT_BT_LOG(BLUE, REG_READ_VALUE, 1, reg_32_value);
                            ptr_buf[2] = (reg_32_value & 0xFF);
                            ptr_buf[3] = ((reg_32_value>>8) & 0xFF);
                            ptr_buf[4] = ((reg_32_value>>16) & 0xFF);
                            ptr_buf[5] = ((reg_32_value>>24) & 0xFF);
                            for (i=0; i<4; i++)
                            {
                                RT_BT_LOG(WHITE, _DEBUG_, 2, i, ptr_buf[i+2]);
                            }
                            break;
                        default:

                            break;
                    }
                    ptr_buf[1] = 0x4;
                }
                ptr_buf[0] = 0xFE;
                send_packet = 1;
                break;

            default:
                break;
        }

        RT_BT_LOG(BLUE, ISR_TRIGGER, 0, 0);
        if (send_packet == 1)
        {
            if (pending_cmd_packet < BT_FW_CMD_BUFFERS)
            {
                RT_BT_LOG(BLUE, ISR_TRIGGER, 0, 0);
                tx_cmd_fifo_index[pending_cmd_packet] = cmd_tx_index;
            }
            else
            {
                RT_BT_LOG(RED, LOOPBACK_TX_INDEX_ERROR, 1, cmd_tx_index);
                goto error;
            }

            pending_cmd_packet++;
            
            pf_hci_transport_write(HCI_TRANSPORT_EVENT_PKT_TYPE, pbuf, pbuf[1],0);
        }
        send_packet = 0;
    error:
        return TRUE;
    }

    return FALSE;
}

#endif

#ifdef _RX_CMD_DBG_
void cmd_rx_test_step_1()
{
    UINT8 *temp_addr_2;
    temp_addr_2 = ((UINT32)temp_addr_1 | 0xa0000000);
    temp_addr_2[0] = 0x12;
    temp_addr_2[1] = 0x07;
    temp_addr_2[2] = 0x00;
    temp_addr_2[3] = 0x01;
    temp_addr_2[4] = 0x02;
    temp_addr_2[5] = 0x03;
    temp_addr_2[6] = 0x04;
    temp_addr_2[7] = 0x05;
    temp_addr_2[8] = 0x06;

    dma_evt_rx_des[evt_rx_index].rx_des_dw1.payload_start_add= (0xFFFF & (UINT32) (temp_addr_2 + 2));
    dma_evt_rx_des[evt_rx_index].rx_des_evt_dw2.event_pkt_length= 0x07;
    dma_evt_rx_des[evt_rx_index].rx_des_evt_dw2.event_code= 0x12;
    dma_evt_rx_des[evt_rx_index].rx_des_dw1.own = 1;

    RT_BT_LOG(YELLOW, EVENT_RX_DES_RING_BASE_LOG, 1, DMA_DWORD_READ(EVENT_DES_RING_BASE_REG));
    RT_BT_LOG(RED, EVT_RX_SRAM_CONTEXT, 1, (temp_addr_2 + 2));
    RT_BT_LOG(RED, EVT_RX_SRAM_CONTEXT, 1, temp_addr_2[0]);
    RT_BT_LOG(RED, EVT_RX_SRAM_CONTEXT, 1, temp_addr_2[1]);
    RT_BT_LOG(RED, EVT_RX_SRAM_CONTEXT, 1, temp_addr_2[2]);
    RT_BT_LOG(RED, EVT_RX_SRAM_CONTEXT, 1, temp_addr_2[3]);
    RT_BT_LOG(YELLOW, EVT_RX_SRAM_CONTEXT, 1, dma_evt_rx_des[evt_rx_index].rx_des_dw1_u32);
    RT_BT_LOG(YELLOW, EVT_RX_SRAM_CONTEXT, 1, dma_evt_rx_des[evt_rx_index].rx_des_dw2_u32);
    //notify DMA that there is a valid event packet
    DMA_DWORD_WRITE(CMD_TX_BUF_FREE_RX_START_CTL_REG, 0x20000000);
}

void cmd_rx_test_step_2()
{
    UINT8 *temp_addr_2;
    temp_addr_2 = ((UINT32)temp_addr_1 | 0xa0000000);
    temp_addr_2[0] = 0x12;
    temp_addr_2[1] = 0x07;
    temp_addr_2[2] = 0x00;
    temp_addr_2[3] = 0x01;
    temp_addr_2[4] = 0x02;
    temp_addr_2[5] = 0x03;
    temp_addr_2[6] = 0x04;
    temp_addr_2[7] = 0x05;
    temp_addr_2[8] = 0x06;
    evt_rx_index++;
    if (evt_rx_index == (EVT_RX_DES_NUM-1))
    {
        evt_rx_index = 0;
    }
    RT_BT_LOG(YELLOW, DBG_RX_INDEX, 1, evt_rx_index);
    dma_evt_rx_des[evt_rx_index].rx_des_dw1.payload_start_add= (0xFFFF & (UINT32) (temp_addr_2 + 2));
    dma_evt_rx_des[evt_rx_index].rx_des_evt_dw2.event_pkt_length= 0x07;
    dma_evt_rx_des[evt_rx_index].rx_des_evt_dw2.event_code= 0x12;
    dma_evt_rx_des[evt_rx_index].rx_des_dw1.own = 1;

    RT_BT_LOG(YELLOW, EVENT_RX_DES_RING_BASE_LOG, 1, DMA_DWORD_READ(EVENT_DES_RING_BASE_REG));
    RT_BT_LOG(RED, EVT_RX_SRAM_CONTEXT, 1, (temp_addr_2 + 2));
    RT_BT_LOG(RED, EVT_RX_SRAM_CONTEXT, 1, temp_addr_2[0]);
    RT_BT_LOG(RED, EVT_RX_SRAM_CONTEXT, 1, temp_addr_2[1]);
    RT_BT_LOG(RED, EVT_RX_SRAM_CONTEXT, 1, temp_addr_2[2]);
    RT_BT_LOG(RED, EVT_RX_SRAM_CONTEXT, 1, temp_addr_2[3]);
    RT_BT_LOG(YELLOW, EVT_RX_SRAM_CONTEXT, 1, dma_evt_rx_des[evt_rx_index].rx_des_dw1_u32);
    RT_BT_LOG(YELLOW, EVT_RX_SRAM_CONTEXT, 1, dma_evt_rx_des[evt_rx_index].rx_des_dw2_u32);

    //notify DMA that there is a valid event packet
    DMA_DWORD_WRITE(CMD_TX_BUF_FREE_RX_START_CTL_REG, 0x20000000);
    //free tx fifo
    //dma_tx_fifo_order_free(free_bitmap,HCI_TRANSPORT_CMD_PKT_TYPE);
}
#endif



#ifdef _RX_ACL_DBG_
void acl_rx_test_step_1()
{
    UINT8 *temp_addr_2, j;
    temp_addr_2 = ((UINT32)temp_addr_1 | 0xa0000000);
    temp_addr_2[0] = 0xFF;
    temp_addr_2[1] = 0xFF;
    temp_addr_2[2] = 0x10;
    temp_addr_2[3] = 0x00;
    for (j=0; j<64; j++)
    {
        temp_addr_2[4+j] = j;
    }
    for (j=0; j< 16; j++)
    {
        RT_BT_LOG(RED, _DEBUG_, 2, j, temp_addr_2[4+j]);
    }
    RT_BT_LOG(YELLOW, DBG_RX_INDEX, 1, acl_rx_index);
    dma_acl_rx_des[acl_rx_index].rx_des_dw1.payload_start_add= (0xFFFF & (UINT32) (temp_addr_2 + 4));
    dma_acl_rx_des[acl_rx_index].rx_des_acl_dw2.data_total_length = 0x10;
    dma_acl_rx_des[acl_rx_index].rx_des_acl_dw2.handle = 0xFFF;
    dma_acl_rx_des[acl_rx_index].rx_des_acl_dw2.pb_flag = 0x3;
    dma_acl_rx_des[acl_rx_index].rx_des_acl_dw2.bc_flag = 0x3;
    dma_acl_rx_des[acl_rx_index].rx_des_dw1.own = 1;

    RT_BT_LOG(YELLOW, EVENT_RX_DES_RING_BASE_LOG, 1, DMA_DWORD_READ(ACL_RX_DES_RING_BASE_REG));
    RT_BT_LOG(RED, EVT_RX_SRAM_CONTEXT, 1, (temp_addr_2 + 4));
    RT_BT_LOG(YELLOW, EVT_RX_SRAM_CONTEXT, 1, dma_acl_rx_des[acl_rx_index].rx_des_dw1_u32);
    RT_BT_LOG(YELLOW, EVT_RX_SRAM_CONTEXT, 1, dma_acl_rx_des[acl_rx_index].rx_des_dw2_u32);
    RT_BT_LOG(YELLOW, EVT_RX_SRAM_CONTEXT, 1, dma_acl_rx_des);
    RT_BT_LOG(YELLOW, EVT_RX_SRAM_CONTEXT, 1, (dma_acl_rx_des+acl_rx_index));
    //notify DMA that there is a valid acl packet
    DMA_DWORD_WRITE(CMD_TX_BUF_FREE_RX_START_CTL_REG, 0x40000000);
}
void acl_rx_test_step_2()
{
    UINT8 *temp_addr_2, j;
    UINT8 dbg_len;
    UINT32 buffer_index = 0;
    isr_index++;
    acl_rx_index++;
    if (acl_rx_index == (ACL_RX_DES_NUM-1))
    {
        acl_rx_index = 0;
    }
    RT_BT_LOG(YELLOW, DBG_RX_INDEX, 1, acl_rx_index);
    buffer_index = 16*(isr_index%4);
    temp_addr_2 = ((UINT32)temp_addr_1 | 0xa0000000);

//------------------------------------------------------------------------------
    RT_BT_LOG(BLUE, ISR_INDEX_CHECK, 1, isr_index);
//------------------------------------------------------------------------------
    dma_acl_rx_des[acl_rx_index].rx_des_dw1.payload_start_add= (0xFFFF & (UINT32) (temp_addr_2 + 4 + buffer_index));
    dma_acl_rx_des[acl_rx_index].rx_des_acl_dw2.data_total_length = 0x10;
    dma_acl_rx_des[acl_rx_index].rx_des_acl_dw2.handle = 0xFFF;
    dma_acl_rx_des[acl_rx_index].rx_des_acl_dw2.pb_flag = 0x3;
    dma_acl_rx_des[acl_rx_index].rx_des_acl_dw2.bc_flag = 0x3;
    dma_acl_rx_des[acl_rx_index].rx_des_dw1.own = 1;

    RT_BT_LOG(YELLOW, EVENT_RX_DES_RING_BASE_LOG, 1, DMA_DWORD_READ(ACL_RX_DES_RING_BASE_REG));
    RT_BT_LOG(YELLOW, EVT_RX_SRAM_CONTEXT, 1, dma_acl_rx_des[acl_rx_index].rx_des_dw1_u32);
    RT_BT_LOG(YELLOW, EVT_RX_SRAM_CONTEXT, 1, dma_acl_rx_des[acl_rx_index].rx_des_dw2_u32);
    RT_BT_LOG(YELLOW, EVT_RX_SRAM_CONTEXT, 1, dma_acl_rx_des);
    RT_BT_LOG(YELLOW, EVT_RX_SRAM_CONTEXT, 1, (dma_acl_rx_des+acl_rx_index));

    //notify DMA that there is a valid acl packet
    DMA_DWORD_WRITE(CMD_TX_BUF_FREE_RX_START_CTL_REG, 0x40000000);
}
#endif


#ifdef _RX_SCO_DBG_
void sco_rx_test_step_1()
{
    UINT8 *temp_addr_2, j;
    temp_addr_2 = ((UINT32)temp_addr_1 | 0xa0000000);
    temp_addr_2[0] = 0xFF;
    temp_addr_2[1] = 0xFF;
    temp_addr_2[2] = 0x18;
    for (j=0; j<64; j++)
    {
        temp_addr_2[3+j] = j;
    }
    for (j=0; j< 24; j++)
    {
        RT_BT_LOG(RED, _DEBUG_, 2,j, temp_addr_2[3+j]);
    }
    dma_sco_rx_des[evt_rx_index].rx_des_dw1.payload_start_add= (0xFFFF & (UINT32) (temp_addr_2 + 3));
    dma_sco_rx_des[evt_rx_index].rx_des_sco_dw2.data_total_length = 0x18 ;
    dma_sco_rx_des[evt_rx_index].rx_des_sco_dw2.handle = 0xFFF;
    dma_sco_rx_des[evt_rx_index].rx_des_sco_dw2.pkt_sta_flag = 0x3;
    dma_sco_rx_des[evt_rx_index].rx_des_sco_dw2.reserved_14_15 = 0x3;
    dma_sco_rx_des[evt_rx_index].rx_des_dw1.own = 1;

    RT_BT_LOG(YELLOW, EVENT_RX_DES_RING_BASE_LOG, 1, DMA_DWORD_READ(SCO_RX_DES_RING_BASE_REG));
    RT_BT_LOG(RED, EVT_RX_SRAM_CONTEXT, 1, (temp_addr_2 + 3));
    RT_BT_LOG(RED, EVT_RX_SRAM_CONTEXT, 1, temp_addr_2[0]);
    RT_BT_LOG(RED, EVT_RX_SRAM_CONTEXT, 1, temp_addr_2[1]);
    RT_BT_LOG(RED, EVT_RX_SRAM_CONTEXT, 1, temp_addr_2[2]);
    RT_BT_LOG(RED, EVT_RX_SRAM_CONTEXT, 1, temp_addr_2[3]);
    RT_BT_LOG(YELLOW, EVT_RX_SRAM_CONTEXT, 1, dma_sco_rx_des[evt_rx_index].rx_des_dw1_u32);
    RT_BT_LOG(YELLOW, EVT_RX_SRAM_CONTEXT, 1, dma_sco_rx_des[evt_rx_index].rx_des_dw2_u32);
}

void sco_rx_test_step_2()
{
    UINT8 *temp_addr_2, j;
    temp_addr_2 = ((UINT32)temp_addr_1 | 0xa0000000);
    sco_rx_index++;
    if (sco_rx_index == (SCO_RX_DES_NUM-1))
    {
        sco_rx_index = 0;
    }
    for (j=0; j< 24; j++)
    {
        RT_BT_LOG(RED, _DEBUG_, 2,j, temp_addr_2[3+j]);
    }
    RT_BT_LOG(YELLOW, DBG_RX_INDEX, 1, sco_rx_index);
    dma_sco_rx_des[sco_rx_index].rx_des_dw1.payload_start_add= (0xFFFF & (UINT32) (temp_addr_2 + 3));
    dma_sco_rx_des[sco_rx_index].rx_des_sco_dw2.data_total_length = 0x18 ;
    dma_sco_rx_des[sco_rx_index].rx_des_sco_dw2.handle = 0xFFF;
    dma_sco_rx_des[sco_rx_index].rx_des_sco_dw2.pkt_sta_flag = 0x3;
    dma_sco_rx_des[sco_rx_index].rx_des_sco_dw2.reserved_14_15 = 0x3;
    dma_sco_rx_des[sco_rx_index].rx_des_dw1.own = 1;

    RT_BT_LOG(YELLOW, EVENT_RX_DES_RING_BASE_LOG, 1, DMA_DWORD_READ(SCO_RX_DES_RING_BASE_REG));
    RT_BT_LOG(YELLOW, EVT_RX_SRAM_CONTEXT, 1, dma_sco_rx_des[sco_rx_index].rx_des_dw1_u32);
    RT_BT_LOG(YELLOW, EVT_RX_SRAM_CONTEXT, 1, dma_sco_rx_des[sco_rx_index].rx_des_dw2_u32);
    DMA_DWORD_WRITE(CMD_TX_BUF_FREE_RX_START_CTL_REG, 0x80000000);
}
#endif

#ifdef _TX_CMD_DBG_
void cmd_tx_test(UINT8* in_buf, UINT16 pkt_len)
{
    UINT16 bitmap, i;
    DMA_TX_DBG_LOG(BLUE, TX_CMD_PACKET_LENGTH, 1, pkt_len);
    DMA_TX_DBG_LOG(BLUE, TX_DES_INDEX, 1, cmd_tx_index);
    for (i=0; i<pkt_len; i++)
    {
        DMA_TX_DBG_LOG(WHITE, _DEBUG_, 2, i, in_buf[i]);
    }
    isr_index++;
//#ifdef _ENABLE_PCIE_DMA_
//	for (i=5; i<(pkt_len-1); i++) {
//#else
    for (i=3; i<(pkt_len-1); i++)
    {
//#endif
        if (in_buf[i]!=0xFF)
        {
            if (in_buf[i+1] != (in_buf[i] + 1))
            {
                RT_BT_LOG(RED, TX_CMD_PACKET_LENGTH, 1, pkt_len);
                RT_BT_LOG(RED, LOOPBACK_TEST_TX_ERROR, 1, i);
                debug_tx = 1;
            }
        }
    }
//-----------------------------------------
    if (debug_tx == 1)
    {
        for (i=0; i<pkt_len; i++)
        {
            RT_BT_LOG(RED, _DEBUG_, 2, i, in_buf[i]);
        }
    }
    debug_tx = 0;
//-------------------------------------------
    bitmap = cmd_tx_index;
#ifndef _TX_CMD_FIFO_DBG_
    if (debug_tx == 0)
        dma_tx_fifo_order_free(bitmap, HCI_TRANSPORT_CMD_PKT_TYPE);
#endif
}
#endif


#ifdef _TX_SCO_DBG_
void sco_tx_test(UINT8* in_buf, UINT16 pkt_len)
{
    UINT16 bitmap, i;
    DMA_TX_DBG_LOG(BLUE, TX_SCO_PACKET_LENGTH, 1, pkt_len);
    DMA_TX_DBG_LOG(BLUE, TX_DES_INDEX, 1, sco_tx_index);
    for (i=0; i<pkt_len; i++)
    {
        DMA_TX_DBG_LOG(WHITE, _DEBUG_, 2, i, in_buf[i]);
    }
    isr_index++;
    for (i=4; i<(pkt_len-1); i++)
    {
        if (in_buf[i+1] != (in_buf[i] + 1))
        {
            RT_BT_LOG(WHITE, LOOPBACK_TEST_TX_ERROR, 1, i);
            debug_tx = 1;
        }
    }
//-----------------------------------------
    if (debug_tx == 1)
    {
        for (i=0; i<pkt_len; i++)
        {
            RT_BT_LOG(WHITE, _DEBUG_, 2, i, in_buf[i]);
        }
    }
//-------------------------------------------
    bitmap = sco_tx_index;
#ifndef _TX_SCO_FIFO_DBG_
    if (debug_tx == 0)
        dma_tx_fifo_order_free(bitmap, HCI_TRANSPORT_SYNC_DATA_PKT_TYPE);
#endif
}
#endif

#ifdef _TX_ACL_DBG_
void acl_tx_test(UINT8* in_buf, UINT16 pkt_len)
{
    UINT16 bitmap, i;
    DMA_TX_DBG_LOG(BLUE, TX_PKT_LEN, 1, pkt_len);
    DMA_TX_DBG_LOG(BLUE, TX_DES_INDEX, 1, acl_tx_index);
    for (i=0; i<pkt_len; i++)
    {
        DMA_TX_DBG_LOG(WHITE, _DEBUG_, 2, i, in_buf[i]);
    }
    isr_index++;

    for (i=4; i<(pkt_len-1); i++)
    {
        if (in_buf[i]!=0xFF)
        {
            if (in_buf[i+1] != (in_buf[i] + 1))
            {
                RT_BT_LOG(RED, TX_CMD_PACKET_LENGTH, 1, pkt_len);
                RT_BT_LOG(RED, LOOPBACK_TEST_TX_ERROR, 1, i);
                debug_tx = 1;
            }
        }
    }
//-----------------------------------------
    if (debug_tx == 1)
    {
        for (i=0; i<pkt_len; i++)
        {
            RT_BT_LOG(RED, _DEBUG_, 2, i, in_buf[i]);
        }
    }
    debug_tx = 0;
//-------------------------------------------
    bitmap = acl_tx_index;
#ifndef _TX_ACL_FIFO_DBG_
    if (debug_tx == 0)
        dma_tx_fifo_order_free(bitmap, HCI_TRANSPORT_ACL_DATA_PKT_TYPE);
#endif
}
#endif

#ifdef _TX_LE_ACL_DBG_
void le_acl_tx_test(UINT8* in_buf, UINT16 pkt_len)
{
    UINT16 bitmap, i;
    DMA_TX_DBG_LOG(BLUE, TX_CMD_PACKET_LENGTH, 1, pkt_len);
    DMA_TX_DBG_LOG(BLUE, TX_DES_INDEX, 1, le_acl_tx_index);
    for (i=0; i<pkt_len; i++)
    {
        DMA_TX_DBG_LOG(WHITE, _DEBUG_, 2, i, in_buf[i]);
    }
    isr_index++;

    for (i=4; i<(pkt_len-1); i++)
    {
        if (in_buf[i]!=0xFF)
        {
            if (in_buf[i+1] != (in_buf[i] + 1))
            {
                RT_BT_LOG(RED, TX_CMD_PACKET_LENGTH, 1, pkt_len);
                RT_BT_LOG(RED, LOOPBACK_TEST_TX_ERROR, 1, i);
                debug_tx = 1;
            }
        }
    }    

//-----------------------------------------
    if (debug_tx == 1)
    {
        for (i=0; i<pkt_len; i++)
        {
            RT_BT_LOG(RED, _DEBUG_, 2, i, in_buf[i]);
        }
    }
    debug_tx = 0;
//-------------------------------------------
    bitmap = le_acl_tx_index;
#ifndef _TX_LE_ACL_FIFO_DBG_
    if (debug_tx == 0)
        dma_tx_fifo_order_free(bitmap, HCI_TRANSPORT_LE_ACL_DATA_PKT_TYPE);
#endif
}
#endif


#ifdef _SUPPORT_POLLING_BASED_LPM_L1_
BOOLEAN checkRxDescAllFree(UINT8 type)
{

    if(type == CHECK_BY_CNT)
    {

        if((dma_man.evt_available_num != (EVT_RX_DES_NUM-1))
            ||(dma_man.acl_available_num != (ACL_RX_DES_NUM-1))
            ||(dma_man.sco_available_num != (SCO_RX_DES_NUM-1)))
        {
#ifdef _SUPPORT_USB_LOG_ENABLE_
            if(g_usb_misc&BIT0)
            {
                RT_BT_LOG(WHITE, FONGPIN_DMA_DESC_FREE_CNT, 6,
                dma_man.evt_available_num,EVT_RX_DES_NUM,
                dma_man.acl_available_num,ACL_RX_DES_NUM,
                dma_man.sco_available_num,SCO_RX_DES_NUM);//9, 17,17
            }
#endif        
            return FALSE;
        }
    }// if(type == 0)
    else
    {
        UINT8 i;
        for (i = 0; i < (EVT_RX_DES_NUM - 1); i++)
        {
            if( dma_evt_rx_des[i].rx_des_dw1.own != 0)
            {
                return FALSE;
            }
        }
        for (i = 0; i < (ACL_RX_DES_NUM - 1); i++)
        {   
            if( dma_acl_rx_des[i].rx_des_dw1.own != 0)
            {
                return FALSE;
            }         
        }
        for (i = 0; i < (SCO_RX_DES_NUM - 1); i++)
        {
            if( dma_sco_rx_des[i].rx_des_dw1.own != 0)
            {
                return FALSE;
            }        
        }
    }
    return TRUE;

    
}

void set_lpm_l1_token_reponse(UINT8 response)
{
    //RT_BT_LOG(WHITE, YL_DBG_HEX_1, 1,response);

    if((response != L1_NYET)&&(response != L1_ACK))
    {
        response = L1_NYET;
        RT_BT_LOG(RED, FONGPIN_USB_SET_L1_PARA_ERROR, 1,response);
        
    }

    BTON_USB_LPM_REG_S_TYPE bton_USB_LPM_reg;
    bton_USB_LPM_reg.d32 = VENDOR_READ(BTON_USB_LPM);
    bton_USB_LPM_reg.b.LPM_Allow = response;
    VENDOR_WRITE(BTON_USB_LPM, bton_USB_LPM_reg.d32);
}

UINT8 get_lpm_l1_token_reponse(void)
{
    BTON_USB_LPM_REG_S_TYPE bton_USB_LPM_reg;
    bton_USB_LPM_reg.d32 = VENDOR_READ(BTON_USB_LPM);
    return bton_USB_LPM_reg.b.LPM_Allow;
}

#endif /* #ifdef _SUPPORT_POLLING_BASED_LPM_L1_ */

#ifdef _SUPPORT_NEW_USB_LPM_L1_DATA_PATH_150629_
UINT8 hci_dma_usb_check_usb_lpm_l1_then_queue_notification(UINT8 pkt_type)
    {
#ifdef _ROM_CODE_PATCHED_
    if (rcp_hci_dma_usb_check_usb_lpm_l1_then_queue_notification != NULL)
        {
        return rcp_hci_dma_usb_check_usb_lpm_l1_then_queue_notification(&pkt_type);
        }
#endif         
            
#ifdef _ALLOW_ENTER_USB_LPM_L1_VIA_HW_TIMER2_
    /* Turn off hw timer2 */
    timer_on_off(TIMER2_ID, 0, 0); 
    g_usb_lpm_l1_hw_timer_is_running = 0;
#endif

    /* check any rx desc queues of hci dma is pending or not */
    if (g_usb_lpm_l1_hci_dma_notification_queue_bm)
    {
        g_usb_lpm_l1_hci_dma_notification_queue_bm |= 1 << pkt_type;
        return TRUE;
    }


#if 0 /* mark this to avoid asynchonrous operation (austin) */
    /* check current lpm l1 token */
    if (get_lpm_l1_token_reponse() == L1_NYET)
    {
        return FALSE;
    }
#endif
    
    /* update new lpm l1 token response to usb sie */
    set_lpm_l1_token_reponse(L1_NYET);


    /* check current lpm l1 state of usb bus */ 
    if (is_usb_lpm_l1_state())
    {
        /* does host allow remote wakeup ? */
        if (usb_read_L1_remote_wakeup_feature())
        {
            usb_remote_wakeup();
        }
        
        /* must queue the packet */
        g_usb_lpm_l1_hci_dma_notification_queue_bm |= 1 << pkt_type;        
        
        return TRUE;
    }

    return FALSE; 
}

UINT8 hci_dma_usb_check_and_allow_enter_usb_lpm_l1(void)
{
#ifdef _ROM_CODE_PATCHED_
    if (rcp_hci_dma_usb_check_and_allow_enter_usb_lpm_l1 != NULL)
    {
        return rcp_hci_dma_usb_check_and_allow_enter_usb_lpm_l1(NULL);
    }
#endif

    /* never enter lpm l1 when any sco or esco links are created !! */
    if ((lmp_self_device_data.total_no_of_sco_conn > 0) ||
       (lmp_self_device_data.number_of_esco_connections > 0))
    {
        return FALSE;
    }

    /* check hci dma rx desc is free or not ? */    
    if (checkRxDescAllFree(CHECK_BY_OWN))
    {
        /* check current sie endpoint buffer is empty now ? */
        USB_SIE_0x52_REG_S_TYPE u8_sie_ep_sts;
        u8_sie_ep_sts.d8 = (UINT8)safe_indirect_read_sie(READ_SIE_BYTE, SIE_BUFF_OK_REG);//3 
        if (u8_sie_ep_sts.d8 & 0x07)
        {
            /* bit 0,1,2 mean ep0,1,2 buf ok.
               when any bit is TRUE, it means the data is using by USB SIE !! */
            return FALSE;   
        }

#ifdef _ALLOW_ENTER_USB_LPM_L1_VIA_HW_TIMER2_
        /* Turn on hw timer2 to check again after 3 ms */
        UINT8 expire_ms = otp_str_data.USB_LPM_Control >> 3;

        if (!g_usb_lpm_l1_hw_timer_is_running)
        {
            timer_on_off(TIMER2_ID, expire_ms*1000, TRUE); 
            g_usb_lpm_l1_hw_timer_is_running = 1;
        }        
#else 
        /* update new lpm l1 token response to usb sie */
        set_lpm_l1_token_reponse(L1_ACK);
#endif
        return TRUE;
    }
    return FALSE;
}

UINT8 hci_dma_usb_check_and_dequeue_notification(void)
{
    UINT8 bmqueue;

#ifdef _ROM_CODE_PATCHED_
    if (rcp_hci_dma_usb_check_and_dequeue_notification != NULL)
    {
        return rcp_hci_dma_usb_check_and_dequeue_notification(NULL);
    }
#endif

    /* check current lpm l1 state of usb bus is in lpm l1 or not */     
    if (is_usb_lpm_l1_state())
    {
        return FALSE;
    }

    if (g_usb_lpm_l1_hci_dma_notification_queue_bm)
    {
        /* re-operate any pending notification to hci dma */ 

        bmqueue = g_usb_lpm_l1_hci_dma_notification_queue_bm;
        g_usb_lpm_l1_hci_dma_notification_queue_bm = 0;

        if (bmqueue & (1 << HCI_TRANSPORT_ACL_DATA_PKT_TYPE))
        {
            //notify DMA that there is a valid event packet
            DMA_DWORD_WRITE(CMD_TX_BUF_FREE_RX_START_CTL_REG, ACL_NOTIFY_HW);            
        }
        if (bmqueue & (1 << HCI_TRANSPORT_SYNC_DATA_PKT_TYPE))
        {
            //notify DMA that there is a valid event packet
            DMA_DWORD_WRITE(CMD_TX_BUF_FREE_RX_START_CTL_REG, SCO_NOTIFY_HW);        
        }
        if (bmqueue & (1 << HCI_TRANSPORT_EVENT_PKT_TYPE))
        {
            //notify DMA that there is a valid event packet
            DMA_DWORD_WRITE(CMD_TX_BUF_FREE_RX_START_CTL_REG, EVT_NOTIFY_HW);        
        }

        return TRUE;
    }
    else
    {        
        hci_dma_usb_check_and_allow_enter_usb_lpm_l1();
        return FALSE;
    }
}
#endif

