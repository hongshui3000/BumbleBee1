/***************************************************************************
 Copyright (C) RealTek Ltd.
 This module is a confidential and proprietary property of RealTek and
 a possession or use of this module requires written permission of RealTek.
 ***************************************************************************/

enum { __FILE_NUM__= 213 };

#include "mailbox.h"
#include "bt_fw_os.h"
#include "lc_internal.h"
#include "otp.h"
#include "mem.h"
#include "lmp_ch_assessment.h"
#include "pta.h"
#include "hci_vendor_defines.h"
#include "lc.h"
#include "timer.h"
#include "h5.h"
#include "pta_meter.h"
#include "le_hw_reg.h"
#include "lc_phy_init.h"
#ifdef _BT_ONLY_
#include "new_io.h"
#endif
#ifdef MWS_ENABLE
#include "mws_imp.h"
#include "mws.h"
#endif
#include "le_hci_4_0.h"
#include "uart.h"

#ifdef _ENABLE_MAILBOX_
UINT32 mailbox_reg_base_addr[8];
UINT32 mailbox_reg_offset = 0;
UINT32 mailbox_reg_value = 0;
UINT16 mailbox_reg_w_value = 0;
UINT32 *mailbox_sram_addr = NULL;
UINT8 mailbox_rom_code_page_mode = 0;
UINT16 mailbox_rom_code_len = 0;
UINT16 mailbox_page_count = 0;
UINT8 g_wifi_ch = 0;
UINT8 mailbox_out_data_busy = 0;
UINT32 mailbox_count = 0;
OS_HANDLE mailbox_handle;
LMP_CONNECTION_ENTITY *ceptr = NULL;
UINT8 bBtPsdMode=3;
UINT8 bHIDenable=1;
UINT8 bHIDCount=0;
UINT8 bBtLNAConstraint=0;
UINT8 a2dp_rate=1;

#ifdef MWS_ENABLE
UINT8 wlan_off_to_on_flag = 0;
UINT8 wlan_on_to_off_flag = 0;
TIMER_ID mws_hand_shake_timer1 = OS_INVALID_HANDLE;
#endif

#ifdef _ROM_CODE_PATCHED_
PF_ROM_CODE_PATCH_FUNC rcp_mailbox_handle_command = NULL;
PF_ROM_CODE_PATCH_FUNC rcp_mailbox_interrupt_handler = NULL;
PF_ROM_CODE_PATCH_FUNC rcp_mailbox_handle_mp_extra_command = NULL;
#endif
extern UCHAR rf_transmit_test_params[11];

#ifdef _IS_ASIC_
extern INT8 g_rtl8723_btrf_txpower_track_extra_delta;
extern void rtl8723_btfr_SetTxGainTable(INT8 RFGainDiff);
extern void rtl8723_btrf_TxPowerTrack(INT8 N_new);
#endif

#ifdef _ANTENNA_DETECTION_
TimerHandle_t hTimerIDsingletone;
void set_le_tx_mode_parameter(UINT16 slave_tx_power, UINT16 adv_tx_power);
void mailbox_handle_start_le_packet_tx(UINT8 rf_channel , UINT8 payload_len , UINT8 payload_pattern);
void mailbox_handle_stop_le_packet_rx(TimerHandle_t timer);
#endif

#ifdef _IQK_HANDSHAKE_FLOW_
UINT8 g_iqk_flow;
void iqk_flow_when_wifi_alive(void);
void iqk_flow_when_wifi_off(void);
void iqk_mailbox_notify_wifi_timeout(TimerHandle_t timer);
UINT16 iqk_result_x[3] = {0};
UINT16 iqk_result_y[3] = {0};
TimerHandle_t iqk_mode_wait_response_timer = NULL;
#endif
extern OS_HANDLE  isr_extended_task_handle;

#ifdef _SUPPORT_MAILBOX_MP_COMMAND_
MAILBOX_WIFI_MP_MISC_PARAM_S patch_mp_misc_param;
#endif
#ifdef MWS_ENABLE
TIMER_ID mws_coex_hand_shake_timer = OS_INVALID_HANDLE;
UINT8	mws_flag_wlan_off2on_when_bt_on = 0;
UINT8	mws_flag_wlan_on2off_when_bt_on = 0;
#define MWS_HANDSHAKE_TIMEOUT	250
#endif
#ifdef MWS_ENABLE
#include "mws.h"
#include "mws_imp.h"
#ifdef MWS_POLL_UART_IDLE
#define MWS_NUM_POLL_MAX 200
#define MWS_POLL_TIMEOUT_LOG1() RT_BT_LOG(RED, LOG_MWS_POLL_TIMEOUT1, 0, 0)
#define MWS_WAIT_FOR_UART_IDLE() \
    do {\
        int n = 1;\
        while (!(mws_read_register_new(LTE_COEX_STATUS_REG)&0x3))\
        {\
            if (n > MWS_NUM_POLL_MAX)\
            {\
                MWS_POLL_TIMEOUT_LOG1();\
                break;\
            }\
            ++n;\
        }\
    } while (0)
#else
#define MWS_WAIT_FOR_UART_IDLE() do {} while (!(mws_read_register_new(LTE_COEX_STATUS_REG)&0x3))
#endif

#endif

#ifdef _A2DP_CHECK_QUEUE_TDMA_MODE_
extern HCI_ACL_DATA_PKT *acl_q_head;
#define NUMBER_OF_A2DP_PKT_IN_Q	0
void check_a2dp_pkt_inQ(void);
#endif

void mailbox_init()
{
    /* only enable interrupt of in mailbox data ready */
    VENDOR_WRITE(MAILBOX_CTL_REG, BIT17);

#ifdef _ACTIVE_CHECK_WIFI_ON_OFF_
    check_wifi_alive();
#endif

    mailbox_reg_base_addr[0] = BB_BASE_ADDR;
    mailbox_reg_base_addr[1] = BT_DMA_REG_BASE_ADDR;
    mailbox_reg_base_addr[2] = TIMER_BASE_ADDR;
    mailbox_reg_base_addr[3] = VENDOR_BASE_ADDRESS;
    mailbox_reg_base_addr[4] = GPIO_BASE_ADDRESS;
    mailbox_reg_base_addr[5] = UART_BASE_ADDR;
    mailbox_reg_base_addr[6] = BT_DMA_UART_BASE_ADDRESS;
    mailbox_reg_base_addr[7] = (UINT32) &otp_str_data;

    memset(&patch_mp_misc_param, 0, sizeof(MAILBOX_WIFI_MP_MISC_PARAM_S));

    //---Create Mailbox task---
    if(OS_CREATE_TASK( &mailbox_handle, MAILBOX_TASK_NAME,
                   MAILBOX_TASK_PRI,
                   (OS_TASK_FUNCTION)mailbox_task,
                   MAILBOX_TASK_Q_SIZE,
                   NAILBOX_TASK_BUSY_PERIOD) != BT_ERROR_OK)
    {
    }
}

UINT8 pf_os_trigger_mailbox_task(UINT16 type, UINT32 data_1, UINT32 data_2)
{
    OS_SIGNAL signal;
    MAILBOX_BUF_S_TYPE *buf;

#ifdef _ACTIVE_CHECK_WIFI_ON_OFF_
    if (check_wifi_alive() == FALSE)
    {
        // dape marked requested by baron.
    	//RT_BT_LOG(GREEN,YL_DBG_DEC_1,1,3);
        return FALSE;
    }
#endif

    signal.type = type;
    buf = (MAILBOX_BUF_S_TYPE *) &signal;

    buf->buf_1 = data_1;
    buf->buf_2 = data_2;
    // dape marked requested by baron.
    //RT_BT_LOG(GREEN,YL_DBG_DEC_1,1,4);
    OS_SEND_SIGNAL_TO_TASK(mailbox_handle, signal);
    // dape marked requested by baron.
    //RT_BT_LOG(GREEN,YL_DBG_DEC_1,1,5);
    return TRUE;
}

void mailbox_reset_task(void)
{
    DEF_CRITICAL_SECTION_STORAGE;

    MINT_OS_ENTER_CRITICAL();

    /* write 1 to clear interrupt of In mailbox data ready */
    VENDOR_WRITE(MAILBOX_CTL_REG, BIT16);

    /* enable interrupt of In mailbox data ready */
    VENDOR_WRITE(MAILBOX_CTL_REG, BIT17);

    OS_reset_task(mailbox_handle);

    MINT_OS_EXIT_CRITICAL();
}

void mailbox_task_check(void)
{
    if (mailbox_out_data_busy)
    {
        mailbox_count++;

        do
        {
#ifdef _ACTIVE_CHECK_WIFI_ON_OFF_
            if (check_wifi_alive())
            {
                if (mailbox_count <= otp_str_data.mailbox_max_timeout)
                {
                    break;
                }
            }
#else
            if (mailbox_count <= otp_str_data.mailbox_max_timeout)
            {
                break;
            }
#endif

            mailbox_count = 0;

            //Clear mailbox task and reset mailbox out queue
            //RT_BT_LOG(RED, DMA_SOFT_TEST_COMPLETE, 0, 0);
            mailbox_out_data_busy = 0;

            mailbox_reset_task();
        }
        while (0);
    }
}

void mailbox_handle_wifi_patch_cmd_stage(UINT32 *wdata)
{
    mailbox_sram_addr = (UINT32 *)wdata[1];
    mailbox_page_count = 0;

    if ((UINT32)mailbox_sram_addr & 0x03)
    {
        /* must 4 byte alignment */
        return;
    }

#ifndef IS_BTSOC
#ifdef _ROM_CODE_PATCHED_
    /* (austin) because our page start address must be changed after 8821b_mp,
       so we add one workaround flow to avoid that wifi fw can set old start
       page address */
    if ((UINT32)mailbox_sram_addr == 0x80106000)
    {
        /* change old start address to new start address of patch memory */
        mailbox_sram_addr = (UINT32 *)ROM_CODE_PAGE_START_ADDRESS;
    }

    if ((UINT32)mailbox_sram_addr < ROM_CODE_PAGE_START_ADDRESS)
    {
        /* invalid address */
        return;
    }
#endif
#endif

    mailbox_rom_code_len = (wdata[0] >> 16);

    if (mailbox_rom_code_len == 0)
    {
        /* invalid length */
        return;
    }

    mailbox_rom_code_page_mode = 1;
}


void mailbox_handle_wifi_patch_data_stage(UINT32 *wdata)
{
    if (mailbox_rom_code_len > 8)
    {
        mailbox_rom_code_len -= 8;
    }
    else
    {
        mailbox_rom_code_len = 0;
        mailbox_rom_code_page_mode = 0;
    }

    if (!IS_ROM_CODE_PATCHED) /* do not accpet patch again after POR */
    {
        /* we do not need to care 8 byte or not - austin */
        mailbox_sram_addr[mailbox_page_count++] = wdata[0];
        mailbox_sram_addr[mailbox_page_count++] = wdata[1];

        if (mailbox_rom_code_len == 0)
        {
           SIGN_ROM_CODE_PATCH_SIGNATURE;
           SIGN_FW_TRIG_WDG_TO_SIGNATURE;
#ifdef _YL_RTL8723A_B_CUT
           if (g_fun_interface_info.b.bt_interface == UART_INTERFACE)
           {
               UINT32 uart_status_d32 = (UINT32)UART_DWORD_READ(DMA_UART_H5_INTSTS);
               SIGN_PATCH_UART_STATUS(uart_status_d32);
           }
#endif
           WDG_TIMER_TIMEOUT_SOON;
           while (1); /* infinite loop here to wait watch dog timeout */
        }
    }
}

void mailbox_interrupt_handler(void)
{
    MAILBOX_CTL_INFO_TYPE mailbox_ctl_reg;
    UINT8 update_count =  0;
    UINT32 wdata[2] = {0, 0};

    mailbox_ctl_reg.d32 = VENDOR_READ(MAILBOX_CTL_REG);

#ifdef _ROM_CODE_PATCHED_
    if (rcp_mailbox_interrupt_handler != NULL)
    {
        if (rcp_mailbox_interrupt_handler(&mailbox_ctl_reg.d32))
        {
            return;
        }
    }
#endif

    if (mailbox_ctl_reg.b.in_ready && mailbox_ctl_reg.b.in_ready_int_en)
    {
        /* write 1 (IN_READY) to clean interrupt */
        wdata[0] = VENDOR_READ(MAILBOX_IN_DATA0_REG);
        wdata[1] = VENDOR_READ(MAILBOX_IN_DATA1_REG);
        update_count |= BIT0;
        //RT_BT_LOG(WHITE,YL_DBG_HEX_2,2,wdata[0],wdata[1]);
    }

    //if (mailbox_out_data_busy &&  mailbox_ctl_reg.b.out_empty_int_en)
    /* dape mark mailbox_out_data_busy since this parameter will be cleared
       in HCI_Reset and this will cause interrupt triggered before HCI_Reset not be
       cleared which will cause Watchdog reset. */
    if (mailbox_ctl_reg.b.out_empty_int_en)
    {
        if (!mailbox_ctl_reg.b.out_ready)
        {
            /* out mailbox is ready */
            mailbox_out_data_busy = 0;

            /* disable out empty interrupt to avoid enter ISR infinite */
            mailbox_ctl_reg.b.out_empty_int_en = 0;
            update_count |= BIT1;
        }
    }

    if (update_count)
    {
#ifdef _ACTIVE_CHECK_WIFI_ON_OFF_
        is_wifi_alive = TRUE;
#endif

        mailbox_ctl_reg.b.out_ready = 0; /* avoid to notify wifi again before
                                            out data is ready */
        VENDOR_WRITE(MAILBOX_CTL_REG, mailbox_ctl_reg.d32);

        if (update_count & BIT0)
        {
            if (mailbox_rom_code_page_mode)
            {
                mailbox_handle_wifi_patch_data_stage(wdata);
            }
            else
            {
                if ((wdata[0] & 0x3F) == 0x0A) /* wifi patch command */
                {
                    mailbox_handle_wifi_patch_cmd_stage(wdata);
                }
                else
                {
                    pf_os_trigger_mailbox_task(MAILBOX_READ, wdata[0], wdata[1]);
                }
            }
        }
    }
}

UCHAR mailbox_task(OS_SIGNAL *signal_ptr)
{
    MAILBOX_CTL_INFO_TYPE mailbox_ctl_reg;
    MAILBOX_BUF_S_TYPE *buf;

    buf = (MAILBOX_BUF_S_TYPE *) signal_ptr;

    DEF_CRITICAL_SECTION_STORAGE;

    MINT_OS_ENTER_CRITICAL();

    switch(signal_ptr->type)
    {
        case MAILBOX_READ:
            mailbox_handle_command(&buf->buf_1);
            break;

        case MAILBOX_WRITE:
            if (!mailbox_out_data_busy)
            {
                mailbox_ctl_reg.d32 = VENDOR_READ(MAILBOX_CTL_REG);
                mailbox_count = 0;
                mailbox_out_data_busy = 1;

                VENDOR_WRITE(MAILBOX_OUT_DATA0_REG, buf->buf_1);
                VENDOR_WRITE(MAILBOX_OUT_DATA1_REG, buf->buf_2);
                mailbox_ctl_reg.b.out_ready = 1;
                mailbox_ctl_reg.b.out_empty_int_en = 1;
                VENDOR_WRITE(MAILBOX_CTL_REG, mailbox_ctl_reg.d32);
            }
            else
            {
                pf_os_trigger_mailbox_task(MAILBOX_WRITE, buf->buf_1, buf->buf_2);
            }
            break;
        default:
            /* Not the registered signal */
            break;
    }

    MINT_OS_EXIT_CRITICAL();

    return BT_FW_SUCCESS;
}

UINT32 mailbox_read_write_register(UINT8 type, UINT8 is_write, UINT32 reg_address)
{
    UINT32 read_value = 0;
    if (!type)
    {
        /* 32 bit access */
        if (mailbox_reg_offset & 0x03)
        {
            /* invalid access */
            return 0xDEADBEEF;
        }

        if (is_write)
        {
            RTK_VENDOR_DW_WRITE(reg_address, mailbox_reg_offset, mailbox_reg_value);
        }
        else
        {
            read_value = RTK_VENDOR_DW_READ(reg_address, mailbox_reg_offset);
        }
    }
    else
    {
        /* 16 bit access */
        if (mailbox_reg_offset & 0x01)
        {
            /* invalid access */
            return 0xDEAD;
        }

        if (is_write)
        {
            RTK_VENDOR_W_WRITE(reg_address, mailbox_reg_offset, mailbox_reg_value);
        }
        else
        {
            read_value = RTK_VENDOR_W_READ(reg_address, mailbox_reg_offset);
        }
    }
    return read_value;
}

// TODO: It is recommened to use mailbox_read_write_rf_modem_register_pi()
UINT32 mailbox_read_write_rf_modem_register(UINT8 addr, UINT8 type,
                                            UINT32 value, UINT8 is_write)
{
    UINT32 read_value = 0;
    if (is_write)
    {
        rtk_write_modem_radio_reg(addr, type, value);
    }
    else
    {
        read_value = rtk_read_modem_radio_reg(addr, type);
    }
    return read_value;
}

#ifdef _NEW_MODEM_PI_ACCESS_
UINT32 mailbox_read_write_modem_register_pi(UINT8 modem_page, UINT8 addr,
                                            UINT32 value, UINT8 is_write)
{
    UINT32 read_value = 0;
    if (is_write)
    {
//        rtk_write_modem_radio_reg_pi(modem_page, addr, type, value);
        RTK_WRITE_MODEM_REG_PI(modem_page, addr, value);
    }
    else
    {
//        read_value = rtk_read_modem_radio_reg_pi(modem_page, addr, type);
        read_value = RTK_READ_MODEM_REG_PI(modem_page, addr);
    }
    return read_value;
}
#endif


#ifdef _SUPPORT_MAILBOX_MP_COMMAND_
void patch_mbox_mp_handle_wifi_cmmads(UINT8 *data)
{
    MAILBOX_WIFI_MP_CMD_S *pMpW2B;
    UINT8 tmp;
    UINT32 wr_dword = 0;
    UINT8 mp_status = MP_MBOX_ERROR_CODE_OK;
    UINT8 mp_len = 0;

    UINT8 rd_mode;

    pMpW2B = (MAILBOX_WIFI_MP_CMD_S *) data;
    tmp = pMpW2B->rd_data[0];
    rd_mode = pMpW2B->rd_data[1];

    switch (pMpW2B->sub_id)
    {
    case MCMD30_MP_BT_OP_GET_BT_CAPABILITY_REQ:                
        wr_dword = fw_lmp_sub_version | (PATCH_MP_FW_VERSION << 16);
        mp_len = 3;
        break;

	case MCMD30_MP_BT_OP_WR_REG_VALUE_REQ:
        patch_mp_misc_param.reg_value_w =
                pMpW2B->rd_data[0] | (pMpW2B->DWord[1] & 0xffff) << 8;
        patch_mp_misc_param.reg_w_state = 1;
        break;

    case MCMD30_MP_BT_OP_WR_REG_ADDR_REQ:
    case MCMD30_MP_BT_OP_RD_REG_REQ:
        {
            UINT16 addr;
            UINT32 value = 0;
            UINT8 new_page;
            UINT8 is_wr;

            mp_status = MP_MBOX_ERROR_CODE_WRONG_PARAMETERS;

            if (tmp > 4)
            {
                break;
            }

            addr = pMpW2B->DWord[1] & 0xffff;

            if (pMpW2B->sub_id == MCMD30_MP_BT_OP_WR_REG_ADDR_REQ)
            {
                if (!patch_mp_misc_param.reg_w_state)
                {
                    break;
                }

                is_wr = TRUE;
                value = patch_mp_misc_param.reg_value_w;
            }
            else
            {
                is_wr = FALSE;
                mp_len = 3;
            }

            switch (tmp)
            {
            case 0:
                /* RF (only 6-bit address valid) */
                if (is_wr)
                {
                    RTK_WRITE_RF_REG(addr & 0x3F, value);
                }
                else
                {
                    wr_dword = RTK_READ_RF_REG(addr & 0x3F);
                }
                mp_status = MP_MBOX_ERROR_CODE_OK;
                break;

            case 1:
                /* Modem,AFE (only 7-bit address valid) */
                new_page = (addr >> 8) & 0x01;
                if (new_page)
                {
                    /* switch page 1 */
                    RTK_WRITE_MODEM_REG(TRANS_MODEM_REG(0), new_page);
                }

                if (is_wr)
                {
                    RTK_WRITE_MODEM_REG(TRANS_MODEM_REG(addr & 0x7F), value);
                }
                else
                {
                    wr_dword = RTK_READ_MODEM_REG(TRANS_MODEM_REG(addr & 0x7F));
                }

                if (new_page)
                {
                    /* switch back page 0 */
                    RTK_WRITE_MODEM_REG(TRANS_MODEM_REG(0), 0);
                }
                mp_status = MP_MBOX_ERROR_CODE_OK;
                break;

            case 2:
                /* Bluewize Regsiter */
                if (addr & 0x01)
                {
                    break;
                }

                if (is_wr)
                {
                    BB_write_baseband_register(addr, value);
                }
                else
                {
                    wr_dword = BB_read_baseband_register(addr);
                }
                mp_status = MP_MBOX_ERROR_CODE_OK;
                break;

            case 3:
                /* Vendor Regsiter */
                if (addr & 0x01)
                {
                    break;
                }

                if (is_wr)
                {
                    WR_16BIT_IO(VENDOR_BASE_ADDRESS, addr, value);
                }
                else
                {
                    wr_dword = RD_16BIT_IO(VENDOR_BASE_ADDRESS, addr);
                }
                mp_status = MP_MBOX_ERROR_CODE_OK;
                break;

#ifdef LE_MODE_EN
            case 4:
                /* LE Register */
                if (addr & 0x01)
                {
                    break;
                }

                if (is_wr)
                {
                    WR_LE_REG(addr, value);
                }
                else
                {
                    wr_dword = RD_LE_REG(addr);
                }
                mp_status = MP_MBOX_ERROR_CODE_OK;
                break;
#endif

#ifdef _BT_ONLY_
            case 5:
                /* Page 0 Register */
                if (addr & 0x01)
                {
                    break;
                }

                if (is_wr)
                {
                    WR_16BIT_SYSON_IO(addr, value);
                }
                else
                {
                    wr_dword = RD_16BIT_SYSON_IO(addr);
                }
                mp_status = MP_MBOX_ERROR_CODE_OK;
                break;
#endif
            default:
                break;
            }
        }
        break;

// 20120911 morgan add for AFH map
        case MCMD30_MP_BT_OP_GET_AFH_MAP_L:
        case MCMD30_MP_BT_OP_GET_AFH_MAP_M:
        case MCMD30_MP_BT_OP_GET_AFH_MAP_H:
        {
            UINT8 piconet_id = 0; // piconet_id is an input parameter
            UINT8 ce_index;
            LMP_CONNECTION_ENTITY *ce_ptr = NULL;
            UINT8 *ptr_tmp = NULL;

            piconet_id = tmp;
            //ceptr = report_AFH_map_low_word(piconet_id, (UCHAR*)&wr_dword);
            //lmp_get_any_afh_enabled_connections(0)
            wr_dword = 0;
            /*
            0: AFH map result
            1: Wifi psd only
            2: Wifi chnl BW only
            3: BT psd only
            4: Host classification only
            */

            switch(rd_mode)
            {
            case 0:	// report AFH map result
                if (pMpW2B->sub_id == MCMD30_MP_BT_OP_GET_AFH_MAP_L)
                {
                    for (ce_index = 0; ce_index < LMP_MAX_CE_DATABASE_ENTRIES; ce_index++)
                    {
                        ce_ptr = &lmp_connection_entity[ce_index];
                        if(ce_ptr->phy_piconet_id == piconet_id)
                        {
                            // real afh_map
                            ptr_tmp = ce_ptr->afh_map;
                            ceptr = ce_ptr;
                            break;
                        }
                        else
                        {
                            ceptr = NULL;
                        }
                    }
                }
                else
                {
                    if ( ceptr != NULL )
                    {
                        //report_AFH_map_middle_word(ceptr, (UCHAR*)&wr_dword);
                        ptr_tmp = ceptr->afh_map;
                    }
                }
                break;

            case 1:	// report   Wifi psd only
                ptr_tmp = afh_map_psd;
                break;

            case 2:	// report Wifi chnl BW only
                ptr_tmp = afh_wifi_set_ch_bw;
                break;

            case 3:	// report  BT psd only
                ptr_tmp = &afh_bt_psd_map[piconet_id][0];
                break;

            case 4:	// report Host classification only
                ptr_tmp = afh_host_classification;
                break;

            default:
                break;

            }

            if (ptr_tmp != NULL)
            {
                if (pMpW2B->sub_id == MCMD30_MP_BT_OP_GET_AFH_MAP_L)
                {
                    wr_dword = ( ptr_tmp[0] | (ptr_tmp[1]<<8) | (ptr_tmp[2]<<16) | (ptr_tmp[3]<<24) );
                    mp_len = 4;
                }
                else if (pMpW2B->sub_id == MCMD30_MP_BT_OP_GET_AFH_MAP_M)
                {
                    wr_dword = ptr_tmp[4] | (ptr_tmp[5]<<8) | (ptr_tmp[6]<<16) | (ptr_tmp[7]<<24);
                    mp_len = 4;
                }
                else
                {
                    wr_dword = ptr_tmp[8] | (ptr_tmp[9]<<8);
                    mp_len = 2;
                }
            }
        }
        break;

    case MCMD30_MP_BT_OP_GET_AFH_STATUS:
        if (ceptr != NULL)
        {
            wr_dword = ( ceptr->afh_map[8] | (ceptr->afh_map[9]<<8) );
        }
	break;
#ifdef _ANTENNA_DETECTION_
	case MCMD30_MP_BT_OP_SET_ANT_DETECTION:
		{
			mp_len = 1;
			pMpW2B = (MAILBOX_WIFI_MP_CMD_S *) data;
			UINT8 txperiod;
			UINT8 btchnl;

			if(lc_check_lps_for_idle(1))
			{
				txperiod = pMpW2B->rd_data[0];
				btchnl = pMpW2B->rd_data[1];
                set_le_tx_mode_parameter(0x2b,0x2b);
                lc_start_write_scan_mode(0);
				mailbox_handle_start_le_packet_tx(btchnl,0x25,0x2);

				if (hTimerIDsingletone != NULL)
		        {
		            OS_DELETE_TIMER(&hTimerIDsingletone);
		        }
		        OS_CREATE_TIMER(ONESHOT_TIMER, &hTimerIDsingletone,
		                mailbox_handle_stop_le_packet_rx, NULL, 0);
				OS_START_TIMER(hTimerIDsingletone, txperiod);
                wr_dword = 0x1; //can tx now
			}
			else
			{
				wr_dword = 0x0; //cant tx now
			}
		}
			break;
#endif

    default:
#ifdef _ROM_CODE_PATCHED_
        if (rcp_mailbox_handle_mp_extra_command != NULL)
        {
            if (rcp_mailbox_handle_mp_extra_command(data))
            {
                return;
            }
        }
#endif

        mp_status = MP_MBOX_ERROR_CODE_UNKNOWN_OPCODE;
        break;
    }

    pMpW2B->status = (mp_status & 0x0f) | (mp_len << 4);
    pMpW2B->seq &= 0xf0;
    pMpW2B->seq |= PATCH_MP_FW_VERSION;
    pf_os_trigger_mailbox_task(MAILBOX_WRITE, pMpW2B->DWord[0], wr_dword);
}
#endif

void mailbox_handle_command(UINT32* in_data)
{
    UINT32 temp_reg_addr = 0;
    UINT8 modem_rf_addr = 0;
    UINT32 out_data1, out_data2;
    UINT16 wdata;
    UINT32 dw_value = 0;
    PKT_HEADER_TYPE pkt_header;

#ifdef _ROM_CODE_PATCHED_
    if (rcp_mailbox_handle_command != NULL)
    {
        if (rcp_mailbox_handle_command((void *)in_data))
        {
            return;
        }
    }
#endif

    pkt_header.d8 = in_data[0] & 0xff;


#ifdef _NEW_MODEM_PI_ACCESS_
    if ( (pkt_header.b.cmd_type < 10) || (pkt_header.b.cmd_type == MODEM_REGISTER_CMD_PI))
#else
    if (pkt_header.b.cmd_type < 10)
#endif
    {
        mailbox_reg_offset = in_data[0]>>16;
        mailbox_reg_value = in_data[1];
        modem_rf_addr = (UCHAR) (mailbox_reg_offset & 0xFF);

        //MAILBOX_LOG(WHITE,MAILBOX_CMD_ID,2,pkt_header.b.cmd_type, pkt_header.b.is_write);
        //MAILBOX_LOG(WHITE,LMP_PAYLOAD_INFO,1,in_data[0]);
        //MAILBOX_LOG(WHITE,LMP_PAYLOAD_INFO,1,mailbox_reg_offset);

        if (pkt_header.b.cmd_type == MODEM_REGISTER_CMD)
        {
            dw_value = mailbox_read_write_rf_modem_register(modem_rf_addr, TYPE_MODEM,
                                        mailbox_reg_value, pkt_header.b.is_write);
        }
        else if (pkt_header.b.cmd_type == RF_REGISTER_CMD)
        {
            dw_value = mailbox_read_write_rf_modem_register(modem_rf_addr, TYPE_RF,
                                        mailbox_reg_value, pkt_header.b.is_write);
        }
#ifdef _NEW_MODEM_PI_ACCESS_
        else if (pkt_header.b.cmd_type == MODEM_REGISTER_CMD_PI)
        {
            UINT8 modem_page = (UINT8) ( (mailbox_reg_offset>>8) & 0xFF);
            dw_value = mailbox_read_write_modem_register_pi(modem_page, modem_rf_addr,
                                        mailbox_reg_value, pkt_header.b.is_write);
        }
#endif
        else
        {
            temp_reg_addr = mailbox_reg_base_addr[pkt_header.b.cmd_type];
            dw_value = mailbox_read_write_register(pkt_header.b.is_16b,
                                        pkt_header.b.is_write, temp_reg_addr);
        }

        if (!pkt_header.b.is_write)
        {
            pkt_header.b.is_write = 1;

            out_data1 = (in_data[0] & 0xffffff00) | pkt_header.d8;
            out_data2 = dw_value;

            //MAILBOX_LOG(BLUE,MAILBOX_READ_VALUE,1, dw_value);
            pf_os_trigger_mailbox_task(MAILBOX_WRITE, out_data1, out_data2);
        }
    }
    else
    {
        switch (pkt_header.b.cmd_type)
        {
#ifdef _IQK_HANDSHAKE_FLOW_
		case BT_IQK_MODE:
		{
            if( in_data[0] & BIT16)
            {
                g_iqk_flow = IQK_HS_FLOW_SUCCESS;
                if (iqk_mode_wait_response_timer != NULL)
                {
                    OS_DELETE_TIMER(&iqk_mode_wait_response_timer);
                }
                execute_iqk_lok();
                pf_os_trigger_mailbox_task(MAILBOX_WRITE, 0x0134, 0x00); // notify wifi that bt iqk finish
            }
        }
#endif
        case WIFI_CHANNEL_AND_BANDWIDTH_CMD:
            {
                WIFI_INFO_DW1_TYPE wifi_info_dw1;
                WIFI_INFO_DW2_TYPE wifi_info_dw2;

                wifi_info_dw1.d32 = in_data[0];
                wifi_info_dw2.d32 = in_data[1];

                if (wifi_info_dw1.b.wifi_connect)
                {
                    if (wifi_info_dw2.b.ch_mask_num > 51)
                        wifi_info_dw2.b.ch_mask_num = 50;

                    if (wifi_info_dw2.b.ch_mask_num < 20)
                        wifi_info_dw2.b.ch_mask_num = 20;

                    g_wifi_ch = wifi_info_dw1.b.wifi_ch;
                    MAILBOX_LOG(BLUE,WIFI_INFO, 3, wifi_info_dw1.b.wifi_ch,
                                                   wifi_info_dw2.b.wifi_bw,
                                                   wifi_info_dw2.b.ch_mask_num);
                }
                else
                {
                    g_wifi_ch = 0;
                    MAILBOX_LOG(RED,WIFI_DISCONNECT,0,0);
                }
                //----- add by neil chen ---2011---04---19--
                lmp_wifi_host_irq(wifi_info_dw1.d32,wifi_info_dw2.d32);
    	        //--------------------------------------
            }

            break;
        case WIFI_CL_EN:
            hci_vendor_cmd_generate_event(HCI_VENDOR_WIFI_CL_EN, 0, NULL);
            break;

        case WIFI_FORCE_TX_POWER_CMD:
#ifdef _IS_ASIC_
            {
                UINT8 decade_level;
                INT8 start;
                decade_level = ((in_data[0]>>16)&0xFF);
                if (otp_str_data.EFuse_TxPowerTrack_En &&
                    (otp_str_data.EFuse_ThermalUpdateInterval > 0))
                {
                    INT8 n_cur = g_rtl8723_btrf_txpower_track_n_old;
                    g_rtl8723_btrf_txpower_track_extra_delta = decade_level;
                    g_rtl8723_btrf_txpower_track_n_old = 0x55;
                    rtl8723_btrf_TxPowerTrack(n_cur);
                }
                else
                {
                    start = (-1) * decade_level;
                    rtl8723_btfr_SetTxGainTable(start);
                }
                MAILBOX_LOG(BLUE, WIFI_ASSIGN_TX_POWER, 1, decade_level);
            }
#endif
            break;

        case BT_ENABLE_IGNORE_WLAN_ACT_CMD:
            wdata = ((in_data[0]>>16)&0xFF) ? 0x220 : 0x200;
            BB_write_baseband_register(0x1A6, wdata);
            RT_BT_LOG(BLUE, CTRL_BT_IGNORE_WLAN_ACT, 1, wdata);
            break;

        case ENABLE_ISOLATION_MEASURE:
            if (in_data[0]>>16)
            {
                //power level
                rf_transmit_test_params[0] = ((in_data[0]>>24) & 0xFF);
                //BT channel
                rf_transmit_test_params[1] = ((in_data[1]) & 0xFF);
                //Pattern
                rf_transmit_test_params[2] = 0x1;
                //packet type
                rf_transmit_test_params[3] = 0xf;
                //packet length
                rf_transmit_test_params[4] = 0xf;
                rf_transmit_test_params[5] = 0;
                //hop enable
                rf_transmit_test_params[6] = 1;
                //test type
                rf_transmit_test_params[7] = 1;
                //amaddr
                rf_transmit_test_params[8] = 1;
                //burst length
                rf_transmit_test_params[9] = 0xFF;

                hci_set_vendor_rf_transmit_test_config(rf_transmit_test_params);

                hci_start_rf_transmit_test();
            }
            else
            {
                hci_stop_rf_transmit_test();
            }
            break;

        case WIFI_RESET_BT_CMD:
            if (((in_data[0]>>16)&0xFF))
            {
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
                while(1);
            }
            break;

		case BT_REPORT_CONN_SCO_INQ_INFO:
			mailbox_bt_report_info(BT_REPORT_CONN_SCO_INQ_INFO);
			break;

#ifdef _SUPPORT_MAILBOX_MP_COMMAND_
        case MB_MP_CMD_GROUP:
            patch_mbox_mp_handle_wifi_cmmads((UINT8*)in_data);
            break;
#endif

        case BT_LOOPBACK:
            pf_os_trigger_mailbox_task(MAILBOX_WRITE, in_data[0], in_data[1]);
            MAILBOX_LOG(BLUE,MAILBOX_BT_LOOPBACK,2,in_data[0],in_data[1]);
            break;
#ifdef MWS_ENABLE
		case MWS_CONTROL_INFO:
			{
				//Byte0:3A , name:MWS_CONTROL_INFO , Byte1:length = 6
				out_data1 = 0x063A;
				out_data2 = 0;
				switch(in_data[0]>>16)
				{
					case 0x3:
					{
						if(wlan_off_to_on_flag == 0)
						{
							MWS_WAIT_FOR_UART_IDLE();
							mws_write_register(LTECOEX_CTRL_REG,mws_read_register_new(LTECOEX_CTRL_REG)&(~BIT7),0xf);
							wlan_off_to_on_flag = 1;
							pf_os_trigger_mailbox_task(MAILBOX_WRITE, out_data1||(0x6)<<16, out_data2);
							OS_CREATE_TIMER((UCHAR)ONESHOT_TIMER, &mws_hand_shake_timer1,
                   			 (OS_TIMEOUT_HANDLER)mws_hand_shake_timer,(void *)NULL, 0);

    						RT_BT_LOG(YELLOW, MWS_HAND_SHAKE_DONE2, 0,0);
   							OS_START_TIMER(mws_hand_shake_timer1, 300);
						}
						else
						{
							RT_BT_LOG(WHITE,MWS_HAND_SHAKE_DONE,0,0);
							OS_DELETE_TIMER(&mws_hand_shake_timer1);
							wlan_off_to_on_flag = 0;
						}
					}
					break;
					case 0x01:
					{
						mws_write_register(LTECOEX_CTRL_REG,mws_read_register_new(LTECOEX_CTRL_REG)|BIT7,0xf);
						pf_os_trigger_mailbox_task(MAILBOX_WRITE, out_data1||(0x4)<<16, out_data2);
					}
				}
			}
			break;
#endif

#ifdef _BT_ONLY_
        case I2C_ENABLE:
            if (((in_data[0] >> 16) & 0x01) == 0x01)
            {
                pf_os_trigger_mailbox_task(MAILBOX_WRITE, in_data[0], in_data[1]);
                // enable auto report
                // add an gloabl variable
                pta_meter_var.bI2cEnable= 1;
            }
            else
            {
                //disable auto report
                pta_meter_var.bMailboxBtAutoReport = 0;
                // reset I2C
                // write system on area enable I2C
                UINT32 pta_i2c_enable = 0;

                // DISABLE
                pta_i2c_enable = (RD_32BIT_SYSON_IO(0x40)) | 0x00000000;
                //indirect_access_write_syson_reg
                WR_32BIT_SYSON_IO(0x40,pta_i2c_enable);

                // ENABLE
                //indirect_access_read_syson_reg(0x42,0);
                pta_i2c_enable = (RD_32BIT_SYSON_IO(0x40)) | 0x00800000;
                //indirect_access_write_syson_reg
                WR_32BIT_SYSON_IO(0x40,pta_i2c_enable);
            }
            break;
#endif

        default:
            MAILBOX_LOG(RED,MAILBOX_DATA_ERROR,0,0);
            break;

        }
    }
    return;
}

void mailbox_bt_report_info(UINT8 mailbox_cmd_idx)
{
    UINT8 bConnection = 0;
    UINT8 bSCOeSCO = 0;
    UINT8 bInQPage = 0;
    UINT8 bACLConn = 0;
    UINT8 bSCOBusy = 0;
    UINT8 bHID = 0;
    UINT8 bA2DP = 0;
    UINT8 bFTP = 0;
    UINT32 out_data1;
    UINT32 out_data2;
    UINT16 retry_count;
	UINT16 tmp_cnt = 0;

#ifdef SUPPORT_A2DP_RATE_REPORT
    a2dp_rate = 1;
#endif

#ifdef PTA_EXTENSION_DEBUG_LOG
    RT_BT_LOG(GREEN, PTA_DISPLAY_BT_INFO_MESSAGE2,0,0);
#endif

    DEF_CRITICAL_SECTION_STORAGE;

    MINT_OS_ENTER_CRITICAL();

    if (  lmp_self_device_data.number_of_acl_conn >0 )
    {
        bConnection = 1;
        if  ((pta_meter_var.dwPtaACLTxCnt + pta_meter_var.dwPtaACLRxCnt+ pta_meter_var.dwPtaACLTxCntStore + pta_meter_var.dwPtaACLRxCntStore) > 0x05 ) // means ACL has transport
        {
            bACLConn = 1;
        }
    }

    if ((lmp_self_device_data.number_of_esco_connections > 0) ||
            (lmp_self_device_data.total_no_of_sco_conn > 0))
    {
        bSCOeSCO = 1;
        bConnection = 1;
        if ((pta_meter_var.dwPtaSCOTxCnt + pta_meter_var.dwPtaSCORxCnt + pta_meter_var.dwPtaSCOTxCntStore + pta_meter_var.dwPtaSCORxCntStore) > 0x05 )
        {
            bSCOBusy = 1;
        }
    }

    if ( lmp_self_device_data.lc_cur_dev_state != 0 )
    {
        bInQPage = 1;
    }

#ifndef NO_CAPTURE_PROFILE_INFO
#ifdef BT_DRIVER_PROVIDE_PROFILE_INFO
    if (!g_host_set_profile)
#endif
    {
        UINT16 bCeIndex;

        if ( pta_meter_var.HID_Exist )
        {
            bHID = 1;
            if (LMP_GET_CE_INDEX_FROM_CONN_HANDLE(pta_meter_var.HID_connection_handle,&bCeIndex) != API_SUCCESS )
            {
                // no connection exist
                pta_meter_var.HID_Exist = 0;
                pta_meter_var.HID_connection_handle = 0xFFFF;
                BB_write_baseband_register( 0x190, 0x0000);
                bHID = 0;
                bHIDCount = 0;
            }
        }

        //pta_meter_var.A2DP_Exist handle by A2DP estimation
        bA2DP = pta_meter_var.A2DP_Exist;

        if ( bA2DP == 1 )
        {
            if (LMP_GET_CE_INDEX_FROM_CONN_HANDLE(pta_meter_var.A2DP_connection_handle,&bCeIndex) != API_SUCCESS )
            {
                // no connection exist
                pta_meter_var.A2DP_Exist = 0;
                pta_meter_var.A2DP_connection_handle = 0xFFFF;
                bA2DP = 0;
            }
            else
            {
#ifdef SUPPORT_A2DP_RATE_REPORT
                if (lmp_connection_entity[bCeIndex].feat_page0[3] & (BIT1 | BIT2))
                {
                    a2dp_rate = 0;
                }
#endif
            }
        }
        // FTP
        bFTP = pta_meter_var.FTP_Exist;
    }
#ifdef BT_DRIVER_PROVIDE_PROFILE_INFO
    else
    {
        if (bConnection == 0)
        {
            g_driver_report_hid_state = 0;
            g_driver_report_ftp_state = 0;
            g_driver_report_a2dp_state = 0;
        }
        bHID = g_driver_report_hid_state;
        bFTP = g_driver_report_ftp_state;
        bA2DP = g_driver_report_a2dp_state;
    }
#endif
#endif

    retry_count = pta_meter_var.dwTXRetryCntStore;

    MINT_OS_EXIT_CRITICAL();

    out_data1 = ( (retry_count << 24) | (bConnection |(bSCOeSCO << 1) | (bInQPage << 2 ) | (bACLConn << 3 ) | ( bSCOBusy << 4) | ( bHID <<5 )|(bA2DP <<6) |(bFTP << 7) )<<16)| (0x400) /* length = byte1*/ | (mailbox_cmd_idx);

    if (bt_afh_rssi.rssi_pkts == 0)
    {
        out_data2 = bt_afh_rssi.rssi_count;
    }
    else
    {
        out_data2 = bt_afh_rssi.rssi_count/bt_afh_rssi.rssi_pkts;
    }
    out_data2 &= 0xff;

#ifdef SUPPORT_A2DP_RATE_REPORT
    out_data2 |= (a2dp_rate << 8);
#endif

#ifdef _SUPPORT_REPORT_RESTART_
	// BYTE 1
	// bit0 : a2dp_rate
	// bit1 : need AFH map
	// bit2 : BT LNA constraint
	if(pta_meter_var.ReInit_flag )
	{
		//RT_BT_LOG(GRAY,AFH_NEED_WIFI_PSD,1,0);
		out_data2 |= 0x200;
		pta_meter_var.ReInit_flag=0;
	}
	//20120913 morgan add throughput report
	tmp_cnt = ((pta_meter_var.dwPtaACLRxCntStore+pta_meter_var.dwPtaACLTxCntStore) >> 3 );
	out_data2 |= (tmp_cnt << 16);
	out_data2 |= (( bBtLNAConstraint << 2 )<<8); // LNA constraint status
    bEnableIgnoreWlanActive = (BB_read_baseband_register(0x1A6) == 0x220) ? 1 : 0;
	out_data2 |= (( bEnableIgnoreWlanActive<< 3 )<<8); // Enable ignore wlan activw status
	out_data2 |= (( pta_meter_var.bMailboxBtAutoReport<< 4 )<<8); // Mailbox Bt auto report status
#endif

#ifdef MAILBOX_EXTENSION_DEBUG_LOG
    RT_BT_LOG(BLUE, PTA_DISPLAY_BT_INFO_MESSAGE3, 12, bConnection, bSCOeSCO, bInQPage, bACLConn, bSCOBusy,retry_count, bHID, bA2DP, bFTP, out_data2, bt_afh_rssi.rssi_count, bt_afh_rssi.rssi_pkts);
#endif

    pf_os_trigger_mailbox_task(MAILBOX_WRITE, out_data1, out_data2);
}


void mailbox_set_afh_map(UINT8 wifi_ch, UINT8 *p_map)
{
}
#endif

#ifdef _ANTENNA_DETECTION_
void set_le_tx_mode_parameter(UINT16 slave_tx_power, UINT16 adv_tx_power)
{
	LE_REG_S_SET ll_reg;
    /* set default power to maximum */
    ll_reg.value = RD_LE_REG(LE_REG_SLAVE_CONN_WIN_SIZE);
    ll_reg.slave_win_size.slave_tx_power = slave_tx_power;
    WR_LE_REG(LE_REG_SLAVE_CONN_WIN_SIZE, ll_reg.value);

    /* set advertising channel tx power */
    ll_reg.value = RD_LE_REG(LE_REG_CBK_CONTROL);
    ll_reg.cbk_ctrl.adv_tx_power = adv_tx_power;
    WR_LE_REG(LE_REG_CBK_CONTROL, ll_reg.value);
}
void mailbox_handle_start_le_packet_tx(UINT8 rf_channel , UINT8 payload_len , UINT8 payload_pattern)
{
	HCI_CMD_PKT hci_cmd_local;
	/* create le transmitter test command parameters */
	hci_cmd_local.cmd_opcode = HCI_LE_TRANSMITTER_TEST_OPCODE;
	hci_cmd_local.param_total_length = 3;
	hci_cmd_local.cmd_parameter[0] = rf_channel;
	hci_cmd_local.cmd_parameter[1] = payload_len;
	hci_cmd_local.cmd_parameter[2] = payload_pattern;
	hci_handle_le_transmitter_test(&hci_cmd_local);
}
void mailbox_handle_stop_le_packet_rx(TimerHandle_t timer)
{
    HCI_CMD_PKT hci_cmd_local;
    /* create le test end command parameters */
    hci_cmd_local.cmd_opcode = HCI_LE_TEST_END_OPCODE;
    hci_cmd_local.param_total_length = 0;

    set_le_tx_mode_parameter(0x2b,0x2b);

    hci_handle_le_test_end(&hci_cmd_local);
    lc_start_write_scan_mode(lmp_self_device_data.scan_enable);
}
#endif
#ifdef _A2DP_CHECK_QUEUE_TDMA_MODE_
void check_a2dp_pkt_inQ(void)
{
    HCI_ACL_DATA_PKT *pkt;
	pkt = acl_q_head;
	UINT8 q_count = 0;
	UINT8 a2dp_count = 0;
	while(pkt != NULL)
	{
		q_count ++ ;
		if( pkt->connection_handle == pta_meter_var.A2DP_connection_handle )
		{
			a2dp_count++;
		}
        pkt = pkt->next;
	}
	if( a2dp_count == NUMBER_OF_A2DP_PKT_IN_Q )
	{
		pf_os_trigger_mailbox_task(MAILBOX_WRITE, 0x010100|BT_A2DP_EMPTY, 0x0);
	}
}
#endif
#ifdef _IQK_HANDSHAKE_FLOW_
/* 
	this function is to handle IQK handshake when wifi alive by
	setup a 150 ms timer after wifi detected alive
*/
void iqk_flow_when_wifi_alive(void)
{
    UINT32 dw0 = 0;
    dw0 = (BT_IQK_MODE | (0x01<<8 ) | (0x01<<16));
    pf_os_trigger_mailbox_task(MAILBOX_WRITE, dw0, 0x00);
    if (iqk_mode_wait_response_timer != NULL)
    {
        OS_DELETE_TIMER(&iqk_mode_wait_response_timer);
    }
    OS_CREATE_TIMER(ONESHOT_TIMER, &iqk_mode_wait_response_timer,
            iqk_mailbox_notify_wifi_timeout, NULL, 0);
    OS_START_TIMER(iqk_mode_wait_response_timer, 150);
}
/* 
	this function is to handle IQK handshake when wifi not alive
	fill the bulletin 0x35AD0000 , after finishing iqk flow , fill 0x0.
*/
void iqk_flow_when_wifi_off(void)
{
	VENDOR_WRITE(MAILBOX_OUT_DATA0_REG, 0x35AD0000);
	g_iqk_flow = NO_IQK_HS_FLOW_CAL;
	execute_iqk_lok();
	VENDOR_WRITE(MAILBOX_OUT_DATA0_REG, 0x0);
}
/* 
	iqk handshake flow timer , when send 0x34 to wifi , if wifi respond
	in 150 ms , iqk flow will be finish and success .
	if timeout , double check wifi if alive , if yes , use default value 
	                                          if not , bt iqk
*/
void iqk_mailbox_notify_wifi_timeout(TimerHandle_t timer)
{
    if (iqk_mode_wait_response_timer != NULL)
    {
        OS_DELETE_TIMER(&iqk_mode_wait_response_timer);
    }
	if (g_iqk_flow == IQK_HS_FLOW_DEFAULT)
	{
#ifdef _ACTIVE_CHECK_WIFI_ON_OFF_
		if(check_wifi_alive())
#else
		if(0)
#endif
	    {
		    UINT16 RFC_LOK;
		    read_lok_txiqk_auto(&RFC_LOK, &iqk_result_x[0], &iqk_result_y[0]);
		    g_iqk_flow = IQK_HS_FLOW_FAIL;
		}
		else
		{
			iqk_flow_when_wifi_off();
		}
	}
}
#endif
