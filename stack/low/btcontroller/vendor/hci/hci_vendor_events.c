/***************************************************************************
 Copyright (C) MindTree Ltd.
 This module is a confidential and proprietary property of MindTree and
 a possession or use of this module requires written permission of MindTree.
 ***************************************************************************/

/********************************* Logger *************************/ 
enum { __FILE_NUM__= 35 };
/********************************* Logger *************************/
#include "platform.h"
#include "bt_fw_hci_internal.h"
#include "hci_vendor_defines.h"
#include "hci_vendor_internal.h"
#include "vendor.h"
#include "bz_auth.h"
#include "mem.h"
#include "spi.h"
#include "lbm.h"
#include "crypto.h"
#include "h5.h"
#include "uart.h"

#ifdef _BT_ONLY_
#include "new_io.h"
#include "efuse.h"
#endif

#ifdef _PLC_TEST_MODE_ENABLE_
#include "plc.h"
#endif

#include "fm.h"

#if defined(RT_VENDOR_CMDS)
extern UINT16 hci_vendor_g_read_address;
extern UINT8 hci_vendor_g_read_len;
extern UCHAR rf_transmit_test_params[11];
extern UCHAR hci_vendor_g_read_radio_address;
#ifdef _NEW_MODEM_PI_ACCESS_
extern UCHAR hci_vendor_g_read_radio_modem_page;
#endif
#ifndef IS_BTSOC
extern UINT8 rom_code_patch_block_index;
#endif
extern UINT32 hci_vendor_spi_address;
extern UINT16 hci_vendor_spi_len;

extern UINT8 dbg_vendor_log_interface;
extern UINT16 dbg_vendor_log_conn_handle;
extern UINT8 *dbg_vendor_log_buf;

extern UINT32 g_hci_vendor_read_data;
extern UINT8 g_hci_vendor_read_data_len;
extern UINT16 g_hci_vendor_read_sie_data;
extern UINT8 g_hci_vendor_read_sie_data_len;


#ifdef _SUPPORT_MSFT_BT_HCI_EXTENSION_
extern UINT16 g_hci_vs_msft_sub_opcode;
extern UINT8 g_hci_vs_msft_le_monitor_handle;
extern UINT8 g_msft_event_prefix[8];
#endif

#ifdef _ROM_CODE_PATCHED_
PF_ROM_CODE_PATCH_FUNC rcp_hci_vendor_generate_cmd_complete_func = NULL;
#endif

#ifdef _CCH_RTL8723A_B_CUT
// _CCH_ISSC_VENDOR_
extern ISSC_SEC_STRUCT issc_sec;
#endif

#define HCI_COMMAND_COMPLETE_EVT_SET_STATUS(evt, status) \
    ((evt)->event_parameter[3] = (status))
#define HCI_COMMAND_COMPLETE_EVT_GET_STATUS(evt) ((evt)->event_parameter[3])

#ifdef BT2FM_INDIRECT_ACCESS
extern UINT16 g_fm_rd_addr;
extern UINT8 g_fm_rd_len;
void hci_vendor_gen_fm_read_cmd_cplt_evt(HCI_EVENT_PKT *evt,
        UINT8 *param_len)
{
    UINT8 pos = *param_len;
    UINT8 i;
    if (HCI_COMMAND_COMPLETE_EVT_GET_STATUS(evt) != HCI_COMMAND_SUCCEEDED)
    {
        return;
    }
    for (i = 0; i < g_fm_rd_len; ++i)
    {
        UINT8 data;
        if (!fm_read_register(g_fm_rd_addr + i, &data))
        {
            HCI_COMMAND_COMPLETE_EVT_SET_STATUS(evt, HARDWARE_FAILURE_ERROR);
            // doesn't change param_len to discard all data read.
            return;
        }
        evt->event_parameter[pos] = data;
        ++pos;
    }
    *param_len += g_fm_rd_len;
}
#endif

/**************************************************************************
 * Function   :  hci_vendor_generate_command_complete_event
 *
 * Description:
 *      This function generates a Command Complete Event that is used by the
 *      Host Controller to transmit return status of a command and returns
 *      other event parameters that are specified for the  HCI command. Note
 *		that this function generates Command Complete for ONLY 
 *		vendor specific commands. 
 *
 * Parameters :
 *
 *      UINT16 command_opcode :
 *          Opcode of the command for which the Command Complete Event
 *          is generated.
 *
 *      UCHAR status :
 *          Status that is required to be sent to the upper layers for
 *          the command issued.
 *
 *
 * Returns    :
 *      HCI_EVENT_PKT *   : Pointer to an HCI event packet
 *
 *************************************************************************/
 HCI_EVENT_PKT * hci_vendor_generate_command_complete_event(UINT16 hci_cmd_opcode,
												   UCHAR cmd_status, void *arg)
{
    HCI_EVENT_PKT *hci_event_pkt_ptr ;
	UCHAR param_length;
	API_RESULT	result;

    result = OS_ALLOC_BUFFER(
				tx_table[HCI_EVENT_HANDLER_TASK].pool_handle,
				(OS_ADDRESS *)&hci_event_pkt_ptr);

	if( result != BT_ERROR_OK)
	{
		BT_FW_HCI_ERR(UNABLE_TO_ALLOCATE_MEMORY_FROM_EVENT_BUFFER_POOL,0,0);
		return NULL;
	}

    hci_event_pkt_ptr->event_opcode = HCI_COMMAND_COMPLETE_EVENT ;

    /* Num HCI comamnd from self dev datbase */
    if (hci_cmd_opcode == 0x00)
	{
        hci_event_pkt_ptr -> event_parameter[0] = 0x00 ;
	}
    else
	{
        hci_event_pkt_ptr -> event_parameter[0] =
                            lmp_self_device_data.num_hci_command_packets ;
	}
    hci_event_pkt_ptr->event_parameter[1] = LSB(hci_cmd_opcode);
    hci_event_pkt_ptr->event_parameter[2] = MSB(hci_cmd_opcode);
    hci_event_pkt_ptr->event_parameter[3] = cmd_status ;
    param_length = 4;

#ifdef _ROM_CODE_PATCHED_
    /* TODO: ROM CODE PATCH HERE if required */
    if (rcp_hci_vendor_generate_cmd_complete_func != NULL)
    {
        if (rcp_hci_vendor_generate_cmd_complete_func(
                           (void*)hci_event_pkt_ptr, &param_length, arg))
        {
            hci_event_pkt_ptr->param_total_length  = param_length;

            return hci_event_pkt_ptr;
        }
    }
#endif  

    switch(hci_cmd_opcode)
	{
#ifdef _REPORT_CRC_ERR_PKT_IN_LE_RX_TEST_MODE
        case HCI_VENDOR_READ_LE_RX_TEST_PKT:
            /*RT_BT_LOG(BLUE, DAPE_TEST_LOG522, 2,
                ll_manager.test_unit.num_of_rx_pkts,
                ll_manager.test_unit.num_of_rx_pkts_crc_err);*/
            hci_event_pkt_ptr->event_parameter[param_length] = ll_manager.test_unit.num_of_rx_pkts & 0xFF;
            hci_event_pkt_ptr->event_parameter[param_length+1] = ll_manager.test_unit.num_of_rx_pkts >> 8;
            hci_event_pkt_ptr->event_parameter[param_length+2] = ll_manager.test_unit.num_of_rx_pkts_crc_err & 0xFF;
            hci_event_pkt_ptr->event_parameter[param_length+3] = ll_manager.test_unit.num_of_rx_pkts_crc_err >> 8;
    
            param_length+=4;
        
            break;
#endif

#ifndef IS_BTSOC
#ifdef _ROM_CODE_PATCHED_
        case HCI_VENDOR_DOWNLOAD:
            hci_event_pkt_ptr->event_parameter[param_length] = 
                                            rom_code_patch_block_index;
            param_length++;
            break;
#endif
#endif
    
        case HCI_VENDOR_SET_ASSOC:      
            {            
                UINT8 index_c;         
                UINT8 i;
                UINT8 *ptr = lmp_self_device_data.rtk_assoc_data;
                
                index_c = lc_generate_rand_number(15);
                hci_event_pkt_ptr->event_parameter[param_length] = index_c ^ 0xAA;                
                param_length++;

                for (i = 0; i < 16; i++)
                {
                    hci_event_pkt_ptr->event_parameter[param_length] = 
                                  lmp_self_device_data.security_key[i] ^ 0xAA;
                    param_length++;
                }

                /* generate ans_c [k] */
                for (i = 0; i < 6; i++)
                {
                    ptr[i] = lmp_self_device_data.security_key[index_c];                
                    ptr[i] ^= ~otp_str_data.bt_bd_addr[i];                    
                    index_c = (index_c + 1) & 0x0F;             
                }
            }
            break;
            
        case HCI_VENDOR_GET_ASSOC:
            memcpy(&hci_event_pkt_ptr->event_parameter[param_length],
                    lmp_self_device_data.rtk_assoc_data, 12);
            param_length += 12;
            break;

/*	Added by Wallice for USB LPM HCI Vendor command.	2012/03/19	*/
	    case HCI_VENDOR_USB_LPM_CTRL:

		//*(unsigned int *)((unsigned char *)(hci_event_pkt_ptr->event_parameter) + param_length) = VENDOR_READ(BTON_USB_LPM);
		//param_length += 0x04;				
		break;
/*	End Added by Wallice for USB LPM HCI Vendor command.	2012/03/19	*/


        case HCI_VENDOR_8703B_BTGPIO_LOG_ENABLE:
            break;


        case HCI_VENDOR_SET_LOG_ENABLE:            
            dbg_vendor_set_log_complete_event_buf = (UINT8*)hci_event_pkt_ptr;
            break;

#ifdef _DAPE_TEST_NEW_HW
        case HCI_VENDOR_SET_MUTE_ENABLE:
            break;
#endif

#ifdef _SPIC_FUNC_ENABLE_
        case HCI_VENDOR_SPI_INIT:
        case HCI_VENDOR_SPI_ERASE:
        case HCI_VENDOR_SPI_WRITE:
            break;
            
        case HCI_VENDOR_SPI_READ:
            {
                UINT32 addr = hci_vendor_spi_address;
                UINT8 len = hci_vendor_spi_len;

                /* fill reserved field */
                hci_event_pkt_ptr->event_parameter[4] = 0;
                hci_event_pkt_ptr->event_parameter[5] = 0;
                param_length = 6;

                /* fill spi read data field */
                while (len >= 12)
                {                    
                    spi_read_data(addr, 12, 
                            &hci_event_pkt_ptr->event_parameter[param_length]);
                    len -= 12;
                    addr += 12;
                    param_length += 12;
                }

                if (len > 0)
                {
                    spi_read_data(addr, len, 
                            &hci_event_pkt_ptr->event_parameter[param_length]); 
                    param_length += len;
                }
            }
            break;
#endif
    
#ifdef _YL_H5_ISSC_WARM_RST    
        case HCI_VENDOR_ISSC_WARM_RESET:
            break;
#endif            

        case HCI_VENDOR_WRITE:
            break;

        case HCI_VENDOR_READ:
            {
                UINT8 i;
                UINT32 temp = g_hci_vendor_read_data;
                for (i = 0; i < g_hci_vendor_read_data_len; i++)
                {
                    hci_event_pkt_ptr->event_parameter[param_length] = temp;
                    param_length++;
                    temp >>= 8;
                }
                
            }  
            break;

#ifdef _SUPPORT_VENDER_READ_SIE_
        case HCI_VENDOR_READ_SIE:
            {

                UINT8 i;
                UINT32 temp = g_hci_vendor_read_sie_data;                
                for (i = 0; i < g_hci_vendor_read_sie_data_len; i++)
                {
                    hci_event_pkt_ptr->event_parameter[param_length] = temp;
                    param_length++;
                    temp >>= 8;
                }
            }
            break;
#endif

        case HCI_VENDOR_WRITE_BB_REGISTER:
            break;

        case HCI_VENDOR_READ_BB_REGISTER:
            {
                UINT16 address;
                UINT16 val = 0;
                address = hci_vendor_g_read_address;

                /* page 0: Bluewiz (0xb6000000)
                   page 1: HCI DMA (0xb4000000)
                   page 2: Timer (0xb0004000)
                   page 3: GPIO (0xb0006000)
                   page 4: UART (0xb0000000)
                   page 5: H-UART (0xb5000000)
                   page 6: BZDMA (0xb000a000)
                   page 7: LE (0xb6001000)                   
                   page 8: Vendor (0xb000a000) */
                   
                UINT8 page = address >> 12;
                UINT16 reg_offset = address & 0xFFF;
                UINT32 base;

                if (reg_offset & 0x01)
                {
                    val =  0xDEAD;
                    base = 0;
                    //RT_BT_LOG(RED, MSG_HCI_VENDOR_CMDS_ERR, 2, page, reg_offset);
                }
                else
                {
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
                        {
                            base = 0xffffffff;
                            UINT32 read_offset;
                            
                            switch (reg_offset>>8)
                            {
                                case 0: // read the status of modem sram debug //
                                    hci_event_pkt_ptr->event_parameter[param_length] = 
                                            g_modem_sram_debug_en |
                                            (g_modem_sram_debug_log_en<<1) |
                                            (g_modem_sram_debug_captured_flag<<2) |
                                            (g_modem_sram_debug_xdh5_trig_en<<3) |
                                            (g_modem_sram_debug_xdh5_trig_crc_ok<<4) |
                                            (g_modem_sram_debug_le_trig_en<<5) |
                                            (g_modem_sram_debug_le_trig_crc_ok<<6) |
                                            (g_modem_sram_debug_xdh5_xdh3_3dh1_error_log_en<<7);
                                    param_length++;
                                    hci_event_pkt_ptr->event_parameter[param_length] = 0;
                                    param_length++;
                                    break;
                                case 1: // read report data and status//
                                    hci_event_pkt_ptr->event_parameter[param_length] = 
                                            g_modem_sram_debug_en |
                                            (g_modem_sram_debug_log_en<<1) |
                                            (g_modem_sram_debug_captured_flag<<2) |
                                            (g_modem_sram_debug_xdh5_trig_en<<3) |
                                            (g_modem_sram_debug_xdh5_trig_crc_ok<<4) |
                                            (g_modem_sram_debug_le_trig_en<<5) |
                                            (g_modem_sram_debug_le_trig_crc_ok<<6) |
                                            (g_modem_sram_debug_xdh5_xdh3_3dh1_error_log_en<<7);
                                    param_length++;
                                    hci_event_pkt_ptr->event_parameter[param_length] = 0;
                                    param_length++;
                                    read_offset = (reg_offset&0xFF) << 6; // address unit: 64 Bytes, LSB should be 0 ==> block size = 128 Bytes //
                                    if (g_modem_sram_debug_array != NULL)
                                    {
                                        memcpy((void *)&(hci_event_pkt_ptr->event_parameter[param_length]), 
                                                (void *)(((UINT8 *)g_modem_sram_debug_array)+read_offset), 128);
                                    }
                                    else
                                    {
                                        memset((void *)&(hci_event_pkt_ptr->event_parameter[param_length]), 
                                                0xff, 128);
                                    }
                                    param_length+=128;
                                    break;
                                case 2:
                                    break;
                                default:
                                    break;
                            }
                        }
                        break;
      #endif                
                    default:
                        base = BB_BASE_ADDR;
                        break;
                    }

                    if (base!=0xffffffff)
                    {
                        val = RD_16BIT_IO(base, reg_offset);
                    }
                }

                RT_BT_LOG(WHITE, MSG_HCI_VENDOR_CMDS_RD, 3, 
                                base, reg_offset, val);

                hci_event_pkt_ptr->event_parameter[param_length] = (UCHAR)val;
                param_length++;

                hci_event_pkt_ptr->event_parameter[param_length] = 
                                                        (UCHAR) (val >> 8) ;
                param_length++;

                break;
            }

#ifdef BT2FM_INDIRECT_ACCESS
        case HCI_VENDOR_FM_READ:
            hci_vendor_gen_fm_read_cmd_cplt_evt(hci_event_pkt_ptr,
                    &param_length);
            break;
#endif

        case HCI_VENDOR_READ_RF_TRANSMIT_TEST_CONFIG:
            memcpy(&(hci_event_pkt_ptr->event_parameter[param_length]),
                   rf_transmit_test_params, 11);
            param_length+=11;
            break;

        case HCI_VENDOR_RF_RADIO_REG_READ:
            {
                UINT16 data;
                UCHAR  bModem = 0;
                bModem = hci_vendor_g_read_radio_address&0x80;
                hci_vendor_g_read_radio_address &= 0x7f;
                if(bModem)
                {
                    //RT_BT_LOG(WHITE, HCI_VENDOR_EVENTS_211, 1, hci_vendor_g_read_radio_address);
                    data = rtk_read_modem_radio_reg(hci_vendor_g_read_radio_address, TYPE_MODEM);    				
                }
                else
                {
                    //RT_BT_LOG(WHITE, HCI_VENDOR_EVENTS_218, 1, hci_vendor_g_read_radio_address);
                    data = rtk_read_modem_radio_reg(hci_vendor_g_read_radio_address, TYPE_RF);
                }
                hci_event_pkt_ptr->event_parameter[param_length] = data & 0xFF;
                hci_event_pkt_ptr->event_parameter[param_length+1] = data >> 8;
                param_length+=2;
            }
            break;

#if 0            
#ifdef _MODEM_HCI_VENDOR_8821A_LOK_
        case HCI_VENDOR_EXECUTE_8821A_LOK:
            {
                hci_event_pkt_ptr->event_parameter[param_length] = hci_vendor_g_read_radio_address & 0xFF;
                hci_event_pkt_ptr->event_parameter[param_length+1] = hci_vendor_g_read_radio_address >> 8;
                param_length+=2;            
            }
            break;
#endif
#endif
            
#ifdef _UART_BAUD_ESTIMATE_           
        case HCI_VENDOR_UART_SYNC:
                hci_event_pkt_ptr->event_parameter[param_length] = (hci_uart_man_sram.baud_current_setting.d32 & 0xFF);
                hci_event_pkt_ptr->event_parameter[param_length+1] = ((hci_uart_man_sram.baud_current_setting.d32 >> 8) & 0xFF);
                hci_event_pkt_ptr->event_parameter[param_length+2] = ((hci_uart_man_sram.baud_current_setting.d32 >> 16) & 0xFF);
                hci_event_pkt_ptr->event_parameter[param_length+3] = ((hci_uart_man_sram.baud_current_setting.d32 >> 24) & 0xFF);
                param_length+=4;        
            break;
#endif
#ifdef _8821A_BTON_DESIGN_
        case HCI_VENDOR_UART_PARA_CHANGE:
            {
                UINT8 ii = 0;
                for (ii = 0; ii < g_hci_cmd_ptr_for_evt_gen->param_total_length; ii++)
                {
                    hci_event_pkt_ptr->event_parameter[param_length] = g_hci_cmd_ptr_for_evt_gen->cmd_parameter[ii];
                    param_length++;
#ifdef _YL_H5_TEST_UART_PARA_CHG
                    RT_BT_LOG(BLUE, YL_DBG_HEX_3, 3, ii, param_length, g_hci_cmd_ptr_for_evt_gen->cmd_parameter[ii]);
#endif                    
                }
            }
            break;
#endif

#ifdef _PLC_TEST_USED_VENDOR_COMMAND_
        case HCI_VENDOR_SEND_TEST_DATA:
            hci_event_pkt_ptr->event_parameter[param_length] = 
                                                    plt_test_var.rx_seqn;
            param_length++;
            break;
#endif
            
#ifdef _NEW_MODEM_PI_ACCESS_
        case HCI_VENDOR_RF_RADIO_REG_READ_PI:
            {
                UINT16 data;
                UCHAR  bModem = 0;
                bModem = hci_vendor_g_read_radio_address&0x80;
                hci_vendor_g_read_radio_address &= 0x7f;
                if(bModem)
                {
#ifdef _NEW_MODEM_PI_ACCESS_BY_VENDOR_REG_
//                        data = rtk_read_modem_radio_reg_pi(hci_vendor_g_read_radio_modem_page, hci_vendor_g_read_radio_address, TYPE_MODEM);
                    data = RTK_READ_MODEM_REG_PI(hci_vendor_g_read_radio_modem_page, hci_vendor_g_read_radio_address);
#else
                    if (g_efuse_modem_pi_enable) // yilinli: required? //
                    {
                        data = rtk_read_modem_radio_reg_pi(hci_vendor_g_read_radio_modem_page, hci_vendor_g_read_radio_address, TYPE_MODEM);
                    }
                    else
                    {
                        data = rtk_read_modem_radio_reg(hci_vendor_g_read_radio_address, TYPE_MODEM);
                    }
#endif                    
                }
                else
                {
                    //RT_BT_LOG(WHITE, HCI_VENDOR_EVENTS_218, 1, hci_vendor_g_read_radio_address);
                    data = rtk_read_modem_radio_reg(hci_vendor_g_read_radio_address, TYPE_RF);
                }
                hci_event_pkt_ptr->event_parameter[param_length] = data & 0xFF;
                hci_event_pkt_ptr->event_parameter[param_length+1] = data >> 8;
                param_length+=2;
            }
            break;
#endif

#ifdef _BT_ONLY_
#ifdef _PG_EFUSE_VIA_HCI_VENDOR_COMMAND_
        case HCI_VENDOR_WRITE_EFUSE_DATA:
            hci_event_pkt_ptr->event_parameter[4] = LSB(efuse_manger.app_len);
            hci_event_pkt_ptr->event_parameter[5] = MSB(efuse_manger.app_len);
            param_length = 6;
            break;
            
        case HCI_VENDOR_READ_EFUSE_DATA: 
            {
                UINT16 i;
                UINT16 len = 0;
                UINT16 addr = efuse_manger.app_start_addr;

                do
                {
                    param_length = 6;

                    if (cmd_status != HCI_COMMAND_SUCCEEDED)
                    {                    
                        break;
                    }
                 
                    for (i = 0; i < efuse_manger.app_len; i++)
                    {
                        hci_event_pkt_ptr->event_parameter[param_length] = 
                                 efuse_page0_read(efuse_manger.app_bank, addr);

                        if (efuse_manger.cur_status != EFUSE_PG_STATUS_OK)
                        {
                            /* update error code */
                            hci_event_pkt_ptr->event_parameter[3] = 
                                                    HARDWARE_FAILURE_ERROR; 
                            break;
                        }
                        
                        param_length++;
                        addr++;
                        len++;                                                
                    }
                } 
                while (0);

                hci_event_pkt_ptr->event_parameter[4] = LSB(len);
                hci_event_pkt_ptr->event_parameter[5] = MSB(len);
            }
            break;
#endif
#endif

        case HCI_VENDOR_READ_RTK_ROM_VERSION:        
            hci_event_pkt_ptr->event_parameter[param_length] = _ROM_VER_;
            param_length++;    
            break;
			
#ifdef _SUPPORT_MSFT_BT_HCI_EXTENSION_
       case HCI_VENDOR_MSFT_EXTENSION:
            if (g_hci_vs_msft_sub_opcode == 0xffff)
            {
                break;
            }

            /* fill subcommand_opcode released in v1.1B , SubCommand_OpCode*/
            hci_event_pkt_ptr->event_parameter[param_length] = g_hci_vs_msft_sub_opcode;
            param_length ++;

            switch (g_hci_vs_msft_sub_opcode)
            {
            case HCI_SUB_VENDOR_MSFT_READ_SUPPORTED_FEATURES:
                /* fill supported features field */
                memset(&hci_event_pkt_ptr->event_parameter[param_length], 0, 8);
                hci_event_pkt_ptr->event_parameter[param_length] = 0x0F;
                param_length += 8;

                /* fill MSFT event prefix length field */
                hci_event_pkt_ptr->event_parameter[param_length] = 0x08;
                param_length++;

                /* fill the data of MSFT event prefix field */
                memcpy(&hci_event_pkt_ptr->event_parameter[param_length], 
                                                    g_msft_event_prefix, 8);
                param_length += 8;                        
                break;

            case HCI_SUB_VENDOR_MSFT_LE_MONITOR_ADVERTISEMENT:
                /* fill the handle field assigned by bt controller */
                hci_event_pkt_ptr->event_parameter[param_length] = g_hci_vs_msft_le_monitor_handle;
                param_length++;
                break;
                
            case HCI_SUB_VENDOR_MSFT_MONITOR_RSSI:                
            case HCI_SUB_VENDOR_MSFT_CANCEL_MONITOR_RSSI:                            
            case HCI_SUB_VENDOR_MSFT_CANCEL_MONITOR_ADVERTISEMENT:
            case HCI_SUB_VENDOR_MSFT_LE_SET_ADVERTISEMENT_FILTER_ENABLE:
                break;

            case HCI_SUB_VENDOR_MSFT_READ_ABSOLUTE_RSSI:  
                {
                    UINT16 ce_index;   
                    INT8 rssi_dbm;
                    
                    if (LMP_GET_CE_INDEX_FROM_CONN_HANDLE(
                                g_hci_vs_msft_le_monitor_handle, &ce_index) == 
                                API_SUCCESS)
                    {
                        rssi_dbm = lc_calculate_log_from_rssi((UINT16)
                                      (lmp_connection_entity[ce_index].rssi));
                    }
                    else
                    {
                        rssi_dbm = 127;
                    }
                    
                    /* fill the handle and rssi field */
                    hci_event_pkt_ptr->event_parameter[param_length] = 
                                        LSB(g_hci_vs_msft_le_monitor_handle);
                    param_length++;
                    hci_event_pkt_ptr->event_parameter[param_length] = 
                                       MSB(g_hci_vs_msft_le_monitor_handle);
                    param_length++;                   
                    hci_event_pkt_ptr->event_parameter[param_length] = 
                                                            (UINT8)rssi_dbm;
                    param_length++; 
                }
                break;
            
            default:
                break;
            }
            
            g_hci_vs_msft_sub_opcode = 0xffff;            
            break;
#endif
			
        default:
            break;
    }

    hci_event_pkt_ptr->param_total_length  = param_length;

    return hci_event_pkt_ptr ;
}
#endif// (RT_VENDOR_CMDS)

#ifdef TEST_MODE
#if 0
/**************************************************************************
 * Function   : hci_generate_test_control_event
 *
 * Description:
 *      This function generates test control Event when the DUT gets a
 *		Test Control PDU.
 *
 * Parameters :
 *      UINT16 ce_index: connection entity index
 *
 * Returns    :
 *      HCI_EVENT_PKT *   : Pointer to an HCI event packet
 *
 *************************************************************************/
HCI_EVENT_PKT *hci_generate_test_control_event(UINT16 ce_index)
{
    HCI_EVENT_PKT *hci_event_pkt_ptr ;
	OS_SIGNAL signal_send;
    TEST_CONTROL_PARAMS *tcp;

	BT_FW_HCI_INF(GENERATING_TEST_CONTROLEVENT,0,0);

    if(OS_ALLOC_BUFFER(tx_table[HCI_EVENT_HANDLER_TASK].pool_handle,
                       (OS_ADDRESS *)&hci_event_pkt_ptr)
                       != BT_ERROR_OK)
    {
		BT_FW_HCI_ERR(UNABLE_TO_ALLOCATE_MEMORY_FROM_EVENT_BUFFER_POOL,0,0);
		return NULL;
	}

    tcp = &lmp_connection_entity[ce_index].test_mode_info.tc_params;

    hci_event_pkt_ptr->event_opcode = HCI_TEST_CONTROL_EVENT ;        
    hci_event_pkt_ptr->event_parameter[0] = TEST_CONTROL_EVENT_TYPE;
    hci_event_pkt_ptr->event_parameter[1] = tcp->test_scenario;
    hci_event_pkt_ptr->event_parameter[2] = tcp->hopping_mode;
    hci_event_pkt_ptr->event_parameter[3] = tcp->tx_frequency;
    hci_event_pkt_ptr->event_parameter[4] = tcp->rx_frequency;
    hci_event_pkt_ptr->event_parameter[5] = tcp->power_control_mode;
    hci_event_pkt_ptr->event_parameter[6] = tcp->poll_period;
    hci_event_pkt_ptr->event_parameter[7] = tcp->packet_type_in_pdu;
    hci_event_pkt_ptr->event_parameter[8] = tcp->num_packets & 0xFF;
    hci_event_pkt_ptr->event_parameter[9] = tcp->num_packets >> 8; 
    hci_event_pkt_ptr->param_total_length = 10;

	/* Signal to event task for delivering the event */
	signal_send.type = HCI_DELIVER_HCI_EVENT_SIGNAL ;
    signal_send.param =(OS_ADDRESS) hci_event_pkt_ptr ;

	if(OS_SEND_SIGNAL_TO_TASK(hci_event_task_handle, signal_send) != BT_ERROR_OK)
	{
		BT_FW_HCI_ERR(LOG_LEVEL_HIGH, OS_SEND_SIGNAL_TO_TASK_FAILED,0,0);
	}

    return hci_event_pkt_ptr ;
}
#endif
#endif /* TEST_MODE */

UCHAR hci_vendor_generate_log_data_packet(void)
{
#ifdef ENABLE_LOGGER
    HCI_EVENT_PKT *hci_event_pkt_ptr;
    HCI_ACL_RX_DATA_PKT *hci_acl_pkt_ptr;    
    OS_SIGNAL signal_send;
    UINT16 len;
    UINT8 *log_buf;
    UINT16 seg_len;
    UINT16 max_log_len;
    UINT8 two_segment;
    UINT16 rd_ptr;    

    DEF_CRITICAL_SECTION_STORAGE;

    if ((lbm_queue.length == 0) || 
        (dbg_vendor_log_buf != NULL) ||
        (dbg_vendor_set_log_complete_event_buf != NULL))
    {
        return API_SUCCESS;
    }

    if (dbg_vendor_log_interface == VENDOR_LOG_PACKET_TYPE_EVENT)
    {            
        if (OS_ALLOC_BUFFER(tx_table[HCI_EVENT_HANDLER_TASK].pool_handle,
                           (OS_ADDRESS *)&hci_event_pkt_ptr) != BT_ERROR_OK)
        {
            return API_FAILURE;
        }

        dbg_vendor_log_buf = (UINT8*)hci_event_pkt_ptr;        
        hci_event_pkt_ptr->event_opcode = HCI_VENDOR_SPECIFIC_EVENT;
        hci_event_pkt_ptr->event_parameter[0] = 0x20;
        hci_event_pkt_ptr->event_parameter[1] = 0x00;
        log_buf = &hci_event_pkt_ptr->event_parameter[2];
        max_log_len = GET_MAX_LOG_EVENT_SIZE;
        if (max_log_len == 0)
        {
            max_log_len = 252; /* default size */
        }
    }
    else
    {
        if (os_get_reserved_buffers() >= (BT_FW_TOTAL_ACL_PKTS_TO_HOST - 2))
        {
            /* do not affect normal protocol */
            return API_FAILURE;              
        }

        os_reserve_buffer();        

        if (OS_ALLOC_BUFFER(tx_table[ACL_DATA_HANDLER_TASK].pool_handle,
                           (OS_ADDRESS *)&hci_acl_pkt_ptr) != BT_ERROR_OK)
        {
            os_free_reserved_buffer();  

            return API_FAILURE;            
        }
        
        dbg_vendor_log_buf = (UINT8*)hci_acl_pkt_ptr;          
        hci_acl_pkt_ptr->connection_handle = dbg_vendor_log_conn_handle;
        hci_acl_pkt_ptr->packet_boundary_flag = 0;
        hci_acl_pkt_ptr->broadcast_flag = 0;
        log_buf = &hci_acl_pkt_ptr->hci_acl_data_pkt[0];
        max_log_len = GET_MAX_LOG_ACL_SIZE;

        if (max_log_len > otp_str_data.bt_read_buffer_size)            
        {
            max_log_len = otp_str_data.bt_read_buffer_size;
        }
        if (max_log_len == 0)
        {
            /* give a default size */
            max_log_len = otp_str_data.bt_read_buffer_size >> 1;
        }
    }   

    MINT_OS_ENTER_CRITICAL();

    if (lbm_queue.length >= max_log_len)
    {
        len = max_log_len;
    }
    else
    {
        len = lbm_queue.length;
    }
    lbm_queue.length -= len;

    if (dbg_vendor_log_interface == VENDOR_LOG_PACKET_TYPE_EVENT)
    {      
        hci_event_pkt_ptr->param_total_length = len + 2; /* add SubEventCode */
    }
    else
    {
        hci_acl_pkt_ptr->acl_data_total_length = len;
    }
    
    rd_ptr = lbm_queue.rd_ptr;

    if ((lbm_queue.rd_ptr + len) > LBM_BUFFER_SIZE)
    {
        two_segment = TRUE;
        seg_len = LBM_BUFFER_SIZE - lbm_queue.rd_ptr; /* first segment */
        lbm_queue.rd_ptr = len - seg_len;             /* second segment */
    }
    else
    {
        two_segment = FALSE;
        seg_len = len;
        lbm_queue.rd_ptr += seg_len;
        lbm_queue.rd_ptr &= (LBM_BUFFER_SIZE - 1);        
    }

    MINT_OS_EXIT_CRITICAL();  

    /* first segment */
    memcpy(log_buf, &lbm_queue.buf[rd_ptr], seg_len); 

    if (two_segment)
    {
        /* second segment */
        log_buf += seg_len;
        memcpy(log_buf, &lbm_queue.buf[0], len - seg_len);        
    }      

    if (dbg_vendor_log_interface == VENDOR_LOG_PACKET_TYPE_EVENT)
    {        
         signal_send.type  = HCI_TD_HCI_EVENT_TO_HOST_SIGNAL;
         signal_send.param = (OS_ADDRESS *)hci_event_pkt_ptr;
         
        /* Total length of the event packet = 
         * param_length (2nd  byte)+ Event type (1) + length field(1) */
         signal_send.length = hci_event_pkt_ptr->param_total_length + 2;
        
        if (OS_SEND_SIGNAL_TO_TASK(hci_td_task_handle, 
                                   signal_send) != BT_ERROR_OK)
        {
            OS_FREE_BUFFER(tx_table[HCI_EVENT_HANDLER_TASK].pool_handle,
                            dbg_vendor_log_buf);             
            dbg_vendor_log_buf = NULL;
        }
    }
    else
    {
        signal_send.type = HCI_TD_ACL_DATA_TO_HOST_SIGNAL;
        signal_send.param = (OS_ADDRESS *)hci_acl_pkt_ptr;
        
        if (OS_SEND_SIGNAL_TO_TASK(hci_td_task_handle, 
                                    signal_send) != BT_ERROR_OK)
        {
            OS_FREE_BUFFER(tx_table[ACL_DATA_HANDLER_TASK].pool_handle,
                           hci_acl_pkt_ptr); 
            os_free_reserved_buffer();            
            dbg_vendor_log_buf = NULL;
        }        
    }
#endif

    return API_SUCCESS;
}

UINT8 hci_vendor_check_log_data_packet(UINT8 type, UINT8 *buf, UINT8 free)
{
    if (IS_SUPPORT_HCI_LOG_PKT && (dbg_vendor_log_buf != NULL))
    {
        if (dbg_vendor_log_interface == type)
        {
            if (dbg_vendor_log_buf == buf)
            {
                if (free)
                {                    
                    dbg_vendor_log_buf = NULL;
                    hci_vendor_generate_log_data_packet();
                }
                return TRUE;
            }
        }
    }
    return FALSE;
}

#ifdef _SUPPORT_MSFT_BT_HCI_EXTENSION_
extern UINT8 g_msft_event_prefix[8];

/**************************************************************************
 * Function     : hci_generate_vs_msft_rssi_event
 *
 * Description  : This function is used to handle generate and send a 
 *                HCI_VS_MSFT_RSSI_Event to upstream.
 *                Please refer chap 4.1 of MS final BT HCI extensions 
 *                spec to know the detailed definition.
 *
 * Parameters   : status - Success (0x00) or Failure (0x01 ~ 0xFF) 
 *                conn_handle - The Handle for the connection for which 
 *                                the RSSI has to be monitored.
 *                rssi - The measured link RSSI value for the connection.
 *
 * Returns      : None
 *
 *************************************************************************/
void hci_generate_vs_msft_rssi_event(UINT8 status, UINT16 conn_handle, INT8 rssi)
{
    HCI_EVENT_PKT *hci_event_pkt_ptr;              
    
    if (OS_ALLOC_BUFFER(tx_table[HCI_EVENT_HANDLER_TASK].pool_handle,
                       (OS_ADDRESS *)&hci_event_pkt_ptr) == BT_ERROR_OK)
    {
        hci_event_pkt_ptr->event_opcode = HCI_VENDOR_SPECIFIC_EVENT;
        hci_event_pkt_ptr->param_total_length = 13;
        memcpy(hci_event_pkt_ptr->event_parameter, g_msft_event_prefix, 8);
        hci_event_pkt_ptr->event_parameter[8] = 0x01; /* msft event code */
        hci_event_pkt_ptr->event_parameter[9] = status;
        hci_event_pkt_ptr->event_parameter[10] = LSB(conn_handle);
        hci_event_pkt_ptr->event_parameter[11] = MSB(conn_handle);
        hci_event_pkt_ptr->event_parameter[12] = (UINT8)rssi;
        hci_td_deliver_event_to_host((UCHAR*)hci_event_pkt_ptr);
    }      
}

/**************************************************************************
 * Function     : hci_generate_vs_msft_le_monitor_event
 *
 * Description  : This function is used to handle generate and send a 
 *                HCI_VS_MSFT_LE_Monitor_Device_Event to upstream.
 *                Please refer chap 4.2 of MS final BT HCI extensions 
 *                spec to know the detailed definition.
 *
 * Parameters   : addr_type - Piublic or Random device address 
 *                paddr - the pointer of the Bluetooth address of the device 
 *                         to be monitored.
 *                mon_handle - The handle to the filter
 *                state - stop (0x00) or start (0x01) monitor
 *
 * Returns      : None
 *
 *************************************************************************/
void hci_generate_vs_msft_le_monitor_event(UINT8 addr_type, UINT8 *paddr, 
                                     UINT16 mon_handle, UINT8 state)
{
    HCI_EVENT_PKT *hci_event_pkt_ptr;              
    
    if (OS_ALLOC_BUFFER(tx_table[HCI_EVENT_HANDLER_TASK].pool_handle,
                       (OS_ADDRESS *)&hci_event_pkt_ptr) == BT_ERROR_OK)
    {
        hci_event_pkt_ptr->event_opcode = HCI_VENDOR_SPECIFIC_EVENT;
        hci_event_pkt_ptr->param_total_length = 18;
        memcpy(hci_event_pkt_ptr->event_parameter, g_msft_event_prefix, 8);
        hci_event_pkt_ptr->event_parameter[8] = 0x02; /* msft event code */
        hci_event_pkt_ptr->event_parameter[9] = addr_type;
        memcpy(&hci_event_pkt_ptr->event_parameter[10], paddr, 6);
        hci_event_pkt_ptr->event_parameter[16] = mon_handle;
        hci_event_pkt_ptr->event_parameter[17] = state;
        hci_td_deliver_event_to_host((UCHAR*)hci_event_pkt_ptr);
    }      
}

#endif

