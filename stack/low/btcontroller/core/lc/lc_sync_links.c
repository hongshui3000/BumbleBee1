/***************************************************************************
 Copyright (C) MindTree Ltd.
 This module is a confidential and proprietary property of MindTree and
 a possession or use of this module requires written permission of MindTree.
 ***************************************************************************/

/**
 * \file
 *  Contains the LC module SCO and ESCO implementation.
 */

/********************************* Logger *************************/
enum { __FILE_NUM__= 43 };
/********************************* Logger *************************/


/* ========================= Include File Section ==================== */
#include "bt_fw_os.h"
#include "lc_internal.h"
#include "btc_defines.h"
#include "bt_fw_globals.h"
#include "hci_td.h"
#include "bz_debug.h"
#include "bb_driver.h"
#include "led_debug.h"
#include "mem.h"

#ifdef SCO_OVER_HCI
#include "bz_fw_isoch.h"
#include "lc.h"
#endif //#ifdef SCO_OVER_HCI

#include "bt_fw_hci_2_1.h"
#include "bzdma.h"

#include "plc.h"

#ifdef _SUPPORT_BT_CONTROL_PCM_MUX_
#include "gpio.h"
#endif

#ifdef _DAPE_TEST_GEN_FAKE_ESCO_DATA
extern void gen_esco_data_packet(UINT8 change);
extern UINT8 g_gen_fake_esco_data;

#endif
#ifdef _DAPE_TEST_ALWAYZ_NAK_ESCO_BY_VENDOR_CMD
UINT8 g_nak_esco = 0;
#endif
/* ===================== Variable Declaration Section ================== */
#ifdef _IMPROVE_PCM_MUTE_CTRL_TIMING_
UINT8 g_sco_no_rx_count = 0;
UINT8 g_sco_no_rx_force_output_zero_en = FALSE;
#endif

#ifdef _BRUCE_MSBC_IS_RESTRICTED_PKT_TYPE
extern UCHAR  g_air_mode;
extern UINT16 g_voice_setting;
#endif

#ifdef _SUPPORT_BT_CONTROL_PCM_MUX_
UINT32 u32BtGpioStateBkup = 0;
UINT8 u8BtVenBtGpioCtrlBkup = 0;
#endif


extern UCHAR lc_start_sync_pkt_intr;
extern UINT8 lc_sco_pause_status;

#ifdef COMPILE_ESCO
extern LMP_ESCO_CONN_HANDLE_TO_CE_INDEX_TABLE
lmp_esco_ch_to_ce_index_table[LMP_MAX_ESCO_CONN_HANDLES];
extern LMP_ESCO_CONNECTION_ENTITY
lmp_esco_connection_entity[LMP_MAX_ESCO_CONN_ENTITIES];

extern POOL_ID synchronous_data_to_host_pool_id;
extern POOL_ID synchronous_data_pool_id;
extern OS_HANDLE hci_event_task_handle;
extern UCHAR num_sync_links_over_codec;
extern UCHAR lmp_esco_over_codec;
extern ESCO_SCHEDULER esco_scheduler;

extern UCHAR lc_esco_window[];

extern UCHAR lc_esco_stop_reception[];

extern UCHAR lc_esco_pkt_tx[];

/* Indicates if a packet is stored in the global esco buffer */
extern UCHAR global_esco_buf_valid[];
/* Indicates if the data stored in the global buffer has CRC error */
extern UCHAR global_esco_data_correct[];

#endif /* COMPILE_ESCO */

#ifdef SCO_OVER_HCI
/* Externs */
extern UINT16 lc_var_esco_fifo_flush_length_and_tx_pkt_len_reg;
extern POOL_ID synchronous_data_to_host_pool_id;
extern OS_HANDLE hci_event_task_handle;

extern UCHAR num_sync_links_over_codec;
extern UCHAR lmp_esco_over_codec;
extern UINT16 bz_sync_ch_conn_handle;

extern SECTION_SRAM UINT8 sco_tx_zero_buf[64];

#endif //#ifdef SCO_OVER_HCI

/* ================== For rom code patch function point ================== */
#ifdef _ROM_CODE_PATCHED_
#ifdef _CCH_8723_RCP_
PF_ROM_CODE_PATCH_FUNC rcp_lc_handle_connect_sco_end = NULL;
PF_ROM_CODE_PATCH_FUNC rcp_lc_make_esco_connection_end = NULL;
PF_ROM_CODE_PATCH_FUNC rcp_lc_handle_sco_rx_interrupt = NULL;
PF_ROM_CODE_PATCH_FUNC rcp_lc_handle_sco_erroneous_data = NULL;
PF_ROM_CODE_PATCH_FUNC rcp_lc_handle_esco_rx_packet = NULL;
#endif

#ifdef _CCH_NEW_SPEC_RCP_
PF_ROM_CODE_PATCH_FUNC rcp_lc_make_esco_connection = NULL;
PF_ROM_CODE_PATCH_FUNC rcp_lc_make_esco_connection_OverCodec = NULL;
PF_ROM_CODE_PATCH_FUNC rcp_lc_make_sco_connection_OverCodec = NULL;
#endif


#ifdef _BRUCE_8821B_PLC_RCP_
PF_ROM_CODE_PATCH_VOID rcp_lc_handle_connect_sco_func = NULL;
PF_ROM_CODE_PATCH_VOID rcp_lc_make_esco_connection_func = NULL;
PF_ROM_CODE_PATCH_VOID rcp_lc_kill_sco_connection_over_codec_func = NULL;
PF_ROM_CODE_PATCH_VOID rcp_lc_kill_esco_connect_over_codec_func = NULL;
#endif

#ifdef _SUPPORT_BT_CONTROL_PCM_MUX_
PF_ROM_CODE_PATCH_FUNC rcp_restore_pcm_mux_state_when_disconnect = NULL;
//PF_ROM_CODE_PATCH_FUNC rcp_switch_pcm_mux_after_con_established = NULL;
PF_ROM_CODE_PATCH_FUNC rcp_switch_pcm_mux_after_con_established_pcmoutctrl = NULL;
#endif

PF_ROM_CODE_PATCH_FUNC rcp_lc_handle_esco_instant_interrupt_new = NULL;
PF_ROM_CODE_PATCH_FUNC rcp_lc_handle_sco_instant_interrupt = NULL;
#endif
/* ===================== Function Definition Section =================== */
#ifdef ENABLE_PCM_ONLY_WHEN_SCO_LINK_CREATED
void lc_pcm_enable_control(UINT8 enable)
{
    UINT32 u32_data = VENDOR_READ(0x150);
    if (enable)
    {
        if (!(u32_data & 0x10))
        {
            u32_data |= 0x10;
            VENDOR_WRITE(0x150, u32_data);
        }
    }
    else
    {
        if (u32_data & 0x10)
        {
            u32_data &= ~0x10;
            VENDOR_WRITE(0x150, u32_data);
        }
    }
}
#endif

#ifdef _IMPROVE_PCM_MUTE_CTRL_TIMING_
void lc_pcm_mute_control(void)
{
    UINT16 read;

    if (g_sco_no_rx_force_output_zero_en)
    {
        if (g_sco_no_rx_count == 0)
        {
            read = BB_read_baseband_register(BB_PCM_CTRL2_REG);
            if (read & BIT14)
            {
                BB_write_baseband_register(BB_PCM_CTRL2_REG, read & ~BIT14);
            }
            g_sco_no_rx_force_output_zero_en = FALSE;
        }
    }
    else
    {
        if (g_sco_no_rx_count >= (1 << (g_efuse_lps_setting_3.iot_sco_noise_no_sco_count << 2)))
        {
            read = BB_read_baseband_register(BB_PCM_CTRL2_REG);
            if (!(read & BIT14))
            {
                BB_write_baseband_register(BB_PCM_CTRL2_REG, read | BIT14);
            }
            g_sco_no_rx_force_output_zero_en = TRUE;
        }
    }
}
#endif

#ifdef COMPILE_ESCO
/* piconet_id is log_piconet_id for non-scatternet and phy_piconet_id for scatternet. */
UINT16 lc_get_desco_value(UCHAR piconet_id,
                                        UCHAR tc_flags, UINT16 tesco, UINT16 desco)
{
    UINT32 master_clock;
    UINT32 largest_instant;
    UINT16 new_desco;
    UCHAR offset;
    UCHAR delta_offset;

    /* Program the baseband with modified desco */
    lc_get_clock_in_scatternet(&master_clock, piconet_id);

    /* shift clock by one bit to ignore clk0 bit */
    master_clock >>=1;

    /* Check if INIT2 procedure has to be used for synchronising
       Master clock used for finding Desco depeneds on the timing
       control flag.
    */

    if(tc_flags & ESCO_TC_FLAG_INIT2)
    {
        //ESCO_LOG_ERROR(LOG_LEVEL_LOW,"using INIT 2 procedure to create esco");

        /* applying INIT2 procedure .
           (!clk,clk(26-1)) mod Tesco = Desco
        */

        /* Check if the 27'th bit is set */
        if(master_clock & 0x0004000000)
        {
            /* Set the 27'th bit to 0 */
            master_clock &=0x0003FFFFFF;
        }
        else
        {
            /* Set the 27'th bit to 1 */
            master_clock |=0x0004000000;
        }
    }

    /* The altered master clock based on the Init procedure used
       for synchronization is used only for finding the offset
    */
    if((master_clock != 0) || (tesco != 0))
    {
        offset = (UCHAR) (master_clock % tesco);
    }
    else
    {
        offset = 0;
    }

    delta_offset = (UCHAR) (tesco - offset);
    new_desco = (UINT16) (delta_offset + desco);
    if(new_desco > tesco)
    {
        new_desco = (UINT16) (new_desco - tesco);
    }
    if(new_desco <= 1)
    {
        new_desco = (UINT16) (new_desco + tesco);
    }

    largest_instant = lc_esco_get_largest_instant();

    /**
     * Replaced while loop to find final value of new_desco
     * with direct calculation. Clock wraparound is still
     * not handled - but the firmware will not get stuck
     * if clock wraps around and largest instant is in the past.
     */
    if((master_clock + new_desco) < largest_instant)
    {
        /**
         * We have to find smallest integral value of temp such that
         * masterclock + (new_desco + temp*tesco) >= largest_instant
         * ie; temp >= (largest_instant - master_clock - new_desco)/tesco.
         */
        UINT32 offset = (largest_instant - master_clock - new_desco);
        UINT32 num_tesco = offset/tesco;

        /** If (num_tesco/tesco) is not integral, it'll be rounded to ceil value.*/
        if((num_tesco*tesco) < offset)
        {
            ++num_tesco;
        }
        new_desco = (UINT16)(new_desco + num_tesco*tesco);
    }

    return new_desco;
}

/**
 * Programs the baseband to create a esco link.
 * Baseband will programmed with midified Desco value.
 *
 * \param esco_ce_index eSCO connection entity index.
 * \param lt_addr AM_ADDR/LT_ADDR on which eSCO link has to be created.
 *
 * \return None.
 */
void lc_make_esco_connection(UINT16 esco_ce_index, UCHAR lt_addr)
{

#ifdef _ROM_CODE_PATCHED_
#ifdef _CCH_NEW_SPEC_RCP_
    UCHAR return_status;
    if (rcp_lc_make_esco_connection != NULL)
    {
        if ( rcp_lc_make_esco_connection((void *)&return_status, esco_ce_index, lt_addr) )
        {
            return;
        }
    }
#endif
#endif


#ifdef _CCH_RETENTION_FLOW_FOR_DLPS_
#ifdef _PLC_FUNC_ENABLE_
    plc_init();
#endif
#endif

    UINT16 tesco;
    UINT16 desco;
    UINT16 esco_window;
    UINT16 rx_pkt_length;
    UINT16 tx_pkt_length;
    UINT16 rx_pkt_type;
    UINT16 tx_pkt_type;
    UINT16 acl_ce_index;
    UINT16 con_am_addr;
    UINT16 make_esco_bit;
    UINT16 new_desco;
    UINT16 esco_bit;
    UINT16 temp_reg;
    UINT16 esco_codec_bit;
    UINT32 master_clock;
    UINT32 new_master_clock;
    UCHAR esco_link_index;
    UINT16 address, read;
    UINT16 fifo_config;
    LMP_CONNECTION_ENTITY *ce_ptr;
    LMP_ESCO_CONNECTION_ENTITY *esco_ce_ptr;
    DEF_CRITICAL_SECTION_STORAGE;

    UCHAR piconet_id;

/*  Added by Wallice Su for PCM external CODEC. 2011/01/03  */
    UINT16 CoedcCodeTable = 0x0;
/*  End Added by Wallice Su for PCM external CODEC. 2011/01/03  */

#ifdef ENABLE_LOGGER_LEVEL_2
    ESCO_LOG_INFO(LOG_LEVEL_LOW,INSTRUCTING_BASEBAND_TO_CREATE_ESCO_CONNECTION,0,0);

    ESCO_LOG_INFO(LOG_LEVEL_LOW,CREATING_ESCO_FOR_LT_ADDR,1,lt_addr);
#endif
    /* Esco link index will have the baseband Esco link number. It will be
       derived from esco_ce_index. Esco link index = esco_ce_index + 1
    */
    esco_link_index = (UCHAR) (esco_ce_index + 1);

    esco_ce_ptr = &lmp_esco_connection_entity[esco_ce_index];

    acl_ce_index = esco_ce_ptr->ce_index;

    ce_ptr = &lmp_connection_entity[acl_ce_index];

    piconet_id = ce_ptr->phy_piconet_id;

    con_am_addr = lt_addr;
    con_am_addr <<= 5;

    /* Set the esco bit */
    esco_bit = 1<<9;

    /* Set the make esco connection bit */
    make_esco_bit = 1 << 4;

    /* If ESCO and SCO (OVER_NULL) links can be established at the same
     * time, then the following method of setting 'fifo_config' would result
     * in a race condition (BB_FLUSH also uses this these bits and it is done
     * in the interrupt context also).
     */
    RT_BT_LOG(GRAY, LC_SYNC_LINKS_280, 4,
        lmp_esco_over_codec, num_sync_links_over_codec, esco_ce_index, lt_addr);

    fifo_config = BB_read_baseband_register(SYNC_FIFO_CONFIG_REGISTER);
    if( (num_sync_links_over_codec < MAX_NUMBER_OF_CODEC_LINKS) &&
            (lmp_esco_over_codec == TRUE))
    {
        esco_codec_bit = 1 << 10;
        esco_ce_ptr->use_codec = TRUE;
//        num_sync_links_over_codec++;
        fifo_config = (UINT16)(fifo_config & (~SYNC_FIFO1_CONFIG_MASK));
        fifo_config = (UINT16)(fifo_config | SYNC_FIFO1_USING_PCM);
    }
    else
    {
        esco_codec_bit = 0;
        esco_ce_ptr->use_codec = FALSE;
        fifo_config = (UINT16)(fifo_config & (~SYNC_FIFO2_CONFIG_MASK));
        fifo_config = (UINT16)(fifo_config | SYNC_FIFO2_OVER_HCI);

        /* in 0380, we use new codec path */
        esco_codec_bit = 1 << 10;
        bz_isoch_add_sco_queues(esco_ce_ptr->conn_handle, acl_ce_index);
    }

    BB_write_baseband_register(SYNC_FIFO_CONFIG_REGISTER, fifo_config);

    con_am_addr = (UINT16)(con_am_addr | esco_bit | make_esco_bit
                  | esco_link_index | esco_codec_bit | (piconet_id << 11) );

    tesco =  esco_ce_ptr->tesco;
    esco_window = esco_ce_ptr->esco_window;
    desco = esco_ce_ptr->desco;

    if (ce_ptr->remote_dev_role == MASTER)
    {
        rx_pkt_length = esco_ce_ptr->m_to_s_packet_length;
        tx_pkt_length = esco_ce_ptr->s_to_m_packet_length;
        rx_pkt_type = esco_ce_ptr->m_to_s_packet_type;
        tx_pkt_type = esco_ce_ptr->s_to_m_packet_type;

        if(rx_pkt_type == LMP_DEFAULT_ESCO_PKT)
        {
            rx_pkt_type = BB_POLL;
        }
    }
    else
    {
        rx_pkt_length = esco_ce_ptr->s_to_m_packet_length;
        tx_pkt_length = esco_ce_ptr->m_to_s_packet_length;
        rx_pkt_type = esco_ce_ptr->s_to_m_packet_type;
        tx_pkt_type = esco_ce_ptr->m_to_s_packet_type;

        if(rx_pkt_type == LMP_DEFAULT_ESCO_PKT)
        {
            rx_pkt_type = BB_NULL;
        }
    }

    temp_reg = 0x0200;
    temp_reg = (UINT16) (temp_reg << esco_link_index);

    if( (tx_pkt_type == LMP_2_EV5) || (tx_pkt_type == LMP_3_EV5) ||
            (tx_pkt_type == LMP_EV4) || (tx_pkt_type == LMP_EV5) )
	{
		/* Write in baseband register to indicate that the tx pkt type
		* is a multislot packet
		*/
		lc_var_esco_fifo_flush_length_and_tx_pkt_len_reg |= temp_reg;
	}
	else
	{
		lc_var_esco_fifo_flush_length_and_tx_pkt_len_reg &= (~temp_reg);
	}

	BB_write_baseband_register(ESCO_FIFO_FLUSH_LENGTH_AND_TX_PKT_LEN_REGISTER,
		lc_var_esco_fifo_flush_length_and_tx_pkt_len_reg);

    /* Program primary am_addr: in TX_GAIN bits for ESCO LUT. */
    address = reg_MASTER_UPPER_LUT[lt_addr];
    read = BB_read_baseband_register((UCHAR)address);
    read = (UINT16)(read & 0xF1FF);
    read = (UINT16)(read | ((ce_ptr->am_addr) << 9));

    BB_write_baseband_register(address, read);

    /* program default tx packet */
    address = reg_MASTER_LOWER_LUT[lt_addr];
    BB_write_baseband_register(address, (tx_pkt_type) << 12 | tx_pkt_length);

    /* Set the PTT bit in the upper LUT for EDR pkts. */
    if ((tx_pkt_type == LMP_2_EV3) || (tx_pkt_type == LMP_3_EV3) ||
        (tx_pkt_type == LMP_2_EV5) || (tx_pkt_type == LMP_3_EV5))
    {
        address = reg_MASTER_UPPER_LUT[lt_addr];
        read = BB_read_baseband_register(address);
        read |= 0x8000;
        BB_write_baseband_register( (UCHAR) address , read);
    }

    /* Program Desco */
    master_clock = 0;
    new_master_clock = 0;

    MINT_OS_ENTER_CRITICAL();

    lc_get_clock_in_scatternet(&master_clock, piconet_id);

    master_clock >>=1;

    /* wait till the clock bit toggles */
    while(master_clock + 1 > new_master_clock)
    {
        lc_get_clock_in_scatternet(&new_master_clock, piconet_id);

        new_master_clock >>= 1;
    }

    UINT16 esco_reg;
    UINT16 esco_rx_len_reg;
    UINT16 esco_win_reg;

    master_clock = new_master_clock;

    new_desco = lc_get_desco_value(piconet_id,
                       esco_ce_ptr->timing_control_flags, esco_ce_ptr->tesco,
                       esco_ce_ptr->desco);
#ifndef _DAPE_TEST_FIX_ESCO_WINDOW_WRONG
    while(new_desco < LC_MIN_VAL_DESCO)
    {
#endif
        new_desco += tesco;
#ifndef _DAPE_TEST_FIX_ESCO_WINDOW_WRONG
    }
#endif

    /* Program Codec Register */
    if (esco_ce_index != ISOCH_SCO_MAX_CONNS)
    {
        if (ce_ptr->is_esco_channel)
        {
            /* configure new SCO conversion table */
            if (ce_ptr->is_pcm_input)
            {
                read = BB_read_baseband_register(BB_SCO_CONV_TABLE_REG);
                read &= ~(0x03 << (esco_ce_index << 1));
                read |= ce_ptr->pcm_conv_type << (esco_ce_index << 1);
                BB_write_baseband_register(BB_SCO_CONV_TABLE_REG, read);
            }

            read = BB_read_baseband_register(BB_CODEC_CODE_TABLE_REG);
            read &= ~(0x0F << (esco_ce_index << 2));
            read |= (ce_ptr->trx_codec_conv_type |
                     (ce_ptr->txfifo_in_8bit_code << 3)) << (esco_ce_index << 2);
            BB_write_baseband_register(BB_CODEC_CODE_TABLE_REG, read);
        }

        ce_ptr->is_esco_channel = TRUE;
        ce_ptr->esco_ce_idx = esco_ce_index;
    }

/*  Add some code for CODEC convert with BZDMA, Wallice Su. 2010/12/18  */
    if (lmp_esco_over_codec == TRUE)
    {

#ifdef _ROM_CODE_PATCHED_
#ifdef _CCH_NEW_SPEC_RCP_
        if (rcp_lc_make_esco_connection_OverCodec != NULL)
        {
            if ( rcp_lc_make_esco_connection_OverCodec((void *)&esco_ce_index, lt_addr, pcm_ex_codec_format, pcm_ex_codec_format_8bit, pcmifctrl1, pcmifctrl2, scoconv, pcmconvert, hci_excodec_state, pcmifctrl3) )
            {
                MINT_OS_EXIT_CRITICAL();

                return;
            }
        }
#endif
#endif


        con_am_addr |= 0x400; //CODEC link

#ifdef _HW_AUTO_PER_PKT_MUTE_
        UINT16 reg_val;
        reg_val = BB_read_baseband_register(VOICE_SETTING_REGISTER);
        reg_val &= ~BIT3;            //Don't check CRC error
        reg_val |= (BIT0 | BIT2);    //External CODEC link and hw per-pkt mute
        BB_write_baseband_register(VOICE_SETTING_REGISTER, reg_val);
#else
#ifdef _HW_AUTO_MUTE_
        UINT16 reg_val;
        reg_val = BB_read_baseband_register(VOICE_SETTING_REGISTER);
        reg_val &= (BIT4|BIT1);
        reg_val |= (BIT0);    //External CODEC link
        BB_write_baseband_register(VOICE_SETTING_REGISTER, reg_val);
#else
        BB_write_baseband_register(VOICE_SETTING_REGISTER, 0x01); //External CODEC link
#endif
#endif

        read = BB_read_baseband_register(SYNC_FIFO_CONFIG_REGISTER);
        BB_write_baseband_register(SYNC_FIFO_CONFIG_REGISTER,
                                    (read & 0xC000) | 0x05);     //External CODEC link


/* Updated by Wallice Su for mSBC over PCM.	2013/11/20	*/
	  if (pcm_ex_codec_format == PCM_CODEC_TYPE_MSBC)
	  {
        	        read = BB_read_baseband_register(VOICE_SETTING_REGISTER);
        	        read |= (BIT10);    //mSBC enable
        	        BB_write_baseband_register(VOICE_SETTING_REGISTER, read);
	  }

        switch (esco_ce_ptr->air_mode)
        {
            case 0x0://u-law
                switch (pcm_ex_codec_format)
                {
                    case PCM_CODEC_TYPE_LINEAR://Linear
                        if (pcm_ex_codec_format_8bit)//Linear 8 bits
                        {
                            CoedcCodeTable = 0x09;
                        }
                        else
                        {
                            CoedcCodeTable = 0x01;
                        }
                        break;

                    case PCM_CODEC_TYPE_ALAW://A-law
                        CoedcCodeTable = 0x0B;
                        break;

                    case PCM_CODEC_TYPE_CVSD://CVSD
                        CoedcCodeTable = 0x07;
                        break;

                    case PCM_CODEC_TYPE_MSBC://mSBC
                        CoedcCodeTable = 0x07;
                        break;

                    default:
                    case PCM_CODEC_TYPE_ULAW://u-law
	                 if (pcm_ex_codec_format_8bit)//Linear 8 bits
	                 {
	                     CoedcCodeTable = 0x08;
	                 }
	                 else
	                 {
                        CoedcCodeTable = 0x00;
	                 }
                        break;
                }
                break;

            case 0x1://A-law
                switch (pcm_ex_codec_format)
                {
                    case PCM_CODEC_TYPE_LINEAR://Linear
                        if (pcm_ex_codec_format_8bit)//Linear 8 bits
                        {
                            CoedcCodeTable = 0x0A;
                        }
                        else
                        {
                            CoedcCodeTable = 0x02;
                        }
                        break;

                    case PCM_CODEC_TYPE_ULAW://u-law
                        CoedcCodeTable = 0x0C;
                        break;

                    case PCM_CODEC_TYPE_CVSD://CVSD
                    case PCM_CODEC_TYPE_MSBC://mSBC
                        CoedcCodeTable = 0x06;
                        break;

                    default:
                    case PCM_CODEC_TYPE_ALAW://A-law
	                 if (pcm_ex_codec_format_8bit)//Linear 8 bits
	                 {
	                     CoedcCodeTable = 0x08;
	                 }
	                 else
	                 {
                        CoedcCodeTable = 0x00;
	                 }
                        break;
                }
                break;

        case 0x2://CVSD
            switch (pcm_ex_codec_format)
            {
                case PCM_CODEC_TYPE_LINEAR://Linear
                    if (pcm_ex_codec_format_8bit)//Linear 8 bits
                    {
                        CoedcCodeTable = 0x0D;
                    }
                    else
                    {
                        CoedcCodeTable = 0x05;
                    }
                    break;

                case PCM_CODEC_TYPE_ULAW://u-law
                    CoedcCodeTable = 0x0F;
                    break;

                case PCM_CODEC_TYPE_ALAW://A-law
                    CoedcCodeTable = 0x0E;
                    break;

                    case PCM_CODEC_TYPE_CVSD://CVSD
                    case PCM_CODEC_TYPE_MSBC://mSBC
                        if (pcm_ex_codec_format_8bit)//Linear 8 bits
                        {
                            CoedcCodeTable = 0x0D;
                        }
                        else
                        {
                            CoedcCodeTable = 0x05;
                        }
                        break;

                default://Transparent
	                 if (pcm_ex_codec_format_8bit)//Linear 8 bits
	                 {
	                     CoedcCodeTable = 0x08;
	                 }
	                 else
	                 {
                    CoedcCodeTable = 0x00;
	                 }
                    break;

            }
            break;

            case 0x3://Transparent
                switch (pcm_ex_codec_format)
                {
                    case PCM_CODEC_TYPE_CVSD://CVSD
                    case PCM_CODEC_TYPE_MSBC://mSBC
                        if (pcm_ex_codec_format_8bit)//Linear 8 bits
                        {
                            CoedcCodeTable = 0x0D;
                        }
                        else
                        {
                            CoedcCodeTable = 0x05;
                        }
                        break;

                    case PCM_CODEC_TYPE_LINEAR://Linear
                    case PCM_CODEC_TYPE_ULAW://u-law
                    case PCM_CODEC_TYPE_ALAW://A-law
                    default://Transparent
	                 if (pcm_ex_codec_format_8bit)//Linear 8 bits
	                 {
	                     CoedcCodeTable = 0x08;
	                 }
	                 else
	                 {
	                     CoedcCodeTable = 0x00;
	                 }
	                 break;

                }
                break;

            default:
                CoedcCodeTable = 0x00;
                break;
        }

        read = BB_read_baseband_register(BB_CODEC_CODE_TABLE_REG);
        read &= ~(0x0F << (esco_ce_index << 2));
        read |= CoedcCodeTable << (esco_ce_index << 2);
        BB_write_baseband_register(BB_CODEC_CODE_TABLE_REG, read);

        BB_write_baseband_register(BB_PCM_CTRL1_REG, pcmifctrl1);
        BB_write_baseband_register(BB_PCM_CTRL2_REG, pcmifctrl2);
        BB_write_baseband_register(BB_PCM_CTRL3_REG, pcmifctrl3);

        read = BB_read_baseband_register(BB_SCO_CONV_TABLE_REG);
        read &= ~(0x03 << (esco_ce_index << 1));
        read |= scoconv << (esco_ce_index << 1);
        BB_write_baseband_register(BB_SCO_CONV_TABLE_REG, read);

        read = BB_read_baseband_register(PLC_CTRL2_REGISTER);
        read &= 0xFE1F; //Clear BIT8~BIT5
        read |=  ((pcmconvert & 0x0F) << 0x05);
        BB_write_baseband_register(PLC_CTRL2_REGISTER, read);

        read = BB_read_baseband_register(BB_PCMOUT_SHIFT_REG);
        read &= 0xFF87; //Clear BIT6~BIT3
        read |=  ((pcmconvert & 0xF0) >> 0x01);
        BB_write_baseband_register(BB_PCMOUT_SHIFT_REG, read);

#ifdef _BRUCE_DROP_MUTED_ISO_OUT_PKTS_OVER_CODEC
        if(IS_MUTED_OVER_CODEC_ENABLE)
        {
        read = BB_read_baseband_register(BB_MUTED_ISO_OUT_PKTS_CTRL1_REG);
        /*bit[0:14],
                  the compared amplitude for pcm_in data,
                  pcm_in_mute_lower_bound = -1 * pcm_in_comp_amp;
                  pcm_in_mute_upper_bound = +1 * pcm_in_comp_amp; */
        read |= 0x1F4;
        BB_write_baseband_register(BB_MUTED_ISO_OUT_PKTS_CTRL1_REG,read);
        read = BB_read_baseband_register(BB_MUTED_ISO_OUT_PKTS_CTRL2_REG);
        /* the threshold of muted sample counts for pcm_in data */
        read |= 0x2710;
        BB_write_baseband_register(BB_MUTED_ISO_OUT_PKTS_CTRL2_REG,read);
       }
#endif

#ifdef _PLC_FUNC_ENABLE_
        if(IS_PLC_ENABLE)
        {
        //(bruce) setting 0xE0[11]=1,codec_plc_en.
        read = BB_read_baseband_register(VOICE_SETTING_REGISTER);
        read |= (UINT16)(BIT11);
        if (pcm_ex_codec_format != PCM_CODEC_TYPE_MSBC)
        {
            read |= (UINT16)(BIT2);
        }
        read |= (UINT16)(BIT3);
        //RT_BT_LOG(YELLOW,YL_DBG_DEC_1,1,read);
        BB_write_baseband_register(VOICE_SETTING_REGISTER,read);
        }

#ifdef _BRUCE_DEBUG_PORT
        UINT16 read;
        read=BB_read_baseband_register(0x25c);
        read &= (UINT16)(0x00FF);
        BB_write_baseband_register(0x25c,read);
#endif
#endif

#ifdef ENABLE_PCM_ONLY_WHEN_SCO_LINK_CREATED
        lc_pcm_enable_control(1);// for v5, v6
#endif

#if 0
#ifdef _SUPPORT_BT_CONTROL_PCM_MUX_
        if ((pcmifctrl3 & 0x6000))
        {
            switch_pcm_mux_after_con_established_pcmoutctrl(pcmifctrl3); //PCM_OUT drivedata
        }
        else
        {
            switch_pcm_mux_after_con_established();// combo v6 only
        }
#endif
#endif
#ifdef _SUPPORT_BT_CONTROL_PCM_MUX_
            switch_pcm_mux_after_con_established_pcmoutctrl(pcmifctrl3);
#endif

    }

#ifdef _ROM_CODE_PATCHED_
#ifdef _BRUCE_8821B_PLC_RCP_
		if (rcp_lc_make_esco_connection_func != NULL)
		{
		    rcp_lc_make_esco_connection_func();
		}
#endif
#endif

/*  End Add some code for CODEC convert with BZDMA, Wallice Su. 2010/12/18  */

    /* Programming the connector register and make esco bit. */
    BB_write_baseband_register(CONNECTOR_REGISTER, con_am_addr);

    esco_reg = (UINT16) (tesco << 8);
    BB_write_baseband_register(ESCO_INTERVAL_REGISTER, esco_reg);

    esco_rx_len_reg = (UINT16) ( (rx_pkt_type << 12) | rx_pkt_length);

    RT_BT_LOG(GREEN,BRUCE_ESCO_PKT_TYPE, 1,rx_pkt_type);

    BB_write_baseband_register(ESCO_RX_LENGTH_TYPE_REGISTER,
                               esco_rx_len_reg);

    /* Programming the Desco register. */
    esco_win_reg = (UINT16) ((esco_window << 8) | (new_desco - 2));
    BB_write_baseband_register(ESCO_WINDOW_DESCO_REGISTER, esco_win_reg);
#ifdef _DAPE_SET_ESCO_RETRY_PRIORITY_HIGHER_WHILE_NO_FULL_BW
    /* If esco is not full bandwidth, then we can set the
       priority of esco retry window to high.*/
    UINT16 reg;
    if ((esco_window <= (tesco- 2)) && (ll_manager.conn_unit.connection_cnts == 0))
    {
        reg = BB_read_baseband_register(0x5E);
        BB_write_baseband_register(0x5E, reg|BIT14);
        reg = BB_read_baseband_register(0x120);
        BB_write_baseband_register(0x120, reg | BIT8);
    }
    else
    {
        reg = BB_read_baseband_register(0x5E);
        BB_write_baseband_register(0x5E, reg&(~BIT14));
        reg = BB_read_baseband_register(0x120);
        BB_write_baseband_register(0x120, reg & (~BIT8));
    }
#endif

    esco_ce_ptr->next_instant = master_clock + new_desco;

    /* Instruct the baseband to make eSCO connection. */
    BB_write_baseband_register(INSTRUCTION_REGISTER, BB_EXECUTE);

    /* Set the State in the ESCO CE to indicate that BB connection is created
    * for this ESCO link
    */
    esco_ce_ptr->bb_esco_connection = TRUE;

    {
        UINT16 reg_val;

        reg_val = BB_read_baseband_register(SCO_PACKET_TYPE_REGISTER);
        if (lmp_check_sync_conn_bandwidth(TRUE) == API_FAILURE)
        {
            reg_val = (UINT16)(reg_val | BIT8); /* Set eSCO full-bw bit */
        }
        else
        {
            reg_val = (UINT16)(reg_val & ~BIT8); /* Clear eSCO full-bw bit */
        }
        BB_write_baseband_register(SCO_PACKET_TYPE_REGISTER, reg_val);
    }

    MINT_OS_EXIT_CRITICAL();

    if (lmp_esco_over_codec == TRUE)
    {
        RT_BT_LOG(GREEN, LC_ESCO_LINKS_CODEC, 4,
                        esco_ce_index,
                        esco_ce_ptr->air_mode,
                        pcm_ex_codec_format,
                        pcm_ex_codec_format_8bit);
        RT_BT_LOG(GREEN, LC_PCM_CODEC_REG, 9,
                        BB_read_baseband_register(BB_CODEC_CODE_TABLE_REG),
                        BB_read_baseband_register(BB_SCO_CONV_TABLE_REG),
                        BB_read_baseband_register(BB_PCM_CTRL1_REG),
                        BB_read_baseband_register(BB_PCM_CTRL2_REG),
                        BB_read_baseband_register(BB_PCM_CTRL3_REG),
                        BB_read_baseband_register(VOICE_SETTING_REGISTER),
                        BB_read_baseband_register(PLC_CTRL2_REGISTER),
                        BB_read_baseband_register(BB_PCMOUT_SHIFT_REG),
                        BB_read_baseband_register(0x260));
    }

#ifdef _CCH_SLOT_OFFSET_
    //RT_BT_LOG(RED, CCH_DBG_019, 0,0);
    lmp_force_global_slot_offset(acl_ce_index,
        LMP_MAX_CE_DATABASE_ENTRIES+ esco_ce_index, tesco, esco_window, desco);
#endif

    RT_BT_LOG(GRAY, LC_SYNC_LINKS_ESCO_REG_MSG, 4,
              con_am_addr, esco_reg, esco_rx_len_reg, esco_win_reg);

    LC_LOG_INFO(LOG_LEVEL_LOW, ESCO_MSG_CODEC_SETTING, 5,
                esco_ce_index,
                ce_ptr->is_esco_channel, ce_ptr->is_pcm_input,
                BB_read_baseband_register(BB_SCO_CONV_TABLE_REG),
                BB_read_baseband_register(BB_CODEC_CODE_TABLE_REG));

#ifdef ENABLE_LOGGER_LEVEL_2
    ESCO_LOG_INFO(LOG_LEVEL_LOW,MASTER_CLOCK_USED,1,master_clock);
    ESCO_LOG_INFO(LOG_LEVEL_LOW,ESCO_WINDOW_DESCO_REGISTER_STR,1,((esco_window << 8) | (new_desco - 2)));
    ESCO_LOG_INFO(LOG_LEVEL_LOW,ESCO_INTERVAL,1,tesco);
    ESCO_LOG_INFO(LOG_LEVEL_LOW,ESCO_WINDOW,1,esco_window);
    ESCO_LOG_INFO(LOG_LEVEL_LOW,ESCO_RX_LENGTH,1,rx_pkt_length);
    ESCO_LOG_INFO(LOG_LEVEL_LOW,ESCO_RX_TYPE,1,rx_pkt_type);
    ESCO_LOG_INFO(LOG_LEVEL_LOW,DESCO,1,new_desco);

    ESCO_LOG_INFO(LOG_LEVEL_LOW,MTOS_PACKET_LEN,1,
                  esco_ce_ptr->m_to_s_packet_length);
    ESCO_LOG_INFO(LOG_LEVEL_LOW,STOM_PACKET_LEN,1,
                  esco_ce_ptr->s_to_m_packet_length);
#endif

    lc_check_and_enable_scans_in_scatternet();

    bz_sync_ch_conn_handle = esco_ce_ptr->conn_handle;

    if(esco_ce_ptr->use_codec == TRUE)
    {
        num_sync_links_over_codec++;
    }

#ifdef _ROM_CODE_PATCHED_
#ifdef _CCH_8723_RCP_
    if (rcp_lc_make_esco_connection_end != NULL)
    {
        rcp_lc_make_esco_connection_end((void *)&esco_ce_index, lt_addr);
    }
#endif
#endif

//}}
    return;
}

/**
 * Kills the BB level eSCO connection.
 *
 * \param esco_ce_index eSCO connection entity index.
 *
 * \return None.
 */
void lc_kill_esco_connection(UINT16 esco_ce_index)
{
    UINT16 con_am_addr;
    UINT16 make_esco_bit;
    UINT16 esco_bit;
    UINT16 temp_reg;
    UCHAR esco_link_index;
    UINT16 fifo_config;
    UINT16 address;
    UINT16 read;

    LMP_ESCO_CONNECTION_ENTITY *esco_ce_ptr;
    LMP_CONNECTION_ENTITY *ce_ptr;

    /* Decrement the number of esco connections */
    if (lmp_self_device_data.number_of_esco_connections != 0)
    {
        lmp_self_device_data.number_of_esco_connections--;
    }

    esco_ce_ptr = &lmp_esco_connection_entity[esco_ce_index];

    ce_ptr = &lmp_connection_entity[esco_ce_ptr->ce_index];

#ifdef _BRUCE_IMPLEMENTED_NO_OF_ESCO_CONN_
    if (ce_ptr->no_of_esco_connections != 0)
    {
        ce_ptr->no_of_esco_connections--;
    }
#endif

#ifdef ENABLE_LOGGER_LEVEL_2
    ESCO_LOG_ERROR(LOG_LEVEL_LOW,INSTRUCTING_BASEBAND_TO_KILL_THE_ESCO_CONNECTION,0,0);
#endif

#ifdef _CCH_SLOT_OFFSET_
    lmp_put_global_slot_offset(esco_ce_ptr->ce_index,
                                LMP_MAX_CE_DATABASE_ENTRIES + esco_ce_index);
#endif

    esco_link_index = (UCHAR) (esco_ce_index + 1);

    con_am_addr = esco_ce_ptr->lt_addr << 5;

    /* Set the esco connection bit to 1 and make bit to 0 to kill connection */
    esco_bit = 1 << 9;

    make_esco_bit = 0 << 4;

    con_am_addr = (UINT16) ( con_am_addr | (esco_bit | make_esco_bit |
                                            esco_link_index | (ce_ptr->phy_piconet_id << 11) ) );

    {
        DEF_CRITICAL_SECTION_STORAGE;

        MINT_OS_ENTER_CRITICAL();

        BB_write_baseband_register(CONNECTOR_REGISTER, con_am_addr);

        /* Instruct the baseband to kill eSCO connection */
        BB_write_baseband_register(INSTRUCTION_REGISTER, BB_EXECUTE);
        MINT_OS_EXIT_CRITICAL();
    }

    /* Reset the State in the ESCO CE to indicate that BB connection is killed
     * for this ESCO link
     */
    esco_ce_ptr->bb_esco_connection = FALSE;

    /*If this link was created with the codec , then decrement the number of
      codec link
     */
    if(esco_ce_ptr->use_codec == TRUE)
    {
#ifdef _BRUCE_DROP_MUTED_ISO_OUT_PKTS_OVER_CODEC
        //reset
        read = BB_read_baseband_register(BB_MUTED_ISO_OUT_PKTS_CTRL1_REG);
        /*pcm_in_comp_amp bit[0:14] & pcm_in stream is muted bit[15] reset 0   */
        read &= 0x0000;
        BB_write_baseband_register(BB_MUTED_ISO_OUT_PKTS_CTRL1_REG,read);
        read = BB_read_baseband_register(BB_MUTED_ISO_OUT_PKTS_CTRL2_REG);
        /* the threshold of muted sample counts for pcm_in data */
        read &= 0x00;
        BB_write_baseband_register(BB_MUTED_ISO_OUT_PKTS_CTRL2_REG,read);
#endif

#ifdef _PLC_FUNC_ENABLE_
        if(IS_PLC_ENABLE)
        {
        UINT16 reg_val;
        //(bruce) setting 0xE0[11]=0,codec_plc_disable.
        reg_val = BB_read_baseband_register(VOICE_SETTING_REGISTER);
        reg_val &=(UINT16)~(BIT11);
        reg_val &=(UINT16)~(BIT3);
        BB_write_baseband_register(VOICE_SETTING_REGISTER,reg_val);
        }
#endif

        if(num_sync_links_over_codec>0)
        {
            num_sync_links_over_codec--;
        }

        esco_ce_ptr->use_codec = FALSE;
        /* Reset the FIFO Control register bits based on the FIFO used */
        fifo_config = BB_read_baseband_register(SYNC_FIFO_CONFIG_REGISTER);
        fifo_config = (UINT16)(fifo_config & (~SYNC_FIFO1_CONFIG_MASK));
        BB_write_baseband_register(SYNC_FIFO_CONFIG_REGISTER, fifo_config);


#ifdef ENABLE_PCM_ONLY_WHEN_SCO_LINK_CREATED
       lc_pcm_enable_control(0);// v5, v6
#endif

#ifdef  _SUPPORT_BT_CONTROL_PCM_MUX_
        // restore btgpio status
        restore_pcm_mux_state_when_disconnect();//v6
#endif

#ifdef _ROM_CODE_PATCHED_
#ifdef _BRUCE_8821B_PLC_RCP_
        if (rcp_lc_kill_esco_connect_over_codec_func != NULL)
        {
            rcp_lc_kill_esco_connect_over_codec_func();
        }
#endif
#endif
    }
    else
    {
        bz_isoch_remove_sco_queues(esco_ce_ptr->conn_handle, esco_ce_ptr->ce_index);
    }

    /* Reset the baseband register that indicates that the tx pkt type
     * is a multislot packet
     */
    temp_reg = 0xFEFF;
    temp_reg = (UINT16) ( temp_reg << (esco_link_index + 1 ) );
    temp_reg &= (~0x03FF);

    BB_write_baseband_register(ESCO_FIFO_FLUSH_LENGTH_AND_TX_PKT_LEN_REGISTER,
                               (UINT16)  (lc_var_esco_fifo_flush_length_and_tx_pkt_len_reg & temp_reg) );

    /* Reset the PTT bit in the upper LUT for EDR pkts. */
    address = reg_MASTER_UPPER_LUT[esco_ce_ptr->lt_addr];
    read = BB_read_baseband_register(address);
    read &= (~0x8000);
    BB_write_baseband_register( (UCHAR) address, read);
#ifdef _RESET_HW_LINK_INFO_TO_INIT_STATE_
    address = reg_MASTER_LOWER_LUT[esco_ce_ptr->lt_addr];
    BB_write_baseband_register(address, LC_INVALID_PACKET_TYPE);
#endif

    {
        UINT16 reg_val;

        reg_val = BB_read_baseband_register(SCO_PACKET_TYPE_REGISTER);
        if (lmp_check_sync_conn_bandwidth(TRUE) == API_FAILURE)
        {
            reg_val = (UINT16)(reg_val | BIT8); /* Set eSCO full-bw bit */
        }
        else
        {
            reg_val = (UINT16)(reg_val & ~BIT8); /* Clear eSCO full-bw bit */
        }
        BB_write_baseband_register(SCO_PACKET_TYPE_REGISTER, reg_val);
    }

    //lc_check_and_enable_scans_in_scatternet();

    return;
}

/**
 * Disables the retransmission window of the
 * esco links created on the ACL link given by \a acl_ce_index.
 *
 * \param acl_ce_index  ACL connection entity index.
 *
 * \return None.
 */
void lc_disable_esco_retx(UINT16 acl_ce_index)
{
    UCHAR index;
    UINT16 new_esco_window;
    UINT16 lt_addr;
    UINT16 esco_bit;
    DEF_CRITICAL_SECTION_STORAGE;

    LMP_ESCO_CONNECTION_ENTITY *esco_ce_ptr;

    for(index = 0; index < LMP_MAX_ESCO_CONN_ENTITIES; index++)
    {
        esco_ce_ptr = &lmp_esco_connection_entity[index];

        if((esco_ce_ptr->status != NO_CONNECTION) &&
                (esco_ce_ptr->ce_index == acl_ce_index))
        {
            /* Check if retransmission is enabled */
            if(esco_ce_ptr->wesco != 0)
            {
                /* set the esco bit */
                esco_bit = 1;
                esco_bit = (UINT16) (esco_bit << 9);
                /* Write the connector register with the esco LT ADDR */
                lt_addr = esco_ce_ptr->lt_addr;
                lt_addr <<= 5;

                /* Find the new esco window with retransmission disabled */
                new_esco_window = (UINT16)
                                  (esco_ce_ptr->esco_window - esco_ce_ptr->wesco);

                MINT_OS_ENTER_CRITICAL();
                BB_write_baseband_register(CONNECTOR_REGISTER,
                                           (UINT16) (lt_addr | esco_bit));

                /* Write the new esco window to the baseband */
                BB_write_baseband_register(ESCO_WINDOW_DESCO_REGISTER,
                                           (UINT16) (new_esco_window << 8));
                MINT_OS_EXIT_CRITICAL();
            } /* if(lmp_esco_connection_entity[index].wesco != 0) */
        } /* if((lmp_esco_connection_entity[index].status */
    } /* for(index = 0;index < LMP_MAX_ESCO_CONN_ENTITIES; index++) */
}


/**
 * Restore the retransmission window of the
 * esco links created on the ACL link given by \a acl_ce_index.
 *
 * \param acl_ce_index ACL connection entity index.
 *
 * \return None.
 */
void lc_restore_esco_retx(UINT16 acl_ce_index)
{
    UCHAR index;
    UINT16 esco_window;
    UINT16 lt_addr;
    UINT16 esco_bit;
    LMP_ESCO_CONNECTION_ENTITY *esco_ptr;
    DEF_CRITICAL_SECTION_STORAGE;

    for (index = 0; index < LMP_MAX_ESCO_CONN_ENTITIES; index++)
    {
        esco_ptr = &lmp_esco_connection_entity[index];

        if ((esco_ptr->status != NO_CONNECTION) &&
            (esco_ptr->ce_index == acl_ce_index))
        {
            /* Check if retransmission is enabled */
            if(esco_ptr->wesco != 0)
            {
                /* Set the esco bit */
                esco_bit = 1;
                esco_bit = (UINT16) (esco_bit << 9);
                /* Write the connector register with the esco LT ADDR */
                lt_addr = esco_ptr->lt_addr;
                lt_addr <<= 5;

                esco_window = esco_ptr->esco_window;

                MINT_OS_ENTER_CRITICAL();
                BB_write_baseband_register(CONNECTOR_REGISTER,
                                           (UINT16) (lt_addr | esco_bit));

                /* Updating esco window value in the Baseband */
                BB_write_baseband_register(ESCO_WINDOW_DESCO_REGISTER,
                                           (UINT16) (esco_window << 8));
                MINT_OS_EXIT_CRITICAL();
            }
        } /* if((esco_ptr->status != NO_CONNECTION) */
    } /* for(index = 0;index < LMP_MAX_ESCO_CONN_ENTITIES; index++) */
}

/**
 * Resets all the parameters of the ESCO scheduler.
 *
 * \param None.
 *
 * \return None.
 */
void lc_init_esco_scheduler(void)
{
    UINT16 index;

    ESCO_SCHEDULER_NODE *esco_node;

    ESCO_LOG_INFO(LOG_LEVEL_LOW,INITILIZING_THE_ESCO_SCHEDULER,0,0);

    esco_node = &esco_scheduler.esco_scheduler_queue;
    esco_node->active = INACTIVE;
    esco_node->esco_ce_index = INVALID_ESCO_INDEX;
    esco_node->ack_received = FALSE;
    esco_node->esco_instant = 0;
    esco_node->packet_received = FALSE;

    for (index = 0; index < 8; index++)
    {
        lc_esco_stop_reception[index] = FALSE;
        lc_esco_window[index] = FALSE;
        lc_esco_pkt_tx[index] = FALSE;
        global_esco_buf_valid[index] = FALSE;
        global_esco_data_correct[index] = FALSE;
#ifdef _ESCO_NEW_SCHEDULER_FOR_FULL_BW_
        lc_esco_window_overlap[index] = FALSE;
#endif
    }

    return;
}

/**
 * Queues the data packet \a esco_pkt to the eSCO connection entity.
 *
 * \param esco_pkt  Pointer to eSCO data packet.
 * \param esco_ce_index eSCO connection entity index.
 *
 * \return BT_FW_SUCCESS, if the operation is successful. BT_FW_ERROR,
 *         otherwise.
 */
UCHAR lc_queue_esco_data_pkt(HCI_SYNC_DATA_PKT *esco_pkt, UINT16 esco_ce_index)
{
    UCHAR write_index;

    LMP_ESCO_DATA_Q *esco_data_q_ptr;

    esco_data_q_ptr = &(lmp_esco_connection_entity[esco_ce_index].esco_data_q);

    if (esco_data_q_ptr->pending_length < MAX_ESCO_DATA_Q_LENGTH)
    {
        ESCO_LOG_INFO(LOG_LEVEL_LOW,QUEUING_THE_ESCO_DATA_PACKET_IN_SCHEDULER,0,0);

        DEF_CRITICAL_SECTION_STORAGE;

        MINT_OS_ENTER_CRITICAL();

        /* Take write index from connection entity */
        write_index = esco_data_q_ptr->write_index;

        /* Put data packet at the tail of queue */
        esco_data_q_ptr->esco_data_pkt_Q[write_index] = esco_pkt;

        /* Update write index of the queue*/
        esco_data_q_ptr->write_index++;
        if(esco_data_q_ptr->write_index >= MAX_ESCO_DATA_Q_LENGTH)
        {
            esco_data_q_ptr->write_index = 0;
        }

        /* Update length of queue */
        esco_data_q_ptr->pending_length++;

        MINT_OS_EXIT_CRITICAL();

#ifdef _NEW_ESCO_RX_ENABLE_DBG_
        RT_BT_LOG(YELLOW, ESCO_MSG_QUEUE_HCI_PKT, 5,
                                        esco_data_q_ptr->wait_cnt,
                                        esco_data_q_ptr->pending_length,
                                        esco_data_q_ptr->write_index,
                                        esco_data_q_ptr->read_index,
                                        esco_pkt);
#endif
    }
    else
    {
#ifdef TEST_MODE
        if (lmp_self_device_data.test_mode == HCI_REMOTE_LOOPBACK_MODE)
        {
            OS_FREE_BUFFER(synchronous_data_to_host_pool_id, (void *)esco_pkt);
            return BT_FW_SUCCESS;
        }
#endif

        dma_tx_fifo_pkt_free((void * )esco_pkt,
                        HCI_TRANSPORT_SYNC_DATA_PKT_TYPE);

        RT_BT_LOG(RED, ESCO_MSG_TXPKT_QUEUE_FULL, 4,
                                    esco_data_q_ptr->write_index,
                                    esco_data_q_ptr->read_index,
                                    esco_data_q_ptr->q_length,
                                    esco_data_q_ptr->pending_length);
    }

    return BT_FW_SUCCESS;
}

/**
 * Returns the largest eSCO instant of active eSCO connections.
 *
 * \param None.
 *
 * \return Largest eSCO instant value.
 */
UINT32 lc_esco_get_largest_instant(void)
{
    UCHAR index;
    UINT32 largest_instant = 0x0;
    UCHAR esco_ce_index = INVALID_ESCO_INDEX;
    LMP_ESCO_CONNECTION_ENTITY *esco_ce_ptr;

    for(index = 0; index < LMP_MAX_ESCO_CONN_ENTITIES; index ++)
    {
        esco_ce_ptr = &lmp_esco_connection_entity[index];

        if((esco_ce_ptr->entity_status == ASSIGNED) &&
                (esco_ce_ptr->bb_esco_connection == TRUE))
        {
            if(esco_ce_ptr->next_instant >= largest_instant)
            {
                esco_ce_index = index;
                largest_instant = esco_ce_ptr->next_instant;
                ESCO_LOG_INFO(LOG_LEVEL_LOW, FOUND_INSTANT_AT_INDEX,2,
                              largest_instant,esco_ce_index);
            }
        }
    }

    if(esco_ce_index != INVALID_ESCO_INDEX)
    {
        return largest_instant;
    }
    else
    {
        return 0;
    }
}

/**
 * Returns the esco_ce_index whose instant is
 * the least among all the active eSCO connection entities.
 *
 * \param None
 *
 * \return eSCO connection entity index of the least instant value.
 */
UCHAR lc_esco_get_least_instant(void)
{
    UCHAR index;
    UINT32 least_instant = 0xFFFFFFFF;
    UCHAR esco_ce_index = INVALID_ESCO_INDEX;
    LMP_ESCO_CONNECTION_ENTITY *esco_ce_ptr;

    for(index = 0; index < LMP_MAX_ESCO_CONN_ENTITIES; index ++)
    {
        esco_ce_ptr = &lmp_esco_connection_entity[index];

        if((esco_ce_ptr->entity_status == ASSIGNED) &&
                (esco_ce_ptr->bb_esco_connection == TRUE))
        {
            if(esco_ce_ptr->next_instant <= least_instant)
            {
                esco_ce_index = index;
                least_instant = esco_ce_ptr->next_instant;
                ESCO_LOG_INFO(LOG_LEVEL_LOW, FOUND_INSTANT_AT_INDEX,2,
                              least_instant,esco_ce_index);
            }
        }
    }

    if(esco_ce_index != INVALID_ESCO_INDEX)
    {
        return esco_ce_index;
    }
    else
    {
        ESCO_LOG_ERROR(LOG_LEVEL_LOW,THERE_ARE_NO_ESCO_ENTITIES_ASSIGNED,0,0);
        return INVALID_ESCO_INDEX;
    }
}

/**
 * Finds the difference between two 28 bit clocks and
 * it returns the first_clock - second clock value taking into
 * account the clock wrap around scenarios.
 *
 * \param exp_clk 28bit bluetooth clock value
 * \param cur_clk 28bit bluetooth clock value
 *
 * \return Difference between first_clk and second_clk.
 */

UINT32 lc_get_clock_diff(UINT32 exp_clk, UINT32 cur_clk, UINT8 lsb_num)
{
    UINT32 diff;

    if (exp_clk >= cur_clk)
    {
        /* (msb)-- exp_clk ---------- cur_clk --(lsb) BB_TIMER
                       |<------diff------>|                 */

        diff = exp_clk - cur_clk;
    }
    else
    {
        /* (msb)----cur_clk ---------- exp_clk -----(lsb) BB_TIMER
                |diff->|                   |<--diff-|       */
        diff = (0x10000000 >> lsb_num) - cur_clk + exp_clk;
    }

    diff &= (0x0FFFFFFF >> lsb_num); /* avoid wrap around */

    return diff;
}


void lc_reset_esco_fifo_new(void)
{
    UINT8 i;

    /* Check for any Esco connections on this link. */
    if (lmp_self_device_data.number_of_esco_connections != 0)
    {
        for (i = 0; i < LMP_MAX_ESCO_CONN_ENTITIES; i++)
        {
            if (lmp_esco_connection_entity[i].entity_status == ASSIGNED)
            {
                memset(&lmp_esco_connection_entity[i].esco_data_q, 0,
                       sizeof(LMP_ESCO_DATA_Q));
            }
        }
    }

    clear_sco_buffer();
}

void lc_load_esco_fifo_new(UCHAR esco_ce_index, UINT8 lag)
{
    UINT16 tx_len; /* my target tx pkt length */
    UINT16 acl_ce_index;
    UINT16 length_sent = 0;
    UCHAR *buffer = NULL;
    UCHAR rptr;
    UCHAR packet_length;
    UCHAR frag_data_ptr;
    LMP_ESCO_DATA_Q *esco_data_q_ptr;
    LMP_ESCO_CONNECTION_ENTITY *esco_ce_ptr;
    UINT16 dma_len = 0;

    esco_ce_ptr = &lmp_esco_connection_entity[esco_ce_index];
    esco_data_q_ptr = &esco_ce_ptr->esco_data_q;
    acl_ce_index = esco_ce_ptr->ce_index;

    /* decide sent packet length */
    if (lmp_connection_entity[acl_ce_index].remote_dev_role == MASTER)
    {
        tx_len = esco_ce_ptr->s_to_m_packet_length;
    }
    else
    {
        tx_len = esco_ce_ptr->m_to_s_packet_length;
    }

    if (lmp_connection_entity[acl_ce_index].trx_codec_conv_type !=
                                            BZDMA_CODEC_TX_CONV_TYPE_BYPASS)
    {
        /* this means the voice data needs to convert via codec */
        if (!lmp_connection_entity[acl_ce_index].txfifo_in_8bit_code)
        {
            /* double data length to meet constant rate */
            tx_len <<= 1;
        }
    }

    /* fill the tx descriptor of BZDMA to reassamble or fragment sent BB pkt
       from the pkt list of pending hci synchronous pkt */
    BZDMA_TX_DESC_SEGMENT txdesc[BZDMA_TX_ENTRY_MAX_SEGS_SCO];
    UINT8 seg_idx = 0;

#ifdef _NEW_ESCO_RX_ENABLE_DBG_
    UINT8 ori_rptr = esco_data_q_ptr->read_index;
    UINT8 ori_pend_len = esco_data_q_ptr->pending_length;
    UINT16 ori_frag = esco_data_q_ptr->fragmented_data_ptr;
#endif

    esco_data_q_ptr->wait_index = esco_data_q_ptr->read_index;

    while (1)
    {
        if (tx_len == 0)
        {
            break;
        }

        /* load hci synchronous pkt info from eSCO data pkt Queue */
        rptr = esco_data_q_ptr->read_index;

        if (esco_data_q_ptr->esco_data_pkt_Q[rptr] != NULL)
        {
            /* there is a pending pkt in queue */
            buffer = esco_data_q_ptr->esco_data_pkt_Q[rptr]->hci_sync_data_packet;
            packet_length = esco_data_q_ptr->esco_data_pkt_Q[rptr]->packet_length;
            frag_data_ptr = esco_data_q_ptr->fragmented_data_ptr;
            buffer += frag_data_ptr;
        }
        else
        {
            /* there is no pending pkt in queue */
            break;
        }

        /* Check if enough bytes is avaliable to write to FIFO */
        if((packet_length - frag_data_ptr) < tx_len)
        {
            /* the remain payload length in current pkt is not enough */
            dma_len = packet_length - frag_data_ptr;
        }
        else
        {
            dma_len = tx_len;
        }
        length_sent += dma_len;
        tx_len -= dma_len;

        /* Update the fragment pointer in the ESCO_CE */
        esco_data_q_ptr->fragmented_data_ptr += dma_len;

        if (lag == FALSE)
        {
            txdesc[seg_idx].DWord0 = 0;
            txdesc[seg_idx].DWord1 = 0;
            txdesc[seg_idx].start_addr = (UINT32)buffer;
            txdesc[seg_idx].len = dma_len;
        }

        seg_idx++;

        /* If all the fragmens have been loaded to the FIFO and scheduler
         * then increment the read_index
         */
        if(esco_data_q_ptr->fragmented_data_ptr >= packet_length)
        {
            ESCO_LOG_INFO(LOG_LEVEL_LOW,ALL_FRAGMENTS_WRITTEN_INCREMENTING_READ_INDEX,0,0);
            esco_data_q_ptr->read_index++;
            if (esco_data_q_ptr->read_index >= MAX_ESCO_DATA_Q_LENGTH)
            {
                esco_data_q_ptr->read_index = 0;
            }
            esco_data_q_ptr->fragmented_data_ptr = 0;
            esco_ce_ptr->esco_data_q.pending_length--;
            esco_data_q_ptr->wait_cnt++;

            if (lag)
            {
                lc_handle_esco_free_pkts_callback_new(esco_ce_index);
            }
        }

        if (seg_idx >= BZDMA_TX_ENTRY_MAX_SEGS_SCO)
        {
            break;
        }
    }

#ifdef _FILL_ZEROS_TO_ESCO_TX_PACKET_
    UINT8 j;
    UINT8 i = 0;
    BZDMA_TX_DESC_SEGMENT txdesc_new[BZDMA_TX_ENTRY_MAX_SEGS_SCO];

    /* fill zero to esco tx fifo */
    if ((tx_len > 0) && (lag == FALSE))
    {
        length_sent += tx_len;

        /* fill zero to the buffer */
        while (tx_len != 0)
        {
            txdesc_new[i].DWord0 = 0;
            txdesc_new[i].DWord1 = 0;
            txdesc_new[i].start_addr = (UINT32)sco_tx_zero_buf;

            if (tx_len > 64)
            {
                tx_len -= 64;
                txdesc_new[i].len = 64;
            }
            else
            {
                txdesc_new[i].len = tx_len;
                tx_len = 0;
            }
            i++;
        }
    }
#endif

    if ((length_sent > 0) && (lag == FALSE))
    {
#ifdef _FILL_ZEROS_TO_ESCO_TX_PACKET_
        for (j = 0; j < seg_idx; j++)
        {
            txdesc_new[i].DWord0 = txdesc[j].DWord0;
            txdesc_new[i].DWord1 = txdesc[j].DWord1;
            i++;
        }
        seg_idx = i;

        /* Update the number of free bytes in the ESCO Scheduler */
        txdesc_new[seg_idx - 1].isLast = TRUE;
        BB_dma_write_baseband_TX_FIFO(txdesc_new, seg_idx, 0, 0, 1, esco_ce_index);
#else
        /* Update the number of free bytes in the ESCO Scheduler */
        txdesc[seg_idx - 1].isLast = TRUE;
        BB_dma_write_baseband_TX_FIFO(txdesc, seg_idx, 0, 0, 1, esco_ce_index);
#endif
    }

#ifdef _NEW_ESCO_RX_ENABLE_DBG_
    RT_BT_LOG(RED, ESCO_MSG_LOAD_FIFO, 8,
                    ori_pend_len, esco_data_q_ptr->pending_length,
                    ori_rptr, esco_data_q_ptr->read_index,
                    ori_frag, esco_data_q_ptr->fragmented_data_ptr,
                    seg_idx, length_sent);
#endif
}

void lc_add_to_esco_scheduler_new(UCHAR esco_ce_index)
{
    ESCO_SCHEDULER_NODE *esco_node;

    esco_node = &esco_scheduler.esco_scheduler_queue;
    esco_node->esco_ce_index = esco_ce_index;
    esco_node->ack_received = FALSE;
    esco_node->esco_instant = lmp_esco_connection_entity[esco_ce_index].next_instant;
}

UINT8 lc_esco_scheduler_new(UCHAR esco_ce_index)
{
    UINT16 ce_index;
    UINT32 current_clock;
    UINT32 next_instant;
#ifndef ESCO_OVER_HCI_PUMP_DATA
    UCHAR esco_data_q_length;
#endif
    LMP_ESCO_CONNECTION_ENTITY *esco_ce_ptr;
    UINT8 pkt_type = DEFAULT_PACKET;
    UINT8 lag = FALSE;
    UINT8 exit = FALSE;

    esco_ce_ptr = &lmp_esco_connection_entity[esco_ce_index];

    /* Get the current clock and the next instant */
    ce_index = esco_ce_ptr->ce_index;

#ifndef _USE_ONE_NEW_BZDMA_TXCMD_FOR_SCO_
    /* Flush whole sco tx fifo */
    lc_var_esco_fifo_flush_length_and_tx_pkt_len_reg &= 0xFC00;

    BB_flush_baseband_SYNC_TX_FIFO(SYNC_FIFO1,
                lc_var_esco_fifo_flush_length_and_tx_pkt_len_reg);
#endif /* end of #ifndef _USE_ONE_NEW_BZDMA_TXCMD_FOR_SCO_ */

    lc_get_clock_in_scatternet(&current_clock,
                         lmp_connection_entity[ce_index].phy_piconet_id);

#ifndef _FAST_SCO_ESCO_RE_SYNC_ANCHOR_POINT_
    current_clock = current_clock >> 1; /* 625 us in unit */
#endif

    while (exit != TRUE)
    {
#ifndef _FAST_SCO_ESCO_RE_SYNC_ANCHOR_POINT_
        next_instant = esco_ce_ptr->next_instant;

        if (lc_get_clock_diff(next_instant, current_clock, 1) > esco_ce_ptr->tesco)
        {
            lag = TRUE;
        }
        else
        {
            lag = FALSE;
            exit = TRUE;
        }
#else
        next_instant = esco_ce_ptr->next_instant << 1;

        /* use SCO likly process (austin) */
        UINT32 diff = lc_get_clock_diff(next_instant, current_clock, 0);
        if (diff & BIT27)
        {
            lag = TRUE;
        }
        else
        {
            lag = FALSE;
            exit = TRUE;
        }
#endif

#ifndef ESCO_OVER_HCI_PUMP_DATA
        /* Get the number of data that is not yet loaded to FIFO/Scheduler and
         * pending in the ESCO CE queue
         */
        esco_data_q_length = esco_ce_ptr->esco_data_q.pending_length;
#endif

#ifndef ESCO_OVER_HCI_PUMP_DATA
#ifdef _NEW_ESCO_RX_ENABLE_DBG_
        RT_BT_LOG(RED, ESCO_MSG_SCHEDUAL_NEW, 3, esco_data_q_length,
                                        next_instant, current_clock);
#endif
#endif

        /* Check if any esco data pkt is pending in the esco data queue */
        if ((esco_ce_ptr->use_codec == FALSE)
#ifndef ESCO_OVER_HCI_PUMP_DATA
                && (esco_data_q_length > 0)
#endif
          )
        {
            /* Load the ESCO fifo with the ESCO packet */
            lc_load_esco_fifo_new(esco_ce_index, lag);

            if (!lag)
            {
                pkt_type = FIFO_PACKET;
            }
        }
#ifdef _FAST_SCO_ESCO_RE_SYNC_ANCHOR_POINT_
        else if (lag)
        {
            /* speed up quit time */
            esco_ce_ptr->next_instant = (current_clock + 1 + (esco_ce_ptr->tesco << 1)) >> 1;
            esco_ce_ptr->next_instant &= 0x07FFFFFF;
            break;
        }
#endif

        /* Update next instant */
        esco_ce_ptr->next_instant += esco_ce_ptr->tesco;
        esco_ce_ptr->next_instant &= 0x07FFFFFF;
    }

    if (pkt_type == FIFO_PACKET)
    {
        return TRUE;
    }
    else
    {
        return FALSE;
    }
}

void lc_init_esco_windows_at_instant(UCHAR esco_lt_addr, UINT16 esco_ce_index)
{
    LMP_ESCO_CONNECTION_ENTITY *esco_ce_ptr;
    UINT16 read;
    ESCO_SCHEDULER_NODE *sche_q = &esco_scheduler.esco_scheduler_queue;

    esco_ce_ptr = &lmp_esco_connection_entity[esco_ce_index];

    lc_esco_window[esco_lt_addr] = TRUE;
    lc_esco_pkt_tx[esco_lt_addr] = FALSE;

    /* Reset the Stop reception bit */
    lc_esco_stop_reception[esco_lt_addr] = FALSE;

    read = BB_read_baseband_register(ESCO_STOP_RX_REGISTER);
    read &= ESCO_ENABLE_RX;
    BB_write_baseband_register(ESCO_STOP_RX_REGISTER, read);


    /* Add an entry for Default Packet to the scheduler */
    lc_add_to_esco_scheduler_new(esco_ce_index);

    /* If start of instant is received for an already active ESCO window
     * It means that it is the case when the ESCO link is continous.
     * This status is set here and the window is made inactive depending
     * on this in the ESCO window end interrupt.
     */
    if (esco_ce_ptr->esco_window_status == ACTIVE)
    {
        esco_ce_ptr->esco_continous_status = ACTIVE;
    }
    else
    {
        esco_ce_ptr->esco_window_status = ACTIVE;
    }

    /* Update the status in the ESCO CE */
    sche_q->packet_received = FALSE;
    sche_q->active = ACTIVE;
}

void lc_handle_esco_instant_interrupt_new(UCHAR esco_lt_addr, UINT16 ce_index)
{
    UINT16 esco_ce_index;
    UINT16 length;
    UCHAR packet_type;
    UINT16 lower_lut_address;
    UINT16 upper_lut_address;
    UINT16 lower_lut;
    UINT16 upper_lut;
    UINT16 read;

#ifdef _ROM_CODE_PATCHED_
    if (rcp_lc_handle_esco_instant_interrupt_new != NULL)
    {
        if (rcp_lc_handle_esco_instant_interrupt_new( (void *)&esco_lt_addr, ce_index))
        {
            return;
        }
    }
#endif

#ifdef _BRUCE_DROP_MUTED_ISO_OUT_PKTS_OVER_CODEC
    UINT16 is_PCM_in_mute_stream = 0;
#endif

#ifdef _BRUCE_TEST_PLC_PKT_STATUS
            UINT16 read_temp;
            read=BB_read_baseband_register(0x25c); //0x25c[7]=0  hw  monitoring pkt miss;
            read_temp=(UINT16)((read >>7) & 01);
            if(read_temp)
            {   //0x25c[7]=0 ;pkt correct

#ifdef _BRUCE_RECORD_ERASECNT_STATUS
                if(plc_pkt_status_var.g_burst_error_flag)
                {
                    plc_pkt_status_var.g_burst_error_flag=0;
                }
                if(plc_pkt_status_var.g_record_flag)
                {
                    plc_pkt_status_var.g_record_cnt=0;
                    plc_pkt_status_var.g_record_flag=0;
                    plc_pkt_status_var.g_burst_error_flag=1;
                }
#endif
            }
            else
            {   //0x25c[7]=0 ;pkt miss
#ifdef _BRUCE_RECORD_ERASECNT_STATUS
                if(plc_pkt_status_var.g_burst_error_flag)
                {
                    plc_pkt_status_var.g_plc_pkt_miss_burst_error++;
                    plc_pkt_status_var.g_burst_error_flag=0;
                }
                if(plc_pkt_status_var.g_record_flag)
                {
                    plc_pkt_status_var.g_record_cnt++;
                }
                plc_pkt_status_var.g_record_flag=1;
                switch (plc_pkt_status_var.g_record_cnt)
                {
                case 2:
                    plc_pkt_status_var.g_plc_pkt_miss_erasecnt_2++;
                    break;
                case 3:
                    plc_pkt_status_var.g_plc_pkt_miss_erasecnt_3++;
                    break;
                case 5:
                    plc_pkt_status_var.g_plc_pkt_miss_erasecnt_5++;
                    break;
                default:
                    break;
                }
#endif
                plc_pkt_status_var.g_plc_HW_pkt_miss++;
#ifdef _BRUCE_DEBUG_PORT
                read=BB_read_baseband_register(0x25c); // 0x25c[3]=1; g_plc_HW_pkt_miss
                read |= (UINT16)BIT3;
                BB_write_baseband_register(0x25c,read);
#endif
            }
#endif


    LMP_ESCO_CONNECTION_ENTITY *esco_ce_ptr;
    UINT32 current_clock;
    UINT32 next_instant;
    UINT8 fill_data = FALSE;

    /* Get the ESCO Connection entity from the am_address */
    if (lmp_get_esco_ce_index_from_am_addr(esco_lt_addr,
                              lmp_connection_entity[ce_index].phy_piconet_id,
                              &esco_ce_index) != API_SUCCESS)
    {
        ESCO_LOG_ERROR(LOG_LEVEL_LOW,
                        COULD_NOT_FIND_ESCO_CE_FOR_LT_ADDR,1,esco_lt_addr);
        return;
    }

#ifdef _ESCO_NEW_SCHEDULER_FOR_FULL_BW_
    if (lc_esco_window[esco_lt_addr] == TRUE)
    {
        lc_esco_window_overlap[esco_lt_addr] = TRUE;
    }
    else
    {
        lc_esco_window_overlap[esco_lt_addr] = FALSE;
        lc_init_esco_windows_at_instant(esco_lt_addr, esco_ce_index);
    }
#else
    lc_init_esco_windows_at_instant(esco_lt_addr, esco_ce_index);
#endif

    esco_ce_ptr = &lmp_esco_connection_entity[esco_ce_index];

    /* reset tx queue of fw/hw if alternative setting is changed */
    if (alternatset_change_tx_clear)
    {
        lc_reset_esco_fifo_new();
    }

    /* Check if any hci synchronous pkts are waiting in the pending list */
    if (esco_ce_ptr->esco_data_q.pending_length == 0)
    {
        lc_get_clock_in_scatternet(&current_clock,
                         lmp_connection_entity[ce_index].phy_piconet_id);

#ifndef _FAST_SCO_ESCO_RE_SYNC_ANCHOR_POINT_
        current_clock = current_clock >> 1;

        while (1)
        {
            next_instant = esco_ce_ptr->next_instant;

            /* Update next instant */
            esco_ce_ptr->next_instant += esco_ce_ptr->tesco;
            esco_ce_ptr->next_instant &= 0x07FFFFFF;

            if (lc_get_clock_diff(next_instant, current_clock, 1) < esco_ce_ptr->tesco)
            {
                break;
            }
        }
#else
        while (1)
        {
            next_instant = esco_ce_ptr->next_instant << 1;

            /* use SCO likly process (austin) */
            UINT32 diff = lc_get_clock_diff(next_instant, current_clock, 0);
            if (diff & BIT27)
            {
                /* speed up quit time */
                esco_ce_ptr->next_instant = (current_clock + 1 + (esco_ce_ptr->tesco << 1)) >> 1;
                esco_ce_ptr->next_instant &= 0x07FFFFFF;
                break;
            }
            else
            {
                /* Update next instant */
                esco_ce_ptr->next_instant += esco_ce_ptr->tesco;
                esco_ce_ptr->next_instant &= 0x07FFFFFF;
                break;
            }
        }
#endif
    }
    else
    {
        /* trigger eSCO scheduler */
        fill_data = lc_esco_scheduler_new(esco_ce_index);
    }

#ifdef _BRUCE_DROP_MUTED_ISO_OUT_PKTS_OVER_CODEC
    if (esco_ce_ptr->use_codec == TRUE)
    {
         /*BB_MUTED_ISO_OUT_PKTS_CTRL1_REG bit[15],
                   indicates the pcm_in stream is muted
                   0: Inside infor is not muted
                   1: Inside infor is muted*/
        read = BB_read_baseband_register(BB_MUTED_ISO_OUT_PKTS_CTRL1_REG);
        is_PCM_in_mute_stream = (read >> 15) & 0x01;
    }
#endif

    /* If FIFO is not loaded for this instant then load default packet
     * else load the packet as in the ESCO CE
     * Load the LUT with the ESCO packet from the scheduler for this Instant
     */
    if (lmp_connection_entity[esco_ce_ptr->ce_index].remote_dev_role == MASTER)
    {
        if (IS_USE_FOR_BQB ||
            (fill_data == TRUE) ||
            (esco_ce_ptr->use_codec == TRUE) ||
            (IS_USE_FOR_MUTE)
            )
        {
#ifdef _BRUCE_DROP_MUTED_ISO_OUT_PKTS_OVER_CODEC
            if(is_PCM_in_mute_stream)
            {  //  send NULL pkt
                length = 0;
                packet_type = BB_NULL;
            }
            else
            {
                /* tx data is ready */
                length = esco_ce_ptr->s_to_m_packet_length;
                packet_type = esco_ce_ptr->s_to_m_packet_type;
            }
#else
            /* tx data is ready */
            length = esco_ce_ptr->s_to_m_packet_length;
            packet_type = esco_ce_ptr->s_to_m_packet_type;
#endif
        }
        else
        {
            /* tx data is not ready */
            length = 0;
            packet_type = BB_NULL;
        }
    }
    else
    {
        if (IS_USE_FOR_BQB ||
            (fill_data == TRUE) ||
            (esco_ce_ptr->use_codec == TRUE) ||
            (IS_USE_FOR_MUTE)
#ifdef _DAPE_TEST_GEN_FAKE_ESCO_DATA
            ||(g_gen_fake_esco_data)
#endif
            )
        {
#ifdef _BRUCE_DROP_MUTED_ISO_OUT_PKTS_OVER_CODEC
            if(is_PCM_in_mute_stream)
            {  //  send POLL pkt
                length = 0;
                packet_type = BB_POLL;
            }
            else
            {
                /* tx data is ready */
                length = esco_ce_ptr->m_to_s_packet_length;
                packet_type = esco_ce_ptr->m_to_s_packet_type;
            }
#else
            /* tx data is ready */
            length = esco_ce_ptr->m_to_s_packet_length;
            packet_type = esco_ce_ptr->m_to_s_packet_type;
#endif
        }
        else
        {
            /* tx data is not ready */
            length = 0;
            packet_type = BB_POLL;
        }
    }

    lower_lut_address = lut_ex_table[esco_lt_addr].lower_lut_address;
    upper_lut_address = lut_ex_table[esco_lt_addr].upper_lut_address;
    lower_lut = (UINT16) ( (packet_type << 12) | length);

    /* Set the Flow bit to 1 , ESCO pkts should be transmitted with Flow = 1
     * and the flow should be ignored on reception
     */
    upper_lut = BB_read_baseband_register(upper_lut_address);
    upper_lut |= 0x01;
    upper_lut &= ~BIT2; /* Current HW can not reset ARQN bit automatically.
                           So set it to NACK first to avoid false
                           acknowledgement before next eSCO instant is coming
                           - austin */
    BB_write_baseband_register(upper_lut_address, upper_lut);
    BB_write_baseband_register(lower_lut_address, lower_lut);

    return;
}

void lc_handle_esco_window_expiry_interrupt_new(UCHAR esco_lt_addr,UCHAR piconet_id)
{
    UINT16 esco_ce_index;
    UINT16 rx_packet_length;
    UINT16 ce_index;
    LMP_ESCO_CONNECTION_ENTITY *esco_ce_ptr;
    ESCO_SCHEDULER_NODE *sche_q = &esco_scheduler.esco_scheduler_queue;
    OS_SIGNAL signal;

#ifdef _CCH_ESCO_STOP_NULL_
    UINT16 lower_lut_address;
#endif

    /* Get the ESCO Connection entity from the lt_address */
    if (lmp_get_esco_ce_index_from_am_addr(esco_lt_addr,piconet_id,
                                          &esco_ce_index) != API_SUCCESS)
    {
        ESCO_LOG_ERROR(LOG_LEVEL_LOW,
                      COULD_NOT_FIND_ESCO_CE_FOR_LT_ADDR, 1, esco_lt_addr);
        return;
    }
    esco_ce_ptr = &lmp_esco_connection_entity[esco_ce_index];

    /* Set esco_window to FALSE only if the link is not continuous. */
    if (esco_ce_ptr->esco_continous_status != ACTIVE)
    {
        lc_esco_window[esco_lt_addr] = FALSE;
    }

    lc_esco_pkt_tx[esco_lt_addr] = FALSE;

    ce_index = esco_ce_ptr->ce_index;

    if (lmp_connection_entity[ce_index].remote_dev_role == MASTER)
    {
        rx_packet_length = esco_ce_ptr->m_to_s_packet_length;
    }
    else
    {
        rx_packet_length = esco_ce_ptr->s_to_m_packet_length;
    }

    /* If a window is ending then send the packet which was
     * received in the esco window or corresponding error status
     * - send a signal to this effect.
     */
    signal.type = LC_HANDLE_ESCO_PACKET;
    signal.param = (OS_ADDRESS) ((rx_packet_length << 8) | esco_ce_index);
    signal.ext_param = (OS_ADDRESS)(0xFFFF);
    OS_ISR_SEND_SIGNAL_TO_TASK(lc_rx_task_handle, signal);

    /* Check if this eSCO window is active in the ESCO Connection Entity */
    if (esco_ce_ptr->esco_window_status == ACTIVE)
    {
        /* Send ACK signal only if a payload was transmitted in the current
         * ESCO window.
         */

        /* Make the eSCO window status in the ESCO CE to INACTIVE */
        /* If it is a continous ESCO link then just reset the continous
         * status as the end of window has not yet come for the second window
         */
        if (esco_ce_ptr->esco_continous_status == ACTIVE)
        {
            esco_ce_ptr->esco_continous_status = INACTIVE;
        }
        else
        {
            esco_ce_ptr->esco_window_status = INACTIVE;
        }
        sche_q->active = INACTIVE;

#ifdef _CCH_ESCO_STOP_NULL_
        if(( (esco_ce_ptr->tesco) >= (esco_ce_ptr->esco_window + 2) ) &&
            ( esco_ce_ptr->status == ESCO_CONNECTED))
        {
            lower_lut_address = lut_ex_table[esco_lt_addr].lower_lut_address;
            BB_write_baseband_register(lower_lut_address, LC_SLAVE_DEFAULT_PACKET_TYPE);
        }
#endif
    }

#ifdef _USE_ONE_NEW_BZDMA_TXCMD_FOR_SCO_
        UINT8 txcmd_id;
        txcmd_id = BZDMA_TX_ENTRY_TYPE_NEW_SCO0;
        UINT8 result = FALSE;
        if ((Bzdma_Manager.TxEntSta[txcmd_id].used==1)&&
            (lc_esco_window_overlap[esco_lt_addr] == FALSE))
        {
            /* free esco tx pkt */
            lc_handle_esco_free_pkts_callback_new(Bzdma_Manager.TxEntSta[txcmd_id].ch_id);

            /* invalidate the bzdma */
            result=bzdma_invalid_txcmd(txcmd_id, 0, Bzdma_Manager.TxEntSta[txcmd_id].ch_id);

            /* reset tx queue of fw/hw if alternative setting is changed */
            if (alternatset_change_tx_clear)
            {
                lc_reset_esco_fifo_new();
            }
            RT_BT_LOG(RED,BRUCE_TEST_7018,4,lc_esco_window_overlap[esco_lt_addr],
            Bzdma_Manager.TxEntSta[txcmd_id].used,
            Bzdma_Manager.bmFreeTxEnt,result);
        }

#endif

#ifdef _ESCO_NEW_SCHEDULER_FOR_FULL_BW_
    if (lc_esco_window_overlap[esco_lt_addr] == TRUE)
    {
        lc_init_esco_windows_at_instant(esco_lt_addr, esco_ce_index);
        lc_esco_window_overlap[esco_lt_addr] = FALSE;
    }
#endif

}

void lc_handle_esco_free_pkts_callback_new(UCHAR esco_ce_index)
{
    LMP_ESCO_DATA_Q *esco_data_q;
    UCHAR wait_index;
    LMP_ESCO_CONNECTION_ENTITY *esco_ce_ptr;
    HCI_SYNC_DATA_PKT *ppkt;

    esco_ce_ptr = &lmp_esco_connection_entity[esco_ce_index];

    if (esco_ce_ptr->entity_status != ASSIGNED)
    {
        return;
    }

    if (esco_ce_ptr->use_codec == TRUE)
    {
        return;
    }

    esco_data_q = &(esco_ce_ptr->esco_data_q);


    DEF_CRITICAL_SECTION_STORAGE;

    MINT_OS_ENTER_CRITICAL();

    /* dequeue acked or expired eSCO tx packets */
    while (esco_data_q->wait_cnt)
    {
        /* Update the Fragment Pointer */
        wait_index = esco_data_q->wait_index;
        ppkt = esco_data_q->esco_data_pkt_Q[wait_index];

        if (ppkt == NULL)
        {
            break;
        }

#ifdef TEST_MODE
        if (lmp_self_device_data.test_mode == HCI_REMOTE_LOOPBACK_MODE)
        {
            OS_FREE_BUFFER(synchronous_data_to_host_pool_id, (void * )ppkt);
        }
        else
#endif
        {
#ifdef _DAPE_TEST_GEN_FAKE_ESCO_DATA
            if (g_gen_fake_esco_data)
            {
                gen_esco_data_packet(FALSE);
            }
            else
            {
                /* Free the ESCO Data Buffer */
                dma_tx_fifo_pkt_free((void *)ppkt,HCI_TRANSPORT_SYNC_DATA_PKT_TYPE);
            }
#else

            /* Free the ESCO Data Buffer */
            dma_tx_fifo_pkt_free((void *)ppkt,HCI_TRANSPORT_SYNC_DATA_PKT_TYPE);
#endif
        }

        esco_data_q->esco_data_pkt_Q[wait_index] = NULL;

        /* Increment the Wait Index */
        esco_data_q->wait_index++;
        if (esco_data_q->wait_index >= MAX_ESCO_DATA_Q_LENGTH)
        {
            esco_data_q->wait_index = 0;
        }

        /* Send Num Complete fro ESCO packet*/
        esco_ce_ptr->num_of_completed_esco_packets++;

        esco_data_q->wait_cnt--;
    }

#ifdef TEST_MODE
    if (lmp_self_device_data.test_mode == HCI_REMOTE_LOOPBACK_MODE)
#endif
    {
#ifdef _DAPE_TEST_GEN_FAKE_ESCO_DATA
        if (!g_gen_fake_esco_data)
        {
            /* Num complete events will be generated from here */
            hci_generate_number_of_completed_packets_event();
        }
#else
        /* Num complete events will be generated from here */
        hci_generate_number_of_completed_packets_event();
#endif
    }

    MINT_OS_EXIT_CRITICAL();
}

/**
 * Handles the receive interrupt on an esco lt_address.
 *
 * \param esco_lt_addr LT_ADDR on which the packet received.
 * \param piconet_id Piconet id in which the packet was received.
 * \param packet_header Packet Header of the received packet.
 *
 * \return None.
 */

#ifdef _CCH_SC_ECDH_P256_MIC_FLAG
void lc_handle_esco_rx_interrupt(UCHAR esco_lt_addr, UCHAR piconet_id,
                                 UINT16 packet_header, UINT16 payload_header)
#else
void lc_handle_esco_rx_interrupt(UCHAR esco_lt_addr, UCHAR piconet_id,
                                 UINT16 packet_header)
#endif
{
    UINT16 ce_index;
    UINT16 esco_ce_index;
    UINT16 read;
    OS_SIGNAL signal;
    UINT16 upper_lut;
    UINT16 rx_packet_length;
    UINT16 new_packet = FALSE;
#ifndef _DAPE_TEST_NEW_HW_READ_RSSI_STACK
#ifdef COMPILE_RSSI_REPORTING
    UINT16 rssi = 0;
#endif
#endif
    UINT16 lower_lut_address;
    UINT16 upper_lut_address;
    ESCO_SCHEDULER_NODE *esco_scheq;
    BZ_REG_S_RX_STATUS *rx_status = (BZ_REG_S_RX_STATUS *)&packet_header;
#ifdef _CCH_SC_ECDH_P256_MIC_FLAG
    BZ_REG_S_RX_PL_HDR *pl_hdr = (BZ_REG_S_RX_PL_HDR *)&payload_header;
#endif
    ESCO_LOG_INFO(LOG_LEVEL_LOW,
                    ESCO_REVEIVE_INTERRUPT_LT_ADDR_PICONET_ID_PACKET_HEADER,3,
                    esco_lt_addr,piconet_id,packet_header);

    if (lmp_self_device_data.number_of_esco_connections == 0)
    {
        return;
    }

    /* Get the ESCO Connection entity from the lt_address */
    if(lmp_get_esco_ce_index_from_am_addr(esco_lt_addr,piconet_id,
                                          &esco_ce_index) != API_SUCCESS)
    {
        ESCO_LOG_ERROR(LOG_LEVEL_LOW,COULD_NOT_FIND_ESCO_CE_FOR_LT_ADDR,
                      1,esco_lt_addr);
        return;
    }

    ce_index = lmp_esco_connection_entity[esco_ce_index].ce_index;

#ifndef _DAPE_TEST_NEW_HW_READ_RSSI_STACK
#ifdef COMPILE_RSSI_REPORTING
    rssi = lc_read_rssi_register();
    lc_calculate_and_store_rssi(rssi, ce_index);
#endif
#endif
    esco_scheq = &esco_scheduler.esco_scheduler_queue;

    /* Check if ACK is received for the transmitted packet */
    if (rx_status->arqn && (lc_esco_pkt_tx[esco_lt_addr] == TRUE))
    {
        /* Set Ack received in the scheduler at read_ptr */
        esco_scheq->ack_received = TRUE;
        lower_lut_address = lut_ex_table[esco_lt_addr].lower_lut_address;
        BB_write_baseband_register(lower_lut_address, LC_SLAVE_DEFAULT_PACKET_TYPE);
    }

    if(lmp_connection_entity[ce_index].remote_dev_role == MASTER)
    {
        rx_packet_length =
            lmp_esco_connection_entity[esco_ce_index].m_to_s_packet_length;
    }
    else
    {
        rx_packet_length =
            lmp_esco_connection_entity[esco_ce_index].s_to_m_packet_length;
    }

    /* Set indication in scheduler that packet was received */
    if ((1 << rx_status->pkt_type) & BB_RX_ISR_ESCO_DATA_PKT)
    {
        if (esco_scheq->packet_received == FALSE)
        {
            new_packet = 1;
            esco_scheq->packet_received = TRUE;
        }
        else
        {
            new_packet = 0;
        }
    }

    /* Check if the received packet is having CRC Error */
#ifdef _CCH_SC_ECDH_P256_MIC_FLAG
    if ( (rx_status->crc_err) || (pl_hdr->mic_err) )
#else
    if (rx_status->crc_err)
#endif
    {
        ESCO_LOG_INFO(LOG_LEVEL_LOW,PACKET_RECD_WITH_CRC_ERROR_PACKET_HEADER,1,
                      packet_header);

        /* Baseband will send NAK for this packet */
        /* Send Packet received signal only if it is a EV Packet */
        if ((1 << rx_status->pkt_type) & BB_RX_ISR_ESCO_DATA_PKT)
        {
            if(lc_esco_stop_reception[esco_lt_addr] == FALSE)
            {
                /* Send Signal to handle the received ESCO Packet,
                 * ext_param is set to indicate that if is a duplicate
                 * and if it is received with CRC error
                 */
                signal.type = LC_HANDLE_ESCO_PACKET;
                signal.param = (OS_ADDRESS)((rx_packet_length << 8) | esco_ce_index);
                signal.ext_param = (OS_ADDRESS)(new_packet << 8);
                OS_ISR_SEND_SIGNAL_TO_TASK(lc_rx_task_handle, signal);
            }
        }
    }
    else
    {
        /* CRC is correct */
        /* Set the Stop reception bit in the Baseband */
        /* This bit will stop the baseband from writing
           further ESCO packets in this window into the rx-FIFO. */
        read = BB_read_baseband_register(ESCO_STOP_RX_REGISTER);
        read |= ESCO_STOP_RX;
        BB_write_baseband_register(ESCO_STOP_RX_REGISTER, read);

        /* If Stop reception bit was already set it means that the FIFO will
         * not be loaded with the received packet.
         */
        if ((1 << rx_status->pkt_type) & BB_RX_ISR_ESCO_DATA_PKT)
        {
            if (lc_esco_stop_reception[esco_lt_addr] == FALSE)
            {
                /* Send Signal to handle the received ESCO Packet,
                 * ext_param is set to indicate that if is a duplicate
                 * and if it is received correctly without CRC error
                 */
                signal.type = LC_HANDLE_ESCO_PACKET;
                signal.param = (OS_ADDRESS)((rx_packet_length << 8) | esco_ce_index);
                signal.ext_param = (OS_ADDRESS)((new_packet << 8) | 0x0001);
                OS_ISR_SEND_SIGNAL_TO_TASK(lc_rx_task_handle, signal);
            }
        }
        lc_esco_stop_reception[esco_lt_addr] = TRUE;
    }

    /* If Ack is received for the transmitted packet and if the
     * packet from the remote device was received successfully
     * then send NULL with ACK , BB spec 8.6.3 page 158
     */
    if ((esco_scheq->ack_received == TRUE) && (lc_esco_stop_reception[esco_lt_addr] == TRUE))
    {
        upper_lut_address = lut_ex_table[esco_lt_addr].upper_lut_address;
        upper_lut = BB_read_baseband_register(upper_lut_address);
        /* Set the ACK bit in the Upper LUT */
        upper_lut |= 0x0004;
        BB_write_baseband_register(upper_lut_address,upper_lut);
    }

#ifdef _DAPE_TEST_ALWAYZ_NAK_ESCO_BY_VENDOR_CMD
    if (g_nak_esco)
    {
        upper_lut_address = lut_ex_table[esco_lt_addr].upper_lut_address;
        upper_lut = BB_read_baseband_register(upper_lut_address);
        BB_write_baseband_register(upper_lut_address, upper_lut&(~BIT2));
        //RT_BT_LOG(RED, YL_DBG_HEX_3, 3,esco_lt_addr, upper_lut,
        	//BB_read_baseband_register(upper_lut_address));

    }
#endif
    return;
}

/**
 * Reads the received packet from the
 * ESCO RX FIFO and if the status indicates that it has to be
 * passed to the host then it is send to the HCI task. Else a
 * dummy read is performed to read the packet out of the FIFO.
 *
 * \param pkt_len_ce_index The length of the bytes to be read
 *                         from the FIFO in the upper byte and
 *                         the ce_index of esco CE in the lower.
 * \param status The status (upper byte = 0x01 indicates if the
 *               packet is duplicate of a previously received
 *               packet and lower byte = 0x01 indicates if it
 *               is without CRC error).
 *
 * \return None.
 */
void lc_handle_esco_rx_packet(UINT16 pkt_len_ce_index, UINT16 status)
{
    UINT16 packet_length;
    UCHAR esco_ce_index;
    UINT16 esco_ch;
    UCHAR err_data;
    UINT8 lt_addr;
    LMP_ESCO_CONNECTION_ENTITY *esco_ce_ptr;

    ESCO_LOG_INFO(LOG_LEVEL_LOW,RECEIVED_ESCO_PACKET_LEN_AND_CE_INDEX_STATUS,2,
                  pkt_len_ce_index,status);

    esco_ce_index = (UCHAR)(pkt_len_ce_index & 0x00FF);

#ifdef _ROM_CODE_PATCHED_
#ifdef _CCH_8723_RCP_
    if (rcp_lc_handle_esco_rx_packet != NULL)
    {
        if ( rcp_lc_handle_esco_rx_packet((void*)(&pkt_len_ce_index), status) )
        {
            return;
        }
    }
#endif
#endif

    esco_ce_ptr = &lmp_esco_connection_entity[esco_ce_index];

#ifndef _CCH_SCO_OUTPUT_ZERO_WHEN_NO_RX_
    if(esco_ce_ptr->use_codec == TRUE)
    {

#ifdef _BRUCE_TEST_PLC_PKT_STATUS
        lt_addr = esco_ce_ptr->lt_addr;
        if(status == 0xFFFF)
        {
            if(global_esco_buf_valid[lt_addr] == FALSE)
            {
                plc_pkt_status_var.g_plc_pkt_miss++;
#ifdef _BRUCE_DEBUG_PORT
                UINT16 read;
                read=BB_read_baseband_register(0x25c); // 0x25c[2]=1; g_plc_pkt_miss
                read |= (UINT16)BIT2;
                BB_write_baseband_register(0x25c,read);
#endif
            }
            else if (global_esco_data_correct[lt_addr] == FALSE)
            {//CRC error

#ifdef _BRUCE_TEST_PLC_PKT_STATUS
                plc_pkt_status_var.g_plc_crc_err++;
#ifdef _BRUCE_DEBUG_PORT
                UINT16 read;
                read=BB_read_baseband_register(0x25c); // 0x25c[0]=1; g_plc_crc_err
                read |=(UINT16)BIT0;
                BB_write_baseband_register(0x25c,read);
#endif
#endif
            }
             global_esco_buf_valid[lt_addr] = FALSE;//reset status
             global_esco_data_correct[lt_addr] = FALSE;//reset status
             return;
        }
        if (status & 0xFF00) /* this is a new pkt */
        {
#ifdef _BRUCE_TEST_PLC_PKT_STATUS
            plc_pkt_status_var.g_plc_tot_num++;
            UINT16 read;
            //0x25A[15:0]  is the mse that hw obtained.
            read=BB_read_baseband_register(0x25A);
            plc_pkt_status_var.g_plc_pkt_mse += read;
            //RT_BT_LOG(GREEN,YL_DBG_DEC_1,1,read);
#endif

            global_esco_buf_valid[lt_addr] = TRUE;
            if (status & 0x00FF)
            {
                /* without CRC error */
                global_esco_data_correct[lt_addr] = TRUE;
            }
            else
            {
                /* CRC error */
                global_esco_data_correct[lt_addr] = FALSE;
            }
         }
         else   /* this is a retry pkt */
         {
            if (status & 0x00FF)
            {
                /* without CRC error */
                if (global_esco_data_correct[lt_addr] == FALSE)
                {
                    global_esco_data_correct[lt_addr] = TRUE;
                }
            }
            else
            {
                /* CRC error */
            }
         }
#endif
        return;
    }
#endif

    packet_length = (UINT16)(pkt_len_ce_index >> 8);
    esco_ch = esco_ce_ptr->conn_handle;
    err_data = esco_ce_ptr->erroneous_data_reporting;
    lt_addr = esco_ce_ptr->lt_addr;

    /* If the signal is because of end of the esco window , then send
     * out the currently received packet to the host
     */
    if(status == 0xFFFF)
    {
       /* If device didn't receive any packet in restransmition window,
               * firmware need to check if it need send the lost packet.
               */

#ifdef _CCH_SCO_OUTPUT_ZERO_WHEN_NO_RX_
        if(esco_ce_ptr->use_codec != TRUE)
        {
#endif
            if (global_esco_buf_valid[lt_addr] == FALSE)
            {
                bz_isoch_send_data_to_host(esco_ch,
                                       err_data,
                                       1,
                                       packet_length, ERR_LOST_DATA);
            }
            else if (global_esco_data_correct[lt_addr] == FALSE)
            {
                /* Enqueue data */
                bz_isoch_send_data_to_host(esco_ch,
                                       err_data,
                                       1,
                                       packet_length, ERR_INVALID_DATA);
            }

#ifdef _CCH_SCO_OUTPUT_ZERO_WHEN_NO_RX_
        }
        else
        {
#ifdef _IMPROVE_PCM_MUTE_CTRL_TIMING_
            if ((global_esco_buf_valid[lt_addr] == FALSE) &&
                (g_efuse_lps_setting_3.iot_sco_noise_pcm_send0))
            {
                if (g_efuse_lps_setting_3.iot_sco_noise_no_sco_count != 0)
                {
                    g_sco_no_rx_count++;
                }
            }
            else
            {
                g_sco_no_rx_count = 0;
            }
#else

            if (global_esco_data_correct[lt_addr] == FALSE)
            {

                // reuse lmp_sco_connection_data.sco_no_rx_count and sco_no_rx_force_output_zero_en
                // esco_ce_index = 0~2
                // 	sco_ce_index = 0~2 , too.


                if( (lmp_sco_connection_data[esco_ce_index].sco_no_rx_count < (2<<(g_efuse_lps_setting_3.iot_sco_noise_no_sco_count <<2)) )
                    && (g_efuse_lps_setting_3.iot_sco_noise_no_sco_count != 0))
                {
                    lmp_sco_connection_data[esco_ce_index].sco_no_rx_count  += 1;
                }
                else
                {
                    if(!lmp_sco_connection_data[esco_ce_index].sco_no_rx_force_output_zero_en)
                    {
                        UINT16 read;
                        read = BB_read_baseband_register(BB_PCM_CTRL2_REG);
                        read |= BIT14;
                        BB_write_baseband_register(BB_PCM_CTRL2_REG, read);

                        RT_BT_LOG(BLUE, CCH_DBG_141, 2, esco_ce_index, lmp_sco_connection_data[esco_ce_index].sco_no_rx_count);
                    }
                    lmp_sco_connection_data[esco_ce_index].sco_no_rx_count  = 0;
                    lmp_sco_connection_data[esco_ce_index].sco_no_rx_force_output_zero_en = 1;
                }

            }
#endif
        }
#endif
        global_esco_buf_valid[lt_addr] = FALSE;
        global_esco_data_correct[lt_addr] = FALSE;

        return;
    } /* end of if(status == 0xFFFF) */

    ESCO_LOG_INFO(LOG_LEVEL_LOW,RX_PKT_LEN_ESCO_CE_INDEX_CH,3
                  packet_length,esco_ce_index,esco_ch);

    if (status & 0xFF00) /* this is a new pkt */
    {
        global_esco_buf_valid[lt_addr] = TRUE;

        if (status & 0x00FF)
        {
            /* without CRC error */
            global_esco_data_correct[lt_addr] = TRUE;

#ifdef _CCH_SCO_OUTPUT_ZERO_WHEN_NO_RX_
            if(esco_ce_ptr->use_codec != TRUE)
            {
#endif
                /* Enqueue data */
                bz_isoch_send_data_to_host(esco_ch,
                                       err_data,
                                       1,
                                       packet_length, ERR_CORRECT_DATA);
#ifdef _CCH_SCO_OUTPUT_ZERO_WHEN_NO_RX_
            }
#endif
        }
        else
        {
            /* CRC error */
            global_esco_data_correct[lt_addr] = FALSE;
        }
    }
    else
    {
        /* this is a retransmit pkt */
        if (status & 0x00FF)
        {
            /* without CRC error */
            if (global_esco_data_correct[lt_addr] == FALSE)
            {
                global_esco_data_correct[lt_addr] = TRUE;

#ifdef _CCH_SCO_OUTPUT_ZERO_WHEN_NO_RX_
                if(esco_ce_ptr->use_codec != TRUE)
#endif
                {

#ifdef _BRUCE_FLUSH_STALED_PKT_IN_ESCO_LINK_OVER_HCI
                    DMA_read_RXFIFO_and_flush_for_sync_link(packet_length, TRUE, esco_ch);
#endif
                    /* Enqueue data */
                    bz_isoch_send_data_to_host(esco_ch,
                                           err_data,
                                           1,
                                           packet_length, ERR_CORRECT_DATA);
                }
            }
            else
            {
#ifdef _CCH_SCO_OUTPUT_ZERO_WHEN_NO_RX_
                if(esco_ce_ptr->use_codec != TRUE)
#endif
                {
#ifdef _BRUCE_FLUSH_STALED_PKT_IN_ESCO_LINK_OVER_HCI
                    /* flush duplicated pkt in the sco rx fifo */
                    DMA_read_RXFIFO_and_flush_for_sync_link(packet_length, TRUE, esco_ch);
#else
                    /* flush duplicated pkt in the sco rx fifo */
                    BB_flush_baseband_SYNC_RX_FIFO(1, packet_length);
#endif
                }
            }
        }
#ifdef _BRUCE_FLUSH_STALED_PKT_IN_ESCO_LINK_OVER_HCI
        else
        {
            DMA_read_RXFIFO_and_flush_for_sync_link(packet_length, TRUE, esco_ch);
        }
#endif
    }

#ifdef _CCH_SCO_OUTPUT_ZERO_WHEN_NO_RX_
#ifndef _IMPROVE_PCM_MUTE_CTRL_TIMING_
    if(esco_ce_ptr->use_codec == TRUE)
    {

        if( global_esco_data_correct[lt_addr] == TRUE )
        {


            if(lmp_sco_connection_data[esco_ce_index].sco_no_rx_force_output_zero_en)
            {
                UINT16 read;
                read = BB_read_baseband_register(BB_PCM_CTRL2_REG);
                read &= (~BIT14);
                BB_write_baseband_register(BB_PCM_CTRL2_REG, read);

                RT_BT_LOG(BLUE, CCH_DBG_140, 1, esco_ce_index);
            }


            lmp_sco_connection_data[esco_ce_index].sco_no_rx_count  = 0;
            lmp_sco_connection_data[esco_ce_index].sco_no_rx_force_output_zero_en = 0;
        }
    }
#endif
#endif

    return;
}

/**
 * Return back to the retransmissions in the curreltly active esco
 * after it was interrupted by a send packet.
 *
 * \param None.
 *
 * \return None.
 */
void lc_return_to_current_esco(void)
{
    UCHAR esco_ce_index;
    UINT16 conn_am_addr;

    if (lmp_self_device_data.number_of_esco_connections == 0)
    {
        return;
    }

    /* Check if the esco entity in the esco_schedulers read_index is valid */
    esco_ce_index = esco_scheduler.esco_scheduler_queue.esco_ce_index;

#ifdef  _FIX_CROSSBAR_ERROR_
    if (esco_ce_index >= LMP_MAX_ESCO_CONN_ENTITIES)
    {
        return;
    }
#endif

    /* If the ESCO CE state is not in connection then just return */
    if(lmp_esco_connection_entity[esco_ce_index].status == NO_CONNECTION)
    {
        return;
    }

    if(lmp_esco_connection_entity[esco_ce_index].esco_window_status != ACTIVE)
    {
        return;
    }

    /* Issue send packet to return back to the current active esco
     * lt address
     */
    conn_am_addr = (UINT16) (lmp_esco_connection_entity[esco_ce_index].lt_addr << 5);

    BB_write_baseband_register(CONNECTOR_REGISTER,conn_am_addr);
#ifndef _DONT_USE_BZ_SEND_PACKET_INSTRUCTION
    BB_write_baseband_register(INSTRUCTION_REGISTER,BB_SEND_PACKET);
#endif

    return;
}

/**
 * Resets the eSCO scheduler parameters.
 * This function is called when the last esco link in the device
 * is disconnected to reset the esco scheduler. Esco interrupts
 * are the triggers for esco scheduler. If there are packets
 * already scheduled and the link is disconnected,esco scheduler
 * has to be explicitly reset. Esco packets should be flushed.
 *
 * \param acl_ce_index ACL connection entity index.
 * \param esco_ce_index eSCO connection entity index.
 *
 * \return None.
 */
void lc_reset_esco_scheduler(UCHAR acl_ce_index, UCHAR esco_ce_index)
{
    if(lmp_self_device_data.number_of_esco_connections != 0)
    {
        /* This is not the last esco link. Do not reset the scheduler */
        return;
    }

    /* Flush whole sco tx fifo */
    lc_var_esco_fifo_flush_length_and_tx_pkt_len_reg &= 0xFC00;
    BB_flush_baseband_SYNC_TX_FIFO(SYNC_FIFO1,
                            lc_var_esco_fifo_flush_length_and_tx_pkt_len_reg);

    lc_init_esco_scheduler();
}
#endif /* COMPILE_ESCO */

#ifdef ENABLE_SCO
/**
 * Programs the BB to establish BB level SCO link with the chosen parameters.
 *
 * \param am_addr Active Member Address of the ACL link on which SCO is being
 *                established.
 * \param sco_num BB ID for the SCO link.
 * \param piconet_id Physical piconet ID.
 *
 * \return API_SUCCESS, if the operation is successful. API_FAILURE,
 *         otherwise.
 */
API_RESULT lc_handle_connect_sco(UCHAR am_addr, UINT16 sco_ce_index,
                                 UCHAR phy_piconet_id)
{


#ifdef _CCH_RETENTION_FLOW_FOR_DLPS_
#ifdef _PLC_FUNC_ENABLE_
    plc_init();
#endif
#endif

    UCHAR Dsco;
    UCHAR Tsco;
    UCHAR lmp_pkt_type;
    UINT16 connector_register;
    UCHAR bb_pkt_type;
    UINT16 sco_codec_bit;
    UINT16 reg_val;
    UINT16 fifo_config;
    UINT16 nslots;
    LMP_SCO_CONNECTION_DATA *sco_ce_ptr;
    UCHAR temp_am_addr;
    UINT32 work_clock;

/*  Added by Wallice Su for PCM external CODEC. 2011/01/03  */
    UINT16 CoedcCodeTable = 0x0;
/*  End Added by Wallice Su for PCM external CODEC. 2011/01/03  */

    DEF_CRITICAL_SECTION_STORAGE;

    LC_EXIT_SM_MODE();

    /* Disable EIR Receive when sco is connected */
    hci_check_and_enable_eir_recv();

    sco_ce_ptr = &lmp_sco_connection_data[sco_ce_index];

    Dsco = sco_ce_ptr->Dsco;
    Tsco = sco_ce_ptr->Tsco;
    lmp_pkt_type = sco_ce_ptr->pkt_type;

    if((Tsco == 0) || (am_addr == 0))
    {
        return API_FAILURE;
    }

    /* Configure packet_type */
    if(lmp_pkt_type == 0x00)
    {
        bb_pkt_type = BB_HV1;
    }
    else if(lmp_pkt_type == 0x01)
    {
        bb_pkt_type = BB_HV2;
    }
    else if(lmp_pkt_type == 0x02)
    {
        bb_pkt_type = BB_HV3;
    }
    else
    {
        return API_FAILURE;
    }

#ifdef SCO_OVER_HCI
    /* Hack to avoid dangling pointer references in the SCO scheduler design.
     * It is mainly used in RX and TX data paths.
     */
    sco_ce_ptr->sch_valid_flag = TRUE;
    sco_ce_ptr->bb_pkt_type = bb_pkt_type;
    sco_ce_ptr->pkt_length = (UCHAR)((lmp_pkt_type+1) * 10);
#endif /* SCO_OVER_HCI */

#ifdef _CCH_SCO_OUTPUT_ZERO_WHEN_NO_RX_
    sco_ce_ptr->sco_no_rx_count = 0;
    sco_ce_ptr->sco_no_rx_force_output_zero_en = 0;
#endif

    sco_ce_ptr->sco_number = (UCHAR)sco_ce_index;

    /* Program Codec Register */
    LMP_CONNECTION_ENTITY *ce_ptr = NULL;

    if (sco_ce_index != ISOCH_SCO_MAX_CONNS)
    {
        ce_ptr = &lmp_connection_entity[sco_ce_ptr->conn_entity_index];

        if (ce_ptr->is_sco_channel)
        {
            /* configure new SCO conversion table */
            if (ce_ptr->is_pcm_input)
            {
                reg_val = BB_read_baseband_register(BB_SCO_CONV_TABLE_REG);
                reg_val &= ~(0x03 << (sco_ce_index << 1));
                reg_val |= ce_ptr->pcm_conv_type << (sco_ce_index << 1);
                BB_write_baseband_register(BB_SCO_CONV_TABLE_REG, reg_val);
            }

            reg_val = BB_read_baseband_register(BB_CODEC_CODE_TABLE_REG);
            reg_val &= ~(0x0F << (sco_ce_index << 2));
            reg_val |= (ce_ptr->trx_codec_conv_type |
                        (ce_ptr->txfifo_in_8bit_code << 3)) << (sco_ce_index << 2);
            BB_write_baseband_register(BB_CODEC_CODE_TABLE_REG, reg_val);
        }
    }
    /*
    LC_LOG_INFO(LOG_LEVEL_LOW, SCO_MSG_CODEC_SETTING, 5,
                        sco_ce_ptr->conn_entity_index,
                        ce_ptr->is_sco_channel, ce_ptr->is_pcm_input,
                        BB_read_baseband_register(BB_SCO_CONV_TABLE_REG),
                        BB_read_baseband_register(BB_CODEC_CODE_TABLE_REG));
*/
    /* Configure Tsco */
    LC_LOG_INFO(LOG_LEVEL_LOW, LC_PROGRAMING_BB_REGS_DSCO_TSCO_SCO_CE_INDEX, 3, Dsco, Tsco, sco_ce_index);

    /* If the SCO_OVER_HCI is not defined, the sco_start_of_sync interrupt
     * will not be handled and SCO_PKT_TYPE will never be set. So we are
     * setting it here.
     */
    reg_val = BB_read_baseband_register(SCO_PACKET_TYPE_REGISTER);
    reg_val = (UINT16)(reg_val & 0xFFF0);
    reg_val = (UINT16)(reg_val | bb_pkt_type);
    BB_write_baseband_register(SCO_PACKET_TYPE_REGISTER, reg_val);

    BB_write_baseband_register_lower_octet(SCO_INTERVAL_REGISTER, Tsco);

    /* FIFO config register configuration */
    fifo_config = 0x00;

#ifdef SCO_OVER_HCI
        if(sco_ce_ptr->codec_state == OVER_NULL)
        {
            fifo_config = SYNC_FIFO1_OVER_HCI;
        }
        else
#endif /* SCO_OVER_HCI */
        if (sco_ce_ptr->codec_type == PCM_CODEC)
        {
            fifo_config = SYNC_FIFO1_USING_PCM;
        }
        else if (sco_ce_ptr->codec_type == UDA_CODEC)
        {
            fifo_config = SYNC_FIFO1_USING_UDA;
        }
        else
        {
            SCO_HCI_INF(LMP_DIDN_T_SET_THE_SCO_CONNECTION_INFO_PROPERL_1,0,0);
        }

    /* Configure Connector Register:
     * Set SCO link number, am_addr, codec_link(is_codec?), make bits in the
     * connector_register.
     */
    connector_register = 0x01;
    /* nth SCO link will have nth bit set*/
    connector_register = (UINT16)(connector_register << (sco_ce_ptr->sco_number)/*sco_ce_index*/);
    temp_am_addr = (UCHAR) (am_addr << 5);

    sco_codec_bit = (UINT16)(1 << 10);

    /* Program the baseband to establish SCO connection */
    connector_register = (UINT16)(connector_register |
                                  (temp_am_addr | 0x0010 /* "make" bit */ | sco_codec_bit));

    connector_register = (UINT16) (connector_register | (phy_piconet_id << 11) );

    /* Spin until the clk1 transition happens. This ensures that we have about
    * 625 micro seconds to program BB to make the SCO link.
    */
    MINT_OS_ENTER_CRITICAL();

    lc_spin_until_clk1_transition(phy_piconet_id);

    /* Program the number of slots to the first SCO instant */
    work_clock = lc_get_nslots_to_first_instant(Tsco, Dsco,
                                                sco_ce_ptr->time_control_flags,
                                                phy_piconet_id, &nslots);

    /* Pick largest nslots such that fits in 4 bits. */
    while(nslots < 9)
    {
        nslots += Tsco;
    }

    BB_write_baseband_register_lower_octet(SCO_SLOT_OFFSET_INTERVAL_REGISTER,
                                           nslots);
    work_clock += nslots;
    work_clock = (work_clock << 1) & 0x0FFFFFFF;

    /* Configure the FIFO Control Register */
    reg_val = BB_read_baseband_register(SYNC_FIFO_CONFIG_REGISTER);

    if ((reg_val & SYNC_FIFO1_CONFIG_MASK) != fifo_config)
    {
        if (sco_ce_ptr->fifo_num == SYNC_FIFO1)
        {
            reg_val = (UINT16)(reg_val & (~SYNC_FIFO1_CONFIG_MASK));
        }
        else
        {
            reg_val = (UINT16)(reg_val & (~SYNC_FIFO2_CONFIG_MASK));
        }

        reg_val = (UINT16)(reg_val | fifo_config);
        BB_write_baseband_register(SYNC_FIFO_CONFIG_REGISTER, reg_val);
    }

/*  Add some code for CODEC convert with BZDMA, Wallice Su. 2010/12/18  */
    if (sync_link_codec_state == OVER_CODEC)
    {
#ifdef _ROM_CODE_PATCHED_
#ifdef _CCH_NEW_SPEC_RCP_
        if (rcp_lc_make_sco_connection_OverCodec != NULL)
        {
            if ( rcp_lc_make_sco_connection_OverCodec((void *)&sco_ce_index, am_addr, phy_piconet_id, pcm_ex_codec_format, pcm_ex_codec_format_8bit, pcmifctrl1, pcmifctrl2, scoconv, pcmconvert, hci_excodec_state, pcmifctrl3) )
            {
                MINT_OS_EXIT_CRITICAL();
                return API_SUCCESS;
            }
        }
#endif
#endif

        connector_register = (UINT16) (connector_register | 0x400); //CODEC link

#ifdef _HW_AUTO_PER_PKT_MUTE_
        UINT16 reg_val;
        reg_val = BB_read_baseband_register(VOICE_SETTING_REGISTER);
        reg_val &= ~BIT3;            //do not check CRC error
        reg_val |= (BIT0 | BIT2);    //External CODEC link and per-pkt mute
        BB_write_baseband_register(VOICE_SETTING_REGISTER, reg_val);
#else
#ifdef _HW_AUTO_MUTE_
        UINT16 reg_val;
        reg_val = BB_read_baseband_register(VOICE_SETTING_REGISTER);
        reg_val &= (BIT4|BIT1);
        reg_val |= (BIT0);    //External CODEC link
        BB_write_baseband_register(VOICE_SETTING_REGISTER, reg_val);
#else
        BB_write_baseband_register(VOICE_SETTING_REGISTER, 0x01); //External CODEC link
#endif
#endif

        reg_val = BB_read_baseband_register(SYNC_FIFO_CONFIG_REGISTER);
        BB_write_baseband_register(SYNC_FIFO_CONFIG_REGISTER,
                                    (reg_val & 0xC000) | 0x05);     //External CODEC link


        switch (sco_ce_ptr->air_mode)
        {
            case 0x0://u-law
                switch (pcm_ex_codec_format)
                {
                    case PCM_CODEC_TYPE_LINEAR://Linear
                        if (pcm_ex_codec_format_8bit)//Linear 8 bits
                        {
                            CoedcCodeTable = 0x09;
                        }
                        else
                        {
                            CoedcCodeTable = 0x01;
                        }

                        break;

                    case PCM_CODEC_TYPE_ALAW://A-law
                        CoedcCodeTable = 0x0B;
                        break;

                    default:
                    case PCM_CODEC_TYPE_ULAW://u-law
                        CoedcCodeTable = 0x00;
                        break;
                }

                break;

            case 0x1://A-law
                switch (pcm_ex_codec_format)
                {
                    case PCM_CODEC_TYPE_LINEAR://Linear
                        if (pcm_ex_codec_format_8bit)//Linear 8 bits
                        {
                            CoedcCodeTable = 0x0A;
                        }
                        else
                        {
                            CoedcCodeTable = 0x02;
                        }
                        break;

                    case PCM_CODEC_TYPE_ULAW://u-law
                        CoedcCodeTable = 0x0C;
                        break;

                    default:
                    case PCM_CODEC_TYPE_ALAW://A-law
                        CoedcCodeTable = 0x00;
                        break;
                }
                break;

            case 0x2://CVSD
                switch (pcm_ex_codec_format)
                {
                    case PCM_CODEC_TYPE_LINEAR://Linear
                        if (pcm_ex_codec_format_8bit)//Linear 8 bits
                        {
                            CoedcCodeTable = 0x0D;
                        }
                        else
                        {
                            CoedcCodeTable = 0x05;
                        }
                        break;

                    case PCM_CODEC_TYPE_ULAW://u-law
                        CoedcCodeTable = 0x0F;
                        break;

                    case PCM_CODEC_TYPE_ALAW://A-law
                        CoedcCodeTable = 0x0E;
                        break;

                    default://Transparent
                        CoedcCodeTable = 0x00;
                        break;

                }
                break;

            case 0x3://Transparent
            default:
                CoedcCodeTable = 0x00;
                break;
        }

        reg_val = BB_read_baseband_register(BB_CODEC_CODE_TABLE_REG);
        reg_val &= ~(0x0F << (sco_ce_index << 2));
        reg_val |= CoedcCodeTable << (sco_ce_index << 2);
        BB_write_baseband_register(BB_CODEC_CODE_TABLE_REG, reg_val);

        BB_write_baseband_register(BB_PCM_CTRL1_REG, pcmifctrl1);
        BB_write_baseband_register(BB_PCM_CTRL2_REG, pcmifctrl2);
        BB_write_baseband_register(BB_PCM_CTRL3_REG, pcmifctrl3);

        reg_val = BB_read_baseband_register(BB_SCO_CONV_TABLE_REG);
        reg_val &= ~(0x03 << (sco_ce_index << 1));
        reg_val |= scoconv << (sco_ce_index << 1);
        BB_write_baseband_register(BB_SCO_CONV_TABLE_REG, reg_val);

#ifdef _BRUCE_TEST_PLC_PKT_STATUS
        UINT16 read = BB_read_baseband_register(PLC_CTRL2_REGISTER);
        read &= 0xFE1F; //Clear BIT8~BIT5
        read |=  ((pcmconvert & 0x0F) << 0x05);
        BB_write_baseband_register(PLC_CTRL2_REGISTER, read);

        read = BB_read_baseband_register(BB_PCMOUT_SHIFT_REG);
        read &= 0xFF87; //Clear BIT6~BIT3
        read |=  ((pcmconvert & 0xF0) >> 0x01);
        BB_write_baseband_register(BB_PCMOUT_SHIFT_REG, read);
#endif

#ifdef _BRUCE_DROP_MUTED_ISO_OUT_PKTS_OVER_CODEC
        if(IS_MUTED_OVER_CODEC_ENABLE)
        {
        reg_val = BB_read_baseband_register(BB_MUTED_ISO_OUT_PKTS_CTRL1_REG);
        /*bit[0:14],
                  the compared amplitude for pcm_in data,
                  pcm_in_mute_lower_bound = -1 * pcm_in_comp_amp;
                  pcm_in_mute_upper_bound = +1 * pcm_in_comp_amp; */
        reg_val |= 0x1F4;
        BB_write_baseband_register(BB_MUTED_ISO_OUT_PKTS_CTRL1_REG,reg_val);
        reg_val = BB_read_baseband_register(BB_MUTED_ISO_OUT_PKTS_CTRL2_REG);
        /* the threshold of muted sample counts for pcm_in data */
        reg_val |= 0x2710;
        BB_write_baseband_register(BB_MUTED_ISO_OUT_PKTS_CTRL2_REG,reg_val);
        }
#endif

#ifdef _PLC_FUNC_ENABLE_
        if(IS_PLC_ENABLE)
        {
        //(bruce) setting 0xE0[11]=1,codec_plc_en.
        reg_val = BB_read_baseband_register(VOICE_SETTING_REGISTER);
        reg_val |= (UINT16)(BIT11|BIT2);
        //RT_BT_LOG(YELLOW,YL_DBG_DEC_1,1,reg_val);
        BB_write_baseband_register(VOICE_SETTING_REGISTER,reg_val);
        }
#endif

#ifdef ENABLE_PCM_ONLY_WHEN_SCO_LINK_CREATED
        lc_pcm_enable_control(1);// v5, v6
#endif
#if 0
#ifdef _SUPPORT_BT_CONTROL_PCM_MUX_
        if ((pcmifctrl3 & 0x6000))
        {
            switch_pcm_mux_after_con_established_pcmoutctrl(pcmifctrl3); //PCM_OUT drivedata
        }
        else
        {
            switch_pcm_mux_after_con_established();// combo v6 only
        }
#endif
#endif
#ifdef _SUPPORT_BT_CONTROL_PCM_MUX_
        switch_pcm_mux_after_con_established_pcmoutctrl(pcmifctrl3); //PCM_OUT drivedata, // combo v6 only
#endif
    }
/*  End Add some code for CODEC convert with BZDMA, Wallice Su. 2010/12/18  */

#ifdef _ROM_CODE_PATCHED_
#ifdef _BRUCE_8821B_PLC_RCP_
		if (rcp_lc_handle_connect_sco_func != NULL)
		{
		    rcp_lc_handle_connect_sco_func();
		}
#endif
#endif

    /* Make BB level SCO link */
    BB_write_baseband_register(CONNECTOR_REGISTER, connector_register);
    BB_write_baseband_register(INSTRUCTION_REGISTER, BB_EXECUTE);
    MINT_OS_EXIT_CRITICAL();

    /* Temporarily set the full bandwidth flag, so that the connection
     * establishment transition will be smooth. The slave has to override SCO
     * instants until receiving ACK for the LMP_accepted(LMP_sco_link_req)
     * PDU.
     */
    BB_set_slave_full_bandwidth_flag();

//    LC_LOG_INFO(LOG_LEVEL_HIGH, SCO_CONN_PROGRAM_INFO, 3,
//       BB_read_baseband_register(SYNC_FIFO_CONFIG_REGISTER), connector_register,
//       nslots);

#ifdef SCO_OVER_HCI
    if (sco_ce_ptr->codec_state == OVER_NULL &&
            sco_ce_ptr->fifo_num != SYNC_FIFO3)
    {
        SCO_ISOCH_MAP *sco_map;

        /* Only map iso-q if non dummy fifo */
        sco_map = bz_isoch_add_sco_queues(sco_ce_ptr->sco_conn_handle,
                                          sco_ce_ptr->conn_entity_index);

        sco_map->next_instant = work_clock;

        /* Flush the whole FIFO for removing residual data */
        BB_flush_baseband_SYNC_TX_FIFO(sco_ce_ptr->fifo_num, 0);
        BB_flush_baseband_SYNC_RX_FIFO(sco_ce_ptr->fifo_num, 0);

        LC_LOG_INFO(LOG_LEVEL_HIGH, MSG_SCO_OVER_HCI_1ST_INSTANT, 3,
                                            sco_ce_ptr->sco_conn_handle,
                                            sco_ce_ptr->conn_entity_index,
                                            sco_map->next_instant);
    }
#endif /* SCO_OVER_HCI */

    lmp_self_device_data.sco_pkt_type = SCO_PKT_TYPE_LMP_TO_1_1_HCI(lmp_pkt_type);
/*
    pf_enable_usb_interface_sco_filter(sco_ce_ptr->sco_number, 1,
                                        sco_ce_ptr->sco_conn_handle, 1);
*/

    bz_sync_ch_conn_handle = sco_ce_ptr->sco_conn_handle;

    if(sco_ce_ptr->codec_state == OVER_CODEC)
    {
        num_sync_links_over_codec++;
    }

    if (sync_link_codec_state == OVER_CODEC)
    {
        RT_BT_LOG(GREEN, LC_SYNC_LINKS_CODEC, 8,
                        sco_ce_index,
                        sco_ce_ptr->conn_entity_index,
                        sco_ce_ptr->Dsco,
                        sco_ce_ptr->Tsco,
                        sco_ce_ptr->pkt_type,
                        sco_ce_ptr->air_mode,
                        pcm_ex_codec_format,
                        pcm_ex_codec_format_8bit);

        RT_BT_LOG(GREEN, LC_PCM_CODEC_REG, 9,
                    BB_read_baseband_register(BB_CODEC_CODE_TABLE_REG),
                    BB_read_baseband_register(BB_SCO_CONV_TABLE_REG),
                    BB_read_baseband_register(BB_PCM_CTRL1_REG),
                        BB_read_baseband_register(BB_PCM_CTRL2_REG),
                        BB_read_baseband_register(BB_PCM_CTRL3_REG),
                        BB_read_baseband_register(VOICE_SETTING_REGISTER),
                        BB_read_baseband_register(PLC_CTRL2_REGISTER),
                        BB_read_baseband_register(BB_PCMOUT_SHIFT_REG),
                        BB_read_baseband_register(0x260));
    }

#ifdef _ROM_CODE_PATCHED_
#ifdef _CCH_8723_RCP_
    if (rcp_lc_handle_connect_sco_end != NULL)
    {
        rcp_lc_handle_connect_sco_end((void *)&am_addr, sco_ce_index, phy_piconet_id);
    }
#endif
#endif

    return API_SUCCESS;
}

/**
 * Detach the sco connection denoted by \a sco_number at the baseband level.
 *
 * \param sco_number The index of the SCO link to be detached.
 * \param am_addr The Active Member(AM_ADDR) or LT_ADDR of the sco link.
 * \param fifo_num The FIFO used for the SCO link to be detached.
 *
 * \return None.
 */
void lc_detach_bb_level_sco_connection(UCHAR sco_number,
        UINT16 ce_index,
        UCHAR fifo_num,
        UCHAR is_over_codec)
{
    UINT16 connector_register;

    DEF_CRITICAL_SECTION_STORAGE;

    LMP_CONNECTION_ENTITY *ce_ptr = NULL;

    UCHAR am_addr;

    ce_ptr = &lmp_connection_entity[ce_index];
    am_addr = ce_ptr->am_addr;

    /* Make sure that make bit is zero. */
    connector_register = 0x01;
    connector_register = (UINT16)(connector_register << sco_number);
    connector_register = (UINT16)(connector_register | (am_addr << 5));
    connector_register = (UINT16)(connector_register |
                                  (ce_ptr->phy_piconet_id << 11));

    MINT_OS_ENTER_CRITICAL();

    UINT16 read_16;
    BZ_REG_S_PRI_CTRL *pri_ctrl_reg;
    pri_ctrl_reg = (BZ_REG_S_PRI_CTRL *)&read_16;

    read_16 = BB_read_baseband_register(SCA_PRIORITY_REGISTER);
    pri_ctrl_reg->pause_sco = FALSE;
    BB_write_baseband_register(SCA_PRIORITY_REGISTER, read_16);

    lc_sco_pause_status = 0x00;

    BB_write_baseband_register(CONNECTOR_REGISTER, connector_register);
    BB_write_baseband_register(INSTRUCTION_REGISTER, BB_EXECUTE);
    MINT_OS_EXIT_CRITICAL();

    /* Clear the voice settings register. Note - this is only temporary,
    and should be cleaned up when codec drivers are  implemented. Currently we
    clear the voice settings when any codec SCO conn is being disconnected,
    regardless of if any other codec conn is still in existence */
    if (is_over_codec)
    {
        //RT_BT_LOG(GRAY, LC_SYNC_LINKS_3281, 0, 0);
#ifdef _HW_AUTO_MUTE_
        UINT16 reg_val;
        reg_val = BB_read_baseband_register(VOICE_SETTING_REGISTER);
        reg_val &= (BIT4|BIT1);
        BB_write_baseband_register(VOICE_SETTING_REGISTER, reg_val);
#else
        BB_write_baseband_register(VOICE_SETTING_REGISTER, 0x00);
#endif
    }

    RT_BT_LOG(GRAY, LC_SYNC_LINKS_3267, 2, read_16, is_over_codec);

    //hci_check_and_enable_eir_recv();

    return;
}

/**
 * Kill the BB level SCO connection identified by \a sco_ce_index.
 *
 * \param sco_ce_index The index to the SCO Connection Database.
 *
 * \return API_SUCCESS, on successful disconnection. API_FAILURE, otherwise.
 */
API_RESULT lc_kill_sco_connection(UINT16 sco_ce_index)
{
    LMP_SCO_CONNECTION_DATA* sco_ce_ptr;
    UCHAR am_addr;
    LMP_CONNECTION_ENTITY *ce_ptr;

    RT_BT_LOG(GREEN, LC_SYNC_LINKS_3318, 0, 0);

    sco_ce_ptr = &lmp_sco_connection_data[sco_ce_index];

    ce_ptr = &lmp_connection_entity[sco_ce_ptr->conn_entity_index];

    am_addr = ce_ptr->am_addr;

#ifdef _CCH_SLOT_OFFSET_
    lmp_put_global_slot_offset(sco_ce_ptr->conn_entity_index, sco_ce_index + 2*LMP_MAX_CE_DATABASE_ENTRIES);
#endif


#ifdef _CCH_WHQL_SCO_
        if(lmp_self_device_data.total_no_of_sco_conn !=0 )
        {
            lmp_self_device_data.total_no_of_sco_conn--;
        }
#else
        lmp_self_device_data.total_no_of_sco_conn--;
#endif

        if (ce_ptr->no_of_sco_connections != 0)
        {
            ce_ptr->no_of_sco_connections--;
        }


#ifdef SCO_OVER_HCI
    if (sco_ce_ptr->codec_state == OVER_CODEC)
    {
//{{ added by guochunxia
        if(num_sync_links_over_codec>0)
        {
            num_sync_links_over_codec--;
        }

        sco_ce_ptr->sch_valid_flag = FALSE;
//}}
#endif /* SCO_OVER_HCI */

        lc_detach_bb_level_sco_connection((UCHAR) sco_ce_index,
                                          sco_ce_ptr->conn_entity_index, sco_ce_ptr->fifo_num, TRUE);
#ifdef _BRUCE_DROP_MUTED_ISO_OUT_PKTS_OVER_CODEC
        //reset
        UINT16 read;
        read = BB_read_baseband_register(BB_MUTED_ISO_OUT_PKTS_CTRL1_REG);
        /*pcm_in_comp_amp bit[0:14] & pcm_in stream is muted bit[15] reset 0   */
        read &= 0x00;
        BB_write_baseband_register(BB_MUTED_ISO_OUT_PKTS_CTRL1_REG,read);
        read = BB_read_baseband_register(BB_MUTED_ISO_OUT_PKTS_CTRL2_REG);
        /* the threshold of muted sample counts for pcm_in data */
        read &= 0x00;
        BB_write_baseband_register(BB_MUTED_ISO_OUT_PKTS_CTRL2_REG,read);
#endif

#ifdef _PLC_FUNC_ENABLE_
        if(IS_PLC_ENABLE)
        {
        //(bruce) setting 0xE0[11]=0,codec_plc_disable.
        UINT16 reg_val;
        reg_val = BB_read_baseband_register(VOICE_SETTING_REGISTER);
        reg_val &= (UINT16)~(BIT11);
        reg_val &= (UINT16)~(BIT3);
        BB_write_baseband_register(VOICE_SETTING_REGISTER,reg_val);
        }
#endif

#ifdef ENABLE_PCM_ONLY_WHEN_SCO_LINK_CREATED
        lc_pcm_enable_control(0);
#endif
#ifdef  _SUPPORT_BT_CONTROL_PCM_MUX_
        // restore btgpio status
        restore_pcm_mux_state_when_disconnect();
#endif

#ifdef _ROM_CODE_PATCHED_
#ifdef _BRUCE_8821B_PLC_RCP_
        if (rcp_lc_kill_sco_connection_over_codec_func != NULL)
        {
            rcp_lc_kill_sco_connection_over_codec_func();
        }
#endif
#endif

#ifdef SCO_OVER_HCI
    }
    else
    {
        sco_ce_ptr->sch_valid_flag = FALSE;
        bz_isoch_remove_sco_queues(sco_ce_ptr->sco_conn_handle,
                                   sco_ce_ptr->conn_entity_index);

        lc_detach_bb_level_sco_connection((UCHAR) sco_ce_index,
                                          sco_ce_ptr->conn_entity_index, sco_ce_ptr->fifo_num, FALSE);

    } /* end of if (is the SCO link over codec) */
#endif /* SCO_OVER_HCI */
//}}


#ifdef _CCH_SCO_OUTPUT_ZERO_WHEN_NO_RX_
    sco_ce_ptr->sco_no_rx_count = 0;
    sco_ce_ptr->sco_no_rx_force_output_zero_en = 0;
#endif


    return API_SUCCESS;
}

#ifdef SCO_OVER_HCI
/**
 * Handles the SCO instant interrupt. The SCO instant interrupt will be
 * generated by the baseband 312.5us ahead of actual SCO instant. This handler
 * is nothing but the sco scheduler which multiplexes multiple sco connections
 * on a single resource (FIFO).
 *
 * \param sco_ce_index The index of SCO connection database for which this
 *                     instant has occurred.
 *
 * \return None.
 */
SECTION_ISR void lc_handle_sco_instant_interrupt(UCHAR sco_ce_index)
{
    LMP_SCO_CONNECTION_DATA* sco_ce_ptr;
    UINT16 sco_pkt_type;

    sco_ce_ptr = &lmp_sco_connection_data[sco_ce_index];

#ifdef _ROM_CODE_PATCHED_
#ifdef _BRUCE_8821B_PLC_RCP_
    if (rcp_lc_handle_sco_instant_interrupt != NULL)
    {
        if (rcp_lc_handle_sco_instant_interrupt( (void *)&sco_ce_index))
        {
            return;
        }
    }
#endif
#endif

    /*
     * OVER_CODEC links will not get start_instant interrupts after detach. So
     * if sco_ce_ptr->sch_valid_flag is FALSE, then it should have been for
     * SCO_OVER_HCI link for which the disconnection is requested.(Even if
     * sco_ce_ptr structure's content is not valid, this check will not harm -
     * because the "codec_state" would be NOT_APPLICABLE. So this will proceed
     * further as expected).
     */
    if (sco_ce_ptr->codec_state == OVER_CODEC)
    {
#ifdef _BRUCE_TEST_PLC_PKT_STATUS
        UINT16 read_temp;
        UINT16 read=BB_read_baseband_register(0x25c); //SCO 0x25c[7]=0  hw  monitoring pkt miss;
        read_temp=(UINT16)((read >>7) & 01);
        if(read_temp)
        {   //0x25c[7]=0 ;pkt correct
#ifdef _BRUCE_RECORD_ERASECNT_STATUS
            if(plc_pkt_status_var.g_burst_error_flag)
            {
                plc_pkt_status_var.g_burst_error_flag=0;
            }
            if(plc_pkt_status_var.g_record_flag)
            {
                plc_pkt_status_var.g_record_cnt=0;
                plc_pkt_status_var.g_record_flag=0;
                plc_pkt_status_var.g_burst_error_flag=1;
            }
#endif
        }
        else
        {   //0x25c[7]=0 ;pkt miss
#ifdef _BRUCE_RECORD_ERASECNT_STATUS
            if(plc_pkt_status_var.g_burst_error_flag)
            {
                plc_pkt_status_var.g_plc_pkt_miss_burst_error++;
                plc_pkt_status_var.g_burst_error_flag=0;
            }
            if(plc_pkt_status_var.g_record_flag)
            {
                plc_pkt_status_var.g_record_cnt++;
            }
            plc_pkt_status_var.g_record_flag=1;
            switch (plc_pkt_status_var.g_record_cnt)
            {
            case 2:
                plc_pkt_status_var.g_plc_pkt_miss_erasecnt_2++;
                break;
            case 3:
                plc_pkt_status_var.g_plc_pkt_miss_erasecnt_3++;
                break;
            case 5:
                plc_pkt_status_var.g_plc_pkt_miss_erasecnt_5++;
                break;
            default:
                break;
            }
#endif
            plc_pkt_status_var.g_plc_HW_pkt_miss++;
#ifdef _BRUCE_DEBUG_PORT
            read=BB_read_baseband_register(0x25c); //SCO 0x25c[3]=1; g_plc_HW_pkt_miss
            read |= (UINT16)BIT3;
            BB_write_baseband_register(0x25c,read);
#endif
        }
#endif

#ifdef _BRUCE_DROP_MUTED_ISO_OUT_PKTS_OVER_CODEC
            UINT16 read;
            UINT16 is_PCM_in_mute_stream;
             /*BB_MUTED_ISO_OUT_PKTS_CTRL1_REG bit[15],
                       indicates the pcm_in stream is muted
                       0: Inside infor is not muted
                       1: Inside infor is muted*/
            read=BB_read_baseband_register(BB_MUTED_ISO_OUT_PKTS_CTRL1_REG);
            is_PCM_in_mute_stream= (read >> 15)& (0x01);
#endif


        /*
         * If OVER_CODEC and OVER_NULL links coexist, then we need to update
         * the corresponding BB_PKT_TYPE in the SCO_PACKET_TYPE_REGISTER. This
         * is required because for OVER_NULL links, if the data is not
         * available, we would update SCO_PACKET_TYPE_REGISTER with NULL and
         * it should not remain as it is for the OVER_CODEC links.
         */

        sco_pkt_type = BB_read_baseband_register(SCO_PACKET_TYPE_REGISTER);

#ifdef _BRUCE_DROP_MUTED_ISO_OUT_PKTS_OVER_CODEC
            LMP_CONNECTION_ENTITY *ce_ptr;
            ce_ptr = &lmp_connection_entity[sco_ce_ptr->conn_entity_index];
            if(is_PCM_in_mute_stream)
            {  //  send POLL/NULL pkt
                sco_pkt_type &= 0xFFF0;
                sco_pkt_type |= (ce_ptr->remote_dev_role == MASTER) ? BB_NULL : BB_POLL;
                BB_write_baseband_register(SCO_PACKET_TYPE_REGISTER, sco_pkt_type);
            }
            else
            {
                if ((sco_pkt_type & 0x000F) != sco_ce_ptr->bb_pkt_type)
                {
                    sco_pkt_type &= 0xFFF0;
                    sco_pkt_type |= sco_ce_ptr->bb_pkt_type;
                    BB_write_baseband_register(SCO_PACKET_TYPE_REGISTER, sco_pkt_type);
                }
            }
#else
        if ((sco_pkt_type & 0x000F) != sco_ce_ptr->bb_pkt_type)
        {
            sco_pkt_type &= 0xFFF0;
            sco_pkt_type |= sco_ce_ptr->bb_pkt_type;
            BB_write_baseband_register(SCO_PACKET_TYPE_REGISTER, sco_pkt_type);
        }
#endif

#ifdef _BRUCE_DEBUG_PORT
        read=BB_read_baseband_register(0x25c); //SCO 0x25c[3]=0; g_plc_HW_pkt_miss
        read &= (UINT16)~(BIT3);
        BB_write_baseband_register(0x25c,read);
#endif

        return; /* We dont bother to schedule OVER_CODEC links */
    }

#ifndef _USE_ONE_NEW_BZDMA_TXCMD_FOR_SCO_
#if 0
    /* For improving voice quality, if we only have one sco link,
       do not flush SCO TRX fifo */
    if (lmp_self_device_data.total_no_of_sco_conn > 1)
    {
        /* Flush whole sco tx fifo */
        /* Flush the whole FIFO for removing residual data */
        BB_flush_baseband_SYNC_TX_FIFO(SYNC_FIFO1, 0);
        BB_flush_baseband_SYNC_RX_FIFO(SYNC_FIFO1, 0);
    }
#else
    /* dape modified. To flush the sco packets that
       has be lated instant.*/
    /* Flush the whole FIFO for removing residual data */
    BB_flush_baseband_SYNC_TX_FIFO(SYNC_FIFO1, 0); /* mark it until hw has fixed the bug */
#endif
#endif

#if defined(_USE_ONE_NEW_BZDMA_TXCMD_FOR_SCO_) && defined(_USE_ONE_NEW_BZDMA_TXCMD_FOR_SCO_WORKAROUND_)
    UINT8 is_bzdma_sco_tx_entry_reentry = FALSE;
    {
        UINT8 txcmd_id = BZDMA_TX_ENTRY_TYPE_NEW_SCO0;

        /* Tx Command for (e)SCO Tx FIFO */
        if (Bzdma_Manager.TxEntSta[txcmd_id].used)
        {
            if (Bzdma_Manager.TxEntSta[txcmd_id].is_sco_lt)
            {
                /* free sco tx pkt */
                pf_hci_free_tx_pkt(Bzdma_Manager.TxEntSta[txcmd_id].ch_id);

                /* invalidate the bzdma */
                bzdma_invalid_txcmd(txcmd_id, 0, Bzdma_Manager.TxEntSta[txcmd_id].ch_id);
                is_bzdma_sco_tx_entry_reentry = TRUE;
            }
        }
    }
#endif

    /*=====================================================================*/
    /* this is OVER_HCI state and we will send SCO pkt to the air - austin */
    /*=====================================================================*/
    sco_pkt_type = BB_read_baseband_register(SCO_PACKET_TYPE_REGISTER);
    sco_pkt_type &= 0xFFF0;

    /* reset tx queue of fw/hw if alternative setting is changed */
    if (alternatset_change_tx_clear)
    {
        pf_hci_transport_reset_isoch_queue();
    }

    if (bz_isoch_write_sco_tx_data(sco_ce_ptr) == API_SUCCESS)
    {
        /* Instruct BB to send the SCO packet */
        sco_pkt_type |= sco_ce_ptr->bb_pkt_type;
    }
    else
    {
        /* Since there is no data in the fifo - it will underrun and
         * the last byte will be sent continously over air */

        LMP_CONNECTION_ENTITY *ce_ptr;
        ce_ptr = &lmp_connection_entity[sco_ce_ptr->conn_entity_index];

#ifdef _SCO_RSVD_SLOT_DONT_SEND_POLL_NULL_PKT_

#ifdef _CCH_WHQL_SCO_
        if ( (lmp_self_device_data.test_mode == HCI_REMOTE_LOOPBACK_MODE) &&
            ( sco_ce_ptr->bb_pkt_type != BB_HV1 ) )
        {
            /* The host has not provided the data */
            sco_pkt_type |= (ce_ptr->remote_dev_role == MASTER) ? BB_NULL : BB_POLL;
        }
        else
#endif
        {
    	    sco_pkt_type |= sco_ce_ptr->bb_pkt_type;
        }
#else

#ifdef _HV1_RSVD_SLOT_DONT_SEND_POLL_NULL_PKT_
    	if ( sco_ce_ptr->bb_pkt_type == BB_HV1 )
    	{
    		sco_pkt_type |= sco_ce_ptr->bb_pkt_type;
    	}
        else
#endif
    	{
            /* The host has not provided the data */
            sco_pkt_type |= (ce_ptr->remote_dev_role == MASTER) ? BB_NULL : BB_POLL;
    	}
#endif
    }

    /* Set the correct SCO packet type for this instant */
    BB_write_baseband_register(SCO_PACKET_TYPE_REGISTER, sco_pkt_type);

    /* add log to notify bzdma sco tx entry reentry occur*/
#if defined(_USE_ONE_NEW_BZDMA_TXCMD_FOR_SCO_) && defined(_USE_ONE_NEW_BZDMA_TXCMD_FOR_SCO_WORKAROUND_)
    if(is_bzdma_sco_tx_entry_reentry)
    {
        RT_BT_LOG(RED, DAPE_TEST_LOG587, 1, lc_sco_pause_status);
    }
#endif
}

/**
 * Handles the SCO receive interrupt. This interrupt occurs after the
 * successful reception of a SCO packet. This function will not be called for
 * POLL/NULL received in the SCO slot. It signals a task to perform the actual
 * reading operation.
 *
 * \param sco_ce_index The index of the SCO connection database for which this
 *                     interrupt has occurred.
 *
 * \return None.
 */
void lc_handle_sco_rx_interrupt(UCHAR sco_ce_index)
{
    OS_SIGNAL signal;

#ifdef _ROM_CODE_PATCHED_
#ifdef _CCH_8723_RCP_
    if (rcp_lc_handle_sco_rx_interrupt != NULL)
    {
        if ( rcp_lc_handle_sco_rx_interrupt((void*)(&sco_ce_index)) )
        {
            return;
        }
    }
#endif
#endif

    if (lmp_sco_connection_data[sco_ce_index].codec_state == OVER_CODEC)
    {
        /* should not use it when over hci, it is hard to debug !! */
#ifdef _CCH_SCO_OUTPUT_ZERO_WHEN_NO_RX_
#ifdef _CCH_RTL8723A_B_CUT
#ifdef _IMPROVE_PCM_MUTE_CTRL_TIMING_
        g_sco_no_rx_count = 0;
#else
// _CCH_8723_A_ECO_
        if( (g_efuse_lps_setting_3.iot_sco_noise_pcm_send0)
            && (lmp_sco_connection_data[sco_ce_index].sco_no_rx_force_output_zero_en) )
        {

            UINT16 read;
            read = BB_read_baseband_register(BB_PCM_CTRL2_REG);
            read &= (~BIT14);
            BB_write_baseband_register(BB_PCM_CTRL2_REG, read);

            RT_BT_LOG(BLUE, CCH_DBG_140, 1, sco_ce_index);
        }

        lmp_sco_connection_data[sco_ce_index].sco_no_rx_count  = 0;
        lmp_sco_connection_data[sco_ce_index].sco_no_rx_force_output_zero_en = 0;
#endif
#endif
#endif
        return; /* We dont bother about OVER_CODEC links */
    }

    /* Signal the LC_RX_TASK to read the data from the FIFO */
    signal.type = LC_HANDLE_RX_SCO_DATA_SIGNAL;
    signal.param = (OS_ADDRESS)((UINT32)sco_ce_index);
    OS_ISR_SEND_SIGNAL_TO_TASK(lc_rx_task_handle, signal);
}


/**
 * Handles erroneous data for a sco connection. This function is called in
 * case of missing or erroneous received data for a sco channel. Hence it
 * should be called for HEC error, Access Error, and on receiving NULL/POLL
 *
 * \param sco_ce_index The index of the SCO connection database for which this
 *                     interrupt has occurred.
 * \param data_status  Status of erroneous data.
 *
 * \return None.
 */
void lc_handle_sco_erroneous_data(UCHAR sco_ce_index, UCHAR data_status)
{
    OS_SIGNAL signal;

#ifdef _ROM_CODE_PATCHED_
#ifdef _CCH_8723_RCP_
    if (rcp_lc_handle_sco_erroneous_data != NULL)
    {
        if ( rcp_lc_handle_sco_erroneous_data((void*)(&sco_ce_index), data_status) )
        {
            return;
        }
    }
#endif
#endif

    if (lmp_sco_connection_data[sco_ce_index].codec_state == OVER_CODEC ||
            lmp_sco_connection_data[sco_ce_index].sch_valid_flag == FALSE
       )
    {
    /* Toggle LED ? */
#ifdef _CCH_SCO_OUTPUT_ZERO_WHEN_NO_RX_
#ifdef _CCH_RTL8723A_B_CUT
// _CCH_8723_A_ECO_
    if( g_efuse_lps_setting_3.iot_sco_noise_pcm_send0)
    {
        if(data_status == ERR_LOST_DATA)
        {
#ifdef _IMPROVE_PCM_MUTE_CTRL_TIMING_
            if (g_efuse_lps_setting_3.iot_sco_noise_no_sco_count != 0)
            {
                g_sco_no_rx_count++;
            }
#else
            if ((lmp_sco_connection_data[sco_ce_index].sco_no_rx_count < (2<<(g_efuse_lps_setting_3.iot_sco_noise_no_sco_count <<2)) )
                && (g_efuse_lps_setting_3.iot_sco_noise_no_sco_count != 0))
            {
                lmp_sco_connection_data[sco_ce_index].sco_no_rx_count  += 1;
            }
            else
            {
                if (!lmp_sco_connection_data[sco_ce_index].sco_no_rx_force_output_zero_en)
                {
                    UINT16 read;
                    read = BB_read_baseband_register(BB_PCM_CTRL2_REG);
                    read |= BIT14;
                    BB_write_baseband_register(BB_PCM_CTRL2_REG, read);

                    RT_BT_LOG(BLUE, CCH_DBG_141, 2, sco_ce_index, lmp_sco_connection_data[sco_ce_index].sco_no_rx_count);
                }
                lmp_sco_connection_data[sco_ce_index].sco_no_rx_count  = 0;
                lmp_sco_connection_data[sco_ce_index].sco_no_rx_force_output_zero_en = 1;
            }
#endif
        }
    }
#endif
#endif

        return; /* We dont bother about OVER_CODEC links */
    }

    /* Signal the LC_RX_TASK to read the data from the FIFO, and mark status. */
    signal.type = LC_HANDLE_RX_SCO_ERR_DATA_SIGNAL;
    signal.param = (OS_ADDRESS)((UINT32)sco_ce_index);
    signal.ext_param = (OS_ADDRESS)((UINT32)data_status);
    OS_ISR_SEND_SIGNAL_TO_TASK(lc_rx_task_handle, signal);
}

/**
 * Read the received SCO packet from the FIFO and make a request to send the
 * obtained data to host.
 *
 * \param sco_ce_index The index of the SCO link for which the data from the
 *                     FIFO has to be read.
 *
 * \return None.
 */
void lc_handle_sco_rx_packet(int sco_ce_index)
{
    LMP_SCO_CONNECTION_DATA* sco_ce_ptr;

    sco_ce_ptr = &lmp_sco_connection_data[sco_ce_index];

    /* Is it marked for disconnection? */
    if (sco_ce_ptr->sch_valid_flag == FALSE)
    {
        /* Simply flush the RX FIFO with the correct length */
        if(lmp_self_device_data.total_no_of_sco_conn)
        {
            UCHAR length;

            BZ_ASSERT(lmp_self_device_data.sco_pkt_type != HV1,
                      "number_of_sco_conn should be zero for HV1");

            length = (UCHAR)
                     ((lmp_self_device_data.sco_pkt_type == HV2) ? 20: 30);

#ifdef _BRUCE_FLUSH_STALED_PKT_IN_ESCO_LINK_OVER_HCI
            DMA_read_RXFIFO_and_flush_for_sync_link(length, TRUE, sco_ce_ptr->sco_conn_handle);
#else
            BB_flush_baseband_SYNC_RX_FIFO(sco_ce_ptr->fifo_num,
                                           length);
#endif
        }
    }
    else
    {
        /* Enqueue data */
        bz_isoch_send_data_to_host(sco_ce_ptr->sco_conn_handle,
                                   sco_ce_ptr->erroneous_data_reporting,
                                   sco_ce_ptr->fifo_num,
                                   sco_ce_ptr->pkt_length, ERR_CORRECT_DATA);
    }
}
#endif /* SCO_OVER_HCI */
#endif /* ENABLE_SCO */

#if defined(_NEW_BZDMA_FROM_V7_) && (defined(BZDMA_USE_NEW_SCO_TXCMD) || \
                                    defined(_USE_ONE_NEW_BZDMA_TXCMD_FOR_SCO_))
void lc_handle_sco_txcmd_ito_interrupt(UINT16 status)
{
    /* we have any pending bzdma tx command that hw does not schedule it,
       so we need to invalidate them and free the data packets */
    UINT8 i;
    UINT8 txcmd_id;
    UINT8 imax;

#if defined(_USE_ONE_NEW_BZDMA_TXCMD_FOR_SCO_)
    imax = 1;
#else
    imax = 3;
#endif

#ifdef _BRUCE_CHECK_ITO
    UINT8 sco_num = 0;
    if (status & 0x0400)
    {
        /* SCO1 Interrupt */
        sco_num = 0;
    }
    else if (status & 0x0800)
    {
        /* SCO2 Interrupt */
        sco_num = 1;
    }
    else if (status & 0x1000)
    {
        /* SCO3  interrupt */
        sco_num = 2;
    }
#endif

    for (i = 0; i < imax; i++)
    {
#if !defined(_USE_ONE_NEW_BZDMA_TXCMD_FOR_SCO_)
        /* check bit 10, 11 and 12 */
        if ((1 << (10 + i)) & status)
#endif
        {
            txcmd_id = BZDMA_TX_ENTRY_TYPE_NEW_SCO0 + i;

#ifdef _BRUCE_CHECK_ITO
            if(Bzdma_Manager.TxEntSta[txcmd_id].ch_id != sco_num)
            {
                RT_BT_LOG(RED,BRUCE_TEST_LOG_7008,3,sco_num,txcmd_id,
                         Bzdma_Manager.TxEntSta[txcmd_id].ch_id);
                return;
            }
#endif

            /* Tx Command for (e)SCO Tx FIFO */
            if (Bzdma_Manager.TxEntSta[txcmd_id].used)
            {
                if (Bzdma_Manager.TxEntSta[txcmd_id].is_sco_lt)
                {
                    /* free sco tx pkt */
                    pf_hci_free_tx_pkt(Bzdma_Manager.TxEntSta[txcmd_id].ch_id);

                    /* invalidate the bzdma */
                    bzdma_invalid_txcmd(txcmd_id, 0, Bzdma_Manager.TxEntSta[txcmd_id].ch_id);
                    /* reset tx queue of fw/hw if alternative setting is changed */
                    if (alternatset_change_tx_clear)
                    {
                        pf_hci_transport_reset_isoch_queue();
                    }
                }
                else
                {
                    /* free esco tx pkt */
                    lc_handle_esco_free_pkts_callback_new(Bzdma_Manager.TxEntSta[txcmd_id].ch_id);

                    /* invalidate the bzdma */
                    bzdma_invalid_txcmd(txcmd_id, 0, Bzdma_Manager.TxEntSta[txcmd_id].ch_id);

                    /* reset tx queue of fw/hw if alternative setting is changed */
                    if (alternatset_change_tx_clear)
                    {
                        lc_reset_esco_fifo_new();
                    }
                }
            }
        }
    }
}
#endif

#ifdef _SUPPORT_BT_CONTROL_PCM_MUX_
#if 0
void switch_pcm_mux_after_con_established()
{

#ifdef _ROM_CODE_PATCHED_

        if (rcp_switch_pcm_mux_after_con_established != NULL)
        {
            if ( rcp_switch_pcm_mux_after_con_established(NULL))
            {
                 return;
            }
        }
#endif

    UINT8 u8data;
    // backup btgpio state
    u32BtGpioStateBkup = VENDOR_READ(REG_BTON_REG_CTRL0);// backup
    u8BtVenBtGpioCtrlBkup = VENDOR_BYTE_READ(REG_VEN_GPIO_CTRL_HIGH_BYTE_3);// backup

    // select pcm mux
    SWITCH_BTON_GPIO_TO_OFF_DOMAIN(BTON_GPIOMAP_00|BTON_GPIOMAP_01|BTON_GPIOMAP_02|BTON_GPIOMAP_03);
    u8data = u8BtVenBtGpioCtrlBkup;
    CLR_BIT(u8data, (BIT7|BIT6));
    VENDOR_BYTE_WRITE(REG_VEN_GPIO_CTRL_HIGH_BYTE_3, u8data);

    RT_BT_LOG(WHITE, FONGPIN_SWITCH_TO_PCM_MUX, 2,u32BtGpioStateBkup, u8BtVenBtGpioCtrlBkup);

}
#endif
void restore_pcm_mux_state_when_disconnect()
{
    // make sure that u32BtGpioStateTmp can be configured by
    // 1. bt efuse
    // 2. config file
    // 3. patch
    // 4. vender command

    // a. switch to on area
    // b. de-sel pcm mux


#ifdef _ROM_CODE_PATCHED_

    if (rcp_restore_pcm_mux_state_when_disconnect != NULL)
    {
        if ( rcp_restore_pcm_mux_state_when_disconnect(NULL))
        {
             return;
        }
    }
#endif
    enum{KEEP_STATE = 0, RESTORE_STATE = 1};
    PARK_PCM_GPIO_PIN_S_TYPE efuse_park_pcm_mux_state;
    *(UINT16*)&efuse_park_pcm_mux_state = otp_str_data.efuse_park_pcm_mux_mgr;

    //RT_BT_LOG(BLUE, YL_DBG_HEX_2, 2,efuse_park_pcm_mux_state.d16, efuse_park_pcm_mux_state.b.restore_pcm_mux_state_sel);


    // 0: keep state, do nothing, return
    // 1: restore state before connection
    if(efuse_park_pcm_mux_state.b.restore_pcm_mux_state_sel != RESTORE_STATE)// default 1
    {
        // do nothing
        return;
    }

    // from efuse
    // set gpio on, off area
    UINT8 i = 0;
    //UINT32 u32BtGpioStateRestore = 0;
    UINT32 bit_mask;

    //u32BtGpioStateRestore = u32BtGpioStateBkup;
    for( i = 0; i<4; i++)
    {
        bit_mask = GEN_BIT_MASK(i);
        gpio_ctrl_set_bton(bit_mask);
        #if 0
        if((efuse_park_pcm_mux_state.d16 & (1<<i)) == 0)
        {   // on area gpio

            //u32BtGpioStateRestore = u32BtGpioStateBkup & (~(1<<i));

            RT_BT_LOG(WHITE, FONGPIN_SWITCH_PCM_TO_ON, 1,i);
        }
        else
        {
            //u32Temp |= (1<<i);
            //u32BtGpioStateRestore |= (1<<i);
            RT_BT_LOG(WHITE, FONGPIN_SWITCH_PCM_TO_OFF, 1,i);
        }
        //RT_BT_LOG(WHITE, YL_DBG_HEX_1, 1,u32BtGpioStateRestore);
        #endif
    }
    //VENDOR_WRITE(REG_BTON_REG_CTRL0, u32BtGpioStateRestore);

    // set output or input
    for( i = 4; i<8; i++)
    {
        if((efuse_park_pcm_mux_state.d16 & (1<<i)) == 0)
        {   // input
            //u32BtGpioStateRestore = u32BtGpioStateRestore & (~(1<<((i-4+1)+20)));
            gpio_ctrl_set_in_out(GEN_BIT_MASK(i-4),0);
            //RT_BT_LOG(GREEN, FONGPIN_SET_PCM_AS_INPUT, 1,i-4);
        }
        else
        {
            //output enable
            //u32BtGpioStateRestore |= (1<<((i-4+1)+20));
            gpio_ctrl_set_in_out(GEN_BIT_MASK(i-4),0x1FFFFF);
            //RT_BT_LOG(GREEN, FONGPIN_SET_PCM_AS_OUTPUT, 1,i-4);


            //for( i = 8; i<12; i++)
            {

                if((efuse_park_pcm_mux_state.d16 & (1<<(i+4))) == 0)
                {   // output low
                    //u32Tmp = u32Tmp & (~(1<<((i-8+1)+9)));
                    gpio_ctrl_write_gpio((i+4)-8,0);
                    RT_BT_LOG(BLUE, FONGPIN_SET_PCM_AS_OUTPUT_LOW, 1,(i+4)-8);
                }
                else
                {
                    //output high
                    //u32Tmp |= (1<<((i-8+1)+9));
                    gpio_ctrl_write_gpio((i+4)-8,1);
                    RT_BT_LOG(BLUE, FONGPIN_SET_PCM_AS_OUTPUT_HIGH, 1,(i+4)-8);

                }
                //RT_BT_LOG(BLUE, YL_DBG_HEX_1, 1,u32Tmp);
            }

        }
        //RT_BT_LOG(GREEN, YL_DBG_HEX_1, 1,u32BtGpioStateRestore);
    }

    //VENDOR_WRITE(REG_BTON_REG_CTRL0, u32BtGpioStateRestore);
#if 0
    // set output value
    //UINT32 u32Tmp = VENDOR_READ(REG_BTON_REG_CTRL1);
    for( i = 8; i<12; i++)
    {

        if((efuse_park_pcm_mux_state.d16 & (1<<i)) == 0)
        {   // output low
            //u32Tmp = u32Tmp & (~(1<<((i-8+1)+9)));
            gpio_ctrl_write_gpio(i-8,0);
            RT_BT_LOG(BLUE, FONGPIN_SET_PCM_AS_OUTPUT_LOW, 1,i-8);
        }
        else
        {
            //output high
            //u32Tmp |= (1<<((i-8+1)+9));
            gpio_ctrl_write_gpio(i-8,1);
            RT_BT_LOG(BLUE, FONGPIN_SET_PCM_AS_OUTPUT_HIGH, 1,i-8);

        }
        //RT_BT_LOG(BLUE, YL_DBG_HEX_1, 1,u32Tmp);
    }
#endif
    //VENDOR_WRITE(REG_BTON_REG_CTRL1, u32Tmp);
    //bton_gpio_ctrl1_reg_write((BTON_GPIO_CTRL1_REG)u32Tmp);

    //---
    // de-sel pcm, restore vendor reg 0x38
    VENDOR_BYTE_WRITE(REG_VEN_GPIO_CTRL_HIGH_BYTE_3, u8BtVenBtGpioCtrlBkup);
}

#if 0
void switch_pcm_mux_after_con_established_pcmoutctrl(UINT16 pcmctrl3)
{
#ifdef _ROM_CODE_PATCHED_

        if (rcp_switch_pcm_mux_after_con_established_pcmoutctrl != NULL)
        {
            if ( rcp_switch_pcm_mux_after_con_established_pcmoutctrl((void*)&pcmctrl3))
            {
                 return;
            }
        }
#endif

    UINT8 u8data;
    UINT32 tmpData32 = 0x00;

    // backup btgpio state
    u32BtGpioStateBkup = VENDOR_READ(REG_BTON_REG_CTRL0);// backup
    u8BtVenBtGpioCtrlBkup = VENDOR_BYTE_READ(REG_VEN_GPIO_CTRL_HIGH_BYTE_3);// backup

    // select pcm mux
    SWITCH_BTON_GPIO_TO_OFF_DOMAIN(BTON_GPIOMAP_00|BTON_GPIOMAP_02|BTON_GPIOMAP_03);
    SWITCH_BTON_GPIO_TO_ON_DOMAIN(BTON_GPIOMAP_01);

    tmpData32 =  (VENDOR_READ(REG_BTON_REG_CTRL0) | 0x400000);
    VENDOR_WRITE(REG_BTON_REG_CTRL0, tmpData32); //GPIO[1]_E Enable

    if ((pcmctrl3 & 0x4000)) //PCMOUT drive HIGH
    {
        tmpData32 =  (VENDOR_READ(REG_BTON_REG_CTRL1) | 0x800);//GPIO[1] Drive HIGH
    }

    if ((pcmctrl3 & 0x2000)) //PCMOUT drive LOW
    {
        tmpData32 =  (VENDOR_READ(REG_BTON_REG_CTRL1) & 0xFFFFF7FF);//GPIO[1] Drive LOW
    }

    VENDOR_WRITE(REG_BTON_REG_CTRL1,  tmpData32);

    RT_BT_LOG(YELLOW, MSG_PCM_MUX_PCMOUT_0, 3, VENDOR_BYTE_READ(REG_VEN_GPIO_CTRL_HIGH_BYTE_3), VENDOR_READ(REG_BTON_REG_CTRL0), VENDOR_READ(REG_BTON_REG_CTRL1));

    u8data = u8BtVenBtGpioCtrlBkup;
    CLR_BIT(u8data, (BIT7|BIT6));
    VENDOR_BYTE_WRITE(REG_VEN_GPIO_CTRL_HIGH_BYTE_3, u8data);

    RT_BT_LOG(WHITE, FONGPIN_SWITCH_TO_PCM_MUX, 2,u32BtGpioStateBkup, u8BtVenBtGpioCtrlBkup);
}//switch_pcm_mux_after_con_established_pcmoutctrl
#endif

void switch_pcm_mux_after_con_established_pcmoutctrl(UINT16 pcmctrl3)
{
#ifdef _ROM_CODE_PATCHED_

        if (rcp_switch_pcm_mux_after_con_established_pcmoutctrl != NULL)
        {
            if ( rcp_switch_pcm_mux_after_con_established_pcmoutctrl((void*)&pcmctrl3))
            {
                 return;
            }
        }
#endif

    UINT8 u8data;
    //UINT32 tmpData32 = 0x00;

    // backup btgpio state
    u32BtGpioStateBkup = VENDOR_READ(REG_BTON_REG_CTRL0);// backup
    u8BtVenBtGpioCtrlBkup = VENDOR_BYTE_READ(REG_VEN_GPIO_CTRL_HIGH_BYTE_3);// backup

    // select pcm mux
    SWITCH_BTON_GPIO_TO_OFF_DOMAIN(BTON_GPIOMAP_00|BTON_GPIOMAP_01|BTON_GPIOMAP_02|BTON_GPIOMAP_03);

    if ((pcmifctrl3 & 0x6000))
    {
        //UINT32 tmpData32 = 0x00;
        SWITCH_BTON_GPIO_TO_ON_DOMAIN(BTON_GPIOMAP_01);

        //UINT32 tmpData32 =  (VENDOR_READ(REG_BTON_REG_CTRL0) | 0x400000);
        //VENDOR_WRITE(REG_BTON_REG_CTRL0, tmpData32); //GPIO[1]_E Enable

        BTON_GPIO_CTRL0_REG btgpio_ctrl0 = bton_gpio_ctrl0_reg_read();
        btgpio_ctrl0.gpio1_out_en = 1;
        bton_gpio_ctrl0_reg_write(btgpio_ctrl0);//GPIO[1]_E Enable

        if ((pcmctrl3 & 0x4000)) //PCMOUT drive HIGH
        {
            //tmpData32 =  (VENDOR_READ(REG_BTON_REG_CTRL1) | 0x800);//GPIO[1] Drive HIGH

            gpio_ctrl_write_gpio(1,1);
        }

        if ((pcmctrl3 & 0x2000)) //PCMOUT drive LOW
        {
            //tmpData32 =  (VENDOR_READ(REG_BTON_REG_CTRL1) & 0xFFFFF7FF);//GPIO[1] Drive LOW

            gpio_ctrl_write_gpio(1,0);
        }

        //VENDOR_WRITE(REG_BTON_REG_CTRL1,  tmpData32);
        //bton_gpio_ctrl1_reg_write((BTON_GPIO_CTRL1_REG)tmpData32);

        RT_BT_LOG(YELLOW, MSG_PCM_MUX_PCMOUT_0, 3, VENDOR_BYTE_READ(REG_VEN_GPIO_CTRL_HIGH_BYTE_3), VENDOR_READ(REG_BTON_REG_CTRL0), VENDOR_READ(REG_BTON_REG_CTRL1));
    }

    u8data = u8BtVenBtGpioCtrlBkup;
    CLR_BIT(u8data, (BIT7|BIT6));
    VENDOR_BYTE_WRITE(REG_VEN_GPIO_CTRL_HIGH_BYTE_3, u8data);

    RT_BT_LOG(WHITE, FONGPIN_SWITCH_TO_PCM_MUX, 2,u32BtGpioStateBkup, u8BtVenBtGpioCtrlBkup);
}

#endif

