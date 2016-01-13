/***************************************************************************
 Copyright (C) MindTree Consulting Ltd.
 This module is a confidential and proprietary property of MindTree and
 a possession or use of this module requires written permission of MindTree.
 ***************************************************************************/

/**
 * \file
 *  Contains the baseband 4.1 feature interfaces.
 *
 */
#ifdef _SUPPORT_VER_4_1_
enum { __FILE_NUM__= 137};


/* ----------------------Includes------------------------------------- */
#include "bt_fw_types.h"
#include "bb_driver.h"
#include "bb_driver_4_1.h"
#include "lc.h"

/* ----------------------Global variables----------------------------- */

/* ----------------------Static functions----------------------------- */

/* ----------------------External functions--------------------------- */
#ifdef _ROM_CODE_PATCHED_
PF_ROM_CODE_PATCH_FUNC rcp_bb_set_piconet_adjust_func = NULL;
PF_ROM_CODE_PATCH_FUNC rcp_bb_prepare_clk_adj_broadcast_func = NULL;
#endif
/**
 * Calculates the absoulte value for the dblevel required and then
 * writes it to the baseband radio power register(s).
 * The dblevel value may be approximated.
 * NOTE: piconet_id is not used - scatternet.
 * 
 * \param am_addr AM Address.
 * \param piconet_id Scatternet piconet id.
 * \param level Absoulte value to write to the register.
 * 
 * \return None.
 */
void BB_set_piconet_adjust(UCHAR piconet_id, UINT32 clk_slot, UINT16 clk_cnt, UINT32 instant)
{
    UINT32 clk;
    UINT16 clk_bit0 = 0;
    if (clk_cnt < 312)
    {
        clk_bit0 = 1;
    }    
    lc_get_clock_in_scatternet(&clk, piconet_id);
    BB_write_baseband_register(MWS_CLKADJ_CTRL2, instant - 1); /* "-1" is hw limitation.*/


    BB_write_baseband_register(MWS_CLKADJ_CTRL0, (clk_slot << 1)+ clk_bit0);
    BB_write_baseband_register(MWS_CLKADJ_CTRL1, ((clk_slot << 1)>>16) 
    | (piconet_id <<12) | BIT15);


    BZ_REG_S_MWS_CLKADJ_CTRL3 reg272;
    *(UINT16*) &reg272 = BB_read_baseband_register(MWS_CLKADJ_CTRL3);
    reg272.clk_adj_cnt9_0 = clk_cnt;
    reg272.clk_adj_inst21_16 = ((instant - 1)>> 16);
    BB_write_baseband_register(MWS_CLKADJ_CTRL3, *(UINT16*)&reg272);
#ifdef _ROM_CODE_PATCHED_
    if (rcp_bb_set_piconet_adjust_func != NULL)
    {
        if (rcp_bb_set_piconet_adjust_func((void*)&piconet_id, &clk_slot, &clk_cnt, &instant))
        {
            return;
        }
    }
#endif
    
    RT_BT_LOG(GREEN, DAPE_TEST_LOG562, 9, clk,
    instant - 1, BB_read_baseband_register(MWS_CLKADJ_CTRL2)| ((BB_read_baseband_register(MWS_CLKADJ_CTRL3)>>10)<<16),
    (((clk_slot << 1)>>16) | (piconet_id <<12) | BIT15),
    ((clk_slot << 1) + clk_bit0)&((UINT16)(0xFFFF)), clk_cnt, 
    BB_read_baseband_register(MWS_CLKADJ_CTRL1),
    BB_read_baseband_register(MWS_CLKADJ_CTRL0),
    BB_read_baseband_register(MWS_CLKADJ_CTRL3)&(0x3FF));

}

void BB_prepare_clk_adj_broadcast(UCHAR piconet_id)
{
    UINT16 address;
    UINT16 lut_contents;
    UINT8 pdu_length;
    
    pdu_length = 15;

    /* Write nbc */
    BB_write_baseband_register_lower_octet(RE_TRANSMISSION_COUNT_REGISTER, 10);

    BB_write_baseband_TX_FIFO_scatternet (
        bzdma_tx_buf[piconet_id],
        pdu_length, BC_AM_ADDR, piconet_id);

    address = (UINT16) (PICONET_LOOK_UP_TABLE_BASE_ADDR);

    lut_contents = (BB_DM1 << 12) | (0x03 << 10) |
                    pdu_length;
    UINT16 read = BB_read_baseband_register(
                    PICONET_LOOK_UP_TABLE_BASE_ADDR + 2);
    read &= 0xE1FF;
    read |= MAX_RADIO_TX_POWER << 9;

#ifdef _SUPPORT_LBT_FUNC_FOR_RF_ADAPTIVITY_
    if (rtl8723_btrf_check_and_enable_lbt(MAX_RADIO_TX_POWER))
    {
        /* enable lbt_en_lut bit */
        read |= BIT12;
    }
#endif
    
    BB_write_baseband_register(address + 2, read);
    BB_write_baseband_register(address, lut_contents);

    LC_SET_2M_TX_POWER(0,MAX_RADIO_TX_POWER);
    LC_SET_3M_TX_POWER(0,MAX_RADIO_TX_POWER);

    /*RT_BT_LOG(GREEN, DAPE_TEST_LOG525, 6, 
    BB_read_baseband_register(0x70),
    bzdma_tx_buf[piconet_id][0],
    bzdma_tx_buf[piconet_id][1],
    bzdma_tx_buf[piconet_id][2],
    bzdma_tx_buf[piconet_id][3],
    bzdma_tx_buf[piconet_id][4]);
*/

#ifdef _ROM_CODE_PATCHED_
    if (rcp_bb_prepare_clk_adj_broadcast_func != NULL)
    {
        if (rcp_bb_prepare_clk_adj_broadcast_func((void*)&piconet_id))
        {
            return;
        }
    }
#endif

    BB_write_baseband_register(CONNECTOR_REGISTER,
                    (UINT16) ( (BC_AM_ADDR << 5) | (piconet_id << 11)) ) ;

    BB_write_baseband_register(INSTRUCTION_REGISTER, BB_SEND_PACKET);


}
#endif
