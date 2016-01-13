/***************************************************************************
 Copyright (C) Mindtree Ltd.
 This module is a confidential and proprietary property of MindTree and
 a possession or use of this module requires written permission of MindTree.
 ***************************************************************************/

/**
 * \file
 *  Contains the lc 2.1 feature supporting functions.
 * 
 * \author Muthu Subramanian K
 * 
 */
/********************************* Logger *************************/ 
enum { __FILE_NUM__= 38 };
/********************************* Logger *************************/

/* ----------------------Includes------------------------------------- */
#include "lc.h"
#include "lc_2_1.h"

#if defined(RT_VENDOR_CMDS)
#include "hci_vendor_defines.h"
#endif//(RT_VENDOR_CMDS)


/* ----------------------Global variables----------------------------- */

/* ----------------------Static functions----------------------------- */

/* ----------------------External functions--------------------------- */

#ifdef COMPILE_SNIFF_MODE
/**
 * Enters SSR mode.
 * To be called after sniff instant and/or after sniff timeout.
 * 
 * \param ce_index CE Index.
 * 
 * \return None.
 */
void lc_enter_ssr_mode(UINT16 ce_index)
{
	LMP_CONNECTION_ENTITY *ce_ptr;

	ce_ptr = &lmp_connection_entity[ce_index];

	/* Check state */
    if(ce_ptr->ssr_data.lc_ssr_state != LC_SSR_IDLE)
    {
        return;
    }

	ce_ptr->ssr_data.lc_ssr_state = LC_SSR_SUBRATE;

    bb_kill_sniff_in_scatternet(ce_ptr->am_addr, ce_ptr->phy_piconet_id, TRUE);

    /* Start sniff mode with new ssr parameters */
    lc_start_sniff_mode(ce_index, 1);
}

/**
 * Exits SSR mode and moves to sniff mode.
 * 
 * \param ce_index CE Index.
 * 
 * \return None.
 */
void lc_exit_ssr_mode(UINT16 ce_index)
{
	LMP_CONNECTION_ENTITY *ce_ptr;

	ce_ptr = &lmp_connection_entity[ce_index];

	/* Check state */
	if(ce_ptr->ssr_data.lc_ssr_state != LC_SSR_SUBRATE)
    {
        return;
    }

	ce_ptr->ssr_data.lc_ssr_state = LC_SSR_IDLE;

	/* Restart sniff with disabled SSR */
    bb_kill_sniff_in_scatternet(ce_ptr->am_addr, ce_ptr->phy_piconet_id, TRUE);

    lc_start_sniff_mode(ce_index, 2);
}
#endif /* COMPILE_SNIFF_MODE */

/**
 * Writes Broadcast AM-ADDR power level.
 * Used for 'write_inq_tx_pwr_level' to send ID packets.
 * 
 * \note: No error checks are done.
 * 
 * \param val Value to write in upper lut.
 * 
 * \return None.
 */
void lc_write_bc_pwr_level(UCHAR val)
{
    UINT16 temp_val;
    UINT16 address;
    
    address = reg_MASTER_UPPER_LUT[0];

    temp_val = val;
    temp_val <<= 9;

#ifdef _SUPPORT_LBT_FUNC_FOR_RF_ADAPTIVITY_
    if (rtl8723_btrf_check_and_enable_lbt(val))
    {
        temp_val |= BIT12;
    }
#endif                        

    Update_val_with_bb_reg(address, (UINT16)temp_val, (0x07 << 9) | BIT12); 

	/* set 2M/3M modulation table */
	LC_SET_2M_TX_POWER(0, val);
	LC_SET_3M_TX_POWER(0, val);
    
	LC_LOG_INFO(LOG_LEVEL_HIGH,WRITING_TX_POWER_LEVEL,1, val); 
}

