/***************************************************************************
 Copyright (C) MindTree Consulting Ltd.
 This module is a confidential and proprietary property of MindTree and
 a possession or use of this module requires written permission of MindTree.
 ***************************************************************************/

/**
 * \file
 *  Contains the baseband 3.0 feature interfaces.
 *
 */

#ifdef VER_3_0

/* ----------------------Includes------------------------------------- */
#include "bt_fw_types.h"
#include "bb_driver.h"
#include "bb_driver_3_0.h"
#include "lc.h"

/* ----------------------Global variables----------------------------- */

/* ----------------------Static functions----------------------------- */

/* ----------------------External functions--------------------------- */

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
void BB_write_pwr_level(UINT16 ce_index, UCHAR am_addr, CHAR dblevel)
{
    lc_program_power_level_in_lut_for_scatternet(am_addr,
            (UINT16) dblevel, ce_index);
}
#endif
