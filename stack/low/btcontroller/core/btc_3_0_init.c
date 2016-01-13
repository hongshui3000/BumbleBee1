/***************************************************************************
 Copyright (C) MindTree Consulting Ltd.
 This module is a confidential and proprietary property of MindTree and
 a possession or use of this module requires written permission of MindTree.
 ***************************************************************************/

/**
 * \file
 *  Contains BTC_3_0 module initialization routines implementation.
 *
 *  \author Muthu Subramanian K
 *  
 */

#ifdef VER_3_0
/* -------------- Includes ------------------------------ */
#include "lmp.h"
#include "lc_internal.h"
#include "lmp_3_0.h"

/* Move/Include a header file */
void lmp_init_epc(void);

/* -------------- Global Variables ---------------------- */

/* -------------- Static Functions ---------------------- */

/* -------------- External Functions -------------------- */

/** 
 * Initialize 2_1 connection entity variables.
 *
 * \param ce_index Connection Entity Index.
 * 
 * \return None.
 */
void init_3_0_connection_entity(UINT16 ce_index)
{
    lmp_init_epc_connection_entity(ce_index);
    return;
}

/**
 * Initialize 2_1 Self device data variables.
 * 
 * \return None.
 */
void init_3_0_self_device_data(void)
{
  //  HCI_LOG_INFO(LOG_LEVEL_HIGH, "Init 3.0 Self device Data");
    lmp_init_epc();
}

#endif
