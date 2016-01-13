/***************************************************************************
 Copyright (C) MindTree Ltd.
 This module is a confidential and proprietary property of MindTree and
 a possession or use of this module requires written permission of MindTree.
 ***************************************************************************/
/********************************* Logger *************************/ 
enum { __FILE_NUM__= 15 };
/********************************* Logger *************************/

/**
 * \file
 *  Contains BTC_2_1 module initialization routines implementation.
 *
 *  \author Muthu Subramanian K
 *  
 */
/* -------------- Includes ------------------------------ */
#include "lmp.h"
#include "lc_internal.h"
#include "lmp_internal.h"
#include "lmp_2_1.h"

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
void init_2_1_connection_entity(UINT16 ce_index)
{
    lmp_connection_entity[ce_index].enhanced_flush = FALSE;   

#ifdef COMPILE_SNIFF_MODE
    /* SSR Parameters */
    lmp_connection_entity[ce_index].ssr_data.neg_in_progress = FALSE;
    lmp_connection_entity[ce_index].ssr_data.min_local_timeout = 0x0;
    lmp_connection_entity[ce_index].ssr_data.min_remote_timeout = 0x0;
    lmp_connection_entity[ce_index].ssr_data.max_latency = 0x0;
    lmp_connection_entity[ce_index].ssr_data.tsniff = 0x0;
    lmp_connection_entity[ce_index].ssr_data.rem_max_ssr = 0x0;
    lmp_reset_ssr_parameters(ce_index);
#endif

    return;
}

/**
 * Initialize 2_1 Self device data variables.
 * 
 * \return None.
 */
void init_2_1_self_device_data(void)
{
    /** Initialize EIR Data structure */
    lmp_self_device_data.eir_data.length = 0x0;
    lmp_self_device_data.eir_data.fec_req = 0x0;
    lmp_self_device_data.eir_data.packet_type = 0x0;
    lmp_self_device_data.eir_data.data = lmp_self_eir_data_buf;
    lmp_self_device_data.inq_id_tx_pwr = RADIO_POWER_VAL;

#ifdef SCO_OVER_HCI
    lmp_self_device_data.default_erroneous_data_reporting = 0x0;
#endif
}

