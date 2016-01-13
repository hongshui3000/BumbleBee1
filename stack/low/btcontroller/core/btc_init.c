/***************************************************************************
 Copyright (C) Mindtree Ltd.
 This module is a confidential and proprietary property of MindTree and
 a possession or use of this module requires written permission of MindTree.
 ***************************************************************************/
/********************************* Logger *************************/ 
enum { __FILE_NUM__= 16 };
/********************************* Logger *************************/
#include "bt_fw_globals.h"
#include "btc_defines.h"
#include "lmp.h"
#include "lc_internal.h"
#include "lc_1_2_internal.h"
#include "vendor.h"
#include "mem.h"

#ifdef COMPILE_CHANNEL_ASSESSMENT 
#include "lmp_ch_assessment.h"
#endif


#define LMP_AFH_CH_CL_UNKNOWN    0x00

#ifdef COMPILE_AFH_HOP_KERNEL
extern UCHAR pre_cls_map[LMP_AFH_MAP_SIZE];
#endif /* COMPILE_AFH_HOP_KERNEL */

#ifdef COMPILE_ESCO
extern LMP_ESCO_CONN_HANDLE_TO_CE_INDEX_TABLE
    lmp_esco_ch_to_ce_index_table[LMP_MAX_ESCO_CONN_HANDLES];
extern LMP_ESCO_CONNECTION_ENTITY 
    lmp_esco_connection_entity[LMP_MAX_ESCO_CONN_ENTITIES];
#endif /* COMPILE_ESCO */

#ifdef COMPILE_CHANNEL_ASSESSMENT 
extern TimerHandle_t la_period_timer;
#endif

/**************************************************************************
 * Function     : init_1_2_self_device_data
 *
 * Description  : This function initializes the self device data parameters 
 *                  related to Bluetooth 1.2 features
 *
 * Parameters   : None.
 *
 * Returns      : None.
 *
 *************************************************************************/
void init_1_2_self_device_data(void)
{
    #ifdef _NISH_CHECK_EIR_
    lmp_self_device_data.inquiry_scan_type  = HCI_INTERLACED_INQ_SCAN;
    #else
    lmp_self_device_data.inquiry_scan_type  = HCI_STANDARD_INQ_SCAN;
    #endif
    lmp_self_device_data.inquiry_mode       = HCI_STANDARD_INQ_RESULT_EVENT;
    lmp_self_device_data.page_scan_type     = HCI_STANDARD_PAGE_SCAN;
    lmp_self_device_data.interlaced_inq_scan = FALSE;
    lmp_self_device_data.interlaced_page_scan = FALSE;
    lmp_self_device_data.afh_channel_assessment_mode = AFH_ENABLE;

#ifdef COMPILE_AFH_HOP_KERNEL
    lmp_self_device_data.cl_rep_min_interval = LMP_CH_REPORTING_MIN_INTERVAL;
    lmp_self_device_data.cl_rep_max_interval = LMP_CH_REPORTING_MAX_INTERVAL;
#endif /* COMPILE_AFH_HOP_KERNEL */

    return;
}

/**************************************************************************
 * Function   : init_1_2_connection_entity_after_hlc
 *
 * Description: This Function initializes the Connection Entity parameters 
 *              related to the Bluetooth 1.2 features, after Hardware level 
 *              connection is setup, or successful MSS.
 *
 * Parameters : None.
 *
 * Returns    : None.
 *
 *************************************************************************/
void init_1_2_connection_entity_after_hlc(UINT16 ce_index)
{
#if defined(COMPILE_AFH_HOP_KERNEL)
    LMP_CONNECTION_ENTITY *ce_ptr = &lmp_connection_entity[ce_index];
#endif

#if defined(COMPILE_AFH_HOP_KERNEL)
    UINT32 i;
#endif

#ifdef COMPILE_AFH_HOP_KERNEL
    ce_ptr->afh_min_interval                    = 0;
    ce_ptr->afh_max_interval                    = 0;
    ce_ptr->afh_ch_cl_reporting_mode            = AFH_DISABLE;
    ce_ptr->waiting_for_set_afh_pdu_ack         = FALSE;
    ce_ptr->need_to_send_ch_cls_req             = FALSE;
    ce_ptr->need_to_send_ch_cls                 = FALSE;
    ce_ptr->last_set_afh_sent_clk               = 0xFFFFFFFF;

#ifdef COMPILE_PARK_MODE
    ce_ptr->park_pending_for_afh                = FALSE;
    ce_ptr->afh_disabled_for_park               = FALSE;
    ce_ptr->rem_park_pending_for_afh            = FALSE;
#endif

    ce_ptr->afh_mode = AFH_DISABLE;
    for(i=0;i<LMP_AFH_MAP_SIZE;i++)
    {
        ce_ptr->afh_map[i] = 0xFF;
        ce_ptr->last_recd_ch_cl_map[i] = LMP_AFH_CH_CL_UNKNOWN;
    }
    
    ce_ptr->new_ch_cl_map = 0x0;
#endif /* COMPILE_AFH_HOP_KERNEL */

#ifdef COMPILE_AFH_HOP_KERNEL
    ce_ptr->afh_instant_timer_handle = NULL;
    ce_ptr->mss_cmd_pending = FALSE;
    ce_ptr->mss_pdu_pending = FALSE;
    ce_ptr->status_event_mss_cmd_pending = FALSE;
#endif

    return;
}

/**************************************************************************
* Function   : init_1_2_connection_entity
*
* Description: This Function initializes the Connection Entity parameters 
*              related to the Bluetooth 1.2 features
*
* Parameters : None.
*
* Returns    : None.
*
*************************************************************************/
void init_1_2_connection_entity(UINT16 ce_index)
{
	init_1_2_connection_entity_after_hlc(ce_index);

	lmp_connection_entity[ce_index].ptt_status = LMP_PTT_IDLE;

#ifdef _CCH_IOT_CSR_RS_
    lmp_connection_entity[ce_index].waiting_for_rs_several_times = FALSE;
#endif	
	return;
}

#ifdef COMPILE_ESCO
/**************************************************************************
 * Function   : init_esco_ce_mapping_tables
 *
 * Description: This function initializes the tables which map AM_ADDR and
 *              eSCO connection handles to eSCO connection entities. 
 *
 * Parameters : None.
 *
 * Returns    : None.
 *
 *************************************************************************/
void init_esco_ce_mapping_tables(void)
{
    UINT16 index;

    for(index = 0; index < LMP_MAX_CONN_HANDLES; index++)
    {
        memset(&lmp_esco_ch_to_ce_index_table[index],0,
               sizeof(LMP_ESCO_CONN_HANDLE_TO_CE_INDEX_TABLE));
    }
}


/**************************************************************************
 * Function   : init_esco_connection_entity
 *
 * Description: This function initalizes connection entity table for eSCO.
 *
 * Parameters : None.
 *
 * Returns    : None.
 *
 *************************************************************************/
void init_esco_connection_data(void)
{
    UCHAR i;

    lc_init_esco_scheduler();

    for(i = 0; i < LMP_MAX_ESCO_CONN_ENTITIES; i++)
    {
        lmp_init_esco_connection_entity(i, (UCHAR) (i + 1) );
    }

    /* Reset the temp eSCO connection entity */
    lmp_reset_temp_esco_connection_entity();

    /* Reset the global variables used by eSCO feature. */
    lmp_reset_esco_global_variables();
}
#endif /* COMPILE_ESCO */


