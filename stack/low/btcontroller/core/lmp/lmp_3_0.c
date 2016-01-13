/***************************************************************************
 Copyright (C) MindTree Consulting Ltd.
 This module is a confidential and proprietary property of MindTree and
 a possession or use of this module requires written permission of MindTree.
 ***************************************************************************/

/**
 * \file
 *  Contains the functions that create/handle BT 3.0 pdus.
 * 
 */
#ifdef VER_3_0

enum { __FILE_NUM__= 119};

/* ---------------------- Includes ------------------------------------- */
#include "lmp_internal.h"
#include "btc_defines.h"
#include "lmp_3_0.h"
#include "bb_driver_3_0.h"

/* ---------------------- Defines ------------------------------------- */
typedef struct 
{ 
    /** Maximum power for specific modulation scheme */
    CHAR   mod_max_db;
    /** Minimum power for specific modulation scheme */
    CHAR   mod_min_db;
    /** Max step number for specific modulation scheme */
    UCHAR  mod_max_step;
    /** Min step number for specific modulation scheme */
    UCHAR  mod_min_step;
}MODULATION_POWER_DATA;

typedef struct 
{ 
    /** Array for different modulation scheme's data */
    MODULATION_POWER_DATA modulation_power_data[RF_NUM_MODULATIONS];
    /** Maximum power for any modulation scheme */
    CHAR max_db;
    /** Minimum power for any modulation scheme */
    CHAR min_db;
    /** Step size */
    UCHAR step_db;
    /** Number of steps */
    UCHAR num_steps;
    /** Step for power 0db */
    UCHAR start_step;  
    /** Max step number for any modulation scheme */
    // UCHAR max_step; // will be num_steps - 1
    /** Min step number for any modulation scheme */
    // UCHAR min_step; // will be 0
}EPC_POWER_TABLE;

/* This return equivalent power to the given step number */
#define STEP_TO_POWER(step) \
    (step*epc_power_table.step_db + epc_power_table.min_db)
    
/* This returns ceiling step number from given power*/
#define POWER_TO_STEP_CEILING(power) \
    ((power + (epc_power_table.step_db - 1) - epc_power_table.min_db) \
    /epc_power_table.step_db)
    
/* This returns floor step number from given power*/
#define POWER_TO_STEP_FLOOR(power) \
    ((power - epc_power_table.min_db) \
    /epc_power_table.step_db)


/* ----------------------Global variables----------------------------- */
static EPC_POWER_TABLE epc_power_table;

/* ----------------------Static functions----------------------------- */
/** 
 * Increments or decrements the power step for given connection, and 
 * returns the response code.
 * 
 * \param ce_index    Connection entity index. 
 * \param level       Increase/decrease flag - set by caller.
 *
 * \return 0xFF (UCHAR)API_FAILURE on failure else response.
 */
UCHAR lmp_change_pwr(UINT16 ce_index, UCHAR level)
{
    UCHAR cur_step = lmp_connection_entity[ce_index].epc_data.epc_step;
    UCHAR response = 0;
    UINT32 i;
    UINT16 temp_response;

    switch(level)
    {
        case LMP_PWR_CTRL_DEC_ONE_STEP:
            if(cur_step != 0)
            {
                cur_step--;
            }
            break;
        case LMP_PWR_CTRL_INC_ONE_STEP:
            cur_step++;
            break;
        case LMP_PWR_CTRL_INC_UPTO_MAXIMUM:
            cur_step = epc_power_table.num_steps - 1;
            break;
        default:
            return (UCHAR)API_FAILURE;
    }

    if(cur_step >= epc_power_table.num_steps)
    {
        cur_step = epc_power_table.num_steps - 1;
    }

    lmp_connection_entity[ce_index].epc_data.epc_step = cur_step;

    // TODO: This API might change when EPC is implemented in BB	
	BB_write_pwr_level(ce_index, 
		lmp_connection_entity[ce_index].am_addr, cur_step);
    // Appropiate return with values.

    for(i=0;i<RF_NUM_MODULATIONS;i++)
    {
        if(cur_step >= epc_power_table.modulation_power_data[i].mod_max_step)
        {
            temp_response = EPC_MAX_POWER_REACHED;
        }
        else if(cur_step <= 
                epc_power_table.modulation_power_data[i].mod_min_step)
        {
            temp_response = EPC_MIN_POWER_REACHED;
        }
        else
        {
            temp_response = EPC_POWER_SGL_STEP_INC_DEC;
        }
        temp_response <<= (i << 1);
        response = response | temp_response;
    }

    return response;
}


/* --------------------- [Static] PDU Handlers ----------------------- */
/**
 * Handles lmp power control request pdu.
 * 
 * \param lmp_pdu_ptr LMP PDU packet pointer.
 * \param ce_index CE Index.
 * 
 * \return None.
 */
void lmp_handle_lmp_power_ctrl_req_pdu(LMP_PDU_PKT *lmp_pdu_ptr,
        UINT16 ce_index)
{
#ifdef POWER_CONTROL
    UCHAR am_addr;
    UCHAR piconet_id;
    UCHAR action = 0xFF;
    UCHAR power_ctrl_act_res = 0xFF;

    am_addr = lmp_connection_entity[ce_index].am_addr ;
    piconet_id = lmp_connection_entity[ce_index].phy_piconet_id ;

    if(lmp_feature_data.feat_page0[7] & LMP_EPC_FEATURE)
    {
        action = (UCHAR)(lmp_pdu_ptr->payload_content[2] & 0x03);
        if((power_ctrl_act_res = lmp_change_pwr(ce_index, action)) == 
                                                             (UCHAR)API_FAILURE)
        {
            lmp_send_lmp_not_accepted(ce_index,
                    LMP_POWER_CTRL_REQ_OPCODE, 
                    (UCHAR)LMP_GET_TRANSACTION_ID(lmp_pdu_ptr), 
                    INVALID_LMP_PARAMETERS_ERROR);
        }
        else
        {
            /* generate the response */
            lmp_generate_power_ctrl_res_pdu(ce_index, power_ctrl_act_res);
        }
    }
    else /* if(lmp_piconet_data[piconet_id */
#endif /* POWER_CONTROL */
    {
        lmp_send_lmp_not_accepted(ce_index, LMP_POWER_CTRL_REQ_OPCODE,
                (UCHAR)LMP_GET_TRANSACTION_ID(lmp_pdu_ptr),
                UNSUPPORTED_REMOTE_FEATURE_ERROR);
    }

    RT_BT_LOG(YELLOW, LMP_HANDLE_POWER_CTRL_REQ_MSG, 3, 
            (lmp_feature_data.feat_page0[7] & LMP_EPC_FEATURE) ? 1 : 0, 
             action, power_ctrl_act_res);

    return; 
}

/**
 * Handles lmp power control response pdu.
 * 
 * \param lmp_pdu_ptr LMP PDU packet pointer.
 * \param piconet_id  Piconet ID.
 * 
 * \return None/
 */
void lmp_handle_lmp_power_ctrl_res_pdu(LMP_PDU_PKT *lmp_pdu_ptr, 
        UINT16 ce_index)
{
#ifdef POWER_CONTROL
    UCHAR am_addr;
    UCHAR piconet_id;
    UCHAR power_ctrl_res;

    am_addr = lmp_connection_entity[ce_index].am_addr ;
    piconet_id = lmp_connection_entity[ce_index].phy_piconet_id ;

    lmp_connection_entity[ce_index].epc_data.epc_req_sent = FALSE;

    /* 
     * Update the EPC state (depending on the incr/decr).
     * 1) More req's can be queued.
     * Depending on response received.
     */
    /* TODO: Update this when we have rssi - read for different modulations */
    if(lmp_feature_data.feat_page0[7] & LMP_EPC_FEATURE)
    {
        power_ctrl_res = (UCHAR)lmp_pdu_ptr->payload_content[2];
        if(power_ctrl_res == LMP_PWR_ALL_MOD_MAX)
        {
            /* Cannot send any more incrs. */
            lmp_connection_entity[ce_index].power_ctrl_resp = LMP_MAX_POWER_RCVD;
            RT_BT_LOG(GRAY, LMP_MSG_EPC_REACH_MAX_POWER, 0, 0);
        }
        else if(power_ctrl_res == LMP_PWR_ALL_MOD_MIN)
        {
            /* Cannot send any more decrs. */
            lmp_connection_entity[ce_index].power_ctrl_resp = LMP_MIN_POWER_RCVD;
            RT_BT_LOG(GRAY, LMP_MSG_EPC_REACH_MIN_POWER, 0, 0);

        }
    }
    else /* if(lmp_piconet_data[piconet_id */
#endif /* POWER_CONTROL */
    {
        lmp_send_lmp_not_accepted(ce_index, LMP_POWER_CTRL_RES_OPCODE,
                (UCHAR)LMP_GET_TRANSACTION_ID(lmp_pdu_ptr),
                UNSUPPORTED_REMOTE_FEATURE_ERROR);
    }
}

/* ----------------------External functions--------------------------- */

/**
 * Returns the current power level in db.
 * 
 * \param ce_index CE Index.
 * 
 * \return The current power in db.
 */
CHAR lmp_epc_get_cur_pwr_level(UINT16 ce_index)
{
    return STEP_TO_POWER(lmp_connection_entity[ce_index].epc_data.epc_step);
}

/**
 * Returns the maximum power level in db.
 * 
 * \param ce_index CE Index.
 * 
 * \return The maximum power in db.
 */
CHAR lmp_epc_get_max_pwr_level(void)
{
    return epc_power_table.max_db;
}


/**
 * Initializes the EPC module. The main data structures is initialized 
 * is epc_power_table. The max power, min power and number of steps
 * are to be taken from config parameters. Other value are 
 * extrapolated and stored for convenience.
 *
 * \param None.
 *
 * \return None
 */
void lmp_init_epc(void)
{
    UINT8 i;

    epc_power_table.start_step = max_rtk_radio_tx_step_index;
    epc_power_table.num_steps = max_rtk_radio_tx_step_index + 1;
    epc_power_table.max_db = 0;
    epc_power_table.min_db = -3*max_rtk_radio_tx_step_index;
    epc_power_table.step_db = 3;
	
    for (i = 0; i < RF_NUM_MODULATIONS; i++)
    {
        epc_power_table.modulation_power_data[i].mod_max_step = max_rtk_radio_tx_step_index;
        epc_power_table.modulation_power_data[i].mod_min_step = 0;
        epc_power_table.modulation_power_data[i].mod_max_db = 0;
        epc_power_table.modulation_power_data[i].mod_min_db = -3*max_rtk_radio_tx_step_index;
    }
}

/**
 * Initializes the EPC Parameters for a particular ce_index.
 * 
 * \param ce_index CE Index.
 * 
 * \return None.
 */
void lmp_init_epc_connection_entity(UINT16 ce_index)
{
    lmp_connection_entity[ce_index].epc_data.epc_req_sent = FALSE;
    lmp_connection_entity[ce_index].epc_data.epc_step = 
                                            epc_power_table.start_step;
}

/**
 * Handles the 3.0 pdus.
 * 
 * \param lmp_pdu_ptr LMP PDU packet pointer.
 * \param piconet_id  Piconet ID.
 * 
 * \return BT_FW_ERROR or BT_FW_SUCCESS
 */
UCHAR lmp_handle_3_0_incoming_pdu(LMP_PDU_PKT *lmp_pdu_ptr, UINT16 ce_index)
{
    UCHAR pdu_opcode;
    UCHAR pdu_ext_opcode;
    UCHAR am_addr;

    pdu_opcode = (UCHAR)(lmp_pdu_ptr->payload_content[0] >> 1);
    pdu_ext_opcode = lmp_pdu_ptr->payload_content[1];

    //RT_BT_LOG(GRAY, LMP_HANDLE_BT3_0_PDU_MSG, 2, pdu_opcode, pdu_ext_opcode);

    am_addr = lmp_connection_entity[ce_index].am_addr;

    switch(pdu_opcode)
    {
        case LMP_ESCAPE1_OPCODE:
        case LMP_ESCAPE2_OPCODE:
        case LMP_ESCAPE3_OPCODE:
            break;

        case LMP_ESCAPE4_OPCODE:
            switch(pdu_ext_opcode)
            {
                case LMP_POWER_CTRL_REQ_OPCODE:
                        lmp_handle_lmp_power_ctrl_req_pdu(lmp_pdu_ptr,
                                ce_index);
                        break;

                case LMP_POWER_CTRL_RES_OPCODE:
                        lmp_handle_lmp_power_ctrl_res_pdu(lmp_pdu_ptr,
                                ce_index);
                        break;

                default:
                        return BT_FW_ERROR;
            }
            break;
            
        default:
            return BT_FW_ERROR;
    }

    /* need to free BT3.0 req - added by austin */
	if(OS_FREE_BUFFER(lmp_pdu_buffer_pool_handle, lmp_pdu_ptr) != BT_ERROR_OK)
	{
		RT_BT_LOG(GRAY, LMP_EDTM_522, 0, 0);
	}
    
    return BT_FW_SUCCESS;
}

/* --------------------- PDU Generaters ------------------------------ */
/**
 * Generates power control request pdu.
 *
 * \param ce_index CE Index.
 * \param power_adjustment Power Adjustment 
 *
 * \return API_FAILURE if remote device doesnot support EPC.
 */
API_RESULT lmp_generate_power_ctrl_req_pdu(UINT16 ce_index,
                                           UCHAR power_adjustment)
{
    UCHAR parameter_list[LMP_POWER_CTRL_REQ_LEN];
    LMP_TRAN_ID tid;
    LMP_CONNECTION_ENTITY *ce_ptr;
    UCHAR piconet_id;

    ce_ptr = &lmp_connection_entity[ce_index];
    piconet_id = lmp_connection_entity[ce_index].phy_piconet_id;
#ifdef _DAPE_TEST_DEFAULT_MATER_AFH_POWER_CTRL_OFF
    return API_FAILURE;
#endif

    if(((lmp_connection_entity[ce_index].feat_page0[7] & LMP_EPC_FEATURE) == 0)
      || ((lmp_feature_data.feat_page0[7] & LMP_EPC_FEATURE) == 0))
    {
        //RT_BT_LOG(RED, LMP_REM_DEV_NO_SUPPORT_EPC, 0, 0);
        return API_FAILURE;
    }
#ifdef _NO_SEND_AFH_POWER_CTRL_WHILE_ROLE_SW
#ifdef _ENABLE_COMPILE_NESTED_PAUSE_RESUME_
    UCHAR lut_index;
    lut_index = lc_get_lut_index_from_phy_piconet_id(ce_ptr->am_addr, ce_ptr->phy_piconet_id);
    if(paused_lut_index[lut_index])
    {
         RT_BT_LOG(RED, DAPE_TEST_LOG586, 3, ce_index, LMP_ESCAPE4_OPCODE, LMP_POWER_CTRL_REQ_OPCODE);
         return API_FAILURE;
    }
#else
    if(ce_ptr->pause_data_transfer)
    {
         RT_BT_LOG(RED, DAPE_TEST_LOG586, 3, ce_index, LMP_ESCAPE4_OPCODE, LMP_POWER_CTRL_REQ_OPCODE);
         return API_FAILURE;
    }
#endif 
#endif
	RT_BT_LOG(GREEN, LMP_MSG_SEND_PWR_CTRL_REQ, 1, power_adjustment);

    if(ce_ptr->epc_data.epc_req_sent == TRUE)
    {
        return API_SUCCESS;
    }
    ce_ptr->epc_data.epc_req_sent = TRUE;

    parameter_list[0] = LMP_ESCAPE4_OPCODE;
    parameter_list[2] = LMP_POWER_CTRL_REQ_OPCODE;
    parameter_list[3] = power_adjustment;

    if (ce_ptr->remote_dev_role == SLAVE)
    {
        tid = MASTER_TID;
    }
    else
    {
        tid = SLAVE_TID;
    }

    lmp_generate_pdu(ce_index, parameter_list, LMP_POWER_CTRL_REQ_LEN,
                         tid, LMP_NO_STATE_CHANGE);
    return API_SUCCESS;
}

/**
 * Generates power control response pdu.
 *
 * \param ce_index CE Index.
 * \param response_parameter Content
 * 
 *
 * \return None.
 */
void lmp_generate_power_ctrl_res_pdu(UINT16 ce_index, UCHAR response_parameter)
{
    UCHAR parameter_list[LMP_POWER_CTRL_RES_LEN];
    LMP_TRAN_ID tid;

    parameter_list[0] = LMP_ESCAPE4_OPCODE;
    parameter_list[2] = LMP_POWER_CTRL_RES_OPCODE;
    parameter_list[3] = response_parameter;

    if (lmp_connection_entity[ce_index].remote_dev_role == SLAVE)
    {
        tid = SLAVE_TID;
    }
    else
    {
        tid = MASTER_TID;
    }

    lmp_generate_pdu(ce_index, parameter_list, LMP_POWER_CTRL_RES_LEN, 
              tid, LMP_NO_STATE_CHANGE);
}

#endif /* VER_3_0 */
