/***************************************************************************
 Copyright (C) MindTree Ltd.
 This module is a confidential and proprietary property of MindTree and
 a possession or use of this module requires written permission of MindTree.
 ***************************************************************************/

/**
 * \file
 *  Contains the functions that create/handle BT 2.1 pdus.
 * 
 * \author Muthu Subramanian K
 * 
 */
/********************************* Logger *************************/ 
enum { __FILE_NUM__= 48 };
/********************************* Logger *************************/

/* ----------------------Includes------------------------------------- */
#include "lmp_internal.h"
#include "lmp_2_1.h"
#include "btc_defines.h"
#include "bt_fw_acl_q.h"
#include "bt_fw_hci_2_1.h"
#include "mem.h"
/* ----------------------Global variables----------------------------- */

/* ----------------------Static functions----------------------------- */

#ifdef COMPILE_SNIFF_MODE
/**
 * Calculates the sniff_subrate (local)
 * 
 * \param ce_index CE Index.
 * 
 * \return None.
 */
UCHAR lmp_calculate_sniff_subrate(UINT16 ce_index)
{
    UCHAR max_sniff_subrate;
    UINT16 tsniff;
    UINT16 super_to;
    UINT16 sniff_interval;
    UINT16 sniff_attempt;
    UCHAR temp_ssr;

	LMP_CONNECTION_ENTITY *ce_ptr;

	ce_ptr = &lmp_connection_entity[ce_index];

	sniff_interval = ce_ptr->sniff_interval;
	super_to = ce_ptr->link_supervision_timeout;
	sniff_attempt = ce_ptr->sniff_attempt;

	max_sniff_subrate = 
		(UCHAR)( (ce_ptr->ssr_data.max_latency)/sniff_interval);

    /* Self device tsniff */
    tsniff = (UINT16) (max_sniff_subrate * sniff_interval);

    /** Check for supervision timeout */
    if(tsniff > super_to && super_to != 0)
    {
         tsniff = super_to;
         max_sniff_subrate = (UCHAR)(tsniff/sniff_interval);
         if(max_sniff_subrate != 0)
         {
            max_sniff_subrate--;
         }
    }
    
    /** Check for sniff attempt rules */
    tsniff = (UINT16) (max_sniff_subrate*sniff_interval);
    if(sniff_attempt < 20 && max_sniff_subrate != 1)
    {
        if(sniff_attempt < ( ((tsniff) >> 12) << 2) )
        {
            tsniff = (UINT16) (sniff_attempt>>2);
            tsniff = (UINT16) (tsniff << 10);
            tsniff += 0x1000;

            temp_ssr = (UCHAR)(tsniff/sniff_interval);
            
            if(temp_ssr < max_sniff_subrate)
            {
                max_sniff_subrate = temp_ssr;
            }
            else
            {
				LMP_LOG_ERROR(LOG_LEVEL_HIGH, SUBRATE_CHECK_REQUIRED, 2, temp_ssr, max_sniff_subrate);
            }
        }
    }

    if(max_sniff_subrate == 0)
    {
        max_sniff_subrate = 1;
    }

	LMP_LOG_INFO(LOG_LEVEL_HIGH, ML_SN_SUBRATE, 3,
		ce_ptr->ssr_data.max_latency,
		ce_ptr->sniff_interval,
		max_sniff_subrate);

    return max_sniff_subrate;
}

/**
 * Calculates the new tsniff for SSR (local) 
 * (Updates ssr_data.tsniff).
 * 
 * Note: Does not implement 'j' specified in spec.
 * 
 * \param ce_index
 * 
 * \return None.
 */
void lmp_calculate_ssr_tsniff(UINT16 ce_index)
{
    LMP_CONNECTION_ENTITY *ce_ptr;
    UINT16 ssr = lmp_calculate_sniff_subrate(ce_index);

    ce_ptr = &lmp_connection_entity[ce_index];

    if(ssr > ce_ptr->ssr_data.rem_max_ssr)
    {
        ssr = ce_ptr->ssr_data.rem_max_ssr;
    }

    ce_ptr->ssr_data.tsniff = (UINT16)(ce_ptr->sniff_interval * ssr);
	LMP_LOG_INFO(LOG_LEVEL_HIGH, NEW_SSR_TSNIFF, 1, ce_ptr->ssr_data.tsniff);

	return;
}

/**
 * Programs Sniff subrating parameters on successful negotiation.
 * 
 * \param ce_index CE Index.
 * 
 * \return None.
 */
void lmp_program_ssr_parameters(UINT16 ce_index)
{
	LMP_CONNECTION_ENTITY *ce_ptr;

	ce_ptr = &lmp_connection_entity[ce_index];

	ce_ptr->ssr_data.lmp_ssr_state = LMP_SSR_TRANSITION;

	/* Calculate ssr_tsniff */
	lmp_calculate_ssr_tsniff(ce_index);

	if(ce_ptr->ssr_data.tsniff <= ce_ptr->sniff_interval)
	{
		/* Disable SSR */
		ce_ptr->ssr_data.lmp_ssr_state = LMP_SSR_TRIED;
		LMP_LOG_INFO(LOG_LEVEL_HIGH, SSR_IS_DISABLED, 0, 0);
    }

	return;
}

/**
 * Calculates the sniff sub rate instant.
 * 
 * \param ce_index CE Index.
 * 
 * \return None.
 */
void lmp_calculate_ssr_instant(UINT16 ce_index)
{
    UINT32 ssr_instant;
    UINT32 master_clock;
    UINT32 clock_msb;
    UINT32 dsniff;
    UINT32 tsniff; 
    LMP_CONNECTION_ENTITY *ce_ptr = &lmp_connection_entity[ce_index];

    dsniff = ce_ptr->sniff_slot_offset;
    tsniff = ce_ptr->sniff_interval;

    ssr_instant = tsniff * LMP_SSR_INSTANT_MUL;

    if(ce_ptr->sniff_attempt < LMP_SSR_INSTANT_MIN_ATTEMPT)
    {
        ssr_instant <<= 1;
    }
    if(ssr_instant < LMP_MIN_SSR_INSTANT_OFFSET)
    {
        ssr_instant = LMP_MIN_SSR_INSTANT_OFFSET;
    }

    /** SSR instant is only given by the master */
	lc_get_clock_in_scatternet(&master_clock, ce_ptr->phy_piconet_id);

	master_clock >>= 1;
	clock_msb = master_clock & BIT26;
	master_clock &= BT_CLOCK_26_BITS;

	/** Find next valid sniff anchor point 
	* greater than current clock + ssr_instant
	*/
	ssr_instant += master_clock;
	ssr_instant = (ssr_instant / tsniff) + 1;
	ssr_instant = dsniff + (tsniff * ssr_instant);
	ssr_instant ^= clock_msb;

#if 0
	LMP_LOG_INFO(LOG_LEVEL_HIGH, CALCULATED_INSTANT, 1, ssr_instant);
#endif

	ce_ptr->ssr_data.ssr_instant = ssr_instant;

	return;
}

/* --------------------- [Static] PDU Handlers ----------------------- */
/**
 * Handles the lmp_sniff_subrating_req PDU.
 * 
 * \param lmp_pdu_ptr LMP PDU packet pointer.
 * \param ce_index CE Index.
 * 
 * \return None.
 */
void lmp_handle_sniff_subrating_req_pdu(LMP_PDU_PKT *lmp_pdu_ptr, 
                                                            UINT16 ce_index)
{
    UINT16 min_sniff_mode_timeout;
    UINT32 sniff_subrating_instant;
    UCHAR max_sniff_subrate;
    UCHAR tid;
    UCHAR status = 0;

	LMP_CONNECTION_ENTITY *ce_ptr;

	ce_ptr = &lmp_connection_entity[ce_index];

	tid = (UCHAR) (lmp_pdu_ptr->payload_content[0] & 1);

	/**
	* Note: Don't change the order of error checking - it is in 
	* the reverse order.
	*/
	if(ce_ptr->ssr_data.neg_in_progress == TRUE &&
		ce_ptr->remote_dev_role == SLAVE)
	{
		RT_BT_LOG(GRAY, LMP_2_1_267, 0, 0);
		status = LMP_ERROR_TRANSACTION_COLLISION_ERROR;
	}

        if(ce_ptr->ce_status != LMP_SNIFF_MODE)
	{
		status = PDU_NOT_ALLOWED_ERROR;
	}
#ifdef _SUPPORT_VER_4_1_
#ifdef _SUPPORT_PCA_ADJUST
        if (IS_BT41)
        {
            if (bt_pca_manager.pca_updating)
            {
                status = PDU_NOT_ALLOWED_ERROR;
            }    
        }
#endif
#endif

	/* Check for local features */
	if((lmp_feature_data.feat_page0[5] & SNIFF_SUBRATING) == 0)
	{
        status = UNSUPPORTED_FEATURE_OR_PARAMETER_ERROR;
    }

    /* Check if not accepted needs to be sent */
    if(status != 0)
    {
        lmp_send_lmp_not_accepted_ext(
                                ce_index,
                                (UCHAR)(lmp_pdu_ptr->payload_content[0]>>1),
                                lmp_pdu_ptr->payload_content[1],
                                tid,
                                status
                                     );
        return;
    }
    
    max_sniff_subrate = lmp_pdu_ptr->payload_content[2];
    min_sniff_mode_timeout = (UINT16)((lmp_pdu_ptr->payload_content[3]) |
                                      (lmp_pdu_ptr->payload_content[4] << 8));
    BT_FW_EXTRACT_32_BITS(sniff_subrating_instant,
                                 &(lmp_pdu_ptr->payload_content[5]));

    /* Store values */
	ce_ptr->ssr_data.rem_max_ssr = max_sniff_subrate;
	ce_ptr->ssr_data.rem_min_timeout = min_sniff_mode_timeout;
	if(ce_ptr->remote_dev_role == MASTER)
	{
		ce_ptr->ssr_data.ssr_instant = sniff_subrating_instant;
	}

    /* Queue the res pdu */
    lmp_generate_sniff_subrating_res_pdu(ce_index);

    /* Program ssr parameters to LMP */
    lmp_program_ssr_parameters(ce_index);

    hci_generate_sniff_subrating_event(HCI_COMMAND_SUCCEEDED,ce_index);

    return;
}

/**
 * Handles the lmp_sniff_subrating_res PDU.
 * 
 * \param lmp_pdu_ptr LMP PDU packet pointer.
 * \param ce_index CE Index.
 * 
 * \return None.
 */
void lmp_handle_sniff_subrating_res_pdu(LMP_PDU_PKT *lmp_pdu_ptr, 
                                                            UINT16 ce_index)
{
    UINT16 min_sniff_mode_timeout;
    UINT32 sniff_subrating_instant;
    UCHAR max_sniff_subrate;

	LMP_CONNECTION_ENTITY *ce_ptr;

	ce_ptr = &lmp_connection_entity[ce_index];

	max_sniff_subrate = lmp_pdu_ptr->payload_content[2];
	min_sniff_mode_timeout = (UINT16)((lmp_pdu_ptr->payload_content[3]) |
		                            (lmp_pdu_ptr->payload_content[4] << 16));

    sniff_subrating_instant = lmp_pdu_ptr->payload_content[5] |
                            (lmp_pdu_ptr->payload_content[6] << 8) |
                            (lmp_pdu_ptr->payload_content[7] << 16) |
                            (lmp_pdu_ptr->payload_content[8] << 24);

	RT_BT_LOG(GRAY, LMP_2_1_349, 3,
		max_sniff_subrate,min_sniff_mode_timeout, sniff_subrating_instant);
    
	/* Store values */
	ce_ptr->ssr_data.rem_max_ssr = max_sniff_subrate;
	ce_ptr->ssr_data.rem_min_timeout = min_sniff_mode_timeout;
	if(ce_ptr->remote_dev_role == MASTER)
	{
		ce_ptr->ssr_data.ssr_instant = sniff_subrating_instant;
	}

	ce_ptr->ssr_data.neg_in_progress = FALSE;

    /* Program ssr parameters to LMP */
    lmp_program_ssr_parameters(ce_index);

    hci_generate_sniff_subrating_event(HCI_COMMAND_SUCCEEDED,ce_index);

    return;
}
#endif /* COMPILE_SNIFF_MODE */

/* ----------------------External functions--------------------------- */

/**
 * Handle 2_1 (incoming) PDUs.
 * 
 * \param lmp_pdu_ptr LMP PDU packet pointer.
 * \param piconet_id  Piconet ID.
 * 
 * \return BT_FW_SUCCESS or BT_FW_ERROR.
 */
UCHAR lmp_handle_2_1_incoming_pdu(LMP_PDU_PKT *lmp_pdu_ptr, UINT16 ce_index)
{
	UCHAR pdu_opcode;
	UCHAR pdu_ext_opcode;

	pdu_opcode = (UCHAR)(lmp_pdu_ptr->payload_content[0] >> 1);

	LMP_LOG_INFO(LOG_LEVEL_LOW, PDU_OPCODE, 2, pdu_opcode,
		lmp_pdu_ptr->payload_content[1]);	

    switch(pdu_opcode)
    {
       case LMP_ESCAPE1_OPCODE:
       case LMP_ESCAPE2_OPCODE:
       case LMP_ESCAPE3_OPCODE:
            break;

        case LMP_ESCAPE4_OPCODE:
            pdu_ext_opcode = lmp_pdu_ptr->payload_content[1];
            switch(pdu_ext_opcode)
            {
#ifdef COMPILE_SNIFF_MODE
                case LMP_SNIFF_SUBRATING_REQ_OPCODE:
                    lmp_handle_sniff_subrating_req_pdu(lmp_pdu_ptr, ce_index);
                    break;

                case LMP_SNIFF_SUBRATING_RES_OPCODE:
                    lmp_handle_sniff_subrating_res_pdu(lmp_pdu_ptr, ce_index);
                    break;
#endif

                default:
                    return BT_FW_ERROR;
            }
            break;

        default:
            return BT_FW_ERROR;
    }

    return BT_FW_SUCCESS;
}

#ifdef COMPILE_SNIFF_MODE
/**
 * Disable SSR on transmission/reception of supervision timeout pdu.
 * 
 * \param ce_index CE Index.
 * 
 * \return None.
 */
void lmp_ssr_disable(UINT16 ce_index)
{
	LMP_CONNECTION_ENTITY *ce_ptr;

	ce_ptr = &lmp_connection_entity[ce_index];

	LMP_LOG_INFO(LOG_LEVEL_LOW, DISABLING_SSR, 0,0);

	if(ce_ptr->ssr_data.lc_ssr_state != LC_SSR_IDLE &&
		ce_ptr->ssr_data.lmp_ssr_state != LMP_SSR_IDLE)
	{
		ce_ptr->ssr_data.lmp_ssr_state = LMP_SSR_IDLE;
		/* reset_ssr_params will be called again from exit_ssr */
		return;
	}

	if(ce_ptr->ssr_data.lmp_ssr_state != LMP_SSR_IDLE)
	{
		lmp_reset_ssr_parameters(ce_index);
	}

	return;
}

/* --------------------- PDU Generaters ------------------------------ */
/**
 * Generates Sniff Subrating Request pdu.
 * 
 * \param ce_index CE Index.
 * 
 * \return None.
 */
void lmp_generate_sniff_subrating_req_pdu(UINT16 ce_index)
{
    UCHAR parameter_list[LMP_SNIFF_SUBRATING_REQ_LEN];
    LMP_TRAN_ID tid;

	LMP_CONNECTION_ENTITY *ce_ptr;

	ce_ptr = &lmp_connection_entity[ce_index];

	/* Change State to Transition mode */
	ce_ptr->ssr_data.lmp_ssr_state = LMP_SSR_TRANSITION;
	ce_ptr->ssr_data.ssr_instant = FORCE_FLUSH_CLOCK;

	/* Reset ssr_tsniff value */
	ce_ptr->ssr_data.tsniff = ce_ptr->sniff_interval;

	parameter_list[0] = LMP_ESCAPE4_OPCODE;
	parameter_list[2] = LMP_SNIFF_SUBRATING_REQ_OPCODE;
	parameter_list[3] = lmp_calculate_sniff_subrate(ce_index);
    parameter_list[4] = LSB(ce_ptr->ssr_data.min_remote_timeout);
    parameter_list[5] = MSB(ce_ptr->ssr_data.min_remote_timeout);    

	if(ce_ptr->remote_dev_role == SLAVE)
	{
		/* Fill Valid Instant */
		lmp_calculate_ssr_instant(ce_index);
        parameter_list[6] = (UINT8)ce_ptr->ssr_data.ssr_instant;
        parameter_list[7] = (UINT8)(ce_ptr->ssr_data.ssr_instant >> 8);        
        parameter_list[8] = (UINT8)(ce_ptr->ssr_data.ssr_instant >> 16);
        parameter_list[9] = (UINT8)(ce_ptr->ssr_data.ssr_instant >> 24);
        tid = MASTER_TID;
    }
    else
    {
        /* Fill zero instant */
        parameter_list[6] = 0;
        parameter_list[7] = 0;        
        parameter_list[8] = 0;
        parameter_list[9] = 0;        
        tid = SLAVE_TID;
    }


    ce_ptr->lmp_expected_pdu_opcode  |= lmp_get_opcode_mask( LMP_ESCAPE4_OPCODE,
                                                LMP_SNIFF_SUBRATING_RES_OPCODE);
    
    lmp_generate_pdu(ce_index, parameter_list, LMP_SNIFF_SUBRATING_REQ_LEN,
            tid, LMP_NO_STATE_CHANGE);

    return;
}

/**
 * Generates Sniff Subrating Response pdu.
 * 
 * \param ce_index CE Index.
 * 
 * \return None.
 */
void lmp_generate_sniff_subrating_res_pdu(UINT16 ce_index)
{
    UCHAR parameter_list[LMP_SNIFF_SUBRATING_RES_LEN];
    LMP_TRAN_ID tid;

	LMP_CONNECTION_ENTITY *ce_ptr;

	ce_ptr = &lmp_connection_entity[ce_index];

	parameter_list[0] = LMP_ESCAPE4_OPCODE;
	parameter_list[2] = LMP_SNIFF_SUBRATING_RES_OPCODE;
	parameter_list[3] = lmp_calculate_sniff_subrate(ce_index);
    parameter_list[4] = LSB(ce_ptr->ssr_data.min_remote_timeout);
    parameter_list[5] = MSB(ce_ptr->ssr_data.min_remote_timeout);

	if(ce_ptr->remote_dev_role == SLAVE)
	{
		/* Calculate instant */
		lmp_calculate_ssr_instant(ce_index);
		tid = SLAVE_TID;
	}
	else
	{
		tid = MASTER_TID;
	}

	/* Master [Self device/remote device] calculated SSR Instant */
	memcpy(&parameter_list[6], &ce_ptr->ssr_data.ssr_instant,4);

    lmp_generate_pdu(ce_index, parameter_list, LMP_SNIFF_SUBRATING_RES_LEN,
            tid, LMP_NO_STATE_CHANGE);

    return;
}

/** 
 * Resets the lmp ssr parameters.
 * 
 * \param ce_index CE Index.
 * 
 * \return None.
 */
void lmp_reset_ssr_parameters(UINT16 ce_index)
{
	SSR_DATA *ssr_data_ptr;
	DEF_CRITICAL_SECTION_STORAGE;

	MINT_OS_ENTER_CRITICAL();
	ssr_data_ptr = & (lmp_connection_entity[ce_index].ssr_data);

	ssr_data_ptr->lmp_ssr_state = LMP_SSR_IDLE;
	ssr_data_ptr->lc_ssr_state = LC_SSR_IDLE;
	ssr_data_ptr->neg_in_progress = FALSE;

	/**
	*  Note: don't reset other ssr variables that will have to be reused when
	*  re-entering sniff
	*/
	ssr_data_ptr->ssr_instant = FORCE_FLUSH_CLOCK;
	ssr_data_ptr->enter_exit_ssr = 0;
	ssr_data_ptr->prev_pkt_time = FORCE_FLUSH_CLOCK;
	MINT_OS_EXIT_CRITICAL();

	return;
}

/**
 * Initiates Sniff Subrating. Queues lmp_sniff_subrating_req PDU.
 * 
 * \param ce_index CE Index.
 * 
 * \return Status.
 */
UCHAR lmp_initiate_sniff_subrating(UINT16 ce_index)
{
    UCHAR max_sniff_subrate;
	LMP_CONNECTION_ENTITY *ce_ptr;
	SSR_DATA *ssr_data_ptr;

	ce_ptr = &lmp_connection_entity[ce_index];
	ssr_data_ptr = & (ce_ptr->ssr_data);

	/* Check features */
	if(((lmp_feature_data.feat_page0[5] & SNIFF_SUBRATING) == 0) ||
		((ce_ptr->feat_page0[5]) & SNIFF_SUBRATING) == 0)
	{
		return BT_FW_ERROR;
	}

	/* State check */
	if(ssr_data_ptr->neg_in_progress == TRUE)
	{
		return BT_FW_ERROR;
	}

    if(ce_ptr->ce_status != LMP_SNIFF_MODE)
	{
		LMP_LOG_INFO(LOG_LEVEL_HIGH, NOT_ENTERING_SSR_NOT_IN_SNIFF_MODE, 0,0);

		return BT_FW_ERROR;
	}

	max_sniff_subrate = (UCHAR)(
		(ssr_data_ptr->max_latency)/ce_ptr->sniff_interval);

	if(ssr_data_ptr->lmp_ssr_state == LMP_SSR_IDLE ||
		ssr_data_ptr->lmp_ssr_state == LMP_SSR_TRIED)
	{
		if(max_sniff_subrate <= 0x1)
		{
			/* Sniff mode - SSR not set */
			LMP_LOG_INFO(LOG_LEVEL_HIGH, NOT_ENTERING_SSR_HOST_HAS_NOT_SET_SSR_OR_LATENCY_IS_LESS, 0,0);

			return BT_FW_ERROR;
		}
	}

	lmp_generate_sniff_subrating_req_pdu(ce_index);
	ssr_data_ptr->neg_in_progress = TRUE;

    return BT_FW_SUCCESS;
}
#endif /* COMPILE_SNIFF_MODE */

