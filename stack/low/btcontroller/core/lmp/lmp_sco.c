/***************************************************************************
 Copyright (C) MindTree Ltd.
 This module is a confidential and proprietary property of MindTree and
 a possession or use of this module requires written permission of MindTree.
 ***************************************************************************/
/********************************* Logger *************************/ 
enum { __FILE_NUM__= 56 };
/********************************* Logger *************************/
/** 
 * \file
 *  SCO LMP Layer implementation. All the SCO related variables and functions
 *  are defined in this file.
 * 
 * \author Santhosh kumar M
 * \date 2007-02-01
 */

/* ========================= Include File Section ========================= */
#include "lmp_internal.h"
#include "hci_vendor_internal.h"
#include "bz_debug.h"
#include "UartPrintf.h"

#ifdef SCO_OVER_HCI
#include "bz_utils.h"
#endif //#ifdef SCO_OVER_HCI

#include "bt_fw_hci_2_1.h"
#include "bzdma.h"

/* ====================== Macro Declaration Section ======================= */


/* ==================== Structure Declaration Section ===================== */
/** 
 * Defines the LMP_sco_link_req_pdu parameters.
 */
typedef struct
{
    UCHAR handle;               /**< SCO handle */
    UCHAR timing_ctrl_flags;    /**< Timing Control Flags */
    UCHAR Dsco;                 /**< SCO offset */
    UCHAR Tsco;                 /**< SCO interval */
    UCHAR pkt_type;             /**< LMP packet type (LMP_[HV1/HV2/HV3]) */
    UCHAR air_mode;             /**< LMP Air mode */
    UCHAR tid;                  /**< Transaction ID */
} LMP_SCO_PARAMS;

/* ===================== Variable Declaration Section ===================== */
#if defined(ENABLE_SCO) || defined(COMPILE_ESCO)
#ifdef SCO_OVER_HCI
/** Codec state (OVER_CODEC, OVER_NULL) */
UCHAR sync_link_codec_state = OVER_CODEC;
#endif

/** Codec Type (PCM, UDA, HCI) */
UCHAR sync_link_codec_type = NOT_APPLICABLE;
UCHAR pcm_codec_availability = TRUE;    /**< Availability of PCM codec */
UCHAR uda_codec_availability = TRUE;    /**< Availability of UDA codec */

UCHAR pcm_ex_codec_format;
UCHAR pcm_ex_codec_format_8bit;
UINT16 pcmifctrl1;
UINT16 pcmifctrl2;
UCHAR scoconv;

/*  Added by Wallice for PCM Enhancement.  2013/07/08  */
UCHAR hci_excodec_state;
UINT16 pcmifctrl3;
UINT16 pcmconvert;
/*  End Added by Wallice for PCM Enhancement.  2013/07/08  */

#endif /* ENABLE_SCO || COMPILE_ESCO */

#ifdef ENABLE_SCO

/* ================== Static Function Prototypes Section ================== */
UCHAR lmp_sco_check_against_full_bw_policy(UINT16 pkt_type,
        UINT16 ce_index);
UCHAR lmp_check_allow_new_sco_link(UINT16 type, UINT16 ce_index);

#ifndef _CCH_SLOT_OFFSET_
API_RESULT lmp_generate_dsco(UCHAR* pDsco);
#endif

#ifdef _CCH_SLOT_OFFSET_
UCHAR lmp_generate_sco_params(UINT16 sco_ce_index,
        UINT16 allowed_pkt_types, UINT16 max_latency,
        UINT16 preferred_pkt_type, UINT16 orig_sco_ce_index);
#else
UCHAR lmp_generate_sco_params(UINT16 sco_ce_index,
        UINT16 allowed_pkt_types, UINT16 max_latency,
        UINT16 preferred_pkt_type);
#endif

UCHAR lmp_validate_sco_params(UINT16 sco_ce_index,
        UINT16 allowed_pkt_types, UINT16 max_latency);
UCHAR lmp_check_sync_fifo_availability(UINT16 index);
void lmp_update_pkts_after_sco_links_change(UINT16 ce_index);
void lmp_kill_sco_connection(UINT16 ce_index, UINT16 sco_ce_index,
        UCHAR reason);

#if LMP_PARAMS_CHECK > 0
UCHAR lmp_validate_pdu_sco_params(const LMP_SCO_PARAMS* sco_params,
        UCHAR remote_dev_role);
#endif

#if (LMP_PARAMS_CHECK > 0) || (LMP_TID_CHECK > 0)
void lmp_handle_invalid_sco_link_req(UINT16 ce_index,
        UINT16 sco_ce_index, UCHAR reason, UINT16* psco_ce_index);
#endif

UCHAR lmp_handle_change_sco_conn_pkt_type(UINT16 ce_index,
        UINT16 sco_ce_index, LMP_SCO_PARAMS* sco_params);
UCHAR lmp_handle_sco_link_req_pdu_for_new_link(UINT16 ce_index,
        UINT16 sco_ce_index, LMP_SCO_PARAMS* sco_params);
UCHAR lmp_handle_sub_sco_link_req(UINT16 ce_index,
        LMP_SCO_PARAMS* sco_params, UINT16* psco_ce_index);



/* ================== For rom code patch function point ================== */
#ifdef _ROM_CODE_PATCHED_
#ifdef _CCH_8723_RCP_
PF_ROM_CODE_PATCH_FUNC rcp_lmp_generate_sco_params_rej = NULL;
#ifndef _REDUCE_LPS_AFTER_RTL8703B_
PF_ROM_CODE_PATCH_FUNC rcp_lmp_handle_remove_sco_link_request_pdu = NULL;
#endif
#endif
#endif

/* ===================== Function Definition Section ====================== */
/** 
 * Check whether the sco connection with \a pkt_type, if established, will
 * violate the bluewiz full bandwidth policy.
 * 
 * \param pkt_type Packet type chosen for the SCO link to be established.
 * \param ce_index ACL connection entity index associated with the SCO link to
 *                 be established.
 * 
 * \return HCI_COMMAND_SUCCEEDED, if it doesn't violate the bluewiz full
 *         bandwidth policy. A valid error code otherwise.
 *
 * \warning This function has to be called only if we have more than one ACL
 *          connection and establishment of this SCO link will occupy the
 *          entire bandwith. Otherwise the return value of this function is
 *          undefined.
 */
UCHAR lmp_sco_check_against_full_bw_policy(UINT16 pkt_type,
        UINT16 ce_index)
{
    UCHAR i, j, k;
    UCHAR nconns_for_ce_index = 0;
    UCHAR num_acl_conns;
    UCHAR num_sco_conns;
    UCHAR any_ce_has_2_conns = FALSE;
    UCHAR ret_error = HCI_COMMAND_SUCCEEDED;
    struct
    {
        UINT16 ce_index;
        UCHAR count;
    } array3[3] = {{0xff, 0}, {0xff, 0}, {0xff, 0}};

    for (i = 0, j = 0; i < LMP_MAX_SCO_CONNECTIONS; i++)
    {
        UINT16 l_ce_index;

        if (lmp_sco_connection_data[i].sco_conn_status &
                    (SCO_CONNECTED | SCO_CHG_PARAMS | SCO_DISCONNECTING))
        {
            l_ce_index = lmp_sco_connection_data[i].conn_entity_index;
            for (k = 0; k < j; k ++)
            {
                if (array3[k].ce_index == l_ce_index)
                {
                    array3[k].count ++;
                    break;
                }
            }
            if (k >= j)
            {
                array3[j].ce_index = l_ce_index;
                array3[j++].count ++;
            }
        } /* end if(the connection entity is associated with a connection) */
    } /* end for(each sco connection entity) */

    for (k = 0; k < j; k++)
    {
        if (array3[k].ce_index == ce_index)
        {
            nconns_for_ce_index = array3[k].count;
        }
        if (array3[k].count == 2)
        {
            any_ce_has_2_conns = TRUE;
        }
    }

    num_sco_conns = lmp_self_device_data.total_no_of_sco_conn;
    num_acl_conns = lmp_self_device_data.number_of_acl_conn;
    BZ_ASSERT(num_acl_conns > 1,
            "This logic has to be used only when num_acl_conns > 1 "
            "and full bandwidth will be occupied if the new sco link is "
            "established");

    num_sco_conns ++;
    if ( (num_acl_conns > num_sco_conns)
            || ((num_acl_conns == num_sco_conns)
                && ((nconns_for_ce_index != 0) || any_ce_has_2_conns))
            || ((num_acl_conns < num_sco_conns) /* num_sco == 3 */
                && (nconns_for_ce_index >= num_acl_conns)) )
    {
        ret_error = HOST_REJECTED_LIMITED_RESOURCES_ERROR;
    }

    return ret_error;
}

/**
 * Checks if the sco connection that is being created can be allowed or not.
 *
 * \param type SCO Packet type selected for the connection being created (HV1,
 *             HV2 or HV3 and it should be in 1.1 HCI [ADD_SCO] format).
 * \param ce_index ACL Connection entity index.
 *
 * \return HCI_COMMAND_SUCCEEDED, if the new sco link can be allowed. A valid
 *         error code otherwise.
 */
UCHAR lmp_check_allow_new_sco_link(UINT16 type, UINT16 ce_index)
{
    UCHAR ret_error = HCI_COMMAND_SUCCEEDED;
    UCHAR num_sco_conns;
    UCHAR num_acl_conns;
    UCHAR check_for_policy = FALSE;

#ifdef COMPILE_ESCO
    /* SCO and Esco are not allowed together. */
    if (lmp_self_device_data.number_of_esco_connections)
    {
        return HOST_REJECTED_LIMITED_RESOURCES_ERROR;
    }
#endif

    num_sco_conns = lmp_self_device_data.total_no_of_sco_conn;
    num_acl_conns = lmp_self_device_data.number_of_acl_conn;

    switch (type)
    {
        case HV1:
            if (num_sco_conns == 1)
            {
                ret_error = MAX_SCO_CONNECTIONS_REACHED_ERROR;
            }
            else if (num_acl_conns > 1)
            {
                ret_error = HOST_REJECTED_LIMITED_RESOURCES_ERROR;
            }

            /* Reject if the device is in scatternet. */
            if(lc_check_if_device_is_in_scatternet() == TRUE)
            {
                RT_BT_LOG(GRAY, LMP_SCO_235, 0, 0);
                /* ret_error = UNSPECIFIED_ERROR; */
                ret_error = HOST_REJECTED_LIMITED_RESOURCES_ERROR;
            }

            break;

        case HV2:
            if (num_sco_conns == 2)
            {
                ret_error = MAX_SCO_CONNECTIONS_REACHED_ERROR;
            }
            else if ((num_sco_conns == 1) && (num_acl_conns > 1))
            {
                check_for_policy = TRUE;
            }

            /* Reject if the device is in scatternet. */
            if(lc_check_if_device_is_in_scatternet() == TRUE)
            {
                RT_BT_LOG(GRAY, LMP_SCO_257, 0, 0);
                /* ret_error = UNSPECIFIED_ERROR; */
                ret_error = HOST_REJECTED_LIMITED_RESOURCES_ERROR;
            }
            break;

        case HV3:
            if (num_sco_conns == 3)
            {
                ret_error = MAX_SCO_CONNECTIONS_REACHED_ERROR;
            }
            else if ( (num_sco_conns == 2) && (num_acl_conns > 1) )
            {
                check_for_policy = TRUE;
            }

            /* Reject if the device is in scatternet and there is 
            already a SCO connection present. */
            if( (lc_check_if_device_is_in_scatternet() == TRUE) &&
                (num_sco_conns != 0) )
            {
#ifdef _DAPE_ALLOW_SYNC_LINK_FOR_WIN8    
                LMP_SCO_CONNECTION_DATA* sco_ce_ptr_other;
                UINT8 ii = 0;
                UINT8 sco_no_reject = 0;
                for (ii = 0; ii< 3; ii++)
                {
                    sco_ce_ptr_other = &lmp_sco_connection_data[ii];
                    if (sco_ce_ptr_other->sco_conn_status == SCO_DISCONNECTING)
                    {
                        sco_no_reject = 1;   
                        break;
                    }
                }
                if (!sco_no_reject)
#endif           
                {
                    RT_BT_LOG(GRAY, LMP_SCO_281, 0, 0);
                    /* ret_error = UNSPECIFIED_ERROR; */
                    ret_error = HOST_REJECTED_LIMITED_RESOURCES_ERROR;
                }
            }
            break;

        default:
            ret_error = UNSPECIFIED_ERROR;
            break;
    }

    if (check_for_policy == TRUE)
    {
        ret_error = lmp_sco_check_against_full_bw_policy(type, ce_index);
    }

    return ret_error;
}

#ifndef _CCH_SLOT_OFFSET_
/**
 * Generates a Dsco value, if possible, which doesn't violate any of the 
 * existing SCO connections.
 *
 * \param pDsco Address where the generated Dsco will be stored.
 *
 * \return API_SUCCESS, if the operation was successful. API_FAILURE,
 *         otherwise.
 */
API_RESULT lmp_generate_dsco(UCHAR* pDsco)
{
    LMP_SCO_CONNECTION_DATA* sco_ce_ptr;
    int i;
    UCHAR dsco_bitmap = 0;
    UCHAR dsco = 0;

    if (lmp_self_device_data.total_no_of_sco_conn > 0)
    {
        for (i = 0, sco_ce_ptr = &lmp_sco_connection_data[0];
                i < LMP_MAX_SCO_CONNECTIONS; i++, sco_ce_ptr++)
        {
            if (!(sco_ce_ptr->sco_conn_status  &
                    (SCO_FREE | SCO_CONNECTING | SCO_WAITING_FOR_CONN_ACCEPT
                     | SCO_ALLOCATED)))
            {
                dsco_bitmap = (UCHAR)(dsco_bitmap | (1 << sco_ce_ptr->Dsco));
            }
        }
        if ((dsco_bitmap & BIT0) == 0)
        {
            dsco = 0x00;
        }
        else if ((dsco_bitmap & BIT2) == 0)
        {
            dsco = 0x02;
        }
        else if ((dsco_bitmap & BIT4) == 0)
        {
            dsco = 0x04;
        }
    }

    *pDsco = dsco;
    return API_SUCCESS;
}
#endif
/** 
 * Generate SCO parameters and store them in the SCO connection entity, if
 * the operation was successful.
 * 
 * \param sco_ce_index SCO connection entity index.
 * \param allowed_pkt_types Allowed packet types for this connection (Should
 *                          be in 1.1 HCI [ADD_SCO] format).
 * \param max_latency Allowed max latency for this connection (Should be in
 *                    slots).
 * \param preferred_pkt_type Preferred packet type for this connection (Should
 *                           be in 1.1 HCI [ADD_SCO] format). This packet type
 *                           is given highest priority over others and it
 *                           should be 0, if no packet type is preferred.
 * 
 * \return BT_FW_SUCCESS, if SCO parameters were generated successfully. A
 *         valid error code otherwise.
 *
 * \warning This function updates the SCO connection entity with the generated
 *          parameters (sco_handle, Tsco, Dsco, pkt_type, time_control_flags),
 *          if the operation was successful.
 */
#ifdef _CCH_SLOT_OFFSET_
UCHAR lmp_generate_sco_params(UINT16 sco_ce_index,
        UINT16 allowed_pkt_types, UINT16 max_latency,
        UINT16 preferred_pkt_type, UINT16 orig_sco_ce_index)
#else
UCHAR lmp_generate_sco_params(UINT16 sco_ce_index,
        UINT16 allowed_pkt_types, UINT16 max_latency,
        UINT16 preferred_pkt_type)
#endif
 
{
    UINT16 sco_1_1_pkt_type;
    UINT16 present_1_1_pkt_type;
    LMP_SCO_CONNECTION_DATA *sco_ce_ptr;
    UINT16 ce_index;
    UCHAR lmp_pkt_type;
    UCHAR tsco;
    UCHAR dsco;
    UCHAR timing_ctrl_flags;
    UCHAR sco_handle;
    UCHAR reason;

    LMP_CONNECTION_ENTITY *ce_ptr;

    sco_ce_ptr = &lmp_sco_connection_data[sco_ce_index];
    ce_index = sco_ce_ptr->conn_entity_index;

    ce_ptr = &lmp_connection_entity[ce_index];

    max_latency = (UINT16)(max_latency - 2);

    present_1_1_pkt_type = preferred_pkt_type;
    if (lmp_self_device_data.total_no_of_sco_conn > 0)
    {
        present_1_1_pkt_type = lmp_self_device_data.sco_pkt_type;
    }

    /* Only single SCO connection is allowed if the device is already in scatternet. */
    if( (lc_check_if_device_is_in_scatternet() == TRUE) && 
        (lmp_self_device_data.total_no_of_sco_conn != 0) )
    {
#ifdef _DAPE_ALLOW_SYNC_LINK_FOR_WIN8    
        LMP_SCO_CONNECTION_DATA* sco_ce_ptr_other;
        UINT8 ii = 0;
        UINT8 sco_no_reject = 0;
        for (ii = 0; ii< 3; ii++)
        {
            sco_ce_ptr_other = &lmp_sco_connection_data[ii];        
            if (sco_ce_ptr_other->sco_conn_status == SCO_DISCONNECTING)
            {
                sco_no_reject = 1;   
                break;
            }
        }
        if (!sco_no_reject)
#endif
        {
            RT_BT_LOG(GRAY, LMP_SCO_438, 0, 0);
            /* return UNSPECIFIED_ERROR; */
            return HOST_REJECTED_LIMITED_RESOURCES_ERROR;
        }
    }

#ifdef SCO_REFACTORING_DEBUG
    LMP_LOG_INFO(LOG_LEVEL_LOW, LMP_SELF_DEVICE_DATA_NUMBER_OF_SCO_CONN, 
            1,   lmp_self_device_data.total_no_of_sco_conn);
    LMP_LOG_INFO(LOG_LEVEL_LOW, LMP_SELF_DEVICE_DATA_SCO_PKT_TYPE, 1, 
            lmp_self_device_data.sco_pkt_type);

    if (lmp_self_device_data.total_no_of_sco_conn > 0)
    {
        switch (lmp_self_device_data.sco_pkt_type)
        {
            case HV1:
            case HV2:
            case HV3:
                break;
            default:
                BZ_ASSERT(0, "I doubted lmp_self_device_data.sco_pkt_type");
        }
    }
#endif /* SCO_REFACTORING_DEBUG */

    do
    {
        lmp_pkt_type = 0xFF;
        reason = BT_FW_SUCCESS; /* Assume, we can find a viable pkt type */
        if (preferred_pkt_type & allowed_pkt_types & present_1_1_pkt_type)
        {
            lmp_pkt_type = SCO_PKT_TYPE_1_1_HCI_TO_LMP(preferred_pkt_type);
        }
        else
        {
            switch (lmp_self_device_data.total_no_of_sco_conn)
            {
            case 0:
                lmp_pkt_type = SCO_PKT_TYPE_1_1_HCI_TO_LMP(allowed_pkt_types);
                if (lmp_pkt_type > LMP_HV3)
                {
                    lmp_pkt_type = LMP_HV3;
                }
                break;

            case 1:
            case 2:     /* Fall through */
                if (present_1_1_pkt_type & allowed_pkt_types)
                {
                    lmp_pkt_type =
                        SCO_PKT_TYPE_1_1_HCI_TO_LMP(present_1_1_pkt_type);
                }
                break;

            default:
                BZ_ASSERT(0, "lmp_self_device_data.total_no_of_sco_conn is"
                            "invalid");
            } /* end switch(lmp_self_device_data.sco_pkt_type) */
        } /* end if(preferred_pkt_type is allowed) */

        if (lmp_pkt_type == 0xFF)
        {
            /* We couldn't even select a packet type, so there is no way we
             * can continue the process with other allowed packet types.
             */
            return SCO_INTERVAL_REJECTED_ERROR;
        }

        tsco = (UCHAR)(2 + (lmp_pkt_type<<1));
        sco_1_1_pkt_type = SCO_PKT_TYPE_LMP_TO_1_1_HCI(lmp_pkt_type);
        if (tsco > max_latency)
        {
            reason = SCO_INTERVAL_REJECTED_ERROR;
        }
        else
        {
            reason = lmp_check_allow_new_sco_link(sco_1_1_pkt_type, ce_index);
            if (reason == HCI_COMMAND_SUCCEEDED)
            {
                reason = BT_FW_SUCCESS;
            }
        }

        /* Remove the selected packet type from the list of allowed packet
         * types.
         */
        allowed_pkt_types = (UINT16)(allowed_pkt_types & (~sco_1_1_pkt_type));
    }
    while (reason != BT_FW_SUCCESS && allowed_pkt_types != 0
        && lmp_self_device_data.total_no_of_sco_conn == 0);

    if (reason != BT_FW_SUCCESS)
    {
        return reason;
    }


#ifdef _CCH_SLOT_OFFSET_
    UINT16 temp_slot_num;
    UINT16 temp_sco_ce_index;
    temp_slot_num = 2;

    temp_sco_ce_index = sco_ce_index;
    if(sco_ce_index == LMP_MAX_SCO_CONNECTIONS)
    {
        temp_sco_ce_index = orig_sco_ce_index;
    }	
    dsco = lmp_get_global_slot_offset(ce_index, temp_sco_ce_index + 2*LMP_MAX_CE_DATABASE_ENTRIES , tsco, temp_slot_num, 0, 0);

#ifdef _ROM_CODE_PATCHED_
#ifdef _CCH_8723_RCP_
// RCP for not reject SCO paramter
// Paramter: Actually only need dsco
// Change Paramter: dsco

    if (rcp_lmp_generate_sco_params_rej != NULL)
    {
        rcp_lmp_generate_sco_params_rej((void *)&dsco);
    }
#endif
#endif

    UCHAR lmp_get_global_slot_offset_fail;
    lmp_get_global_slot_offset_fail = 1;

    if ((dsco) < GLOBAL_SLOT_INTERVAL)
    {
        lmp_get_global_slot_offset_fail = 0;
    }
    if (lmp_get_global_slot_offset_fail)
    {
        lmp_put_global_slot_offset(ce_index, temp_sco_ce_index + 2*LMP_MAX_CE_DATABASE_ENTRIES);
        return SCO_OFFSET_REJECTED_ERROR;
    }
#endif

    if (ce_ptr->remote_dev_role == SLAVE)
    {	
        UINT32 clock_value;

#ifndef _CCH_SLOT_OFFSET_
        if (lmp_generate_dsco(&dsco) != API_SUCCESS)
        {
            return SCO_OFFSET_REJECTED_ERROR;
        }
#endif


        lc_get_clock_in_scatternet(&clock_value, ce_ptr->phy_piconet_id);

        if (clock_value & BIT27)
        {
            timing_ctrl_flags = BIT1;
        }
        else
        {
            timing_ctrl_flags = 0;
        }
        sco_handle = (UCHAR)(sco_ce_index+1);
    }
    else
    {

#ifdef _CCH_SLOT_OFFSET_
        if(sco_ce_index == LMP_MAX_SCO_CONNECTIONS)
        {
            dsco = 0;
        }
#else	
        dsco = 0;
#endif
        timing_ctrl_flags = 0;
        sco_handle = 0;
    }



    /* Populate the SCO connection entity with the generated params */
    sco_ce_ptr->sco_handle = sco_handle;
    sco_ce_ptr->time_control_flags = timing_ctrl_flags;
    sco_ce_ptr->Tsco = tsco;
    sco_ce_ptr->Dsco = dsco;
    sco_ce_ptr->pkt_type = lmp_pkt_type;

    return BT_FW_SUCCESS;
}

/** 
 * Validates the SCO parameters given by \a sco_ce_index against the \a
 * allowed_pkt_types, \a max_latency and present conditions.
 * 
 * \param sco_ce_index SCO connection entity index.
 * \param allowed_pkt_types Allowed packet types (Should be in 1.1 HCI
 *                          [ADD_SCO] format).
 * \param max_latency Allowed max latency (Should be in slots).
 * 
 * \return HCI_COMMAND_SUCCEEDED, if the given parameters meet the necessary
 *         conditions. A valid error code, otherwise.
 */
UCHAR lmp_validate_sco_params(UINT16 sco_ce_index,
        UINT16 allowed_pkt_types, UINT16 max_latency)
{
    UCHAR ret_error;
    UINT16 sco_1_1_pkt_type;
    UINT16 temp_1_1_pkt_type;
    LMP_SCO_CONNECTION_DATA* sco_ce_ptr;

    sco_ce_ptr = &lmp_sco_connection_data[sco_ce_index];

    sco_1_1_pkt_type = SCO_PKT_TYPE_LMP_TO_1_1_HCI(sco_ce_ptr->pkt_type);
    temp_1_1_pkt_type = sco_1_1_pkt_type;
    if (lmp_self_device_data.total_no_of_sco_conn > 0)
    {
        temp_1_1_pkt_type = lmp_self_device_data.sco_pkt_type;
    }

    if (!(sco_1_1_pkt_type & allowed_pkt_types & temp_1_1_pkt_type)
            || (sco_ce_ptr->Tsco > (max_latency-2)))
    {
        return SCO_INTERVAL_REJECTED_ERROR;
    }

    ret_error = lmp_check_allow_new_sco_link(sco_1_1_pkt_type,
                                             sco_ce_ptr->conn_entity_index);
    if (ret_error != HCI_COMMAND_SUCCEEDED)
    {
        return ret_error;
    }

    return BT_FW_SUCCESS;
}

/** 
 * Generate SCO parameters for changing the packet type of the SCO connection
 * given by \a sco_ce_index and store them in a Temporary SCO connection
 * entity, if the operation was successful.
 * 
 * \param sco_ce_index SCO connection entity index for which packet type is
 *                     being changed.
 * \param allowed_pkt_types Allowed packet types for this new negotiation
 *                          (Should be in 1.1 HCI [ADD_SCO] format).
 * \param max_latency Allowed max latency for this new negotiation (Should be
 *                    in slots).
 * \param preferred_pkt_type Preferred packet type for this negotiation (Should
 *                           be in 1.1 HCI [ADD_SCO] format). This packet type
 *                           is given highest priority over others and it
 *                           should be 0, if no packet type is preferred.
 * 
 * \return BT_FW_SUCCESS, if SCO parameters were generated successfully. A
 *         valid error code otherwise.
 *
 * \warning This function allocates the temporary SCO connecton entity and
 *          updates it with the generated parameters (sco_handle, Tsco, Dsco,
 *          pkt_type, time_control_flags), if the operation was successful.
 *          The new allocated temporary SCO connection entity's index is
 *          #LMP_MAX_SCO_CONNECTIONS. This index has to be used for referring
 *          the newly generated parameters for change connection packet type.
 *          If the new negotiation is successful then these parameters have to
 *          be copied to the original \a sco_ce_index and the temporary entity
 *          has to be freed. In case the negotiation fails, the temporary
 *          entity has to be freed and \a sco_ce_index's status has to be
 *          updated to SCO_CONNECTED. The role of the temporary entity is to
 *          avoid overwriting the existing parameters during the negotiation.
 */
UCHAR lmp_generate_sco_params_for_change_pkt_type(UINT16 sco_ce_index,
        UINT16 allowed_pkt_types, UINT16 max_latency,
        UINT16 preferred_pkt_type)
{
    LMP_SCO_CONNECTION_DATA* sco_ce_ptr;
    LMP_SCO_CONNECTION_DATA* temp_sco_ce_ptr;
    UCHAR reason;
    UINT16 ce_index;

    sco_ce_ptr = &lmp_sco_connection_data[sco_ce_index];
    temp_sco_ce_ptr = &lmp_sco_connection_data[LMP_MAX_SCO_CONNECTIONS];
    ce_index = sco_ce_ptr->conn_entity_index;

    if (lmp_allocate_temp_sco_conn_entity(ce_index) == FALSE)
    {
        BZ_ASSERT(0, "I would have forgot to restore the status(?)");
        /* The temporary SCO connection entity is being used by another
         * parallel SCO change connection packet type transaction (may be
         * with different slave or piconet). So it is not possible for us to
         * proceed (It is a limitation of our implementation).
         */
        return HOST_REJECTED_LIMITED_RESOURCES_ERROR;
    }

    /* Copy necessary fields from source SCO conn entity to temp entity */
    temp_sco_ce_ptr->conn_entity_index = ce_index;
    temp_sco_ce_ptr->air_mode = sco_ce_ptr->air_mode;

    /* Host given params for Change SCO conn pkt_type */
    temp_sco_ce_ptr->host_allowed_packets = allowed_pkt_types;
    temp_sco_ce_ptr->max_latency = max_latency;

    /* Changing SCO pkt type is equivalent to creating new SCO */
    lmp_self_device_data.total_no_of_sco_conn--;
    if(lmp_connection_entity[ce_index].no_of_sco_connections != 0)
    {
        lmp_connection_entity[ce_index].no_of_sco_connections--;
    }

#ifdef _CCH_SLOT_OFFSET_
    reason = lmp_generate_sco_params(LMP_MAX_SCO_CONNECTIONS,
        allowed_pkt_types, max_latency, preferred_pkt_type, sco_ce_index);
#else
    reason = lmp_generate_sco_params(LMP_MAX_SCO_CONNECTIONS,
        allowed_pkt_types, max_latency, preferred_pkt_type);
#endif

	
    /* Restore temp changes */
    lmp_self_device_data.total_no_of_sco_conn++;
    lmp_connection_entity[ce_index].no_of_sco_connections++;

    temp_sco_ce_ptr->sco_handle = sco_ce_ptr->sco_handle;
    if (reason != BT_FW_SUCCESS)
    {
        lmp_free_temp_sco_conn_entity();
    }

    return reason;
}

/** 
 * Initiates a new SCO connection with the given parameters.
 * 
 * \param ce_index ACL connection entity index.
 * \param max_latency Allowed maximum latency (Shoud be in slots)
 * \param allowed_pkt_types Allowed packet types (Should be in 1.1 HCI
 *                          [ADD_SCO] format).
 * \param hci_air_mode Allowed air mode for this connection (Should be in HCI
 *                     format not LMP PDU format).
 * \param gen_conn_complete Indicates whether to generate
 *                          Conn_Complete_Evt(TRUE) or
 *                          Sync_Conn_Complete_evt(FALSE).
 * 
 * \return HCI_COMMAND_SUCCEEDED, if the operation was successful. A valid
 *         error code, otherwise.
 */
UCHAR lmp_initiate_new_sco_connection(UINT16 ce_index, UINT16 max_latency,
        UINT16 allowed_pkt_types, UCHAR hci_air_mode, UCHAR gen_conn_complete)
{
    UINT16 sco_ce_index;
    UCHAR ret_error;
    LMP_SCO_CONNECTION_DATA* sco_ce_ptr;
    LMP_CONNECTION_ENTITY* ce_ptr;

    ce_ptr = &lmp_connection_entity[ce_index];

    ret_error = lmp_allocate_sco_conn_entity(&sco_ce_index, ce_index);
    if (ret_error != BT_FW_SUCCESS)
    {
        return ret_error;
    }

    sco_ce_ptr = &lmp_sco_connection_data[sco_ce_index];
    sco_ce_ptr->gen_conn_complete = gen_conn_complete;
    sco_ce_ptr->host_allowed_packets = allowed_pkt_types;
    sco_ce_ptr->max_latency = max_latency;
    sco_ce_ptr->air_mode =
        lmp_convert_air_mode(hci_air_mode, HCI_LAYER, LMP_LAYER);

#ifdef _CCH_SLOT_OFFSET_
    ret_error = lmp_generate_sco_params(sco_ce_index, allowed_pkt_types,
            max_latency, 0, 0);
#else
    ret_error = lmp_generate_sco_params(sco_ce_index, allowed_pkt_types,
            max_latency, 0);
#endif


    if (ret_error != BT_FW_SUCCESS)
    {
        lmp_free_sco_conn_entity(sco_ce_index);
        return ret_error;
    }

    if(ce_ptr->remote_dev_role == MASTER)
    {
        ce_ptr->lmp_expected_pdu_opcode |= lmp_get_opcode_mask(
                LMP_SCO_LINK_REQ_OPCODE, 0x0);
    }

    sco_ce_ptr->sco_conn_status = SCO_CONNECTING;
    lmp_sco_connect_update_ce_status(ce_index);
    lmp_send_sco_link_request_pdu(ce_index, sco_ce_index, SELF_DEV_TID);
    lmp_sco_set_trans_status(sco_ce_ptr, TRS_SELF_INITIATED);

    return HCI_COMMAND_SUCCEEDED;
}

/** 
 * Handles a new SCO connection request from the remote device. This function
 * doesn't directly handle the LMP_sco_link_req from the remote device.
 * Instead, the handler for the LMP_sco_link_req would allocate a SCO connection
 * entity and store the remote suggested parameters in it. Then, it generates
 * Connection_Request_Event to the host and the Host sends
 * HCI_Accept_[Sync]_Conn_req_Command with acceptable parameters. The handler
 * for that Command calls this function.
 * 
 * \param sco_ce_index SCO connection entity (index) which has the remote
 *                     device given parameters.
 * \param max_latency Allowed maximum latency (Shoud be in slots)
 * \param allowed_pkt_types Allowed packet types (Should be in 1.1 HCI
 *                          [ADD_SCO] format).
 * \param hci_air_mode Allowed air mode for this connection (Should be in HCI
 *                     format not LMP PDU format).
 * \param gen_conn_complete Indicates whether to generate
 *                          Conn_Complete_Evt(TRUE) or
 *                          Sync_Conn_Complete_evt(FALSE).
 * 
 * \return BT_FW_SUCCESS, if the operation was successful. A valid
 *         error code, otherwise.
 */
UCHAR lmp_handle_new_sco_conn_req(UINT16 sco_ce_index, UINT16 max_latency,
        UINT16 allowed_pkt_types, UCHAR hci_air_mode, UCHAR gen_conn_complete)
{
    UCHAR reason;
    UCHAR lmp_air_mode;
    UINT16 preferred_pkt_type;
    UINT16 ce_index;
    LMP_SCO_CONNECTION_DATA* sco_ce_ptr;
    LMP_CONNECTION_ENTITY* ce_ptr;

    sco_ce_ptr = &lmp_sco_connection_data[sco_ce_index];
    ce_index = sco_ce_ptr->conn_entity_index;
    ce_ptr = &lmp_connection_entity[ce_index];

    sco_ce_ptr->gen_conn_complete = gen_conn_complete;
    sco_ce_ptr->host_allowed_packets = allowed_pkt_types;
    sco_ce_ptr->max_latency = max_latency;
    lmp_air_mode = lmp_convert_air_mode(hci_air_mode, HCI_LAYER, LMP_LAYER);

    if (sco_ce_ptr->air_mode != lmp_air_mode)
    {
        return SCO_AIR_MODE_REJECTED_ERROR;
    }

    if (ce_ptr->remote_dev_role == SLAVE)
    {
        preferred_pkt_type = SCO_PKT_TYPE_LMP_TO_1_1_HCI(sco_ce_ptr->pkt_type);

#ifdef _CCH_SLOT_OFFSET_		
        reason = lmp_generate_sco_params(sco_ce_index, allowed_pkt_types,
                max_latency, preferred_pkt_type, 0);
#else
        reason = lmp_generate_sco_params(sco_ce_index, allowed_pkt_types,
                max_latency, preferred_pkt_type);
#endif



        if (reason != BT_FW_SUCCESS)
        {
            return reason;
        }
        lmp_send_sco_link_request_pdu(ce_index, sco_ce_index, REMOTE_DEV_TID);
    }
    else
    {
        reason = lmp_validate_sco_params(sco_ce_index, allowed_pkt_types,
                max_latency);
        if (reason != BT_FW_SUCCESS)
        {
            return reason;
        }
        lmp_send_lmp_accepted_dm1(ce_index, LMP_SCO_LINK_REQ_OPCODE,
                REMOTE_DEV_TID, LMP_NO_STATE_CHANGE);
    }
    sco_ce_ptr->sco_conn_status = SCO_CONNECTING;

    return BT_FW_SUCCESS;
}

/** 
 * Gets the \a sco_ce_index corresponding to the \a conn_handle.
 * 
 * \param conn_handle Connection handle of the SCO link.
 * \param sco_ce_index Index to the SCO connection database.
 * 
 * \return API_SUCCESS, if the operation is successful. API_FAILURE,
 *         otherwise.
 */
API_RESULT lmp_get_sco_ce_index_from_conn_handle(const UINT16 conn_handle,
                                                 UINT16* sco_ce_index)
{
    UCHAR index;
    UCHAR acl_conn_handles = LMP_MAX_CE_DATABASE_ENTRIES +
                             LMP_MAX_BC_CONNECTIONS + 1;

    if (conn_handle == 0 || conn_handle > LMP_MAX_CONN_HANDLES ||
        conn_handle < acl_conn_handles)
    {
        return API_FAILURE;
    }

    index = (UCHAR)(conn_handle - acl_conn_handles);

    if(index >= LMP_MAX_SCO_CONNECTIONS
            || lmp_sco_connection_data[index].sco_conn_status == SCO_FREE)
    {
        return API_FAILURE;
    }
    *sco_ce_index = index;

    return API_SUCCESS;
}

/** 
 * Gets the index of SCO connection which is in \a status state and
 * corresponding to the \a ce_index (ACL connection index).
 * 
 * \param ce_index ACL connection entity index.
 * \param status Status of the SCO connection (associated with the ACL).
 * \param sco_ce_index Index to the SCO connection database.
 * 
 * \return API_SUCCESS, if the operation if successful. API_FAILURE,
 *         otherwise.
 *
 * \warning If two or more SCO connections associated with the same ACL
 *          connection (\a ce_index) are in the same state (\a status), any
 *          one of the SCO connection index is returned and no particular
 *          order should be assumed.
 */
API_RESULT lmp_get_sco_ce_index_from_ce_index(const UINT16 ce_index,
        const LMP_SCO_STATE status, UINT16* sco_ce_index)
{
    UINT16 i;
    API_RESULT result = API_FAILURE;

    for (i = 0; i < LMP_MAX_SCO_CONNECTIONS; i++)
    {
#if 0
        RT_BT_LOG(GRAY, LMP_SCO_906, 3,
                    i,
                    lmp_sco_connection_data[i].conn_entity_index,
                    lmp_sco_connection_data[i].sco_conn_status);
#endif
        if (lmp_sco_connection_data[i].conn_entity_index == ce_index
                && (lmp_sco_connection_data[i].sco_conn_status & status))
        {
            *sco_ce_index = i;
            result = API_SUCCESS;
            break;
        }
    }

    RT_BT_LOG(GRAY, LMP_SCO_902, 4, ce_index, status, result, *sco_ce_index);

    return result;
}

/** 
 * Gets the index of the SCO connection associated with the \a sco_handle and
 * \a ce_index. Here, ce_index is required because, sco_handle is unique only
 * within the piconet.
 * 
 * \param sco_handle SCO handle provided by the Master for this SCO link.
 * \param ce_index ACL connection entity index.
 * \param sco_ce_index SCO connection entity index.
 * 
 * \return API_SUCCESS, if the operation is successful. API_FAILURE,
 *         otherwise.
 */
API_RESULT lmp_get_sco_ce_index_from_sco_handle(const UCHAR sco_handle,
        const UINT16 ce_index, UINT16* sco_ce_index)
{
    UINT16 i;
    LMP_SCO_CONNECTION_DATA* sco_ce_ptr;
    API_RESULT result = API_FAILURE;

    for (i = 0; i < LMP_MAX_SCO_CONNECTIONS; i++)
    {
        sco_ce_ptr = &lmp_sco_connection_data[i];

#if 0
        RT_BT_LOG(GRAY, LMP_SCO_949, 3,
                i, sco_ce_ptr->sco_handle, sco_ce_ptr->conn_entity_index);
#endif

        if (sco_ce_ptr->sco_handle == sco_handle
                && sco_ce_ptr->conn_entity_index == ce_index
                && sco_ce_ptr->sco_conn_status != SCO_FREE)
        {
            *sco_ce_index = i;
            result = API_SUCCESS;
            break;
        }
    }

    RT_BT_LOG(GRAY, LMP_SCO_942, 4, sco_handle, ce_index, result, *sco_ce_index);
    
    return result;
}

/** 
 * Checks whether any of the FIFO is available for the new SCO connection which
 * might get established. It also checks the availabilty of the codecs.
 * 
 * \param index The index of the lmp_sco_connection_data that corresponds to
 *              the new SCO connection.
 * 
 * \return API_SUCCESS on successful allocation of a FIFO to the new
 *         connection. It also updates the lmp_sco_connection_data[index]
 *         with the allocations made.
 *         API_FAILURE when FIFO can not be allocated.
 *
 * \warning It also consumes the resources (FIFO, CODEC) allocated. So, it has
 *          to be freed(#lmp_free_sco_conn_entity) after their usage.
 */
UCHAR lmp_check_sync_fifo_availability(UINT16 index)
{
    LMP_SCO_CONNECTION_DATA* sco_ce_ptr;

    sco_ce_ptr = &lmp_sco_connection_data[index];

#ifdef SCO_OVER_HCI
    if(sync_link_codec_state == OVER_CODEC) /* does current SCO connection
                                               process is over codec? */
    {
#endif /* SCO_OVER_HCI */
        switch(sync_link_codec_type)        /* PCM or UDA? */
        {
            case PCM_CODEC:
                if (pcm_codec_availability == FALSE)
                {
                    return API_FAILURE;
                }
                break;

            case UDA_CODEC:
                if (uda_codec_availability == FALSE)
                {
                    return API_FAILURE;
                }
                break;

            default:
                return API_FAILURE;
        } /* Switch(sync_link_codec_type) */

        if (sync_link_codec_type == PCM_CODEC)
        {
            pcm_codec_availability = FALSE;
        }
        else
        {
            uda_codec_availability = FALSE;
        }
        sco_ce_ptr->codec_type = sync_link_codec_type;
        sco_ce_ptr->codec_state = OVER_CODEC;
#ifdef SCO_OVER_HCI
    }
    else /* sync_link_codec_state != OVER_CODEC */
    {
        sco_ce_ptr->codec_state = OVER_NULL;
        sco_ce_ptr->codec_type = NOT_APPLICABLE;
    } /* end of if (sync_link_codec_state == OVER_CODEC) */
#endif /* SCO_OVER_HCI */

    RT_BT_LOG(RED, LMP_SCO_1063, 3,
        sco_ce_ptr->codec_state, sco_ce_ptr->codec_type, sco_ce_ptr->fifo_num);

    return API_SUCCESS;
}

/** 
 * Allocates a free SCO connection entity if available.
 * 
 * \param sco_ce_index Allocated SCO connection entity index.
 * \param ce_index ACL connection entity index with which the allocated SCO
 *                 connection entity will be associated.
 * 
 * \return BT_FW_SUCCESS, if the operation if successful. A valid error code,
 *         otherwise.
 *
 * \warning The FIFO and CODEC for the new entity is also allocated and stored
 *          in the allocated entity and the allocated FIFO and CODEC are also
 *          consumed (#lmp_check_sync_fifo_availability). So every allocated
 *          entity has to be freed with #lmp_free_sco_conn_entity.
 */
UCHAR lmp_allocate_sco_conn_entity(UINT16* sco_ce_index, UINT16 ce_index)
{
    UINT16 i;

    for (i = 0; i < LMP_MAX_SCO_CONNECTIONS; i++)
    {
        if (lmp_sco_connection_data[i].sco_conn_status == SCO_FREE)
        {
            RT_BT_LOG(RED, LMP_SCO_1092, 1, i);

            if (lmp_check_sync_fifo_availability(i) == API_FAILURE)
            {
                return HOST_REJECTED_LIMITED_RESOURCES_ERROR;
            }
            lmp_sco_connection_data[i].sco_conn_status = SCO_ALLOCATED;
            lmp_sco_connection_data[i].conn_entity_index = ce_index;
#ifdef SCO_OVER_HCI
            lmp_sco_connection_data[i].erroneous_data_reporting =
                lmp_self_device_data.default_erroneous_data_reporting;
#endif
            *sco_ce_index = i;

            lmp_connection_entity[ce_index].is_sco_channel = TRUE;
            lmp_connection_entity[ce_index].sco_ce_idx = i;

            return BT_FW_SUCCESS;
        }
    }

#if LMP_MAX_SCO_CONNECTIONS == 1
    return HOST_REJECTED_LIMITED_RESOURCES_ERROR;
#else
    return MAX_SCO_CONNECTIONS_REACHED_ERROR;
#endif
}

/** 
 * Frees the SCO connection entity associated with the \a sco_ce_index.
 * 
 * \param sco_ce_index Index of the SCO connection entity to be freed.
 *
 * \return None.
 */
void lmp_free_sco_conn_entity(const UINT16 sco_ce_index)
{
    lmp_sco_connection_data_cleanup_after_detach(sco_ce_index);
}

/** 
 * Updates the \a ce_status of the ACL connection given by \a ce_index to
 * LMP_ADDING_SCO_LINK or LMP_ADDING_SCO_LINK_DURING_SNIFF. It has to be
 * called when the device is in SCO connection setup phase or SCO connection
 * packet type re-negotiation case.
 * 
 * \param ce_index ACL Connection entity index.
 *
 * \return None.
 */
void lmp_sco_connect_update_ce_status(const UINT16 ce_index)
{
    LMP_CONNECTION_ENTITY* ce_ptr;

    ce_ptr = &lmp_connection_entity[ce_index];

    switch (ce_ptr->ce_status)
    {
    case LMP_CONNECTED:
        lmp_set_ce_status(ce_index, LMP_ADDING_SCO_LINK);
        break;        
#ifdef COMPILE_SNIFF_MODE
    case LMP_SNIFF_MODE:
        lmp_set_ce_status(ce_index, LMP_ADDING_SCO_LINK_DURING_SNIFF);
        break;
    case LMP_ADDING_SCO_LINK_DURING_SNIFF:
#endif
    case LMP_ADDING_SCO_LINK:
      return;

    default:
        break;
    }

    lmp_self_device_data.adding_new_sco_conn++;

    return;
}

/** 
 * Restores the \a ce_status of the ACL connection given by \a ce_index to
 * LMP_CONNECTED or LMP_SNIFF_MODE. This function has to be called when SCO
 * link estabilish/re-negotiation procedure completed.
 * 
 * \param ce_index ACL connection entity index.
 *
 * \return None.
 */
void lmp_sco_connect_restore_ce_status(const UINT16 ce_index)
{
    LMP_CONNECTION_ENTITY* ce_ptr;

    ce_ptr = &lmp_connection_entity[ce_index];

    switch(ce_ptr->ce_status)
    {
    case LMP_ADDING_SCO_LINK:
        lmp_set_ce_status(ce_index, LMP_CONNECTED);
        break;

#ifdef COMPILE_SNIFF_MODE
    case LMP_ADDING_SCO_LINK_DURING_SNIFF:
        lmp_set_ce_status(ce_index, LMP_SNIFF_MODE);
        break;
#endif
    default:
        return;
    }

    if(lmp_self_device_data.adding_new_sco_conn)
    {
        lmp_self_device_data.adding_new_sco_conn--;
    }

    return;
}

/** 
 * Updates the packet types allowed for the ACL connection given by \a
 * ce_index and sends LMP_maxslot pdu to remote device accordingly. This
 * function has to be called whenever there is a change in SCO connections.
 * 
 * \param ce_index ACL Connection entity index.
 *
 * \return None.
 */
void lmp_update_pkts_after_sco_links_change(UINT16 ce_index)
{
    LMP_CONNECTION_ENTITY* ce_ptr;

    ce_ptr = &lmp_connection_entity[ce_index];

#ifndef COMPILE_SINGLE_SLOT_PACKETS_ONLY
    lmp_send_max_slot_pdu_to_all_devices();

    /* Force last actd max slot to 1, No negotiation is required here. */
    ce_ptr->last_accepted_max_slot = LMP_LAST_ACCEPTED_MAX_SLOT_1;
#endif

#ifndef COMPILE_SINGLE_SLOT_PACKETS_ONLY
    if (lmp_self_device_data.total_no_of_sco_conn == 0)
    {
        ce_ptr->last_accepted_max_slot = LMP_LAST_ACCEPTED_MAX_SLOT_INVALID;
    }
#endif

    lc_update_pkts_allowed(ce_index);

    return;
}

//#ifdef SCO_DETACH_WORKAROUND_REMIND
/** 
 * Checks whether we can detach our sco link or not.
 * 
 * \param ce_index ACL connection entity index (SCO link to be detached is
 *                 associated with this ACL).
 * 
 * \return TRUE, if any SCO link can be removed. FALSE, otherwise.
 *
 * \note This function is introduced as a workaround for our existing
 *       implementation. Host initiated detach can simply be rejected, if this
 *       function returns FALSE. Incase of remote initiated detach, we have to
 *       kill the link then and there, if this function returns FALSE, instead
 *       of waiting for BB_ACK of LMP_accepted for LMP_remove_sco_link_req.
 *       Basically, this fn tries to avoid parallel SCO disconnect
 *       transactions though it is rare.
 */
UCHAR lmp_is_remove_sco_link_possible(UINT16 ce_index)
{
    int i;

    for (i = 0; i < LMP_MAX_SCO_CONNECTIONS; i++)
    {
        if (lmp_sco_connection_data[i].conn_entity_index == ce_index
                && lmp_sco_connection_data[i].sco_conn_status
                == SCO_DISCONNECTING)
        {
            return FALSE;
        }
    }

    return TRUE;
}
//#endif /* SCO_DETACH_WORKAROUND_REMIND */

/** 
 * Kills the LMP level SCO connection, generates event and frees all the data
 * structures.
 * 
 * \param ce_index ACL connection entity index.
 * \param sco_ce_index SCO Connection entity index.
 * \param reason Reason for disconnection.
 * 
 * \return None.
 */
void lmp_kill_sco_connection(UINT16 ce_index, UINT16 sco_ce_index,
        UCHAR reason)
{
    LMP_SCO_CONNECTION_DATA* sco_ce_ptr;

    RT_BT_LOG(GREEN, LMP_SCO_1289, 0, 0);

    sco_ce_ptr = &lmp_sco_connection_data[sco_ce_index];

    hci_generate_disconnection_complete_event(HCI_COMMAND_SUCCEEDED,
        sco_ce_ptr->sco_conn_handle, reason);
    
    lc_kill_sco_connection(sco_ce_index);

#if 0
#ifdef _CCH_WHQL_SCO_
    if(lmp_self_device_data.total_no_of_sco_conn !=0 )
    {
        lmp_self_device_data.total_no_of_sco_conn--;
    }	
#else
    lmp_self_device_data.total_no_of_sco_conn--;
#endif

    if(lmp_connection_entity[ce_index].no_of_sco_connections != 0)
    {
        lmp_connection_entity[ce_index].no_of_sco_connections--;
    }
#endif

    lmp_free_sco_conn_entity(sco_ce_index);

#ifdef COMPILE_SNIFF_MODE
    lmp_determine_full_bandwidth();
#endif

    lmp_update_pkts_after_sco_links_change(ce_index);

#ifndef COMPILE_SINGLE_SLOT_PACKETS_ONLY
    lmp_decide_and_update_max_slot_req(ce_index);
    /* Restore max-slot for all the connections. */
    lmp_send_max_slot_req_pdu_to_all_devices();
#endif
    lc_check_and_enable_scans_in_scatternet();
    return;
}


/** 
 * Handles the BB_ACK of LMP_remove_sco_link_req pdu.
 * 
 * \param ce_index ACL connection entity index.
 * 
 * \return BT_FW_SUCCESS, if the operation is successful. BT_FW_ERROR,
 *         otherwise.
 */
UCHAR lmp_handle_remove_sco_link_req_ack_recd(UINT16 ce_index)
{
    UINT16 sco_ce_index;

    RT_BT_LOG(GREEN, LMP_SCO_1337, 0, 0);

    if (lmp_get_sco_ce_index_from_ce_index(ce_index, SCO_DISCONNECTING,
                &sco_ce_index) != API_SUCCESS)
    {
        BZ_ASSERT(0, "I hate obscure error handling!!");
    }
    else
    {
#if 1
        lmp_kill_sco_connection(ce_index, sco_ce_index,
            CONNECTION_TERMINATED_LOCAL_HOST_ERROR);
#endif
    }

    return BT_FW_SUCCESS;
}

/** 
 * Handles the LMP_remove_sco_link_req pdu from the remote device.
 * 
 * \param lmp_pdu_ptr Pointer to LMP PDU.
 * \param ce_index ACL connection entity index.
 * 
 * \return BT_FW_SUCCESS, if the operation is successful. BT_FW_ERROR,
 *         otherwise.
 */
UCHAR lmp_handle_remove_sco_link_request_pdu(LMP_PDU_PKT *lmp_pdu_ptr,
        UINT16 ce_index)
{
    UCHAR sco_handle;
    UINT16 sco_ce_index;
#ifndef _REDUCE_LPS_AFTER_RTL8703B_
#ifdef _ROM_CODE_PATCHED_
#ifdef _CCH_8723_RCP_
// RCP for change the whole function or Just change the input paramter
// Change Parameter: only ce_index can be changed, lmp_pdu_ptr can't

    if (rcp_lmp_handle_remove_sco_link_request_pdu != NULL)
    {
        if ( rcp_lmp_handle_remove_sco_link_request_pdu((void*)(&ce_index), lmp_pdu_ptr) )
        {
            return BT_FW_SUCCESS;
        }
    }
#endif
#endif
#endif
    sco_handle = lmp_pdu_ptr->payload_content[1];

    if (lmp_get_sco_ce_index_from_sco_handle(sco_handle, ce_index,
                &sco_ce_index) != API_SUCCESS)
    {
        UCHAR parameter_list[LMP_NOT_ACCEPTED_LEN];
        BZ_ASSERT(0, "If remote device didn't misbehave then it's our"
                "problem");

        parameter_list[0] = LMP_NOT_ACCEPTED_OPCODE;
        parameter_list[2] = LMP_REMOVE_SCO_LINK_REQ_OPCODE;
        parameter_list[3] = INVALID_LMP_PARAMETERS_ERROR;
        lmp_generate_pdu_dm1(ce_index, parameter_list, LMP_NOT_ACCEPTED_LEN,
                (LMP_TRAN_ID)REMOTE_DEV_TID, LMP_NO_STATE_CHANGE);
    }
    else
    {
#ifdef SCO_DETACH_WORKAROUND_REMIND
        if (lmp_is_remove_sco_link_possible(ce_index) == FALSE)
        {
            BZ_ASSERT(0, "Not possible to detach SCO link");
        }
#endif
        lmp_send_lmp_accepted_dm1(ce_index, LMP_REMOVE_SCO_LINK_REQ_OPCODE,
                REMOTE_DEV_TID, LMP_NO_STATE_CHANGE);

#ifdef _CCH_WHQL_SCO_
        LMP_SCO_CONNECTION_DATA* sco_ce_ptr;
        sco_ce_ptr = &lmp_sco_connection_data[sco_ce_index];
        if (sco_ce_ptr->sco_conn_status == SCO_CONNECTED)
        {
#endif		
            lmp_kill_sco_connection(ce_index, sco_ce_index,
                    CONNECTION_TERMINATED_USER_ERROR);
#ifdef _CCH_WHQL_SCO_
        }
#endif	
        RT_BT_LOG(YELLOW, CCH_DBG_000, 1, sco_ce_ptr->sco_conn_status);
    }

    return BT_FW_SUCCESS;
}

/** 
 * Handles LMP_not_accepted for LMP_sco_link_req pdu.
 * 
 * \param reason Reason for rejection.
 * \param ce_index ACL connection entity.
 * 
 * \return BT_FW_SUCCESS, if the operation is successful. BT_FW_ERROR,
 *         otherwise.
 */
UCHAR lmp_handle_sco_link_req_not_accepted(UCHAR reason, UINT16 ce_index)
{
    UINT16 sco_ce_index;
    LMP_SCO_CONNECTION_DATA* sco_ce_ptr;

    RT_BT_LOG(GRAY, LMP_SCO_1424, 0, 0);

    if (lmp_get_sco_ce_index_from_ce_index(ce_index,
            (LMP_SCO_STATE)(SCO_CONNECTING | SCO_CHG_PARAMS), &sco_ce_index)
            != API_SUCCESS)
    {
        BZ_ASSERT(0, "Wrong pdu received or SCO state machine is wrong");
        return BT_FW_ERROR;
    }

    sco_ce_ptr = &lmp_sco_connection_data[sco_ce_index];

    switch (sco_ce_ptr->sco_conn_status)
    {
        case SCO_CONNECTING:
            lmp_gen_approp_sco_conn_completion_event(ce_index, sco_ce_index,
                    reason);
            lmp_free_sco_conn_entity(sco_ce_index);

#ifdef _CCH_SLOT_OFFSET_
            lmp_put_global_slot_offset(sco_ce_ptr->conn_entity_index, sco_ce_index + 2*LMP_MAX_CE_DATABASE_ENTRIES);
#endif

            break;

        case SCO_CHG_PARAMS:
            if (lmp_sco_get_trans_status(sco_ce_ptr) == TRS_SELF_INITIATED)
            {
                hci_generate_connection_packet_type_changed_event(
                        reason, sco_ce_ptr->sco_conn_handle,
                        lmp_sco_connection_data[LMP_MAX_SCO_CONNECTIONS].
                        host_allowed_packets);
            }
            lmp_free_temp_sco_conn_entity();
            sco_ce_ptr->sco_conn_status = SCO_CONNECTED;
            lmp_sco_set_trans_status(sco_ce_ptr, TRS_INVALID);


#ifdef _CCH_SLOT_OFFSET_
            lmp_force_global_slot_offset(sco_ce_ptr->conn_entity_index, sco_ce_index + 2*LMP_MAX_CE_DATABASE_ENTRIES, 
            						sco_ce_ptr->Tsco, 2, sco_ce_ptr->Dsco);			
#endif
            break;

        default:
            BZ_ASSERT(0, "It will never happen");
    }
    lmp_sco_connect_restore_ce_status(ce_index);

    /* In multi slave scenario, during the transition period of establishing a
     * new SCO link, we set the multi_slave_full_bandwidth_flag in the baseband
     * to properly POLL the slave. It is necessary to do this only when the
     * new SCO link will occupy the full bandwidth, if established. For
     * simplicity, it is done for every SCO link creation.
     * Here the SCO link creation has failed, so clear the
     * multi_slave_full_bandwidth_flag.
     */
    if (lmp_self_device_data.number_of_acl_conn > 1)
    {
        BB_clear_multi_slave_full_bandwidth_flag();
    }

    return BT_FW_SUCCESS;
}

/** 
 * Handles the LMP_accepted for LMP_sco_link_req pdu.
 * 
 * \param ce_index ACL connection entity index.
 * 
 * \return BT_FW_SUCCESS, if the operation is successful. BT_FW_ERROR,
 *         otherwise.
 */
UCHAR lmp_handle_sco_link_req_accepted(UINT16 ce_index)
{
    UINT16 sco_ce_index;
    LMP_SCO_CONNECTION_DATA* sco_ce_ptr;
    LMP_SCO_CONNECTION_DATA* temp_sco_ce_ptr;
    LMP_CONNECTION_ENTITY* ce_ptr;

    ce_ptr = &lmp_connection_entity[ce_index];

    if (lmp_get_sco_ce_index_from_ce_index(ce_index,
            (LMP_SCO_STATE)(SCO_CONNECTING | SCO_CHG_PARAMS), &sco_ce_index)
            != API_SUCCESS)
    {
        RT_BT_LOG(GRAY, LMP_SCO_1499, 0, 0);
        BZ_ASSERT(0, "Wrong pdu received or SCO state machine is wrong");
        return BT_FW_ERROR;
    }

    sco_ce_ptr = &lmp_sco_connection_data[sco_ce_index];

    switch (sco_ce_ptr->sco_conn_status)
    {
        case SCO_CONNECTING:
#if 0
            if (ce_ptr->remote_dev_role == SLAVE)
#endif
            {
                lmp_gen_approp_sco_conn_completion_event(ce_index, sco_ce_index,
                        HCI_COMMAND_SUCCEEDED);
            }
            lmp_self_device_data.total_no_of_sco_conn++;
            ce_ptr->no_of_sco_connections++;
            break;

        case SCO_CHG_PARAMS:
            lc_kill_sco_connection(sco_ce_index);

            lmp_self_device_data.total_no_of_sco_conn++;
            ce_ptr->no_of_sco_connections++;

            /* Copy the re-negotiated params to the original SCO entity */
            temp_sco_ce_ptr = &lmp_sco_connection_data[LMP_MAX_SCO_CONNECTIONS];
            sco_ce_ptr->pkt_type = temp_sco_ce_ptr->pkt_type;
            sco_ce_ptr->Tsco = temp_sco_ce_ptr->Tsco;
            sco_ce_ptr->Dsco = temp_sco_ce_ptr->Dsco;
            sco_ce_ptr->time_control_flags = temp_sco_ce_ptr->time_control_flags;



#ifdef _CCH_SLOT_OFFSET_
            lmp_force_global_slot_offset(sco_ce_ptr->conn_entity_index, sco_ce_index + 2*LMP_MAX_CE_DATABASE_ENTRIES, 
            						sco_ce_ptr->Tsco, 2, sco_ce_ptr->Dsco);			
#endif

            if (lmp_sco_get_trans_status(sco_ce_ptr) == TRS_SELF_INITIATED)
            {
                hci_generate_connection_packet_type_changed_event(
                    HCI_COMMAND_SUCCEEDED,
                    sco_ce_ptr->sco_conn_handle,
                    temp_sco_ce_ptr->host_allowed_packets);
            }
            lmp_free_temp_sco_conn_entity();
            break;

        default:
            BZ_ASSERT(0, "It will never happen");
    }

    sco_ce_ptr->sco_conn_status = SCO_CONNECTED;
    lmp_sco_set_trans_status(sco_ce_ptr, TRS_INVALID);

    lc_handle_connect_sco(ce_ptr->am_addr, sco_ce_index,ce_ptr->phy_piconet_id);

//#if 0
//  lmp_determine_full_bandwidth();
//#else
    /* Master updates the full-bandwidth flags upon receiving the
    * LMP_accepted(LMP_sco_link_req) whereas the slave waits for the BB_ACK
    * of LMP_accepted(LMP_sco_link_req) to update the same.
    */
#ifdef COMPILE_SNIFF_MODE
    if (ce_ptr->remote_dev_role == SLAVE)
    {
        lmp_determine_full_bandwidth();
    }
#endif /* COMPILE_SNIFF_MODE */
//#endif

    /* Master updates the full-bandwidth flags upon receiving the
    * LMP_accepted(LMP_sco_link_req) whereas the slave waits for the BB_ACK
    * of LMP_accepted(LMP_sco_link_req) to update the same.
    */
#ifdef COMPILE_SNIFF_MODE
    if (ce_ptr->remote_dev_role == SLAVE)
    {
        lmp_determine_full_bandwidth();
    }
#endif /* COMPILE_SNIFF_MODE */

    lmp_sco_connect_restore_ce_status(ce_index);

    lmp_update_pkts_after_sco_links_change(ce_index);
    lc_check_and_enable_scans_in_scatternet();
    return BT_FW_SUCCESS;
}

/**
 * Handle PDU_SENT_SIGNAL for LMP_accepted of LMP_sco_link_req. It creates the
 * SCO link.
 *
 * \param ce_index ACL connection entity index.
 *
 * \return BT_FW_SUCCESS, if the operation is successful. BT_FW_ERROR,
 *         otherwise.
 */
UCHAR lmp_handle_sco_link_req_accepted_sent(UINT16 ce_index)
{
    UCHAR result;
    result = lmp_handle_sco_link_req_accepted(ce_index);
    if(result == BT_FW_SUCCESS)
    {
        lmp_update_pkts_after_sco_links_change(ce_index);
    }
    return result;
}

/** 
 * Send LMP_sco_link_req_pdu to the remote device with the parameters given by
 * \a sco_ce_index.
 * 
 * \param ce_index ACL connection entity index.
 * \param sco_ce_index SCO connection entity index.
 * \param tid Transaction ID to be used for PDU transmission.
 *
 * \return None.
 */
void lmp_send_sco_link_request_pdu(UINT16 ce_index, UINT16 sco_ce_index,
        LMP_TRAN_ID tid)
{
    LMP_SCO_CONNECTION_DATA* sco_ce_ptr;
    UCHAR parameter_list[LMP_SCO_LINK_REQ_LEN];

    lmp_update_pkts_after_sco_links_change(ce_index);

    sco_ce_ptr = &lmp_sco_connection_data[sco_ce_index];
    parameter_list[0] = LMP_SCO_LINK_REQ_OPCODE;
    parameter_list[2] = sco_ce_ptr->sco_handle;
    parameter_list[3] = sco_ce_ptr->time_control_flags;
    parameter_list[4] = sco_ce_ptr->Dsco;
    parameter_list[5] = sco_ce_ptr->Tsco;
    parameter_list[6] = sco_ce_ptr->pkt_type;
    parameter_list[7] = sco_ce_ptr->air_mode;
    lmp_generate_pdu(ce_index, parameter_list, LMP_SCO_LINK_REQ_LEN, tid,
            LMP_NO_STATE_CHANGE);

    /* In multi slave scenario, during the transition period of establishing a
     * new SCO link, we set the multi_slave_full_bandwidth_flag in the baseband
     * to properly POLL the slave. It is necessary to do this only when the
     * new SCO link will occupy the full bandwidth, if established. For
     * simplicity, it is done for every SCO link creation.
     * Here we initiate SCO link creation, so set the
     * multi_slave_full_bandwidth_flag.
     */

#ifndef _CCH_NO_PAUSE_SCO_	
    if (lmp_self_device_data.number_of_acl_conn > 1)
#else
    if (lmp_self_device_data.number_of_acl_conn > 3)
#endif	
    {
        BB_set_multi_slave_full_bandwidth_flag();
    }
}

#if LMP_PARAMS_CHECK > 0
/** 
 * Validates the \a sco_params against the specification.
 * 
 * \param sco_params Parameters given by the remote device.
 * \param remote_dev_role Remote device role (MASTER or SLAVE).
 * 
 * \return BT_FW_SUCCESS, if the parameters are valid. A valid error code,
 *         otherwise.
 */
UCHAR lmp_validate_pdu_sco_params(const LMP_SCO_PARAMS* sco_params,
        UCHAR remote_dev_role)
{
    if (sco_params->pkt_type > LMP_HV3
            || sco_params->Tsco != (2+(sco_params->pkt_type<<1))
            || sco_params->air_mode > LMP_TRANSPARENT_DATA
            || (remote_dev_role == MASTER && sco_params->tid == 0))
    {
        return INVALID_LMP_PARAMETERS_ERROR;
    }

    return BT_FW_SUCCESS;
}
#endif

#if (LMP_PARAMS_CHECK > 0) || (LMP_TID_CHECK > 0)
/** 
 * Handles the invalid PDU received from the remote device and takes necessary
 * actions.
 * 
 * \param ce_index ACL connection entity index.
 * \param sco_ce_index SCO connection entity index.
 * \param reason Reason for reject the remote device PDU as invalid.
 * \param psco_ce_index Pointer to SCO connection entity index (Out parameter
 *                      - the caller will free sco_ce_index, if it is not
 *                      INVALID_CE_INDEX).
 * \return None.
 */
void lmp_handle_invalid_sco_link_req(UINT16 ce_index,
        UINT16 sco_ce_index, UCHAR reason, UINT16* psco_ce_index)
{
    LMP_SCO_CONNECTION_DATA* sco_ce_ptr;

    sco_ce_ptr = &lmp_sco_connection_data[sco_ce_index];
    *psco_ce_index = INVALID_CE_INDEX;
    if (lmp_sco_get_trans_status(sco_ce_ptr) == TRS_SELF_INITIATED)
    {
        /* This case is as good as receiving a LMP_not_accepted for
         * LMP_sco_link_req_pdu. The error handling to be done is also same.
         */
        lmp_handle_sco_link_req_not_accepted(reason, ce_index);
    }
    else if (sco_ce_ptr->sco_conn_status == SCO_ALLOCATED)
    {
        *psco_ce_index = sco_ce_index;
    }
}
#endif

/** 
 * Handles Change SCO connection packet type request from the remote device.
 * 
 * \param ce_index ACL connection entity index.
 * \param sco_ce_index SCO connection entity index.
 * \param sco_params SCO parameters given by the remote device.
 * 
 * \return BT_FW_SUCCESS, if the operation was successful. A valid error code,
 *         otherwise.
 */
UCHAR lmp_handle_change_sco_conn_pkt_type(UINT16 ce_index,
        UINT16 sco_ce_index, LMP_SCO_PARAMS* sco_params)
{
    UINT16 preferred_pkt_type;
    UCHAR reason;
    LMP_SCO_CONNECTION_DATA* temp_sco_ce_ptr;
    LMP_SCO_CONNECTION_DATA* sco_ce_ptr;
    LMP_CONNECTION_ENTITY* ce_ptr;

    temp_sco_ce_ptr = &lmp_sco_connection_data[LMP_MAX_SCO_CONNECTIONS];
    sco_ce_ptr = &lmp_sco_connection_data[sco_ce_index];
    ce_ptr = &lmp_connection_entity[ce_index];

    if (lmp_self_device_data.total_no_of_sco_conn > 1)
    {
        return UNSUPPORTED_PARAMETER_VALUE_ERROR;
    }

    if( lc_check_if_device_is_in_scatternet() == TRUE)
    {
        RT_BT_LOG(GRAY, LMP_SCO_1742, 0, 0);
        return UNSPECIFIED_ERROR;
    }

    if (sco_ce_ptr->air_mode != sco_params->air_mode)
    {
        /* Air mode can not be changed */
        return SCO_AIR_MODE_REJECTED_ERROR;
    }

    if (sco_ce_ptr->sco_conn_status == SCO_CONNECTED)  /* Remote dev initiates
                                                          transaction */
    {
        preferred_pkt_type = SCO_PKT_TYPE_LMP_TO_1_1_HCI(sco_params->pkt_type);
        reason = lmp_generate_sco_params_for_change_pkt_type(sco_ce_index,
                preferred_pkt_type, 8 /* DONT CARE */, preferred_pkt_type);
        if (reason != BT_FW_SUCCESS)
        {
            return reason;
        }
    }
    else    /* SCO_CHG_PARAMS */
    {
        BZ_ASSERT(temp_sco_ce_ptr->sco_conn_status != SCO_FREE,
                "Temp sco conn entity must have been allocated already");
        if (temp_sco_ce_ptr->pkt_type != sco_params->pkt_type)
        {
            return SCO_INTERVAL_REJECTED_ERROR;
        }
        temp_sco_ce_ptr->time_control_flags = sco_params->timing_ctrl_flags;
        temp_sco_ce_ptr->Dsco = sco_params->Dsco;
    }

    if (ce_ptr->remote_dev_role == SLAVE)
    {
        lmp_send_sco_link_request_pdu(ce_index, LMP_MAX_SCO_CONNECTIONS,
                (LMP_TRAN_ID)sco_params->tid);
    }
    else
    {
        lmp_send_lmp_accepted_dm1(ce_index, LMP_SCO_LINK_REQ_OPCODE,
                sco_params->tid, LMP_NO_STATE_CHANGE);
    }

    sco_ce_ptr->sco_conn_status = SCO_CHG_PARAMS;

    return BT_FW_SUCCESS;
}

/** 
 * Handles LMP_sco_link_req_pdu for a new sco link from the remote device.
 * 
 * \param ce_index ACL connection entity index.
 * \param sco_ce_index SCO connection entity index.
 * \param sco_params SCO parameters given by the remote device.
 * 
 * \return BT_FW_SUCCESS, if the operation was successful. A valid error code,
 *         otherwise.
 */
UCHAR lmp_handle_sco_link_req_pdu_for_new_link(UINT16 ce_index,
        UINT16 sco_ce_index, LMP_SCO_PARAMS* sco_params)
{
    UCHAR generate_event;
    UCHAR reason;
    LMP_CONNECTION_ENTITY* ce_ptr;
    LMP_SCO_CONNECTION_DATA* sco_ce_ptr;
    UINT16 connection_accept_to;

    ce_ptr = &lmp_connection_entity[ce_index];
    sco_ce_ptr = &lmp_sco_connection_data[sco_ce_index];

    sco_ce_ptr->sco_handle = sco_params->handle;
    sco_ce_ptr->time_control_flags = sco_params->timing_ctrl_flags;
    sco_ce_ptr->Dsco = sco_params->Dsco;
    sco_ce_ptr->Tsco = sco_params->Tsco;
    sco_ce_ptr->pkt_type = sco_params->pkt_type;
    sco_ce_ptr->air_mode = sco_params->air_mode;

    generate_event = hci_pass_event_through_event_filter(
            CONNECTION_SETUP_FILTER, ce_ptr->bd_addr, ce_ptr->class_of_device);
    switch (generate_event)
    {
        case FALSE:
            return HOST_REJECTED_SECURITY_REASONS_ERROR;

        case HCI_CONNECTION_REQUEST_EVENT:
            connection_accept_to = lmp_self_device_data.conn_accept_timeout;
            if (!hci_generate_connection_request_event(ce_ptr->bd_addr,
                    ce_ptr->class_of_device, (UCHAR)SCO_LINK))
            {
                connection_accept_to = 0x1;
            }

            if (OS_CREATE_TIMER(ONESHOT_TIMER, &ce_ptr->conn_accept_timer_handle,
                    lmp_conn_accept_timeout_timer_handler,
                    (void *)((UINT32)sco_ce_ptr->sco_conn_handle), 0) != BT_ERROR_OK)
            {
                BZ_ASSERT(0, "Conn Accept Timer creation failed");
            }

            if (OS_START_TIMER(ce_ptr->conn_accept_timer_handle,
                        connection_accept_to) != BT_ERROR_OK)
            {
                BZ_ASSERT(0, "Conn Accept Timer start failed");
            }
            sco_ce_ptr->sco_conn_status = SCO_WAITING_FOR_CONN_ACCEPT;
            break;

        case HCI_CONNECTION_COMPLETE_EVENT:
        case HCI_CONNECTION_COMPLETE_EVENT_WITH_RS:
        {
            UINT16 allowed_pkt_types;

            if (lmp_self_device_data.lmp_air_mode != sco_params->air_mode)
            {
                return SCO_AIR_MODE_REJECTED_ERROR;
            }

            allowed_pkt_types = (UINT16)(HV1|HV2|HV3);
            sco_ce_ptr->host_allowed_packets = allowed_pkt_types;
            sco_ce_ptr->max_latency = 8 /* DONT CARE */;

            if (ce_ptr->remote_dev_role == SLAVE)
            {

#ifdef _CCH_SLOT_OFFSET_			
                reason = lmp_generate_sco_params(sco_ce_index,
                        allowed_pkt_types, 8 /* DONT CARE */,
                        SCO_PKT_TYPE_LMP_TO_1_1_HCI(sco_params->pkt_type), 0);
#else
		
                reason = lmp_generate_sco_params(sco_ce_index,
                        allowed_pkt_types, 8 /* DONT CARE */,
                        SCO_PKT_TYPE_LMP_TO_1_1_HCI(sco_params->pkt_type));
#endif



                if (reason != BT_FW_SUCCESS)
                {
                    return reason;
                }
                lmp_send_sco_link_request_pdu(ce_index, sco_ce_index,
                        (LMP_TRAN_ID)sco_params->tid);
            }
            else    /* We are slave */
            {
                reason = lmp_validate_sco_params(sco_ce_index,
                        allowed_pkt_types, 8 /* DONT CARE */);
                if (reason != BT_FW_SUCCESS)
                {
                    return reason;
                }
                lmp_send_lmp_accepted_dm1(ce_index, LMP_SCO_LINK_REQ_OPCODE,
                        sco_params->tid, LMP_NO_STATE_CHANGE);
            }
            sco_ce_ptr->sco_conn_status = SCO_CONNECTING;
            break;
        }
        default:
            BZ_ASSERT(0, "Buggy hci_pass_event_through_event_filter");
    } /* end switch(generate_event) */

    return BT_FW_SUCCESS;
}

/** 
 * Handles the LMP_sco_link_req_pdu from the remote device. The handler is
 * divided into #lmp_handle_sco_link_request_pdu and
 * #lmp_handle_sub_sco_link_req to avoid massive error handling code
 * duplication. According to this method, this function just returns the error
 * code, in case of any error. The caller of this function will do the
 * necessary error handling.
 * 
 * \param ce_index ACL connection entity index.
 * \param sco_params SCO parameters given by the remote device.
 * \param psco_ce_index Pointer to SCO connection entity index. This variable
 *                      is set to a valid sco_ce_index if it has to be freed.
 *                      Otherwise INVALID_CE_INDEX is set.
 * 
 * \return BT_FW_SUCCESS, if the operation was successful. A valid error code,
 *         otherwise.
 */
UCHAR lmp_handle_sub_sco_link_req(UINT16 ce_index,
        LMP_SCO_PARAMS* sco_params, UINT16* psco_ce_index)
{
    UINT16 sco_ce_index;
    UCHAR reason;
    LMP_CONNECTION_ENTITY *ce_ptr;
    LMP_SCO_CONNECTION_DATA* sco_ce_ptr;

    ce_ptr = &lmp_connection_entity[ce_index];

    *psco_ce_index = INVALID_CE_INDEX;
#if LMP_PARAMS_CHECK > 0
    if ((reason = lmp_validate_pdu_sco_params(&sco_params,
                    ce_ptr->remote_dev_role)) != BT_FW_SUCCESS)
    {
        RT_BT_LOG(GRAY, LMP_SCO_1934, 0, 0);

        if (lmp_get_sco_ce_index_from_sco_handle(sco_params->handle, ce_index,
                &sco_ce_index) == API_SUCCESS)
        {
            RT_BT_LOG(GRAY, LMP_SCO_1939, 0, 0);

            lmp_handle_invalid_sco_link_req(ce_index, sco_ce_index, reason,
                    psco_ce_index);
        }
        return reason;
    }
#endif

    RT_BT_LOG(GRAY, LMP_SCO_1948, 3,
                ce_ptr->ce_status, sco_params->handle, sco_ce_index);

    switch (ce_ptr->ce_status)
    {
        case LMP_CONNECTED:
        case LMP_ADDING_SCO_LINK:               /* Fall through */
#ifdef COMPILE_SNIFF_MODE
        case LMP_SNIFF_MODE:                    /* Fall through */
        case LMP_ADDING_SCO_LINK_DURING_SNIFF:  /* Fall through */
#endif
            break;

        default:
            return PDU_NOT_ALLOWED_ERROR;
    }
#ifdef _SUPPORT_VER_4_1_
#ifdef _SUPPORT_PCA_ADJUST
    if (IS_BT41)
    {
        if (bt_pca_manager.pca_updating)
        {
            return PDU_NOT_ALLOWED_ERROR;
        }    
    }
#endif
#endif

    lmp_sco_connect_update_ce_status(ce_index);
    if (lmp_get_sco_ce_index_from_sco_handle(sco_params->handle, ce_index,
                &sco_ce_index) != API_SUCCESS)
    {
        //RT_BT_LOG(GRAY, LMP_SCO_1969, 1, ce_ptr->remote_dev_role);

        if (ce_ptr->remote_dev_role == SLAVE ||
                lmp_get_sco_ce_index_from_ce_index(ce_index, SCO_CONNECTING,
                    &sco_ce_index) != API_SUCCESS)
        {
            //RT_BT_LOG(GRAY, LMP_SCO_1975, 0, 0);

            reason = lmp_allocate_sco_conn_entity(&sco_ce_index, ce_index);
            if (reason != BT_FW_SUCCESS)
            {
                return reason;
            }
        }
    }

    RT_BT_LOG(GRAY, LMP_SCO_1985, 1, sco_ce_index);

    *psco_ce_index = sco_ce_index;
    sco_ce_ptr = &lmp_sco_connection_data[sco_ce_index];
    if (lmp_sco_get_trans_status(sco_ce_ptr) == TRS_INVALID)
    {
        lmp_sco_set_trans_status(sco_ce_ptr, TRS_REMOTE_INITIATED);
#if LMP_TID_CHECK > 0
        if (sco_params->tid != ce_ptr->remote_dev_role)
        {
            lmp_handle_invalid_sco_link_req(ce_index, sco_ce_index,
                    INVALID_LMP_PARAMETERS_ERROR, psco_ce_index);
            return INVALID_LMP_PARAMETERS_ERROR;
        }
#endif
    }
#if LMP_TID_CHECK > 0
    else if (lmp_sco_get_trans_status(sco_ce_ptr) == TRS_SELF_INITIATED)
    {
        /* Only slave can recv the SCO_link_req, eventhough it initiated the
         * transaction.
         */
        if (sco_params->tid != SLAVE)
        {
            lmp_handle_invalid_sco_link_req(ce_index, sco_ce_index,
                    INVALID_LMP_PARAMETERS_ERROR, psco_ce_index);
            return INVALID_LMP_PARAMETERS_ERROR;
        }
    }
    else
    {
        BZ_ASSERT(0, "Transaction Status is screwed up")
    }
#endif

    /* For remote device initiated transactions, additional scatternet 
    constraints are enforced. */

    if(lc_check_if_device_is_in_scatternet() == TRUE)
    {
        if(lmp_self_device_data.total_no_of_sco_conn != 0)
        {
            /* Only one SCO connection allowed in scatternet. */
            return HOST_REJECTED_LIMITED_RESOURCES_ERROR;
        }

        if(sco_params->pkt_type != LMP_HV3)
        {
            /* Only HV3 connection is allowed when in scatternet. Other SCO packet types are not allowed. */
            return HOST_REJECTED_LIMITED_RESOURCES_ERROR;
        }
    }

//    RT_BT_LOG(GRAY, LMP_SCO_2050, 1, sco_ce_ptr->sco_conn_status);

#ifdef _CCH_SLOT_OFFSET_
    UINT16 temp_slot_num;
    UCHAR slot_overlap;
    slot_overlap = 0;
    temp_slot_num = 2;
    slot_overlap = lmp_force_global_slot_offset(ce_index, sco_ce_index + 2*LMP_MAX_CE_DATABASE_ENTRIES, sco_params->Tsco, 
                                               temp_slot_num, sco_params->Dsco);
#endif

    switch (sco_ce_ptr->sco_conn_status)
    {
        case SCO_DISCONNECTING:
            *psco_ce_index = INVALID_CE_INDEX;
            return PDU_NOT_ALLOWED_ERROR;

        case SCO_CONNECTED:
        case SCO_CHG_PARAMS:    /* Fall through */
            if (sco_ce_ptr->sco_conn_status == SCO_CHG_PARAMS)
            {
                BZ_ASSERT(ce_ptr->remote_dev_role == MASTER,
                        "Only Slave can be in this state");
            }
            reason = lmp_handle_change_sco_conn_pkt_type(ce_index,
                    sco_ce_index, sco_params);
            if (reason != BT_FW_SUCCESS)
            {
                if (lmp_sco_get_trans_status(sco_ce_ptr) == TRS_SELF_INITIATED)
                {
                    hci_generate_connection_packet_type_changed_event(reason,
                            sco_ce_ptr->sco_conn_handle,
                            lmp_sco_connection_data[LMP_MAX_SCO_CONNECTIONS].
                            host_allowed_packets);
                }
                lmp_free_temp_sco_conn_entity();
                sco_ce_ptr->sco_conn_status = SCO_CONNECTED;
                lmp_sco_set_trans_status(sco_ce_ptr, TRS_INVALID);
                *psco_ce_index = INVALID_CE_INDEX;
                return reason;
            }
            break;

        case SCO_CONNECTING:
            BZ_ASSERT(ce_ptr->remote_dev_role == MASTER,
                    "Master can never be in this state, Some problem!!!");
            sco_ce_ptr->pkt_type = sco_params->pkt_type;
            reason = lmp_validate_sco_params(sco_ce_index,
                    sco_ce_ptr->host_allowed_packets, sco_ce_ptr->max_latency);

            RT_BT_LOG(GREEN, LMP_SCO_2090, 3,
                        reason, sco_params->air_mode, sco_ce_ptr->air_mode);

            if ((reason == BT_FW_SUCCESS) && 
                            (sco_params->air_mode != sco_ce_ptr->air_mode))
            {
                reason = SCO_AIR_MODE_REJECTED_ERROR;
            }

            if (reason != BT_FW_SUCCESS)
            {
                RT_BT_LOG(RED, LMP_SCO_2096, 1, reason);
                lmp_gen_approp_sco_conn_completion_event(ce_index,
                    sco_ce_index, reason);
                return reason;
            }

//            RT_BT_LOG(GRAY, LMP_SCO_2102, 2, sco_ce_ptr->sco_handle, sco_params->handle);

            sco_ce_ptr->Tsco = sco_params->Tsco;
            sco_ce_ptr->Dsco = sco_params->Dsco;
            sco_ce_ptr->sco_handle = sco_params->handle;
            sco_ce_ptr->time_control_flags = sco_params->timing_ctrl_flags;
            lmp_send_lmp_accepted_dm1(ce_index, LMP_SCO_LINK_REQ_OPCODE,
                    sco_params->tid, LMP_NO_STATE_CHANGE);
            break;

        case SCO_ALLOCATED:
            reason = lmp_handle_sco_link_req_pdu_for_new_link(ce_index,
                    sco_ce_index, sco_params);
            return reason;

        default:
            BZ_ASSERT(0, "SCO state machine got screwed up");
    }

    return BT_FW_SUCCESS;
}

/** 
 * Handles the LMP_sco_link_req pdu from the remote device. This function
 * simply extracts the parameters from the PDU and delegates the rest of the
 * work to #lmp_handle_sub_sco_link_req. It does the error handling in case of
 * any error.
 * 
 * \param lmp_pdu_ptr Pointer to LMP PDU.
 * \param ce_index ACL connection entity index.
 * 
 * \return BT_FW_SUCCESS, if the operation is successful. BT_FW_ERROR,
 *         otherwise.
 */
UCHAR lmp_handle_sco_link_request_pdu(LMP_PDU_PKT *lmp_pdu_ptr,
        UINT16 ce_index)
{
    UINT16 sco_ce_index;
    UCHAR reason;
    LMP_SCO_PARAMS sco_params;

    /* Extract the SCO parameters from the SCO_Link_req PDU */
    sco_params.handle = lmp_pdu_ptr->payload_content[1];
    sco_params.timing_ctrl_flags = lmp_pdu_ptr->payload_content[2];
    sco_params.Dsco = lmp_pdu_ptr->payload_content[3];
    sco_params.Tsco = lmp_pdu_ptr->payload_content[4];
    sco_params.pkt_type = lmp_pdu_ptr->payload_content[5];
    sco_params.air_mode = lmp_pdu_ptr->payload_content[6];
    sco_params.tid = (UCHAR)(lmp_pdu_ptr->payload_content[0] & 0x01);

    RT_BT_LOG(GREEN, LMP_SCO_2152, 6,
        sco_params.handle,
        sco_params.timing_ctrl_flags,
        sco_params.Dsco, 
        sco_params.Tsco,
        sco_params.pkt_type,
        sco_params.air_mode);

    reason = lmp_handle_sub_sco_link_req(ce_index, &sco_params, &sco_ce_index);
    if (reason != BT_FW_SUCCESS)
    {
        lmp_send_lmp_not_accepted(ce_index, LMP_SCO_LINK_REQ_OPCODE,
                sco_params.tid, reason);
        if (sco_ce_index != INVALID_CE_INDEX)
        {
            lmp_free_sco_conn_entity(sco_ce_index);
        }
        lmp_sco_connect_restore_ce_status(ce_index);
    }

    return BT_FW_SUCCESS;
}

/** 
 * Disconnect all the SCO links associated with the ACL connection pointed by
 * \a ce_index. It also generates the Disconnection_Complete events
 * appropriately.
 * 
 * \param ce_index Index to the ACL connection entity database.
 * \param reason Disconnect reason.
 *
 * \return None.
 *
 * \note No LMP_remove_sco_link_req PDU will be sent to the remote device. It
 *       just kills the baseband level connections and frees LMP data
 *       structures associated with the links.
 */
void lmp_disconnect_sco_links(UINT16 ce_index, UCHAR reason)
{
    LMP_SCO_CONNECTION_DATA* sco_ce_ptr;
    LMP_CONNECTION_ENTITY* ce_ptr;
    UCHAR index;
#ifdef SCO_OVER_HCI
    int ndisconnects = 0;
    DEF_CRITICAL_SECTION_STORAGE;
#endif /* SCO_OVER_HCI */

    for (index = 0; index < LMP_MAX_SCO_CONNECTIONS; index++)
    {
        sco_ce_ptr = &lmp_sco_connection_data[index];
        if (sco_ce_ptr->sco_conn_status != SCO_FREE
            && sco_ce_ptr->conn_entity_index == ce_index)
        {
            ce_ptr = &lmp_connection_entity[ce_index];
            switch (sco_ce_ptr->sco_conn_status)
            {
                case SCO_CONNECTED:     /* Fall through */
                case SCO_DISCONNECTING: /* Fall through */
                case SCO_CHG_PARAMS:
#ifdef SCO_OVER_HCI
                    MINT_OS_ENTER_CRITICAL();
                    ndisconnects ++;
#endif /* SCO_OVER_HCI */
                    if(lc_kill_sco_connection(index) != API_SUCCESS)
                    {
                        LMP_ERR(SCO_DISCONNECT_FAILED,0,0);
                    }
#ifdef SCO_OVER_HCI
                    MINT_OS_EXIT_CRITICAL();
#endif /* SCO_OVER_HCI */

#if 0
#ifdef _CCH_WHQL_SCO_
                    if(lmp_self_device_data.total_no_of_sco_conn !=0 )
                    {
                        lmp_self_device_data.total_no_of_sco_conn--;
                    }
#else
                    lmp_self_device_data.total_no_of_sco_conn--;
#endif	

                    if(ce_ptr->no_of_sco_connections != 0)
                    {
                        ce_ptr->no_of_sco_connections--;
                    }
#endif

                    if (ce_ptr->no_of_sco_connections == 0)
                    {
#ifndef _USE_ONE_NEW_BZDMA_TXCMD_FOR_SCO_                    
                        bzdma_invalid_txcmd(BZDMA_TX_ENTRY_TYPE_SCO, 0, index);
#else
                        bzdma_invalid_txcmd(BZDMA_TX_ENTRY_TYPE_NEW_SCO0, 0, index);
#endif
                    }

                    hci_generate_disconnection_complete_event(
                            HCI_COMMAND_SUCCEEDED, sco_ce_ptr->sco_conn_handle,
                            reason);
#ifdef _CCH_SLOT_OFFSET_
                    lmp_put_global_slot_offset(ce_index, index + 2*LMP_MAX_CE_DATABASE_ENTRIES);
#endif
                    lmp_free_sco_conn_entity(index);
                    break;

                case SCO_WAITING_FOR_CONN_ACCEPT:
                    OS_DELETE_TIMER(&ce_ptr->conn_accept_timer_handle);
                case SCO_CONNECTING: /* Fall through */
                    lmp_gen_approp_sco_conn_completion_event(ce_index,
                            index, reason);
                case SCO_ALLOCATED: /* Fall through */
                    lmp_free_sco_conn_entity(index);
                    break;

                default:
                    break;
            } /* end of switch(sco connection status) */
        } /* end if (the sco link is on ACL link pointed by ce_index) */
    } /* end for (all the sco links) */

    /* Free the Temp SCO conn entity if it is being used by the ACL connection
     * that is getting killed.
     */
    sco_ce_ptr = &lmp_sco_connection_data[LMP_MAX_SCO_CONNECTIONS];
    if (sco_ce_ptr->sco_conn_status != SCO_FREE
            && sco_ce_ptr->conn_entity_index == ce_index)
    {
        lmp_free_temp_sco_conn_entity();
    }

#ifdef SCO_OVER_HCI
    if (ndisconnects != 0)
    {
        /* Spin for 6 slots */
        bz_spin_for_nclk0_ticks(12);
    }
#endif /* SCO_OVER_HCI */

#ifdef COMPILE_SNIFF_MODE
    lmp_determine_full_bandwidth();
#endif

    hci_check_and_enable_eir_recv();
}

/** 
 * Start the disconnection procedure of the SCO link given by \a conn_handle.
 * 
 * \param conn_handle Connection handle of the SCO link to be disconnected.
 * \param reason Reason for disconnection.
 * \param sent_cmd_status Set to TRUE, if command status event is sent to the
 *                        host. Otherwise, FALSE.
 * 
 * \return API_SUCCESS, if the starting of the SCO disconnection procedure is
 *         successful. API_FAULRE, otherwise.
 */
API_RESULT lmp_handle_sco_disconnect(UINT16 conn_handle, UCHAR reason,
                                     UCHAR* sent_cmd_status)
{
    LMP_SCO_CONNECTION_DATA* sco_ce_ptr;
    UINT16 sco_ce_index;
    UCHAR parameter_list[LMP_REMOVE_SCO_LINK_REQ_LEN];
    UINT16 ce_index;

    if (lmp_get_sco_ce_index_from_conn_handle(conn_handle, &sco_ce_index)
            == API_SUCCESS)
    {
        RT_BT_LOG(GREEN, LMP_SCO_2297, 0, 0);
        sco_ce_ptr = &lmp_sco_connection_data[sco_ce_index];
        ce_index = sco_ce_ptr->conn_entity_index;
#ifdef SCO_DETACH_WORKAROUND_REMIND
        if (sco_ce_ptr->sco_conn_status != SCO_CONNECTED ||
                lmp_is_remove_sco_link_possible(ce_index) == FALSE)
#else
        if (sco_ce_ptr->sco_conn_status != SCO_CONNECTED)
#endif
        {
            *sent_cmd_status = TRUE;
            hci_generate_command_status_event(HCI_DISCONNECT_OPCODE,
                                              COMMAND_DISALLOWED_ERROR);
            return API_SUCCESS;
        }

        parameter_list[0] = LMP_REMOVE_SCO_LINK_REQ_OPCODE;
        parameter_list[2] = sco_ce_ptr->sco_handle;
        parameter_list[3] = reason;

        sco_ce_ptr->sco_conn_status = SCO_DISCONNECTING;
        lmp_sco_set_trans_status(sco_ce_ptr, TRS_SELF_INITIATED);

        lmp_generate_pdu_dm1(ce_index, parameter_list,
                LMP_REMOVE_SCO_LINK_REQ_LEN, SELF_DEV_TID,
                LMP_NO_STATE_CHANGE);
#ifdef _DAPE_PUT_SLOT_OFFSET_EARLIER_FOR_WIN8
#ifdef _CCH_SLOT_OFFSET_
        lmp_put_global_slot_offset(sco_ce_ptr->conn_entity_index, sco_ce_index + 2*LMP_MAX_CE_DATABASE_ENTRIES);
#endif
#endif
        return API_SUCCESS;
    }
    else
    {
        return API_FAILURE;
    }
}

/** 
 * Generate either Conn_Complete_Event or Sync_Conn_Complete_Event based on
 * the host.
 * 
 * \param ce_index ACL connection entity index.
 * \param sco_ce_index SCO connection entity index.
 * \param status Connection Completion Status.
 *
 * \return None.
 */
void lmp_gen_approp_sco_conn_completion_event(UINT16 ce_index,
        UINT16 sco_ce_index, UCHAR status)
{
    LMP_CONNECTION_ENTITY* ce_ptr;
    LMP_SCO_CONNECTION_DATA* sco_ce_ptr;

    ce_ptr = &lmp_connection_entity[ce_index];
    sco_ce_ptr = &lmp_sco_connection_data[sco_ce_index];

    if (sco_ce_ptr->gen_conn_complete == TRUE)
    {
        hci_generate_connection_complete_event(status,
                sco_ce_ptr->sco_conn_handle, ce_ptr->bd_addr,
                SCO_LINK, (UCHAR) bz_auth_get_encryption_mode(ce_index));
    }
    else
    {
        hci_generate_synchronous_conn_complete_event(ce_index, sco_ce_index,
                SCO_LINK, status);
    }
}
#endif /* ENABLE_SCO */
