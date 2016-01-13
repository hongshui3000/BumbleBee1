/***************************************************************************
 Copyright (C) MindTree Ltd.
 This module is a confidential and proprietary property of MindTree and
 a possession or use of this module requires written permission of MindTree.
 ***************************************************************************/

/**
 * \file
 *  Contains the code that implements utility functions
 *  used in the hci module.
 */

/********************************* Logger *************************/
enum { __FILE_NUM__= 12 };
/********************************* Logger *************************/

/* Includes */
#include "mem.h"

#include "bt_fw_hci_internal.h"
#ifdef COMPILE_CHANNEL_ASSESSMENT
#include "lmp_ch_assessment.h"
#endif /* COMPILE_CHANNEL_ASSESSMENT */
#include "lmp_1_2_defines.h"

extern HCI_DATA_RECV_Q hci_received_data_Q;

#ifdef COMPILE_ESCO
extern LMP_ESCO_CONNECTION_ENTITY
lmp_esco_connection_entity[LMP_MAX_ESCO_CONN_ENTITIES];
#endif


/**
 * Converts an integer into a string of 'bytes'
 * number of bytes.
 *
 * \param buffer Pointer to the buffer that is to be used to store
 *               the string which will be obtained from the integer.
 * \param number The integer that will be converted to equivalent
 *                      string.
 * \param bytes  Number of bytes of the string that will be formed
 *               after the conversion.
 *
 * \return SUCCESS or FAILURE
 *
 */
void bt_fw_ultostr(UCHAR *buffer, UINT32 number, UCHAR bytes)
{
    UCHAR count;
    for (count=0; count<bytes; count++)
    {
        buffer[count] = number & 0xFF;
        number >>= 8;
    }
}

/**
 * Converts a char byte stream, starting from a
 * particular location, into appropriate integer value based on the number
 * of bytes and the radix.
 *
 * \param buffer Pointer to the buffer that is to be used to extract
 *               the string which will in turn be used to obtain the
 *               integer.
 * \param bytes  Number of bytes from the buffer that are to be
 *               converted to integer.
 * \param base   The Radix of the integer.
 *
 * \return UINT32 integer obtained from the string.
 *
 */
UINT32 bt_fw_strtoul(const UCHAR *buffer, UCHAR bytes, UCHAR base)
{
    UINT32 number = 0;

    if (base == 16)
    {
        UCHAR count;

        for (count = bytes; count > 0; count--)
        {
            number <<= 8;
            number |= buffer[count-1];
        }
    }
    return number;
}

/**
 * Resets the data structures and the buffer pools created in HCI module.
 *
 * \param None.
 *
 * \return SUCCESS or FAILURE
 *
 */
UCHAR hci_module_reset(void)
{
    hci_initialize_data();

    return BT_FW_SUCCESS ;
}

/**
 * Initializes HCI module global variables to the default values.
 *
 * \param None.
 *
 * \return None.
 *
 */
void hci_initialize_data(void)
{
    hci_baseband_cmd_pending  = FALSE ;
    memset(&hci_received_data_Q,0,sizeof(HCI_DATA_RECV_Q));
}

/**
 * Checks whether a particular Event is to be
 * generated or not (uses the event mask).
 *
 * \param event_opcode Event Opcode.
 *
 * \return TRUE or FALSE.
 *
 */
UCHAR hci_pass_event_through_event_mask(UCHAR event_opcode)
{
    UINT32 calculated_mask[2];
    UINT32 mask[2];

#if defined(RT_VENDOR_CMDS)
    if(event_opcode > 0x40)
    {
        /*
         * Note: Event mask is only 64 bits any opcode
         * greater than 64 cannot be checked with the
         * event mask.
         */
        return TRUE;
    }
    else
#endif// (RT_VENDOR_CMDS)
    if(event_opcode > 0x20)
    {
        mask[0] = 0x00;
        mask[1] = 0x01 << (event_opcode - 0x21);
    }
    else
    {
        mask[0] = 0x01 << (event_opcode - 0x01);
        mask[1] = 0x00;
    }

    memcpy(calculated_mask, &lmp_self_device_data.event_mask[0], 8);

    if((mask[0] & calculated_mask[0]) || (mask[1] & calculated_mask[1]))
    {
        return TRUE ;
    }
    else
    {
        return FALSE ;
    }
}

#ifdef VER_CSA4
/**
 * Checks whether a particular Event is to be
 * generated or not (uses the event mask).
 *
 * \param event_opcode Event Opcode.
 *
 * \return TRUE or FALSE.
 *
 */
UCHAR hci_pass_event_through_event_mask_page2(UCHAR event_opcode)
{
    UINT32 calculated_mask[2];
    UINT32 mask[2];

    if ((event_opcode > 0x80) || (event_opcode < 0x40))
    {
        /* no allocate at event mask page 2 */
        return TRUE;
    }


    if(event_opcode > 0x60)
    {
        mask[0] = 0x00;
        mask[1] = 0x01 << (event_opcode - 0x60);
    }
    else
    {
        mask[0] = 0x01 << (event_opcode - 0x40);
        mask[1] = 0x00;
    }

    memcpy(calculated_mask, &lmp_self_device_data.event_mask_p2[0], 8);

    if((mask[0] & calculated_mask[0]) || (mask[1] & calculated_mask[1]))
    {
        return TRUE ;
    }
    else
    {
        return FALSE ;
    }
}
#endif

/**
 * Checks whether to return reponses during
 * inquiry process or not (uses event filter).
 *
 * \param bd_addr         BD Address of the device that is to be scrutinised.
 * \param class_of_device Class of Device under consideration.
 *
 * \return  generate_event : TRUE  - return the response
 *                           FALSE - ignore.
 *
 */
UCHAR hci_pass_event_through_event_filter_inquiry(UCHAR *bd_addr,
        UINT32 class_of_device)
{
    INT16 index = (INT16)(lmp_event_filters_written - 1);
    CHAR temp_index = -1;
    LMP_EVENT_FILTER *evt_filter;
        
    while(index >= 0)
    {
        evt_filter = &lmp_self_device_data.event_filter[index];
        
        /* Check the filter type */
        if (evt_filter->filter_type == INQUIRY_RESULT_FILTER)
        {
            temp_index = 0;
            switch(evt_filter->filter_condition_type)
            {
            case HCI_NEW_DEV_RESP :
                return TRUE;

            case HCI_NEW_DEV_COD :
                if (((evt_filter->class_of_device & 
                      evt_filter->class_of_device_mask) == 
                      (class_of_device & evt_filter->class_of_device_mask)) ||
                       evt_filter->class_of_device == 0)
                {
                    return TRUE;
                }
                break;
            case HCI_NEW_DEV_BD_ADDR :
                if (memcmp(evt_filter->bd_addr, bd_addr, LMP_BD_ADDR_SIZE) == 0)
                {

                    return TRUE;
                }
                break;
            default:
                break;
            }
        }
        index--;
    }
    if(temp_index == -1)
    {
        /* No Inquiry result filters found */
        return TRUE;
    }

    return FALSE;
}
/**
 * Checks whether a particular Event is to be
 * generated or not. Filter used is connection filter.
 *
 * \param bd_addr         BD Address of the device that is to be scrutinised.
 * \param class_of_device Class of Device under consideration.
 *
 * \return  generate_event : Returns the event to be generated.
 *                           HCI_CONNECTION_REQUEST_EVENT,
 *                           HCI_CONNECTION_COMPLETE_EVENT,
 *                           HCI_CONNECTION_COMPLETE_EVENT_WITH_RS
 *
 */
UCHAR hci_pass_event_through_event_filter_connection(UCHAR *bd_addr,
        UINT32 class_of_device)
{
    UCHAR auto_accept_flag_event[] =
    {
        HCI_CONNECTION_REQUEST_EVENT,   /* Check */
        HCI_CONNECTION_REQUEST_EVENT,
        HCI_CONNECTION_COMPLETE_EVENT,
        HCI_CONNECTION_COMPLETE_EVENT_WITH_RS
    };


    /*
       Filters are checked from the last
       (latest rules over-ride the previous ones)
     */
    INT16 index = (INT16)(lmp_event_filters_written - 1);
    UCHAR send_event_index = 0;
    CHAR temp_index = -1;

    /* Flags to check if these rules were 'latest'ly met */
    UCHAR cod = FALSE;
    UCHAR bd_addr_conn = FALSE;
    UCHAR allow_all_connections = FALSE;

    LMP_EVENT_FILTER *filter;

    while (index >= 0)
    {
        BT_FW_HCI_INF(CHECKING_THE_EVENT_FILTER,1,index);

        filter = &lmp_self_device_data.event_filter[index];

        /* Check the type of filter */
        if (filter->filter_type == CONNECTION_SETUP_FILTER)
        {
            temp_index = 0;
            switch (filter->filter_condition_type)
            {
            case HCI_ALLOW_ALL_CONNECTIONS :
                if(allow_all_connections == TRUE)
                {
                    break;
                }
                temp_index = filter->auto_accept_flag;
                allow_all_connections = TRUE;
                break;


            case HCI_ALLOW_COD_CONNECTIONS:
                if (((filter->class_of_device & filter->class_of_device_mask) ==
                        (class_of_device & filter->class_of_device_mask) ||
                        (filter->class_of_device == 0x0)) && (cod == FALSE))
                {

                    temp_index = filter->auto_accept_flag;
                    cod = TRUE;
                }
                break;

            case HCI_ALL_BD_ADDR_CONNECTIONS :
                if ((memcmp(filter->bd_addr, bd_addr, 6) == 0) &&
                     (bd_addr_conn == FALSE))
                {
                    temp_index = filter->auto_accept_flag;
                    bd_addr_conn = TRUE;
                }
                break;
            }

            if(send_event_index == 0)
            {
                send_event_index = temp_index;
            }

            /* Check for the best choice */
            switch(temp_index)
            {
                case 0:
                    break;

                case 1: /* Send request - choice 3 */
                    break;

                case 2: /* Allow connection with-out role switch - choice 1 */
                    send_event_index = temp_index;
                    break;

                case 3: /* Allow connection only with role switch - choice 2 */
                    if(send_event_index != 2)
                    {
                        send_event_index = temp_index;
                    }
                    break;

                default:
                    break;
            }
        }
        index--;
    }

    if (send_event_index != 0)
    {
        return auto_accept_flag_event[send_event_index];
    }
    if (temp_index == -1)
    {
        /* No Connection filters were found */
        return HCI_CONNECTION_REQUEST_EVENT;
    }

    return FALSE;

}
/**
 * Checks whether a particular Event is to be generated or not.
 *
 * \param filter_type     INQUIRY_RESULT_FILTER or CONNECTION_SETUP_FILTER.
 * \param bd_addr         BD Address of the device that is to be scrutinised.
 * \param class_of_device Class of Device under consideration.
 *
 * \return  generate_event
 *                         : for inquiry filter   : TRUE or FALSE.
 *                         : for connection filter: Event to be generated.
 */
UCHAR hci_pass_event_through_event_filter(UCHAR filter_type,
        UCHAR *bd_addr,
        UINT32 class_of_device)
{
    switch (filter_type)
    {
    case INQUIRY_RESULT_FILTER:
        return hci_pass_event_through_event_filter_inquiry(bd_addr,
                    class_of_device);
    case CONNECTION_SETUP_FILTER:
        return hci_pass_event_through_event_filter_connection(bd_addr,
                    class_of_device);

    default:
        break;
    }
    return FALSE;
}

/**
* Generates Quality Of Service parameters.
*
*
* \param ce_index Connection Entity Index.
* \param flow_direction Whether this fn is invoked by Flow_spec command.
*
* \return API_SUCCESS or API_FAILURE.
*
*/

API_RESULT hci_generate_qos_params(UINT16 ce_index, UCHAR flow_direction)
{
    UINT16 tpoll_token_rate = 0xFFFF, tpoll_latency = 0xFFFF;
    UINT16 super_to;
    LMP_CONNECTION_ENTITY *ce_ptr;

    UINT16 tpoll_val = otp_str_data.bt_t_poll;

    UINT16 *tpoll_ptr = NULL;

    ce_ptr = &lmp_connection_entity[ce_index];

    if (ce_ptr->latency != HCI_DEFAULT_QoS_ACCESS_LATENCY)
    {
        /* Calculate Qos based on access latecny. */
        tpoll_latency = (UINT16) (ce_ptr->latency / 1250) ;

        /* If calculated tpoll is lesser than tpoll-min-threshold,
        reject it with 0x2d */
        if ((tpoll_latency < LMP_MIN_TPOLL) || (tpoll_latency > LMP_MAX_TPOLL))
        {

            if(flow_direction == QOS_SETUP_DUAL_SIDE)
            {
                /* Generate QoS complete event with 0x2d. */
                hci_queue_QoS_complete_event(ce_index, QOS_REJECTED_ERROR);
            }
            else
            {
                /* Generate Flow spec complete event. */
                hci_queue_flow_spec_complete_event(ce_index,
                                                   flow_direction, 
                                                   QOS_REJECTED_ERROR);
            }

            return API_FAILURE;
        }
    }

    if (ce_ptr->token_rate != HCI_QoS_TOKEN_RATE_WILD_CARD)
    {
        tpoll_token_rate =
            (UINT16) hci_calculate_tpoll_token_rate(ce_ptr->token_rate);

        /* If calculated tpoll is not within the sup range,
        then return error 0x2d. */
        if ( (tpoll_token_rate < LMP_MIN_TPOLL) ||
                (tpoll_token_rate > LMP_MAX_TPOLL) )
        {
            if(flow_direction == QOS_SETUP_DUAL_SIDE)
            {
                hci_queue_QoS_complete_event(ce_index, QOS_REJECTED_ERROR);
            }
            else
            {
                /* Generate Flow spec complete event. */
                hci_queue_flow_spec_complete_event(ce_index,
                                                   flow_direction, 
                                                   QOS_REJECTED_ERROR);
            }

            return API_FAILURE;
        }
    }
    else
    {
        if (ce_ptr->token_rate <= hci_get_available_bw())
        {
            tpoll_token_rate =
                (UINT16) hci_calculate_tpoll_token_rate(ce_ptr->token_rate);

            /* If calculated tpoll is not within the sup range,
            then return error 0x2d. */
            if ( (tpoll_token_rate < LMP_MIN_TPOLL) ||
                    (tpoll_token_rate > LMP_MAX_TPOLL) )
            {
                if(flow_direction == QOS_SETUP_DUAL_SIDE)
                {
                    hci_queue_QoS_complete_event(ce_index, QOS_REJECTED_ERROR);
                }
                else
                {
                    /* Generate Flow spec complete event. */
                    hci_queue_flow_spec_complete_event(ce_index,
                                                       flow_direction, 
                                                       QOS_REJECTED_ERROR);
                }

                return API_FAILURE;
            }
        }
    }

    if ( (tpoll_latency != 0xffff) && (tpoll_token_rate != 0xffff) )
    {
        /* Take the min of latency-tpoll and token-rate-tpoll */
        if (tpoll_latency < tpoll_token_rate)
        {
            tpoll_val = tpoll_latency;
        }
        else
        {
            tpoll_val = tpoll_token_rate;
        }
    }

    /* Check if the calculated parameter is out of the allowed range. */
    if (tpoll_val < LMP_MIN_TPOLL)
    {
        tpoll_val = LMP_MIN_TPOLL;
    }
    else if (tpoll_val > LMP_MAX_TPOLL)
    {
        tpoll_val = LMP_MAX_TPOLL;
    }

    /* Tpoll has to be atleast sup-to/8 */
    super_to = ce_ptr->link_supervision_timeout;

    if(flow_direction == QOS_SETUP_DUAL_SIDE)
    {
        tpoll_ptr = & (ce_ptr->qos_tpoll);
    }
    else if(flow_direction == FLOW_SPEC_TX_SIDE)
    {
        tpoll_ptr = & (ce_ptr->flow_tx_tpoll);
    }
    else if(flow_direction == FLOW_SPEC_RX_SIDE)
    {
        tpoll_ptr = & (ce_ptr->flow_rx_tpoll);
    }
    else
    {
        RT_BT_LOG(GRAY, BT_FW_HCI_UTILS_INVALID, 1, flow_direction);
    }

    *tpoll_ptr = tpoll_val;

    if((super_to != 0) &&
            (*tpoll_ptr >= (super_to >> LMP_SUP_TO_TPOLL_DIVISOR ) ) )
    {
        *tpoll_ptr = (UINT16) (super_to >> LMP_SUP_TO_TPOLL_DIVISOR);
    }

    /* Make tpoll even. */
    (*tpoll_ptr)++;
    (*tpoll_ptr) &= (~0x01);

    /* Check if the calculated parameter is out of the allowed range. */
    if (*tpoll_ptr < LMP_MIN_TPOLL)
    {
        *tpoll_ptr = LMP_MIN_TPOLL;
    }
    else if (*tpoll_ptr > LMP_MAX_TPOLL)
    {
        *tpoll_ptr = LMP_MAX_TPOLL;
    }

    return API_SUCCESS;
}

/**
 * Calculates the tpoll based on the tokenrate.
 *
 *
 * \param token_rate The token rate for the connection.
 *
 * \return Tpoll Corresponding Tpoll value for token_rate.
 *
 */
UINT32 hci_calculate_tpoll_token_rate(UINT32 token_rate)
{
    /* Why 180kbps? 723/4. 4 slaves */
    return (180000 / token_rate);
}

/**
* Calculates the tpoll to use, from Qos-setup, flow-tx-side and flow-rx-side.
*
* \param ce_index Index to lmp_connection_entity.
*
* \return Tpoll Corresponding Tpoll value for the connection.
*
*/
UCHAR lmp_calculate_min_tpoll_from_qos_and_flow_spec(UINT16 ce_index)
{
    LMP_CONNECTION_ENTITY *ce_ptr;
    UCHAR qos_setup_tpoll;
    UCHAR flow_tx_tpoll;
    UCHAR flow_rx_tpoll;
    UCHAR qos_tpoll;

    ce_ptr = &lmp_connection_entity[ce_index];

    qos_tpoll = ce_ptr->Tpoll;
    qos_setup_tpoll = ce_ptr->qos_setup_tpoll;
    flow_tx_tpoll = ce_ptr->flow_tx_tpoll;
    flow_rx_tpoll = ce_ptr->flow_rx_tpoll;

    if(qos_setup_tpoll < qos_tpoll)
    {
        qos_tpoll = qos_setup_tpoll;
    }
    if(flow_tx_tpoll < qos_tpoll)
    {
        qos_tpoll = flow_tx_tpoll;
    }
    if(flow_rx_tpoll < qos_tpoll)
    {
        qos_tpoll = flow_rx_tpoll;
    }

    ce_ptr->qos_tpoll = qos_tpoll;

    return API_SUCCESS;
}


/**
 * Queues the QoS complete event to the host.
 *
 * \param ce_index Index to the lmp-connction-entity database.
 * \param error_code The code to be passed to the host.
 *
 * \return None.
 *
 */
void hci_queue_QoS_complete_event(UINT16 ce_index, UCHAR error_code)
{
    OS_SIGNAL sig_send;

    sig_send.type = HCI_GEN_QOS_COMPLETE_EVENT_SIGNAL;
    sig_send.param = (OS_ADDRESS) ((UINT32)ce_index);
    sig_send.ext_param = (OS_ADDRESS)((UINT32) error_code);

    if ( OS_SEND_SIGNAL_TO_TASK (hci_event_task_handle, sig_send)
            != BT_ERROR_OK )
    {
        HCI_LOG_INFO(LOG_LEVEL_LOW, ERROR_SENDING_SIGNAL,0,0);
    }

    return;
}

/**
* Queues the Flow specification complete event to the host.
*
* \param ce_index Index to the lmp-connction-entity database.
* \param flow_direction flow-tx-side or flow-rx-side.
* \param error_code The code to be passed to the host.
*
* \return None.
*
*/
void hci_queue_flow_spec_complete_event(UINT16 ce_index,
                                        UCHAR flow_direction, UCHAR error_code)
{
    OS_SIGNAL sig_send;

    sig_send.type = HCI_GEN_FLOW_SPEC_COMPLETE_EVENT_SIGNAL;
    sig_send.param = (OS_ADDRESS) ((UINT32)ce_index);
    sig_send.ext_param = (OS_ADDRESS) ( (flow_direction << 8) | error_code);

    OS_SEND_SIGNAL_TO_TASK (hci_event_task_handle, sig_send);

    return;
}


/**
 * Returns the available bandwidth.
 *
 * \param None.
 *
 * \return bandwidth The available bandwidth for the connection.
 *
 */
UINT32 hci_get_available_bw()
{
    /* Why 180kbps? 723/4. 4 slaves */
    return (180000);
}

#ifdef COMPILE_SNIFF_MODE
/**
 * Validates the sniff command parameters.
 *
 * \param hci_cmd_ptr received from the host.
 * \param ce_index : Index to the lmp-connction-entity database.
 *
 * \return Success or failure.
 *
 */
UCHAR hci_validate_sniff_params(UCHAR *hci_cmd_ptr, UINT16 ce_index)
{
    UINT16 sniff_max_interval, sniff_min_interval;
    UINT16 sniff_attempt, sniff_timeout ;
    UINT16 super_to;
    UINT32 temp_var;
    LMP_CONNECTION_ENTITY *ce_ptr;

    ce_ptr = &lmp_connection_entity[ce_index];

    super_to = ce_ptr->link_supervision_timeout;

    /* Extract the Sniff Max Interval */
    BT_FW_EXTRACT_16_BITS(sniff_max_interval, &(hci_cmd_ptr[2]));

    /* Extract the Sniff Min Interval */
    BT_FW_EXTRACT_16_BITS(sniff_min_interval, &(hci_cmd_ptr[4]));

    /* Extract the Sniff Attempt value */
    BT_FW_EXTRACT_16_BITS(sniff_attempt, &(hci_cmd_ptr[6]));
    
    /* Extract the Sniff Timeout value */
    BT_FW_EXTRACT_16_BITS(sniff_timeout, &(hci_cmd_ptr[8]));

#ifdef ENABLE_SCO
    /* Check the SCO + Sniff restrictions. but allow hv1 + sniff */
    if ((lc_full_bandwidth_flag == TRUE) && 
        (lmp_self_device_data.sco_pkt_type != HV1))
    {
        return UNSUPPORTED_FEATURE_OR_PARAMETER_ERROR;
    }

#if 0
    /* For every SCO connection, there has to be atleast 4 attempts. */
    if (sniff_attempt < (4 * lmp_self_device_data.total_no_of_sco_conn) )
    {
        return UNSUPPORTED_FEATURE_OR_PARAMETER_ERROR ;
    }
#endif

#endif /* ENABLE_SCO */

    if ((sniff_max_interval < HCI_MIN_VALUE_OF_SNIFF_MAX_SUPPORTED) ||
            (sniff_min_interval < HCI_MIN_VALUE_OF_SNIFF_MIN_SUPPORTED) ||
            (sniff_attempt == 0x0000) ||
            (sniff_max_interval < sniff_min_interval))
    {
        return INVALID_HCI_COMMAND_PARAMETERS_ERROR;
    }

    /* Neither of max and min intervals can be odd. */
    if (((sniff_max_interval | sniff_min_interval) & 0x01) != 0)
    {
        return INVALID_HCI_COMMAND_PARAMETERS_ERROR;
    }

    /* Neither of sniff max or min intervals can be greater than 0x7fff. */
    if (((sniff_max_interval | sniff_min_interval) & (~0x07fff)) != 0)
    {
        return INVALID_HCI_COMMAND_PARAMETERS_ERROR;
    }

    /* Unit of Sniff interval and attempt are different. */
    if(sniff_max_interval < ( (sniff_attempt + sniff_timeout) << 1) )
    {
        return INVALID_HCI_COMMAND_PARAMETERS_ERROR;
    }

    temp_var = sniff_max_interval;

    if (super_to != 0)
    {
        temp_var = super_to - ce_ptr->Tpoll;

#ifdef _SNIFF_DBG_CCH_SNIFF_VAL_	
        if(temp_var>GLOBAL_SLOT_MIN_ALLOW_SNIFF_INT)
            temp_var = temp_var  - temp_var%GLOBAL_SLOT_MIN_ALLOW_SNIFF_INT;
        else
            temp_var = GLOBAL_SLOT_MIN_ALLOW_SNIFF_INT;
#endif 

#ifdef _SNIFF_DBG_CCH_SNIFF_VAL_
	if ( sniff_max_interval == sniff_max_interval)
	{
		temp_var = sniff_min_interval;
	} else 
#endif 
        if ( (temp_var <= sniff_max_interval) &&
                (temp_var >= sniff_min_interval))
        {
#ifdef _SNIFF_DBG_CCH_SNIFF_VAL_
            UINT16 sniff_min_tmp = sniff_min_interval % GLOBAL_SLOT_MIN_ALLOW_SNIFF_INT;
            if (sniff_min_tmp != 0)
            {
                temp_var = sniff_min_interval + GLOBAL_SLOT_MIN_ALLOW_SNIFF_INT - sniff_min_tmp;                
                if (temp_var > sniff_max_interval)
                {
                    temp_var = sniff_min_interval;
                }
            }
            else
#endif 
            {
                temp_var = sniff_min_interval;
            }
        }
        else if(temp_var > sniff_max_interval)
        {
#ifdef _SNIFF_DBG_CCH_SNIFF_VAL_	
            if(sniff_max_interval >GLOBAL_SLOT_MIN_ALLOW_SNIFF_INT)
            {
                temp_var = sniff_max_interval - sniff_max_interval%GLOBAL_SLOT_MIN_ALLOW_SNIFF_INT;
            }
            else
#endif
            {
                temp_var = sniff_max_interval;
            }
        }
        else
        {
            return INVALID_HCI_COMMAND_PARAMETERS_ERROR;
        }
    }

    ce_ptr->sniff_interval = (UINT16)temp_var;

#ifndef _IGNORE_SNIFF_PARAM_LIMIT
    /*
       We need atleast 4 attempts for every 0x1000 slots of Tsniff, and
       a ceiling of 20d.
     */
    if(ce_ptr->sniff_interval > 0x5000)
    {
        if(sniff_attempt < 20)
        {
            return UNSPECIFIED_ERROR;
        }
    }
    else if(sniff_attempt < ((temp_var >> 12) << 2) )
    {
        return UNSPECIFIED_ERROR;
    }
#endif

    return API_SUCCESS;
}
#endif

#ifdef COMPILE_HOLD_MODE
/**
 * Validates the hold command parameters.
 *
 * \param hci_cmd_ptr received from the host.
 * \param ce_index : Index to the lmp-connction-entity database.
 *
 * \return Success or failure.
 *
 */

UCHAR hci_validate_hold_params(UCHAR *hci_cmd_ptr, UINT16 ce_index)
{
    UINT16 hold_max_interval, hold_min_interval ;
    UINT16 hold_mode_interval = 0 ;
    UINT16 super_to;
    LMP_CONNECTION_ENTITY *ce_ptr;

    ce_ptr = &lmp_connection_entity[ce_index];

    BT_FW_EXTRACT_16_BITS(hold_max_interval, &(hci_cmd_ptr[2]));
    BT_FW_EXTRACT_16_BITS(hold_min_interval, &(hci_cmd_ptr[4]));

    if ( (hold_min_interval == 0x0)  ||
            (hold_min_interval > 0xFF00 ) ||
            (hold_max_interval < hold_min_interval) )
    {
        BT_FW_HCI_INF(HOLD_MODE_PARAMTER_CHECKING_FAILED,0,0);
        return INVALID_HCI_COMMAND_PARAMETERS_ERROR;
    }

    /* Neither of max and min interval can be odd. */
    if ( (hold_max_interval | hold_min_interval) & 0x01 )
    {
        BT_FW_HCI_INF(HOLD_MODE_PARAMTER_CHECKING_FAILED,0,0);
        return INVALID_HCI_COMMAND_PARAMETERS_ERROR;
    }

    super_to = ce_ptr->link_supervision_timeout;

    /* Check for super-to violations. */
    if((hold_max_interval > super_to) && (super_to != 0 ))
    {
        if(hold_min_interval > super_to)
        {
            BT_FW_HCI_ERR(HOLD_MIN_INT_LINK_SUPERVISION_TO,0,0);
            return INVALID_HCI_COMMAND_PARAMETERS_ERROR;
        }
        else
        {
            hold_mode_interval = hold_min_interval;
        }
    }
    else
    {
        hold_mode_interval = hold_max_interval;
    }

    if ((super_to != 0) && super_to < (hold_mode_interval + ce_ptr->Tpoll))
    {
        return INVALID_HCI_COMMAND_PARAMETERS_ERROR;
    }

    /*
     * For LMP_hold PDU, hold instant value is atleast 6*Tpoll slots.
     * For LMP_hod_req PDU, hold_instant value is atleast 9*T_poll slots.
     */

    BT_FW_HCI_INF(GIVEN_HOLD_INT_MAX_HOLD_INT_NEG,2,
                  hold_mode_interval, ce_ptr->hold_neg_max_interval);

    ce_ptr->hold_mode_interval = hold_mode_interval ;

    return API_SUCCESS;
}
#endif

#ifdef COMPILE_PARK_MODE

/**
 * Validates the park command parameters.
 *
 * \param hci_cmd_ptr received from the host.
 * \param ce_index : Index to the lmp-connction-entity database.
 *
 * \return Success or failure.
 *
 */

UCHAR hci_validate_park_params(UCHAR *hci_cmd_ptr, UINT16 ce_index)
{
    LMP_CONNECTION_ENTITY *ce_ptr;
    UINT16 min_cal_tb;
    UINT16 local_feature, remote_feature ;
    UINT16 beacon_max, beacon_min;

    ce_ptr = &lmp_connection_entity[ce_index];

    remote_feature = ce_ptr->feat_page0[1];

    local_feature = lmp_feature_data.feat_page0[1];

    /* Check if the local device supports this feature. */
    if((local_feature & LMP_PARK_MODE_FEATURE) == FALSE)
    {
        return UNSUPPORTED_FEATURE_OR_PARAMETER_ERROR;
    }

    /* Check if the remote device supports this feature. */
    if((remote_feature & LMP_PARK_MODE_FEATURE) == FALSE)
    {
        return UNSUPPORTED_REMOTE_FEATURE_ERROR;
    }

#ifdef ENABLE_SCO
    /* Check for any SCO connections on this link. */
    if (ce_ptr->no_of_sco_connections != 0)
    {
        return COMMAND_DISALLOWED_ERROR;
    }
#endif /* ENABLE_SCO */

#ifdef COMPILE_ESCO
    /* Check for any Esco connections on this link. */
#ifdef _BRUCE_IMPLEMENTED_NO_OF_ESCO_CONN_
    if (ce_ptr->no_of_esco_connections != 0)
    {
        return COMMAND_DISALLOWED_ERROR;
    }
#else
    if (lmp_self_device_data.number_of_esco_connections != 0)
    {
        UCHAR temp_var;
        for (temp_var = 0; temp_var < LMP_MAX_ESCO_CONN_ENTITIES; temp_var++)
        {
            if((lmp_esco_connection_entity[temp_var].entity_status == ASSIGNED) &&
                    (lmp_esco_connection_entity[temp_var].ce_index == ce_index) )
            {
                return COMMAND_DISALLOWED_ERROR;
            }
        }
    }
#endif
#endif /* COMPILE_ESCO */

    /* Check if the connection is already in hold mode or sniff mode. */
#ifdef COMPILE_HOLD_MODE
    if (ce_ptr->ce_status == LMP_HOLD_MODE)
    {
        return COMMAND_DISALLOWED_ERROR;
    }
#endif

#ifdef COMPILE_SNIFF_MODE
    if (ce_ptr->in_sniff_mode == TRUE)
    {
        return COMMAND_DISALLOWED_ERROR;
    }
#endif

    if(lc_get_no_of_piconets_connected() != 1)
    {
        RT_BT_LOG(GRAY, BT_FW_HCI_UTILS_1041, 0, 0);
        return COMMAND_DISALLOWED_ERROR;
    }

    /* Extract the Park Mode Max Interval */
    BT_FW_EXTRACT_16_BITS( beacon_max, &(hci_cmd_ptr[2]));

    /* Extract the Park Mode Min Interval */
    BT_FW_EXTRACT_16_BITS(beacon_min, &(hci_cmd_ptr[4]));

    /* Beacon max and min have to be even. */
    beacon_max++;
    beacon_max &= ~0x01;

    beacon_min++;
    beacon_min &= ~0x01;

    /* Verify the relationship between offset, length & total_length */
    if ((beacon_max <  0x0e) || (beacon_min < 0x0e) ||
            (beacon_max < beacon_min))
    {
        return INVALID_HCI_COMMAND_PARAMETERS_ERROR;
    }

    /* Min beacon value supported is 0x200. */
    if ( (beacon_min < otp_str_data.bt_beacon_min_interval) &&
            (beacon_max < otp_str_data.bt_beacon_min_interval) )
    {
        return UNSUPPORTED_FEATURE_OR_PARAMETER_ERROR;
    }

    /* Max beacon value supported is 0x2000. */
    if ( (beacon_max > otp_str_data.bt_beacon_max_interval) &&
            (beacon_min > otp_str_data.bt_beacon_max_interval) )
    {
        return UNSUPPORTED_FEATURE_OR_PARAMETER_ERROR;
    }

    if (beacon_min < otp_str_data.bt_beacon_min_interval)
    {
        beacon_min = otp_str_data.bt_beacon_min_interval;
    }
    if (beacon_max > otp_str_data.bt_beacon_max_interval)
    {
        beacon_max = otp_str_data.bt_beacon_max_interval;
    }

    ce_ptr->beacon_max_interval = beacon_max;
    ce_ptr->beacon_min_interval = beacon_min;

    min_cal_tb = MIN_D_acc + (MIN_M_acc * MIN_T_acc) + MIN_N_poll;

    /*
     * Check if the Max and Min Beacon params from the host is enough
     * to support Park
     */
    if(min_cal_tb > ce_ptr->beacon_max_interval)
    {
        BT_FW_HCI_ERR(HOST_SUPPLIED_MAX_TB_VALUE_IS_NOT_ENOUGH_TO_PARK_A_DEVICE,0,0);
        return INVALID_HCI_COMMAND_PARAMETERS_ERROR;
    }

    return API_SUCCESS;
}

#endif

/**
 * Validates the inquiry command parameters.
 *
 * \param hci_cmd_ptr received from the host.
 *
 * \return Success or failure.
 *
 */
UCHAR hci_validate_inquiry_cmd_params(HCI_CMD_PKT *hci_cmd_ptr)
{
    UINT32 lap;
    UINT16 inquiry_length;

    if (hci_is_full_bandwidth() == TRUE)
    {
        return COMMAND_DISALLOWED_ERROR;
    }

    /* Extract LAP from command packet */
    BT_FW_EXTRACT_24_BITS(lap, hci_cmd_ptr->cmd_parameter);

    /* Validate Inquiry length */
    inquiry_length = hci_cmd_ptr->cmd_parameter[3];

    {
        if ((inquiry_length < HCI_MIN_INQUIRY_LENGTH) ||
            (inquiry_length > HCI_MAX_INQUIRY_LENGTH))
        {
            BT_FW_HCI_ERR(INQUIRY_LENGTH_INVALID,1,inquiry_length);
            return INVALID_HCI_COMMAND_PARAMETERS_ERROR;
        }
    }

    if (!((lap >= LC_INQUIRY_LAP_MIN) && (lap <= LC_INQUIRY_LAP_MAX)))
    {
        BT_FW_HCI_ERR(INQUIRY_LAP_INVALID,1,lap);
        return INVALID_HCI_COMMAND_PARAMETERS_ERROR;
    }

    return API_SUCCESS;
}

/**
* Validates the periodic-inquiry command parameters.
*
* \param hci_cmd_ptr received from the host.
*
* \return Success or failure.
*
*/
UCHAR hci_validate_periodic_inquiry_cmd_params(HCI_CMD_PKT *hci_cmd_ptr)
{
    UINT32 lap;
    UCHAR  inquiry_length;
    UINT16 periodic_inquiry_max_length;
    UINT16 periodic_inquiry_min_length;

    BT_FW_EXTRACT_16_BITS(periodic_inquiry_max_length,
                          &(hci_cmd_ptr->cmd_parameter[0]));

    BT_FW_EXTRACT_16_BITS(periodic_inquiry_min_length,
                          &(hci_cmd_ptr->cmd_parameter[2]));

    /* Extract LAP from command packet */
    BT_FW_EXTRACT_24_BITS(lap,&hci_cmd_ptr->cmd_parameter[4]);

    if ( (lap < LC_INQUIRY_LAP_MIN) || (lap > LC_INQUIRY_LAP_MAX) )
    {
        return INVALID_HCI_COMMAND_PARAMETERS_ERROR;
    }

    /* Extract the Inquiry length */
    inquiry_length = hci_cmd_ptr->cmd_parameter[7];

    /* Validate  Inquiry length */
    if( (inquiry_length < HCI_MIN_INQUIRY_LENGTH ) ||
            (inquiry_length > HCI_MAX_INQUIRY_LENGTH ) )
    {
        return INVALID_HCI_COMMAND_PARAMETERS_ERROR;
    }

    if( (periodic_inquiry_max_length <= periodic_inquiry_min_length) ||
            (periodic_inquiry_min_length <= inquiry_length))
    {
        return INVALID_HCI_COMMAND_PARAMETERS_ERROR ;
    }

    //if(lmp_self_device_data.number_of_acl_conn > 0)
    //{
    //	return COMMAND_DISALLOWED_ERROR;
    //}

    return HCI_COMMAND_SUCCEEDED;
}

/**
* Validates the create_connection command parameters.
*
* \param hci_cmd_ptr received from the host.
*
* \return Success or failure.
*
*/
UCHAR hci_validate_create_connection_cmd_params(HCI_CMD_PKT *hci_cmd_ptr)
{
    UINT16 ce_index ;
    UCHAR index;
    UCHAR temp_bd_addr[LMP_BD_ADDR_SIZE];
    UINT16 temp_packet_type;
    UINT16 temp_page_scan_repetition_mode;
    UINT16 temp_page_scan_mode;
    UINT16 temp_allow_role_switch;
    UINT16 clock_offset ;

    memcpy(temp_bd_addr, &hci_cmd_ptr->cmd_parameter[0], LMP_BD_ADDR_SIZE);
    index = LMP_BD_ADDR_SIZE ;

    if(LMP_GET_CE_INDEX_FROM_BD_ADDR(temp_bd_addr, &ce_index) == API_SUCCESS)
    {
        return ACL_CONNECTION_EXISTS_ERROR ;
    }

    if (hci_is_full_bandwidth() == TRUE)
    {
        return HOST_REJECTED_LIMITED_RESOURCES_ERROR;
    }

    /* Check the Packet type */
    BT_FW_EXTRACT_16_BITS(temp_packet_type,	&(hci_cmd_ptr->cmd_parameter[index]));
    index += 2 ;

    if( (temp_packet_type & ~(ALL_ACL_PKT_TYPES)) != 0)
    {
        return INVALID_HCI_COMMAND_PARAMETERS_ERROR;
    }

    /* Page scan repetition mode */
    temp_page_scan_repetition_mode = (UCHAR) hci_cmd_ptr->cmd_parameter[index];
    index++;

    if(temp_page_scan_repetition_mode > 2)
    {
        return INVALID_HCI_COMMAND_PARAMETERS_ERROR;
    }

    temp_page_scan_mode = (UCHAR) hci_cmd_ptr->cmd_parameter[index];
    index++;

    if(temp_page_scan_mode)
    {
        return INVALID_HCI_COMMAND_PARAMETERS_ERROR;
    }

    BT_FW_EXTRACT_16_BITS(clock_offset, &(hci_cmd_ptr->cmd_parameter[index]));
    index += 2;

    temp_allow_role_switch = hci_cmd_ptr->cmd_parameter[index];

    if (temp_allow_role_switch & (~0x01U))
    {
        return INVALID_HCI_COMMAND_PARAMETERS_ERROR;
    }
    index++;

    return HCI_COMMAND_SUCCEEDED;
}


/**
 * Validates the Switch Role command parameters.
 *
 * \param hci_cmd_ptr received from the host.
 *
 * \return Success or failure.
 *
 */
UCHAR hci_validate_switch_role_params(UCHAR* hci_cmd_ptr)
{
    UINT16 ce_index;
    UCHAR temp_bd_addr[LMP_BD_ADDR_SIZE];
    UCHAR dev_role;
    UCHAR local_feature;
    LMP_CONNECTION_ENTITY *ce_ptr;
    UCHAR ret_val;

    dev_role = hci_cmd_ptr[6] ;

    if (dev_role & (~0x01U) )
    {
        return INVALID_HCI_COMMAND_PARAMETERS_ERROR ;
    }

    memcpy(temp_bd_addr, &hci_cmd_ptr[0], LMP_BD_ADDR_SIZE);

    /*
     * Check the given BD_ADDR device Connected or in park mode.
     * If no such device present return no connection error to the host.
     */
    if(LMP_GET_CE_INDEX_FROM_BD_ADDR(temp_bd_addr, &ce_index) != API_SUCCESS)
    {
        return NO_CONNECTION_ERROR ;
    }

    ce_ptr = &lmp_connection_entity[ce_index];

    local_feature = lmp_feature_data.feat_page0[0];

    /* Check if the local features support Role Switch */
    if((local_feature & LMP_SWITCH_FEATURE) == FALSE)
    {
        return UNSUPPORTED_FEATURE_OR_PARAMETER_ERROR;
    }

    /*
     * Feature Request is done during connection setup.
     * Check the remote device supports role switch
     */
    if((ce_ptr->feat_page0[0] & LMP_SWITCH_FEATURE) == FALSE)
    {
        return UNSUPPORTED_REMOTE_FEATURE_ERROR;
    }

    /* If the connection entity is not in connected state then
     * disallow role switch, this means that either we are not yet
     * connected or we are in some low power mode or in the middle of
     * some other transition
     */
    if(ce_ptr->ce_status != LMP_CONNECTED)
    {
        return SWITCH_NOT_ALLOWED_ERROR ;
    }

    /*
     * If we already have the role which this command is asking to change to
     * reject this command with invalid parameters as error.
     */
    if(((ce_ptr->remote_dev_role == MASTER) && (dev_role == SLAVE)) ||
            ((ce_ptr->remote_dev_role == SLAVE) && (dev_role == MASTER)))
    {
        LMP_LOG_INFO(LOG_LEVEL_HIGH,ROLE_SWITCH_INVALID_HCI_PARAMETER, 0, 0);

        return COMMAND_DISALLOWED_ERROR;
    }

    ret_val = lmp_check_for_role_switch_in_scatternet(ce_index);
    if(ret_val != TRUE)
    {
        return UNSPECIFIED_ERROR;
    }

    return API_SUCCESS;
}

EIR_HEADER *eir_header_find(void *buf, UINT32 buf_size, UINT8 type)
{
    UCHAR *ptr = (UCHAR *) buf;
    UINT32 i = 0;
    while (i < buf_size)
    {
        EIR_HEADER *header = (EIR_HEADER *) &ptr[i];
        if (header->type == type)
        {
            return header;
        }
        i += EIR_DATA_SIZE(header);
    }
    return NULL;
}

