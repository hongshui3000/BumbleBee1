/***************************************************************************
 Copyright (C) Mindtree Ltd.
 This module is a confidential and proprietary property of MindTree and
 a possession or use of this module requires written permission of MindTree.
 ***************************************************************************/

enum { __FILE_NUM__= 57 };
/********************************* Logger *************************/

/* ========================= Include File Section ========================= */
#include "lmp_internal.h"
#include "hci_vendor_internal.h"
#include "bz_debug.h"
#include "bt_fw_hci_2_1.h"
#include "mem.h"
#include "bzdma.h"
#include "bz_auth_internal.h"
/* ==================== Structure declaration Section ===================== */



/* ===================== Variable Declaration Section ===================== */
#ifdef COMPILE_ESCO

/** Esco Conn handle to eSCO CE index mapping table */
LMP_ESCO_CONN_HANDLE_TO_CE_INDEX_TABLE lmp_esco_ch_to_ce_index_table[LMP_MAX_ESCO_CONN_HANDLES];

/** Esco connection entities table */
LMP_ESCO_CONNECTION_ENTITY lmp_esco_connection_entity[LMP_MAX_ESCO_CONN_ENTITIES];

/** Esco connection entity used while changing the existing esco link */
LMP_ESCO_CONNECTION_ENTITY temp_esco_connection_entity;

/**
 * Indicates whether Local device initiated the esco parameter change or not.
 */
UCHAR lmp_local_dev_init_esco_param_change = FALSE;

/** Holds the Maximum value of Esco interval among all links */
UCHAR max_tesco = 0;

/** Holds the Gretest common divisor of the Esco intervals */
UCHAR gcd_of_tesco = 0;

/** Denotes the number of esco links created over codec */
UCHAR num_sync_links_over_codec = 0;

/** Denotes weather the esco links have to be created over codec */
UCHAR lmp_esco_over_codec = TRUE;

/** Holds the negotiation history */
LMP_ESCO_NEGOTIATION_HISTORY esco_negotiation_history;

#endif /* COMPILE_ESCO */

#ifdef _BRUCE_MSBC_IS_RESTRICTED_PKT_TYPE 
UCHAR  g_air_mode=0xFF;
UINT16 g_voice_setting=0xFF;
#endif

/* ================== Static Function Prototypes Section ================== */


/* ================== For rom code patch function point ================== */
#ifdef _ROM_CODE_PATCHED_
#ifdef _CCH_8723_RCP_
PF_ROM_CODE_PATCH_FUNC rcp_lmp_validate_esco_link_req_pdu_start = NULL;
#endif

#ifdef _CCH_NEW_SPEC_RCP_
PF_ROM_CODE_PATCH_FUNC rcp_lmp_create_esco_connection = NULL;
#endif
#endif

/* ===================== Function Definition Section ====================== */
#ifdef COMPILE_ESCO
/**
 * Auto accept a ESCO connection.
 *
 * \param ce_index ACL connection entity index.
 * \param esco_ce_index ESCO connection entity index.
 *
 * \return BT_FW_SUCCESS, if the operation is successful. BT_FW_ERROR/An error
 *         code, otherwise.
 */
UCHAR lmp_accept_esco_connection(UINT16 ce_index,UINT16 esco_ce_index)
{
    UINT16 esco_ce_ind = 0;
    UCHAR parameter_list[LMP_PDU_MAX_LEN];
    UCHAR esco_link_req_response = 0;
    UCHAR reason;
    UCHAR feature_byte_4 = 0;
    LMP_ESCO_CONNECTION_ENTITY *esco_ce_ptr;

    esco_ce_ptr = &lmp_esco_connection_entity[esco_ce_index];

    /* Proceed with ESCO negotiation after filling the data structures
       Policy : We'll accept whatever the other side asks for except
       air_mode as CVSD for time being. Is some problem is there then
       the negotiation would fail, which would be anyway taken care
     */

    esco_ce_ptr->status = ADDING_ESCO_LINK;
    /* Get host params from esco_link_req pdu */
    if(lmp_connection_entity[ce_index].remote_dev_role == MASTER)
    {
        esco_ce_ptr->tx_bandwidth =
            (1600*esco_ce_ptr->s_to_m_packet_length)/(esco_ce_ptr->tesco);
        esco_ce_ptr->rx_bandwidth =
            (1600*esco_ce_ptr->m_to_s_packet_length)/(esco_ce_ptr->tesco);
    }
    else
    {
        esco_ce_ptr->tx_bandwidth =
            (1600*esco_ce_ptr->m_to_s_packet_length)/(esco_ce_ptr->tesco);
        esco_ce_ptr->rx_bandwidth =
            (1600*esco_ce_ptr->s_to_m_packet_length)/(esco_ce_ptr->tesco);
    }

    /* For auto accept of ESCO link our policy is Don't care for max_latency
       and retransmission effort.... we'll do whatever the other side asks for.
       This doesnt mean that ultimately the ESCO link will also be established
       with the same params... negotiation can still fail due to other violations
       or unsupported parameters */
    esco_ce_ptr->retransmission_effort = 0xFF;
    esco_ce_ptr->max_latency = 0xFFFF;

    /* Allow ESCO packet types according to feature
       Note: Take care of SCO, EDR using accept_synchronous when they're implemented*/

    feature_byte_4 = lmp_feature_data.feat_page0[4];

    esco_ce_ptr->packet_type = HCI_EV3;
    if(feature_byte_4 & LMP_ESCO_EV4_PACKET_FEATURE)
    {
        esco_ce_ptr->packet_type |= HCI_EV4;
    }
    if(feature_byte_4 & LMP_ESCO_EV5_PACKET_FEATURE)
    {
        esco_ce_ptr->packet_type |= HCI_EV5;
    }

    /* Spec: While eSCO link is auto accepted, voice setting parameters would be
             whatever is written in using WRITE_VOICE_SETTINGS command */

#if 0 /* Now our policy is not to check for Invalid voice settings for Interop */
    if( lmp_self_device_data.voice_setting  == INVALID_VOICE_SETTINGS)
#endif
    {
        /* Policy: If we've not written anything. Hence we'll consider what air_mode the other
            side asks for & input coding will be linear... Should we see in other esco connection
            entities for input_coding so that we can use that??? */
        if(lmp_self_device_data.number_of_esco_connections)
        {
            while(esco_ce_ind < LMP_MAX_ESCO_CONN_ENTITIES)
            {
                if(lmp_esco_connection_entity[esco_ce_ind].status == ESCO_CONNECTED)
                {
                    /* Take everything except air_mode from existing connection */
                    esco_ce_ptr->voice_setting = (UINT16)
                                                 ((lmp_esco_connection_entity[esco_ce_ind].voice_setting & 0xFFFC) |
                                                  (esco_ce_ptr->air_mode));
                }
                esco_ce_ind++;
            }
        }
        if ((esco_ce_ind == LMP_MAX_ESCO_CONN_ENTITIES))
        {
            parameter_list[0] = LMP_ESCAPE4_OPCODE;
            parameter_list[2] = LMP_NOT_ACCEPTED_EXT_OPCODE;
            parameter_list[3] = LMP_ESCAPE4_OPCODE;
            parameter_list[4] = LMP_ESCO_LINK_REQ_OPCODE;
            parameter_list[5] = PDU_NOT_ALLOWED_ERROR;
            if (lmp_generate_pdu(ce_index, parameter_list,
                                 LMP_NOT_ACCEPTED_EXT_LEN,
                                 REMOTE_DEV_TID, lmp_connection_entity
                                 [ce_index].temp_ce_status)
                    != API_SUCCESS)
            {
                ESCO_ERR(LMP_GENERATE_PDU_FAILED,0,0);
                return BT_FW_ERROR;
            }

            lmp_handle_esco_conn_setup_failed(esco_ce_index,
                                              ce_index,PDU_NOT_ALLOWED_ERROR);
            return PDU_NOT_ALLOWED_ERROR;

        }
    }

    lmp_validate_new_esco_link_req(esco_ce_index,ce_index,
                                   &esco_link_req_response, &reason);

    /*
       Check if the new set of parameters will result in esco reserved
       slots occupying the whole bandwidth.
    */
    if (lmp_check_sync_conn_bandwidth(FALSE) == API_FAILURE)
    {
        ESCO_ERR(NO_BANDWIDTH_TO_ACCEPT_NEW_ESCO_CONNECTION,0,0);
        esco_link_req_response = LMP_NOT_ACCEPTED_EXT_OPCODE;
        reason = UNSUPPORTED_PARAMETER_VALUE_ERROR;
    }
    switch(esco_link_req_response)
    {
        case LMP_ACCEPTED_EXT_OPCODE :
        {
            ESCO_INF(SENDING_ACCEP_FOR_ESCO_LINK_REQ,0,0);
            ESCO_INF(LT_ADDR,1,lmp_esco_connection_entity[esco_ce_index].lt_addr);
            parameter_list[0] = LMP_ESCAPE4_OPCODE;
            parameter_list[2] = LMP_ACCEPTED_EXT_OPCODE;
            parameter_list[3] = LMP_ESCAPE4_OPCODE;
            parameter_list[4] = LMP_ESCO_LINK_REQ_OPCODE;
            if (lmp_generate_pdu(ce_index, parameter_list,
                                 LMP_ACCEPTED_EXT_LEN,
                                 REMOTE_DEV_TID, LMP_NO_STATE_CHANGE)
                    != API_SUCCESS)
            {
                ESCO_ERR(LMP_GENERATE_PDU_FAILED,0,0);
                return BT_FW_ERROR;
            }
        }
        break;

        case LMP_NOT_ACCEPTED_EXT_OPCODE:
        {
            parameter_list[0] = LMP_ESCAPE4_OPCODE;
            parameter_list[2] = LMP_NOT_ACCEPTED_EXT_OPCODE;
            parameter_list[3] = LMP_ESCAPE4_OPCODE;
            parameter_list[4] = LMP_ESCO_LINK_REQ_OPCODE;
            parameter_list[5] = reason;
            if (lmp_generate_pdu(ce_index, parameter_list,
                                 LMP_NOT_ACCEPTED_EXT_LEN,
                                 REMOTE_DEV_TID, lmp_connection_entity
                                 [ce_index].temp_ce_status)
                    != API_SUCCESS)
            {
                ESCO_ERR(LMP_GENERATE_PDU_FAILED,0,0);
                return BT_FW_ERROR;
            }

            lmp_handle_esco_conn_setup_failed(esco_ce_index,
                                              ce_index,reason);
            return reason;
        }

        case LMP_ESCO_LINK_REQ_OPCODE :
        {
            lmp_get_esco_params_from_ce(parameter_list,
                                        esco_ce_index);
            if (lmp_generate_pdu(ce_index, parameter_list,
                                 LMP_ESCO_LINK_REQ_LEN,
                                 REMOTE_DEV_TID,
                                 LMP_NO_STATE_CHANGE)
                    != API_SUCCESS)
            {
                ESCO_ERR(LMP_GENERATE_PDU_FAILED,0,0);
                return BT_FW_ERROR;
            }
            esco_ce_ptr->status = ADDING_ESCO_LINK;
            esco_ce_ptr->negotiation_count++;
        }
        break;

        default:
        {
            ESCO_ERR(INVALID_RESPONSE_PDU,1, esco_link_req_response);
            return PDU_NOT_ALLOWED_ERROR;
        }
    }

    return BT_FW_SUCCESS;

}

/**
 * Check the incoming esco link request pdu and validates the parameters. It
 * also generates the connection request to the host.
 *
 * \param lmp_pdu_ptr The pointer to the LMP PDU packet.
 * \param ce_index ACL connection entity index.
 *
 * \return BT_FW_SUCCESS, if the operation is successful. BT_FW_ERROR/An error
 *         code, otherwise.
 */
UCHAR lmp_handle_esco_link_req_pdu(LMP_PDU_PKT *lmp_pdu_ptr, UINT16 ce_index)
{
    UCHAR esco_handle,lt_addr,tc_flags,desco,tesco,wesco,mtos_pkt_type,
          stom_pkt_type,air_mode,negotiation_state,esco_window=0;
    UCHAR parameter_list[LMP_PDU_MAX_LEN];
    UCHAR response_pdu,reason,transaction_id;
    UCHAR bd_addr[LMP_BD_ADDR_SIZE];
    UINT16 mtos_pkt_length,stom_pkt_length,esco_ce_index=0;
    UINT32 class_of_device;
    UCHAR feature_byte_3=0;
    UINT16 connection_accept_to;
    TimerHandle_t host_timeout_timer_handle;
#ifdef _DAPE_TEST_BLOCK_ACL_2_SLOTS_WHEN_SCO
    UINT16 bb_reg;
#endif
    LMP_CONNECTION_ENTITY *ce_ptr;
    LMP_ESCO_CONNECTION_ENTITY *esco_ce_ptr;

    ESCO_INF(ESCO_LINK_REQ_PDU_RCVD,0,0);

    ce_ptr = &lmp_connection_entity[ce_index];

    transaction_id = (UCHAR)(lmp_pdu_ptr->payload_content[0] & 0x01);

    if(ce_ptr->remote_dev_role != transaction_id)
    {
        ESCO_INF(SELF_DEVICE_INITIATED_THE_TRANSACTION,0,0);
    }
    else
    {
        ESCO_INF(REMOTE_DEVICE_DEVICE_INITIATED_THE_TRANSACTION,0,0);
    }

    feature_byte_3 = lmp_feature_data.feat_page0[3];

    if((feature_byte_3 & LMP_ESCO_MANDATORY_FEATURE)== FALSE)
    {
        lmp_send_lmp_not_accepted_ext(ce_index,LMP_ESCAPE4_OPCODE,
                                      LMP_ESCO_LINK_REQ_OPCODE,
                                      transaction_id,
                                      UNSUPPORTED_REMOTE_FEATURE_ERROR);
        ESCO_ERR(AIR_MODE_NOT_SUPPORTED, 2,air_mode,air_mode);
        return BT_FW_SUCCESS;
    }
#ifdef _SUPPORT_VER_4_1_
#ifdef _SUPPORT_PCA_ADJUST
    if (IS_BT41)
    {
        if (bt_pca_manager.pca_updating)
        {
            lmp_send_lmp_not_accepted_ext(ce_index,LMP_ESCAPE4_OPCODE,
                                          LMP_ESCO_LINK_REQ_OPCODE,
                                          transaction_id,PDU_NOT_ALLOWED_ERROR);
        }    
    }
#endif
#endif

    /* Check the lmp connection entity status */
    switch(ce_ptr->ce_status)
    {
        case LMP_CONNECTED :
            break;

#ifdef COMPILE_SNIFF_MODE
        case LMP_SNIFF_MODE :
#endif
        case LMP_ADDING_ESCO_LINK :
        case LMP_CHANGING_ESCO_PARAMS:
            break ;

        default :
            lmp_send_lmp_not_accepted_ext(ce_index,LMP_ESCAPE4_OPCODE,
                                          LMP_ESCO_LINK_REQ_OPCODE,
                                          transaction_id,PDU_NOT_ALLOWED_ERROR);

            ESCO_ERR(PDU_RCVD_IN_INVALID_ACL_CE_STATUS, 1, lmp_connection_entity[ce_index].ce_status);
            return BT_FW_SUCCESS;
    }

    if (lmp_self_device_data.total_no_of_sco_conn)
    {
        lmp_send_lmp_not_accepted_ext(ce_index, LMP_ESCAPE4_OPCODE,
                                      LMP_ESCO_LINK_REQ_OPCODE,
                                      transaction_id,
                                      HOST_REJECTED_LIMITED_RESOURCES_ERROR);
        return BT_FW_SUCCESS;
    }

    /* Get the parameters from the esco link request pdu */
    esco_handle = lmp_pdu_ptr->payload_content[2];
    lt_addr = lmp_pdu_ptr->payload_content[3];
    tc_flags = lmp_pdu_ptr->payload_content[4];
    desco = lmp_pdu_ptr->payload_content[5];
    tesco = lmp_pdu_ptr->payload_content[6];
    wesco = lmp_pdu_ptr->payload_content[7];
    mtos_pkt_type = lmp_pdu_ptr->payload_content[8];
    stom_pkt_type = lmp_pdu_ptr->payload_content[9];
    mtos_pkt_length = lmp_pdu_ptr->payload_content[10] |
                    (lmp_pdu_ptr->payload_content[11] << 8);
    stom_pkt_length = lmp_pdu_ptr->payload_content[12] |
                    (lmp_pdu_ptr->payload_content[13] << 8);
    air_mode = lmp_pdu_ptr->payload_content[14];
    negotiation_state = lmp_pdu_ptr->payload_content[15];

    //{{added by guochunxia
    lmp_self_device_data.voice_setting &= (~AIR_MODE_MASK);
    lmp_self_device_data.voice_setting |= (lmp_convert_air_mode(air_mode, LMP_LAYER, HCI_LAYER));
    lmp_self_device_data.lmp_air_mode =  air_mode;
//}}

#ifdef _BRUCE_MSBC_IS_RESTRICTED_PKT_TYPE 
    if (lmp_esco_over_codec == TRUE)
    {  
        //remote device no sup msbc
        if(air_mode != 0x03)
        {
            g_voice_setting =0xFF;
        }
    }
#endif

    /**
     * Find the number of slots occupied by the master to slave
     *  packet type
     */
    esco_window = (UCHAR)(esco_window +
                          (UCHAR)lmp_find_packet_slots((UINT16) mtos_pkt_type));

    /**
     * Find the number of slots occupied by the slave to master
     *  packet type
     */
    esco_window = (UCHAR)(esco_window +
                          (UCHAR)lmp_find_packet_slots((UINT16) stom_pkt_type));

    /* Esco window size */
    esco_window = (UCHAR)(esco_window + wesco);

    ESCO_TRC(ESCO_HANDLE,1,esco_handle);
    ESCO_TRC(ESCO_LT_ADDR,1,lt_addr);
    ESCO_TRC(ESCO_WINDOW_1,1,esco_window);

    while(esco_ce_index < LMP_MAX_ESCO_CONN_ENTITIES)
    {
        if((lmp_esco_connection_entity[esco_ce_index].ce_index == ce_index)
                && ((lmp_esco_connection_entity[esco_ce_index].status ==
                     ADDING_ESCO_LINK)
                    || (lmp_esco_connection_entity[esco_ce_index].status ==
                        CHANGING_ESCO_PARAMS)))
        {
            break;
        }
        esco_ce_index++;
    }

    if(esco_ce_index != LMP_MAX_ESCO_CONN_ENTITIES)
    {
        /* This assignment is done when the local device which is slave
           initiates esco link creation and the master replies with
           esco link req pdu having esco handle. This is required to find
           if the current pdu is for updating the existing link
           or for creating a new link.
        */
        if(ce_ptr->remote_dev_role == MASTER)
        {
            lmp_esco_connection_entity[esco_ce_index].esco_handle =
                esco_handle;
        }
    }

    /* Check if the esco handle is already used. If it is not used then
       it implies remote device is initiating a new esco connection.
    */
    if(lmp_get_esco_ce_index_from_esco_handle(esco_handle,&esco_ce_index)
            != API_SUCCESS)
    {
        UCHAR ret_error;
        ret_error = lmp_check_for_allowing_new_esco_connection();

        if(ret_error != HCI_COMMAND_SUCCEEDED)
        {
            lmp_send_lmp_not_accepted_ext(ce_index, LMP_ESCAPE4_OPCODE,
                                          LMP_ESCO_LINK_REQ_OPCODE,
                                          transaction_id,
                                          HOST_REJECTED_LIMITED_RESOURCES_ERROR);

            return BT_FW_SUCCESS;
        }

        /* Remote device is initiating for establishing a new esco link.
           If the number_of_esco_connections is more than the Max_allowed
           Esco connections then send lmp_not_accepted_ext pdu.
        */
        if( lmp_self_device_data.number_of_esco_connections >= LMP_MAX_ESCO_CONNECTIONS)
        {

            lmp_send_lmp_not_accepted_ext(ce_index,LMP_ESCAPE4_OPCODE,
                                          LMP_ESCO_LINK_REQ_OPCODE,
                                          transaction_id,
                                          MAX_SCO_CONNECTIONS_REACHED_ERROR);

            ESCO_ERR(DEVICE_HAS_REACHED_MAX_OF_ESCO_CONNECTIONS,0,0);
            return BT_FW_SUCCESS;
        }

        ESCO_INF(NEG_STARTED_FOR_NEW_ESCO_LINK,0,0);
        /* Remote device is starting neg. for new esco */
        switch(ce_ptr->ce_status)
        {
            case LMP_CONNECTED:
#ifdef COMPILE_SNIFF_MODE
            case LMP_SNIFF_MODE:
#endif
                /* this implies remote device is initiating for a new esco
                   connection
                */
                if(ce_ptr->remote_dev_role == SLAVE)
                {
                    if(lmp_allocate_esco_am_addr(&lt_addr, &esco_ce_index,
                                                 ce_ptr->phy_piconet_id) == API_FAILURE)
                    {
                        ESCO_ERR(ALLOCATE_AM_ADDRESS_FAILED,0,0);
                    }

                    esco_ce_ptr = &lmp_esco_connection_entity[esco_ce_index];

                    esco_ce_ptr->lt_addr =lt_addr;
                }
                else /* if remote device is master */
                {
                    /* allocate esco connection entity */
                    if((esco_ce_index =
                                lmp_allocate_esco_entity_from_ce_database())
                            != BT_FW_ERROR)
                    {
                        ESCO_INF(ALLOCATED_CE_INDEX,1,esco_ce_index);
                        lmp_slave_use_esco_am_addr_ppi(lt_addr,
                                                       ce_ptr->phy_piconet_id, esco_ce_index);
                    }
                    else
                    {
                        ESCO_ERR(ALLOC_OF_CONN_ENTITY_FAILED,0,0);
                    }

                    esco_ce_ptr = &lmp_esco_connection_entity[esco_ce_index];

                    esco_ce_ptr->lt_addr = lt_addr;
                    esco_ce_ptr->esco_handle= esco_handle;
                }

                lmp_connection_entity[ce_index].is_esco_channel = TRUE;
                lmp_connection_entity[ce_index].esco_ce_idx = esco_ce_index;

                /* Update the esco connection entities */
                esco_ce_ptr->timing_control_flags = tc_flags;
                esco_ce_ptr->ce_index = ce_index;
                esco_ce_ptr->desco = desco;
                esco_ce_ptr->tesco = tesco;
                esco_ce_ptr->wesco = wesco;
                esco_ce_ptr->m_to_s_packet_type = mtos_pkt_type;
                esco_ce_ptr->s_to_m_packet_type = stom_pkt_type;

                esco_ce_ptr->m_to_s_packet_length = mtos_pkt_length;
                esco_ce_ptr->s_to_m_packet_length = stom_pkt_length;
                esco_ce_ptr->air_mode = air_mode;
                esco_ce_ptr->negotiation_flag = negotiation_state;
                esco_ce_ptr->esco_window = esco_window;

                ce_ptr->temp_ce_status = ce_ptr->ce_status;

                lmp_set_ce_status(ce_index, LMP_ADDING_ESCO_LINK);

                esco_ce_ptr->status = WAITING_FOR_ACCEPT_SYNC_CONN;

                memcpy(bd_addr, ce_ptr->bd_addr, LMP_BD_ADDR_SIZE);

                /* Generating the Connection request event for the esco
                   link establishment. */

                /* Fill in the class of device parameter */
                class_of_device = ce_ptr->class_of_device;
                connection_accept_to = lmp_self_device_data.conn_accept_timeout;
                switch(hci_pass_event_through_event_filter(
                            CONNECTION_SETUP_FILTER,bd_addr, class_of_device))
                {
                    case HCI_CONNECTION_REQUEST_EVENT:
                        /* Send a request to the host */
                        if (!hci_generate_connection_request_event(bd_addr,
                                class_of_device, ESCO_LINK))
                        {
                            connection_accept_to = 0x1;
                        }
                        break;

                    case HCI_CONNECTION_COMPLETE_EVENT:
                    case HCI_CONNECTION_COMPLETE_EVENT_WITH_RS:
                        /* Auto-Accept the connection */
#ifdef _DAPE_TEST_BLOCK_ACL_2_SLOTS_WHEN_SCO
                         bb_reg = BB_read_baseband_register(PICONET4_INFO_REGISTER);
                         bb_reg |= (UINT16)(0x0040);
                         BB_write_baseband_register(PICONET4_INFO_REGISTER, bb_reg);
#endif						
#ifdef _DAPE_TEST_SEND_MAX_SLOT_BEFORE_ESCO_CREATED
                        lmp_self_device_data.adding_new_esco_conn = TRUE;
                        lmp_send_max_slot_pdu_to_all_devices();
#endif				
                        lmp_accept_esco_connection(ce_index,esco_ce_index);
                        return BT_FW_SUCCESS;

                    default:
                    case FALSE:
                        /* Not accepted */
                        lmp_send_lmp_not_accepted_ext(ce_index,
                                                      LMP_ESCAPE4_OPCODE,
                                                      LMP_ESCO_LINK_REQ_OPCODE, REMOTE_DEV_TID,
                                                      HOST_REJECTED_SECURITY_REASONS_ERROR);

                        lmp_put_esco_am_addr(
                            esco_ce_ptr->lt_addr,
                            ce_ptr->phy_piconet_id);

                        lmp_release_esco_entity_to_ce_database(
                            esco_ce_index);
                        return BT_FW_SUCCESS;
                }

                /* Start a Connection Accept Timeout timer here. If the
                   timer fires then cause an interrupt and handle it to
                   generate an LMP_not_accepted pdu with the reason
                   HOST_TIMEOUT */

                if(OS_CREATE_TIMER(ONESHOT_TIMER,
                        &ce_ptr->conn_accept_timer_handle,
                        lmp_sync_conn_accept_timeout_handler,
                        (void *)((UINT32)esco_ce_ptr->conn_handle), 0) != BT_ERROR_OK)
                {
                    ESCO_ERR(HOST_TO_TIMER_CREATION_FAILED,0,0);
                }

                host_timeout_timer_handle = ce_ptr->conn_accept_timer_handle;

                ESCO_INF(GENERATING_SYNC_CONN_REQ_EVENT,0,0);
                ESCO_INF(STARTING_CONNECTION_ACCEPT_TIMER,0,0);
                if(OS_START_TIMER(host_timeout_timer_handle,
                                  connection_accept_to)
                        != BT_ERROR_OK)
                {
                    ESCO_ERR(OS_START_TIMER_FAILED,0,0);
                }
                return BT_FW_SUCCESS;

            default :
                lmp_send_lmp_not_accepted_ext(ce_index,LMP_ESCAPE4_OPCODE,
                                              LMP_ESCO_LINK_REQ_OPCODE,
                                              transaction_id,
                                              PDU_NOT_ALLOWED_ERROR);

                ESCO_ERR(ESCO_LINK_REQ_PDU_RCVD_IN_INAVLID_STATE_SENDING_NOT_ACC_EXT,0,0);
                ESCO_ERR(CE_STATUS,1,lmp_connection_entity[ce_index].ce_status);

                return BT_FW_SUCCESS;
        }
    }
    else
    {
        /* This implies the pdu received for an ongoing esco link
           negotiation or for changing an existing esco link
        */

        esco_ce_ptr = &lmp_esco_connection_entity[esco_ce_index];

        switch(esco_ce_ptr->status)
        {

            case ADDING_ESCO_LINK:
                /*
                  Moved it here from the end of this function
                  LMP_ESCO_LINK_REQ pdu
                 */
                if(esco_ce_ptr->negotiation_count >=
                        LMP_MAX_NUM_OF_ESCO_NEGOTIATIONS)
                {
                    response_pdu = LMP_NOT_ACCEPTED_EXT_OPCODE;
                    reason = UNSPECIFIED_ERROR;
                    break;
                }
                else
                {
                    esco_ce_ptr->negotiation_count++;
                }

                lmp_store_negotiation_history(esco_ce_ptr, lmp_pdu_ptr);

                /* Validate the esco link req pdu received from the remote
                   device. If the parameters have to be changed then the
                   esco ce or temp ce  will be updated  with the new
                   parameters suggested by the local deivce
                */
                lmp_validate_esco_link_req_pdu(lmp_pdu_ptr,esco_ce_index,
                                               ce_index, &response_pdu,&reason);
                /* Check if the new set of parameters will result in esco
                   reserved slots occupying the whole bandwidth.
                */
                if(lmp_check_sync_conn_bandwidth(FALSE) == API_FAILURE)
                {
                    response_pdu = LMP_NOT_ACCEPTED_EXT_OPCODE;
                    reason = UNSUPPORTED_PARAMETER_VALUE_ERROR;
                }

                if(ce_ptr->remote_dev_role == MASTER)
                {
                    /* This is a case where the slave self-device is
                       negotiating for parameters with master. LT_addr and
                       Esco_handle have to updated in the esco connection
                       entity.
                    */

                    esco_ce_ptr->lt_addr = lt_addr;
                    esco_ce_ptr->esco_handle = esco_handle;
                    lmp_slave_use_esco_am_addr_ppi(lt_addr,
                                                   ce_ptr->phy_piconet_id, esco_ce_index);
                }
                break;

            case CHANGING_ESCO_PARAMS:
                /* If the negotiation count exceeds
                   LMP_MAX_NUM_OF_ESCO_NEGOTIATIONS then end the
                   transaction by sending not_accepted pdu to the remote
                   device.
                 */
                if(temp_esco_connection_entity.negotiation_count
                        >= LMP_MAX_NUM_OF_ESCO_NEGOTIATIONS)
                {
                    response_pdu = LMP_NOT_ACCEPTED_EXT_OPCODE;
                    reason = UNSPECIFIED_ERROR;
                    break;
                }
                else
                {
                    temp_esco_connection_entity.negotiation_count++;
                }

                lmp_store_negotiation_history(
                    &temp_esco_connection_entity, lmp_pdu_ptr);

                /* Validate the esco link req pdu received from the remote
                   device. If the parameters have to be changed then the
                   esco ce or temp ce  will be updated  with the new
                   parameters suggested by the local deivce
                */
                lmp_validate_esco_link_req_pdu(lmp_pdu_ptr,
                                               esco_ce_index, ce_index, &response_pdu, &reason);

                /* Check if the new set of parameters will result in esco
                   reserved slots occupying the whole bandwidth.
                */
                if(lmp_check_sync_conn_bandwidth(FALSE) == API_FAILURE)
                {
                    response_pdu = LMP_NOT_ACCEPTED_EXT_OPCODE;
                    reason = UNSUPPORTED_PARAMETER_VALUE_ERROR;
                }

                if(ce_ptr->remote_dev_role==MASTER)
                {
                    /* This is a case where the slave self-device is
                       negotiating for parameters with master. LT_addr and
                       Esco_handle have to updated in the esco connection
                       entity.
                    */

                    esco_ce_ptr->lt_addr = lt_addr;
                    esco_ce_ptr->esco_handle = esco_handle;
                }
                break;

            case ESCO_CONNECTED:
                /* Negotiation for changing an existing esco link is
                   initiated by the remote device
                */
                ESCO_INF(REM_DEV_INITIATES_CHANGE_IN_ESCO_LINK_PARAMS_ON_ESCO_CE_IDX,1,esco_ce_index);
                temp_esco_connection_entity.lt_addr = lt_addr;
                temp_esco_connection_entity.esco_handle= esco_handle;
                temp_esco_connection_entity.timing_control_flags =tc_flags;
                temp_esco_connection_entity.ce_index = ce_index;
                temp_esco_connection_entity.desco = desco;
                temp_esco_connection_entity.tesco = tesco;
                temp_esco_connection_entity.wesco = wesco;
                temp_esco_connection_entity.m_to_s_packet_type =
                    mtos_pkt_type;
                temp_esco_connection_entity.s_to_m_packet_type =
                    stom_pkt_type;
                temp_esco_connection_entity.m_to_s_packet_length =
                    mtos_pkt_length;
                temp_esco_connection_entity.s_to_m_packet_length =
                    stom_pkt_length;
                temp_esco_connection_entity.air_mode = air_mode;
                temp_esco_connection_entity.negotiation_flag =
                    negotiation_state;
                temp_esco_connection_entity.esco_window = esco_window;

                /*
                   Copy the local device's supported values for the esco
                   link to the temp_esco_connection_entity
                */

                temp_esco_connection_entity.tx_bandwidth =
                    esco_ce_ptr->tx_bandwidth;
                temp_esco_connection_entity.rx_bandwidth =
                    esco_ce_ptr->rx_bandwidth;
                temp_esco_connection_entity.max_latency =
                    esco_ce_ptr->max_latency;
                temp_esco_connection_entity.voice_setting =
                    esco_ce_ptr->voice_setting;
                temp_esco_connection_entity.retransmission_effort =
                    esco_ce_ptr->retransmission_effort;
                temp_esco_connection_entity.packet_type =
                    esco_ce_ptr->packet_type;

                ce_ptr->temp_ce_status = ce_ptr->ce_status;

                lmp_set_ce_status(ce_index, LMP_CHANGING_ESCO_PARAMS);

                esco_ce_ptr->status = CHANGING_ESCO_PARAMS;
                /* Validate the esco link req pdu received from the remote
                   device. If the parameters have to be changed then the
                   esco ce or temp ce  will be updated  with the new
                   parameters suggested by the local deivce
                */
                lmp_validate_esco_link_req_pdu(lmp_pdu_ptr,esco_ce_index,
                                               ce_index,&response_pdu,&reason);
                /* Check if the new set of parameters will result in esco
                   reserved slots occupying the whole bandwidth.
                */
                if(lmp_check_sync_conn_bandwidth(FALSE) == API_FAILURE)
                {
                    response_pdu = LMP_NOT_ACCEPTED_EXT_OPCODE;
                    reason = UNSUPPORTED_PARAMETER_VALUE_ERROR;
                }
                break;

            default:
                response_pdu = LMP_NOT_ACCEPTED_EXT_OPCODE;
                reason = PDU_NOT_ALLOWED_ERROR;
                break;
        }

        switch(response_pdu)
        {
            case LMP_ACCEPTED_EXT_OPCODE:
            {
                lmp_send_lmp_accepted_ext(ce_index,LMP_ESCAPE4_OPCODE,
                                          LMP_ESCO_LINK_REQ_OPCODE,
                                          transaction_id);
                return BT_FW_SUCCESS;
            }

            case LMP_NOT_ACCEPTED_EXT_OPCODE:
            {
                lmp_send_lmp_not_accepted_ext(ce_index,
                                              LMP_ESCAPE4_OPCODE,
                                              LMP_ESCO_LINK_REQ_OPCODE,
                                              transaction_id,
                                              reason);

                ESCO_ERR(ESCO_NEGOTIATION_FAILED_ESCO_CE_STATUS,1,
                         lmp_esco_connection_entity[esco_ce_index].status);

                if(esco_ce_ptr->status == CHANGING_ESCO_PARAMS)
                {
                    /* Restore the esco & acl connections to previous
                       states
                    */

                    lmp_set_ce_status(ce_index, ce_ptr->temp_ce_status);

                    esco_ce_ptr->status = ESCO_CONNECTED;

                    /* Send synchronous connection complete event */
                    hci_generate_synchronous_conn_changed_event(reason,
                            esco_ce_index);
                    lmp_reset_temp_esco_connection_entity();
                    lmp_local_dev_init_esco_param_change = FALSE;
                }
                else
                {
                    /* Restore the acl connection to previous state. */
                    lmp_set_ce_status(ce_index, ce_ptr->temp_ce_status);

                    lmp_handle_esco_conn_setup_failed(
                        esco_ce_index, ce_index, reason);
                }
                return BT_FW_SUCCESS;
            }

            case LMP_ESCO_LINK_REQ_OPCODE:
                ESCO_INF(SENDING_NEW_PARAMS_WITH_ESCO_LINK_RQE_PDU,0,0);
                lmp_get_esco_params_from_ce(parameter_list, esco_ce_index);
                if (lmp_generate_pdu(ce_index, parameter_list,
                                     LMP_ESCO_LINK_REQ_LEN,
                                     (LMP_TRAN_ID)transaction_id,
                                     LMP_NO_STATE_CHANGE) != API_SUCCESS)
                {
                    ESCO_ERR(LMP_GENERATE_PDU_FAILED,0,0);
                    return BT_FW_ERROR;
                }
                esco_ce_ptr->negotiation_count++;
                return BT_FW_SUCCESS;

            default :
                ESCO_ERR(INAVLID_RESPONSE,0,0);
                return BT_FW_SUCCESS;
        }
    }
}

/**
 * Handle the LMP_not_accepted for LMP_esco_link_req.
 *
 * \param lmp_pdu_ptr The pointer to the LMP PDU packet.
 * \param ce_index ACL connection entity index.
 *
 * \return BT_FW_SUCCESS, if the operation is successful. BT_FW_ERROR
 *         otherwise.
 */
UCHAR lmp_handle_esco_link_req_not_accepted(LMP_PDU_PKT *lmp_pdu_ptr,
        UINT16 ce_index)
{
    UINT16 esco_ce_index = 0;
    UCHAR status = 0;
    UCHAR lut_index;

    LMP_CONNECTION_ENTITY *ce_ptr;

    ce_ptr = &lmp_connection_entity[ce_index];

#ifndef _DAPE_TEST_CHK_ESCO_STATUS_BY_LMP_ESCO_CONNECTION_STATUS
    switch(ce_ptr->ce_status)
    {
        case LMP_ADDING_ESCO_LINK:
        case LMP_CHANGING_ESCO_PARAMS:
            break;
        default :
            ESCO_ERR(NOT_ACCEPTED_FOR_ESCO_LINK_REQ_RECEIVED_IN_INVALID_CE_STATUS,1,
                     lmp_connection_entity[ce_index].ce_status);
            return BT_FW_ERROR;
    }
#endif

    while(esco_ce_index < LMP_MAX_ESCO_CONN_ENTITIES)
    {
        if((lmp_esco_connection_entity[esco_ce_index].ce_index == ce_index)
                && ((lmp_esco_connection_entity[esco_ce_index].status ==
                     ADDING_ESCO_LINK) ||
                    (lmp_esco_connection_entity[esco_ce_index].status ==
                     CHANGING_ESCO_PARAMS)))
        {
            break;
        }
        esco_ce_index++;
    }

    if(esco_ce_index == LMP_MAX_ESCO_CONN_ENTITIES)
    {
        ESCO_ERR(UNEXPECTED_PDU_ESCO_CONNECTION_ENTITIES_ARE_NOT_IN_THE_CORRECT_STATUS,0,0);
        return BT_FW_ERROR ;
    }

#ifndef _DAPE_TEST_CHK_ESCO_STATUS_BY_LMP_ESCO_CONNECTION_STATUS
    switch(ce_ptr->ce_status)
    {
        case LMP_ADDING_ESCO_LINK:
            /* Send synchronous connection complete event */
            status = lmp_pdu_ptr->payload_content[4];
            hci_generate_synchronous_conn_complete_event(ce_index,
                    esco_ce_index, ESCO_LINK, status);

            /* release the esco connection entity and Lt_addr */
            lmp_release_esco_resources(ce_index,esco_ce_index);
            lmp_set_ce_status(ce_index, ce_ptr->temp_ce_status);
            lut_index = lc_get_lut_index_from_ce_index(ce_index);

            ESCO_INF(NOT_ACCEPTED_EXT_PDU_RCVD_WHILE_ADDING_ESCO,0,0);
            ESCO_INF(GENERATING_SYNCHRONOUS_CONN_COMPLETE_EVENT,0,0);
            break;

        case LMP_CHANGING_ESCO_PARAMS:
            /* Send synchronous connection changed event */
            status = lmp_pdu_ptr->payload_content[4];
            hci_generate_synchronous_conn_changed_event(status,
                    esco_ce_index);

            /* Reset the esco and acl connection entity's status to
               LMP_CONNECTED & ESCO_CONNECTED
            */

            lmp_set_ce_status(ce_index, ce_ptr->temp_ce_status);

            lut_index = lc_get_lut_index_from_ce_index(ce_index);

            lmp_esco_connection_entity[esco_ce_index].status = ESCO_CONNECTED;

            /* Negotiation for chaging esco link failed. Reset the temp esco
               connection entity.
            */
            lmp_reset_temp_esco_connection_entity();
            lmp_local_dev_init_esco_param_change = FALSE;

#ifdef _CCH_SLOT_OFFSET_
            lmp_force_global_slot_offset(lmp_esco_connection_entity[esco_ce_index].ce_index, 
				LMP_MAX_CE_DATABASE_ENTRIES+ esco_ce_index, 
				lmp_esco_connection_entity[esco_ce_index].tesco, 
				lmp_esco_connection_entity[esco_ce_index].esco_window, 
				lmp_esco_connection_entity[esco_ce_index].desco);			
#endif
			
            break;

        default:
            break;
    }
#else
    switch(lmp_esco_connection_entity[esco_ce_index].status)
    {
        case ADDING_ESCO_LINK:
            /* Send synchronous connection complete event */
            status = lmp_pdu_ptr->payload_content[4];
            hci_generate_synchronous_conn_complete_event(ce_index,
                    esco_ce_index, ESCO_LINK, status);

            /* release the esco connection entity and Lt_addr */
            lmp_release_esco_resources(ce_index,esco_ce_index);
            lmp_set_ce_status(ce_index, ce_ptr->temp_ce_status);
            lut_index = lc_get_lut_index_from_ce_index(ce_index);

            ESCO_INF(NOT_ACCEPTED_EXT_PDU_RCVD_WHILE_ADDING_ESCO,0,0);
            ESCO_INF(GENERATING_SYNCHRONOUS_CONN_COMPLETE_EVENT,0,0);
            break;

        case CHANGING_ESCO_PARAMS:
            /* Send synchronous connection changed event */
            status = lmp_pdu_ptr->payload_content[4];
            hci_generate_synchronous_conn_changed_event(status,
                    esco_ce_index);

            /* Reset the esco and acl connection entity's status to
               LMP_CONNECTED & ESCO_CONNECTED
            */

            lmp_set_ce_status(ce_index, ce_ptr->temp_ce_status);

            lut_index = lc_get_lut_index_from_ce_index(ce_index);

            lmp_esco_connection_entity[esco_ce_index].status = ESCO_CONNECTED;

            /* Negotiation for chaging esco link failed. Reset the temp esco
               connection entity.
            */
            lmp_reset_temp_esco_connection_entity();
            lmp_local_dev_init_esco_param_change = FALSE;

#ifdef _CCH_SLOT_OFFSET_
            lmp_force_global_slot_offset(lmp_esco_connection_entity[esco_ce_index].ce_index, 
				LMP_MAX_CE_DATABASE_ENTRIES+ esco_ce_index, 
				lmp_esco_connection_entity[esco_ce_index].tesco, 
				lmp_esco_connection_entity[esco_ce_index].esco_window, 
				lmp_esco_connection_entity[esco_ce_index].desco);			
#endif
			
            break;

        default:
            break;
    }
#endif

    return BT_FW_SUCCESS;
}

#ifdef ESCO_DISC_DEBUG
UCHAR wait_for_esco_window_end_intr=FALSE;
UCHAR transaction_id_of_disconnection=0, reason_for_disconnection=0;
#endif /* ESCO_DISC_DEBUG*/
UCHAR reason_for_disconnection=0;

/**
 * Handle the LMP_remove_esco_link_req PDU and takes appropriate action. If
 * the pdu is accepted then the resources occupied by the esco link will be
 * freed when ack is received.
 *
 * \param lmp_pdu_ptr The pointer to the LMP PDU packet.
 * \param acl_ce_index ACL connection entity index.
 *
 * \return BT_FW_SUCCESS, if the operation is successful. BT_FW_ERROR
 *         otherwise.
 */
UCHAR lmp_handle_remove_esco_link_req_pdu(LMP_PDU_PKT *lmp_pdu_ptr,
        UINT16 acl_ce_index)
{
    UCHAR esco_handle;
    UINT16 esco_ce_index=0;
    UCHAR reason;
    UCHAR transaction_id;

    LMP_CONNECTION_ENTITY *ce_ptr;

    ce_ptr = &lmp_connection_entity[acl_ce_index];

    transaction_id = (UCHAR)(lmp_pdu_ptr->payload_content[0] & 0x01);

    RT_BT_LOG(YELLOW, CCH_DBG_016, 3,acl_ce_index, esco_ce_index, lmp_esco_connection_entity[esco_ce_index].status);

    if(ce_ptr->remote_dev_role != transaction_id)
    {
        ESCO_INF(SELF_DEVICE_INITIATED_THE_TRANSACTION,0,0);
    }
    else
    {
        ESCO_INF(REMOTE_DEVICE_DEVICE_INITIATED_THE_TRANSACTION,0,0);
    }

    switch(ce_ptr->ce_status)
    {
        case LMP_CONNECTED :
#ifdef COMPILE_SNIFF_MODE
        case LMP_SNIFF_MODE :
#endif
            break;
        default:
            ESCO_ERR(REMOVE_ESCO_LINK_REQ_PDU_RECEIVED_IN_INVALID_CE_STATUS,1,
                     lmp_connection_entity[acl_ce_index].ce_status);
            lmp_send_lmp_not_accepted_ext(acl_ce_index,LMP_ESCAPE4_OPCODE,
                                          LMP_REMOVE_ESCO_LINK_REQ_OPCODE,
                                          transaction_id,
                                          PDU_NOT_ALLOWED_ERROR);
            return BT_FW_ERROR;
    }

    esco_handle = lmp_pdu_ptr->payload_content[2];
    reason = lmp_pdu_ptr->payload_content[3];

    if(lmp_get_esco_ce_index_from_esco_handle(esco_handle, &esco_ce_index) !=
            API_SUCCESS)
    {
        ESCO_ERR(REMOVE_ESCO_LINK_REQ_PDU_RECEIVED_WITH_INVALID_HANDLE,1,esco_handle);
        lmp_send_lmp_not_accepted_ext(acl_ce_index,LMP_ESCAPE4_OPCODE,
                                      LMP_REMOVE_ESCO_LINK_REQ_OPCODE,
                                      transaction_id,
                                      INVALID_LMP_PARAMETERS_ERROR);
        return BT_FW_ERROR ;
    }

    if(lmp_esco_connection_entity[esco_ce_index].status == ESCO_CONNECTED)
    {
        reason_for_disconnection = reason;
        lmp_esco_connection_entity[esco_ce_index].status =  ESCO_DISCONNECTING;
        lmp_send_lmp_accepted_ext(acl_ce_index,LMP_ESCAPE4_OPCODE,
                                  LMP_REMOVE_ESCO_LINK_REQ_OPCODE,
                                  transaction_id);
    }
    else
    {
        lmp_send_lmp_not_accepted_ext(acl_ce_index,LMP_ESCAPE4_OPCODE,
                                      LMP_REMOVE_ESCO_LINK_REQ_OPCODE,
                                      transaction_id,
                                      PDU_NOT_ALLOWED_ERROR);
        ESCO_ERR(REMOVE_ESCO_LINK_PDU_RECEIVED_INVALID_ESCO_CE_STATUS,1,
                 lmp_esco_connection_entity[esco_ce_index].status);
        return BT_FW_ERROR;
    }

    return BT_FW_SUCCESS;
}

/**
 * Handle the LMP_accepted PDU for LMP_remove_esco_link_req pdu and takes
 * appropriate action.
 *
 * \param lmp_pdu_ptr The pointer to the LMP PDU packet.
 * \param ce_index ACL connection entity index.
 *
 * \return BT_FW_SUCCESS, if the operation is successful. BT_FW_ERROR
 *         otherwise.
 */
UCHAR lmp_handle_remove_esco_link_req_accepted(LMP_PDU_PKT *lmp_pdu_ptr,
        UINT16 ce_index)
{
    UINT16 esco_ce_index=0;
    UCHAR am_addr;
    /*    UCHAR lut_index = 0;*/

    LMP_CONNECTION_ENTITY *ce_ptr;
    LMP_ESCO_CONNECTION_ENTITY *esco_ce_ptr;

    ce_ptr = &lmp_connection_entity[ce_index];
////// dape added
    if (lmp_get_esco_ce_index_from_ce_index(ce_index,
                ESCO_DISCONNECTING, &esco_ce_index)
            != API_SUCCESS)
    {
        BZ_ASSERT(0, "Invalid ce_status or lmp_esco_connection_entity"
                     "got screwed up (host could have also misbehaved)");
        return UNSPECIFIED_ERROR;
    }
/////////////////////

    esco_ce_ptr = &lmp_esco_connection_entity[esco_ce_index];
#ifndef _DAPE_ALLOW_SYNC_LINK_FOR_WIN8
    switch(ce_ptr->ce_status)
    {
        case LMP_ESCO_DISCONNECTING:
        case LMP_CONNECTED:
            /* this case maybe happen in some IOT case - austin */
            break;
        default:
            ESCO_ERR(LMP_ACCEPTED_PDU_RECEIVED_IN_CE_STATUS,1,
                     ce_ptr->ce_status);
            return BT_FW_ERROR ;
    }
#endif    
///// dape marked off	
#if 0
    esco_ce_index = 0;

    while(esco_ce_index < LMP_MAX_ESCO_CONN_ENTITIES)
    {
        if((esco_ce_ptr->status == ESCO_DISCONNECTING)&&
                (esco_ce_ptr->ce_index == ce_index))
            break;
        esco_ce_index++;
    }
#endif
    hci_generate_disconnection_complete_event(HCI_COMMAND_SUCCEEDED,
            esco_ce_ptr->conn_handle,
            CONNECTION_TERMINATED_LOCAL_HOST_ERROR);

    ESCO_INF(GENERATING_DISC_COMP_EVENT_FOR_ESCO,0,0);

    /* Killing baseband level connection */
    lc_kill_esco_connection(esco_ce_index);

    if(ce_ptr->remote_dev_role == SLAVE)
    {
        lmp_put_esco_am_addr(esco_ce_ptr->lt_addr, ce_ptr->phy_piconet_id);
    }
    else
    {
        am_addr = esco_ce_ptr->lt_addr;

        lmp_slave_unuse_esco_am_addr_ppi(am_addr, ce_ptr->phy_piconet_id);
    }

    lc_reset_esco_scheduler((UCHAR)ce_index, (UCHAR)esco_ce_index);
    lmp_release_esco_entity_to_ce_database(esco_ce_index);
    lmp_set_ce_status(ce_index, ce_ptr->temp_ce_status);

    lmp_update_max_tesco();
    lmp_update_gcd_of_tesco();

#ifndef COMPILE_SINGLE_SLOT_PACKETS_ONLY
    /* Change the max slot value for the acl link*/
    change_max_slot_with_esco(ce_index);
#endif

    ESCO_INF(UPDATING_THE_PKTS_ALLOWED_ACCORDING_TO_MAX_SLOT,0,0);
    hci_check_and_enable_eir_recv();
    return BT_FW_SUCCESS;
}

/**
 * Allocate a free connection handle for the new ESCO connection.
 *
 * \param conn_handle The new connection handle to be returned.
 *
 * \return API_SUCCESS, if the operation is successful. API_FAILURE, otherwise.
 */
API_RESULT lmp_allocate_esco_conn_handle(UINT16* conn_handle)
{
    UCHAR index;


    for(index = (UCHAR)(lmp_assigned_ce_index+1); index <=
            (LMP_MAX_CE_DATABASE_ENTRIES + LMP_MAX_BC_CONNECTIONS); index++)
    {
        if((lmp_ch_to_ce_index_table[index-1].status == UNASSIGNED) &&
                (lmp_esco_ch_to_ce_index_table[index-1].status == UNASSIGNED))
        {
            lmp_esco_ch_to_ce_index_table[index-1].status = ASSIGNED;
            lmp_assigned_ce_index = index;
            *conn_handle = index;
            ESCO_INF(ALLOCATED_ESCO_CONN_HANDLE,1,*conn_handle);
            return API_SUCCESS;
        }
    }

    for(index = 1 ; index <= lmp_assigned_ce_index ; index++)
    {
        if((lmp_ch_to_ce_index_table[index-1].status == UNASSIGNED) &&
                (lmp_esco_ch_to_ce_index_table[index-1].status == UNASSIGNED))
        {
            lmp_esco_ch_to_ce_index_table[index-1].status = ASSIGNED;
            lmp_assigned_ce_index = index;
            *conn_handle = index;
            ESCO_INF(ALLOCATED_ESCO_CONN_HANDLE,1,*conn_handle);
            return API_SUCCESS;
        }
    }

    return API_FAILURE;
}

/**
 * Release the \a conn_handle provided. The connection handle was for ESCO
 * link.
 *
 * \param conn_handle The connection handle to be freed.
 *
 * \return BT_FW_SUCCESS, if the operation is successful. BT_FW_ERROR,
 *         otherwise.
 */
UCHAR lmp_esco_release_conn_handle(UINT16 conn_handle)
{
    if(conn_handle < 1 || conn_handle > (LMP_MAX_CE_DATABASE_ENTRIES +
                                         LMP_MAX_BC_CONNECTIONS))
    {
        return BT_FW_ERROR;
    }
    lmp_esco_ch_to_ce_index_table[conn_handle-1].status = UNASSIGNED;
    ESCO_INF(FREEING_ESCO_CONN_HANDLE,0,0);
    return BT_FW_SUCCESS;
}

/**
 * Get the ESCO connection entity index from the \a am_addr provided.
 *
 * \param am_addr The Active Member Address.
 * \param piconet_id The Logical Piconet ID.
 * \param ce_index The ESCO connection entity index returned.
 *
 * \return API_SUCCESS, if the operation is successful. API_FAILURE, otherwise.
 */
API_RESULT lmp_get_esco_ce_index_from_am_addr(UCHAR am_addr, UINT16 piconet_id,
        UINT16* ce_index)
{
    if (piconet_id > LMP_MAX_PICONETS_SUPPORTED)
    {
        ESCO_ERR(INVALID_PICONET_ID_RECD_AM_ADDR_PICONET_ID,2,am_addr, piconet_id);

        return API_FAILURE;
    }

    if ((am_addr < 8) && (am_addr != BC_AM_ADDR))
    {
        LMP_AM_ADDR_TO_CE_INDEX_TABLE *table;
        UINT16 index;

        table = &lmp_am_addr_to_ce_index_table_ppi[am_addr-1][piconet_id];
        index = table->ce_index;

        if ((table->status == UNASSIGNED) || 
            !lmp_am2ce_is_esco_ce_index(index))
        {
            return API_FAILURE;
        }

        *ce_index = (UINT16)lmp_am2ce_demangle_esco_ce_index(index);
        return API_SUCCESS;
    }

    return API_FAILURE;
}

//API_RESULT lmp_get_esco_ce_index_from_am_addr_ppi(UCHAR am_addr, UINT16 phy_piconet_id,
//        UINT16* ce_index)
//{
//    if (phy_piconet_id > LMP_MAX_PICONETS_SUPPORTED)
//    {
//        ESCO_ERR(log_file, "Invalid piconet ID recd am_addr=%d, piconet_id=%d",
//               am_addr, phy_piconet_id);
//        return API_FAILURE;
//    }
//
//    if ((am_addr < 8) && (am_addr != BC_AM_ADDR))
//    {
//        UINT16 index;
//
//        index = lmp_am_addr_to_ce_index_table_ppi[am_addr-1][phy_piconet_id].ce_index;
//        if (lmp_am_addr_to_ce_index_table_ppi[am_addr-1][phy_piconet_id].status
//                == UNASSIGNED
//                || !lmp_am2ce_is_esco_ce_index(index))
//        {
//            return API_FAILURE;
//        }
//
//        *ce_index = (UINT16)lmp_am2ce_demangle_esco_ce_index(index);
//        return API_SUCCESS;
//    }
//
//    return API_FAILURE;
//}


/**
 * Get the ESCO connection entity index from the \a conn_handle provided.
 *
 * \param conn_handle The Connection handle of the ESCO link.
 * \param ce_index The ESCO connection entity index returned.
 *
 * \return API_SUCCESS, if the operation is successful. API_FAILURE, otherwise.
 */
API_RESULT lmp_get_esco_ce_index_from_conn_handle(UINT16 conn_handle,
        UINT16* ce_index)
{
    if((conn_handle > 0) &&
            (conn_handle <= (LMP_MAX_CE_DATABASE_ENTRIES +LMP_MAX_BC_CONNECTIONS)))
    {
        if(lmp_esco_ch_to_ce_index_table[conn_handle-1].status == UNASSIGNED)
        {
            return API_FAILURE;
        }
        else
        {
            *ce_index = lmp_esco_ch_to_ce_index_table[conn_handle-1].ce_index ;
            return API_SUCCESS ;
        }
    }

    return API_FAILURE ;
}

/**
 * Allocate a new connection entity from the ESCO connection entity database.
 *
 * \param None.
 *
 * \return New ESCO connection entity index, if the operation is successful.
 *         BT_FW_ERROR, otherwise.
 */
UINT16 lmp_allocate_esco_entity_from_ce_database(void)
{
    UINT16 ce_index ;
    UINT16 conn_handle ;

    RT_BT_LOG(GRAY, LMP_SYNC_LINKS_1435, 0, 0);

    for(ce_index = 0; ce_index < LMP_MAX_ESCO_CONN_ENTITIES; ce_index++)
    {
        if(lmp_esco_connection_entity[ce_index].entity_status == UNASSIGNED)
        {
            if(lmp_allocate_esco_conn_handle(&conn_handle) != API_SUCCESS)
            {
                ESCO_ERR(NO_FREE_CONNECTION_HANDLES,0,0);
                return BT_FW_ERROR ;
            }
            lmp_esco_connection_entity[ce_index].entity_status = ASSIGNED ;
            lmp_esco_ch_to_ce_index_table[conn_handle-1].ce_index = ce_index ;
            lmp_esco_connection_entity[ce_index].conn_handle = conn_handle ;
            lmp_esco_connection_entity[ce_index].erroneous_data_reporting =
                lmp_self_device_data.default_erroneous_data_reporting;
            ESCO_INF(ALLOCATED_CE_INDEX,1,ce_index);
            return ce_index ;
        }
    }

    return BT_FW_ERROR ;
}

/**
 * Release ESCO connection entity index provided.
 *
 * \param ce_index The ESCO connection entity index to be freed.
 *
 * \return BT_FW_SUCCESS, if the operation is successful. BT_FW_ERROR,
 *         otherwise.
 */
UCHAR lmp_release_esco_entity_to_ce_database(UINT16 ce_index)
{
    UINT16 conn_handle ;
    UCHAR esco_handle;
    LMP_ESCO_CONNECTION_ENTITY *esco_ce;
    esco_ce = &lmp_esco_connection_entity[ce_index];

    RT_BT_LOG(GRAY, LMP_SYNC_LINKS_1474, 1, esco_ce->entity_status);

#ifdef _CCH_SLOT_OFFSET_
    lmp_put_global_slot_offset(esco_ce->ce_index, ce_index + LMP_MAX_CE_DATABASE_ENTRIES);	
#endif

    if(esco_ce->entity_status != UNASSIGNED)
    {
#ifdef _DAPE_ALLOW_SYNC_LINK_FOR_WIN8        
        /* to prevent error when another esco is creating. */
        if (lmp_connection_entity[esco_ce->ce_index].ce_status != LMP_ADDING_ESCO_LINK)
#endif  
        {
            lmp_connection_entity[esco_ce->ce_index].sco_status_byte = 0;
            lmp_connection_entity[esco_ce->ce_index].codec_status_byte = 0;
        }
        conn_handle = esco_ce->conn_handle;
        esco_handle = esco_ce->esco_handle;
        lmp_esco_release_conn_handle(conn_handle);
        lmp_init_esco_connection_entity(ce_index,esco_handle);
        esco_ce->entity_status = UNASSIGNED;
        ESCO_INF(FREEING_ESCO_CE,1,ce_index);
        return BT_FW_SUCCESS;
    }

    return BT_FW_ERROR;
}

/**
 * Initialize the ESCO connection entity provided by \a ce_index.
 *
 * \param ce_index ESCO connection entity index to be freed.
 * \param esco_handle The ESCO handle for the esco link.
 *
 * \return BT_FW_SUCCESS, if the operation is successful. BT_FW_ERROR,
 *         otherwise.
 */
UCHAR lmp_init_esco_connection_entity(UINT16 ce_index, UCHAR esco_handle)
{
#if 1
    LMP_ESCO_CONNECTION_ENTITY* esco_ce_ptr;

    esco_ce_ptr = &lmp_esco_connection_entity[ce_index];
    memset((void *)esco_ce_ptr, 0, sizeof(LMP_ESCO_CONNECTION_ENTITY));
    lmp_esco_connection_entity[ce_index].esco_handle = esco_handle;
    lmp_esco_connection_entity[ce_index].esco_window_status = INACTIVE;
    lmp_esco_connection_entity[ce_index].esco_continous_status = INACTIVE;
#else

    lmp_esco_connection_entity[ce_index].entity_status = UNASSIGNED;
    lmp_esco_connection_entity[ce_index].status = NO_CONNECTION;
    lmp_esco_connection_entity[ce_index].bb_esco_connection = FALSE;
    lmp_esco_connection_entity[ce_index].ce_index = 0x00;
    lmp_esco_connection_entity[ce_index].esco_handle = esco_handle;
    lmp_esco_connection_entity[ce_index].lt_addr = 0x00;
    lmp_esco_connection_entity[ce_index].desco = 0x00;
    lmp_esco_connection_entity[ce_index].tesco = 0x00;
    lmp_esco_connection_entity[ce_index].wesco = 0x00;
    lmp_esco_connection_entity[ce_index].esco_window = 0x00;
    lmp_esco_connection_entity[ce_index].m_to_s_packet_type = 0x00;
    lmp_esco_connection_entity[ce_index].s_to_m_packet_type = 0x00;
    lmp_esco_connection_entity[ce_index].air_mode = 0x00;
    lmp_esco_connection_entity[ce_index].retransmission_effort = 0x00;
    lmp_esco_connection_entity[ce_index].negotiation_flag = 0x00;
    lmp_esco_connection_entity[ce_index].negotiation_count = 0x00;
    lmp_esco_connection_entity[ce_index].timing_control_flags = 0x00;
    lmp_esco_connection_entity[ce_index].packet_type = 0x00;
    lmp_esco_connection_entity[ce_index].conn_handle = 0x00;
    lmp_esco_connection_entity[ce_index].m_to_s_packet_length= 0x00;
    lmp_esco_connection_entity[ce_index].s_to_m_packet_length= 0x00;
    lmp_esco_connection_entity[ce_index].max_latency = 0x00;
    lmp_esco_connection_entity[ce_index].voice_setting = 0x00;
    lmp_esco_connection_entity[ce_index].next_instant = 0x00;
    lmp_esco_connection_entity[ce_index].tx_bandwidth = 0x00;
    lmp_esco_connection_entity[ce_index].rx_bandwidth = 0x00;
    lmp_esco_connection_entity[ce_index].no_data_count = 0;
    lmp_esco_connection_entity[ce_index].num_of_completed_esco_packets = 0x00;

    /* Reset the esco buffers */
    reset_esco_ce_buffer(ce_index);
    memset(&lmp_esco_connection_entity[ce_index].esco_data_q,0,
           sizeof(LMP_ESCO_DATA_Q));
    lmp_esco_connection_entity[ce_index].esco_window_status = INACTIVE;
    lmp_esco_connection_entity[ce_index].esco_continous_status = INACTIVE;
    lmp_esco_connection_entity[ce_index].use_codec = FALSE;
    lmp_esco_connection_entity[ce_index].host_pkt_empty_cnts = 0;
#endif
    lmp_reset_negotiation_history();

    return BT_FW_SUCCESS;
}

/**
 * Generate the esco parameters when the local device starts initiating the
 * esco connection.
 *
 * \param esco_ce_index The ESCO connection entity index.
 *
 * \return HCI_COMMAND_SUCCEEDED, if the operation is successful. A valid
 *         bluetooth error code, otherwise.
 */
UCHAR lmp_generate_init_esco_params(UINT16 esco_ce_index)
{
    UCHAR max_latency=0;
    UINT16 acl_ce_index;
    UCHAR retx_effort=0;
    UCHAR use_init=0,timing_control_flags=0;
    UCHAR local_air_mode = 0;
    UINT32 clock_value=0;
    LMP_ESCO_CONNECTION_ENTITY *esco_conn_entity_ptr;

    /* Select the esco ce index whose parameters have to be used to generate
       the negotiation params. Parameters from temp_esco_connection_entity will
       be taken while esco link is being changed. lmp_esco_connection_entity
       will be used if a new connection is being established
    */
    if(lmp_esco_connection_entity[esco_ce_index].status == CHANGING_ESCO_PARAMS)
    {
        esco_conn_entity_ptr = &temp_esco_connection_entity;
    }
    else
    {
        esco_conn_entity_ptr = &lmp_esco_connection_entity[esco_ce_index];
    }
    max_latency = (UCHAR)TIMER_VAL_TO_SLOT_VAL(esco_conn_entity_ptr->max_latency);

    retx_effort = esco_conn_entity_ptr->retransmission_effort;
    /* get the ce index of the acl link */
    acl_ce_index = lmp_esco_connection_entity[esco_ce_index].ce_index;

    if(lmp_generate_esco_params(acl_ce_index,esco_ce_index,
                                esco_conn_entity_ptr->tx_bandwidth,
                                esco_conn_entity_ptr->rx_bandwidth,
                                max_latency,retx_effort) != API_SUCCESS)
    {
        //RT_BT_LOG(RED, LMP_SYNC_LINKS_1595, 0, 0);

        ESCO_ERR(ESCO_PARAM_GEN_FAILED,0,0);
        return INVALID_HCI_COMMAND_PARAMETERS_ERROR;
    }

    if(lmp_connection_entity[acl_ce_index].remote_dev_role == SLAVE)
    {
        /* Calculate timing control flag */

        lc_get_clock_in_scatternet(&clock_value,
                                   lmp_connection_entity[acl_ce_index].phy_piconet_id);

        if(clock_value >> 27 & 0x01)
        {
            use_init = 1 ;   /* Use Initialization 2*/
        }
        else
        {
            use_init = 0 ;   /* Use Initialization 1*/
        }

        timing_control_flags = (UCHAR)(timing_control_flags | (use_init << 1));
        /* Timing_control_flag.  */
        esco_conn_entity_ptr->timing_control_flags = timing_control_flags;
    }

    local_air_mode = (UCHAR)(esco_conn_entity_ptr->voice_setting & 0x03);

    switch(local_air_mode)
    {
        case AIR_CODING_CVSD :
            local_air_mode = LMP_CVSD ;
            break;

        case AIR_CODING_MU_LAW:
            local_air_mode = LMP_U_LAW_LOG ;
            break;

        case AIR_CODING_A_LAW:
            local_air_mode = LMP_A_LAW_LOG ;
            break;

        case AIR_CODING_TRANSPARENT:
            local_air_mode = LMP_TRANSPARENT_DATA ;
            break;

        default :
            break;
    }

    esco_conn_entity_ptr->air_mode = local_air_mode;
    //RT_BT_LOG(GRAY, LMP_SYNC_LINKS_1651, 0, 0);

    return HCI_COMMAND_SUCCEEDED;

}

/**
 * Allocate a free AM_ADDR and ESCO connection entity entry.
 *
 * \param am_addr The new Active Member Address returned.
 * \param ce_index The new ESCO connection entity index returned.
 *
 * \return API_SUCCESS, if the operation is successful. API_FAILURE, otherwise.
 */
API_RESULT lmp_allocate_esco_am_addr(UCHAR *am_addr, UINT16 *ce_index,
                                     UCHAR phy_piconet_id)
{
    UCHAR temp_am_addr;
    UINT16 temp_ce_index;

    UCHAR piconet_id = PICONET_MASTER;

    piconet_id = phy_piconet_id;

    if (lmp_get_esco_am_addr(&temp_am_addr, piconet_id) == API_SUCCESS)
    {
        temp_ce_index = lmp_allocate_esco_entity_from_ce_database();
        if (temp_ce_index != BT_FW_ERROR)
        {
            lmp_am_addr_to_ce_index_table_ppi[temp_am_addr-1][piconet_id].ce_index
                                = lmp_am2ce_mangle_esco_ce_index(temp_ce_index);

            lmp_esco_connection_entity[temp_ce_index].lt_addr = temp_am_addr;
            *am_addr = temp_am_addr;
            *ce_index = temp_ce_index;

            ESCO_INF(ALLOCATED_AM_ADDR, 1, temp_am_addr);

            return API_SUCCESS;
        }
        else
        {
            ESCO_ERR(ALLOCATION_OF_CONN_ENTITY_FAILED,0,0);
            lmp_put_esco_am_addr(temp_am_addr, piconet_id);
        }
    }

    return API_FAILURE;
}


/**
 * Populate the \a parameter_list for the LMP_esco_link_req pdu taking the
 * values from the ESCO connection entity entry pointed by \a esco_ce_index.
 *
 * \param parameter_list The output parameter list.
 * \param esco_ce_index The ESCO connection entity index.
 *
 * \return None.
 */
void lmp_get_esco_params_from_ce(UCHAR *parameter_list,UINT16 esco_ce_index)
{
    LMP_ESCO_CONNECTION_ENTITY *esco_conn_entity_ptr;
    UINT16 acl_ce_index;

    /* Select the esco ce index whose parameters have to be used to generate
       the negotiation params. Parameters from temp_esco_connection_entity will
       be taken while esco link is being changed. lmp_esco_connection_entity
       will be used if a new connection is being established
    */
    if(lmp_esco_connection_entity[esco_ce_index].status ==CHANGING_ESCO_PARAMS)
    {
        esco_conn_entity_ptr = &temp_esco_connection_entity;
    }
    else
    {
        esco_conn_entity_ptr = &lmp_esco_connection_entity[esco_ce_index];
    }

    acl_ce_index = lmp_esco_connection_entity[esco_ce_index].ce_index;

    parameter_list[0] = LMP_ESCAPE4_OPCODE;
    parameter_list[2] = LMP_ESCO_LINK_REQ_OPCODE;

    parameter_list[3] = lmp_esco_connection_entity[esco_ce_index].esco_handle;
    parameter_list[4] = lmp_esco_connection_entity[esco_ce_index].lt_addr;
    parameter_list[5] = esco_conn_entity_ptr->timing_control_flags;
    parameter_list[6] = esco_conn_entity_ptr->desco;
    parameter_list[7] = esco_conn_entity_ptr->tesco;
    parameter_list[8] = esco_conn_entity_ptr->wesco;
    parameter_list[9] = esco_conn_entity_ptr->m_to_s_packet_type;
    parameter_list[10] = esco_conn_entity_ptr->s_to_m_packet_type;
    parameter_list[11] = LSB(esco_conn_entity_ptr->m_to_s_packet_length);
    parameter_list[12] = MSB(esco_conn_entity_ptr->m_to_s_packet_length);
    parameter_list[13] = LSB(esco_conn_entity_ptr->s_to_m_packet_length);
    parameter_list[14] = MSB(esco_conn_entity_ptr->s_to_m_packet_length);
    parameter_list[15] = esco_conn_entity_ptr->air_mode;
    parameter_list[16] = esco_conn_entity_ptr->negotiation_flag;
}

/**
 * Decide the response to be sent in response to the LMP_esco_link_req pdu
 * received. If the LMP_esco_link_req pdu has to be sent, then the ESCO
 * connection entity entry will be updated with the new parameters.
 *
 * \param esco_ce_index The ESCO connection entity index.
 * \param acl_ce_index The ACL connection entity index.
 * \param response_pdu The response pdu opcode returned.
 * \param reason The reason for rejection (if rejected) returned.
 *
 * \return None.
 */
void lmp_validate_new_esco_link_req(UINT16 esco_ce_index,UINT16 acl_ce_index,
                                    UCHAR *response_pdu, UCHAR *reason)
{
    UCHAR desco,tesco,wesco;
    UCHAR negotiated_max_latency,max_latency;
    UCHAR new_negotiation_state = LMP_ESCO_INIT_NEG_STATE;
    UCHAR negotiation_state;
    UCHAR rx_packet_type,tx_packet_type;
    UCHAR use_init = 0,tc_flags = 0;
    UCHAR retx_effort = 0;
    UINT16 rx_packet_length,tx_packet_length,rem_packet_type;
    UINT32 rx_bandwidth,tx_bandwidth,clock_value = 0;

    LMP_ESCO_CONNECTION_ENTITY *esco_ce_ptr;
    LMP_CONNECTION_ENTITY *ce_ptr;

    esco_ce_ptr = &lmp_esco_connection_entity[esco_ce_index];
    ce_ptr = &lmp_connection_entity[acl_ce_index];

    tc_flags = esco_ce_ptr->timing_control_flags;
    desco = esco_ce_ptr->desco;
    tesco = esco_ce_ptr->tesco;
    wesco = esco_ce_ptr->wesco;

    negotiation_state = esco_ce_ptr->negotiation_flag;

    if(ce_ptr->remote_dev_role == MASTER)
    {
        rx_packet_type = esco_ce_ptr->m_to_s_packet_type;
        rx_packet_length = esco_ce_ptr->m_to_s_packet_length;
        tx_packet_type = esco_ce_ptr->s_to_m_packet_type;
        tx_packet_length = esco_ce_ptr->s_to_m_packet_length;
    }
    else
    {
        rx_packet_type = esco_ce_ptr->s_to_m_packet_type;
        rx_packet_length = esco_ce_ptr->s_to_m_packet_length;
        tx_packet_type =  esco_ce_ptr->m_to_s_packet_type;
        tx_packet_length = esco_ce_ptr->m_to_s_packet_length;
    }

    rem_packet_type = (UINT16)(lmp_to_hci_packet_type(rx_packet_type) |
                               lmp_to_hci_packet_type(tx_packet_type));

    /* Check if the packet types chosen by the remote device for setting
        up esco connection is supported by the local device
    */
    {
        UINT16 allowed_pkt_type = lmp_get_allowed_synchronous_pkt_type
                                  (acl_ce_index, esco_ce_ptr->packet_type);

        if((rem_packet_type & allowed_pkt_type) != rem_packet_type)
        {
            new_negotiation_state = LMP_ESCO_PARAM_NOT_SUPPORTED_NEG_STATE;
        }
    }
    /* Derive the Tx and Rx bandwidth from the parameters received in the
       pdu
    */
    tx_bandwidth = (UINT32)( 1600 * tx_packet_length)/tesco;
    rx_bandwidth = (UINT32)( 1600 * rx_packet_length)/tesco;

    /* Check if the requested Tx bandwidth is supported by the local device */
    if((esco_ce_ptr->tx_bandwidth != 0xFFFFFFFF)&&
            (tx_bandwidth != esco_ce_ptr->tx_bandwidth))
    {
        *response_pdu = LMP_NOT_ACCEPTED_EXT_OPCODE;
        *reason = UNSUPPORTED_PARAMETER_VALUE_ERROR;

        ESCO_ERR(TX_BW_NOT_SUPPORTED_BY_LOCAL_DEV,0,0);
        ESCO_ERR(LOCAL_DEV_S_TX_BW,1,esco_ce_ptr->tx_bandwidth);
        ESCO_ERR(REM_DEV_S_TX_BW,1,tx_bandwidth);

        return;
    }

    /* Check if the requested Rx bandwidth is supported by the local device */
    if((esco_ce_ptr->rx_bandwidth != 0xFFFFFFFF)&&
            (rx_bandwidth != esco_ce_ptr->rx_bandwidth))
    {
        *response_pdu = LMP_NOT_ACCEPTED_EXT_OPCODE;
        *reason = UNSUPPORTED_PARAMETER_VALUE_ERROR;
        ESCO_ERR(RX_BW_NOT_SUPPORTED_BY_LOCAL_DEV,0,0);
        ESCO_ERR(LOCAL_DEV_S_RX_BW,1,esco_ce_ptr->rx_bandwidth);
        ESCO_ERR(REM_DEV_S_RX_BW,1,rx_bandwidth);

        return;
    }

    retx_effort = esco_ce_ptr->retransmission_effort;

    if(wesco == 0)
    {
        /* If the remote device does not support retransmission and the local
           device has retransmission enabled then negotiate the retransmission
           window size.
        */
        if((esco_ce_ptr->retransmission_effort != 0xFF) &&
                ((esco_ce_ptr->retransmission_effort != 0)))
        {
            new_negotiation_state = LMP_ESCO_PARAM_NOT_SUPPORTED_NEG_STATE;
        }
    }
    else
    {
        if(esco_ce_ptr->retransmission_effort == 0xFF)
        {
            retx_effort = 2;
        }
        /* If the remote device does supports retransmission and the
           retransmission is disabled in local device then negotiate the
           retransmission window size.
        */
        if((esco_ce_ptr->retransmission_effort != 0xFF) &&
                ((esco_ce_ptr->retransmission_effort == 0)))
        {
            new_negotiation_state = LMP_ESCO_PARAM_NOT_SUPPORTED_NEG_STATE;
            retx_effort = esco_ce_ptr->retransmission_effort;
        }
    }

    negotiated_max_latency = (UCHAR)esco_ce_ptr->max_latency;

    /* Check if the local device's max_latency is violated.
      max_latency = esco interval + esco window in slots.
    */
    max_latency = (UCHAR)(tesco + esco_ce_ptr->esco_window);
    max_latency = (UCHAR)SLOT_VAL_TO_TIMER_VAL(max_latency);


    /* Checking if the Max latency(in milliseconds)derived from the Tesco and
       Esco window is a fractional value. If yes then round off to the next
       higher value
    */
    if(TIMER_VAL_TO_SLOT_VAL(max_latency) != (tesco +
            esco_ce_ptr->esco_window))
    {
        max_latency++;
    }

    /* If max_latency is don't care for local device and max latency derived
       from remote device's parameters is less than Min val of Max latency
       supported then reject the esco connection */

    if((max_latency < MAX_LATENCY_MIN_VAL) &&
            (esco_ce_ptr->max_latency == 0xFF))
    {
        *response_pdu = LMP_NOT_ACCEPTED_EXT_OPCODE;
        *reason = UNSUPPORTED_PARAMETER_VALUE_ERROR;
        ESCO_ERR(MAX_LATENCY_VAL_IS_INVALID_REM_DEV_S_MAX_LAT_LOCAL_DEV_S_MAX_LAT,2,
                 max_latency,esco_ce_ptr->max_latency);

        return;
    }

    /* If Max latency derived from remote device's parameters is less than the
       Min allowed max latency value or it's greater than local device's
       Max latency , negotiate with the local device's Max Latency
    */
    if((max_latency < MAX_LATENCY_MIN_VAL) ||
            (max_latency > esco_ce_ptr->max_latency))
    {
        new_negotiation_state = LMP_ESCO_MAX_LAT_VIOLATION_NEG_STATE;
        negotiated_max_latency = (UCHAR)esco_ce_ptr->max_latency;
        ESCO_INF(REM_DEV_VIOLATES_LOCAL_DEV_S_MAX_LATENCY,0,0);
        ESCO_INF(REM_DEV_S_MAX_LAT_LOCAL_DEV_S_MAX_LAT, 2, max_latency,
                 esco_ce_ptr->max_latency);
    } /* Accept the Max latency value derived from the remote device's
         parameters
      */
    else
    {
        negotiated_max_latency = max_latency;
    }

    /* Check if the remote devices parameters cause reserve slot violation */
    if(lmp_check_for_reserved_slot_violation(desco,tesco,
            esco_ce_ptr->esco_window,
            acl_ce_index,
            esco_ce_index) != API_SUCCESS)
    {
        if(new_negotiation_state != LMP_ESCO_MAX_LAT_VIOLATION_NEG_STATE)
        {
            new_negotiation_state = LMP_ESCO_RES_SLOT_VIOLATION_NEG_STATE;
        }
        ESCO_INF(REM_DEV_PARAMS_CAUSED_RESERVED_SLOT_VIOLATION,0,0);
    }

    /* Updating the timing control flag if local device is
       Master
    */
    if(ce_ptr->remote_dev_role == SLAVE)
    {
        lc_get_clock_in_scatternet(&clock_value,
                                   ce_ptr->phy_piconet_id);

        if(clock_value >> 27 & 0x01)
        {
            use_init = 1 ;   /*Use Initialization 2*/
        }
        else
        {
            use_init = 0 ;   /*Use Initialization 1*/
        }

        tc_flags = (UCHAR)(use_init << 1);
        /* timing_control_flag.  */
        esco_ce_ptr->timing_control_flags = tc_flags;
    }

    switch(new_negotiation_state)
    {
        case LMP_ESCO_INIT_NEG_STATE:

#ifdef _CCH_PCM_CHOOSE_ESCO_    

// need to be reduce

            if( (sync_link_codec_state == OVER_CODEC)&&
                (sync_link_codec_type == PCM_CODEC) &&
                (negotiation_state == LMP_ESCO_INIT_NEG_STATE) ) 
            {
                lmp_esco_connection_entity[esco_ce_index].negotiation_flag =
                        LMP_ESCO_DEFAULT_NEG_STATE;
                new_negotiation_state = LMP_ESCO_DEFAULT_NEG_STATE;
            }
            else
            {
		
#endif
            if((ce_ptr->remote_dev_role == MASTER))
            {
                *response_pdu = LMP_ACCEPTED_EXT_OPCODE;
            }
            else
            {
                lmp_esco_connection_entity[esco_ce_index].negotiation_flag =
                    LMP_ESCO_DEFAULT_NEG_STATE;
                *response_pdu = LMP_ESCO_LINK_REQ_OPCODE;
            }
            break;

#ifdef _CCH_PCM_CHOOSE_ESCO_   
            }
#endif			
            

        case LMP_ESCO_RES_SLOT_VIOLATION_NEG_STATE:
        case LMP_ESCO_MAX_LAT_VIOLATION_NEG_STATE:
        case LMP_ESCO_PARAM_NOT_SUPPORTED_NEG_STATE:
        case LMP_ESCO_DEFAULT_NEG_STATE:
            /* Calculate the new params which will still support the requested
               bandwidth */

            negotiated_max_latency =
                (UCHAR)TIMER_VAL_TO_SLOT_VAL(negotiated_max_latency);
            if((lmp_generate_esco_params(acl_ce_index,esco_ce_index,
                                         tx_bandwidth,rx_bandwidth,negotiated_max_latency,
                                         retx_effort)
                    == API_SUCCESS))
            {
                *response_pdu = LMP_ESCO_LINK_REQ_OPCODE;
                esco_ce_ptr->negotiation_flag = new_negotiation_state;
                ESCO_INF(NEW_NEG_STATE,1,new_negotiation_state);
            }
            else
            {
                *response_pdu = LMP_NOT_ACCEPTED_EXT_OPCODE;
                if ( new_negotiation_state ==
                        LMP_ESCO_RES_SLOT_VIOLATION_NEG_STATE)
                {
                    *reason = LMP_RESERVED_SLOT_VIOLATION_ERROR;
                }
                else if ( new_negotiation_state ==
                          LMP_ESCO_PARAM_NOT_SUPPORTED_NEG_STATE)
                {
                    *reason = UNSUPPORTED_FEATURE_OR_PARAMETER_ERROR;
                }
                else
                {
                    *reason = INVALID_LMP_PARAMETERS_ERROR;
                }
                ESCO_ERR(ESCO_PARAMETERS_GENEARTION_FAILED,0,0);
            }
            break;
    }
}

/**
 * Check the state of the connection and generate appropriate event and
 * action. This timer is started when synchronous connection request is sent
 * to the host.
 *
 * \param timer_handle The handle for this timer.
 * \param conn_handle_param The user data provided during the timer creation.
 *
 * \return None.
 */
void lmp_sync_conn_accept_timeout_handler(TimerHandle_t timer_handle)
{
    UINT16 acl_ce_index = 0;
    UINT16 esco_ce_index;
    UINT16 conn_handle ;
    OS_SIGNAL signal_send;
    LMP_ESCO_CONNECTION_ENTITY *esco_ce_ptr;
    LMP_CONNECTION_ENTITY *ce_ptr;

    conn_handle = (UINT16)((UINT32)pvTimerGetTimerID(timer_handle)) ;

    if(lmp_get_esco_ce_index_from_conn_handle(conn_handle, &esco_ce_index)
            == API_SUCCESS)
    {
        esco_ce_ptr = &lmp_esco_connection_entity[esco_ce_index];

        ESCO_INF(ESCO_CONNECTION_ACCEPT_TIMEOUT_OCCURED,0,0);
        acl_ce_index = esco_ce_ptr->ce_index;

        ce_ptr = &lmp_connection_entity[acl_ce_index];

        switch(esco_ce_ptr->status)
        {
            case  WAITING_FOR_ACCEPT_SYNC_CONN:

                lmp_set_ce_status(acl_ce_index, ce_ptr->temp_ce_status);

                /* Send LMP_NOT_ACCEPTED to the remote device with the reason
                   HOST_TIMEOUT.
                 */
                lmp_send_lmp_not_accepted_ext(
                    acl_ce_index,LMP_ESCAPE4_OPCODE,
                    LMP_ESCO_LINK_REQ_OPCODE,
                    ce_ptr->remote_dev_role,
                    CONNECTION_ACCEPT_TIMEOUT_EXCEEDED_ERROR);
                ESCO_ERR(HOST_TO_FOR_ESCO_CONN_REQ_EVT,0,0);

                hci_generate_synchronous_conn_complete_event(acl_ce_index,
                        esco_ce_index, ESCO_LINK,
                        CONNECTION_ACCEPT_TIMEOUT_EXCEEDED_ERROR);

                lmp_put_esco_am_addr(
                    esco_ce_ptr->lt_addr, ce_ptr->phy_piconet_id);
                lmp_release_esco_entity_to_ce_database(esco_ce_index);
                break;

            default :
                ESCO_ERR(INVALID_STATE_CONNECTION_ACCEPT_TIMEOUT_IS_RECD,1,
                         ce_ptr->ce_status);
                break;
        }
    }
    else
    {
        RT_BT_LOG(GRAY, LMP_SYNC_LINK_FAIL_TO_GET_ESCO_INDEX, 0, 0);
//		return;
    }

    signal_send.type = LMP_DELETE_TIMER_HANDLE_SIGNAL;
    signal_send.param = (OS_ADDRESS)
                        (lmp_connection_entity[acl_ce_index].conn_accept_timer_handle);
    if (OS_SEND_SIGNAL_TO_TASK(lmp_task_handle, signal_send) != BT_ERROR_OK)
    {
        LMP_ERR(LOG_LEVEL_HIGH, OS_SEND_SIGNAL_TO_TASK_FAILED,0,0);
    }
    lmp_connection_entity[acl_ce_index].conn_accept_timer_handle = NULL;
}

/**
 * Validate the incoming esco link request pdu and decide the response for
 * that pdu.
 *
 * \param lmp_pdu_ptr The Pointer to the LMP PDU packet.
 * \param esco_ce_index The ESCO connection entity index.
 * \param ce_index The ACL connection entity index.
 * \param response_pdu The response pdu returned.
 * \param reason The reason for rejection (if rejected) returned.
 *
 * \return None.
 */
void lmp_validate_esco_link_req_pdu(LMP_PDU_PKT *lmp_pdu_ptr,
                                    UINT16 esco_ce_index, UINT16 ce_index, UCHAR *response_pdu,
                                    UCHAR *reason)
{
    UCHAR tc_flags,desco,tesco,wesco,esco_window=0;
    UCHAR offset = 2;
    UCHAR new_negotiation_state = LMP_ESCO_INIT_NEG_STATE,use_init;
    UCHAR stom_pkt_type,mtos_pkt_type;
    UCHAR air_mode, local_air_mode, retx_effort = 0;
    UINT16 mtos_pkt_length,stom_pkt_length;
    UINT16 max_latency,negotiated_max_latency;
    UINT16 rx_packet_type,rem_packet_type,tx_packet_type;
    UINT16 rx_packet_length,tx_packet_length;
    UINT32 tx_bandwidth,rx_bandwidth,clock_value = 0;
    LMP_ESCO_CONNECTION_ENTITY *esco_conn_entity_ptr;

#ifdef _ROM_CODE_PATCHED_
#ifdef _CCH_8723_RCP_
// RCP for change the whole function or Just change the input paramter
// Used to check eSCO IOT v.s. BH101

    if (rcp_lmp_validate_esco_link_req_pdu_start != NULL)
    {
        if ( rcp_lmp_validate_esco_link_req_pdu_start((void *)&reason, lmp_pdu_ptr,esco_ce_index,ce_index,&response_pdu) )
        {
            return;
        }
    }    
#endif	
#endif

#ifdef _CCH_RTL8723A_B_CUT
// _CCH_8723_A_ECO_
    UCHAR slot_overlap;
#endif

    /* Select the esco ce index whose parameters have to be used to generate
       the negotiation params. Parameters from temp_esco_connection_entity will
       be taken while esco link is being changed. lmp_esco_connection_entity
       will be used if a new connection is being established
    */
    if(lmp_esco_connection_entity[esco_ce_index].status ==CHANGING_ESCO_PARAMS)
    {
        esco_conn_entity_ptr = &temp_esco_connection_entity;
    }
    else
    {
        esco_conn_entity_ptr = &lmp_esco_connection_entity[esco_ce_index];
    }

    /* extract the parameters from the pdu */
    /* skip lt_addr and esco handle */
    offset+=2;

    tc_flags = lmp_pdu_ptr->payload_content[offset];
    offset++;

    desco = lmp_pdu_ptr->payload_content[offset];
    offset++;

    tesco = lmp_pdu_ptr->payload_content[offset];
    offset++;

    wesco = lmp_pdu_ptr->payload_content[offset];
    offset++;

    mtos_pkt_type = lmp_pdu_ptr->payload_content[offset];
    offset++;

    stom_pkt_type = lmp_pdu_ptr->payload_content[offset];
    offset++;

    mtos_pkt_length = lmp_pdu_ptr->payload_content[offset] | 
                     (lmp_pdu_ptr->payload_content[offset + 1] << 8);
    offset += 2;

    stom_pkt_length = lmp_pdu_ptr->payload_content[offset] | 
                     (lmp_pdu_ptr->payload_content[offset + 1] << 8);
    offset += 2;

    air_mode = lmp_pdu_ptr->payload_content[offset];
    offset++;

    /* Get the master to slave and slave to master packet types */
    if(lmp_connection_entity[ce_index].remote_dev_role == MASTER)
    {
        rx_packet_type = mtos_pkt_type;
        rx_packet_length = mtos_pkt_length;
        tx_packet_type = stom_pkt_type;
        tx_packet_length = stom_pkt_length;
    }
    else
    {
        rx_packet_type = stom_pkt_type;
        rx_packet_length = stom_pkt_length;
        tx_packet_type = mtos_pkt_type;
        tx_packet_length = mtos_pkt_length;
    }

    rem_packet_type = (UINT16)(lmp_to_hci_packet_type(rx_packet_type) |
                               lmp_to_hci_packet_type(tx_packet_type));

    /* Check if the packet types chosen by the remote device for setting
        up esco connection is supported by the local device
    */
    {
        UINT16 allowed_pkt_type = lmp_get_allowed_synchronous_pkt_type
                                  (ce_index, esco_conn_entity_ptr->packet_type);
        if((rem_packet_type & allowed_pkt_type) != rem_packet_type)
        {
            new_negotiation_state = LMP_ESCO_PARAM_NOT_SUPPORTED_NEG_STATE;
        }
    }

    /* Derive the Tx and Rx bandwidth from the parameters received in the
       pdu
    */
    tx_bandwidth = (UINT32)(1600 * tx_packet_length)/tesco;
    rx_bandwidth = (UINT32)(1600 * rx_packet_length)/tesco;
    local_air_mode = lmp_convert_air_mode((UCHAR)
                                          (esco_conn_entity_ptr->voice_setting & AIR_MODE_MASK), HCI_LAYER,
                                          LMP_LAYER);

    /* Tx Bandwidth, Rx Bandwidth and Voice_Setting(Air_Mode) can not be
     * renegotiated.
     */
    if ((esco_conn_entity_ptr->tx_bandwidth != 0xFFFFFFFF
            && tx_bandwidth != esco_conn_entity_ptr->tx_bandwidth)
            || (esco_conn_entity_ptr->rx_bandwidth != 0xFFFFFFFF
                && rx_bandwidth != esco_conn_entity_ptr->rx_bandwidth)
            || air_mode != local_air_mode)
    {
        *response_pdu = LMP_NOT_ACCEPTED_EXT_OPCODE;
        *reason = INVALID_LMP_PARAMETERS_ERROR;
        return;
    }

    esco_window = (UCHAR)(esco_pktsize(lmp_to_hci_packet_type(rx_packet_type))+
                          esco_pktsize(lmp_to_hci_packet_type(tx_packet_type)));

    /* Esco window size for this link */
    esco_window = (UCHAR)(esco_window + wesco);

    retx_effort = esco_conn_entity_ptr->retransmission_effort;

    /* Derive the retransmission effort from the parameters in the pdu */
    if (wesco != 0)
    {
        /* Check if the local device supports retransmission parameters
           proposed by the remote device. If local device does not support
           retransmission then negotiate with new parameters for retransmissioon
           window.
        */
        if(esco_conn_entity_ptr->retransmission_effort == 0)
        {
            new_negotiation_state = LMP_ESCO_PARAM_NOT_SUPPORTED_NEG_STATE;
        }

        if(esco_conn_entity_ptr->retransmission_effort == 0xFF)
        {
            retx_effort = 2;
        }

    }
    else
    {
        /* Check if the local device has retransmission enabled. If it is
          enabled then with new parameters for retransmissioon
          window.
                */

        if((esco_conn_entity_ptr->retransmission_effort != 0xFF) &&
                (esco_conn_entity_ptr->retransmission_effort != 0))
        {
            new_negotiation_state = LMP_ESCO_PARAM_NOT_SUPPORTED_NEG_STATE;
        }

    }

    negotiated_max_latency = TIMER_VAL_TO_SLOT_VAL(
                                 esco_conn_entity_ptr->max_latency);

    /* check if the local device's max_latency is violated
       max_latency = esco interval + esco window
    */

    max_latency = (UINT16)(tesco + esco_window);


    /* If the max_latecny derived from the lmp pdu is more than the
       local device's max latency then start negotiating with
       new values. set negotiation state to Latency violation
    */
    if(max_latency > negotiated_max_latency)
    {
        ESCO_INF(REM_DEV_PARAMS_CAUSED_LATENCY_VIOLATION,0,0);
        new_negotiation_state = LMP_ESCO_MAX_LAT_VIOLATION_NEG_STATE;
    }
    else
    {
        negotiated_max_latency = max_latency;
    }


    /* Check if the remote devices parameters cause reserved slot violation */
    if(lmp_check_for_reserved_slot_violation(desco,tesco,esco_window,ce_index,
            esco_ce_index) != API_SUCCESS)
    {
        if(new_negotiation_state != LMP_ESCO_MAX_LAT_VIOLATION_NEG_STATE)
        {
            ESCO_INF(REM_DEV_PARAMS_CAUSED_RESERVED_SLOT_VIOLATION,0,0);
            new_negotiation_state = LMP_ESCO_RES_SLOT_VIOLATION_NEG_STATE;
        }
    }

    switch(new_negotiation_state)
    {
        case LMP_ESCO_INIT_NEG_STATE:
            /* The parameters proposed by the remote devcie are acceptable
               to the local device. Send lmp_accepted for setting up this
               link
            */
            esco_conn_entity_ptr->timing_control_flags = tc_flags;
            esco_conn_entity_ptr->desco = desco;
            esco_conn_entity_ptr->tesco = tesco;
            esco_conn_entity_ptr->wesco = wesco;
            esco_conn_entity_ptr->m_to_s_packet_type = mtos_pkt_type;
            esco_conn_entity_ptr->s_to_m_packet_type = stom_pkt_type;
            esco_conn_entity_ptr->m_to_s_packet_length = mtos_pkt_length;
            esco_conn_entity_ptr->s_to_m_packet_length = stom_pkt_length;
            esco_conn_entity_ptr->air_mode = air_mode;
            esco_conn_entity_ptr->negotiation_flag = new_negotiation_state;
            esco_conn_entity_ptr->esco_window = esco_window;
            *response_pdu = LMP_ACCEPTED_EXT_OPCODE;
            break;

        case LMP_ESCO_RES_SLOT_VIOLATION_NEG_STATE:
        case LMP_ESCO_MAX_LAT_VIOLATION_NEG_STATE:
        case LMP_ESCO_PARAM_NOT_SUPPORTED_NEG_STATE:
        case LMP_ESCO_DEFAULT_NEG_STATE:

            /* The parameters proposed by remote device cause Reserve slot or
               latency violation. Negotiate with new params
            */

            /* calculate the new params which will still support the requested
               bandwidth */

#ifndef _CCH_RTL8723A_B_CUT
     
            tx_bandwidth = (UINT32)( 1600 * tx_packet_length)/tesco;
            rx_bandwidth = (UINT32)( 1600 * rx_packet_length)/tesco;

            /* If this function call is successful then new DESCO, TESCO,WESCO
               M_TO_S_PKT_TYPE, S_TO_M_PKT_TYPE, M_TO_S_PKT_LENGTH,
               S_TO_M_PKT_LENGTH, ESCO_WINDOW will be updated in the
               esco connection entity.
             */
            if((lmp_generate_esco_params(ce_index,esco_ce_index,tx_bandwidth,
                                         rx_bandwidth,negotiated_max_latency,
                                         retx_effort))
                    == API_SUCCESS)
            {

                /* Updating the timing control flag if local device is
                   Master
                */
                if(lmp_connection_entity[ce_index].remote_dev_role == SLAVE)
                {
                    lc_get_clock_in_scatternet(&clock_value,
                                               lmp_connection_entity[ce_index].phy_piconet_id);

                    if(( clock_value >> 27) & 0x01)
                    {
                        use_init = 1 ;   /* Use Initialization 2*/
                    }
                    else
                    {
                        use_init = 0 ;   /* Use Initialization 1*/
                    }

                    tc_flags = 0;
                    tc_flags = (UCHAR)(tc_flags | (use_init << 1));
                    /* timing_control_flag.  */
                    esco_conn_entity_ptr->timing_control_flags = tc_flags;
                }

                esco_conn_entity_ptr->negotiation_flag = new_negotiation_state;
                esco_conn_entity_ptr->air_mode = air_mode;
                *response_pdu = LMP_ESCO_LINK_REQ_OPCODE;
                ESCO_INF(NEW_NEGOTIATION_STATE, 1, esco_conn_entity_ptr->negotiation_flag);

            }
            else
            {
                *response_pdu = LMP_NOT_ACCEPTED_EXT_OPCODE;
                *reason = UNSUPPORTED_PARAMETER_VALUE_ERROR;
                ESCO_ERR(ESCO_PARAMETERS_GENEARTION_FAILED,0,0);
            }
#else
// _CCH_8723_A_ECO_
            if( g_efuse_lps_setting_3.esco_nego_by_other )
            {
                esco_conn_entity_ptr->timing_control_flags = tc_flags;
                esco_conn_entity_ptr->desco = desco;
                esco_conn_entity_ptr->tesco = tesco;
                esco_conn_entity_ptr->wesco = wesco;
                esco_conn_entity_ptr->m_to_s_packet_type = mtos_pkt_type;
                esco_conn_entity_ptr->s_to_m_packet_type = stom_pkt_type;
                esco_conn_entity_ptr->m_to_s_packet_length = mtos_pkt_length;
                esco_conn_entity_ptr->s_to_m_packet_length = stom_pkt_length;
                esco_conn_entity_ptr->air_mode = air_mode;
                esco_conn_entity_ptr->negotiation_flag = new_negotiation_state;
                esco_conn_entity_ptr->esco_window = esco_window;

                slot_overlap = 0;
                slot_overlap = lmp_force_global_slot_offset(ce_index, 
                              LMP_MAX_CE_DATABASE_ENTRIES+ esco_ce_index, tesco, esco_window, desco);

                if( slot_overlap == 0)
                {
    
                    *response_pdu = LMP_ACCEPTED_EXT_OPCODE;
                    break;	
                }else
                {


                    UINT16 sniff_slot_offset = 0;

                    sniff_slot_offset = lmp_get_global_slot_offset(ce_index, 
                              LMP_MAX_CE_DATABASE_ENTRIES+ esco_ce_index, tesco, esco_window, 0, 0);


                    UCHAR lmp_get_global_slot_offset_fail;
                    lmp_get_global_slot_offset_fail = 1;

                    if (sniff_slot_offset < GLOBAL_SLOT_INTERVAL)
                    {
                        lmp_get_global_slot_offset_fail = 0;
                    }
    
                    if (lmp_get_global_slot_offset_fail)
                    {
                        *response_pdu = LMP_NOT_ACCEPTED_EXT_OPCODE;
                        *reason = UNSUPPORTED_PARAMETER_VALUE_ERROR;
                        ESCO_ERR(ESCO_PARAMETERS_GENEARTION_FAILED,0,0);
                    }else
                    {

                        if(lmp_connection_entity[ce_index].remote_dev_role == SLAVE)
                        {
                            lc_get_clock_in_scatternet(&clock_value,
                                                   lmp_connection_entity[ce_index].phy_piconet_id);

                            if(( clock_value >> 27) & 0x01)
                            {
                                use_init = 1 ;   /* Use Initialization 2*/
                            }
                            else
                            {
                                use_init = 0 ;   /* Use Initialization 1*/
                            }
    
                            tc_flags = 0;
                            tc_flags = (UCHAR)(tc_flags | (use_init << 1));
                            /* timing_control_flag.  */
                            esco_conn_entity_ptr->timing_control_flags = tc_flags;
                        }

                        esco_conn_entity_ptr->desco = sniff_slot_offset;
                        esco_conn_entity_ptr->negotiation_flag = new_negotiation_state;
                        esco_conn_entity_ptr->air_mode = air_mode;
                        *response_pdu = LMP_ESCO_LINK_REQ_OPCODE;
                        ESCO_INF(NEW_NEGOTIATION_STATE, 1, esco_conn_entity_ptr->negotiation_flag);
                    }			
                }    
            }else
            {            
                tx_bandwidth = (UINT32)( 1600 * tx_packet_length)/tesco;
                rx_bandwidth = (UINT32)( 1600 * rx_packet_length)/tesco;

                /* If this function call is successful then new DESCO, TESCO,WESCO
                   M_TO_S_PKT_TYPE, S_TO_M_PKT_TYPE, M_TO_S_PKT_LENGTH,
                   S_TO_M_PKT_LENGTH, ESCO_WINDOW will be updated in the
                   esco connection entity.
                 */
                if((lmp_generate_esco_params(ce_index,esco_ce_index,tx_bandwidth,
                                             rx_bandwidth,negotiated_max_latency,
                                             retx_effort))
                        == API_SUCCESS)
                {

                    /* Updating the timing control flag if local device is
                       Master
                    */
                    if(lmp_connection_entity[ce_index].remote_dev_role == SLAVE)
                    {
                        lc_get_clock_in_scatternet(&clock_value,
                                                   lmp_connection_entity[ce_index].phy_piconet_id);

                        if(( clock_value >> 27) & 0x01)
                        {
                            use_init = 1 ;   /* Use Initialization 2*/
                        }
                        else
                        {
                            use_init = 0 ;   /* Use Initialization 1*/
                        }

                        tc_flags = 0;
                        tc_flags = (UCHAR)(tc_flags | (use_init << 1));
                        /* timing_control_flag.  */
                        esco_conn_entity_ptr->timing_control_flags = tc_flags;
                    }

                    esco_conn_entity_ptr->negotiation_flag = new_negotiation_state;
                    esco_conn_entity_ptr->air_mode = air_mode;
                    *response_pdu = LMP_ESCO_LINK_REQ_OPCODE;
                    ESCO_INF(NEW_NEGOTIATION_STATE, 1, esco_conn_entity_ptr->negotiation_flag);

                }
                else
                {
                    *response_pdu = LMP_NOT_ACCEPTED_EXT_OPCODE;
                    *reason = UNSUPPORTED_PARAMETER_VALUE_ERROR;
                    ESCO_ERR(ESCO_PARAMETERS_GENEARTION_FAILED,0,0);
                }
            }		   
#endif	
            break;
    }
}

/**
 * Disconnect the ESCO link. If the ACL connections are being disconnected,
 * then the procedure to remove all esco connections on that link will be
 * initiated from this function.
 *
 * \param conn_handle The Connection handle of the ESCO link.
 * \param reason The reason for disconnection.
 * \param sent_cmd_status Set to TRUE if HCI_COMMAND_STATUS_EVENT is sent to
 *                        the host. FALSE, otherwise.
 *
 * \return BT_FW_SUCCESS, if the operation is successful. BT_FW_ERROR,
 *         otherwise.
 */
UCHAR lmp_handle_esco_disconnect(UINT16 conn_handle,UCHAR reason,
                                 UCHAR *sent_cmd_status)
{
    UINT16 esco_ce_index,acl_ce_index;
    UCHAR parameter_list[LMP_REMOVE_ESCO_LINK_REQ_LEN];
    LMP_CONNECTION_ENTITY *ce_ptr;

    //RT_BT_LOG(GRAY, LMP_SYNC_LINKS_2448, 0, 0);

    if((lmp_get_esco_ce_index_from_conn_handle(conn_handle,&esco_ce_index)
            ==API_SUCCESS))
    {
        //RT_BT_LOG(GRAY, LMP_SYNC_LINKS_2452, 0, 0);

        if(lmp_esco_connection_entity[esco_ce_index].status != ESCO_CONNECTED)
        {

            *sent_cmd_status = TRUE;

            /*Send Command status event for ACL Disconnect*/
            hci_generate_command_status_event(
                HCI_DISCONNECT_OPCODE,
                COMMAND_DISALLOWED_ERROR );
            return BT_FW_SUCCESS;
        }
        acl_ce_index = lmp_esco_connection_entity[esco_ce_index].ce_index;

        ce_ptr = &lmp_connection_entity[acl_ce_index];

        ce_ptr->disconnect_reason = reason;

#ifdef _CCH_PAGE_CON_
        ce_ptr->connect_reason = 0;
#endif	

        parameter_list[0] = LMP_ESCAPE4_OPCODE;
        parameter_list[2] = LMP_REMOVE_ESCO_LINK_REQ_OPCODE;
        parameter_list[3] =
            lmp_esco_connection_entity[esco_ce_index].esco_handle;
        parameter_list[4] = reason;

        ce_ptr->temp_ce_status = ce_ptr->ce_status;
        if (lmp_generate_pdu(acl_ce_index, parameter_list,
                             LMP_REMOVE_ESCO_LINK_REQ_LEN,
                             SELF_DEV_TID, LMP_ESCO_DISCONNECTING) != API_SUCCESS)
        {
            ESCO_ERR(LMP_GENERATE_PDU_FAILED,0,0);
            return BT_FW_ERROR;
        }
#ifdef _DAPE_PUT_SLOT_OFFSET_EARLIER_FOR_WIN8
#ifdef _CCH_SLOT_OFFSET_
        lmp_put_global_slot_offset(lmp_esco_connection_entity[esco_ce_index].ce_index, 
                                    LMP_MAX_CE_DATABASE_ENTRIES + esco_ce_index);
#endif
#endif
        /* Set full bandwidth flag while disconnecting */
        {
            UINT16 reg_val;
            reg_val = BB_read_baseband_register(SCO_PACKET_TYPE_REGISTER);
            reg_val = (UINT16)(reg_val | BIT8); /* Set eSCO full-bw bit */
            BB_write_baseband_register(SCO_PACKET_TYPE_REGISTER, reg_val);
        }


        lmp_esco_connection_entity[esco_ce_index].status =
            ESCO_DISCONNECTING;
        return BT_FW_SUCCESS;
    }

    return BT_FW_ERROR;
}

/**
 * Release the resources allocated for esco and programs Baseband to kill esco
 * connection.
 *
 * \param acl_ce_index The ACL connection entity index.
 *
 * \return BT_FW_SUCCESS, if the operation is successful. BT_FW_ERROR,
 *         otherwise.
 */
UCHAR lmp_kill_esco_connection(UINT16 acl_ce_index)
{
    UINT16 esco_ce_index = 0;
    LMP_ESCO_CONNECTION_ENTITY *esco_ce_ptr;

    while(esco_ce_index < LMP_MAX_ESCO_CONN_ENTITIES)
    {
        if((lmp_esco_connection_entity[esco_ce_index].ce_index == acl_ce_index)
                &&( lmp_esco_connection_entity[esco_ce_index].status ==
                    ESCO_DISCONNECTING))
        {
            break;
        }
        esco_ce_index++;
    }

    if(esco_ce_index == LMP_MAX_ESCO_CONN_ENTITIES)
    {
        return BT_FW_ERROR;
    }

    esco_ce_ptr = &lmp_esco_connection_entity[esco_ce_index];

    hci_generate_disconnection_complete_event(HCI_COMMAND_SUCCEEDED,
            esco_ce_ptr->conn_handle, reason_for_disconnection);

    /* Kill the baseband level connection */
    lc_kill_esco_connection(esco_ce_index);

    lc_reset_esco_scheduler((UCHAR)acl_ce_index, (UCHAR)esco_ce_index);

    lmp_put_esco_am_addr(esco_ce_ptr->lt_addr,
                         lmp_connection_entity[acl_ce_index].phy_piconet_id);

    lmp_release_esco_entity_to_ce_database(esco_ce_index);

    lmp_update_max_tesco();
    lmp_update_gcd_of_tesco();

#ifndef COMPILE_SINGLE_SLOT_PACKETS_ONLY
    /* Change the max slot value for the acl link*/
    change_max_slot_with_esco(acl_ce_index);
#endif

    hci_check_and_enable_eir_recv();
    return BT_FW_SUCCESS;
}

/**
 * Establish ESCO link. This function is called after all the ESCO
 * negotiations are done.
 *
 * \param ce_index The ACL Connection entity index.
 *
 * \return BT_FW_SUCCESS, if the operation is successful. BT_FW_ERROR,
 *         otherwise.
 */
UCHAR lmp_create_esco_connection(UINT16 ce_index)
{


#ifdef _ROM_CODE_PATCHED_
#ifdef _CCH_NEW_SPEC_RCP_
    if (rcp_lmp_create_esco_connection != NULL)
    {
        UCHAR return_status;
        if ( rcp_lmp_create_esco_connection((void *)&return_status, ce_index) )
        {
            return return_status;
        }
    }    
#endif	
#endif

    UINT16 esco_ce_index = 0;
    LMP_CONNECTION_ENTITY *ce_ptr;
    LMP_ESCO_CONNECTION_ENTITY *esco_ce_ptr;

    /** Unconditionally reset negotiation history */
    lmp_reset_negotiation_history();

    ce_ptr = &lmp_connection_entity[ce_index];

    //RT_BT_LOG(GRAY, LMP_SYNC_LINKS_2592, 1, ce_ptr->ce_status);
#ifndef _DAPE_ALLOW_SYNC_LINK_FOR_WIN8
    switch(ce_ptr->ce_status)
    {
        case LMP_CONNECTED:
        case LMP_ADDING_ESCO_LINK:
        case LMP_CHANGING_ESCO_PARAMS:
            break;
        default :
            RT_BT_LOG(RED, INVALID_LMP_CE_STATUS,1,ce_ptr->ce_status);
            return BT_FW_ERROR;
    }
#endif
    while(esco_ce_index < LMP_MAX_ESCO_CONN_ENTITIES)
    {
        if((lmp_esco_connection_entity[esco_ce_index].ce_index == ce_index)
                &&( (lmp_esco_connection_entity[esco_ce_index].status ==
                     ADDING_ESCO_LINK)
                    || (lmp_esco_connection_entity[esco_ce_index].status ==
                        CHANGING_ESCO_PARAMS)))
        {
            break;
        }
        esco_ce_index++;
    }

    if(esco_ce_index == LMP_MAX_ESCO_CONN_ENTITIES)
    {
        return BT_FW_ERROR;
    }

    RT_BT_LOG(GRAY, LMP_SYNC_LINKS_2622, 2, ce_ptr->ce_status, esco_ce_index);

    esco_ce_ptr = &lmp_esco_connection_entity[esco_ce_index];

    ESCO_INF(ESCO_CE_INDEX,1,esco_ce_index);
    ESCO_INF(LT_ADDR_1,1,esco_ce_ptr->lt_addr);

    switch(esco_ce_ptr->status)
    {
        case ADDING_ESCO_LINK:
            /* Increment the number of esco connections */
            lmp_self_device_data.number_of_esco_connections++;
            
#ifdef _BRUCE_IMPLEMENTED_NO_OF_ESCO_CONN_
            ce_ptr->no_of_esco_connections++;
#endif

#ifdef _CCH_SC_ESCO_FLAG

            BZ_AUTH_LINK_PARAMS* auth;
            auth = lmp_connection_entity[ce_index].auth;

#ifdef _CCH_SC_ECDH_P256_LOG
            RT_BT_LOG(BLUE, YL_DBG_HEX_3, 3, ce_index, auth->secure_conn_enabled, auth->not_first_esco);
#endif	

            if(auth->secure_conn_enabled)
            {
#ifdef _CCH_SC_TEST_20130201_ESCO_INI_FLAG
                if(auth->not_first_esco == 0)
#endif            
                {
                    // only set at first time
                    if((esco_ce_ptr->timing_control_flags) & ESCO_TC_FLAG_INIT2)
                    {
                        BB_write_sc_esco_first_flag(ce_index, BIT30|BIT31);
                    }
                    else
                    {
                    BB_write_sc_esco_first_flag(ce_index, BIT31);
                }
                    auth->not_first_esco = 1;
                }
#ifdef _CCH_SC_TEST_20130201_ESCO_INI_FLAG
                else
                {
                    UINT32 daycounter;
                    BB_read_sc_esco_daycounter(ce_index, &daycounter);
		            RT_BT_LOG(BLUE, CCH_DBG_167, 1,daycounter);
                }
#endif
            }			
#endif

            lc_make_esco_connection(esco_ce_index, esco_ce_ptr->lt_addr);
            esco_ce_ptr->status = ESCO_CONNECTED;
            lmp_set_ce_status(ce_index, ce_ptr->temp_ce_status);

            hci_generate_synchronous_conn_complete_event(ce_index,
                    esco_ce_index, ESCO_LINK, HCI_COMMAND_SUCCEEDED);

            /* Update the max_tesco value */
            if(esco_ce_ptr->tesco > max_tesco)
            {
                max_tesco = esco_ce_ptr->tesco;
            }
            lmp_update_gcd_of_tesco();

#ifndef COMPILE_SINGLE_SLOT_PACKETS_ONLY
#ifndef _DAPE_TEST_NO_SEND_MAX_SLOT_IN_ESCO
            change_max_slot_with_esco(ce_index);
#endif
#ifdef _DAPE_TEST_SEND_MAX_SLOT_BEFORE_ESCO_CREATED
            lmp_self_device_data.adding_new_esco_conn = FALSE;
#endif
#endif

            break;

        case CHANGING_ESCO_PARAMS:
            ESCO_INF(CHANGIN_ESCO_PARAMS,0,0);

            lc_kill_esco_connection(esco_ce_index);

            lmp_self_device_data.number_of_esco_connections++;
            
#ifdef _BRUCE_IMPLEMENTED_NO_OF_ESCO_CONN_
            ce_ptr->no_of_esco_connections++;
#endif
            update_esco_connection_entity(ce_index, esco_ce_index);
            /* Reset the temp esco connection entity */
            lmp_reset_temp_esco_connection_entity();
            lc_make_esco_connection(esco_ce_index, esco_ce_ptr->lt_addr);
            esco_ce_ptr->status = ESCO_CONNECTED;

            lmp_set_ce_status(ce_index, ce_ptr->temp_ce_status);

            hci_generate_synchronous_conn_changed_event(HCI_COMMAND_SUCCEEDED,
                    esco_ce_index);
            lmp_update_gcd_of_tesco();

#ifndef COMPILE_SINGLE_SLOT_PACKETS_ONLY
            change_max_slot_with_esco(ce_index);
#endif

            break;

        default:
            ESCO_ERR(ACCEPTED_PDU_RECEIVED_IN_INVALID_STATE,0,0);
            return BT_FW_SUCCESS;
    }
    hci_check_and_enable_eir_recv();

    return BT_FW_SUCCESS;
}

/**
 * Check if the selected values of Tesco, Desco, Wesco violate the existing
 * reserved slots.
 *
 * \param desco The ESCO offset.
 * \param tesco The ESCO interval.
 * \param esco_window The ESCO retransmission window size.
 * \param ce_index The ACL connection entity index.
 * \param esco_ce_index The ESCO connection entity index.
 *
 * \return API_SUCCESS, if the operation is successful. API_FAILURE, otherwise.
 */
API_RESULT lmp_check_for_reserved_slot_violation(UCHAR desco,UCHAR tesco,
        UCHAR esco_window, UINT16 ce_index, UINT16 esco_ce_index)
{
    UCHAR retx_overlap ;
    UCHAR new_gcd_of_tesco;

    if(lmp_self_device_data.number_of_esco_connections == 0)
    {
        return API_SUCCESS;
    }

    new_gcd_of_tesco = (UCHAR)lmp_find_gcd(tesco,gcd_of_tesco);
    /* Check if the new set parameters overlap the reserved  slots
       of existing esco links. New link is allowed to override the
       retx slots.
    */
    retx_overlap = TRUE;
    if(check_esco_window_overlap(desco,(UCHAR)(desco + esco_window),ce_index,
                                 esco_ce_index,retx_overlap,tesco,
                                 new_gcd_of_tesco)
            == API_SUCCESS)
    {
        return API_SUCCESS;
    };

    return API_FAILURE;
}

/**
 * Generate the parameters for LMP_esco_link_req pdu within the range
 * specified.
 *
 * \param ce_index The ACL connection entity index.
 * \param esco_ce_index The ESCO connection entity index.
 * \param tx_bandwidth The transmit bandwidth.
 * \param rx_bandwidth The receive bandwidth.
 * \param max_latency Maximum allowed latency.
 * \param retx_effort The retransmission effort.
 *
 * \return API_SUCCESS, if the operation is successful. API_FAILURE, otherwise.
 */
API_RESULT lmp_generate_esco_params(UINT16 ce_index,UINT16 esco_ce_index,
                                    UINT32 tx_bandwidth, UINT32 rx_bandwidth, UINT16 max_latency,
                                    UCHAR retx_effort)
{
    UINT16 packet_type;
    UINT16 rx_pkt_size = 0;
    UINT16 tx_pkt_size = 0;
    UINT16 rx_pkt = 0;
    UINT16 tx_pkt = 0;
    UINT16 rx_tx_window_size = 0;
    UINT16 number_of_rx_tx_windows = 0;
    UINT16 retx_window;
    UINT16 rx_pkt_length = 0;
    UINT16 tx_pkt_length = 0;
    UINT16 temp_max_latency;
    UINT16 new_max_latency;
    UINT16 history_max_latency;
    UINT16 disallowed_pkt_type;
    UCHAR index;
    UCHAR tx_packet_index;
    UCHAR rx_packet_index;
#ifndef _CCH_SLOT_OFFSET_	
    UCHAR desco = 0;
#endif
    UCHAR tesco = 0;
    UCHAR select = FALSE;
    UINT16 max_pkt_length = 30;
    UCHAR max_window_size = 2;
    UCHAR history_retx_effort = 0;
    LMP_ESCO_CONNECTION_ENTITY *esco_conn_entity_ptr;
    UCHAR offset = 0;
    UINT16 allowed_pkt_types[3 + 4];
    const UCHAR max_no_of_pkts = 7;

    /*  Select the esco ce index whose parameters have to be used to generate
        the negotiation params. Parameters from temp_esco_connection_entity will
        be taken while esco link is being changed. lmp_esco_connection_entity
        will be used if a new connection is being established
     */
    RT_BT_LOG(GRAY, LMP_SYNC_LINKS_MSG, 7,
              ce_index, esco_ce_index, tx_bandwidth, rx_bandwidth, max_latency,
              retx_effort,lmp_esco_connection_entity[esco_ce_index].status);

    if(lmp_esco_connection_entity[esco_ce_index].status ==CHANGING_ESCO_PARAMS)
    {
        esco_conn_entity_ptr = &temp_esco_connection_entity;
    }
    else
    {
        esco_conn_entity_ptr = &lmp_esco_connection_entity[esco_ce_index];
    }

    lmp_generate_constraints_from_history(&history_max_latency,
                                          &disallowed_pkt_type, &history_retx_effort);

    /** Apply derived constraints on existing parameters */

    if(history_max_latency < max_latency)
    {
        max_latency = history_max_latency;
    }
    
    if(retx_effort == 0xFF)
    {
        retx_effort = history_retx_effort;
    }
    
    packet_type = lmp_get_allowed_synchronous_pkt_type
                  (ce_index,esco_conn_entity_ptr->packet_type);

   
#ifdef _BRUCE_MSBC_IS_RESTRICTED_PKT_TYPE 
    if (lmp_esco_over_codec == TRUE)
    {  
        //RT_BT_LOG(YELLOW,YL_DBG_HEX_2,2,g_voice_setting,g_air_mode);
        if(g_voice_setting == 0x03)
        {     
            /*MSBC only sup 2EV3 EV3 */
            packet_type &=0x00;
            packet_type |= (HCI_EV3|HCI_2_EV3);
        }
    }
#endif

    packet_type = (UINT16) (packet_type & ~disallowed_pkt_type);

    RT_BT_LOG(GRAY, LMP_SYNC_LINKS_2805, 3,
              packet_type, max_latency, retx_effort);

    retx_window = 0;

    for(index = 0; index < max_no_of_pkts; index++)
    {
        allowed_pkt_types[index] = FALSE;
    }
	
    /* Check which of the packet types can be selected for negotiation */
    if (packet_type & (HCI_EV3 | HCI_2_EV3 | HCI_3_EV3))
    {
        max_window_size = 2;
        if (packet_type & HCI_2_EV3)
        {
            allowed_pkt_types[offset++] = HCI_2_EV3;
            max_pkt_length = MAX(MAX_NO_OF_BYTES_IN_2_EV3, max_pkt_length);
        }
        if (packet_type & HCI_EV3)
        {
            allowed_pkt_types[offset++] = HCI_EV3;
            max_pkt_length = MAX(MAX_NO_OF_BYTES_IN_EV3, max_pkt_length);
        }
        if (packet_type & HCI_3_EV3)
        {
            allowed_pkt_types[offset++] = HCI_3_EV3;
            max_pkt_length = MAX(MAX_NO_OF_BYTES_IN_3_EV3, max_pkt_length);
        }       
    }
    if (lc_check_if_device_is_in_scatternet() == FALSE)
    {
        if (packet_type & (HCI_EV4 | HCI_EV5 | HCI_2_EV5 | HCI_3_EV5))
        {
            max_window_size = 6;
           
            if (packet_type & HCI_EV4)
            {
                allowed_pkt_types[offset++] = HCI_EV4;
                max_pkt_length = MAX(MAX_NO_OF_BYTES_IN_EV4, max_pkt_length);
            }
           
            if (packet_type & HCI_EV5)
            {
                allowed_pkt_types[offset++] = HCI_EV5;
                max_pkt_length = MAX(MAX_NO_OF_BYTES_IN_EV5, max_pkt_length);
            }

            if (packet_type & HCI_2_EV5)
            {
                allowed_pkt_types[offset++] = HCI_2_EV5;
                max_pkt_length = MAX(MAX_NO_OF_BYTES_IN_2_EV5, max_pkt_length);
            }

            if (packet_type & HCI_3_EV5)
            {
                allowed_pkt_types[offset++] = HCI_3_EV5;
                max_pkt_length = MAX(MAX_NO_OF_BYTES_IN_3_EV5, max_pkt_length);
            }
        }
    }
    
    for(index = offset; index < max_no_of_pkts; index++)
    {
        allowed_pkt_types[index] = FALSE;
    }
    
    RT_BT_LOG(GRAY, LMP_SYNC_LINKS_2881, 10,
              max_window_size, max_pkt_length, retx_effort,
              allowed_pkt_types[0], allowed_pkt_types[1],
              allowed_pkt_types[2], allowed_pkt_types[3],
              allowed_pkt_types[4], allowed_pkt_types[5],
              allowed_pkt_types[6]);

    if(retx_effort != 2)
    {
        new_max_latency = lmp_find_new_max_latency(rx_bandwidth,tx_bandwidth,
                          max_pkt_length,
                          max_window_size,retx_effort);

        if((new_max_latency >= MAX_LATENCY_MIN_VAL_IN_SLOTS) &&
                (new_max_latency < max_latency))
        {
            max_latency = new_max_latency;
        }
    }

    while(max_latency >= MAX_LATENCY_MIN_VAL_IN_SLOTS)
    {
        /* Select the parameters to acheive the Rx and Tx bandwidth */
        for(tx_packet_index = 0; tx_packet_index < max_no_of_pkts;
                tx_packet_index++)
        {
            /* if the Tx bandwidth is 0 the  select the default packet */
            if(tx_bandwidth == 0)
            {
                tx_pkt = LMP_DEFAULT_ESCO_PKT;
            }
            else
            {
                /* Check if this packet type is valid */
                if(allowed_pkt_types[tx_packet_index] == FALSE)
                {
                    continue;
                }
                tx_pkt = allowed_pkt_types[tx_packet_index];
            }

            for(rx_packet_index = 0 ; rx_packet_index < max_no_of_pkts;
                    rx_packet_index++)
            {
                select = FALSE;

                if(rx_bandwidth == 0)
                {
                    /*if the Rx bandwidth is 0 the  select the default packet*/
                    rx_pkt = LMP_DEFAULT_ESCO_PKT;
                }
                else
                {
                    /* Check if this packet type is valid */
                    if(allowed_pkt_types[rx_packet_index] == FALSE)
                    {
                        continue;
                    }
                    rx_pkt = allowed_pkt_types[rx_packet_index];
                }

                /* Get the number of slots occupied by chosen Rx packet */
                rx_pkt_size = esco_pktsize(rx_pkt);
                /* Get the number of slots occupied by chosen Tx packet */
                tx_pkt_size = esco_pktsize(tx_pkt);

                /* Number of slots occupied by every Tx and Rx pair */
                rx_tx_window_size = (UINT16)(rx_pkt_size + tx_pkt_size);

                /* Max latency value from which Retx window and Tesco
                   have to be derived.
                   Max latency = Tesco + Esco window
                */
                temp_max_latency = (UINT16)(max_latency - rx_tx_window_size);

                /* making temp_max_latency even */
                temp_max_latency = (UINT16)((temp_max_latency >> 1) << 1);

                number_of_rx_tx_windows = (UINT16)((temp_max_latency)/
                                                   (rx_tx_window_size));

                if( number_of_rx_tx_windows == 0)
                    continue;

                switch(retx_effort)
                {
                    case 0:
                    case 255:
                        /* Retransmission effort is 0 or Don't care(0xFF), then
                           Tesco will be equal to Max latency
                        */
                        tesco = (UCHAR)temp_max_latency;

                        if(lmp_find_packet_length(tx_bandwidth,
                                                  rx_bandwidth, &tesco, &tx_pkt_length,
                                                  &rx_pkt_length) == API_SUCCESS)
                        {
                            /* Check if the Tx and Rx packet lengths are less
                               than the maximum length of the packet type
                               selected.
                            */
                            if((tx_pkt_length <= max_allowed_length(tx_pkt))
                                    && (rx_pkt_length <= max_allowed_length(rx_pkt))
                                    && (tesco <= LMP_MAX_TESCO_VALUE))
                            {
                                select = TRUE;
                            }
                        }
                        break;

                    case 1:
                        /* Find the Tesco value */
                        tesco = (UCHAR)(temp_max_latency - rx_tx_window_size);
                        retx_window = rx_tx_window_size;

                        /* check if Retransmission can be supported */
                        if((number_of_rx_tx_windows >= 3) &&
                                (tesco > retx_window))
                        {
                            if(lmp_find_packet_length(tx_bandwidth,
                                                      rx_bandwidth, &tesco, &tx_pkt_length,
                                                      &rx_pkt_length) == API_SUCCESS)
                            {
                                /* Check if the Tx and Rx packet lengths are
                                   less than the maximum length of the packet
                                   type selected.
                                */
                                if((tx_pkt_length <=
                                        max_allowed_length(tx_pkt)) &&
                                        (rx_pkt_length <=
                                         max_allowed_length(rx_pkt)) &&
                                        (tesco <= LMP_MAX_TESCO_VALUE))
                                {
                                    select = TRUE;
                                }

                            }
                        }
                        break;

                    case 2:
                        tesco = (UCHAR)(temp_max_latency - rx_tx_window_size);
                        retx_window  = rx_tx_window_size;

                        RT_BT_LOG(GRAY, LMP_SYNC_LINKS_3026, 3,
                                  tesco, retx_window, number_of_rx_tx_windows);

                        if((number_of_rx_tx_windows >= 3) && (tesco > retx_window))
                        {
                            while(tesco >= (retx_window + rx_tx_window_size))
                            {
                                if(lmp_find_packet_length(tx_bandwidth,
                                                          rx_bandwidth, &tesco, &tx_pkt_length,
                                                          &rx_pkt_length)== API_SUCCESS)
                                {
                                    RT_BT_LOG(GRAY, LMP_SYNC_LINKS_3037, 3,
                                              rx_pkt_length, tx_pkt_length, tesco);

                                    /* Check if the Tx and Rx packet lengths are
                                       less than the maximum length of the packet
                                       type selected.
                                    */
                                    if((tx_pkt_length <= max_allowed_length(tx_pkt))
                                            && (rx_pkt_length <= max_allowed_length(rx_pkt))
                                            && (tesco <= LMP_MAX_TESCO_VALUE))
                                    {
                                        select = TRUE;
                                        break;
                                    }
                                }
                                else
                                {
                                    RT_BT_LOG(GRAY, LMP_SYNC_LINKS_3054, 0, 0);
                                }

                                tesco = (UCHAR)(tesco - rx_tx_window_size);
                                retx_window = (UINT16)(retx_window + rx_tx_window_size);
                            }
                        }
                        break;

                    default:
                        ESCO_ERR(INVALID_RETX_EFFORT_VALUE,0,0);
                        //RT_BT_LOG(GRAY, LMP_SYNC_LINKS_3065, 0, 0);

                        return API_FAILURE;
                }
#ifdef _BRUCE_CVSD_IS_RESTRICTED_TRX_PKT_SAME
                RT_BT_LOG(RED, LMP_SYNC_LINKS_2959, 7,
                          rx_pkt, rx_pkt_size, tx_pkt, tx_pkt_size, rx_tx_window_size,
                          temp_max_latency, number_of_rx_tx_windows);
                RT_BT_LOG(RED, YL_DBG_HEX_4, 4,
                         retx_effort,tx_pkt_length,rx_pkt_length,tesco);     
#endif

                if(select == TRUE)
                {
                    RT_BT_LOG(GRAY, LMP_SYNC_LINKS_2959, 7,
                              rx_pkt, rx_pkt_size, tx_pkt, tx_pkt_size, rx_tx_window_size,
                              temp_max_latency, number_of_rx_tx_windows);

                    esco_conn_entity_ptr->tesco = tesco;
                    esco_conn_entity_ptr->wesco = (UCHAR)retx_window;
                    esco_conn_entity_ptr->esco_window = (UCHAR)(retx_window +
                                                        rx_tx_window_size);

#ifndef _CCH_SLOT_OFFSET_
                    /* Select Desco for the esco link */
                    if(select_desco_val(esco_ce_index,(UCHAR)ce_index,&desco) ==
                            BT_FW_SUCCESS)
                    {
                        esco_conn_entity_ptr->desco = desco;
#endif

#ifdef _CCH_SLOT_OFFSET_
                        esco_conn_entity_ptr->desco = 
                                lmp_get_global_slot_offset(ce_index, 
                                LMP_MAX_CE_DATABASE_ENTRIES + esco_ce_index, 
                                tesco, esco_conn_entity_ptr->esco_window, 0, 0);

                        UCHAR lmp_get_global_slot_offset_fail;
                        lmp_get_global_slot_offset_fail = 1;

                        if ((esco_conn_entity_ptr->desco) < GLOBAL_SLOT_INTERVAL)
                        {
                            lmp_get_global_slot_offset_fail = 0;
                        }
                        if (lmp_get_global_slot_offset_fail)
                        {
                            lmp_put_global_slot_offset(ce_index, 
                                LMP_MAX_CE_DATABASE_ENTRIES+ esco_ce_index);
                            return UNSUPPORTED_PARAMETER_VALUE_ERROR;
                        }	
#endif
                        /* Check if after adding the new esco link , there
                           will be free bandwidth.
                        */
                        if(lmp_check_sync_conn_bandwidth(FALSE) == API_SUCCESS)
                        {
                            if(lmp_connection_entity[ce_index].remote_dev_role
                                    == MASTER)
                            {
                                esco_conn_entity_ptr->m_to_s_packet_type =
                                    (UCHAR)hci_to_lmp_packet_type(rx_pkt);
                                esco_conn_entity_ptr->m_to_s_packet_length =
                                    rx_pkt_length;
                                esco_conn_entity_ptr->s_to_m_packet_type =
                                    (UCHAR)hci_to_lmp_packet_type(tx_pkt);
                                esco_conn_entity_ptr->s_to_m_packet_length =
                                    tx_pkt_length;
                            }
                            else
                            {
                                esco_conn_entity_ptr->s_to_m_packet_type =
                                    (UCHAR)hci_to_lmp_packet_type(rx_pkt);
                                esco_conn_entity_ptr->s_to_m_packet_length =
                                    rx_pkt_length;
                                esco_conn_entity_ptr->m_to_s_packet_type =
                                    (UCHAR)hci_to_lmp_packet_type(tx_pkt);
                                esco_conn_entity_ptr->m_to_s_packet_length =
                                    tx_pkt_length;
                            }

                            ESCO_INF(TX_PKT_LENGTH_RX_PKT_LENGTH,2,tx_pkt_length,rx_pkt_length);
                            ESCO_INF(TESCO_RETX_WINDOW,2, tesco,retx_window);

                            RT_BT_LOG(GRAY, LMP_SYNC_LINKS_3119, 0, 0);


                            return API_SUCCESS;
                        } /* if lmp_check_sync_conn_bandwidth */
#ifndef _CCH_SLOT_OFFSET_
						
                    } /* if select_desco_val()*/
                    else
                    {
                        //RT_BT_LOG(GRAY, LMP_SYNC_LINKS_3126, 0, 0);
                    }
#endif
					

                } /* if select = TRUE */

            } /* end of for loop (rx_packet_index) */

        } /* end of for loop (tx_packet_index)*/

        max_latency -= 2;
    }

    RT_BT_LOG(GRAY, LMP_SYNC_LINKS_3138, 0, 0);

    return API_FAILURE;
}

/**
 * Get the size of the \a pkt_type in slots.
 *
 * \param pkt_type The input packet type.
 *
 * \return Number of slots occupied by the \a pkt_type.
 */
UINT16 esco_pktsize(UINT16 pkt_type)
{
    if ( (pkt_type == HCI_EV3) || (pkt_type == HCI_2_EV3) ||
            (pkt_type == HCI_3_EV3) )
    {
        return 1;
    }
    else if ( (pkt_type == HCI_EV4) || (pkt_type == HCI_EV5)||
              (pkt_type == HCI_2_EV5)|| (pkt_type == HCI_3_EV5) )
    {
        return 3;
    }
    else
    {
        return 1;
    }
}

/**
 * Get the maximum allowed length for a packet type in number of bytes.
 *
 * \param pkt_type The input packet type.
 *
 * \return Maximum number of bytes supported by the \a pkt_type.
 */
UINT16 max_allowed_length(UINT16 pkt_type)
{
    if(pkt_type == HCI_EV3)
    {
        return MAX_NO_OF_BYTES_IN_EV3;
    }
    else if(pkt_type == HCI_EV4)
    {
        return MAX_NO_OF_BYTES_IN_EV4;
    }
    else if(pkt_type == HCI_EV5)
    {
        return MAX_NO_OF_BYTES_IN_EV5;
    }
    else if(pkt_type == HCI_2_EV3)
    {
        return MAX_NO_OF_BYTES_IN_2_EV3;
    }
    else if(pkt_type == HCI_3_EV3)
    {
        return MAX_NO_OF_BYTES_IN_3_EV3;
    }
    else if(pkt_type == HCI_2_EV5)
    {
        return MAX_NO_OF_BYTES_IN_2_EV5;
    }
    else if(pkt_type == HCI_3_EV5)
    {
        return MAX_NO_OF_BYTES_IN_3_EV5;
    }
    else
    {
        return 0;
    }
}

/**
 * Get the LMP packet type equivalent of the HCI \a packet_type provided.
 *
 * \param packet_type The input HCI packet type.
 *
 * \return The LMP packet type equivalend of the HCI \a packet_type provided.
 */
UINT16 hci_to_lmp_packet_type(UINT16 packet_type)
{
    if(packet_type == HCI_EV3)
    {
        return LMP_EV3;
    }
    else if(packet_type == HCI_EV4)
    {
        return LMP_EV4;
    }
    else if(packet_type == HCI_EV5)
    {
        return LMP_EV5;
    }
    else if(packet_type == HCI_2_EV3)
    {
        return LMP_2_EV3;
    }
    else if(packet_type == HCI_3_EV3)
    {
        return LMP_3_EV3;
    }
    else if(packet_type == HCI_2_EV5)
    {
        return LMP_2_EV5;
    }
    else if(packet_type == HCI_3_EV5)
    {
        return LMP_3_EV5;
    }
    else
        return LMP_DEFAULT_ESCO_PKT;

}

/**
 * Get the HCI packet type equivalent of the LMP \a packet_type provided.
 *
 * \param packet_type The input LMP packet type.
 *
 * \return The HCI packet type equivalend of the LMP \a packet_type provided.
 */
UINT16 lmp_to_hci_packet_type(UINT16 packet_type)
{
    if(packet_type == LMP_EV3)
    {
        return HCI_EV3;
    }
    else if(packet_type == LMP_EV4)
    {
        return HCI_EV4;
    }
    else if(packet_type == LMP_EV5)
    {
        return HCI_EV5;
    }
    else if(packet_type == LMP_2_EV3)
    {
        return HCI_2_EV3;
    }
    else if(packet_type == LMP_3_EV3)
    {
        return HCI_3_EV3;
    }
    else if(packet_type == LMP_2_EV5)
    {
        return HCI_2_EV5;
    }
    else if(packet_type == LMP_3_EV5)
    {
        return HCI_3_EV5;
    }
    else
        return LMP_DEFAULT_ESCO_PKT;
}

/**
 * Release the AM_ADDR/LT_ADDR and connection entities used by the esco
 * connection.
 *
 * \param ce_index The ACL connection entity index.
 * \param esco_ce_index The ESCO connection entity index.
 *
 * \return None.
 */
void lmp_release_esco_resources(UINT16 ce_index, UINT16 esco_ce_index)
{
    UCHAR am_addr, piconet_id;
    LMP_CONNECTION_ENTITY *ce_ptr;

    ce_ptr = &lmp_connection_entity[ce_index];

    piconet_id = ce_ptr->phy_piconet_id;

    /* Free the connection entitties and the am_address for this connection */
    if(ce_ptr->remote_dev_role == SLAVE)
    {
        lmp_put_esco_am_addr(
            lmp_esco_connection_entity[esco_ce_index].lt_addr, piconet_id);
    }
    else
    {
        am_addr = lmp_esco_connection_entity[esco_ce_index].lt_addr;

        lmp_slave_unuse_esco_am_addr_ppi(am_addr, ce_ptr->phy_piconet_id);
    }

    lmp_release_esco_entity_to_ce_database(esco_ce_index);

    return;
}

/**
 * Generate the synchronous connection complete event when the esco connection
 * setup fails and release the resources.
 *
 * \param esco_ce_index The ESCO connection entity index.
 * \param ce_index The ACL connection entity index.
 * \param status The reason for failure.
 *
 * \return None.
 */
void lmp_handle_esco_conn_setup_failed(UINT16 esco_ce_index,
                                       UINT16 ce_index, UCHAR status)
{
    /* Send synchronous connection complete event */
    hci_generate_synchronous_conn_complete_event(ce_index,
            esco_ce_index, ESCO_LINK, status);

    /* release the esco connection entity and Lt_addr */
    lmp_release_esco_resources(ce_index, esco_ce_index);
}

/**
 * Detach all the esco links associated with the ACL link given by \a
 * ce_index.
 *
 * \param ce_index The ACL connection entity index.
 * \param reason The reason for disconnection.
 *
 * \return None.
 */
void lmp_disconnect_esco_links(UINT16 ce_index,UCHAR reason)
{
    UCHAR index;
    LMP_CONNECTION_ENTITY* ce_ptr;

    for(index = 0; index < LMP_MAX_ESCO_CONN_ENTITIES; index++)
    {
        if((lmp_esco_connection_entity[index].status != NO_CONNECTION) &&
                (lmp_esco_connection_entity[index].ce_index == ce_index))
        {
            switch (lmp_esco_connection_entity[index].status)
            {
                case CHANGING_ESCO_PARAMS:         /* Fall through */
                case ESCO_DISCONNECTING:           /* Fall through */
                case ESCO_CONNECTED:
                    hci_generate_disconnection_complete_event(
                        HCI_COMMAND_SUCCEEDED,
                        lmp_esco_connection_entity[index].conn_handle,
                        reason);

                    /* Killing baseband level connection */
                    lc_kill_esco_connection(index);
                    ce_ptr = &lmp_connection_entity[ce_index];
#ifdef _BRUCE_IMPLEMENTED_NO_OF_ESCO_CONN_
                    if ( ce_ptr->no_of_esco_connections == 0)
#else
                    if (lmp_self_device_data.number_of_esco_connections == 0)

#endif                    
                    {
#ifndef _USE_ONE_NEW_BZDMA_TXCMD_FOR_SCO_                    
                        bzdma_invalid_txcmd(BZDMA_TX_ENTRY_TYPE_SCO, 0, index);
#else
                        bzdma_invalid_txcmd(BZDMA_TX_ENTRY_TYPE_NEW_SCO0, 0, index);
#endif
                    }

                    lc_reset_esco_scheduler((UCHAR)ce_index, index);

                    /* Update the max and min values of tesco */
                    lmp_update_max_tesco();
                    lmp_update_gcd_of_tesco();

#ifdef _CCH_SLOT_OFFSET_
                    lmp_put_global_slot_offset(ce_index, 
                                          LMP_MAX_CE_DATABASE_ENTRIES + index);
#endif					
                    break;

                case ADDING_ESCO_LINK:             /* Fall through */
                case WAITING_FOR_ACCEPT_SYNC_CONN:
                    hci_generate_synchronous_conn_complete_event(ce_index,
                            index, ESCO_LINK, reason);
                    break;

                default:
                    break;
            }

            lmp_release_esco_resources(ce_index, index);
        }
    }
    hci_check_and_enable_eir_recv();
}

/**
 * Get the index of ESCO connection which is in \a status state and
 * corresponding to the \a ce_index (ACL connection index).
 *
 * \param ce_index ACL connection entity index.
 * \param status Status of the ESCO connection (associated with the ACL).
 * \param sco_ce_index Index to the ESCO connection database.
 *
 * \return API_SUCCESS, if the operation if successful. API_FAILURE,
 *         otherwise.
 *
 * \warning If two or more ESCO connections associated with the same ACL
 *          connection (\a ce_index) are in the same state (\a status), any
 *          one of the ESCO connection index is returned and no particular
 *          order should be assumed.
 */
API_RESULT lmp_get_esco_ce_index_from_ce_index(const UINT16 ce_index,
        const ESCO_CE_CONN_STATUS status, UINT16* esco_ce_index)
{
    UINT16 i;

    BZ_ASSERT(esco_ce_index != NULL, "Where will I store the result:-(");

    for (i = 0; i < LMP_MAX_ESCO_CONN_ENTITIES; i++)
    {
        if (lmp_esco_connection_entity[i].ce_index == ce_index
                && lmp_esco_connection_entity[i].status == status)
        {
            *esco_ce_index = i;
            return API_SUCCESS;
        }
    }

    return API_FAILURE;
}

/**
 * Get ESCO connection entity index from the the ESCO handle.
 *
 * \param esco_handle The ESCO handle.
 * \param ce_index The ACL connection entity index.
 *
 * \return API_SUCCESS, if the operation is successful. API_FAILURE, otherwise.
 */
API_RESULT lmp_get_esco_ce_index_from_esco_handle(UCHAR esco_handle,
        UINT16* ce_index)
{
    UCHAR index = 0;

    if(esco_handle > 0)
    {
        while(index < LMP_MAX_ESCO_CONN_ENTITIES)
        {
            if((lmp_esco_connection_entity[index].status != NO_CONNECTION) &&
                    (lmp_esco_connection_entity[index].esco_handle == esco_handle))
            {
                *ce_index = index;
                return API_SUCCESS;
            }
            index++;
        }
    }
    return API_FAILURE;
}

/**
 * Reset the temporary connection entity.
 *
 * \param None.
 *
 * \return None.
 */
void lmp_reset_temp_esco_connection_entity(void)
{
#if 1
    memset(&temp_esco_connection_entity, 0, sizeof(LMP_ESCO_CONNECTION_ENTITY));

    temp_esco_connection_entity.esco_window_status = INACTIVE;
    temp_esco_connection_entity.esco_continous_status = INACTIVE;
#else
    temp_esco_connection_entity.entity_status = UNASSIGNED;
    temp_esco_connection_entity.status = NO_CONNECTION;
    temp_esco_connection_entity.ce_index = 0x00;
    temp_esco_connection_entity.esco_handle = 0x00;
    temp_esco_connection_entity.lt_addr = 0x00;
    temp_esco_connection_entity.desco = 0x00;
    temp_esco_connection_entity.tesco = 0x00;
    temp_esco_connection_entity.wesco = 0x00;
    temp_esco_connection_entity.esco_window = 0x00;
    temp_esco_connection_entity.m_to_s_packet_type = 0x00;
    temp_esco_connection_entity.s_to_m_packet_type = 0x00;
    temp_esco_connection_entity.air_mode = 0x00;
    temp_esco_connection_entity.retransmission_effort = 0x00;
    temp_esco_connection_entity.negotiation_flag = 0x00;
    temp_esco_connection_entity.negotiation_count = 0x00;
    temp_esco_connection_entity.timing_control_flags = 0x00;
    temp_esco_connection_entity.packet_type = 0x00;
    temp_esco_connection_entity.conn_handle = 0x00;
    temp_esco_connection_entity.m_to_s_packet_length= 0x00;
    temp_esco_connection_entity.s_to_m_packet_length= 0x00;
    temp_esco_connection_entity.max_latency = 0x00;
    temp_esco_connection_entity.voice_setting = 0x00;
    temp_esco_connection_entity.next_instant = 0x00;
    temp_esco_connection_entity.tx_bandwidth = 0x00;
    temp_esco_connection_entity.rx_bandwidth = 0x00;
    temp_esco_connection_entity.no_data_count = 0;
    temp_esco_connection_entity.num_of_completed_esco_packets = 0x00;
    memset(&temp_esco_connection_entity.esco_data_q,0,
           sizeof(LMP_ESCO_DATA_Q));
    temp_esco_connection_entity.esco_window_status = INACTIVE;
    temp_esco_connection_entity.esco_continous_status = INACTIVE;
    temp_esco_connection_entity.host_pkt_empty_cnts = 0;
#endif

    lmp_reset_negotiation_history();

}

/**
 * Update the connection entity of the esco link which was changed.
 *
 * \param ce_index The ACL connection entity index.
 * \param esco_ce_index The ESCO connection entity index.
 *
 * \return None.
 */
void update_esco_connection_entity(UINT16 ce_index, UINT16 esco_ce_index)
{
    UCHAR am_addr;

    LMP_ESCO_CONNECTION_ENTITY *esco_ce_ptr;
    LMP_CONNECTION_ENTITY *ce_ptr;

    ce_ptr = &lmp_connection_entity[ce_index];
    esco_ce_ptr = &lmp_esco_connection_entity[esco_ce_index];

    if( esco_ce_ptr->lt_addr != temp_esco_connection_entity.lt_addr )
    {
        ESCO_INF(CHANGING_ESCO_LT_ADDR_OLD_LT_ADDR_NEW_LT_ADDR,2,
                 esco_ce_ptr->lt_addr,
                 temp_esco_connection_entity.lt_addr);

        if(ce_ptr->remote_dev_role == SLAVE)
        {
            lmp_put_esco_am_addr(esco_ce_ptr->lt_addr, ce_ptr->phy_piconet_id);
        }
        else
        {
            am_addr = esco_ce_ptr->lt_addr;

            lmp_slave_unuse_esco_am_addr_ppi(am_addr, ce_ptr->phy_piconet_id);
        }
    }

    if(lmp_local_dev_init_esco_param_change == TRUE)
    {
        esco_ce_ptr->tx_bandwidth = temp_esco_connection_entity.tx_bandwidth;
        esco_ce_ptr->rx_bandwidth = temp_esco_connection_entity.rx_bandwidth;
        esco_ce_ptr->max_latency = temp_esco_connection_entity.max_latency;
        esco_ce_ptr->voice_setting = temp_esco_connection_entity.voice_setting;
        esco_ce_ptr->retransmission_effort =
            temp_esco_connection_entity.retransmission_effort;
        esco_ce_ptr->packet_type = temp_esco_connection_entity.packet_type;
    }

    esco_ce_ptr->lt_addr =
        temp_esco_connection_entity.lt_addr;
    esco_ce_ptr->esco_handle = temp_esco_connection_entity.esco_handle;
    esco_ce_ptr->timing_control_flags =
        temp_esco_connection_entity.timing_control_flags;
    esco_ce_ptr->desco = temp_esco_connection_entity.desco ;
    esco_ce_ptr->tesco = temp_esco_connection_entity.tesco ;
    esco_ce_ptr->wesco = temp_esco_connection_entity.wesco ;
    esco_ce_ptr->m_to_s_packet_type =
        temp_esco_connection_entity.m_to_s_packet_type ;
    esco_ce_ptr->s_to_m_packet_type =
        temp_esco_connection_entity.s_to_m_packet_type ;
    esco_ce_ptr->m_to_s_packet_length =
        temp_esco_connection_entity.m_to_s_packet_length ;
    esco_ce_ptr->s_to_m_packet_length =
        temp_esco_connection_entity.s_to_m_packet_length ;
    esco_ce_ptr->esco_window = temp_esco_connection_entity.esco_window;

#if 0
    RT_TMP_LOG("update_esco_connection_entity: T:%d, W:%d, type:0x%x, len:%d\n",
               esco_ce_ptr->tesco,
               esco_ce_ptr->wesco,
               esco_ce_ptr->m_to_s_packet_type,
               esco_ce_ptr->m_to_s_packet_length);
#endif

    return;
}

/**
 * Free the esco buffers which were still being used at the time of reset.
 *
 * \param esco_ce_index The ESCO connection entity index.
 *
 * \return None.
 */
void reset_esco_ce_buffer(UINT16 esco_ce_index)
{
    UCHAR read_index=0;

    read_index = lmp_esco_connection_entity[esco_ce_index].esco_data_q.wait_index;

    /* Check if the qlength is 0*/
    while(lmp_esco_connection_entity[esco_ce_index].esco_data_q.wait_cnt)
    {
        /* Free the esco buffer */
        if (dma_tx_fifo_pkt_free(
                    (void *)lmp_esco_connection_entity[esco_ce_index].
                    esco_data_q.esco_data_pkt_Q[read_index],
                    HCI_TRANSPORT_SYNC_DATA_PKT_TYPE) != BT_ERROR_OK)
        {
            RT_BT_LOG(GRAY, LMP_SYNC_LINKS_3649, 0, 0);
        }

        ESCO_INF(FREEING_BUFF,1,
                 lmp_esco_connection_entity[esco_ce_index].
                 esco_data_q.esco_data_pkt_Q[read_index]);

        lmp_esco_connection_entity[esco_ce_index].
                            esco_data_q.esco_data_pkt_Q[read_index] = NULL;

        read_index++;

        if(read_index == MAX_ESCO_DATA_Q_LENGTH)
        {
            read_index = 0;
        }

        lmp_esco_connection_entity[esco_ce_index].esco_data_q.wait_cnt--;
    }
}

/**
 * Reset all the global variables used by esco feature.
 *
 * \param None.
 *
 * \return None.
 */
void lmp_reset_esco_global_variables(void)
{  
    /* Set the number of esco connections to 0 */
    lmp_self_device_data.number_of_esco_connections = 0;

    lmp_local_dev_init_esco_param_change = FALSE ;

    gcd_of_tesco = 0;

    max_tesco = 0;

    num_sync_links_over_codec = 0;

/*  Updated by Wallice Su.  2013/07/08  */
#if 0
    if ((otp_str_data.hci_excodec_state == 0x1) || (otp_str_data.hci_excodec_state == 0x2))
#else
    if ((hci_excodec_state == 0x1) || (hci_excodec_state == 0x2))

#endif
    {
#if 0    
    	 if (otp_str_data.hci_excodec_state == 0x1)
    	 {
        	RT_BT_LOG(RED,LC_SYNC_LINKS_293,0,0);
    	 }
	     else
	     {
	 	    RT_BT_LOG(RED,MSG_CODEC_I2S_0, 0, 0);
	     }
#endif
        lmp_esco_over_codec = TRUE;
    }
    else
    {
        RT_BT_LOG(RED,LC_SYNC_LINKS_297,0,0);
        lmp_esco_over_codec = FALSE;
    }

/*  End Updated by Wallice Su.  2013/07/08  */

}


#ifndef COMPILE_SINGLE_SLOT_PACKETS_ONLY
/**
 * Update the max slot value on the acl link, whenever there is a change in
 * the esco link.
 *
 * \param acl_ce_index The ACL connection entity index.
 *
 * \return None.
 */
void change_max_slot_with_esco(UINT16 acl_ce_index)
{
    /* Queue max-slot req pdu here. */
    lmp_send_max_slot_pdu_to_all_devices();

    lc_update_pkts_allowed(acl_ce_index);

    return;
}
#endif


/**
 * Check if the reserved slots of synchronous connections occupy the whole
 * bandwidth.
 *
 * \param consider_wesco Should we have to consider Wesco while checking for
 *                       available bandwidth.
 *
 * \return API_SUCCESS, if there is enough bandwidth. API_FAILURE, otherwise.
 */
API_RESULT lmp_check_sync_conn_bandwidth(UCHAR consider_wesco)
{
    UCHAR index=0;
    UCHAR esco_slots =0;
    UCHAR num_reps_in_max_tesco_slots = 0;
    UCHAR max_tesco_val = 0;
    UCHAR occupied_slots;

    for(index = 0; index < LMP_MAX_ESCO_CONN_ENTITIES; index++)
    {
        if(lmp_esco_connection_entity[index].status != NO_CONNECTION)
        {
            if(lmp_esco_connection_entity[index].tesco > max_tesco_val)
            {
                max_tesco_val = lmp_esco_connection_entity[index].tesco;
            }
        }
    }

    for(index=0; index < LMP_MAX_ESCO_CONN_ENTITIES; index++)
    {
        if((lmp_esco_connection_entity[index].status != NO_CONNECTION))
        {
            num_reps_in_max_tesco_slots = (UCHAR)(max_tesco_val /
                                                  lmp_esco_connection_entity[index].tesco);

            occupied_slots = lmp_esco_connection_entity[index].esco_window;
            if (!consider_wesco)
            {
                occupied_slots = (UCHAR)(occupied_slots -
                                         lmp_esco_connection_entity[index].wesco);
            }
            /* esco slots is number of slots occupied by the reserved slots
               of all esco links
            */
            esco_slots = (UCHAR) (esco_slots + (num_reps_in_max_tesco_slots *
                                                occupied_slots));
        }
    }

    /* If Max. Tesco - esco_slots < 2 , it implies there is not enough
     * free bandwidth.
     */
    if((max_tesco_val - esco_slots) < 2)
    {
        ESCO_ERR(NO_FREE_BANDWIDTH,0,0);
        return API_FAILURE;
    }
    else
        return API_SUCCESS;
}

/**
 * Check if the window specified by \a start_of_new_window and \a
 * end_of_new_window overlaps the esco reserved slots on the acl connection
 * over acl_ce_index.
 *
 * \param start_of_new_window Start of ESCO window.
 * \param end_of_new_window End of ESCO windows.
 * \param acl_ce_index The ACL connection entity index.
 *
 * \return API_SUCCESS, if the esco window doesn't overlap. API_FAILURE,
 *         otherwise.
 */
API_RESULT lmp_check_res_slot_overlap(UCHAR start_of_new_window,
                                      UCHAR end_of_new_window, UINT16 acl_ce_index)
{
    UCHAR index = 0,rep_index = 0;
    UCHAR start_of_esco_win = 0,end_of_esco_win = 0;
    UCHAR num_reps_in_max_tesco_slots = 0;

    for(index = 0; index < LMP_MAX_ESCO_CONN_ENTITIES; index++)
    {
        if(lmp_esco_connection_entity[index].status != NO_CONNECTION)
        {

            start_of_esco_win = (UCHAR)(lmp_esco_connection_entity[index].desco %
                                        lmp_esco_connection_entity[index].tesco);

            /* For Max slot calulation only reserved slots are considered */

            end_of_esco_win = (UCHAR)(start_of_esco_win +
                                      lmp_esco_connection_entity[index].esco_window);

            num_reps_in_max_tesco_slots = (UCHAR)(max_tesco /
                                                  lmp_esco_connection_entity[index].tesco);

            for(rep_index = 0; rep_index < num_reps_in_max_tesco_slots;
                    rep_index++)
            {
                /* Check if the window overlaps esco slots
                   conditions checked are ,
                                           1) If the start of window lies in
                                              the esco reserved slots
                                           2) If the end of window lies inside
                                              the esco reserved slots
                                           3) If the window completely overlaps
                                              the esco reserved slots
                */
                if( ((start_of_new_window >= start_of_esco_win) &&
                        (start_of_new_window < end_of_esco_win)) ||
                        ((end_of_new_window > start_of_esco_win) &&
                         (end_of_new_window <= end_of_esco_win))   ||
                        ((start_of_new_window < start_of_esco_win) &&
                         (end_of_new_window >= end_of_esco_win))
                  )
                {
                    return API_FAILURE;
                }

                start_of_esco_win = (UCHAR)(start_of_esco_win +
                                            lmp_esco_connection_entity[index].tesco);
                end_of_esco_win = (UCHAR)(end_of_esco_win +
                                          lmp_esco_connection_entity[index].tesco);
            }

        }
    }

    return API_SUCCESS;
}

/**
 * Find the packet type to be used while negotiating for esco link. The
 * allowed packet types will be chosen from the remote device's features and
 * from the packet types specified by the local host.
 *
 * \param acl_ce_index The ACL connection entity index.
 * \param local_host_packet_type The local host packet type.
 *
 * \return The allowed packet types.
 */
UINT16 lmp_get_allowed_synchronous_pkt_type(UINT16 acl_ce_index,
        UINT16 local_host_packet_type)
{
    UCHAR feature;
    UINT16 conn_packet_type = 0x0;

    feature = lmp_connection_entity[acl_ce_index].feat_page0[3];

    if( (feature & LMP_ESCO_MANDATORY_FEATURE) &&
            (local_host_packet_type & HCI_EV3) )
    {
        conn_packet_type |= HCI_EV3;
    }

#ifdef _CCH_ESCO_SCA_
    feature = lmp_connection_entity[acl_ce_index].feat_page0[5];

    if( (feature & EDR_ESCO_2MBPS_FEATURE) &&
            ( ! (local_host_packet_type & HCI_2_EV3) ) )
    {
        conn_packet_type |= HCI_2_EV3;
    }

    if( (feature & EDR_ESCO_3MBPS_FEATURE) &&
            ( ! (local_host_packet_type & HCI_3_EV3) ) )
    {
        conn_packet_type |= HCI_3_EV3;
    }
#endif

	
    if(lc_check_if_device_is_in_scatternet() == FALSE)
    {
        feature = lmp_connection_entity[acl_ce_index].feat_page0[4];

        if( (feature & LMP_ESCO_EV4_PACKET_FEATURE) &&
                (local_host_packet_type & HCI_EV4) )
        {
            conn_packet_type |= HCI_EV4;
        }

        if( (feature & LMP_ESCO_EV5_PACKET_FEATURE) &&
                (local_host_packet_type & HCI_EV5) )
        {
            conn_packet_type |= HCI_EV5;
        }

        feature = lmp_connection_entity[acl_ce_index].feat_page0[5];

        if( (feature & EDR_ESCO_2MBPS_FEATURE) &&
                ( ! (local_host_packet_type & HCI_2_EV3) ) )
        {
            conn_packet_type |= HCI_2_EV3;
        }

        if( (feature & EDR_ESCO_3MBPS_FEATURE) &&
                ( ! (local_host_packet_type & HCI_3_EV3) ) )
        {
            conn_packet_type |= HCI_3_EV3;
        }

        if( (feature & EDR_ESCO_2MBPS_FEATURE) &&
                (feature & EDR_ESCO_3_SLOT_FEATURE) &&
                ( ! (local_host_packet_type & HCI_2_EV5) ) )
        {
            conn_packet_type |= HCI_2_EV5;
        }

        if( (feature & EDR_ESCO_3MBPS_FEATURE) &&
                (feature & EDR_ESCO_3_SLOT_FEATURE) &&
                ( ! (local_host_packet_type & HCI_3_EV5) ) )
        {
            conn_packet_type |= HCI_3_EV5;
        }
    }

    ESCO_INF(HOST_PKT_TYPE,1,local_host_packet_type);
    ESCO_INF(CONNECTION_PKT_TYPE,1,conn_packet_type);
//    RT_BT_LOG(GREEN, CCH_DBG_014, 2,local_host_packet_type, conn_packet_type);

    return conn_packet_type;
}
#ifndef _CCH_SLOT_OFFSET_
/**
 * Select the Desco value for the esco link which is being established.
 *
 * \param esco_ce_index The ESCO connection entity index.
 * \param ce_index The ACL connection entity index.
 * \param desco The ESCO offset returned.
 *
 * \return BT_FW_SUCCESS, if the operation is successful. BT_FW_ERROR,
 *         otherwise.
 */
UCHAR select_desco_val(UINT16 esco_ce_index,UCHAR ce_index,UCHAR *desco)
{
    UCHAR end_of_window = 0,start_of_window = 0;
    UCHAR retx_overlap = 0;
    UCHAR new_gcd_of_tesco;


    if(lmp_self_device_data.number_of_esco_connections == 0)
    {
        *desco = 0;
        return BT_FW_SUCCESS;
    }


    new_gcd_of_tesco = (UCHAR)lmp_find_gcd(lmp_esco_connection_entity
                                           [esco_ce_index].tesco,gcd_of_tesco);

    /* Setting the initial value of desco to 0 */
    start_of_window = 0;
    end_of_window = (UCHAR)(start_of_window +
                            lmp_esco_connection_entity[esco_ce_index].esco_window);

    /* Check if the desco value can be chosen such that it does not overlap
       any of the esco windows, including retransmission window.
       retx_overlap is set to TRUE indicating , retx windows should not be
       overridden
    */
    retx_overlap = FALSE;
    while(end_of_window <= max_tesco)
    {
        /* Check if the window overlaps the esco window of existing esco link*/
        if(check_esco_window_overlap(start_of_window,end_of_window,ce_index,
                                     esco_ce_index,retx_overlap,
                                     lmp_esco_connection_entity[esco_ce_index].
                                     tesco,new_gcd_of_tesco) == API_SUCCESS)
        {
            *desco = start_of_window;
            return BT_FW_SUCCESS;
        };

        start_of_window += 2;
        end_of_window += 2;
    }


    start_of_window = 0;
    end_of_window = (UCHAR)(start_of_window +
                            lmp_esco_connection_entity[esco_ce_index].esco_window);

    /* Desco could not be chosen without overriding the retransmission windows,
        now try to select desco by overriding the esco retransmission windows.
        If the new esco link is to the same device then retransmission window
        cannot be overridden.
        retx_overlap is set to TRUE indicating , retx windows can be
        overridden
     */
    retx_overlap = TRUE;

    while(end_of_window <= max_tesco)
    {
        if(check_esco_window_overlap(start_of_window,end_of_window,ce_index,
                                     esco_ce_index,retx_overlap,
                                     lmp_esco_connection_entity[esco_ce_index].
                                     tesco,new_gcd_of_tesco)
                == API_SUCCESS)
        {
            *desco = start_of_window;
            return BT_FW_SUCCESS;
        };

        start_of_window += 2;
        end_of_window += 2;
    }

    return BT_FW_ERROR;
}
#endif
/**
 * Check if the window specified by \a start_of_new_window and \a
 * end_of_new_window overlaps the ESCO slots. It will check whether the window
 * overlaps the esco winfdow, if the esco link is to the same device or only
 * the reserved slots if the link is being created to a new link.
 *
 * \param start_of_new_window Start of ESCO window.
 * \param end_of_new_window End of ESCO window.
 * \param acl_ce_index The ACL connection entity index.
 * \param esco_ce_index The ESCO connection entity index.
 * \param retx_overlap TRUE / FALSE.
 * \param tesco The ESCO interval.
 * \param new_gcd_of_tesco The new Greatest Common Divisor of ESCO interval.
 *
 * \return API_SUCCESS, if the operation is successful. API_FAILURE, otherwise.
 */
API_RESULT check_esco_window_overlap(UCHAR start_of_new_window,
                                     UCHAR end_of_new_window, UINT16 acl_ce_index, UINT16 esco_ce_index,
                                     UCHAR retx_overlap, UCHAR tesco, UCHAR new_gcd_of_tesco)
{
    UCHAR index = 0,rep_index = 0;
    UCHAR start_of_esco_win = 0;
    UCHAR end_of_esco_win = 0;
    UCHAR esco_window_size = 0,desco_diff = 0;
    UCHAR num_reps_in_max_tesco_slots = 0;
    UCHAR new_esco_window_size = 0;


    new_esco_window_size = (UCHAR)(end_of_new_window - start_of_new_window);

    for(index = 0; index < LMP_MAX_ESCO_CONN_ENTITIES; index++)
    {
        if((lmp_esco_connection_entity[index].status != NO_CONNECTION) &&
                (index != esco_ce_index))
        {
            start_of_esco_win = (UCHAR)(lmp_esco_connection_entity[index].desco %
                                        lmp_esco_connection_entity[index].tesco);

            if(retx_overlap == FALSE)
            {
                end_of_esco_win = (UCHAR)(start_of_esco_win +
                                          lmp_esco_connection_entity[index].
                                          esco_window);
            }
            else
            {
                /* If the new esco link is to the same device , then retx
                   window of the existing esco link cannot be overridden.
                */

                if(lmp_esco_connection_entity[index].ce_index == acl_ce_index)
                {
                    end_of_esco_win = (UCHAR)(start_of_esco_win +
                                              lmp_esco_connection_entity[index].
                                              esco_window);
                }
                else
                {
                    end_of_esco_win = (UCHAR)(start_of_esco_win +
                                              (lmp_esco_connection_entity[index].
                                               esco_window -
                                               lmp_esco_connection_entity[index].wesco));
                }
            }

            esco_window_size = (UCHAR)(end_of_esco_win - start_of_esco_win);

            /* If the sum of esco windows is larger than the gcd of tesco
               values, then the new esco window will overlap this current
               esco link.
            */
            if((esco_window_size + new_esco_window_size) > new_gcd_of_tesco)
            {
                return API_FAILURE;
            }

            /* If tesco of the new link and this esco link are not the same,
               then the difference between their Desco values should not be
               a mulitple of the gcd.
            */
            if(lmp_esco_connection_entity[index].tesco != tesco)
            {
                desco_diff = (UCHAR)((start_of_new_window > start_of_esco_win) ?
                                     start_of_new_window - start_of_esco_win :
                                     start_of_esco_win - start_of_new_window);

                if((desco_diff % new_gcd_of_tesco) == 0)
                {
                    return API_FAILURE;
                }
            }

            num_reps_in_max_tesco_slots = (UCHAR)(max_tesco /
                                                  lmp_esco_connection_entity[index].tesco);

            for(rep_index = 0; rep_index < num_reps_in_max_tesco_slots;
                    rep_index++)
            {
                /* Check if the window overlaps esco slots
                     conditions checked are,
                                           1) If the start of window lies in
                                              the esco reserved slots
                                           2) If the end of window lies inside
                                              the esco reserved slots
                                           3) If the window completely overlaps
                                              the esco reserved slots
                */

                if( ((start_of_new_window >= start_of_esco_win)&&
                        (start_of_new_window < end_of_esco_win)) ||
                        ((end_of_new_window > start_of_esco_win) &&
                         (end_of_new_window <= end_of_esco_win))   ||
                        ((start_of_new_window < start_of_esco_win) &&
                         (end_of_new_window >= end_of_esco_win))
                  )
                {
                    return API_FAILURE;
                }

                start_of_esco_win = (UCHAR)(start_of_esco_win +
                                            lmp_esco_connection_entity[index].tesco);
                end_of_esco_win = (UCHAR)(end_of_esco_win +
                                          lmp_esco_connection_entity[index].tesco);
            }
        }
    }

    return API_SUCCESS;
}

/**
 * Update the maximum Tesco(ESCO interval) value
 *
 * \param None.
 *
 * \return None.
 */
void lmp_update_max_tesco(void)
{
    UCHAR index;

    /* reset the value max_tesco */
    max_tesco = 0;

    for(index = 0; index < LMP_MAX_ESCO_CONN_ENTITIES; index++)
    {
        if((lmp_esco_connection_entity[index].status != NO_CONNECTION) &&
                (lmp_esco_connection_entity[index].tesco > max_tesco))
        {
            max_tesco = lmp_esco_connection_entity[index].tesco;
        }
    }
}

/**
 * Update the Greatest Common Divisor of the Tesco (ESCO interval) values.
 *
 * \param None.
 *
 * \return None.
 */
void lmp_update_gcd_of_tesco(void)
{
    UCHAR num1 = 0, num2 = 0;
    UCHAR index;
    switch(lmp_self_device_data.number_of_esco_connections)
    {
        case 0 :
            gcd_of_tesco = 0;
            break;

        case 1 :
            /* There is only one Esco link in the device. Tesco of that link
               will be the GCD.
            */

            for(index = 0; index < LMP_MAX_ESCO_CONN_ENTITIES ; index++)
            {
                if(lmp_esco_connection_entity[index].status != NO_CONNECTION)
                {
                    gcd_of_tesco = lmp_esco_connection_entity[index].tesco;
                }
            }
            break;

        default :
            for(index = 0; index < LMP_MAX_ESCO_CONN_ENTITIES ; index++)
            {
                /* This loop will find the GCD of Tesco of two links and then
                   finds the GCD with every other Tesco value.
                */
                if(lmp_esco_connection_entity[index].status != NO_CONNECTION)
                {
                    /* num2 is assigned the Tesco of the Esco links ,other than
                       the one assigned to num1 in each iteration.
                       When this is done num1 would be holding the Tesco value
                       of 1'st Esco link.
                    */
                    if(num1 != 0)
                    {
                        num2 =  lmp_esco_connection_entity[index].tesco;
                    }

                    /* num1 is assigned the Tesco of the 1'st Esco link
                       starting from connection Entity 0.
                    */
                    if((num1 == 0) && (num2 == 0))
                    {
                        num1 = lmp_esco_connection_entity[index].tesco;
                    }

                    /* Both num1 and num2 are assigned Tesco values.
                       Find the gcd of num1 and num2.
                    */
                    if((num1 != 0) && (num2 != 0))
                    {
                        num1 = (UCHAR)lmp_find_gcd(num1,num2);
                    }
                }
            }
            gcd_of_tesco = num1;
            break;
    }
}

/**
 * Find the GCD of two numbers.
 *
 * \param number1 The first number.
 * \param number2 The second number.
 *
 * \return The GCD of \a number1 and \a number2.
 */
UINT32 lmp_find_gcd(UINT32 number1, UINT32 number2)
{
    if((number1 != 0) && (number2 != 0))
    {
        while(number1 != number2)
        {
            if(number1 > number2)
            {
                number1 = (UINT32)(number1 - number2);
            }
            else
            {
                number2 = (UINT32)(number2 - number1);
            }
        }
        return number1;
    }
    return 0;
}


/**
  * Calculate no of slots occupied by packet.
  *
  * \param pkt_type  LMP
  *
  * \return No of slots occupied by pkt passed as
  *             argument.
  */
UINT16 lmp_find_packet_slots(UINT16 pkt_type)
{
    UINT16 no_of_slots = 0;

    switch(pkt_type)
    {
        case LMP_DEFAULT_ESCO_PKT :
            no_of_slots = 0;
            break;

        case LMP_EV3 :
        case LMP_2_EV3 :
        case LMP_3_EV3 :
            no_of_slots = 1;
            break;

        case LMP_EV4 :
        case LMP_EV5 :
        case LMP_2_EV5 :
        case LMP_3_EV5 :
            no_of_slots = 3;
            break;

    }

    return no_of_slots;
}



/**
  * Reset the negotiation history
  *
  * \param None
  *
  * \return None
  */
void lmp_reset_negotiation_history(void)
{

#if 1
    memset(&esco_negotiation_history, 0, sizeof(LMP_ESCO_NEGOTIATION_HISTORY));
#else
    esco_negotiation_history.possible_cause_of_rejection = 0;
    esco_negotiation_history.cause_to_constraint = 0;
    esco_negotiation_history.possible_disallowed_master_pkt = 0;
    esco_negotiation_history.possible_disallowed_slave_pkt = 0;
    esco_negotiation_history.max_latency = 0;
    esco_negotiation_history.retx_window = 0;
#endif
}


/**
  * Records the relevant negotiation history details.
  *
  * \param self_parameters Esco connection structure containing the
  *         the details for previous packet sent by the host device.
  *         Will refer to lmp_eco_connection_entity if ADDING_LINK
  *         or to temp_esco_connection_entity if CHANGING_LINK.
  * \param recieved_pdu    PDU recieved from the peer device.
  *
  * \return None
  */
void lmp_store_negotiation_history(LMP_ESCO_CONNECTION_ENTITY* self_parameters,
                                   LMP_PDU_PKT* recieved_pdu)
{
    UINT16 negotiation_state;
    UCHAR previous_constrained_causes;

    negotiation_state = recieved_pdu->payload_content[15];

    /** Eliminate causes which were considered in last sent PDU */
    esco_negotiation_history.possible_cause_of_rejection =
        (UCHAR) (esco_negotiation_history.possible_cause_of_rejection &
                 ~esco_negotiation_history.cause_to_constraint);

    previous_constrained_causes = esco_negotiation_history.cause_to_constraint;
    esco_negotiation_history.cause_to_constraint = 0;

    /** Consider possible causes which could have caused
      *   previous PDU to be rejected .
      */

    switch(negotiation_state)
    {
        case 2 :
            break;

        case 3 :
        {
            UINT16 temp_max_latency = 0;
            temp_max_latency = (UINT16) (self_parameters->
                                         wesco + self_parameters->tesco +
                                         lmp_find_packet_slots(self_parameters->
                                                 m_to_s_packet_type) +
                                         lmp_find_packet_slots(self_parameters->
                                                 s_to_m_packet_type));

            if((esco_negotiation_history.max_latency == 0) ||
                    (esco_negotiation_history.max_latency > temp_max_latency))
            {
                esco_negotiation_history.max_latency = temp_max_latency;
            }
        }
        break;
        default :
            esco_negotiation_history.possible_disallowed_master_pkt =
                (UINT16)( esco_negotiation_history.
                          possible_disallowed_master_pkt |
                          lmp_to_hci_packet_type(self_parameters->
                                                 m_to_s_packet_type) );
            esco_negotiation_history.possible_disallowed_slave_pkt =
                (UINT16)( esco_negotiation_history.
                          possible_disallowed_slave_pkt |
                          lmp_to_hci_packet_type(self_parameters->
                                                 s_to_m_packet_type) );

            /** If remote device has suggested any parameters
                   remove them from possible disallowed packets */
            esco_negotiation_history.possible_disallowed_master_pkt =
                (UINT16)(esco_negotiation_history.
                         possible_disallowed_master_pkt &
                         ~(lmp_to_hci_packet_type(recieved_pdu->
                                                  payload_content[8])) );

            esco_negotiation_history.possible_disallowed_slave_pkt =
                (UINT16) (esco_negotiation_history.
                          possible_disallowed_slave_pkt &
                          ~(lmp_to_hci_packet_type(recieved_pdu->
                                                   payload_content[9])) );

            esco_negotiation_history.possible_cause_of_rejection |=
                MASTER_PKT_CAUSE | SLAVE_PKT_CAUSE ;

            if(self_parameters->retransmission_effort == 0xFF)
            {
                /** If initial link_req_pdu contains 0 as retx_window
                try to constrain retx_window to be negotiated to 0 */
                if((esco_negotiation_history.retx_window == 0) &&
                        (recieved_pdu->payload_content[7] == 0))
                {
                    esco_negotiation_history.
                    possible_cause_of_rejection |=
                        RETX_EFFORT_CAUSE;
                }

                /** Store the largest retx window suggested by peer */
                if(recieved_pdu->payload_content[7] >
                        esco_negotiation_history.retx_window)
                {
                    esco_negotiation_history.retx_window =
                        recieved_pdu->payload_content[7];
                    esco_negotiation_history.
                    possible_cause_of_rejection |=
                        RETX_EFFORT_CAUSE;
                }
            }

            break;
    }

    /** Select causes to constrain in the next PDU to be generated */

    if((previous_constrained_causes & RETX_EFFORT_CAUSE) == 0 &&
            (esco_negotiation_history.possible_cause_of_rejection
             & RETX_EFFORT_CAUSE))
    {
        /** If the retx_effort cause had not been constrained earlier. */
        esco_negotiation_history.cause_to_constraint |= RETX_EFFORT_CAUSE;
    }
    else
    {
        if(esco_negotiation_history.possible_cause_of_rejection &
                MASTER_PKT_CAUSE)
        {
            esco_negotiation_history.cause_to_constraint |= MASTER_PKT_CAUSE;
        }

        if(esco_negotiation_history.possible_cause_of_rejection &
                SLAVE_PKT_CAUSE)
        {
            esco_negotiation_history.cause_to_constraint |= SLAVE_PKT_CAUSE;
        }
    }
}

/**
  * Records the relevant negotiation history details.
  *
  * \param max_latency Will be set to max_latency for which the peer
  *         device returned latency violation.
  *         If no violation had occured will be set to 0xFFFF.
  * \param disallowed_packet_type Will be set to those packets(in HCI type)
  *         which for the other device is renegotiating.
  * \param retx_effort    Will be set to estimated retransmission
  *         effort of the peer device.
  *
  * \return None
  */
void lmp_generate_constraints_from_history(UINT16* max_latency,
        UINT16* disallowed_packet_type, UCHAR* retx_effort)
{
    if(esco_negotiation_history.max_latency != 0)
    {
        *max_latency = esco_negotiation_history.max_latency;
    }
    else
    {
        *max_latency = 0xFFFF;
    }


    if(esco_negotiation_history.cause_to_constraint & RETX_EFFORT_CAUSE)
    {
        if(esco_negotiation_history.retx_window == 0)
        {
            *retx_effort = 0;
        }
        else
        {
            *retx_effort = 1;
        }
    }

    *disallowed_packet_type = 0;

    if(esco_negotiation_history.cause_to_constraint & MASTER_PKT_CAUSE)
    {
        *disallowed_packet_type = (UINT16) (*disallowed_packet_type |
                                            esco_negotiation_history.possible_disallowed_master_pkt);
    }

    if(esco_negotiation_history.cause_to_constraint & SLAVE_PKT_CAUSE)
    {
        *disallowed_packet_type = (UINT16)( *disallowed_packet_type |
                                            esco_negotiation_history.possible_disallowed_slave_pkt);
    }

    return;
}

/**
 * Find the packet length according to specified Tx and Rx bandwidth.
 * and maximum Tesco value
 *
 * \param tx_bandwidth     Transmission bandwidth .
 * \param rx_bandwidth     Recieve bandwidth.
 * \param tesco               Maximum allowed value of Tesco. Can be reset by callee
 *              to value less than original value if suitable packet length is found.
 * \param tx_pkt_length Will be set by callee to appropriate value if suitable
 *              combination of values is found.
 * \param rx_pkt_length Will be set by callee to appropriate value if suitable
 *              combination of values is found.
 *
 * \return API_SUCCESS, if the operation is successful. API_FAILURE, otherwise.
 */
API_RESULT lmp_find_packet_length(UINT32 tx_bandwidth, UINT32 rx_bandwidth,
                                  UCHAR* tesco, UINT16* tx_pkt_length, UINT16* rx_pkt_length)
{
    UINT16 slotno = 1600;
    UINT16 tx_gcd, rx_gcd;
    UINT16 cofactor_tx_num, cofactor_tx_denom;
    UINT16 cofactor_rx_num, cofactor_rx_denom;
    UINT16 lcm_of_cofactors = 0;

    if(tx_bandwidth==0)
    {
        cofactor_tx_denom = 0;
        cofactor_tx_num = 1;
    }
    else
    {
        /* Find GCD of bandwidth and no of slots(1600) */
        tx_gcd = (UINT16)lmp_find_gcd(slotno, tx_bandwidth);
        /* Find cofactors of 1600 and bandwidth with respect to their gcd.
          * This will make them into reduced form .
          */
        cofactor_tx_denom = (UINT16)(tx_bandwidth/tx_gcd);
        cofactor_tx_num = (UINT16)(slotno/tx_gcd);
    }

    if(rx_bandwidth==0)
    {
        cofactor_rx_denom = 0;
        cofactor_rx_num = 1;
    }
    else
    {
        rx_gcd = (UINT16)lmp_find_gcd(slotno, rx_bandwidth);

        cofactor_rx_denom = (UINT16)(rx_bandwidth/rx_gcd);
        cofactor_rx_num = (UINT16)(slotno/rx_gcd);
    }

    /* lcm of the slotNo cofactors */
    lcm_of_cofactors = (UINT16)( (cofactor_tx_num*cofactor_rx_num) /
                                 lmp_find_gcd(cofactor_tx_num, cofactor_rx_num) );


    if((lcm_of_cofactors > (*tesco))  || (lcm_of_cofactors==0))
    {
        return API_FAILURE;
    }

    /*Reduce tesco to closest original value and divisible by obtained lcm */
    *tesco = (UCHAR)((*tesco)/lcm_of_cofactors);
    *tesco = (UCHAR)((*tesco)*lcm_of_cofactors);

    *tx_pkt_length = (UINT16)((*tesco)/lcm_of_cofactors);
    *tx_pkt_length = (UINT16)((lcm_of_cofactors/cofactor_tx_num)*
                              cofactor_tx_denom*(*tx_pkt_length));

    *rx_pkt_length = (UINT16)((*tesco)/lcm_of_cofactors);
    *rx_pkt_length = (UINT16)((lcm_of_cofactors/cofactor_rx_num)*
                              cofactor_rx_denom*(*rx_pkt_length));

    if(( (*tx_pkt_length) * slotno) !=  ((*tesco) * tx_bandwidth))
    {
        return API_FAILURE;
    }

    if(( (*rx_pkt_length) * slotno) !=  ((*tesco) * rx_bandwidth))
    {
        return API_FAILURE;
    }
    return API_SUCCESS;
}



/**
 * Find a modified max_latency value within which the parameters for setting
 * up ESCO connection can be found out. This is necessary when the host
 * specifies a large max_latency value.
 *
 * \param rx_bandwidth The receive bandwidth specified by Host.
 * \param tx_bandwidth The transmit bandwidth specified by Host.
 * \param max_pkt_length Maximum possible packet length that can be chosen for
 *                       setting up the link. This is derived from allowed
 *                       packet types.
 * \param max_window_size Maximum size of a Rx and Tx pair on the ESCO link.
 * \param retx_effort The retransmission effort specified by Host.
 *
 * \return The new max_latency specified in slots.
 */
UINT16 lmp_find_new_max_latency(UINT32 rx_bandwidth,UINT32 tx_bandwidth,
                                UINT16 max_pkt_length, UCHAR max_window_size, UCHAR retx_effort)
{
    UCHAR tx_tesco = 0,rx_tesco = 0,tesco = 0;

    if(tx_bandwidth != 0)
    {
        /* Find the Max. Tesco value with which Tx Bandwidth can be
           acheived
        */
        tx_tesco = (UCHAR)((1600 * max_pkt_length)/tx_bandwidth);

        /* Making Tesco even */
        tx_tesco = (UCHAR)((tx_tesco >> 1) << 1);
    }

    if(rx_bandwidth != 0)
    {
        /* Find the Max. Tesco value with which Rx Bandwidth can be
           acheived
        */
        rx_tesco = (UCHAR)((1600 * max_pkt_length)/rx_bandwidth);
        /* Making Tesco even */
        rx_tesco = (UCHAR)((rx_tesco >> 1) << 1);
    }

    /* This is a case when the Device wants to setup a connection
       with Tx Bw = 0 and Rx Bw = 0. Since value of Tesco can be
       anything, choose the default Max_latency value
    */
    if((rx_tesco == 0) && (tx_tesco == 0))
    {
        tesco = LMP_DEFAULT_TESCO_VALUE;
    }
    else if((rx_tesco != 0) && (tx_tesco != 0))
    {
        /* If Rx_Tesco and Tx_Tesco are different,then choose
           the smaller among them as the Tesco value, since it's
           possible to acheive the Bandwidth , by reducing the
           length if Tesco is smaller than the estimated value.
        */
        tesco = (UCHAR)((tx_tesco > rx_tesco)?rx_tesco : tx_tesco);
    }
    else
    {
        /* If either Rx_Tesco or Tx_tesco is zero, then choose
           the non-zero value.
        */
        tesco = (UCHAR)((tx_tesco > rx_tesco)?tx_tesco : rx_tesco);
    }

    /* Find the Max_latency value based on the Retransmission
       Effort.
    */
    switch(retx_effort)
    {
            /* For Retx Effort = 0 or Don't care(0xFF) */
        case 0:
        case 255:
            return (UINT16)(tesco + max_window_size);

        case 1:
            return (UINT16)(tesco + 2*max_window_size);
        case 2:
            return (UINT16)(tesco +
                            (LMP_MAX_NUM_OF_RETRANSMISSIONS + 1)*max_window_size);
        default:
            ESCO_ERR(INVALID_RETX_EFFORT_PARAMETERS,0,0);
            return 0;
    }

}

/**
 * Calculate the max slot value on the acl link based on ESCO links'
 * parameters.
 *
 * \param acl_ce_index The ACL connection entity index.
 *
 * \return max_slot
 */
UCHAR lmp_calculate_max_slot_for_esco(UINT16 acl_ce_index)
{
    UCHAR max_slot = 5;

#ifdef COMPILE_ESCO
    /* Check max_slots based on eSCO connections */
    UCHAR end_of_window = 0,start_of_window = 0;
    UCHAR size_of_window =0;
#endif /* COMPILE_ESCO */

    if (lmp_self_device_data.number_of_esco_connections != 0)
    {
#ifdef COMPILE_ESCO
        max_slot = 5;

        /* min size of the window to transmit a 5 slot packet */
        size_of_window = 6;

        /* window required to support a 5 slot packet */
        end_of_window = (UCHAR)(start_of_window + size_of_window);

        while(size_of_window >= 2)
        {
            start_of_window = 0;
            end_of_window = (UCHAR)(start_of_window + size_of_window);

            while( end_of_window <= max_tesco)
            {
                /* Check if the window overlaps any of the reserved slots */
                if(lmp_check_res_slot_overlap(start_of_window,end_of_window,
                                              acl_ce_index) == API_SUCCESS)
                {
                    max_slot = (UCHAR)(size_of_window - 1);
                    ESCO_INF(NEW_MAX_SLOT_VAL,1,max_slot);

                    /* Send the LMP Max slot pdu only if the Max slot for this
                       link has to be changed .
                                        */

                    return max_slot;
                };

                start_of_window += 2;
                end_of_window = (UCHAR)(start_of_window + size_of_window);
            }

            size_of_window  -= 2;
        }/* end of while */
#endif /* COMPILE_ESCO */
    }

    return max_slot;
}
#endif /* COMPILE_ESCO */

