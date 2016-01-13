/***************************************************************************
 Copyright (C) MindTree Ltd.
 This module is a confidential and proprietary property of MindTree and
 a possession or use of this module requires written permission of MindTree.
 ***************************************************************************/

/**
 * \file
 *  Contains the functions that create/handle BT 1.2 pdus.
 */


/********************************* Logger *************************/ 
enum { __FILE_NUM__= 47 };
/********************************* Logger *************************/

/* ========================= Include File Section ========================= */
#include "lmp_internal.h"
#include "bz_debug.h"
#include "mem.h"
#include "UartPrintf.h"
#include "lmp_2_1.h"
#ifdef _SUPPORT_SECURE_CONNECTION_
#include "bz_auth_internal.h"
#endif
/* ==================== Structure declaration Section ===================== */


/* ===================== Variable Declaration Section ===================== */

#ifdef _DAPE_TEST_DISABLE_EDR_BY_VENDOR_CMD_FOR_BR_PKT
extern UCHAR g_ptt_enabled;
#endif
/* ================== Static Function Prototypes Section ================== */



/* ===================== Function Definition Section ====================== */
/**
 * Check and validate the incoming PDU. Depending on the type of PDU received,
 * it routes the received PDU to appropriate function.
 *
 * \param lmp_pdu_ptr The pointer to the #LMP_PDU_PKT.
 * \param piconet_id The Logical Piconet ID.
 *
 * \return BT_FW_SUCCESS, if the operation is successful. BT_FW_ERROR
 *         otherwise.
 */
UCHAR lmp_handle_1_2_incoming_pdu(LMP_PDU_PKT *lmp_pdu_ptr, UINT16 ce_index)
{
	UCHAR pdu_opcode ;
	UCHAR pdu_ext_opcode;
	UCHAR error_code = BT_FW_SUCCESS;

    pdu_opcode = (UCHAR)(lmp_pdu_ptr->payload_content[0] >> 1);

	LMP_INF(PDU_RECEIVED_OPCODE,1,pdu_opcode);

    switch (pdu_opcode)
    {
       case LMP_SET_AFH_OPCODE:
#ifdef COMPILE_AFH_HOP_KERNEL
           lmp_handle_set_afh_pdu(lmp_pdu_ptr,ce_index);
#else /* COMPILE_AFH_HOP_KERNEL */
           UCHAR transaction_id = (UCHAR)LMP_GET_TRANSACTION_ID(lmp_pdu_ptr);
           lmp_send_lmp_not_accepted(ce_index, pdu_opcode,
               transaction_id, UNSUPPORTED_REMOTE_FEATURE_ERROR);
#endif /* COMPILE_AFH_HOP_KERNEL */
           break;
       case LMP_ESCAPE1_OPCODE:
       case LMP_ESCAPE2_OPCODE:
       case LMP_ESCAPE3_OPCODE:
            break;

        case LMP_ESCAPE4_OPCODE:
            pdu_ext_opcode = lmp_pdu_ptr->payload_content[1];
			LMP_INF(PDU_EXT4_RECEIVED_OPCODE,1,pdu_ext_opcode);
            switch (pdu_ext_opcode)
            {
                case LMP_CHANNEL_CLASSIFICATION_REQ_OPCODE:
#ifdef COMPILE_AFH_HOP_KERNEL
                    lmp_handle_channel_classification_req_pdu(lmp_pdu_ptr,
                            ce_index);
#else /* COMPILE_AFH_HOP_KERNEL */
                    UCHAR transaction_id = (UCHAR)LMP_GET_TRANSACTION_ID(lmp_pdu_ptr);
                    lmp_send_lmp_not_accepted(ce_index, pdu_opcode,
                        transaction_id, UNSUPPORTED_REMOTE_FEATURE_ERROR);
#endif /* COMPILE_AFH_HOP_KERNEL */
                    break;
                case LMP_CHANNEL_CLASSIFICATION_OPCODE :
#ifdef COMPILE_AFH_HOP_KERNEL
                    lmp_handle_channel_classification_pdu(lmp_pdu_ptr,ce_index);
#else /* COMPILE_AFH_HOP_KERNEL */
                    UCHAR transaction_id = (UCHAR)LMP_GET_TRANSACTION_ID(lmp_pdu_ptr);
                    lmp_send_lmp_not_accepted(ce_index, pdu_opcode,
                        transaction_id, UNSUPPORTED_REMOTE_FEATURE_ERROR);
#endif /* COMPILE_AFH_HOP_KERNEL */
                    break;

                case LMP_ESCO_LINK_REQ_OPCODE:
#ifdef COMPILE_ESCO
                     lmp_handle_esco_link_req_pdu(lmp_pdu_ptr,ce_index);
#else /* COMPILE_ESCO */
                     UCHAR transaction_id = (UCHAR)LMP_GET_TRANSACTION_ID(lmp_pdu_ptr);
                     lmp_send_lmp_not_accepted(ce_index, pdu_opcode,
                         transaction_id, UNSUPPORTED_REMOTE_FEATURE_ERROR);
#endif /* COMPILE_ESCO */
                     break;
                case LMP_REMOVE_ESCO_LINK_REQ_OPCODE:
#ifdef COMPILE_ESCO
                     lmp_handle_remove_esco_link_req_pdu(lmp_pdu_ptr, ce_index);
#else /* COMPILE_ESCO */
                     UCHAR transaction_id = (UCHAR)LMP_GET_TRANSACTION_ID(lmp_pdu_ptr);
                     lmp_send_lmp_not_accepted(ce_index, pdu_opcode,
                         transaction_id, UNSUPPORTED_REMOTE_FEATURE_ERROR);
#endif /* COMPILE_ESCO */
                     break;

                case LMP_ACCEPTED_EXT_OPCODE:
                     error_code =
                         lmp_handle_accepted_ext_pdu(lmp_pdu_ptr, ce_index);
                     break;
                case LMP_NOT_ACCEPTED_EXT_OPCODE:
                     error_code =
                         lmp_handle_not_accepted_ext_pdu(lmp_pdu_ptr, ce_index);
                     break;
                case LMP_FEATURES_REQ_EXT_OPCODE :
#ifdef COMPILE_FEATURE_REQ_EXT
                     lmp_handle_features_req_ext_pdu(lmp_pdu_ptr,ce_index);
#else /* COMPILE_FEATURE_REQ_EXT */
                     UCHAR transaction_id = (UCHAR)LMP_GET_TRANSACTION_ID(lmp_pdu_ptr);
                     lmp_send_lmp_not_accepted(ce_index, pdu_opcode,
                         transaction_id, UNSUPPORTED_REMOTE_FEATURE_ERROR);
#endif /* COMPILE_FEATURE_REQ_EXT */
                     break;
                case LMP_FEATURES_RES_EXT_OPCODE :
#ifdef COMPILE_FEATURE_REQ_EXT
                     lmp_handle_features_res_ext_pdu(lmp_pdu_ptr,ce_index);
#else /* COMPILE_FEATURE_REQ_EXT */
                     UCHAR transaction_id = (UCHAR)LMP_GET_TRANSACTION_ID(lmp_pdu_ptr);
                     lmp_send_lmp_not_accepted(ce_index, pdu_opcode,
                         transaction_id, UNSUPPORTED_REMOTE_FEATURE_ERROR);
#endif /* COMPILE_FEATURE_REQ_EXT */
                     break;

                case LMP_PTT_REQ_OPCODE:
                    lmp_handle_ptt_req_pdu(lmp_pdu_ptr,ce_index);
                    break;

                default:
                    return BT_FW_ERROR;
            }
            break;
        default:
            return BT_FW_ERROR;
    }
    return error_code;
}

/**
 * Handle the BB_ACK received for an 1.2 PDU.
 *
 * \param lmp_pdu_ptr The pointer to the LMP PDU packet.
 * \param piconet_id The Logical Piconet ID.
 *
 * \return None.
 */
void lmp_handle_1_2_pdu_ack_recd(LMP_PDU_PKT *lmp_pdu_ptr, 
								 UCHAR piconet_id, UINT16 ce_index)
{
	UCHAR pdu_opcode ;
	UCHAR pdu_ext_opcode;

    pdu_opcode = (UCHAR)(lmp_pdu_ptr->payload_content[0] >> 1);
	
    switch(pdu_opcode)
    {
#ifdef COMPILE_AFH_HOP_KERNEL
    case LMP_SET_AFH_OPCODE:
        lmp_handle_ack_received_for_set_afh_pdu(ce_index);
        break;
#endif

    case LMP_ESCAPE1_OPCODE:
    case LMP_ESCAPE2_OPCODE:
    case LMP_ESCAPE3_OPCODE:
        break;

    case LMP_ESCAPE4_OPCODE:
            pdu_ext_opcode = lmp_pdu_ptr->payload_content[1];
			LMP_INF(PDU_EXT4_RECEIVED_OPCODE,1,pdu_ext_opcode);

            switch(pdu_ext_opcode)
            {
#ifdef COMPILE_AFH_HOP_KERNEL
                case LMP_CHANNEL_CLASSIFICATION_OPCODE :
                case LMP_CHANNEL_CLASSIFICATION_REQ_OPCODE:
                    break;
#endif
                case LMP_ACCEPTED_EXT_OPCODE:
                    lmp_handle_ack_rcvd_for_accepted_ext(lmp_pdu_ptr,
                                     ce_index);
                    break;
                default:
                    return ;
            }
            break;
    default:
        break;
    }
    return ;
}

/**
 * Check and validate the LMP_accepted_ext PDU received.
 *
 * \param lmp_pdu_ptr The pointer to the #LMP_PDU_PKT.
 * \param ce_index Connection entity index of the connection.
 *
 * \return None.
 */
UCHAR lmp_handle_accepted_ext_pdu(LMP_PDU_PKT *lmp_pdu_ptr,UINT16 ce_index)
{
    UCHAR pdu_opcode,pdu_ext_opcode;

    pdu_opcode = lmp_pdu_ptr->payload_content[2];
    switch (pdu_opcode)
    {
        case LMP_ESCAPE1_OPCODE:
        case LMP_ESCAPE2_OPCODE:
        case LMP_ESCAPE3_OPCODE:
            break;

        case LMP_ESCAPE4_OPCODE:
            pdu_ext_opcode = lmp_pdu_ptr->payload_content[3];
            switch (pdu_ext_opcode)
            {
#ifdef COMPILE_ESCO
                case LMP_ESCO_LINK_REQ_OPCODE:
                    lmp_create_esco_connection(ce_index);
                    break;

                case LMP_REMOVE_ESCO_LINK_REQ_OPCODE:
                    //RT_BT_LOG(GRAY, LMP_1_2_PDU_268, 0, 0);
                    lmp_handle_remove_esco_link_req_accepted(lmp_pdu_ptr,
                            ce_index);
                    break;
#endif /* COMPILE_ESCO */

                case LMP_PTT_REQ_OPCODE:
                    lmp_handle_ptt_req_accepted(lmp_pdu_ptr,ce_index);
                    break;

                default:
                    return BT_FW_ERROR;

            }
            break;

        default:
            return BT_FW_ERROR;
    }

    return BT_FW_SUCCESS;
}

/**
 * Check and validate the LMP_not_accepted_ext PDU received.
 *
 * \param lmp_pdu_ptr The pointer to the #LMP_PDU_PKT.
 * \param ce_index Connection entity index of the connection.
 *
 * \return None.
 */
UCHAR lmp_handle_not_accepted_ext_pdu(LMP_PDU_PKT *lmp_pdu_ptr, UINT16 ce_index)
{
    UCHAR pdu_opcode, pdu_ext_opcode;

    pdu_opcode = lmp_pdu_ptr->payload_content[2];
    switch (pdu_opcode)
    {
        case LMP_ESCAPE1_OPCODE:
        case LMP_ESCAPE2_OPCODE:
        case LMP_ESCAPE3_OPCODE:
            break;

        case LMP_ESCAPE4_OPCODE:
            pdu_ext_opcode = lmp_pdu_ptr->payload_content[3];
            switch (pdu_ext_opcode)
            {
#ifdef COMPILE_ESCO
                case LMP_ESCO_LINK_REQ_OPCODE:
                    lmp_handle_esco_link_req_not_accepted(lmp_pdu_ptr,ce_index);
                    return BT_FW_SUCCESS;

                case LMP_REMOVE_ESCO_LINK_REQ_OPCODE:
                    /* To Do */
                    return BT_FW_SUCCESS;
#endif /* COMPILE_ESCO */

                case LMP_SNIFF_SUBRATING_REQ_OPCODE:
                    /* Drop this PDU: Might be due to trans collision  */
                    /* Reset ssr neg state */
                    lmp_connection_entity[ce_index].ssr_data.neg_in_progress 
                                                                       = FALSE;

                    /* Stop the PDU response timer here. */
                    lmp_handle_stop_pdu_response_time(ce_index, lmp_pdu_ptr);
                    return BT_FW_SUCCESS;
                default:
                    return BT_FW_ERROR;
            }
        default:
            return BT_FW_ERROR;
    }

    return BT_FW_SUCCESS;
}

/**
 * Handle the BB_ACK received for the LMP_accepted_ext.
 *
 * \param lmp_pdu_ptr The pointer to the #LMP_PDU_PKT.
 * \param ce_index Connection entity index of the connection.
 *
 * \return None.
 */
void lmp_handle_ack_rcvd_for_accepted_ext(LMP_PDU_PKT *lmp_pdu_ptr,
        UINT16 ce_index)
{
    UCHAR pdu_opcode,pdu_ext_opcode;

    pdu_opcode = lmp_pdu_ptr->payload_content[2];

    switch(pdu_opcode)
    {
        case LMP_ESCAPE1_OPCODE:
        case LMP_ESCAPE2_OPCODE:
        case LMP_ESCAPE3_OPCODE:
            break;

        case LMP_ESCAPE4_OPCODE:
            pdu_ext_opcode = lmp_pdu_ptr->payload_content[3];
            switch(pdu_ext_opcode)
            {
#ifdef COMPILE_ESCO
                case LMP_ESCO_LINK_REQ_OPCODE:
                    ESCO_INF(ACK_RCVD_FOR_LMP_ACCEPTED_EXT_PDU_FOR_ESCO_LINK_REQ,0,0);
                    lmp_create_esco_connection(ce_index);
                    break;

                case LMP_REMOVE_ESCO_LINK_REQ_OPCODE:
                    lmp_kill_esco_connection(ce_index);
                    break;
#endif /* COMPILE_ESCO */

                case LMP_PTT_REQ_OPCODE:
                    lmp_handle_ptt_req_actd_ack_recd(ce_index);
                    break;

               default:
                    return ;
            }
            break;
       default:
                break;
        }
        return ;
}

/**
 * Send the LMP_accepted_ext PDU to the remote device.
 *
 * \param ce_index Connection entity index of the connection.
 * \param opcode Opcode for which this LMP_accepted_ext is sent.
 * \param ext_opcode Extended Opcode (New in 1.2).
 * \param trans_id Transaction ID.
 *
 * \return None.
 */
void lmp_send_lmp_accepted_ext(UINT16 ce_index, UCHAR opcode,
        UCHAR ext_opcode, UCHAR trans_id)
{
    UCHAR parameter_list[LMP_ACCEPTED_EXT_LEN];

    parameter_list[0] = LMP_ESCAPE4_OPCODE;
    parameter_list[2] = LMP_ACCEPTED_EXT_OPCODE;
    parameter_list[3] = opcode;
    parameter_list[4] = ext_opcode;
    lmp_generate_pdu(ce_index, parameter_list, LMP_ACCEPTED_EXT_LEN,
            (LMP_TRAN_ID)trans_id, LMP_NO_STATE_CHANGE);
    return;
}

/**
 * Send the LMP_not_accepted_ext PDU to the remote device.
 *
 * \param ce_index Connection entity index of the connection.
 * \param opcode Opcode for which this LMP_not_accepted_ext is sent.
 * \param ext_opcode Extended Opcode (New in 1.2).
 * \param trans_id Transaction ID.
 * \param reason Reason for rejecting the transaction.
 *
 * \return None.
 */
void lmp_send_lmp_not_accepted_ext(UINT16 ce_index, UCHAR opcode,
        UCHAR ext_opcode, UCHAR trans_id, UCHAR reason)
{
      UCHAR parameter_list[LMP_NOT_ACCEPTED_EXT_LEN];

       /*send link key to remote device*/
       parameter_list[0] = LMP_ESCAPE4_OPCODE;
       parameter_list[2] = LMP_NOT_ACCEPTED_EXT_OPCODE;
       parameter_list[3] = opcode;
       parameter_list[4] = ext_opcode;
       parameter_list[5] = reason;
       lmp_generate_pdu(ce_index, parameter_list, LMP_NOT_ACCEPTED_EXT_LEN,
               (LMP_TRAN_ID)trans_id, LMP_NO_STATE_CHANGE);
       return;
}

#ifdef COMPILE_FEATURE_REQ_EXT
/**
 * Get maximum supported features page.
 *
 * \param None.
 *
 * \return maximum page.
 */
UINT8 lmp_get_max_features_page(void)
{
    UINT8 max_page;
    
    for (max_page = (LMP_MAX_FEAT_REQ_PAGES - 1); max_page > 0; max_page--)
    {
        if (lmp_feature_data.features_dword[max_page][0] != 0)
        {
            /* found non-zero bits between bit[31:0] */
#ifdef _SECURE_CONN_TEST_LOG
RT_BT_LOG(GREEN, DAPE_TEST_LOG293, 1,max_page);
#endif
            
            break;
        }

        if (lmp_feature_data.features_dword[max_page][1] != 0)
        {
            /* found non-zero bits between bit[63:32] */
#ifdef _SECURE_CONN_TEST_LOG
RT_BT_LOG(GREEN, DAPE_TEST_LOG293, 1,max_page);
#endif
            
            break;
        }                
    }

    lmp_feature_data.max_supp_pages = max_page;
#ifdef _SECURE_CONN_TEST_LOG
RT_BT_LOG(BLUE, DAPE_TEST_LOG525, 6,LMP_MAX_FEAT_REQ_PAGES, 
LMP_MAX_EXTENDED_FEAT_REQ_PAGES, max_page,0,0,0);
#endif
    return max_page;
}

/**
 * Populate the local supported features into local data structure.
 *
 * \param piconet_id Whether master-piconet or slave-piconet.
 * \param page The page number of the features to be populated.
 * \param buf[8] The supported features array.
 *
 * \return None.
 */
void lmp_fill_features_page(UCHAR page, UCHAR *buf)
{
	if (page < LMP_MAX_FEAT_REQ_PAGES)
	{
		/* Requesting for some existing page */
		memcpy(&buf[0], &lmp_feature_data.features[page][0],
			BTC_FEATURES_LEN);
	}
    else
    {
        /* We don't have this extended page have it */
        memset(&buf[0], 0, BTC_FEATURES_LEN);
    }
}


/**
 * Sends lmp_feature_req_ext or lmp_feature_res_ect PDU.
 *
 * \param ce_index Index to the connection entity table.
 * \param page The page number of the features to be sent.
 * \param opcode The opcode of the PDU to be sent.
 * \param tid The transaction ID of the PDU to be sent.
 *
 * \return None.
 */
void lmp_send_features_req_or_res_ext_pdu(UINT16 ce_index, UCHAR page,
        UCHAR opcode, LMP_TRAN_ID tid)
{
    UCHAR parameter_list[LMP_FEATURES_REQ_EXT_LEN];

    BZ_ASSERT(opcode == LMP_FEATURES_REQ_EXT_OPCODE
            || opcode == LMP_FEATURES_RES_EXT_OPCODE,
            "I can't send any PDU other than LMP_feature_req/res_ext");
    parameter_list[0] = LMP_ESCAPE4_OPCODE;
    parameter_list[2] = opcode;
    parameter_list[3] = page;
	parameter_list[4] = lmp_get_max_features_page();
	lmp_fill_features_page(page, &parameter_list[5]);

	lmp_generate_pdu(ce_index, parameter_list, LMP_FEATURES_REQ_EXT_LEN, tid,
		LMP_NO_STATE_CHANGE);

	return;
}

/**
 * Validate the LMP_features_req_ext PDU and send LMP_features_res_ext PDU in
 * response.
 *
 * \param lmp_pdu_ptr The pointer to the LMP PDU packet.
 * \param ce_index Connection entity index of the ACL connection.
 *
 * \return None.
 */
void lmp_handle_features_req_ext_pdu(LMP_PDU_PKT *lmp_pdu_ptr, UINT16 ce_index)
{
    UCHAR page;

    page = lmp_pdu_ptr->payload_content[2];
    /* Store the remote features, if possible */
    if (page < LMP_MAX_FEAT_REQ_PAGES)
    {
        memcpy(&lmp_connection_entity[ce_index].features[page][0],
                &lmp_pdu_ptr->payload_content[4], BTC_FEATURES_LEN);
    }
    lmp_send_features_req_or_res_ext_pdu(ce_index, page,
            LMP_FEATURES_RES_EXT_OPCODE, REMOTE_DEV_TID);
}

/**
 * Validate the LMP_features_res_ext PDU and send
 * HCI_READ_REMOTE_EXT_FEATURES_COMPLETE_EVENT if the PDU is correct.
 *
 * \param lmp_pdu_ptr The pointer to the LMP PDU packet.
 * \param ce_index Connection entity index of the ACL connection.
 *
 * \return None.
 */
void lmp_handle_features_res_ext_pdu(LMP_PDU_PKT *lmp_pdu_ptr, UINT16 ce_index)
{
    UCHAR page;
    LMP_CONNECTION_ENTITY* ce_ptr;

    ce_ptr = &lmp_connection_entity[ce_index];
    page = lmp_pdu_ptr->payload_content[2];

    if (ce_ptr->hci_cmd_bits & REMOTE_EX_FEA_TRUE_HOST_BIT_MASK)
    {
        ce_ptr->hci_cmd_bits |= REMOTE_EX_FEA_BIT_MASK;
    }
#ifdef _SECURE_CONN_TEST_LOG
//RT_BT_LOG(RED, DAPE_TEST_LOG525, 6,ce_ptr->requested_ext_page, 
//page, ce_ptr->hci_cmd_bits, ce_ptr->host_requested_ext_page,LMP_MAX_FEAT_REQ_PAGES,0);
#endif
#ifndef _SUPPORT_EXT_FEATURES_PAGE_2_
    if (((ce_ptr->hci_cmd_bits & REMOTE_EX_FEA_BIT_MASK) == FALSE) || 
        (ce_ptr->requested_ext_page != page))
#else
    if (((ce_ptr->hci_cmd_bits & REMOTE_EX_FEA_BIT_MASK) == FALSE) || 
        ((ce_ptr->host_requested_ext_page != page) &&
        (ce_ptr->requested_ext_page != page)))
#endif
    {
        /* We don't expect this PDU/page now */
        lmp_send_lmp_not_accepted_ext(ce_index, LMP_ESCAPE4_OPCODE,
                LMP_FEATURES_RES_EXT_OPCODE,
                (UCHAR)LMP_GET_TRANSACTION_ID(lmp_pdu_ptr),
                PDU_NOT_ALLOWED_ERROR);
        return;
    }

    /* Store the remote features, if possible */
    if (page < LMP_MAX_FEAT_REQ_PAGES)
    {
        memcpy(&ce_ptr->features[page][0], &lmp_pdu_ptr->payload_content[4],
                BTC_FEATURES_LEN);

#ifdef _SUPPORT_EXT_FEATURES_PAGE_2_
        UINT8 max_page;
        UINT8 i;
        
        max_page = MIN(lmp_pdu_ptr->payload_content[3], LMP_MAX_EXTENDED_FEAT_REQ_PAGES);
#ifdef _SUPPORT_SECURE_CONNECTION_
        BZ_AUTH_LINK_PARAMS* auth;
        auth = lmp_connection_entity[ce_index].auth;

        if (max_page >=2)
        {
            if (page == max_page)
            {
#ifdef _SECURE_CONN_TEST_LOG
RT_BT_LOG(RED, DAPE_TEST_LOG525, 6,page,ce_ptr->features[1][0], 
 ce_ptr->features[page][1], lmp_feature_data.features[1][0],
 lmp_feature_data.features[2][1],0);
#endif
            
                //if (lmp_feature_data.features[2][1] & BIT0)
                if ((ce_ptr->features[1][0] & SECURE_CONNECTION_HOST_SUPPORT) &&
                   (ce_ptr->features[2][1] & SECURE_CONNECTION_CONTROLLER_SUPPORT) &&
                   (lmp_feature_data.features[1][0] & SECURE_CONNECTION_HOST_SUPPORT) &&
                   (lmp_feature_data.features[2][1] & SECURE_CONNECTION_CONTROLLER_SUPPORT))
                {
                    auth->secure_conn_enabled = 1;
                    auth->len_prime = 8;

#ifdef _CCH_SC_ECDH_P256_PAUSE_ENC
                    auth->sc_use_enc_rand = 0;
#endif    

                    RT_BT_LOG(GREEN, SECURE_CONN_LOG6, 2,ce_index,
                    lmp_feature_data.features[2][1] & SUPPORT_PING);
                }
                else
                {
                    auth->secure_conn_enabled = 0;
                    auth->len_prime = 6;
                    RT_BT_LOG(YELLOW, SECURE_CONN_LOG7, 3,ce_index,
                    (ce_ptr->features[1][0] & SECURE_CONNECTION_HOST_SUPPORT),
                    (ce_ptr->features[2][1] & SECURE_CONNECTION_CONTROLLER_SUPPORT));
                }
            }
        }
        else
        {
            auth->secure_conn_enabled = 0;
            auth->len_prime = 6;
        }
#endif
        ce_ptr->requested_ext_page_bm &= ((1 << (max_page + 1)) - 1);
        ce_ptr->requested_ext_page_bm &= ~(1 << page);
#ifdef _SECURE_CONN_TEST_LOG
RT_BT_LOG(RED, DAPE_TEST_LOG525, 6, 
lmp_pdu_ptr->payload_content[3], LMP_MAX_EXTENDED_FEAT_REQ_PAGES,
max_page, ce_ptr->requested_ext_page_bm, 
((1 << (max_page + 1)) - 1), ~(1 << page));
#endif
        
        if (ce_ptr->requested_ext_page_bm > 0)
        {
            for (i = 1; i <= max_page; i++)
            {
                if (ce_ptr->requested_ext_page_bm & (1 << i))
                {
                    ce_ptr->requested_ext_page = i;
                    break;
                }
            }
        }
#endif        
    }
     
#ifdef _SUPPORT_EXT_FEATURES_PAGE_2_
    if ((ce_ptr->requested_ext_page_bm == 0) ||
        ((ce_ptr->hci_cmd_bits & REMOTE_EX_FEA_TRUE_HOST_BIT_MASK) &&
          (page == ce_ptr->host_requested_ext_page)))
#endif
    {
        if (g_efuse_lps_setting_3.iot_ralink_send_fea_req_later)
        {
            if (ce_ptr->got_remote_ext_feature == 0)
            {
                /* We have initiated the feature request automatically as part of ACL
                 * connection establishment/Remote_Name_Request... Now let's proceed
                 * with completing the connection establishment procedure..
                 */
                lmp_continue_conn_proc_after_feature_exchange(ce_index);
                ce_ptr->got_remote_ext_feature = 1;
            }
    	}
    	else
    	{
            if (ce_ptr->ce_status == LMP_BB_HL_CONNECTED)
            {
                /* We have initiated the feature request automatically as part of ACL
                 * connection establishment/Remote_Name_Request... Now let's proceed
                 * with completing the connection establishment procedure..
                 */
                lmp_continue_conn_proc_after_feature_exchange(ce_index);
            }
        }
    }
        
    if (ce_ptr->hci_cmd_bits & REMOTE_EX_FEA_TRUE_HOST_BIT_MASK)
    {
#ifdef _SUPPORT_EXT_FEATURES_PAGE_2_
        if (page == ce_ptr->host_requested_ext_page)
#endif
        {
            hci_generate_remote_ext_features_complete_event(HCI_COMMAND_SUCCEEDED,
                ce_ptr->connection_type.connection_handle, page,
            lmp_get_max_features_page(),
                &lmp_pdu_ptr->payload_content[4]);

            ce_ptr->hci_cmd_bits &= (~REMOTE_EX_FEA_BIT_MASK);
            ce_ptr->hci_cmd_bits &= ~REMOTE_EX_FEA_TRUE_HOST_BIT_MASK;
        }         
    }    

#ifdef _SUPPORT_EXT_FEATURES_PAGE_2_
    if (ce_ptr->requested_ext_page_bm > 0)
    {
        /* send features req ext to another ext page */
        lmp_send_features_req_or_res_ext_pdu(ce_index, ce_ptr->requested_ext_page,
                                LMP_FEATURES_REQ_EXT_OPCODE, SELF_DEV_TID);        
        ce_ptr->hci_cmd_bits |= REMOTE_EX_FEA_BIT_MASK;
    }
#endif	

}
#endif /* COMPILE_FEATURE_REQ_EXT */

/**
 * Handle the LMP_ptt_req and takes the appropriate action.
 *
 * \param lmp_pdu_ptr The pointer to the LMP PDU packet.
 * \param ce_index Connection entity index of the ACL connection.
 *
 * \return None.
 */
void lmp_handle_ptt_req_pdu(LMP_PDU_PKT * lmp_pdu_ptr, UINT16 ce_index)
{
    UCHAR edr_val;
    UCHAR status = BT_FW_SUCCESS;
    LMP_CONNECTION_ENTITY *ce_ptr;

    ce_ptr = &lmp_connection_entity[ce_index];
    edr_val = lmp_pdu_ptr->payload_content[2];


#ifdef _CCH_SC_ECDH_P256_TEST_PKT
    if (( ( lmp_feature_data.feat_page0[3] & EDR_ACL_2MBPS_FEATURE) == 0)||((ce_ptr->connection_type.packet_type&0x3306) == 0x3306))
#else
	if ( ( lmp_feature_data.feat_page0[3] & EDR_ACL_2MBPS_FEATURE) == 0)
#endif	
	{
		/* Check the feature bits for supported features. */
		status = UNSUPPORTED_FEATURE_OR_PARAMETER_ERROR;
	}
#ifdef _DAPE_TEST_DISABLE_EDR_BY_VENDOR_CMD_FOR_BR_PKT
    if (g_ptt_enabled == 0)
    {
        status = UNSUPPORTED_FEATURE_OR_PARAMETER_ERROR;  
    }
#endif

    /* PTT value can only be 0 or 1. */
    if ( (edr_val >> 1) != 0 )
    {
        status = INVALID_LMP_PARAMETERS_ERROR;
    }

#if 0 /* Disable state check */
    /* Check if the status is already enabled, and remote device is trying
        to enable second time. */
    if ( (edr_val == PTT_ENABLE) && (ce_ptr->ptt_status == LMP_PTT_ENABLED) )
    {
        status = INVALID_LMP_PARAMETERS_ERROR;
    }
#endif

    /* Check if the status is already disabled, and remote device is trying
        to disable again */
    if ( (edr_val == PTT_DISABLE) && (ce_ptr->ptt_status == LMP_PTT_IDLE) )
    {
        status = INVALID_LMP_PARAMETERS_ERROR;
    }

    /* Send lmp_accepted_ext. */
    if (status == BT_FW_SUCCESS)
    {
        lmp_send_lmp_accepted_ext(ce_index, LMP_ESCAPE4_OPCODE,
            LMP_PTT_REQ_OPCODE, REMOTE_DEV_TID);
    }
    else
    {
        lmp_send_lmp_not_accepted_ext(ce_index, LMP_ESCAPE4_OPCODE,
            LMP_PTT_REQ_OPCODE, REMOTE_DEV_TID, status);
        return;
    }

    /* Update the lmp status to ENABLINB/DISABLING, wait for BB_ACK for
        the lmp_accepted_ext */
    if (edr_val == PTT_ENABLE)
    {
        ce_ptr->ptt_status = LMP_PTT_ENABLING;
    }
    else
    {
        ce_ptr->ptt_status = LMP_PTT_DISABLING;
    }

    return;
}

/**
 * Handle the LMP_accepted for LMP_ptt_req and takes the appropriate action.
 *
 * \param lmp_pdu_ptr The pointer to the LMP PDU packet.
 * \param ce_index Connection entity index of the ACL connection.
 *
 * \return None.
 */
void lmp_handle_ptt_req_accepted(LMP_PDU_PKT* lmp_pdu_ptr, UINT16 ce_index)
{
    LMP_CONNECTION_ENTITY *ce_ptr;

    ce_ptr = &lmp_connection_entity[ce_index];

    switch (ce_ptr->ptt_status)
    {
        case LMP_PTT_ENABLING:
            ce_ptr->ptt_status = LMP_PTT_ENABLED;
            /* Enable the PTT bit in the LUT. */
            lc_enable_ptt_bit(ce_index);
            break;

        case LMP_PTT_DISABLING:
            ce_ptr->ptt_status = LMP_PTT_IDLE;
            /* Disable the PTT bit in the LUT. */
            lc_disable_ptt_bit(ce_index);
            break;

        default:
            /* Invalid state. */
            break;
    }

    /* Update the packet types */
    lc_update_pkts_allowed(ce_index);

    return;
}

/**
 * Handle the BB_ACK for LMP_accepted PDU for LMP_ptt_req PDU and takes
 * appropriate action.
 *
 * \param ce_index The Connection entity index of the ACL connection.
 *
 * \return None.
 */
void lmp_handle_ptt_req_actd_ack_recd(UINT16 ce_index)
{
    LMP_CONNECTION_ENTITY *ce_ptr;

    ce_ptr = &lmp_connection_entity[ce_index];

    switch (ce_ptr->ptt_status)
    {
        case LMP_PTT_ENABLING:
            ce_ptr->ptt_status = LMP_PTT_ENABLED;
            /* Enable the PTT bit in the LUT. */
            lc_enable_ptt_bit(ce_index);

            break;

        case LMP_PTT_DISABLING:
            ce_ptr->ptt_status = LMP_PTT_IDLE;
            /* Disable the PTT bit in the LUT. */
            lc_disable_ptt_bit(ce_index);

            break;

        default:
            /* Invalid state. */
            break;
    }

    lc_update_pkts_allowed(ce_index);

    return;
}

/**
 * Generate a LMP Packet Type Table request PDU with the given value.
 *
 * \param ce_index Connection entity index of the ACL connection.
 * \param pkt_type_table Value to be sent in the PDU.
 *
 * \return
 */
UCHAR lmp_generate_lmp_ptt_req_pdu(UINT16 ce_index, UCHAR pkt_type_table)
{
    UCHAR parameter_list[LMP_PTT_REQ_LEN];
    LMP_CONNECTION_ENTITY *ce_ptr;
#ifdef COMPILE_CHECK_LOCAL_FEATURES
    UCHAR local_feature;
#endif

    ce_ptr = &lmp_connection_entity[ce_index];

#ifdef COMPILE_CHECK_LOCAL_FEATURES
    /* Check local feature bits here. */
	local_feature = lmp_feature_data.feat_page0[3];
	if( (local_feature & EDR_ACL_2MBPS) == FALSE)
    {
        return API_SUCCESS;
    }
#endif
#ifdef _DAPE_TEST_DISABLE_EDR_BY_VENDOR_CMD_FOR_BR_PKT
    if (g_ptt_enabled == 0)
    {
        return API_SUCCESS;
    }
#endif
    /* Check for ACL-U data. If it is in progress, then stop it, and 
       then initiate the PTT exchange. */
    parameter_list[0] = LMP_ESCAPE4_OPCODE;
    parameter_list[2] = LMP_PTT_REQ_OPCODE;
    parameter_list[3] = pkt_type_table;

    /* Update the PTT state machine. */
    if (pkt_type_table == PTT_ENABLE)
    {
        ce_ptr->ptt_status = LMP_PTT_ENABLING;
    }
    if (pkt_type_table == PTT_DISABLE)
    {
        ce_ptr->ptt_status = LMP_PTT_DISABLING;
    }

    lmp_generate_pdu(ce_index, parameter_list, LMP_PTT_REQ_LEN, SELF_DEV_TID,
            LMP_NO_STATE_CHANGE);
    return API_SUCCESS;
}

