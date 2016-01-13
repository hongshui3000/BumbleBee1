/***************************************************************************
 Copyright (C) Mindtree Ltd.
 This module is a confidential and proprietary property of MindTree and
 a possession or use of this module requires written permission of MindTree.
 ***************************************************************************/

/**
 * \file bz_auth_internal.c
 *  BlueWiz Authentication module internal interface implementation. This
 *  file implements functions which shall only be used within this
 *  authentication module.
 *
 * \author Santhosh kumar M
 * \date 2007-08-28
 */
/********************************* Logger *************************/ 
enum { __FILE_NUM__= 20 };
/********************************* Logger *************************/
/* ========================= Include File Section ========================= */
#include "mem.h"
#include "lc.h"
#include "lmp.h"
#include "bz_debug.h"
#include "crypto.h"
#include "crypto11.h"
#include "bz_auth_states.h"
#include "bz_auth_extern_accessors.h"
#include "bz_auth_hci.h"
#include "bz_auth_internal.h"
#include "bz_auth_internal_2_1.h"
#ifdef COMPILE_BROADCAST_ENCRYPTION
#include "bz_auth_mlk.h"
#endif
#include "bz_auth_lkdb.h"

#ifdef _ROM_CODE_PATCHED_
#ifdef _SUPPORT_SECURE_CONNECTION_
PF_ROM_CODE_PATCH_FUNC rcp_bz_auth_start_ping_timer = NULL;
#endif
PF_ROM_CODE_PATCH_FUNC rcp_bz_auth_handle_auth_completion = NULL; 

#endif

#ifdef _SECURE_CONN_TEST_LOG
TIMER_ID g_timer_temp = OS_INVALID_HANDLE;
#endif
#ifdef _DAPE_TEST_SEND_PING_RES_BY_VENDOR_CMD    
extern UINT8 g_send_ping_response;
#endif
extern UINT32 OS_is_timer_running( OS_HANDLE timer_handle );

/* ====================== Macro Declaration Section ======================= */
/* This is an hack and this should not be used anywhere else */
#define bz_auth_is_switching_to_temp_key(auth)                          \
    ((auth)->super_state == AUTHENTICATED_TEMP_AUTHENTICATING           \
        || (auth)->super_state == ENCRYPTED_TEMP_ENCRYPTING)


/* ==================== Structure Declaration Section ===================== */


/* ===================== Variable Declaration Section ===================== */

/* ================== Static Function Prototypes Section ================== */
#ifdef COMPILE_BROADCAST_ENCRYPTION
BOOLEAN bz_auth_is_to_use_temp_enc_key_size(
        BZ_AUTH_LINK_PARAMS* auth);
#endif
void bz_auth_compute_link_key_from_contributions(UINT16 ce_index,
        BZ_AUTH_LINK_PARAMS* auth, UCHAR local_type, const UCHAR* remote_uck,
        UCHAR remote_type);


/* ===================== Function Definition Section ====================== */
/**
 * Generate authentication related LMP pdu and deliver it to the remote device.
 * It simply queues the pdu to the LC module and it in turn delivers the PDU
 * to remote device.
 *
 * \param ce_index The index of the Connection entity database corresponding
 *                 to the link.
 * \param param_list LMP PDU parameter list. Element0 will be the opcode,
 *                   Element1 will be our role(Master/Slave), Element2 and
 *                   the rest will be specific to the PDU to be sent.
 * \param pdu_len Length of the PDU (Use predefined macros like
 *                #LMP_ACCEPTED_LEN - if length has to be hard coded, then it
 *                should be Bluetooth spec defined length of that PDU plus
 *                one).
 * \param tran_id Transaction ID to be used for this PDU.
 *
 * \return API_SUCCESS, if the operation is successful. API_FAILURE, otherwise.
 */
API_RESULT bz_auth_generate_pdu(UINT16 ce_index, UCHAR* param_list,
        UCHAR pdu_len, LMP_TRAN_ID tran_id)
{
    BOOLEAN start_tid_timer = TRUE;

	LMP_CONNECTION_ENTITY *ce_ptr;

	ce_ptr = &lmp_connection_entity[ce_index];

    switch (param_list[0]) /* pdu_opcode */
    {
    case LMP_ESCAPE4_OPCODE:
        switch (param_list[2])  /* ext_pdu_opcode */
        {
        case LMP_KEYPRESS_NOTIFICATION_OPCODE:
            start_tid_timer = FALSE;
            break;

        case LMP_PAUSE_ENCRYPTION_REQ_OPCODE:
            /* Pause data transfer */
#ifdef COMPILE_NESTED_PAUSE_RESUME
			bz_auth_pause_data_transfer(ce_index, ACL_PAUSED_ENCRYPTION);
#else /* COMPILE_NESTED_PAUSE_RESUME */
			aclq_mark_am_addr_as_paused(ce_ptr->am_addr,ce_ptr->phy_piconet_id);
#endif /* COMPILE_NESTED_PAUSE_RESUME */
          
            break;
        }
        break;
#ifdef _SUPPORT_SECURE_CONNECTION_        
    case LMP_PAUSE_ENCRYPTION_AES_REQ_OPCODE:
#endif    
    case LMP_ENCRYPTION_MODE_REQ_OPCODE:
    case LMP_STOP_ENCRYPTION_REQ_OPCODE:
        /* Pause data transfer */
#ifdef COMPILE_NESTED_PAUSE_RESUME
		bz_auth_pause_data_transfer(ce_index, ACL_PAUSED_ENCRYPTION);
#else /* COMPILE_NESTED_PAUSE_RESUME */
		aclq_mark_am_addr_as_paused(ce_ptr->am_addr, ce_ptr->phy_piconet_id);
#endif /* COMPILE_NESTED_PAUSE_RESUME */
        break;

    case LMP_ENCRYPTION_KEY_SIZE_MASK_REQ_OPCODE:
    case LMP_ENCRYPTION_KEY_SIZE_MASK_RES_OPCODE:
        start_tid_timer = FALSE;
        break;
    default:
        break;
    }

    if (start_tid_timer == TRUE)
    {
        /* Restart TID timer */
        bz_auth_start_tid_timer(ce_index, LMP_AUTH_TID_TIMEOUT);
    }

    return lmp_generate_pdu(ce_index, param_list, pdu_len, tran_id,
            LMP_NO_STATE_CHANGE);

}

/**
 * Starts/restarts the LMP response timer for the authentication module.
 *
 * \param ce_index ACL connection entity index.
 * \param timeout_value Timeout value to be used for starting the timer.
 *
 * \return None.
 */
void bz_auth_start_tid_timer(UINT16 ce_index, UINT32 timeout_value)
{
    if (lmp_connection_entity[ce_index].auth->tid_timer != NULL)
    {
        OS_START_TIMER(lmp_connection_entity[ce_index].auth->tid_timer,
                timeout_value);
    }
}

/**
 * LMP response timeout handler for the authentication module. The handler
 * properly rolebacks the transaction that resulted in the LMP response
 * timeout.
 *
 * \param timer_handle Timer handle.
 * \param index ACL connection entity index.
 *
 * \return None.
 */
void bz_auth_tid_timer_handler(TimerHandle_t timer_handle)
{
    LMP_CONNECTION_ENTITY* ce_ptr;
    UINT16 ce_index;
    BZ_AUTH_LINK_PARAMS* auth;

    ce_index = (UINT16)((UINT32)pvTimerGetTimerID(timer_handle));
    ce_ptr = &lmp_connection_entity[ce_index];
    auth = ce_ptr->auth;

	LMP_LOG_INFO(LOG_LEVEL_HIGH, TID_TIMEOUT_TIMER_EXPIRES_FOR_CE_INDEX, 1, ce_index);

    if (auth->pending_pdu != NULL)
    {
        bz_auth_free_pending_pdu(auth);
    }

    switch (auth->sub_state)
    {
        case BZ_AUTH_SUB_STATE_IDLE:
            break;
        case INTR_LINK_KEY_REQUESTED:
        case INTR_CHALLENGED_REMOTE_HOST:
        case INTR_RESP_AWAIT_AU_RAND_PDU_NOT_ACCEPTED:
        case INTR_AWAIT_AU_RAND_PDU_NOT_ACCEPTED:
        case INTR_PIN_CODE_REQUESTED:
        case INTR_IN_RAND_PDU_SENT:
        case INTR_RESP_AWAIT_IN_RAND_PDU_NOT_ACCEPTED:
        case INTR_SENT_COMB_KEY:
        case INTR_SENT_UNIT_KEY:
        case INTR_CHANGING_KEY_SENT_COMB_KEY:
        case INTR_RESP_CHANGING_KEY_AWAIT_COMB_KEY_NOT_ACCEPTED:
        case INTR_RESP_AWAIT_CHALLENGE:
        case INTR_CHALLENGED_REMOTE_HOST_MUTUAL:
        case INTR_AWAIT_REMOTE_CHALLENGE:
        case RESP_LINK_KEY_REQUESTED:
        case RESP_PIN_CODE_REQUESTED:
        case INTR_RESP_AWAIT_IN_RAND_PDU:
        case RESP_IN_RAND_PDU_SENT:
        case RESP_AWAIT_UNIT_OR_COMB_KEY:
        case RESP_AWAIT_CHALLENGE_MUTUAL:
        case RESP_AWAIT_CHALLENGED_REMOTE_HOST:
        case RESP_AWAIT_TEMP_KEY:
        case USE_SEMI_PER_KEY_PDU_SENT:
            bz_auth_handle_auth_completion(ce_index,
                    LMP_RESPONSE_TIMEOUT_ERROR);
            break;
            
            /* PUBLIC_KEY_EXCHANGE, */
        case PUBLIC_KEY_EXCHANGE_SENDING_ENCAPSULATED_PDU:
        case PUBLIC_KEY_EXCHANGE_RECEIVING_ENCAPSULATED_PDU:
            /* reset edtm module for this connection */
            lmp_edtm_init_linkparams(ce_index);
            /*@fallthrough@*/
        case INTR_IO_CAP_REQUESTED:
        case INTR_AWAIT_REMOTE_IO_CAP_RESPONSE:
        case INTR_RESP_AWAIT_IO_CAP_REQ:
        case RESP_IO_CAP_REQUESTED:
            /* AUTH_STAGE_I, */
            /*  OOB, */
        case OOB_INTR_REMOTE_OOB_DATA_REQUESTED:
        case OOB_INTR_AWAIT_SP_NUM:
        case OOB_INTR_SENT_SP_NUM:
        case OOB_RESP_REMOTE_OOB_DATA_REQUESTED:
        case OOB_RESP_AWAIT_SP_NUM:
        case OOB_RESP_SENT_SP_NUM:
            /*   NC, */
        case NC_INTR_USER_CONFIRM_REQUESTED:
        case NC_INTR_AWAIT_SP_CONFIRM:
        case NC_INTR_AWAIT_SP_NUM:
        case NC_INTR_SENT_SP_NUM:
        case NC_RESP_USER_CONFIRM_REQUESTED:
        case NC_RESP_AWAIT_DH_KEY_TO_SKIP:
        case NC_RESP_AWAIT_SP_NUM:
        case NC_RESP_SENT_SP_NUM:
            /*  PE, */
        case PE_INTR_PASS_KEY_REQUESTED:
        case PE_INTR_AWAIT_SP_NUM:
        case PE_INTR_SENT_SP_CONFIRM:
        case PE_INTR_SENT_SP_NUM:
        case PE_RESP_PASSKEY_REQUESTED:
        case PE_RESP_AWAIT_SP_CONFIRM_TO_SKIP:
        case PE_RESP_AWAIT_SP_CONFIRM:
        case PE_RESP_SENT_SP_CONFIRM:
        case PE_RESP_SENT_SP_NUM:
            /* AUTH_STAGE_II, */
        case INTR_AWAIT_DHKEY_RESP:
        case INTR_AWAIT_DHKEY_CHECK:
        case RESP_AWAIT_DHKEY_CHECK:
        case RESP_AWAIT_DHKEY_RESP:
            bz_auth_handle_simple_pairing_complete(ce_index,
                    LMP_RESPONSE_TIMEOUT_ERROR);
            break;

            /* Encryption Pause or Stop */
        case AWAIT_DISABLE_ENCRYPTION_MODE_REQ:
        case AWAIT_STOP_ENCRYPTION:
        case DISABLE_ENCRYPTION_MODE_REQ_SENT:
        case STOP_ENCRYPTION_SENT:
        case AWAIT_PAUSE_ENCRYPTION:
        case AWAIT_PAUSE_ENCRYPTION_NOT_ACCEPTED:
        case AWAIT_DISABLE_ENCRYPTION_MOD_REQ_NOT_ACCEPTED:
            bz_auth_handle_disable_enc_completion(ce_index,
                    LMP_RESPONSE_TIMEOUT_ERROR);
            break;
            /* Encryption Resume or Start */
        case AWAIT_ENABLE_ENCRYPTION_MODE_REQ:
        case AWAIT_RESUME_ENCRYPTION:
        case AWAIT_ENCRYPTION_KEY_SIZE_REQ:
        case ENABLE_ENCRYPTION_MODE_REQ_SENT:
        case AWAIT_START_ENCRYPTION:
        case ENCRYPTION_KEY_SIZE_SENT:
        case START_ENCRYPTION_SENT:
        case AWAIT_ENABLE_ENCRYPTION_MOD_REQ_NOT_ACCEPTED:
            bz_auth_handle_enable_enc_completion(ce_index,
                    LMP_RESPONSE_TIMEOUT_ERROR);
            break;
		default:
			break;
    }
    return;
}

/**
 * Transitions the sub state machine to the given \a next_state.
 *
 * \param auth Authentication parameters containing the state machine.
 * \param next_state Next sub state to be transitioned to.
 *
 * \return None.
 */
void bz_auth_transition_to_sub_state(BZ_AUTH_LINK_PARAMS* auth, UINT8 next_state)
{
    /* log super_state:sub_state -> next_state */
    auth->sub_state = next_state;
}

/**
 * Performs the super state transition based on the current super state and
 * the input \a event.
 *
 * \param auth Authentication parameters containing the state machine.
 * \param event Event triggering the super state transition.
 *
 * \return None.
 */
void bz_auth_perform_super_state_transition(BZ_AUTH_LINK_PARAMS* auth, UINT8 event)
{
    /* log super_state -> next_state:sub_state */
    auth->super_state = bz_auth_next_super_state[auth->super_state][event];
}

/**
 * Frees any outstanding PDUs queued for delayed processing.
 *
 * \param auth Authentication parameters containing the pending PDU.
 *
 * \return None.
 */
void bz_auth_free_pending_pdu(BZ_AUTH_LINK_PARAMS* auth)
{
    BZ_ASSERT(auth->pending_pdu != NULL, "Why the hell do you free NULL ptr?");

    OS_FREE_BUFFER(lmp_pdu_buffer_pool_handle, auth->pending_pdu);
    auth->pending_pdu = NULL;
}

/**
 * Stores the given \a pdu for delayed processing in the authentication
 * parameters (\a auth) associated with the ACL link.
 *
 * \param auth Authentication parameters containing pending PDU storage.
 * \param pdu PDU to be stored.
 * \param can_free_pdu Pointer to store whether the PDU can be freed by the
 *                     PDU handler module. This will be set to FALSE here as
 *                     the PDU is queued for delayed processing and should not
 *                     be freed.
 *
 * \return None.
 */
void bz_auth_set_pending_pdu(BZ_AUTH_LINK_PARAMS* auth, LMP_PDU_PKT* pdu,
        BOOLEAN* can_free_pdu)
{
    BZ_ASSERT(auth->pending_pdu == NULL, "We already have a pending pdu!!");

    auth->pending_pdu = pdu;
    *can_free_pdu = FALSE;
}

/**
 * Returns whether the SSP is allowed under the current circumstances. It
 * considers the local and remote feature bits to arrive at the decision.
 *
 * \param ce_ptr ACL connection entity pointer.
 *
 * \return TRUE, if the SSP is allowed. FALSE, otherwise.
 */
BOOLEAN bz_auth_is_ssp_allowed(LMP_CONNECTION_ENTITY* ce_ptr)
{
        UCHAR cont, host;
     
		cont = bz_auth_get_local_features(ce_ptr->phy_piconet_id)[6];
		cont = (UCHAR)(cont & bz_auth_get_remote_features(ce_ptr)[6]);

		host = bz_auth_get_local_ext_features(ce_ptr->phy_piconet_id)[0][0];
		host = (UCHAR)(host & bz_auth_get_remote_ext_features(ce_ptr)[0][0]);

        return (BOOLEAN)((host & SECURE_SIMPLE_PAIRING_HOST_SUPPORT)
                && (cont & SECURE_SIMPLE_PAIRING));
}

#ifdef COMPILE_BROADCAST_ENCRYPTION
/**
 * Returns which encryption key size should be used (Point-to-Point
 * enc_key_size or the broadcast enc_key_size).
 *
 * \param auth Authentication parameters containing the input to make the
 *             decision.
 *
 * \return TRUE, if the broadcast (temp) enc_key_size has to be used. FALSE,
 *         otherwise.
 */
BOOLEAN bz_auth_is_to_use_temp_enc_key_size(BZ_AUTH_LINK_PARAMS* auth)
{
    switch(auth->super_state)
    {
        case TEMP_AUTHENTICATED_ENCRIPTING:
        case TEMP_AUTHENTICATED:
        case ENCRYPTED_TEMP_ENCRYPTING:
        case TEMP_ENCRYPTED:
        case TEMP_ENCRYPTED_REFRESH_KEY:
        case TEMP_ENCRYPTED_PAUSED:
        case TEMP_ENCRYPTED_RESUMING_ENCRYPTION:
            return TRUE;
        default:
            return FALSE;
    }
}
#endif /* COMPILE_BROADCAST_ENCRYPTION */

/**
 * Returns which encryption key should be used (Point-to-Point enc_key or the
 * broadcast enc_key).
 *
 * \param auth Authentication parameters containing the input to make the
 *             decision.
 *
 * \return TRUE, if the broadcast (temp) enc_key has to be used. FALSE,
 *         otherwise.
 */
BOOLEAN bz_auth_is_to_use_temp_key_for_encryption(
        BZ_AUTH_LINK_PARAMS* auth)
{
#ifdef COMPILE_BROADCAST_ENCRYPTION
    switch(auth->super_state)
    {
        case ENCRYPTED_TEMP_ENCRYPTING:
        case TEMP_AUTHENTICATED_ENCRIPTING:
        case TEMP_ENCRYPTED_REFRESH_KEY:
        case TEMP_ENCRYPTED_RESUMING_ENCRYPTION:
            return TRUE;
        default:
            return FALSE;
    }
#else
    return FALSE;
#endif /* COMPILE_BROADCAST_ENCRYPTION */
}

/**
 * Generate and send LMP_unit_key PDU to the remote device.
 *
 * \param ce_index ACL connection entity index.
 * \param auth Authentication parameter to be associated with the \a ce_index.
 * \param tid Transaction ID to be used for the PDU.
 *
 * \return None.
 */
void bz_auth_send_unit_key_pdu(UINT16 ce_index, BZ_AUTH_LINK_PARAMS* auth,
        LMP_TRAN_ID tid)
{
    UCHAR param_list[LMP_UNIT_KEY_LEN];
    register int i;

    param_list[0] = LMP_UNIT_KEY_OPCODE;
    /* copy the unit key */
    memcpy(&param_list[2], bz_auth_dev_params.unit_key,
            BZ_AUTH_LINK_KEY_SIZE);
    memcpy(&auth->txn_params.local_uck[0],
            bz_auth_dev_params.unit_key, BZ_AUTH_LINK_KEY_SIZE);

    /* protect the unit key with the current link key */
    for (i = 0; i < BZ_AUTH_LINK_KEY_SIZE; i++)
    {
        param_list[2+i] = (UCHAR)(param_list[2+i]^auth->txn_params.init_key[i]);
    }

    /* send protected unit key */
    bz_auth_generate_pdu(ce_index, param_list, LMP_UNIT_KEY_LEN, tid);
}

/**
 * Generate and send LMP_comb_key PDU to the remote device.
 *
 * \param ce_index ACL connection entity index.
 * \param auth Authentication parameter to be associated with the \a ce_index.
 * \param tid Transaction ID to be used for the PDU.
 *
 * \return None.
 */
void bz_auth_send_comb_key_pdu(UINT16 ce_index, BZ_AUTH_LINK_PARAMS* auth,
        LMP_TRAN_ID tid)
{
    UCHAR param_list[LMP_COMB_KEY_LEN];
    register int i;

    param_list[0] = LMP_COMB_KEY_OPCODE;
    ssp_rng_get(&param_list[2]);    /* get comb_key_rand(random_number) */

#ifdef _CCH_SC_TEST_20130129_MRV_01
    lmp_connection_entity[ce_index].done_h3 = 0;
#ifdef _SECURE_CONN_TEST_LOG        
    RT_BT_LOG(YELLOW, DAPE_TEST_LOG522, 2,auth->secure_conn_enabled, lmp_connection_entity[ce_index].done_h3);
#endif
#endif

    /* Calculate self device combination key */
    lp_E21(&param_list[2], bz_auth_get_local_bd_addr(),
            &auth->txn_params.local_uck[0]);

    /* protect the comb key random number with current link key */
    for (i = 0; i < BZ_AUTH_LINK_KEY_SIZE; i++)
    {
        param_list[2+i] = (UCHAR)(param_list[2+i]^auth->txn_params.init_key[i]);
    }

    /* send protected random number */
    bz_auth_generate_pdu(ce_index, param_list, LMP_COMB_KEY_LEN, tid);
}

/**
 * Send either LMP_unit_key or LMP_comb_key based on the current device
 * configuration. It also performs the state transitions based on the PDU
 * sent. This is done as initiator.
 *
 * \param ce_index ACL connection entity index.
 * \param auth Authentication parameters associated with the \a ce_index.
 *
 * \return None.
 */
void bz_auth_intr_send_unit_or_comb_key(UINT16 ce_index,
        BZ_AUTH_LINK_PARAMS* auth)
{
    if (bz_auth_is_unit_key_enabled())
    {
        /* compute unit key and send it */
        bz_auth_send_unit_key_pdu(ce_index, auth, SELF_DEV_TID);
        bz_auth_transition_to_sub_state(auth, INTR_SENT_UNIT_KEY);
    }
    else
    {
        /* compute comb key and send it */
        bz_auth_send_comb_key_pdu(ce_index, auth, SELF_DEV_TID);
        bz_auth_transition_to_sub_state(auth, INTR_SENT_COMB_KEY);
    }
}

/**
 * Computes the final link key from the local unit/comb_key and remote
 * unit/comb_key. The computed key and the key_type is updated in
 * auth->txn_params.
 *
 * \param ce_index ACL connection entity index.
 * \param auth Authentication parameters associated with \a ce_index.
 * \param local_type Local contribution type
 *                   (LMP_COMB_KEY_OPCODE/LMP_UNIT_KEY_OPCODE).
 * \param remote_uck Remote unit or combination key. This is not the content
 *                   received in the LMP_COMB_KEY_PDU or LMP_UNIT_KEY. It is
 *                   the computed keys from that content.
 * \param remote_type Remote contribution type
 *                    (LMP_COMB_KEY_OPCODE/LMP_UNIT_KEY_OPCODE).
 *
 * \return None.
 */
void bz_auth_compute_link_key_from_contributions(UINT16 ce_index,
        BZ_AUTH_LINK_PARAMS* auth, UCHAR local_type,
        const UCHAR* remote_uck, UCHAR remote_type)
{
    const UCHAR* final_key;
    register int i;

    if (local_type == LMP_COMB_KEY_OPCODE
            && remote_type == LMP_COMB_KEY_OPCODE)
    {
        auth->txn_params.lk_type = BZ_AUTH_LINK_KEY_TYPE_COMBINATION_KEY;

        /* compute link key from both the combination keys */
        for (i = 0; i < BZ_AUTH_LINK_KEY_SIZE; i++)
        {
            auth->txn_params.link_key[i] = (UCHAR)
                (auth->txn_params.local_uck[i] ^ remote_uck[i]);
        }
        if (bz_auth_dev_params.ssp_mode == BZ_AUTH_SSP_MODE_ENABLED
                && (auth->super_state == AUTHENTICATED_CHANGING_KEY
                    || auth->super_state == ENCRYPTED_CHANGING_KEY))
        {
            auth->txn_params.lk_type =
                BZ_AUTH_LINK_KEY_TYPE_CHANGED_COMBINATION_KEY;
        }
        return;
    }
    else if (local_type == LMP_UNIT_KEY_OPCODE
            && remote_type == LMP_UNIT_KEY_OPCODE)
    {
        if (bz_auth_is_master(&lmp_connection_entity[ce_index]))
        {
            /* use self unit_key */
            auth->txn_params.lk_type = BZ_AUTH_LINK_KEY_TYPE_LOCAL_UNIT_KEY;
            final_key = auth->txn_params.local_uck;
        }
        else
        {
            /* use remote unit_key */
            auth->txn_params.lk_type = BZ_AUTH_LINK_KEY_TYPE_REMOTE_UNIT_KEY;
            final_key = remote_uck;
        }
    }
    else if (local_type == LMP_UNIT_KEY_OPCODE)
    {
        /* use local unit_key */
        auth->txn_params.lk_type = BZ_AUTH_LINK_KEY_TYPE_LOCAL_UNIT_KEY;
        final_key = auth->txn_params.local_uck;
    }
    else
    {
        /* use remote unit_key */
        auth->txn_params.lk_type = BZ_AUTH_LINK_KEY_TYPE_REMOTE_UNIT_KEY;
        final_key = remote_uck;
    }

    memcpy(&auth->txn_params.link_key[0], final_key, BZ_AUTH_LINK_KEY_SIZE);
}

/**
 * Computes the final link key from the remote pdu (\a lmp_pdu_ptr) and local
 * unit or comb_key.
 *
 * \param ce_index ACL connection entity index.
 * \param auth Authentication parameters associated with \a ce_index.
 * \param local_type The local contribution type
 *                   (LMP_COMB_KEY_OPCODE/LMP_UNIT_KEY_OPCODE).
 * \param lmp_pdu_ptr Remote LMP_comb_key or LMP_unit_key PDU. The pdu content
 *                    is modified to get the final key.
 *
 * \return None.
 */
void bz_auth_compute_final_key_from_pdu(UINT16 ce_index,
        BZ_AUTH_LINK_PARAMS* auth, UCHAR local_type,
        INOUT LMP_PDU_PKT* lmp_pdu_ptr)
{
    UCHAR remote_uck[BZ_AUTH_LINK_KEY_SIZE];
    UCHAR* remote_rand;
    UCHAR remote_type;
    register int i;

    remote_type = bz_auth_get_pdu_opcode(lmp_pdu_ptr);
    BZ_ASSERT(remote_type == LMP_COMB_KEY_OPCODE
            || remote_type == LMP_UNIT_KEY_OPCODE,
            "We can not entertain any other PDUs here");

    remote_rand = &lmp_pdu_ptr->payload_content[1];
    for (i = 0; i < BZ_AUTH_LINK_KEY_SIZE; i++)
    {
        remote_rand[i] = (UCHAR)(remote_rand[i] ^ auth->txn_params.init_key[i]);
    }

    if (remote_type == LMP_COMB_KEY_OPCODE)
    {
        /* lmp_comb_key PDU contains the random number XORed with init_key.
         * This random number was used by the remote device to calculate it's
         * combination-key-contribution. So we have to calculate his
         * contribution.
         */
        lp_E21(remote_rand,
                bz_auth_get_remote_bd_addr(&lmp_connection_entity[ce_index]),
                &remote_uck[0]);
    }
    else
    {
        /* lmp_unit_key PDU contains the unit_key XORed with init_key,
         * so copy the value as it is.
         */
        memcpy(&remote_uck[0], remote_rand, BZ_AUTH_LINK_KEY_SIZE);
    }

    bz_auth_compute_link_key_from_contributions(ce_index, auth, local_type,
            remote_uck, remote_type);
}

/**
 * Send either LMP_unit_key or LMP_comb_key based on the current device
 * configuration. It also performs the state transitions based on the PDU
 * sent. This is done as a responder.
 *
 * \param ce_index ACL connection entity index.
 * \param auth Authentication parameters associated with the \a ce_index.
 * \param lmp_pdu_ptr LMP pdu packet received from the remote device. The PDU
 *                    contains the remote device contribution to the link key
 *                    to be generated.
 *
 * \return None.
 */
void bz_auth_resp_send_unit_or_comb_key(UINT16 ce_index,
        BZ_AUTH_LINK_PARAMS* auth, LMP_PDU_PKT* lmp_pdu_ptr)
{
    LMP_TRAN_ID tid;
    UCHAR local_type;

    tid = (LMP_TRAN_ID)LMP_GET_TRANSACTION_ID(lmp_pdu_ptr);
    if (bz_auth_is_unit_key_enabled())
    {
        /* compute unit key and send it */
        bz_auth_send_unit_key_pdu(ce_index, auth, tid);
        local_type = LMP_UNIT_KEY_OPCODE;
    }
    else
    {
        /* compute comb key and send it */
        bz_auth_send_comb_key_pdu(ce_index, auth, tid);
        local_type = LMP_COMB_KEY_OPCODE;
    }
#ifdef SECURE_CONN_MUTUAL_AUTH
    if (auth->secure_conn_enabled)
    {
        bz_auth_transition_to_sub_state(auth,
                SECURE_CONN_MUTUAL_AWAIT_AU_RAND);
    }
    else
#endif
    {
    bz_auth_transition_to_sub_state(auth, RESP_AWAIT_CHALLENGE_MUTUAL);
    }
    /* compute the final key */
    bz_auth_compute_final_key_from_pdu(ce_index, auth, local_type, lmp_pdu_ptr);
}


/**
 * Generate and send LMP_in_rand PDU to the remote device.
 *
 * \param ce_index ACL connection entity index.
 * \param auth Authentication parameter to be associated with the \a ce_index.
 * \param tid Transaction ID to be used for the PDU.
 *
 * \return None.
 */
void bz_auth_send_in_rand_pdu(UINT16 ce_index, BZ_AUTH_LINK_PARAMS* auth,
        LMP_TRAN_ID tid)
{
    UCHAR param_list[LMP_IN_RAND_LEN];

    param_list[0] = LMP_IN_RAND_OPCODE;
    ssp_rng_get(&param_list[2]);    /* get in_rand(random_number) */

    /* generate init key */
    lp_E22(&param_list[2], auth->txn_params.pin, auth->txn_params.pin_len,
            bz_auth_get_remote_bd_addr(&lmp_connection_entity[ce_index]),
            &auth->txn_params.init_key[0]);

    bz_auth_generate_pdu(ce_index, param_list, LMP_IN_RAND_LEN, tid);
}

/**
 * Generate and send LMP_accepted(LMP_in_rand) PDU to the remote device.
 *
 * \param ce_index ACL connection entity index.
 * \param auth Authentication parameter to be associated with the \a ce_index.
 * \param lmp_pdu_ptr LMP pdu packet containing the remote LMP_in_rand for
 *                    which the LMP_accepted is being sent.
 *
 * \return None.
 */
void bz_auth_send_in_rand_accepted_pdu(UINT16 ce_index,
        BZ_AUTH_LINK_PARAMS* auth, LMP_PDU_PKT* lmp_pdu_ptr)
{
    BZ_ASSERT(bz_auth_get_pdu_opcode(lmp_pdu_ptr) == LMP_IN_RAND_OPCODE,
            "The pending pdu must be LMP_in_rand");

    /* generate init key */
    lp_E22(&lmp_pdu_ptr->payload_content[1] /* in_rand */, auth->txn_params.pin,
            auth->txn_params.pin_len, bz_auth_get_local_bd_addr(),
            &auth->txn_params.init_key[0]);

    bz_auth_send_accepted_pdu(ce_index, LMP_IN_RAND_OPCODE,
            (LMP_TRAN_ID)LMP_GET_TRANSACTION_ID(lmp_pdu_ptr));
}

/**
 * Sends LMP_in_rand or LMP_accepted(LMP_in_rand) as a response to \a
 * lmp_pdu_ptr based on the pin type. It also calculates the
 * initialization key and stores it in the transaction parameters and
 * transitions to appropriate sub state.
 *
 * \param ce_index ACL connection entity index.
 * \param auth Authentication parameters corresponding to \a ce_index.
 * \param lmp_pdu_ptr LMP_in_rand PDU packet for which the response has to be
 *                    sent.
 *
 * \return None.
 *
 * \note This function should be called only as responder or
 *       initiator_as_responder. The TID of the sent pdu is same as
 *       lmp_pdu_ptr(TID).
 */
void bz_auth_resp_send_in_rand_or_in_rand_accepted(UINT16 ce_index,
        BZ_AUTH_LINK_PARAMS* auth, LMP_PDU_PKT* lmp_pdu_ptr)
{
    BZ_ASSERT(bz_auth_get_pdu_opcode(lmp_pdu_ptr) == LMP_IN_RAND_OPCODE,
            "The pending pdu must be LMP_in_rand");

    if (bz_auth_is_fixed_pin()) /* we have fixed pin */
    {
        /* send lmp_in_rand and calculate init_key */
        bz_auth_send_in_rand_pdu(ce_index, auth,
                (LMP_TRAN_ID)LMP_GET_TRANSACTION_ID(lmp_pdu_ptr));
        bz_auth_transition_to_sub_state(auth, RESP_IN_RAND_PDU_SENT);
    }
    else
    {
        /* send lmp_accepted and calculate init_key */
        bz_auth_send_in_rand_accepted_pdu(ce_index, auth, lmp_pdu_ptr);
        bz_auth_transition_to_sub_state(auth, RESP_AWAIT_UNIT_OR_COMB_KEY);
    }
}

/**
 * Check whether the locally computed SRES and remote SRES are same.
 *
 * \param auth Authentication parameters containing the SRESs to be compared.
 *
 * \return TRUE, if both the computed and received SRES are same.
 */
BOOLEAN bz_auth_is_sres_same(BZ_AUTH_LINK_PARAMS* auth)
{
#ifdef _SUPPORT_SECURE_CONNECTION_
#ifdef _SECURE_CONN_TEST_LOG
RT_BT_LOG(BLUE, DAPE_TEST_LOG525, 6, auth->secure_conn_enabled, 
auth->len_prime, auth->txn_params.sres_m[0], auth->txn_params.sres_m[1],
auth->txn_params.remote_sres[0], auth->txn_params.remote_sres[1]);
#endif
    if (auth->secure_conn_enabled)
    {
        if( lmp_connection_entity[auth->ce_index].remote_dev_role == SLAVE )
        {
            if (memcmp(&auth->txn_params.sres_s[0],
                        &auth->txn_params.remote_sres[0], BZ_AUTH_SRES_SIZE) == 0x0)
            {
                return TRUE;
            }
        }
        else
        {
            if (memcmp(&auth->txn_params.sres_m[0],
                        &auth->txn_params.remote_sres[0], BZ_AUTH_SRES_SIZE) == 0x0)
            {
                return TRUE;    
            }
        }

    }
    else
#endif
    {
    if (memcmp(&auth->txn_params.local_sres[0],
                &auth->txn_params.remote_sres[0], BZ_AUTH_SRES_SIZE) == 0x0)
    {
        return TRUE;
    }
    }
    return FALSE;
}

/**
 * Returns whether the transaction (\a lmp_pdu_ptr) is initiated by remote
 * device or not.
 *
 * \param ce_ptr ACL Connection entity pointer.
 * \param lmp_pdu_ptr LMP PDU packet for which the transaction role has to be
 *                    found.
 *
 * \return TRUE, if the transaction is remote device initiated. FALSE,
 *         otherwise.
 */
BOOLEAN bz_auth_is_remote_initiated(LMP_CONNECTION_ENTITY* ce_ptr,
        LMP_PDU_PKT *lmp_pdu_ptr)
{
    /*
     * if remote_role == TID, then return true.
     */
     if (ce_ptr->remote_dev_role
            == ((UCHAR)LMP_GET_TRANSACTION_ID(lmp_pdu_ptr)))
     {
        return TRUE;
     }
     return FALSE;
}

/**
 * Generate and send LMP_sres PDU to the remote device. It computes the SRES
 * using the Nonce value from the LMP_au_rand.
 *
 * \param ce_index ACL connection entity index.
 * \param auth Authentication parameter to be associated with the \a ce_index.
 * \param lmp_pdu_ptr LMP PDU packet containing the remote LMP_au_rand PDU for
 *                    which the LMP_sres PDU has to be sent.
 *
 * \return None.
 */
void bz_auth_send_sres_pdu(UINT16 ce_index, BZ_AUTH_LINK_PARAMS* auth,
        LMP_PDU_PKT* lmp_pdu_ptr)
{
    UCHAR param_list[LMP_SRES_LEN];
    LMP_TRAN_ID tid;

    BZ_ASSERT(bz_auth_get_pdu_opcode(lmp_pdu_ptr) == LMP_AU_RAND_OPCODE,
            "The pending pdu must be LMP_au_rand");
#ifdef _CCH_SC_ECDH_P256_ 
    if(auth->len_prime == 6)
    {
        lp_E1(auth->txn_params.link_key,
            &lmp_pdu_ptr->payload_content[1] /* au_rand pdu */,
            bz_auth_get_local_bd_addr(),
            &param_list[2] /* sres */, &auth->txn_params.aco[0]);
    }
    else if(auth->len_prime == 8)
    {
        if( lmp_connection_entity[ce_index].remote_dev_role == MASTER )
        {
            memcpy(auth->txn_params.au_rand_m, &lmp_pdu_ptr->payload_content[1], 16);	
#ifdef _CCH_SC_TEST_BDADDR_MSBX
            bz_auth_convert_to_msb(&auth->txn_params.au_rand_m[0], 16);	
#endif
        }
        else
        {
            memcpy(auth->txn_params.au_rand_s, &lmp_pdu_ptr->payload_content[1], 16);
#ifdef _CCH_SC_TEST_BDADDR_MSBX
            bz_auth_convert_to_msb(&auth->txn_params.au_rand_s[0], 16);	
#endif			
        }

        if(lmp_connection_entity[ce_index].remote_dev_role == MASTER)
        {
            lp_h4(auth->txn_params.link_key, (u8*)"btdk", 
                   lmp_connection_entity[ce_index].bd_addr, bz_auth_get_local_bd_addr(), &auth->txn_params.dev_auth_key[0]);
        }
        else
        {
            lp_h4(auth->txn_params.link_key, (u8*)"btdk", 
                   bz_auth_get_local_bd_addr(), lmp_connection_entity[ce_index].bd_addr, &auth->txn_params.dev_auth_key[0]);
        }

        lp_h5(auth->txn_params.dev_auth_key,
	    auth->txn_params.au_rand_m, auth->txn_params.au_rand_s,
            &auth->txn_params.sres_m[0], &auth->txn_params.sres_s[0], &auth->txn_params.aco[0]);

#ifdef _CCH_SC_ECDH_P256_PAUSE_ENC
        BOOLEAN enc_on = bz_auth_is_link_encrypted(ce_index);

#ifdef _CCH_SC_ECDH_P256_LOG
        RT_BT_LOG(BLUE, CCH_DBG_154, 1, enc_on);
#endif

        if ( enc_on == FALSE)
        {
            BB_write_sc_iv(ce_index, &auth->txn_params.aco[0]);
		
#ifdef _CCH_SC_ECDH_P256_LOG
RT_BT_LOG(RED, CCH_DBG_153, 20,
auth->txn_params.aco[0], auth->txn_params.aco[1], auth->txn_params.aco[2],
auth->txn_params.aco[3], auth->txn_params.aco[4], auth->txn_params.aco[5],
auth->txn_params.aco[6], auth->txn_params.aco[7], auth->txn_params.aco[8],
auth->txn_params.aco[9], auth->txn_params.aco[10], auth->txn_params.aco[11], 
auth->txn_params.sres_m[0],auth->txn_params.sres_m[1],
auth->txn_params.sres_m[2],auth->txn_params.sres_m[3],
auth->txn_params.sres_s[0],auth->txn_params.sres_s[1],
auth->txn_params.sres_s[2], auth->txn_params.sres_s[3]);
#endif

        }
#endif

        if( lmp_connection_entity[ce_index].remote_dev_role == MASTER )
        {
            memcpy(&param_list[2], &auth->txn_params.sres_s[0], 4);
        }
        else
        {
            memcpy(&param_list[2], &auth->txn_params.sres_m[0], 4);
        }

    }
 
#else
	lp_E1(auth->txn_params.link_key,
	        &lmp_pdu_ptr->payload_content[1] /* au_rand pdu */,
            bz_auth_get_local_bd_addr(),
            &param_list[2] /* sres */, &auth->txn_params.aco[0]);
#endif

    param_list[0] = LMP_SRES_OPCODE;
    tid = (LMP_TRAN_ID)LMP_GET_TRANSACTION_ID(lmp_pdu_ptr);
    bz_auth_generate_pdu(ce_index, param_list, LMP_SRES_LEN, tid);
}

/**
 * Generate and send LMP_au_rand PDU to the remote device.
 *
 * \param ce_index ACL connection entity index.
 * \param auth Authentication parameter to be associated with the \a ce_index.
 * \param tid Transaction ID to be used for the PDU.
 *
 * \return None.
 */
void bz_auth_send_au_rand_pdu(UINT16 ce_index, BZ_AUTH_LINK_PARAMS* auth,
        LMP_TRAN_ID tid)
{
    UCHAR param_list[LMP_AU_RAND_LEN];

	param_list[0] = LMP_AU_RAND_OPCODE;

#ifdef _CCH_SC_ECDH_P256_TEST_SET_1

	u8 R1[16] = {0xd5, 0xcb, 0x84, 0x54, 0xd1, 0x77, 0x73, 0x3e, 0xff, 0xff, 0xb2, 0xec, 0x71, 0x2b, 0xae, 0xab};
	u8 R2[16] = {0xa6, 0xe8, 0xe7, 0xcc, 0x25, 0xa7, 0x5f, 0x6e, 0x21, 0x65, 0x83, 0xf7, 0xff, 0x3d, 0xc4, 0xcf};

#ifdef _CCH_SC_ECDH_P256_TEST_SET_1_M
    bz_auth_convert_to_msb(&R1[0], 16);	
#else
    bz_auth_convert_to_msb(&R2[0], 16);	
#endif

#ifdef _CCH_SC_ECDH_P256_TEST_SET_1_M
    memcpy(&param_list[2], &R1[0], 16);
#else
    memcpy(&param_list[2], &R2[0], 16);
#endif



#else
    ssp_rng_get(&param_list[2]);    /* get au_rand(random_number) */
#endif

#ifdef SECURE_CONN_MUTUAL_AUTH
    if (auth->secure_conn_enabled)
    {
        if (lmp_connection_entity[ce_index].remote_dev_role == MASTER)
        {
            memcpy(auth->txn_params.au_rand_s, &param_list[2], 16);
        }
        else
        {
            memcpy(auth->txn_params.au_rand_m, &param_list[2], 16);
        }
    }
    else
#endif
#ifdef _CCH_SC_ECDH_P256_ 
    if(auth->len_prime == 6)
#endif    
    {
        /* generate local_sres and ACO */
	    lp_E1(auth->txn_params.link_key, &param_list[2],
	        bz_auth_get_remote_bd_addr(&lmp_connection_entity[ce_index]),
            &auth->txn_params.local_sres[0], &auth->txn_params.aco[0]);
    }

    bz_auth_generate_pdu(ce_index, param_list, LMP_AU_RAND_LEN, tid);
}

/**
 * Generate and send LMP_stop_encryption  PDU to the remote device. It also
 * disables encryption for RX.
 *
 * \param ce_index ACL connection entity index.
 * \param auth Authentication parameter to be associated with the \a ce_index.
 * \param tid Transaction ID to be used for the PDU.
 *
 * \return None.
 */
void bz_auth_send_stop_encryption_pdu(UINT16 ce_index,
        BZ_AUTH_LINK_PARAMS* auth, LMP_TRAN_ID tid)
{
	UCHAR param_list[LMP_STOP_ENCRYPTION_REQ_LEN];
	LMP_CONNECTION_ENTITY *ce_ptr;

	ce_ptr = &lmp_connection_entity[ce_index];

	param_list[0] = LMP_STOP_ENCRYPTION_REQ_OPCODE;
	bz_auth_generate_pdu(ce_index, param_list, 
		LMP_STOP_ENCRYPTION_REQ_LEN, tid);

#ifdef _CCH_SC_ECDH_P256_STOP_ENC	
        ce_ptr->send_stop_enc = 1;
#else
	bz_auth_disable_link_level_encryption(ce_index, FALSE);
	/* Configure to transmit encrypted and receive unencrypted */
	BB_encryption_control(ce_ptr->am_addr, ce_ptr->phy_piconet_id, 
		BB_ENC_TX_ENABLE_RX_DISABLE);
#ifdef _CCH_SC_ECDH_P256_PAUSE_ENC
    if(auth->secure_conn_enabled)
    {
        auth->sc_use_enc_rand = 1;
#ifdef _CCH_SC_ECDH_P256_LOG    
        RT_BT_LOG(BLUE, CCH_DBG_155, 0, 0);
#endif
    }
#endif
#endif
	return;
}

/**
 * Generate and send LMP_start_encryption PDU to the remote device. It also
 * enables the encryption for RX.
 *
 * \param ce_index ACL connection entity index.
 * \param auth Authentication parameter to be associated with the \a ce_index.
 * \param tid Transaction ID to be used for the PDU.
 *
 * \return None.
 */
void bz_auth_send_start_encryption_pdu(UINT16 ce_index,
        BZ_AUTH_LINK_PARAMS* auth, LMP_TRAN_ID tid)
{
    UCHAR param_list[LMP_START_ENCRYPTION_REQ_LEN];
	LMP_CONNECTION_ENTITY *ce_ptr;

	ce_ptr = &lmp_connection_entity[ce_index];

#ifdef COMPILE_BROADCAST_ENCRYPTION
    if (bz_auth_is_to_use_temp_key_for_encryption(auth))
    {
        /* Broadcast encryption key has been generated at the beginning */
        memcpy(&param_list[2], &bz_auth_dev_params.mlk.enc_rand[0],
                BZ_AUTH_ENC_RAND_SIZE);
        memcpy(&auth->enc_key[0], &bz_auth_dev_params.mlk.enc_key[0],
                BZ_AUTH_MAX_ENC_KEY_SIZE);
    }
    else
#endif /* COMPILE_BROADCAST_ENCRYPTION */
    {
        /* Get en_rand and compute encryption key */
        ssp_rng_get(&param_list[2]);

#ifdef _CCH_SC_ECDH_P256_ 
        if(auth->len_prime == 6)
        {
            lp_E3(auth->link_key, &param_list[2], auth->aco, &auth->enc_key[0],
                auth->enc_key_size);
        }else if(auth->len_prime == 8)
        {
#ifdef _SECURE_CONN_TEST_LOG        
#ifdef _CCH_SC_TEST_20130129_MRV_01
RT_BT_LOG(YELLOW, DAPE_TEST_LOG522, 2,auth->secure_conn_enabled, ce_ptr->done_h3);
#endif
#endif
#ifdef _CCH_SC_TEST_20130129_MRV_01
            if(ce_ptr->done_h3 == 0)
            {
                if( lmp_connection_entity[ce_index].remote_dev_role == MASTER )
                {
                    lp_h3(auth->link_key, (u8*)"btak",
                    ce_ptr->bd_addr, bz_auth_get_local_bd_addr(), auth->aco, &auth->enc_key[0]);
                }
                else
                {
                    lp_h3(auth->link_key, (u8*)"btak",
                        bz_auth_get_local_bd_addr(), ce_ptr->bd_addr, auth->aco, &auth->enc_key[0]);
                }
                ce_ptr->done_h3 = 1;
            }
#else
            if( lmp_connection_entity[ce_index].remote_dev_role == MASTER )
            {
                lp_h3(auth->link_key, (u8*)"btak",
                    ce_ptr->bd_addr, bz_auth_get_local_bd_addr(), auth->aco, &auth->enc_key[0]);
            }
            else
            {
                lp_h3(auth->link_key, (u8*)"btak",
                    bz_auth_get_local_bd_addr(), ce_ptr->bd_addr, auth->aco, &auth->enc_key[0]);
            }	
#endif			
        }
#else
        lp_E3(auth->link_key, &param_list[2], auth->aco, &auth->enc_key[0],
                auth->enc_key_size);
#endif
    }

    /* Send lmp_start_encryption_req */
    param_list[0] = LMP_START_ENCRYPTION_REQ_OPCODE;
    bz_auth_generate_pdu(ce_index, param_list, 
                LMP_START_ENCRYPTION_REQ_LEN, tid);
    bz_auth_enable_link_level_encryption(ce_index, FALSE);

#ifdef _CCH_SC_ECDH_P256_START_ENC	
    ce_ptr->send_start_enc = 1;
#else
    /* Configure to transmit unencrypted and receive encrypted */
    BB_encryption_control(ce_ptr->am_addr, ce_ptr->phy_piconet_id, 
        BB_ENC_TX_DISABLE_RX_ENABLE);
#endif
    return;
}

/**
 * Returns the maximum encryption key size supported by the local device.
 *
 * \param auth Authentication parameters containing the necessary inputs to
 *             determine the maximum encryption key size.
 *
 * \return Maximum encryption key size supported by the local device.
 */
UCHAR bz_auth_local_supported_max_enc_key_size(BZ_AUTH_LINK_PARAMS* auth)
{
#ifdef COMPILE_BROADCAST_ENCRYPTION
    if (bz_auth_is_to_use_temp_enc_key_size(auth))
    {
        /* use master link key size */
        return bz_auth_dev_params.max_enc_key_size;
    }
    else
#endif
    {
        return bz_auth_dev_params.max_enc_key_size;
    }
}

/**
 * Returns the minimum encryption key size supported by the local device.
 *
 * \param ce_ptr ACL connection entity pointer.
 * \param auth Authentication parameters containing the necessary inputs to
 *             determine the minimum encryption key size.
 *
 * \return Minimum encryption key size supported by the local device.
 */
UCHAR bz_auth_local_supported_min_enc_key_size(LMP_CONNECTION_ENTITY* ce_ptr,
        BZ_AUTH_LINK_PARAMS* auth)
{
#ifdef COMPILE_BROADCAST_ENCRYPTION
    if (bz_auth_is_master(ce_ptr)
            && bz_auth_is_to_use_temp_enc_key_size(auth))
    {
        /* use master link key size as min enc key size */
        return bz_auth_dev_params.max_enc_key_size;
    }
    else
#endif
    {
        return bz_auth_dev_params.min_enc_key_size;
    }
}

/**
 * Generate and send LMP_encryption_mode_req PDU to the remote device.
 *
 * \param ce_index ACL connection entity index.
 * \param auth Authentication parameter to be associated with the \a ce_index.
 * \param tid Transaction ID to be used for the PDU.
 * \param enc_mode Encryption mode being requested.
 *
 * \return None.
 */
void bz_auth_send_encryption_mode_req_pdu(UINT16 ce_index,
        BZ_AUTH_LINK_PARAMS* auth, LMP_TRAN_ID tid, UINT8 enc_mode)
{
    UCHAR param_list[LMP_ENCRYPTION_MODE_REQ_LEN];

    /* Send encryption_mode_req(enable) */
    param_list[0] = LMP_ENCRYPTION_MODE_REQ_OPCODE;
    param_list[2] = (UCHAR)enc_mode;
    bz_auth_generate_pdu(ce_index, param_list, LMP_ENCRYPTION_MODE_REQ_LEN,
            tid);

    /* Transition to corresponding state */
    if (enc_mode == BZ_AUTH_ENCRYPTION_MODE_ON)
    {
        bz_auth_transition_to_sub_state(auth, ENABLE_ENCRYPTION_MODE_REQ_SENT);
    }
    else
    {
        bz_auth_transition_to_sub_state(auth, DISABLE_ENCRYPTION_MODE_REQ_SENT);
    }
}

/**
 * Generate and send LMP_encryption_key_size_req PDU to the remote device.
 *
 * \param ce_index ACL connection entity index.
 * \param auth Authentication parameter to be associated with the \a ce_index.
 * \param tid Transaction ID to be used for the PDU.
 *
 * \return None.
 */
void bz_auth_send_encryption_key_size_req_pdu(UINT16 ce_index,
        BZ_AUTH_LINK_PARAMS* auth, LMP_TRAN_ID tid)
{
    UCHAR param_list[LMP_ENCRYPTION_KEY_SIZE_REQ_LEN];

    param_list[0] = LMP_ENCRYPTION_KEY_SIZE_REQ_OPCODE;
    param_list[2] = auth->enc_key_size;
    bz_auth_generate_pdu(ce_index, param_list, LMP_ENCRYPTION_KEY_SIZE_REQ_LEN,
            tid);
    /* Transition to appropriate state */
    bz_auth_transition_to_sub_state(auth, ENCRYPTION_KEY_SIZE_SENT);
}

/**
 * Decides the response for the received LMP_encryption_mode_req(enable) PDU
 * and sends it to the remote device.
 *
 * \param ce_index ACL connection entity index.
 * \param auth Authentication parameters associated with the \a ce_index.
 * \param lmp_pdu_ptr LMP PDU packet containing the LMP_encryption_mode_req
 *                    PDU received from the remote device.
 *
 * \return None.
 */
void bz_auth_send_response_to_enable_encryption_mode_req_pdu(UINT16 ce_index,
        BZ_AUTH_LINK_PARAMS* auth, LMP_PDU_PKT* lmp_pdu_ptr)
{
    LMP_TRAN_ID tid;

    tid = (LMP_TRAN_ID)LMP_GET_TRANSACTION_ID(lmp_pdu_ptr);
    bz_auth_send_accepted_pdu(ce_index, LMP_ENCRYPTION_MODE_REQ_OPCODE, tid);
    if (bz_auth_is_master(&lmp_connection_entity[ce_index]))
    {
        /* send lmp_key_size pdu */
        auth->enc_key_size =
            bz_auth_local_supported_max_enc_key_size(auth);
        /* Send encryption key size req and transition to appropriate state
         * (ENCRYPTION_KEY_SIZE_SENT).
         */
        bz_auth_send_encryption_key_size_req_pdu(ce_index, auth, tid);
    }
    else
    {
        bz_auth_transition_to_sub_state(auth, AWAIT_ENCRYPTION_KEY_SIZE_REQ);
    }
}

/**
 * Send LMP_encryption_key_size_mask_req PDU to the remote device.
 *
 * \param ce_index ACL connection entity index.
 *
 * \return None.
 */
void bz_auth_send_encryption_key_size_mask_req_pdu(UINT16 ce_index)
{
    UCHAR param_list[LMP_ENCRYPTION_KEY_SIZE_MASK_REQ_LEN];
    param_list[0] = LMP_ENCRYPTION_KEY_SIZE_MASK_REQ_OPCODE;
    bz_auth_generate_pdu(ce_index, param_list,
            LMP_ENCRYPTION_KEY_SIZE_MASK_REQ_LEN, SELF_DEV_TID);
}

/**
 * Generate and send LMP_encryption_key_size_mask_res PDU to the remote device.
 *
 * \param ce_index ACL connection entity index.
 * \param auth Authentication parameter to be associated with the \a ce_index.
 * \param tid Transaction ID to be used for the PDU.
 *
 * \return None.
 */
void bz_auth_send_encryption_key_size_mask_res_pdu(UINT16 ce_index,
        BZ_AUTH_LINK_PARAMS* auth, LMP_TRAN_ID tid)
{
    UCHAR param_list[LMP_ENCRYPTION_KEY_SIZE_MASK_RES_LEN];
    UINT16 key_size_mask=0xFFFF;
    param_list[0] = LMP_ENCRYPTION_KEY_SIZE_MASK_RES_OPCODE;
    /*
     * All encryption key sizes  from bz_auth_dev_params.min_enc_key_size to
     * bz_auth_dev_params->max_enc_key_size are supported
     */
    key_size_mask = (UINT16)(key_size_mask &
                    (((1 << (bz_auth_dev_params.max_enc_key_size))-1)
                        & ~((1 << (bz_auth_dev_params.min_enc_key_size-1))-1)));
    param_list[1]= (UCHAR) (key_size_mask&0xFF);
    param_list[2]=(UCHAR)(key_size_mask>>8);
    bz_auth_generate_pdu(ce_index, param_list,
            LMP_ENCRYPTION_KEY_SIZE_MASK_RES_LEN, tid);
}

/**
 * Decides the response for the received LMP_encryption_mode_req(disable) PDU
 * and sends it to the remote device.
 *
 * \param ce_index ACL connection entity index.
 * \param auth Authentication parameters associated with the \a ce_index.
 * \param lmp_pdu_ptr LMP PDU packet containing the LMP_encryption_mode_req
 *                    PDU received from the remote device.
 *
 * \return None.
 */
void bz_auth_send_response_to_disable_encryption_mode_req_pdu(UINT16 ce_index,
        BZ_AUTH_LINK_PARAMS* auth, LMP_PDU_PKT* lmp_pdu_ptr)
{
    LMP_TRAN_ID tid;

    tid = (LMP_TRAN_ID)LMP_GET_TRANSACTION_ID(lmp_pdu_ptr);
    bz_auth_send_accepted_pdu(ce_index, LMP_ENCRYPTION_MODE_REQ_OPCODE, tid);
    if (bz_auth_is_master(&lmp_connection_entity[ce_index]))
    {
        /* send lmp_stop_encryption pdu */
        bz_auth_send_stop_encryption_pdu(ce_index, auth, tid);
        bz_auth_transition_to_sub_state(auth, STOP_ENCRYPTION_SENT);
    }
    else
    {
        bz_auth_transition_to_sub_state(auth, AWAIT_STOP_ENCRYPTION);
    }
}

/**
 * Updates the authentication parameters from the transaction parameters
 * created during authentication procedures.
 *
 * \param auth Authentication parameters associated with the ACL link.
 *
 * \return None.
 */
void bz_auth_update_auth_params_from_txn_params(BZ_AUTH_LINK_PARAMS* auth)
{
    /* All link key creation procedures (eg. pairing, change_conn_link_key),
     * set the right link_key, lk_type in the txn_params on successful
     * completion.
     */
    memcpy(&auth->link_key[0], auth->txn_params.link_key,
            BZ_AUTH_LINK_KEY_SIZE);
    auth->lk_type = auth->txn_params.lk_type;
    /* All challenge/response procedures (eg. authentication, pairing, etc.)
     * set the proper ACO in the txn_params on successful completion.
     */
    memcpy(&auth->aco[0], auth->txn_params.aco, BZ_AUTH_ACO_SIZE);
}

/**
 * Handles the either successful or unsuccessful authentication completion
 * procedure. It takes necessary action based on the authentication related
 * procedure that was started.
 *
 * \param ce_index ACL connection entity index.
 * \param status Status of the authentication procedure. It must be a valid
 *               bluetooth error code.
 *
 * \return None.
 */
void bz_auth_handle_auth_completion(UINT16 ce_index, UCHAR status)
{
    
#ifdef _ROM_CODE_PATCHED_
    if (rcp_bz_auth_handle_auth_completion != NULL)
    {
        if (rcp_bz_auth_handle_auth_completion((void *)&ce_index, status))
        {
            return;
        }
        
    }
#endif

    BOOLEAN cb_flag = FALSE;        /* Don't call auth_complete_cb */
    LMP_CONNECTION_ENTITY* ce_ptr;
    BZ_AUTH_LINK_PARAMS* auth;
    UINT8 event = AUTH_STATUS_FAILED;

    ce_ptr = &lmp_connection_entity[ce_index];
    auth = ce_ptr->auth;

    if (auth->tid_timer != NULL)
    {
        OS_STOP_TIMER(auth->tid_timer, 0);
    }

    if (status == HCI_COMMAND_SUCCEEDED)
    {
        if (bz_auth_is_ssp_allowed(ce_ptr))
        {
            event = AUTH_STATUS_SUCCESS_WITH_SSP;
        }
        else
        {
            event = AUTH_STATUS_SUCCESS;
        }
    }

    bz_auth_transition_to_sub_state(auth, BZ_AUTH_SUB_STATE_IDLE);
    switch (auth->super_state)
    {
        case AUTHENTICATING_DURING_CONN:
            if (status == HCI_COMMAND_SUCCEEDED)
            {
                /* update the new link_key, lk_type, aco from txn_params */
                bz_auth_update_auth_params_from_txn_params(auth);
                /* Don't start encryption procedure, if the host acts weird by
                 * enabling only the encryption but not the authentication.
                 */
                if (bz_auth_dev_params.auth_enable
                        && bz_auth_dev_params.enc_enable)
                {
                    /* First, perform the normal auth_completion transition */
                    bz_auth_perform_super_state_transition(auth, event);

                    /* Encryption is enabled, so lets initiate encryption
                     * procedure automatically.
                     */
                    auth->txn_params.auth_role = BZ_AUTH_ROLE_INITIATOR;
                    bz_auth_send_encryption_mode_req_pdu(ce_index, auth,
                            SELF_DEV_TID, BZ_AUTH_ENCRYPTION_MODE_ON);

                    /* Now, perform the top level state transition for the
                     * automatic initiation of encryption procedure.
                     */
                    event =
                        HCI_SET_ENC_OR_ENC_REQ_OR_AUTH_SUCC_ENC_ENABLED;
                    bz_auth_perform_super_state_transition(auth, event);
                    return;
                }
                else
                {
                    /* Authentication during connection is almost complete and
                     * let the connection manager know that authentication
                     * completed.
                     */
                    /* we need to call the call back only for self initiated
                     * authentication.
                     */
                    if (auth->txn_params.auth_role != BZ_AUTH_ROLE_RESPONDER)
                    {
                        cb_flag = TRUE;
                    }
                } /* end if (enc_enabled) */
            }
            else
            {
                if (auth->txn_params.auth_role != BZ_AUTH_ROLE_RESPONDER)
                {
                    lmp_handle_acl_disconnect(ce_index, status);
                    return;
                }
            }
            break;

        case AUTHENTICATING:
        case AUTHENTICATED_UPGRADE_KEY:
            /* update the new link_key, lk_type, aco from txn_params */
            if (status == HCI_COMMAND_SUCCEEDED)
            {
                bz_auth_update_auth_params_from_txn_params(auth);
            }
            if (auth->txn_params.auth_role != BZ_AUTH_ROLE_RESPONDER)
            {
                /* send HCI Auth Completion event (status) */
                bz_auth_generate_authentication_complete_event(ce_index,
                        status);
            }
            break;

        case ENCRYPTED_UPGRADE_KEY:
            /* update the new link_key, lk_type, aco from txn_params */
            if (status == HCI_COMMAND_SUCCEEDED)
            {
                bz_auth_update_auth_params_from_txn_params(auth);
            }
            if (auth->txn_params.auth_role != BZ_AUTH_ROLE_RESPONDER)
            {
                /* send HCI Auth Completion event (status) */
                bz_auth_generate_authentication_complete_event(ce_index,
                        status);
            }
            if (status == HCI_COMMAND_SUCCEEDED
                    && bz_auth_is_epr_supported(ce_ptr))
            {
                /* The mutual authentication during upgrade key procedure has
                 * succeeded. Now, we need to initiate EPR both as initiator
                 * and as responder to avoid legacy-2.1 issue and subsequent
                 * security problem (Some devices don't do EPR as part of
                 * upgrade key procedure, we need to cope up with those
                 * devices as well).
                 */

                /* Terminate the previous transaction */
                bz_auth_perform_super_state_transition(auth, event);

                /* Start the refresh encryption procedure */
                auth->txn_params.auth_role = BZ_AUTH_ROLE_INITIATOR;
                bz_auth_perform_pause_encryption_action(ce_index, auth);
                bz_auth_perform_super_state_transition(auth,
                        HCI_REFRESH_ENCRYPTION_KEY);
                return;
            }
            break;

        case AUTHENTICATED_CHANGING_KEY:
            /* update the new link_key, lk_type, aco from txn_params */
            if (status == HCI_COMMAND_SUCCEEDED)
            {
                bz_auth_update_auth_params_from_txn_params(auth);
            }
            if (auth->txn_params.auth_role != BZ_AUTH_ROLE_RESPONDER)
            {
                /* send HCI Change Connection Link Key event (status) */
                bz_auth_generate_change_connection_link_key_event(ce_index,
                        status);
            }
            break;

        case ENCRYPTED_CHANGING_KEY:
            /* update the new link_key, lk_type, aco from txn_params */
            if (status == HCI_COMMAND_SUCCEEDED)
            {
                bz_auth_update_auth_params_from_txn_params(auth);
            }
            if (status != HCI_COMMAND_SUCCEEDED
                    || !bz_auth_is_epr_supported(ce_ptr))
            {
                if (auth->txn_params.auth_role != BZ_AUTH_ROLE_RESPONDER)
                {
                    /* send HCI Change Connection Link Key event (status) */
                    bz_auth_generate_change_connection_link_key_event(ce_index,
                            status);
                }
            }
            else
            {
                /* We support EPR and CCLK succeeded, so lets pause and resume
                 * encryption.
                 */
                if (auth->txn_params.auth_role == BZ_AUTH_ROLE_INITIATOR)
                {
                    bz_auth_perform_pause_encryption_action(ce_index, auth);
                }
                else
                {
                    bz_auth_transition_to_sub_state(auth,
                            AWAIT_PAUSE_ENCRYPTION);
                }
                return;
            }
            break;

#ifdef COMPILE_BROADCAST_ENCRYPTION
        case AUTHENTICATED_TEMP_AUTHENTICATING:
            /* update the new link_key, lk_type, aco from txn_params */
            if (status == HCI_COMMAND_SUCCEEDED)
            {
                /* take the backup of semi permanent key */
                memcpy(&auth->semi_bk_link_key[0], &auth->link_key[0],
                        BZ_AUTH_LINK_KEY_SIZE);
                auth->semi_bk_lk_type = auth->lk_type;
                memcpy(&auth->semi_bk_aco[0], &auth->aco[0], BZ_AUTH_ACO_SIZE);
                /* copy the new keys */
                bz_auth_update_auth_params_from_txn_params(auth);
            }
            bz_auth_perform_super_state_transition(auth, event);
            auth->txn_params.auth_role = BZ_AUTH_ROLE_INVALID;
            /* Notify top level state machine */
            bz_auth_handle_master_link_key_enable_completion(ce_index, status);
            return;

        case ENCRYPTED_TEMP_ENCRYPTING:
            /* update the new link_key, lk_type, aco from txn_params */
            if (status == HCI_COMMAND_SUCCEEDED)
            {
                /* take the backup of semi permanent key */
                memcpy(&auth->semi_bk_link_key[0], &auth->link_key[0],
                        BZ_AUTH_LINK_KEY_SIZE);
                auth->semi_bk_lk_type = auth->lk_type;
                memcpy(&auth->semi_bk_aco[0], &auth->aco[0], BZ_AUTH_ACO_SIZE);
                /* copy the new keys */
                bz_auth_update_auth_params_from_txn_params(auth);
                if (auth->txn_params.auth_role == BZ_AUTH_ROLE_INITIATOR)
                {
                    bz_auth_perform_stop_encryption_action(ce_index, auth);
                }
                else
                {
                    bz_auth_transition_to_sub_state(auth,
                            AWAIT_DISABLE_ENCRYPTION_MODE_REQ);
                }
                return;
            }
            bz_auth_perform_super_state_transition(auth, event);
            auth->txn_params.auth_role = BZ_AUTH_ROLE_INVALID;
            /* Notify top level state machine */
            bz_auth_handle_master_link_key_enable_completion(ce_index, status);
            return;
        case TEMP_AUTHENTICATED_DISABLING_TEMP_KEY:
            /* update the new link_key, lk_type, aco from txn_params */
            if (status == HCI_COMMAND_SUCCEEDED)
            {
                /* restore semi_per_key */
                memcpy(&auth->link_key[0], &auth->semi_bk_link_key[0],
                        BZ_AUTH_LINK_KEY_SIZE);
                auth->lk_type = auth->semi_bk_lk_type;
                memcpy(&auth->aco[0], &auth->semi_bk_aco[0], BZ_AUTH_ACO_SIZE);
            }
            bz_auth_perform_super_state_transition(auth, event);
            auth->txn_params.auth_role = BZ_AUTH_ROLE_INVALID;
            /* Notify top level state machine */
            bz_auth_handle_master_link_key_disable_completion(ce_index, status);
            return;
        case TEMP_ENCRYPTED_DISABLING_TEMP_KEY:
            /* update the new link_key, lk_type, aco from txn_params */
            if (status == HCI_COMMAND_SUCCEEDED)
            {
                /* restore semi_per_key */
                memcpy(&auth->link_key[0], &auth->semi_bk_link_key[0],
                        BZ_AUTH_LINK_KEY_SIZE);
                auth->lk_type = auth->semi_bk_lk_type;
                memcpy(&auth->aco[0], &auth->semi_bk_aco[0], BZ_AUTH_ACO_SIZE);

                if (auth->txn_params.auth_role == BZ_AUTH_ROLE_INITIATOR)
                {
                    bz_auth_perform_stop_encryption_action(ce_index, auth);
                }
                else
                {
                    bz_auth_transition_to_sub_state(auth,
                            AWAIT_DISABLE_ENCRYPTION_MODE_REQ);
                }
                return;
            }
            bz_auth_perform_super_state_transition(auth, event);
            auth->txn_params.auth_role = BZ_AUTH_ROLE_INVALID;
            /* Notify top level state machine */
            bz_auth_handle_master_link_key_disable_completion(ce_index, status);
            return;
        case TEMP_AUTHENTICATED_AUTHENTICATING:
        case TEMP_ENCRYPTED_AUTHENTICATING:
            if (auth->txn_params.auth_role != BZ_AUTH_ROLE_RESPONDER)
            {
                bz_auth_generate_authentication_complete_event(ce_index,
                        status);
            }
            bz_auth_perform_super_state_transition(auth, event);
            auth->txn_params.auth_role = BZ_AUTH_ROLE_INVALID;
            /* Notify top level state machine */
            bz_auth_handle_master_link_key_reauth_completion(ce_index, status);
            break;
#endif /* COMPILE_BROADCAST_ENCRYPTION */
        case AUTHENTICATED_DURING_CONN: /* only as responder we can get this */
        case ENCRYPTED_DURING_CONN:
            if (status == HCI_COMMAND_SUCCEEDED)
            {
                memcpy(auth->aco, auth->txn_params.aco, BZ_AUTH_ACO_SIZE);
            }
            break;

        default:
            break;
    }

    bz_auth_perform_super_state_transition(auth, event);
    auth->txn_params.auth_role = BZ_AUTH_ROLE_INVALID;

    if (cb_flag)
    {
        /* invoke the auth_completed_cb */
        if (bz_auth_dev_params.auth_completed_cb.cb != NULL)
        {
            bz_auth_dev_params.auth_completed_cb.cb(ce_index, status,
                    status, bz_auth_dev_params.auth_completed_cb.user_data);
        }
    }
#ifdef COMPILE_BROADCAST_ENCRYPTION
    bz_auth_initiate_mlk_procedure_if_pending(ce_index, status);
#endif
}

/**
 * Handles the either successful or unsuccessful enable encryption completion
 * procedure. It takes necessary action based on the procedure that triggered
 * the enable encryption.
 *
 * \param ce_index ACL connection entity index.
 * \param status Status of the enable encryption procedure. It must be a valid
 *               bluetooth error code.
 *
 * \return None.
 */
void bz_auth_handle_enable_enc_completion(UINT16 ce_index, UCHAR status)
{
    BOOLEAN resume_enc_cb_flag = FALSE;     /* Don't call resume_enc_cb */
    BOOLEAN cb_flag = FALSE;                /* Don't call auth_complete_cb */
    UINT8 auth_role;
    UINT8 enc_proc = BZ_AUTH_ENCRYPTION_PROCEDURE_LEGACY;
    LMP_CONNECTION_ENTITY* ce_ptr;
    BZ_AUTH_LINK_PARAMS* auth;
    UINT8 event = RESUMING_OR_START_ENC_STATUS_FAILED;

    ce_ptr = &lmp_connection_entity[ce_index];
    auth = ce_ptr->auth;
#ifdef _CHK_RESTART_ENC
RT_BT_LOG(BLUE, DAPE_TEST_LOG522, 2,status, auth->super_state);
#endif
    if (auth->tid_timer != NULL)
    {
        OS_STOP_TIMER(auth->tid_timer, 0);
    }

    if (status == HCI_COMMAND_SUCCEEDED)
    {
        event = RESUMING_OR_START_ENC_STATUS_SUCCESS;
        /* Update the auth params to reflect the current status */
#ifdef _SUPPORT_SECURE_CONNECTION_
        if (auth->secure_conn_enabled)
        {
            auth->enc_mode = BZ_AUTH_ENCRYPTION_MODE_ON_P256;
        }  
        else
#endif      
        {
            auth->enc_mode = BZ_AUTH_ENCRYPTION_MODE_ON;
        }
        auth->auto_epr_time_count = 0x0;
    }

    bz_auth_transition_to_sub_state(auth, BZ_AUTH_SUB_STATE_IDLE);
    switch (auth->super_state)
    {
        case ENCRYPTING_DURING_CONN:
            if (status == HCI_COMMAND_SUCCEEDED)
            {
                /* Authentication during connection is almost complete and
                 * let the connection manager know that authentication
                 * completed.
                 */
                /* we need to call the call back only for self initiated
                 * authentication.
                 */
                if (auth->txn_params.auth_role != BZ_AUTH_ROLE_RESPONDER)
                {
                    cb_flag = TRUE;
                }
            }
            else
            {
                if (auth->txn_params.auth_role != BZ_AUTH_ROLE_RESPONDER)
                {
                    /* Resume data transfer */
#ifdef COMPILE_NESTED_PAUSE_RESUME
					bz_auth_resume_data_transfer(ce_index, 
						ACL_PAUSED_ENCRYPTION);
#else /* COMPILE_NESTED_PAUSE_RESUME */
					bz_auth_resume_data_transfer(ce_index);
#endif /* COMPILE_NESTED_PAUSE_RESUME */

                    lmp_handle_acl_disconnect(ce_index, status);
                    return;
                }
            }
            break;
        case AUTHENTICATED_ENCRYPTING:
            bz_auth_generate_encryption_change_event(ce_index, status,
                    auth->enc_mode);
            break;
        case ENCRYPTED_CHANGING_KEY:
            /* Only if we support EPR, we would transition to this state.
             * Otherwise, CHANGING_KEY procedure would have completed just
             * after mutual_auth_completion.
             */
            bz_auth_generate_encryption_key_refresh_complete_event(
                        ce_index, status);
            if (auth->txn_params.auth_role != BZ_AUTH_ROLE_RESPONDER)
            {
                bz_auth_generate_change_connection_link_key_event(
                    ce_index, status);
            }
            break;
        case ENCRYPTED_REFRESH_KEY:
        case TEMP_ENCRYPTED_REFRESH_KEY:
            bz_auth_generate_encryption_key_refresh_complete_event(
                        ce_index, status);
            break;
        case ENCRYPTED_RESUMING_ENCRYPTION:
        case TEMP_ENCRYPTED_RESUMING_ENCRYPTION:
            if (bz_auth_is_epr_supported(ce_ptr))
            {
                bz_auth_generate_encryption_key_refresh_complete_event(
                        ce_index, status);
                enc_proc = BZ_AUTH_ENCRYPTION_PROCEDURE_EPR;
            }
            else
            {
                bz_auth_generate_encryption_change_event(ce_index, status,
                        auth->enc_mode);
            }
            resume_enc_cb_flag = TRUE;
            break;
#ifdef COMPILE_BROADCAST_ENCRYPTION
        case ENCRYPTED_TEMP_ENCRYPTING:
            bz_auth_perform_super_state_transition(auth, event);
            auth->txn_params.auth_role = BZ_AUTH_ROLE_INVALID;
            /* Notify master link key complete status */
            bz_auth_handle_master_link_key_enable_completion(ce_index, status);
            break;
        case TEMP_ENCRYPTED_DISABLING_TEMP_KEY:
            bz_auth_perform_super_state_transition(auth, event);
            auth->txn_params.auth_role = BZ_AUTH_ROLE_INVALID;
            /* Notify master link key complete status */
            bz_auth_handle_master_link_key_disable_completion(ce_index, status);
            break;

        case TEMP_AUTHENTICATED_ENCRIPTING:
            bz_auth_perform_super_state_transition(auth, event);
            auth->txn_params.auth_role = BZ_AUTH_ROLE_INVALID;
            bz_auth_generate_encryption_change_event(ce_index, status,
                    auth->enc_mode);
            /* Notify master link key complete status */
            bz_auth_handle_master_link_key_enable_enc_completion(ce_index,
                    status);
            break;
#endif /* COMPILE_BROADCAST_ENCRYPTION */
        default:
            break;
    }

    bz_auth_perform_super_state_transition(auth, event);
    auth_role = auth->txn_params.auth_role;
    auth->txn_params.auth_role = BZ_AUTH_ROLE_INVALID;
    
	/* resume data transfer */
#ifdef COMPILE_NESTED_PAUSE_RESUME
	bz_auth_resume_data_transfer(ce_index, ACL_PAUSED_ENCRYPTION);
#else /* COMPILE_NESTED_PAUSE_RESUME */
	bz_auth_resume_data_transfer(ce_index);
#endif /* COMPILE_NESTED_PAUSE_RESUME */

    if (resume_enc_cb_flag)
    {
        /* invoke the auth_resume_enc_cb */
        if (bz_auth_dev_params.auth_resume_enc_cb.cb != NULL)
        {
            bz_auth_dev_params.auth_resume_enc_cb.cb(ce_index, status,
                    auth_role, enc_proc,
                    bz_auth_dev_params.auth_resume_enc_cb.user_data);
        }
    }
    if (cb_flag)
    {
        /* invoke the auth_completed_cb */
        if (bz_auth_dev_params.auth_completed_cb.cb != NULL)
        {
            bz_auth_dev_params.auth_completed_cb.cb(ce_index, status,
                    status, bz_auth_dev_params.auth_completed_cb.user_data);
        }
    }
#ifdef COMPILE_BROADCAST_ENCRYPTION
    bz_auth_initiate_mlk_procedure_if_pending(ce_index, status);
#endif
}

/**
 * Handles the either successful or unsuccessful disable encryption completion
 * procedure. It takes necessary action based on the procedure that triggered
 * the disable encryption.
 *
 * \param ce_index ACL connection entity index.
 * \param status Status of the disable encryption procedure. It must be a valid
 *               bluetooth error code.
 *
 * \return None.
 */
void bz_auth_handle_disable_enc_completion(UINT16 ce_index, UCHAR status)
{
    BOOLEAN pause_enc_cb_flag = FALSE;        /* Don't call pause_enc_cb */
    UINT8 auth_role;
    UINT8 enc_proc = BZ_AUTH_ENCRYPTION_PROCEDURE_LEGACY;
    LMP_CONNECTION_ENTITY* ce_ptr;
    BZ_AUTH_LINK_PARAMS* auth;
    UINT8 event = PAUSE_OR_STOP_ENC_STATUS_FAILED;

    ce_ptr = &lmp_connection_entity[ce_index];
    auth = ce_ptr->auth;

    if (auth->tid_timer != NULL)
    {
        OS_STOP_TIMER(auth->tid_timer, 0);
    }

    if (status == HCI_COMMAND_SUCCEEDED)
    {
        event = PAUSE_OR_STOP_ENC_STATUS_SUCCESS;
        /* Update the auth params to reflect the current status */
        auth->enc_mode = BZ_AUTH_ENCRYPTION_MODE_OFF;
    }

    bz_auth_transition_to_sub_state(auth, BZ_AUTH_SUB_STATE_IDLE);
    switch (auth->super_state)
    {
        case ENCRYPTED_DISABLING:
            bz_auth_generate_encryption_change_event(ce_index, status,
                    auth->enc_mode);
            break;

        case ENCRYPTED_CHANGING_KEY:
            /* Only if we support EPR, we would transition to this state.
             * Otherwise, CHANGING_KEY procedure would have completed just
             * after mutual_auth_completion.
             */
            if (status != HCI_COMMAND_SUCCEEDED)
            {
                if (auth->txn_params.auth_role != BZ_AUTH_ROLE_RESPONDER)
                {
                    /* gen enc_key_refresh_complete_status (failed) */
                    bz_auth_generate_encryption_key_refresh_complete_event(
                            ce_index, status);
                    /* gen change_conn_link_key_complete_status (success) */
                    bz_auth_generate_change_connection_link_key_event(ce_index,
                            HCI_COMMAND_SUCCEEDED);
                }
                break;
            }
            /*@fallthrough@*/
        case ENCRYPTED_REFRESH_KEY:
        case TEMP_ENCRYPTED_REFRESH_KEY:
            if (status != HCI_COMMAND_SUCCEEDED)
            {
                if (auth->txn_params.auth_role != BZ_AUTH_ROLE_RESPONDER)
                {
                    /* gen enc_key_refresh_complete_status (failed) */
                    bz_auth_generate_encryption_key_refresh_complete_event(
                            ce_index, status);
                }
                break;
            }
            if (auth->txn_params.auth_role == BZ_AUTH_ROLE_INITIATOR)
            {
                /* 2.1 Pause encryption has completed successfully, let's
                 * initiate the Resume encryption :)
                 */
                bz_auth_perform_resume_encryption_action(ce_index, auth);
            }
            else
            {
                if (bz_auth_is_master(ce_ptr))
                {
                    bz_auth_transition_to_sub_state(auth,
                            AWAIT_RESUME_ENCRYPTION);
                }
                else
                {
                    bz_auth_transition_to_sub_state(auth,
                            AWAIT_START_ENCRYPTION);
                }
            }
            return;

        case ENCRYPTED_PAUSING_ENCRYPTION:
        case TEMP_ENCRYPTED_PAUSING_ENCRYPTION:
            /* We support EPR and genuine pause has happened, or as a
             * initiator somebody called bz_auth_pause_encryption().
             */
            if (!bz_auth_is_epr_supported(ce_ptr))
            {
                /* 2.0 Stop encryption procedure is being used and we should
                 * generate the enc_change_event irrespective of the status.
                 */
                bz_auth_generate_encryption_change_event(ce_index, status,
                        auth->enc_mode);
            }
            else
            {
                enc_proc = BZ_AUTH_ENCRYPTION_PROCEDURE_EPR;
                if (auth->txn_params.auth_role != BZ_AUTH_ROLE_RESPONDER
                        && status != HCI_COMMAND_SUCCEEDED)
                {
                    /* 2.1 Pause encryption has failed, intimate it to the
                     * host.
                     */
                    bz_auth_generate_encryption_key_refresh_complete_event(
                            ce_index, status);
                }
            }
            pause_enc_cb_flag = TRUE;
            break;
#ifdef COMPILE_BROADCAST_ENCRYPTION
        case TEMP_ENCRYPTED_DISABLING_ENCRYPTION:
            bz_auth_perform_super_state_transition(auth, event);
            auth->txn_params.auth_role = BZ_AUTH_ROLE_INVALID;
            bz_auth_generate_encryption_change_event(ce_index, status,
                    auth->enc_mode);
            bz_auth_handle_master_link_key_disable_enc_completion(ce_index,
                    status);
            break;

        case ENCRYPTED_TEMP_ENCRYPTING:
            if (status != HCI_COMMAND_SUCCEEDED)
            {
                /* Notify top level MLK module */
                bz_auth_handle_master_link_key_enable_completion(ce_index,
                    status);
                break;
            }
            if (auth->txn_params.auth_role == BZ_AUTH_ROLE_INITIATOR)
            {
                bz_auth_perform_start_encryption_action(ce_index, auth);
            }
            else
            {
                bz_auth_transition_to_sub_state(auth,
                        AWAIT_ENABLE_ENCRYPTION_MODE_REQ);
            }
            return;

        case TEMP_ENCRYPTED_DISABLING_TEMP_KEY:
            if (status != HCI_COMMAND_SUCCEEDED)
            {
                /* Notify top level MLK module */
                bz_auth_handle_master_link_key_disable_completion(ce_index,
                    status);
                break;
            }
            if (auth->txn_params.auth_role == BZ_AUTH_ROLE_INITIATOR)
            {
                bz_auth_perform_start_encryption_action(ce_index, auth);
            }
            else
            {
                bz_auth_transition_to_sub_state(auth,
                        AWAIT_ENABLE_ENCRYPTION_MODE_REQ);

            }
            return;
#endif /* COMPILE_BROADCAST_ENCRYPTION */

        default:
            BZ_ASSERT(0, "I don't expect disable_enc_completion at this state");
            return;
    }

    bz_auth_perform_super_state_transition(auth, event);
    auth_role = auth->txn_params.auth_role;
    auth->txn_params.auth_role = BZ_AUTH_ROLE_INVALID;
    if (auth->super_state != ENCRYPTED_PAUSED &&
            auth->super_state != TEMP_ENCRYPTED_PAUSED)
    {
        /* Resume data transfer */
#ifdef COMPILE_NESTED_PAUSE_RESUME
		bz_auth_resume_data_transfer(ce_index, ACL_PAUSED_ENCRYPTION);
#else /* COMPILE_NESTED_PAUSE_RESUME */
		bz_auth_resume_data_transfer(ce_index);
#endif /* COMPILE_NESTED_PAUSE_RESUME */
    }

    if (pause_enc_cb_flag)
    {
        /* invoke the auth_pause_enc_cb */
        if (bz_auth_dev_params.auth_pause_enc_cb.cb != NULL)
        {
            bz_auth_dev_params.auth_pause_enc_cb.cb(ce_index, status,
                    auth_role, enc_proc,
                    bz_auth_dev_params.auth_pause_enc_cb.user_data);
        }
    }
#ifdef COMPILE_BROADCAST_ENCRYPTION
    bz_auth_initiate_mlk_procedure_if_pending(ce_index, status);
#endif

	return;
}

/**
 * Returns whether the EPR is supported or not. It uses both the local and
 * remote device features to arrive at the decision.
 *
 * \param ce_ptr ACL connection entity pointer.
 *
 * \return TRUE, if the EPR procedure is supported. FALSE, otherwise.
 */
BOOLEAN bz_auth_is_epr_supported(LMP_CONNECTION_ENTITY* ce_ptr)
{
	/* Check for feature bits - local and remote */
	if (bz_auth_get_local_features(ce_ptr->phy_piconet_id)[5]
	   & bz_auth_get_remote_features(ce_ptr)[5]
	   & PAUSE_ENCRYPTION)
    {
        return TRUE;
    }
    return FALSE;
}

/**
 * Initiates the stop encryption action. It invokes the
 * pause_encryption_callback at the end of the transaction.
 *
 * \param ce_index ACL connection entity index.
 * \param auth Authentication parameters associated with the \a ce_index.
 *
 * \return None.
 */
void bz_auth_perform_stop_encryption_action(UINT16 ce_index,
        BZ_AUTH_LINK_PARAMS* auth)
{
    BZ_ASSERT(auth->txn_params.auth_role == BZ_AUTH_ROLE_INITIATOR,
            "Only initiator can call this function");

    /* send LMP_encryption_mode_req(disable, SELF_TID) */
    bz_auth_send_encryption_mode_req_pdu(ce_index, auth, SELF_DEV_TID,
            BZ_AUTH_ENCRYPTION_MODE_OFF);

    /* transition to appropriate sub_state: above call does that too */
}

/**
 * Initiates the start encryption action. It invokes the
 * resume_encryption_callback at the end of the transaction.
 *
 * \param ce_index ACL connection entity index.
 * \param auth Authentication parameters associated with the \a ce_index.
 *
 * \return None.
 */
void bz_auth_perform_start_encryption_action(UINT16 ce_index,
        BZ_AUTH_LINK_PARAMS* auth)
{
    BZ_ASSERT(auth->txn_params.auth_role == BZ_AUTH_ROLE_INITIATOR,
            "Only initiator can call this function");

    /* send LMP_encryption_mode_req(enable, SELF_TID) */
    bz_auth_send_encryption_mode_req_pdu(ce_index, auth, SELF_DEV_TID,
            BZ_AUTH_ENCRYPTION_MODE_ON);

    /* transition to appropriate sub_state: above call does that too */
}

/**
 * Returns the absolute transaction ID from the relative transaction ID. Both
 * SELF_DEV_TID and REMOTE_DEV_TID are called relative TIDs as they are based
 * on the current role of device in the ACL connection given by \a ce_index.
 * The absolute TID is nothing but the MASTER_TID and SLAVE_TID which are used
 * in LMP transactions.
 *
 * \param ce_index ACL connection entity index.
 * \param tid Input (relative) transaction ID.
 *
 * \return Absolute transaction ID equivalent to the input \a tid.
 */
UCHAR bz_auth_get_abs_tid(UINT16 ce_index, LMP_TRAN_ID tid)
{
    UCHAR abs_tid = 0;

    switch (tid)
    {
        case MASTER:
        case SLAVE:
            abs_tid = (UCHAR)tid;
            break;
        case SELF_DEV_TID:
            abs_tid = (UCHAR)
                (lmp_connection_entity[ce_index].remote_dev_role ^ 0x01);
            break;
        case REMOTE_DEV_TID:
            abs_tid = lmp_connection_entity[ce_index].remote_dev_role;
            break;

		default:
			break;
    }

    return abs_tid;
}

/**
 * Returns the TID from authentication role of the local device during any
 * transaction.
 *
 * \param ce_ptr ACL connection entity pointer.
 * \param auth Authentication parameters containing the authentication role.
 *
 * \return Absolute TID to be used during the transaction.
 *
 * \sa bz_auth_get_abs_tid.
 */
UCHAR bz_auth_get_tid_from_auth_role(LMP_CONNECTION_ENTITY* ce_ptr,
        BZ_AUTH_LINK_PARAMS* auth)
{
    /* defaulted to self tid as SLAVE(1) */
    UCHAR tid = SLAVE;

    if (bz_auth_is_master(ce_ptr))
    {
        /* if master then update self tid as MASTER(0) */
        tid = MASTER;
    }

    if (auth->txn_params.auth_role != BZ_AUTH_ROLE_INITIATOR)
    {
        /* if not self initiated transaction then active tid is remote tid */
        tid = (UCHAR)(~tid & 0x1);
    }

    return tid;
}

/**
 * Verifies whether the remote response matches the local device challenge and
 * generates the link key notification event, if required.
 *
 * \param ce_index ACL connection entity index.
 * \param auth Authentication parameters associated with the \a ce_index.
 * \param notify_link_key TRUE, if the link key notification event has to be
 *                        generated. FALSE, otherwise.
 *
 * \return None.
 */
void bz_auth_verify_challenge(UINT16 ce_index, BZ_AUTH_LINK_PARAMS* auth,
        BOOLEAN notify_link_key)
{
    if (bz_auth_is_sres_same(auth))
    {
        if (notify_link_key && !bz_auth_is_switching_to_temp_key(auth))
        {
            bz_auth_generate_link_key_notification_event(ce_index);
        }
        if (notify_link_key)   /* new link key created? */
        {
            LMP_CONNECTION_ENTITY* ce_ptr;

            ce_ptr = &lmp_connection_entity[ce_index];
            /* A new link key has been created and the link_key provided by
             * the host through write_stored_link_key is no more valid. So
             * lets delete it.
             * NOTE: The host might not have given link_key for this BD_ADDR,
             *       but anyway its safe to delete a non-existent key.
             */
            bz_auth_lkdb_del_key(bz_auth_get_remote_bd_addr(ce_ptr));
        }
        /* auth_status(success) */
        bz_auth_handle_auth_completion(ce_index, HCI_COMMAND_SUCCEEDED);
    }
    else
    {
        /* auth_status(failure) */
        bz_auth_handle_auth_completion(ce_index, AUTHENTICATION_FAILURE_ERROR);
    }
}

/**
 * Generate and send LMP_accepted for the given \a opcode. It also takes care
 * of (re)starting authentication TID timer.
 *
 * \param ce_index ACL connection entity index.
 * \param opcode LMP PDU opcode for which the LMP_accepted has to be sent.
 * \param tid Transaction ID to be used for sending the PDU.
 *
 * \return None.
 */
void bz_auth_send_accepted_pdu(UINT16 ce_index, UCHAR opcode, LMP_TRAN_ID tid)
{
	if (opcode == LMP_ENCRYPTION_MODE_REQ_OPCODE)
	{
#ifdef COMPILE_NESTED_PAUSE_RESUME
		bz_auth_pause_data_transfer(ce_index, ACL_PAUSED_ENCRYPTION);
#else /* COMPILE_NESTED_PAUSE_RESUME */
		bz_auth_pause_data_transfer(ce_index);
#endif /* COMPILE_NESTED_PAUSE_RESUME */
	}
	lmp_send_lmp_accepted(ce_index, opcode, (UCHAR)tid, LMP_NO_STATE_CHANGE);
	bz_auth_start_tid_timer(ce_index, LMP_AUTH_TID_TIMEOUT);

	return;
}

/**
 * Generate and send LMP_not_accepted for the given \a opcode. It also takes
 * care of (re)starting authentication TID timer.
 *
 * \param ce_index ACL connection entity index.
 * \param opcode LMP PDU opcode for which the LMP_accepted has to be sent.
 * \param tid Transaction ID to be used for sending the PDU.
 * \param reason Reason for LMP_not_accepted. It must be a valid bluetooth
 *               error code.
 *
 * \return None.
 */
void bz_auth_send_not_accepted_pdu(UINT16 ce_index, UCHAR opcode,
        LMP_TRAN_ID tid, UCHAR reason)
{
    lmp_send_lmp_not_accepted(ce_index, opcode, (UCHAR)tid, reason);
    if (lmp_connection_entity[ce_index].auth->sub_state
            != BZ_AUTH_SUB_STATE_IDLE)
    {
        bz_auth_start_tid_timer(ce_index, LMP_AUTH_TID_TIMEOUT);
    }

	return;
}

/**
 * Generate and send LMP_accepted_ext for the given \a opcode and \a
 * ext_opcode. It also takes care of (re)starting authentication TID timer.
 *
 * \param ce_index ACL connection entity index.
 * \param opcode LMP PDU opcode for which the LMP_accepted has to be sent.
 * \param ext_opcode Extended opcode of the LMP PDU.
 * \param tid Transaction ID to be used for sending the PDU.
 *
 * \return None.
 */
void bz_auth_send_accepted_ext_pdu(UINT16 ce_index, UCHAR opcode,
        UCHAR ext_opcode, LMP_TRAN_ID tid)
{
    lmp_send_lmp_accepted_ext(ce_index, opcode, ext_opcode, (UCHAR)tid);
    bz_auth_start_tid_timer(ce_index, LMP_AUTH_TID_TIMEOUT);
}

/**
 * Generate and send LMP_not_accepted_ext for the given \a opcode and \a
 * ext_opcode. It also takes care of (re)starting authentication TID timer.
 *
 * \param ce_index ACL connection entity index.
 * \param opcode LMP PDU opcode for which the LMP_accepted has to be sent.
 * \param ext_opcode Extended opcode of the LMP PDU.
 * \param tid Transaction ID to be used for sending the PDU.
 * \param reason Reason for LMP_not_accepted. It must be a valid bluetooth
 *               error code.
 *
 * \return None.
 */
void bz_auth_send_not_accepted_ext_pdu(UINT16 ce_index, UCHAR opcode,
        UCHAR ext_opcode, LMP_TRAN_ID tid, UCHAR reason)
{
    lmp_send_lmp_not_accepted_ext(ce_index, opcode, ext_opcode,
            (UCHAR)tid, reason);
    if (lmp_connection_entity[ce_index].auth->sub_state
            != BZ_AUTH_SUB_STATE_IDLE)
    {
        bz_auth_start_tid_timer(ce_index, LMP_AUTH_TID_TIMEOUT);
    }
}
#ifdef _SUPPORT_SECURE_CONNECTION_
//TIMER_ID en_ping_req_timer = OS_INVALID_HANDLE;

void bz_auth_handle_ping_req_pdu(LMP_PDU_PKT *lmp_pdu_ptr,
        UINT16 ce_index, OUT BOOLEAN* can_free_pdu)
{
    LMP_CONNECTION_ENTITY* ce_ptr;
    BZ_AUTH_LINK_PARAMS* auth;
    //UCHAR lmp_error_code = HCI_COMMAND_SUCCEEDED;
    LMP_TRAN_ID tid;

    *can_free_pdu = TRUE;
    ce_ptr = &lmp_connection_entity[ce_index];
    auth = ce_ptr->auth;
    tid = (LMP_TRAN_ID)LMP_GET_TRANSACTION_ID(lmp_pdu_ptr);
    //active_tid = bz_auth_get_tid_from_auth_role(ce_ptr, auth);
    if (auth->secure_conn_enabled)
    {
#ifdef _DAPE_TEST_SEND_PING_RES_BY_VENDOR_CMD    
        if (g_send_ping_response)
#endif 
        {
        bz_auth_send_ping_res_pdu(ce_index, tid);
    }
    }
    else
    {
         ///// to do what???
    }
    
}
void bz_auth_handle_ping_res_pdu(LMP_PDU_PKT *lmp_pdu_ptr,
        UINT16 ce_index, OUT BOOLEAN* can_free_pdu)
{
    *can_free_pdu = TRUE;
    ///// to do what???

    /* we do not stop timer. */
    //if(OS_is_timer_running(lmp_connection_entity[ce_index].en_ping_req_timer) == TRUE)
    //{
    //    OS_STOP_TIMER(lmp_connection_entity[ce_index].en_ping_req_timer, 0);
    //}

    ///////////////////////////////////////////////////////////    
    ///////////////////////////////////////////////////////////
    /* dape: If we have to send lmp_ping_req continuously, then
       we should start timer again. */
    ///////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////
}                         
void bz_auth_send_ping_req_pdu(UINT16 ce_index, LMP_TRAN_ID tid)
{
    UCHAR param_list[LMP_PING_REQ_LEN];
    param_list[0] = LMP_ESCAPE4_OPCODE;
    param_list[2] = LMP_PING_REQ_OPCODE;
    //bz_auth_generate_pdu(ce_index, param_list, 
    //        LMP_PING_REQ_LEN, tid);

    lmp_generate_pdu(ce_index, param_list, LMP_PING_REQ_LEN,
            tid, LMP_NO_STATE_CHANGE);

            
    lmp_connection_entity[ce_index].lmp_expected_pdu_opcode |= 
        lmp_get_opcode_mask(LMP_ESCAPE4_OPCODE, LMP_PING_RES_OPCODE);
    if (OS_IS_TIMER_RUNNING(lmp_connection_entity[ce_index].en_ping_req_timer))
    {
        OS_STOP_TIMER(lmp_connection_entity[ce_index].en_ping_req_timer,0);
    }
    OS_START_TIMER(lmp_connection_entity[ce_index].en_ping_req_timer,
               lmp_connection_entity[ce_index].max_auth_interval*5);   

}
void bz_auth_send_ping_res_pdu(UINT16 ce_index, LMP_TRAN_ID tid)
{
    UCHAR param_list[LMP_PING_RES_LEN];
    param_list[0] = LMP_ESCAPE4_OPCODE;
    param_list[2] = LMP_PING_RES_OPCODE;
    lmp_generate_pdu(ce_index, param_list, LMP_PING_RES_LEN,
            tid, LMP_NO_STATE_CHANGE);


   // bz_auth_generate_pdu(ce_index, param_list, 
   //         LMP_PING_RES_LEN, tid);
}
#ifdef SECURE_CONN_PING_EN
void bz_auth_send_max_auth_timeout_evt_timer_callback(TimerHandle_t timer_handle)
{    
    UINT16 ce_index = (UINT16)((UINT32)pvTimerGetTimerID(timer_handle));
    hci_generate_authentication_payload_timeout_expired_event(lmp_connection_entity[ce_index].connection_type.connection_handle);

    if (OS_IS_TIMER_RUNNING(lmp_connection_entity[ce_index].send_max_auth_timeout_timer))
    {
        OS_STOP_TIMER(lmp_connection_entity[ce_index].send_max_auth_timeout_timer,0);
    }
    OS_START_TIMER(lmp_connection_entity[ce_index].send_max_auth_timeout_timer,
                   lmp_connection_entity[ce_index].max_auth_interval*10);   

}

void bz_auth_start_ping_req_timer_callback(TimerHandle_t timer_handle)
{    
    UINT16 ce_index = (UINT16)((UINT32)pvTimerGetTimerID(timer_handle));
    bz_auth_send_ping_req_pdu(ce_index, SELF_DEV_TID);

}

void bz_auth_start_ping_req_timer_bottom_half(UINT16 ce_index)
{
    LMP_CONNECTION_ENTITY* ce_ptr = &lmp_connection_entity[ce_index];
    if (ce_ptr->en_ping_req_timer != NULL)
    {
        OS_DELETE_TIMER(&ce_ptr->en_ping_req_timer);
    }
    OS_CREATE_TIMER(ONESHOT_TIMER, &ce_ptr->en_ping_req_timer,
            bz_auth_start_ping_req_timer_callback, (void * ) ce_index, 0);
    OS_START_TIMER(ce_ptr->en_ping_req_timer, ce_ptr->max_auth_interval * 5);

    if (ce_ptr->send_max_auth_timeout_timer != NULL)
    {
        OS_DELETE_TIMER(&ce_ptr->send_max_auth_timeout_timer);
    }
    OS_CREATE_TIMER(ONESHOT_TIMER, &ce_ptr->send_max_auth_timeout_timer,
            bz_auth_send_max_auth_timeout_evt_timer_callback,
            (void * ) ce_index, 0);
    OS_START_TIMER(ce_ptr->send_max_auth_timeout_timer,
            ce_ptr->max_auth_interval * 10);
}

void bz_auth_start_ping_req_timer_bottom_half_task(void *no_arg,
        uint32_t ce_index)
{
    bz_auth_start_ping_req_timer_bottom_half((UINT16) ce_index);
}

void bz_auth_start_ping_req_timer(UINT16 ce_index)
{
    LMP_CONNECTION_ENTITY* ce_ptr;  
    ce_ptr = &lmp_connection_entity[ce_index];

#ifdef _ROM_CODE_PATCHED_
    if (rcp_bz_auth_start_ping_timer != NULL)
    {
        if (rcp_bz_auth_start_ping_timer((void *)&ce_index))
        {
            return;
        }
    }
#endif

    /* If remote device's ping feature is enabled, then start ping timer.*/
    if (ce_ptr->features[2][1] & SUPPORT_PING)
    {
        if (IN_ISR())
        {
            BaseType_t high_pri_task_woken = pdFALSE;
            xTimerPendFunctionCallFromISR(
                    bz_auth_start_ping_req_timer_bottom_half_task, NULL,
                    ce_index, &high_pri_task_woken);
            portYIELD_FROM_ISR(high_pri_task_woken);
        }
        else
        {
            bz_auth_start_ping_req_timer_bottom_half(ce_index);
        }

    }
}
void bz_auth_stop_ping_req_timer(UINT16 ce_index)
{
    LMP_CONNECTION_ENTITY* ce_ptr;  
    ce_ptr = &lmp_connection_entity[ce_index];

    if (ce_ptr->en_ping_req_timer != NULL)
    {        
        OS_STOP_TIMER(ce_ptr->en_ping_req_timer,0);
    }
    if (ce_ptr->send_max_auth_timeout_timer != NULL)
    {        
        OS_STOP_TIMER(ce_ptr->send_max_auth_timeout_timer,0);        
    }    
}
#endif
#endif

