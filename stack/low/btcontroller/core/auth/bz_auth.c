/***************************************************************************
 Copyright (C) Mindtree Ltd.
 This module is a confidential and proprietary property of MindTree and
 a possession or use of this module requires written permission of MindTree.
 ***************************************************************************/
/********************************* Logger *************************/
enum { __FILE_NUM__= 17 };
/********************************* Logger *************************/

/**
 * \file bz_auth.c
 *  BlueWiz Authentication module public interface implementation. It contains
 *  the implementation of all the functions which can be used by other
 *  modules.
 *
 * \author Santhosh kumar M
 * \date 2007-08-10
 */


/* ========================= Include File Section ========================= */
#include "bz_auth_states.h"
#include "lmp.h"
#include "bz_auth_hci.h"
#include "bz_auth_extern_accessors.h"
#include "bz_debug.h"
#include "bz_auth_lkdb.h"
#include "lc.h"
#include "bb_driver.h"
#include "crypto.h"
#include "bz_auth_internal.h"
#include "bz_auth_internal_2_1.h"
#include "bz_auth.h"
#ifdef COMPILE_BROADCAST_ENCRYPTION
#include "bz_auth_mlk.h"
#endif
#include "platform.h"

#include "mem.h"
#include "bt_secure_conn.h"
/* ====================== Macro Declaration Section ======================= */


/* ==================== Structure Declaration Section ===================== */


/* ===================== Variable Declaration Section ===================== */
/**
 * Permanent device level authentication parameters. These parameters are
 * initialized during the system power on and changed whenever required. They
 * are not reset during HCI_RESET.
 */
BZ_AUTH_PERMANENT_PARAMS bz_auth_permanent_params;
/**
 * Device level authentication parameters.
 */
BZ_AUTH_DEV_PARAMS bz_auth_dev_params;
/**
 * ACL Link level authentication paramters.
 */
#ifndef _ENABLE_RETENTION_FLOW_FOR_DLPS_ 
BZ_AUTH_LINK_PARAMS bz_auth_link_params[LMP_MAX_CE_DATABASE_ENTRIES];
#else
BZ_AUTH_LINK_PARAMS bz_auth_link_params_bton;
SECTION_LOW_BSS BZ_AUTH_LINK_PARAMS bz_auth_link_params_btoff[LMP_MAX_CE_DATABASE_ENTRIES - 1];
#endif

/**
 * SSP debug public key.
 */
BZ_STATIC const BZ_AUTH_PUBLIC_KEY ssp_debug_pubkey =
{
    {
        0x15, 0x20, 0x70, 0x09, 0x98, 0x44, 0x21, 0xa6, 0x58, 0x6f, 0x9f, 0xc3,
        0xfe, 0x7e, 0x43, 0x29, 0xd2, 0x80, 0x9e, 0xa5, 0x11, 0x25, 0xf8, 0xed
    },
    {
        0xb0, 0x9d, 0x42, 0xb8, 0x1b, 0xc5, 0xbd, 0x00, 0x9f, 0x79, 0xe4, 0xb5,
        0x9d, 0xbb, 0xaa, 0x85, 0x7f, 0xca, 0x85, 0x6f, 0xb9, 0xf7, 0xea, 0x25
    }
};

/**
 * SSP debug private key.
 */
BZ_STATIC const BZ_AUTH_PRIVATE_KEY ssp_debug_prikey =
{
    0x07, 0x91, 0x5f, 0x86, 0x91, 0x8d, 0xdc, 0x27, 0x00, 0x5d, 0xf1, 0xd6,
    0xcf, 0x0c, 0x14, 0x2b, 0x62, 0x5e, 0xd2, 0xef, 0xf4, 0xa5, 0x18, 0xff
};


#ifdef _CCH_SC_ECDH_P256_
/**
 * SSP debug public key.
 */
BZ_STATIC const BZ_AUTH_PUBLIC_KEY_P256 ssp_debug_pubkey_p256 =
{
    {
        0x20, 0xb0, 0x03, 0xd2, 0xf2, 0x97, 0xbe, 0x2c, 
        0x5e, 0x2c, 0x83, 0xa7, 0xe9, 0xf9, 0xa5, 0xb9, 
        0xef, 0xf4, 0x91, 0x11, 0xac, 0xf4, 0xfd, 0xdb, 
        0xcc, 0x03, 0x01, 0x48, 0x0e, 0x35, 0x9d, 0xe6
    },
    {
        0xdc, 0x80, 0x9c, 0x49, 0x65, 0x2a, 0xeb, 0x6d, 
        0x63, 0x32, 0x9a, 0xbf, 0x5a, 0x52, 0x15, 0x5c, 
        0x76, 0x63, 0x45, 0xc2, 0x8f, 0xed, 0x30, 0x24, 
        0x74, 0x1c, 0x8e, 0xd0, 0x15, 0x89, 0xd2, 0x8b
    }
};

/**
 * SSP debug private key.
 */
BZ_STATIC const BZ_AUTH_PRIVATE_KEY_P256 ssp_debug_prikey_p256 =
{
        0x3f, 0x49, 0xf6, 0xd4, 0xa3, 0xc5, 0x5f, 0x38, 
        0x74, 0xc9, 0xb3, 0xe3, 0xd2, 0x10, 0x3f, 0x50, 
        0x4a, 0xff, 0x60, 0x7b, 0xeb, 0x40, 0xb7, 0x99, 
        0x58, 0x99, 0xb8, 0xa6, 0xcd, 0x3c, 0x1a, 0xbd
};
#endif

#ifdef _SSP_DHKEY_CALCULATE_NEW_FLOW_ 
UINT8 g_ssp_ce_index = 0;
#endif

/* ================== Static Function Prototypes Section ================== */


/* ===================== Function Definition Section ====================== */
/**
 * Initialize the authentication module. This should be invoked only once
 * during the system power on. It initializes the #bz_auth_permanent_params.
 *
 * \param None.
 *
 * \return None.
 */
void bz_auth_init(void)
{
#ifdef SSP_DHKEY_CALCULATE_PARTITION_
    memset(&mpal_manager, 0, sizeof(mpal_manager));
#endif

#ifndef _SPEED_UP_AUTH_INIT_
    ssp_rng_init(pf_get_rng_seed());
#else
    ssp_rng_init(0xc4000003);
#endif

    ssp_get_ecdh_keypair(&bz_auth_local_calc_prikey[0],
                         &bz_auth_local_calc_pubkey);

    /* Initialize ssp debug mode keys */
    memcpy(&bz_auth_local_debug_pubkey, &ssp_debug_pubkey,
           (2*BZ_AUTH_PUBLIC_KEY_CO_ORDINATE_SIZE));
    memcpy(&bz_auth_local_debug_prikey[0], &ssp_debug_prikey[0],
           BZ_AUTH_PRIVATE_KEY_SIZE);

#ifdef _CCH_SC_ECDH_P256_	

    ssp_get_ecdh_keypair_p256(&bz_auth_local_calc_prikey_p256[0],
                         &bz_auth_local_calc_pubkey_p256);

    /* Initialize ssp debug mode keys */
    memcpy(&bz_auth_local_debug_pubkey_p256, &ssp_debug_pubkey_p256,
           (2*BZ_AUTH_PUBLIC_KEY_CO_ORDINATE_SIZE_P256));
    memcpy(&bz_auth_local_debug_prikey_p256[0], &ssp_debug_prikey_p256[0],
           BZ_AUTH_PRIVATE_KEY_SIZE_P256);
#endif

	
}

/**
 * Reset the authentication module.
 *
 * \param None.
 *
 * \return None.
 */
void bz_auth_reset(void)
{
    /* Initialize to default values */
    memset(&bz_auth_dev_params, 0x0, sizeof(bz_auth_dev_params));

    bz_auth_dev_params.max_enc_key_size = BZ_AUTH_MAX_ENC_KEY_SIZE;
    bz_auth_dev_params.min_enc_key_size = BZ_AUTH_MIN_ENC_KEY_SIZE;
    bz_auth_dev_params.pin_type = BZ_AUTH_PIN_TYPE_VARIABLE;

#if 1
    UINT16 ce_index;
    BZ_AUTH_LINK_PARAMS* auth;

    for (ce_index = 0; ce_index < LMP_MAX_CE_DATABASE_ENTRIES; ce_index++)
    {
        auth = lmp_connection_entity[ce_index].auth;
        memset(auth, 0x0, sizeof(BZ_AUTH_LINK_PARAMS));

        auth->tid_timer = NULL;
#ifdef _INI_SECURE_CONN_

#ifdef _CCH_SC_ECDH_P256_PAUSE_ENC		
        auth->sc_use_enc_rand = 0;		
#endif
#ifdef _CCH_SC_TEST_20130201_ESCO_INI_FLAG
        auth->not_first_esco = 0;
#endif 
#ifndef _INI_SECURE_CONN_8_
        auth->secure_conn_enabled = 0;
        auth->len_prime = 6;
#else
        auth->secure_conn_enabled = 1;
        auth->len_prime = 8;
#endif	
#endif
    }
#else
    UINT32 i;

    memset(bz_auth_link_params, 0x0, sizeof(bz_auth_link_params));
    for (i = 0; i < LMP_MAX_CE_DATABASE_ENTRIES; i++)
    {
        bz_auth_link_params[i].tid_timer = OS_INVALID_HANDLE;
#ifdef _INI_SECURE_CONN_

#ifdef _CCH_SC_ECDH_P256_PAUSE_ENC		
        bz_auth_link_params[i].sc_use_enc_rand = 0;		
#endif
#ifdef _CCH_SC_TEST_20130201_ESCO_INI_FLAG
        bz_auth_link_params[i].not_first_esco = 0;
#endif 
#ifndef _INI_SECURE_CONN_8_
        bz_auth_link_params[i].secure_conn_enabled = 0;
        bz_auth_link_params[i].len_prime = 6;
#else
        bz_auth_link_params[i].secure_conn_enabled = 1;
        bz_auth_link_params[i].len_prime = 8;
#endif	
#endif
    }
#endif
    bz_auth_lkdb_init();    
    bz_auth_reset_ssp_ext_feature_bits();
#ifdef _SUPPORT_SECURE_CONNECTION_
    bz_auth_reset_sc_ext_feature_bits();
#endif
}

/**
 * Initialize authentication paramaters of the ACL link specified by \a
 * ce_index.
 *
 * \param ce_index ACL connection entity index.
 *
 * \return None.
 */
void bz_auth_init_linkparams(UINT16 ce_index)
{
    BZ_AUTH_LINK_PARAMS* auth;

#ifndef _ENABLE_RETENTION_FLOW_FOR_DLPS_ 
    auth = &bz_auth_link_params[ce_index];
#else
    if(ce_index == 0)
    {
        auth = &bz_auth_link_params_bton;
    }
    else
    {
        auth = &bz_auth_link_params_btoff[ce_index-1];
    }	

    memset(auth, 0x0, sizeof(BZ_AUTH_LINK_PARAMS)); 	
#endif
		
    lmp_connection_entity[ce_index].auth = auth;
    auth->tid_timer = NULL;
    auth->auto_epr_time_count = 0x0;

    /* Check whether the link is disconnected during temp in use process */
    switch (auth->super_state)
    {
#ifdef COMPILE_BROADCAST_ENCRYPTION
        case AUTHENTICATED_TEMP_AUTHENTICATING:
        case ENCRYPTED_TEMP_ENCRYPTING:
            auth->super_state = UNAUTHENTICATED_DURING_CONN;
            auth->sub_state = BZ_AUTH_SUB_STATE_IDLE;
            bz_auth_handle_mlk_link_disconnected(ce_index,
                                                 MASTER_LINK_KEY_UPDATING);
            break;
        case TEMP_AUTHENTICATED_DISABLING_TEMP_KEY:
        case TEMP_ENCRYPTED_DISABLING_TEMP_KEY:
            auth->super_state = UNAUTHENTICATED_DURING_CONN;
            auth->sub_state = BZ_AUTH_SUB_STATE_IDLE;
            bz_auth_handle_mlk_link_disconnected(ce_index,
                                                 MASTER_LINK_DISABLING);
            break;
        case TEMP_AUTHENTICATED:
        case TEMP_ENCRYPTED:
        case TEMP_AUTHENTICATED_ENCRIPTING:
        case TEMP_AUTHENTICATED_AUTHENTICATING:
        case TEMP_ENCRYPTED_DISABLING_ENCRYPTION:
        case TEMP_ENCRYPTED_REFRESH_KEY:
        case TEMP_ENCRYPTED_PAUSING_ENCRYPTION:
        case TEMP_ENCRYPTED_PAUSED:
        case TEMP_ENCRYPTED_RESUMING_ENCRYPTION:
        case TEMP_ENCRYPTED_AUTHENTICATING:
            auth->super_state = UNAUTHENTICATED_DURING_CONN;
            auth->sub_state = BZ_AUTH_SUB_STATE_IDLE;
            bz_auth_handle_mlk_link_disconnected(ce_index,
                                                 MASTER_LINK_KEY_IN_USE);
            break;
#endif /* COMPILE_BROADCAST_ENCRYPTION */
        default:
            auth->super_state = UNAUTHENTICATED_DURING_CONN;
            auth->sub_state = BZ_AUTH_SUB_STATE_IDLE;
            break;
    }
    /* Although the controller communicates the link key type to the host
     * through Link_Key_Notification_Event, the host doesn't provide this
     * information back to the Controller when using Link_Key_Request_Reply
     * command. So we may have to assume that the host given key is
     * Combination_Key (just my preference ;)). This would still be a problem
     * in case of Change_Connection_Link_Key procedure initiated either by the
     * local device or by the remote device as it requires the current
     * link_key_type to determine whether to initiate Pairing Procedure (in
     * case of Unit_key) or Change_Connection_Link_Key procedure (in case of
     * Comb_key). If the host given key is Unit_Key, then the result would be
     * wrong. It will not be problem, if the controller did undergo pairing
     * procedure to create the link key. In this case, we know the link key
     * type ourselves.
     */
    auth->lk_type = BZ_AUTH_LINK_KEY_TYPE_COMBINATION_KEY;
    if (auth->pending_pdu)
    {
        bz_auth_free_pending_pdu(auth);
    }
    auth->txn_params.auth_role = BZ_AUTH_ROLE_INVALID;
    auth->enc_mode = BZ_AUTH_ENCRYPTION_MODE_OFF;
    auth->key_size_mask = 0xFFFF;
    auth->enc_key_size = bz_auth_dev_params.max_enc_key_size;
}

/**
 * Creates the authentication related timers for the ACL link given
 * by \a ce_index.
 *
 * \param ce_index ACL connection entity index.
 *
 * \return None.
 */
void bz_auth_create_auth_timers(UINT16 ce_index)
{
    BZ_AUTH_LINK_PARAMS* auth;

    auth = lmp_connection_entity[ce_index].auth;
    if (auth->tid_timer != NULL)
    {
        OS_DELETE_TIMER(&auth->tid_timer);
    }
    /* Create a tid timer */
    if (OS_CREATE_TIMER(ONESHOT_TIMER, &auth->tid_timer,
            bz_auth_tid_timer_handler, (void *)((UINT32)ce_index), 0) != BT_ERROR_OK)
    {
        LMP_LOG_INFO(LOG_LEVEL_HIGH, AUTH_TID_TIMER_CREATION_FAILED, 0, 0);
    }
}

/**
 * Resets the authentication related timers for the ACL link given
 * by \a ce_index.
 *
 * \param ce_index ACL connection entity index.
 *
 * \return None.
 */
void bz_auth_reset_auth_timers(UINT16 ce_index)
{
    BZ_AUTH_LINK_PARAMS* auth;

    auth = lmp_connection_entity[ce_index].auth;

    if (auth->tid_timer != NULL)
    {
        OS_DELETE_TIMER(&auth->tid_timer);
    }
}


/**
 * Returns the current encryption mode of the connection specified by \a
 * ce_index.
 *
 * \param ce_index ACL connection entity index.
 *
 * \return Current encryption mode.
 */
UINT8 bz_auth_get_encryption_mode(UINT16 ce_index)
{
    return lmp_connection_entity[ce_index].auth->enc_mode;
}

#ifdef VER_3_0
/**
 * Returns the current encryption keysize of the connection specified by \a
 * ce_index.
 *
 * \param ce_index ACL connection entity index.
 *
 * \return Current encryption keysize.
 */
UCHAR bz_auth_get_encryption_keysize(UINT16 ce_index)
{
    return lmp_connection_entity[ce_index].auth->enc_key_size;
}
#endif

/**
 * Registers pause encryption callback. The callback function \a cb will be
 * invoked whenever the encryption is paused (works both for self-initiated
 * and remote-initiated transactions).
 *
 * \param cb Callback function to be registered.
 * \param user_data User data to be passed to the \a cb when it is invoked.
 *
 * \return TRUE, if the operation is successful. FALSE, otherwise.
 *
 * \note If the pause/stop encryption is self-initiated, then the callback
 *       will be invoked regardless of the encryption procedure used
 *       (EPR/LEGACY). But if the transaction is remote-initiated, the
 *       callback is invoked if and only if the EPR procedure is used (It is
 *       technically impossible to know if the procedure is stop/pause if the
 *       LEGACY encryption procedure is used).
 */
BOOLEAN bz_auth_register_pause_encryption_callback(BZ_AUTH_PAUSE_ENC_CB cb,
        void* user_data)
{
    bz_auth_dev_params.auth_pause_enc_cb.cb = cb;
    bz_auth_dev_params.auth_pause_enc_cb.user_data = user_data;
    return TRUE;
}

/**
 * Registers resume encryption callback. The callback function \a cb will be
 * invoked whenever the encryption is resumed (works both for self-initiated
 * and remote-initiated transactions).
 *
 * \param cb Callback function to be registered.
 * \param user_data User data to be passed to the \a cb when it is invoked.
 *
 * \return TRUE, if the operation is successful. FALSE, otherwise.
 *
 * \note If the resume/start encryption is self-initiated, then the callback
 *       will be invoked regardless of the encryption procedure used
 *       (EPR/LEGACY). But if the transaction is remote-initiated, the
 *       callback is invoked if and only if the EPR procedure is used (It is
 *       technically impossible to know if the procedure is start/resume if the
 *       LEGACY encryption procedure is used).
 */
BOOLEAN bz_auth_register_resume_encryption_callback(BZ_AUTH_RESUME_ENC_CB cb,
        void* user_data)
{
    bz_auth_dev_params.auth_resume_enc_cb.cb = cb;
    bz_auth_dev_params.auth_resume_enc_cb.user_data = user_data;
    return TRUE;
}

/**
 * Registers authentication completed callback. The callback function \a cb
 * will be invoked whenever authentication procedure is completed during
 * connection and the self device is initiator of the authentication
 * transaction.
 *
 * \param cb Callback function to be registered..
 * \param user_data User data to be passed to the \a cb when it is invoked.
 *
 * \return TRUE, if the operation is successful. FALSE, otherwise.
 */
BOOLEAN bz_auth_register_auth_completed_callback(BZ_AUTH_COMPLETED_CB cb,
        void* user_data)
{
    bz_auth_dev_params.auth_completed_cb.cb = cb;
    bz_auth_dev_params.auth_completed_cb.user_data = user_data;
    return TRUE;
}

/**
 * Sends encryption key size mask pdu if required.
 *
 * \param ce_index  ACL connection entity index.
 *
 * \return None
 */
void bz_auth_decide_to_send_encryption_key_size_mask_req(UINT16 ce_index)
{
    LMP_CONNECTION_ENTITY* ce_ptr;
    ce_ptr = &lmp_connection_entity[ce_index];
    if (bz_auth_is_master(ce_ptr) && 
        (bz_auth_get_remote_features(ce_ptr)[2] & LMP_BC_ENCRYPTION_FEATURE) != 0)
    {
        bz_auth_send_encryption_key_size_mask_req_pdu(ce_index);
    }
}

/**
 * Authentication related connection complete hook. This function has to be
 * invoked by the connection manager whenever a new ACL connection is created
 * and Connection_Complete_Event (for ACL) is about to be sent.
 *
 * \param ce_index ACL Connection entity index of the new connection.
 * \param hci_error_code Error code of the connection completion (eg.
 *                       HCI_COMMAND_SUCCEED).
 *
 * \return None.
 */
void bz_auth_handle_acl_connection_complete(UINT16 ce_index,
        UCHAR hci_error_code)
{
    LMP_CONNECTION_ENTITY* ce_ptr;
    BZ_AUTH_LINK_PARAMS* auth;

    ce_ptr = &lmp_connection_entity[ce_index];
    auth = ce_ptr->auth;
    if (hci_error_code != HCI_COMMAND_SUCCEEDED)
    {
        return;
    }

    switch (auth->super_state)
    {
        case UNAUTHENTICATED_DURING_CONN:
        case AUTHENTICATED_DURING_CONN:
        case ENCRYPTED_DURING_CONN:
            break;
        default:
            BZ_ASSERT(0, "Something wrong with the conn_manager interaction. "
                      "While we are in the process of authentication, "
                      "conn_manager is generating conn_complete");
            return;
    }
    bz_auth_perform_super_state_transition(auth, CONNECTION_COMPLETE_SUCCESS);
    bz_auth_decide_to_send_encryption_key_size_mask_req(ce_index);
}

/**
 * Initiates authentication during connection, if required or possible. If it
 * doesn't initiate authentication (return value FALSE), the caller can
 * proceed with completing the ACL connection establishment procedure.
 * Otherwise, the caller shall wait until the authentication procedure
 * completion (and it will be notified using the registered callback - refer
 * bz_auth_register_auth_completed_callback).
 *
 * \param ce_index ACL connection entity index.
 *
 * \return TRUE, if the authentication procedure was initiated. FALSE,
 *         otherwise.
 */
BOOLEAN bz_auth_initiate_authentication_during_connection(UINT16 ce_index)
{
    BZ_AUTH_LINK_PARAMS* auth;
    BOOLEAN ret_val = FALSE;
    auth = lmp_connection_entity[ce_index].auth;
#ifdef _FAKE_P256
    auth->len_prime = 6;
#endif
#ifdef _FAKE_SECURE_CONN_
    auth->secure_conn_enabled = 1;
    auth->len_prime = 8;
#endif

#ifdef _SSP_DHKEY_CALCULATE_NEW_FLOW_
    auth->ce_index = ce_index;
#endif

    /* no authentication procedures shall be allowed when both devices support
     * SSP.
     */
    if (bz_auth_is_ssp_allowed(&lmp_connection_entity[ce_index]))
    {
        return FALSE;
    }
    switch (auth->super_state)
    {
        case UNAUTHENTICATED_DURING_CONN:
            if (bz_auth_dev_params.auth_enable)
            {
                /* we have to initiate the authentication procedure
                 * proactively... so sad :(
                 */
                auth->txn_params.auth_role = BZ_AUTH_ROLE_INITIATOR;
                bz_auth_perform_super_state_transition(auth,
                                                       HCI_AUTH_REQ_OR_AURAND_INRAND_OR_IOCAP_REQ_AUTH_ENABLE);
                ret_val = TRUE;
                bz_auth_enter_link_key_requested_state(ce_index, auth,
                                                       INTR_LINK_KEY_REQUESTED);
            }
            break;

        case AUTHENTICATING_DURING_CONN:
        case ENCRYPTING_DURING_CONN:
            /* The remote device has initiated the authentication procedures
             * and while we are in the middle of processing an authentication
             * transaction, there is a request from our side to check whether
             * we can initiate authentication or not. Since we are already in
             * the middle of authentication procedure, just make a note so
             * that we can intimate our device upon completing the
             * authentication procedure.
             */
            auth->txn_params.auth_role = BZ_AUTH_ROLE_INITIATOR_AS_RESPONDER;
            ret_val = TRUE;
            break;

        case AUTHENTICATED_DURING_CONN:
        case ENCRYPTED_DURING_CONN:
            /* Remote device has already performed the authentication, we
             * don't need to anything :)
             */
            break;

        default:
            BZ_ASSERT(0, "don't know what to do");
    }

    return ret_val;
}

#ifdef COMPILE_BROADCAST_ENCRYPTION
/**
 * Enables broadcast encryption. The next outgoing broadcast packet
 * after this call will be encrypted.
 *
 * \return None.
 */
void bz_auth_enable_broadcast_encryption(UCHAR* key_dash)
{
    BB_write_encryption_keys(BC_AM_ADDR, SCA_PICONET_FIRST, key_dash);

    /* Though piconet_id is only 1-bit, this is a special case, as slave. */
    BB_encryption_control(BC_AM_ADDR,
                          0x03, BB_ENC_TX_ENABLE_RX_ENABLE);

    return;
}

/**
 * Disables broadcast encryption. The next outgoing broadcast
 * packet after this call will be unencrypted.
 *
 * \return None.
 */
void bz_auth_disable_broadcast_encryption(void)
{
    BB_encryption_control(BC_AM_ADDR,
                          0x3, BB_ENC_TX_DISBALE_RX_DISABLE);

    return;
}
#endif

/**
 * Enables link level encryption. The next outgoing packet after this call
 * will be encrypted.
 *
 * \param ce_index Connection entity index of the ACL link to be encrypted.
 * \param affect_bb TRUE, if the encryption has to be programmed to BB. False,
 *                  otherise.
 *
 * \return None.
 */
void bz_auth_enable_link_level_encryption(UINT16 ce_index, BOOLEAN affect_bb)
{
    LMP_CONNECTION_ENTITY* ce_ptr;

    ce_ptr = &lmp_connection_entity[ce_index];


#ifdef _CCH_SC_ECDH_P256_
    BZ_AUTH_LINK_PARAMS* auth;
    auth = lmp_connection_entity[ce_index].auth;
	
    if(auth->secure_conn_enabled)
    {
        BB_write_sc_encryption_keys(ce_index,
                             ce_ptr->auth->enc_key);

#ifdef _CCH_SC_ECDH_P256_PAUSE_ENC
        if(auth->sc_use_enc_rand)
        {
#ifdef _CCH_SC_ECDH_P256_LOG    
            RT_BT_LOG(BLUE, CCH_DBG_155, 0, 0);
#endif


#ifdef _CCH_SC_ECDH_P256_LOG
            RT_BT_LOG(BLUE, YL_DBG_HEX_4, 4, auth->enc_rand[0], auth->enc_rand[1], auth->enc_rand[2], auth->enc_rand[3]);
            RT_BT_LOG(BLUE, YL_DBG_HEX_4, 4, auth->enc_rand_remote[0], auth->enc_rand_remote[1], auth->enc_rand_remote[2], auth->enc_rand_remote[3]);

#endif

#ifdef _CCH_SC_TEST_20130129_QCA_02
            UCHAR temp[BZ_AUTH_ENC_RAND_SIZE];
            memcpy(&temp[0], &auth->enc_rand[0], BZ_AUTH_ENC_RAND_SIZE);
            bz_auth_convert_to_msb(&temp[0], 16);	
            BB_write_sc_iv(ce_index, &temp[0]);					
#else


#ifdef _CCH_SC_TEST_20130129_QCA_04

#ifdef _CCH_SC_TEST_20130129_QCA_05

#ifdef _DAPE_TEST_CHK_SC_ROLE_SW
RT_BT_LOG(RED, DAPE_TEST_LOG293, 1,ce_ptr->remote_dev_role);
#endif
            if(ce_ptr->remote_dev_role == MASTER )
            {
                BB_write_sc_iv(ce_index, &auth->enc_rand_remote[8]);
			}else
			{
                BB_write_sc_iv(ce_index, &auth->enc_rand[8]);
			}

#else
            BB_write_sc_iv(ce_index, &auth->enc_rand[8]);
#endif

#else
            BB_write_sc_iv(ce_index, &auth->enc_rand[0]);
#endif

#endif
        }
#endif

		
    }
	else
#endif
    {
    BB_write_encryption_keys(ce_ptr->am_addr, ce_ptr->phy_piconet_id,
                             ce_ptr->auth->enc_key);
    }

    if (affect_bb)
    {
        BB_encryption_control(ce_ptr->am_addr,
                              ce_ptr->phy_piconet_id, BB_ENC_TX_ENABLE_RX_ENABLE);
    }

#ifdef COMPILE_BROADCAST_ENCRYPTION
    if (bz_auth_is_slave(ce_ptr) &&
            bz_auth_is_to_use_temp_key_for_encryption(ce_ptr->auth))
    {
        bz_auth_enable_broadcast_encryption(ce_ptr->auth->enc_key);
    }
#endif

    return;
}

/**
 * Disables link level encryption. The next outgoing packet after this call
 * will be unencrypted.
 *
 * \param ce_index Connection entity index of the ACL link to be unencrypted.
 * \param affect_bb TRUE, if the encryption has to be programmed to BB. False,
 *                  otherise.
 *
 * \return None.
 */
void bz_auth_disable_link_level_encryption(UINT16 ce_index, BOOLEAN affect_bb)
{
    LMP_CONNECTION_ENTITY* ce_ptr;

    ce_ptr = &lmp_connection_entity[ce_index];
    if (affect_bb)
    {
        BB_encryption_control(ce_ptr->am_addr,
                              ce_ptr->phy_piconet_id, BB_ENC_TX_DISBALE_RX_DISABLE);
    }
#ifdef COMPILE_BROADCAST_ENCRYPTION
    if (bz_auth_is_slave(ce_ptr) &&
            bz_auth_is_master_link_key_in_use(ce_index))
    {
        bz_auth_disable_broadcast_encryption();
    }
#endif
}

/**
 * Initiate pause encryption procedure.
 *
 * \param ce_index ACL connection entity index.
 * \param proc The internal procedure initiated to disable encryption on this
 *             link.
 *
 * \return TRUE, if the pause encryption procedure was initiated. FALSE,
 *         otherwise.
 */
BOOLEAN bz_auth_pause_encryption(UINT16 ce_index)
{
    BZ_AUTH_LINK_PARAMS* auth;
    LMP_CONNECTION_ENTITY* ce_ptr;

    ce_ptr = &lmp_connection_entity[ce_index];
    auth = ce_ptr->auth;

    switch (auth->super_state)
    {
        case ENCRYPTED:
        case TEMP_ENCRYPTED:
            auth->txn_params.auth_role = BZ_AUTH_ROLE_INITIATOR;
            if (bz_auth_is_epr_supported(ce_ptr))
            {
                bz_auth_perform_pause_encryption_action(ce_index, auth);
            }
            else
            {
                bz_auth_perform_stop_encryption_action(ce_index, auth);
            }
            break;

        default:
            /* Probably, either the authentication module is not idle or we
             * are not encrypted at all.
             */
            LMP_LOG_INFO(LOG_LEVEL_HIGH, 
               PAUSE_ENCRYPTION_ERROR_LINK_IS_NOT_ENCRYPTED_OR_NOT_IDLE, 0, 0);

            return FALSE;
    }

    bz_auth_perform_super_state_transition(auth, PAUSE_ENCRYPTION_REQ);

    return TRUE;
}

/**
 * Initiate resume encryption procedure.
 *
 * \param ce_index ACL connection entity index.
 * \param proc The internal procedure initiated to enable encryption on this
 *             link.
 *
 * \return TRUE, if the resume encryption procedure was initiated. FALSE,
 *         otherwise.
 */
BOOLEAN bz_auth_resume_encryption(UINT16 ce_index)
{
    BZ_AUTH_LINK_PARAMS* auth;
    LMP_CONNECTION_ENTITY* ce_ptr;

    ce_ptr = &lmp_connection_entity[ce_index];
    auth = ce_ptr->auth;

    switch (auth->super_state)
    {
        case ENCRYPTED_PAUSED:
        case TEMP_ENCRYPTED_PAUSED:
            auth->txn_params.auth_role = BZ_AUTH_ROLE_INITIATOR;
            bz_auth_perform_super_state_transition(auth, RESUMING_ENCRYPTION_REQ);

            if (bz_auth_is_epr_supported(ce_ptr))
            {
                bz_auth_perform_resume_encryption_action(ce_index, auth);
            }
            else
            {
                bz_auth_perform_start_encryption_action(ce_index, auth);
            }
            break;

        default:
            /* Probably, either the authentication module is not idle or we
             * have not already paused the encryption.
             */
            return FALSE;
    }

    return TRUE;
}

/**
 * Returns whether the ACL link (given by \a ce_index) is encrypted or not.
 *
 * \param ce_index ACL connection entity index.
 *
 * \return TRUE, if the link is encrypted. FALSE, otherwise.
 */
BOOLEAN bz_auth_is_link_encrypted(UINT16 ce_index)
{
#ifdef _SUPPORT_SECURE_CONNECTION_
    if (lmp_connection_entity[ce_index].auth->enc_mode >0)
    {
        return TRUE;
    }
    else
    {
        return FALSE;
    }
#else
    return (BOOLEAN) (lmp_connection_entity[ce_index].auth->enc_mode ==
                      BZ_AUTH_ENCRYPTION_MODE_ON);
#endif                      
}

/**
 * Returns whether the master link key (i.e., temporary key) is current in use
 * or not.
 *
 * \param ce_index ACL Connection entity index of the link for which master
 *                 link key status is required.
 *
 * \return TRUE, if the link is using master link key. FALSE, otherwise.
 */
BOOLEAN bz_auth_is_master_link_key_in_use(UINT16 ce_index)
{
#ifdef COMPILE_BROADCAST_ENCRYPTION
    BZ_AUTH_LINK_PARAMS* auth;
    LMP_CONNECTION_ENTITY* ce_ptr;

    ce_ptr = &lmp_connection_entity[ce_index];
    auth = ce_ptr->auth;

    switch (auth->super_state)
    {
        case AUTHENTICATED_TEMP_AUTHENTICATING:
        case ENCRYPTED_TEMP_ENCRYPTING:
        case TEMP_AUTHENTICATED:
        case TEMP_ENCRYPTED:
        case TEMP_AUTHENTICATED_DISABLING_TEMP_KEY:
        case TEMP_AUTHENTICATED_ENCRIPTING:
        case TEMP_AUTHENTICATED_AUTHENTICATING:
        case TEMP_ENCRYPTED_DISABLING_ENCRYPTION:
        case TEMP_ENCRYPTED_DISABLING_TEMP_KEY:
        case TEMP_ENCRYPTED_REFRESH_KEY:
        case TEMP_ENCRYPTED_PAUSING_ENCRYPTION:
        case TEMP_ENCRYPTED_PAUSED:
        case TEMP_ENCRYPTED_RESUMING_ENCRYPTION:
        case TEMP_ENCRYPTED_AUTHENTICATING:
            return TRUE;
        default:
            break;
    }
#endif /* COMPILE_BROADCAST_ENCRYPTION */
    return FALSE;
}

/**
 * Calculates the DHKey from the local private key and the remote public key.
 *
 * \param auth Authentication parameters to be used.
 *
 * \return None.
 */
void bz_auth_calculate_dhkey(BZ_AUTH_LINK_PARAMS* auth)
{
#ifdef _SSP_DHKEY_CALCULATE_NEW_FLOW_ 
    g_ssp_ce_index = auth->ce_index;
#endif

#ifdef _CCH_SC_ECDH_P256_	
    if(auth->len_prime == 6)
    {
#endif
    /* Calculate the dhkey = p192(our_private_key, remote_public_key)
     * God only knows how much time it will take to perform elliptic curve
     * multiplication :) But anyway, ours is optimized.
     */
    ssp_p192(bz_auth_local_prikey,
             &auth->txn_params.ssp_data.remote_pubkey,
             auth->txn_params.ssp_data.dhkey);

#ifdef _CCH_SC_ECDH_P256_	
    }else if(auth->len_prime == 8)
    {
        ssp_p256(bz_auth_local_prikey_p256,
                 &auth->txn_params.ssp_data.remote_pubkey,
                 auth->txn_params.ssp_data.dhkey);
    }
#endif


#ifdef _CCH_SC_ECDH_P256_LOG


RT_BT_LOG(RED, CCH_DBG_150, 32, 
bz_auth_local_prikey_p256[0],bz_auth_local_prikey_p256[1],bz_auth_local_prikey_p256[2],bz_auth_local_prikey_p256[3],
bz_auth_local_prikey_p256[4],bz_auth_local_prikey_p256[5],bz_auth_local_prikey_p256[6],bz_auth_local_prikey_p256[7],
bz_auth_local_prikey_p256[8],bz_auth_local_prikey_p256[9],bz_auth_local_prikey_p256[10],bz_auth_local_prikey_p256[11],
bz_auth_local_prikey_p256[12],bz_auth_local_prikey_p256[13],bz_auth_local_prikey_p256[14],bz_auth_local_prikey_p256[15],
bz_auth_local_prikey_p256[16],bz_auth_local_prikey_p256[17],bz_auth_local_prikey_p256[18],bz_auth_local_prikey_p256[19],
bz_auth_local_prikey_p256[20],bz_auth_local_prikey_p256[21],bz_auth_local_prikey_p256[22],bz_auth_local_prikey_p256[23],
bz_auth_local_prikey_p256[24],bz_auth_local_prikey_p256[25],bz_auth_local_prikey_p256[26],bz_auth_local_prikey_p256[27],
bz_auth_local_prikey_p256[28],bz_auth_local_prikey_p256[29],bz_auth_local_prikey_p256[30],bz_auth_local_prikey_p256[31]
);
#endif

}

UINT32 s_auto_epr_check_count = BZ_AUTH_AUTO_EPR_TIMER_CHECK_VAL;
/**
 * Timeout handler for periodic encryption pause and resume timer.
 *
 * \param interval Interval for used for this intermediate timeout. Unless
 *                 this intermediate timeouts add up to the required overall
 *                 timeout value, the auto EPR is not done.
 *
 * \return None.
 */
void bz_auth_handle_timeout_for_auto_epr(UINT32 interval)
{
    LMP_CONNECTION_ENTITY* ce_ptr;
    BZ_AUTH_LINK_PARAMS* auth;
    UCHAR i;

    if (s_auto_epr_check_count > interval)
    {
        s_auto_epr_check_count -= interval;
        return;
    }

    s_auto_epr_check_count = BZ_AUTH_AUTO_EPR_TIMER_CHECK_VAL;
    for (i = 0; i < LMP_MAX_CE_DATABASE_ENTRIES; i++)
    {
        ce_ptr = &lmp_connection_entity[i];
        if (ce_ptr->entity_status != ASSIGNED)
        {
            continue;
        }
        auth = ce_ptr->auth;
        switch(auth->super_state)
        {
            case ENCRYPTED:
            case TEMP_ENCRYPTED:
                auth->auto_epr_time_count++;
                if (auth->auto_epr_time_count < BZ_AUTH_AUTO_EPR_TIMER_VAL
                        || (!bz_auth_is_epr_supported(ce_ptr)))
                {
                    break;;
                }
                auth->auto_epr_time_count = 0x0;
                auth->txn_params.auth_role = BZ_AUTH_ROLE_INITIATOR;
                bz_auth_perform_pause_encryption_action(i, auth);
                bz_auth_perform_super_state_transition(auth,
                                                       HCI_REFRESH_ENCRYPTION_KEY);

            default:
                break;
        }
    }
}

