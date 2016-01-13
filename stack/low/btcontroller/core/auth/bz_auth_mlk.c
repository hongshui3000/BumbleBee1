/***************************************************************************
 Copyright (C) MindTree Ltd.
 This module is a confidential and proprietary property of MindTree and
 a possession or use of this module requires written permission of MindTree.
 ***************************************************************************/

/**
 * \file bz_auth_mlk.c
 *  Broadcast encryption (or Master Link Key) implementation.
 *
 * \author Rajan PS
 * \date 2007-08-10
 */
/********************************* Logger *************************/ 
enum { __FILE_NUM__= 25 };
/********************************* Logger *************************/
#ifdef COMPILE_BROADCAST_ENCRYPTION
/* ========================= Include File Section ========================= */
#include "bz_auth.h"
#include "lmp.h"
#include "bt_fw_hci.h"
#include "bt_fw_hci_internal.h"
#include "bz_auth_internal.h"
#include "bz_auth_extern_accessors.h"
#include "bz_debug.h"
#include "bz_auth_lkdb.h"
#include "bz_auth_hci.h"
#include "crypto.h"
#include "crypto11.h"
#include "bz_auth_lmp.h"
#include "bz_auth_mlk.h"
#include "mem.h"
/* ====================== Macro Declaration Section ======================= */


/* ==================== Structure Declaration Section ===================== */


/* ===================== Variable Declaration Section ===================== */

/* ================== Static Function Prototypes Section ================== */


/* ===================== Function Definition Section ====================== */
/**
 * Generate and send LMP_temp_rand PDU to the remote device.
 *
 * \param ce_index ACL connection entity index.
 *
 * \return None.
 */
void bz_auth_send_temp_rand_pdu(UINT16 ce_index)
{
    UCHAR param_list[LMP_TEMP_RAND_LEN];

	param_list[0] = LMP_TEMP_RAND_OPCODE;
    /* copy the temp rand number */
    memcpy(&param_list[2], &bz_auth_dev_params.mlk.temp_rand[0],
            BZ_AUTH_TEMP_RAND_SIZE);
    bz_auth_generate_pdu(ce_index, param_list, LMP_TEMP_RAND_LEN,
            MASTER_TID);
}

/**
 * Generate and send LMP_temp_key PDU to the remote device.
 *
 * \param ce_index ACL connection entity index.
 * \param auth Authentication parameters associated with the \a ce_index.
 *
 * \return None.
 */
void bz_auth_send_temp_key_pdu(UINT16 ce_index,
        BZ_AUTH_LINK_PARAMS* auth)
{
    UCHAR param_list[LMP_TEMP_KEY_LEN];
    UCHAR bd_addr[LMP_BD_ADDR_SIZE] = {0};
    register int i;

    param_list[0] = LMP_TEMP_KEY_OPCODE;
    lp_E22(&bz_auth_dev_params.mlk.temp_rand[0],
            &auth->link_key[0],
            BZ_AUTH_TEMP_KEY_SIZE, &bd_addr[0], &param_list[2]);

    /* protect the temp key random number with master link key */
    for (i = 0; i < BZ_AUTH_TEMP_KEY_SIZE; i++)
    {
        param_list[2+i] =
            (UCHAR)(param_list[2+i]^bz_auth_dev_params.mlk.temp_key[i]);
    }
    /* send protected temp key */
    bz_auth_generate_pdu(ce_index, param_list, LMP_TEMP_KEY_LEN, MASTER_TID);
}

/**
 * Generate and send LMP_use_semi_permanent_key PDU to the remote device.
 *
 * \param ce_index ACL connection entity index.
 *
 * \return None.
 */
void bz_auth_send_use_semi_permanent_key_pdu(UINT16 ce_index)
{
    UCHAR param_list[LMP_USE_SEMI_PERMANENT_KEY_LEN];
    param_list[0] = LMP_USE_SEMI_PERMANENT_KEY_OPCODE;
    bz_auth_generate_pdu(ce_index, param_list,
            LMP_USE_SEMI_PERMANENT_KEY_LEN, MASTER_TID);
}

/**
 * Performs transition to next mlk state.
 *
 * \param next_state Next mlk state.
 *
 * \return None.
 */
void bz_auth_transition_to_mlk_state(BZ_AUTH_MLK_STATE next_state)
{
    bz_auth_dev_params.mlk.state = next_state;
}


/**
 * Checks whether change link key type allowed on this connection.
 *
 * \param ce_ptr Pointer to connection entity.
 *
 * \return TRUE, if change link key type allowed. FALSE, otherwise.
 */
BOOLEAN bz_auth_is_change_key_type_allowed(LMP_CONNECTION_ENTITY* ce_ptr)
{
    if (ce_ptr->ce_status == LMP_CONNECTED
#ifdef COMPILE_SNIFF_MODE
            || ce_ptr->ce_status == LMP_SNIFF_MODE
#endif
            )
    {
        return TRUE;
    }
    return FALSE;
}

/**
 * Checks whether disconnecting mlk link.
 *
 * \param ce_ptr Pointer to connection entity.
 *
 * \return TRUE, if disconnecting otherwise FALSE.
 */
BOOLEAN bz_auth_is_disconnecting_mlk_link(LMP_CONNECTION_ENTITY* ce_ptr)
{
    BZ_AUTH_LINK_PARAMS* auth;
    if (ce_ptr->ce_status == LMP_DISCONNECTING)
    {
        auth = ce_ptr->auth;
        switch(auth->super_state)
        {
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
                return TRUE;
            default:
                break;
        }
    }
    return FALSE;
}

/**
 * Returns the ce_index of any one of the mlk link.
 *
 * \param None.
 *
 * \return ce_index of mlk links if nay otherwise INVALID_CE_INDEX
 */
UINT16 bz_auth_get_any_mlk_link_index(void)
{
    LMP_CONNECTION_ENTITY* ce_ptr;
    BZ_AUTH_LINK_PARAMS* auth;
    UCHAR i;

    for (i = 0; i < LMP_MAX_CE_DATABASE_ENTRIES; i++)
    {
        ce_ptr = &lmp_connection_entity[i];
        if (bz_auth_is_slave(ce_ptr) ||
                !bz_auth_is_change_key_type_allowed(ce_ptr))
        {
            continue;
        }
        auth = ce_ptr->auth;
        switch(auth->super_state)
        {
            case TEMP_AUTHENTICATED:
            case TEMP_AUTHENTICATED_ENCRIPTING:
            case TEMP_AUTHENTICATED_AUTHENTICATING:
            case TEMP_ENCRYPTED_DISABLING_ENCRYPTION:
            case TEMP_ENCRYPTED_REFRESH_KEY:
            case TEMP_ENCRYPTED_PAUSING_ENCRYPTION:
            case TEMP_ENCRYPTED_PAUSED:
            case TEMP_ENCRYPTED_RESUMING_ENCRYPTION:
            case TEMP_ENCRYPTED_AUTHENTICATING:
            case TEMP_ENCRYPTED:
                return i;
            default:
                break;
        }
    }
    return INVALID_CE_INDEX;
}

/**
 * Generates master link key complete event.
 *
 * \param ce_index ACL connection entity index of valid link.
 * \param status status of mlk procedure.
 * \param key_type mlk key type (temp_key(1) or semi_per_key(0)).
 *
 * \return None.
 */
void bz_auth_send_master_link_key_complete_event(UINT16 ce_index,
        UCHAR status, UCHAR key_type)
{
    UCHAR event_parameter[4];
    UINT16 conn_handle = 0x0;

    if (ce_index != INVALID_CE_INDEX)
    {
        conn_handle =
            lmp_connection_entity[ce_index].connection_type.connection_handle;
    }

    event_parameter[0] = status;
    event_parameter[1] = (UCHAR)conn_handle;
    event_parameter[2] = (UCHAR)(conn_handle>>8);
    event_parameter[3] = key_type;
    hci_generate_event(HCI_MASTER_LINK_KEY_COMPLETE_EVENT, event_parameter, 4);
}

/**
 * Changes the broadcast encryption (enable/disable) based on mlk links.
 *
 * \param None.
 *
 * \return None.
 */
void bz_auth_change_broadcast_encryption(void)
{
    LMP_CONNECTION_ENTITY* ce_ptr;
    BZ_AUTH_LINK_PARAMS* auth;
    UCHAR i;
    BOOLEAN is_to_enable_broadcast_enc = FALSE;

    for (i = 0; i < LMP_MAX_CE_DATABASE_ENTRIES; i++)
    {
        ce_ptr = &lmp_connection_entity[i];
        if (bz_auth_is_slave(ce_ptr))
        {
            continue;
        }
        auth = ce_ptr->auth;
        switch(auth->super_state)
        {
            case TEMP_ENCRYPTED_REFRESH_KEY:
            case TEMP_ENCRYPTED_PAUSING_ENCRYPTION:
            case TEMP_ENCRYPTED_PAUSED:
            case TEMP_ENCRYPTED_RESUMING_ENCRYPTION:
            case TEMP_ENCRYPTED_AUTHENTICATING:
            case TEMP_ENCRYPTED:
                is_to_enable_broadcast_enc = TRUE;
                break;
            default:
                break;
        }
    }
    if (is_to_enable_broadcast_enc == TRUE)
    {
        bz_auth_enable_broadcast_encryption(bz_auth_dev_params.mlk.enc_key);
    }
    else
    {
        bz_auth_disable_broadcast_encryption();
    }
}

/**
 * Handles changes on any mlk link, sends mlk completion event if mlk
 * procedure completed on all associated links.
 *
 * \param ce_index ACL connection enity index.
 * \param status status of mlk change.
 *
 * \return None.
 */
void bz_auth_handle_mlk_link_changes(UINT16 ce_index, UCHAR status)
{
    UINT16 mlk_ce_index;
    switch (bz_auth_dev_params.mlk.state)
    {
        case MASTER_LINK_KEY_UPDATING:
            if (bz_auth_dev_params.mlk.n_mlk_successes == 0x0)
            {
                bz_auth_disable_broadcast_encryption();
            }
            else
            {
                bz_auth_change_broadcast_encryption();
            }
            if (bz_auth_dev_params.mlk.n_mlk_links ==
                    (bz_auth_dev_params.mlk.n_mlk_failures
                     + bz_auth_dev_params.mlk.n_mlk_successes))
            {
                /* finished mlk procedure
                 * send mlk completion event
                 */
                if (bz_auth_dev_params.mlk.n_mlk_successes == 0x0)
                {
                    bz_auth_transition_to_mlk_state(MASTER_LINK_KEY_IDLE);
                    mlk_ce_index = ce_index;
                }
                else
                {
                    bz_auth_transition_to_mlk_state(MASTER_LINK_KEY_IN_USE);
                    mlk_ce_index = bz_auth_get_any_mlk_link_index();
                    status = HCI_COMMAND_SUCCEEDED;
                }
                bz_auth_send_master_link_key_complete_event(mlk_ce_index,
                            status, 0x1);
            }
            break;
        case MASTER_LINK_DISABLING:
            if (bz_auth_dev_params.mlk.n_mlk_links ==
                    (bz_auth_dev_params.mlk.n_mlk_failures
                     + bz_auth_dev_params.mlk.n_mlk_successes))
            {
                 bz_auth_disable_broadcast_encryption();
                /* finished mlk procedure
                 * send mlk completion event
                 */
                if (bz_auth_dev_params.mlk.n_mlk_successes != 0x0)
                {
                    status = HCI_COMMAND_SUCCEEDED;
                }
                bz_auth_send_master_link_key_complete_event(ce_index,
                        status, 0x0);
                bz_auth_dev_params.mlk.n_mlk_links =
                    bz_auth_dev_params.mlk.n_mlk_failures =
                    bz_auth_dev_params.mlk.n_mlk_successes = 0x0;

                bz_auth_transition_to_mlk_state(MASTER_LINK_KEY_IDLE);
            }
            break;
        case MASTER_LINK_KEY_IN_USE:
            if (bz_auth_dev_params.mlk.n_mlk_successes == 0x0)
            {
                bz_auth_disable_broadcast_encryption();
                bz_auth_transition_to_mlk_state(MASTER_LINK_KEY_IDLE);
            }
            else
            {
                bz_auth_change_broadcast_encryption();
            }
            break;
        default:
            break;
    }
}

/**
 * Hanldes disconnection complete on mlk link.
 *
 * \param ce_index ACL connection entity index.
 * \param link_mlk_state mlk state of the disconnected link.
 *
 * \return None.
 */
void bz_auth_handle_mlk_link_disconnected(UINT16 ce_index,
        BZ_AUTH_MLK_STATE link_mlk_state)
{
    LMP_CONNECTION_ENTITY* ce_ptr;

    ce_ptr = &lmp_connection_entity[ce_index];

    if (bz_auth_is_slave(ce_ptr))
    {
        return;
    }
    switch(bz_auth_dev_params.mlk.state)
    {
        case MASTER_LINK_KEY_IN_USE:
            bz_auth_dev_params.mlk.n_mlk_successes--;
            bz_auth_dev_params.mlk.n_mlk_links--;
            break;
        case MASTER_LINK_KEY_UPDATING:
            if (link_mlk_state == MASTER_LINK_KEY_IN_USE)
            {
                bz_auth_dev_params.mlk.n_mlk_successes--;
            }
            bz_auth_dev_params.mlk.n_mlk_links--;
            break;
        case MASTER_LINK_DISABLING:
            bz_auth_dev_params.mlk.n_mlk_links--;
            break;
        default:
            return;
    }
    bz_auth_handle_mlk_link_changes(ce_index, UNSPECIFIED_ERROR);
}

/**
 * Handles switch to temp_key completion.
 *
 * \param ce_index ACL connection entity index.
 * \param status status key switch
 *
 * \return None.
 */
void bz_auth_handle_master_link_key_enable_completion(UINT16 ce_index,
        UCHAR status)
{
    LMP_CONNECTION_ENTITY* ce_ptr;

    ce_ptr = &lmp_connection_entity[ce_index];

    if (bz_auth_is_slave(ce_ptr))
    {
        /* send the master link key command complete event. */
        bz_auth_send_master_link_key_complete_event(ce_index, status, 0x1);
        return;
    }

    if (bz_auth_dev_params.mlk.state != MASTER_LINK_KEY_UPDATING)
    {
        if (status != HCI_COMMAND_SUCCEEDED)
        {
            bz_auth_dev_params.mlk.n_mlk_successes--;
            bz_auth_dev_params.mlk.n_mlk_failures++;
            lmp_handle_acl_disconnect(ce_index, status);
            bz_auth_handle_mlk_link_changes(ce_index, status);
        }
    }
    else
    {
        if (status == HCI_COMMAND_SUCCEEDED)
        {
            bz_auth_dev_params.mlk.n_mlk_successes++;
        }
        else
        {
            bz_auth_dev_params.mlk.n_mlk_failures++;
            lmp_handle_acl_disconnect(ce_index, status);
        }
        bz_auth_handle_mlk_link_changes(ce_index, status);
    }

}

/**
 * Handles reauthentication on mlk link.
 *
 * \param ce_index ACL connection entity index.
 * \param status reauth status
 *
 * \return None.
 */
void bz_auth_handle_master_link_key_reauth_completion(UINT16 ce_index,
        UCHAR status)
{
    LMP_CONNECTION_ENTITY* ce_ptr;

    ce_ptr = &lmp_connection_entity[ce_index];

    if (bz_auth_is_slave(ce_ptr))
    {
        return;
    }
    if (status != HCI_COMMAND_SUCCEEDED)
    {
        lmp_handle_acl_disconnect(ce_index, status);
    }
}

/**
 * Handles switch to semi_per_key completion.
 *
 * \param ce_index ACL connection entity index.
 * \param status status key switch
 *
 * \return None.
 */
void bz_auth_handle_master_link_key_disable_completion(UINT16 ce_index,
        UCHAR status)
{
    LMP_CONNECTION_ENTITY* ce_ptr;

    ce_ptr = &lmp_connection_entity[ce_index];

    if (bz_auth_is_slave(ce_ptr))
    {
        /* send the master link key command complete event. */
        bz_auth_send_master_link_key_complete_event(ce_index, status, 0x0);
        return;
    }

    if (bz_auth_dev_params.mlk.state != MASTER_LINK_DISABLING)
    {
        return;
    }

    ce_ptr = &lmp_connection_entity[ce_index];
    if (status == HCI_COMMAND_SUCCEEDED)
    {
        bz_auth_dev_params.mlk.n_mlk_successes++;
        bz_auth_handle_mlk_link_changes(ce_index, status);
    }
    else
    {
        bz_auth_dev_params.mlk.n_mlk_failures++;
        bz_auth_handle_mlk_link_changes(ce_index, status);
        if (bz_auth_dev_params.mlk.n_mlk_failures != 0)
        {
            /*
             * This (mlk.n_mlk_failures--) is to compensate the previous
             * n_mlk_failures++.
             * As we disconnect the link and when we receive the
             * bz_auth_handle_mlk_link_disconnected we decrement n_mlk_links.
             * So this help us to nullify the effect of this link in the final
             * decision of finding the completion of disable master link key
             * procedure.
             */
            bz_auth_dev_params.mlk.n_mlk_failures--;
        }
        lmp_handle_acl_disconnect(ce_index, status);
    }

}

/**
 * Handles enable encryption on mlk link.
 *
 * \param ce_index ACL connection entity index.
 * \param status status of enable encryption.
 *
 * \return None.
 */
void bz_auth_handle_master_link_key_enable_enc_completion(UINT16 ce_index,
        UCHAR status)
{
    LMP_CONNECTION_ENTITY* ce_ptr;

    ce_ptr = &lmp_connection_entity[ce_index];
    if (bz_auth_is_master(ce_ptr))
    {
        if (status != HCI_COMMAND_SUCCEEDED)
        {
            lmp_handle_acl_disconnect(ce_index, status);
        }
        bz_auth_change_broadcast_encryption();
    }
}

/**
 * Handles disable encryption on mlk link.
 *
 * \param ce_index ACL connection entity index.
 * \param status status of disable encryption.
 *
 * \return None.
 */
void bz_auth_handle_master_link_key_disable_enc_completion(UINT16 ce_index,
        UCHAR status)
{
    LMP_CONNECTION_ENTITY* ce_ptr;

    ce_ptr = &lmp_connection_entity[ce_index];
    if (bz_auth_is_master(ce_ptr))
    {
        if (status != HCI_COMMAND_SUCCEEDED)
        {
            lmp_handle_acl_disconnect(ce_index, status);
        }
        bz_auth_change_broadcast_encryption();
    }
}


/**
 * Initiates switch to semi_permanent key procedure.
 *
 * \param ce_index ACL connection entity index.
 *
 * \return TRUE if success, otherwise FALSE
 */
BOOLEAN bz_auth_switch_to_semi_per_key(UINT16 ce_index)
{
    LMP_CONNECTION_ENTITY* ce_ptr;
    BZ_AUTH_LINK_PARAMS* auth;

    ce_ptr = &lmp_connection_entity[ce_index];
    if (bz_auth_is_slave(ce_ptr) ||
            !bz_auth_is_change_key_type_allowed(ce_ptr))
    {
        return FALSE;
    }
    auth = ce_ptr->auth;
    switch(auth->super_state)
    {
        case TEMP_AUTHENTICATED:
        case TEMP_ENCRYPTED:
            break;
        default:
            return FALSE;
    }
    auth->txn_params.auth_role = BZ_AUTH_ROLE_INITIATOR;
    /* send LMP_use_semi_permanent_key */
    bz_auth_send_use_semi_permanent_key_pdu(ce_index);
    bz_auth_transition_to_sub_state(auth, USE_SEMI_PER_KEY_PDU_SENT);
    bz_auth_perform_super_state_transition(auth,
            HCI_MLK_SEMI_PER_OR_LMP_SEMI_PER_KEY);
    return TRUE;
}

/**
 * Initiates switch to temp_key procedure.
 *
 * \param ce_index ACL connection entity index.
 *
 * \return TRUE if success, otherwise FALSE
 */
BOOLEAN bz_auth_switch_to_temp_key(UINT16 ce_index)
{
    LMP_CONNECTION_ENTITY* ce_ptr;
    BZ_AUTH_LINK_PARAMS* auth;

    ce_ptr = &lmp_connection_entity[ce_index];
    auth = ce_ptr->auth;
    if (bz_auth_is_slave(ce_ptr) ||
            !bz_auth_is_change_key_type_allowed(ce_ptr))
    {
        return FALSE;
    }

    switch(auth->super_state)
    {
        case AUTHENTICATED:
        case ENCRYPTED:
            break;
        default:
            return FALSE;
    }

    /* take the backup of semi permanent key */
    memcpy(&auth->semi_bk_link_key[0], &auth->link_key[0],
            BZ_AUTH_LINK_KEY_SIZE);
    auth->semi_bk_lk_type = auth->lk_type;
    memcpy(&auth->semi_bk_aco[0], &auth->aco[0], BZ_AUTH_ACO_SIZE);

    /* copy the master link key to txn_params */
    memcpy(&auth->txn_params.link_key[0], &bz_auth_dev_params.mlk.temp_key[0],
            BZ_AUTH_LINK_KEY_SIZE);
    auth->txn_params.lk_type = auth->lk_type;

    auth->txn_params.auth_role = BZ_AUTH_ROLE_INITIATOR;
    /* send LMP_temp_rand */
    bz_auth_send_temp_rand_pdu(ce_index);
    /* send LMP_temp_key */
    bz_auth_send_temp_key_pdu(ce_index, auth);
    /* send LMP_au_rand */
    bz_auth_send_au_rand_pdu(ce_index, auth, MASTER_TID);
#ifdef SECURE_CONN_MUTUAL_AUTH
    if (auth->secure_conn_enabled)
    {
        bz_auth_transition_to_sub_state(auth,
            SECURE_CONN_MUTUAL_SEND_AU_RAND);
    }
    else
#endif
    {    
    bz_auth_transition_to_sub_state(auth, INTR_CHALLENGED_REMOTE_HOST_MUTUAL);
    }
    bz_auth_perform_super_state_transition(auth, HCI_MLK_TEMP_OR_LMP_TEMP_RAND);
    return TRUE;
}

/**
 * Initiates if the mlk procedure is pending with this connection.
 *
 * \param ce_index ACL connection entity index.
 * \param status status of operation.
 *
 * \return None.
 */
void bz_auth_initiate_mlk_procedure_if_pending(UINT16 ce_index, UCHAR status)
{
    LMP_CONNECTION_ENTITY* ce_ptr;

    ce_ptr = &lmp_connection_entity[ce_index];

    if (bz_auth_is_slave(ce_ptr) ||
            !bz_auth_is_change_key_type_allowed(ce_ptr))
    {
        return;
    }

    switch(bz_auth_dev_params.mlk.state)
    {
        case MASTER_LINK_KEY_UPDATING:
            bz_auth_switch_to_temp_key(ce_index);
            break;
        case MASTER_LINK_DISABLING:
            bz_auth_switch_to_semi_per_key(ce_index);
            break;
        default:
            break;
    }
}

/**
 * Handles master link key command with use semi permanent key
 * \param hci_cmd_ptr Pointer to HCI command packet.
 * \return proper error code or HCI_COMMAND_SUCCEEDED
 */
UCHAR bz_auth_handle_master_link_key_semi(HCI_CMD_PKT* hci_cmd_ptr)
{
    LMP_CONNECTION_ENTITY* ce_ptr;
    BZ_AUTH_LINK_PARAMS* auth;
    UCHAR i;

    hci_generate_command_status_event(hci_cmd_ptr->cmd_opcode,
            HCI_COMMAND_SUCCEEDED);
    bz_auth_dev_params.mlk.n_mlk_links = 0x0;
    for (i = 0; i < LMP_MAX_CE_DATABASE_ENTRIES; i++)
    {
        ce_ptr = &lmp_connection_entity[i];
        if (bz_auth_is_slave(ce_ptr))
        {
          continue;
        }
        if (!bz_auth_is_change_key_type_allowed(ce_ptr))
        {
            if (bz_auth_is_disconnecting_mlk_link(ce_ptr))
            {
                /* Increment the count so that we can freely
                 * decrement this count upon call back.
                 */
                bz_auth_dev_params.mlk.n_mlk_links++;
            }
            continue;
        }
        auth = ce_ptr->auth;
        switch(auth->super_state)
        {
            case TEMP_AUTHENTICATED:
            case TEMP_ENCRYPTED:
                bz_auth_switch_to_semi_per_key(i);
                /*@fallsthrough*/
            case TEMP_AUTHENTICATED_ENCRIPTING:
            case TEMP_AUTHENTICATED_AUTHENTICATING:
            case TEMP_ENCRYPTED_DISABLING_ENCRYPTION:
            case TEMP_ENCRYPTED_REFRESH_KEY:
            case TEMP_ENCRYPTED_PAUSING_ENCRYPTION:
            case TEMP_ENCRYPTED_PAUSED:
            case TEMP_ENCRYPTED_RESUMING_ENCRYPTION:
            case TEMP_ENCRYPTED_AUTHENTICATING:
                    bz_auth_dev_params.mlk.n_mlk_links++;
            default:
                break;
        }
    }

    bz_auth_dev_params.mlk.n_mlk_failures = 0x0;
    bz_auth_dev_params.mlk.n_mlk_successes = 0x0;
    bz_auth_transition_to_mlk_state(MASTER_LINK_DISABLING);
    return HCI_COMMAND_SUCCEEDED;
}

/**
 * Handles master link key command with use temp_key
 * \param hci_cmd_ptr Pointer to HCI command packet.
 * \return proper error code or HCI_COMMAND_SUCCEEDED
 */
UCHAR bz_auth_handle_master_link_key_temp(HCI_CMD_PKT* hci_cmd_ptr)
{
    LMP_CONNECTION_ENTITY* ce_ptr;
    BZ_AUTH_LINK_PARAMS* auth;
    UCHAR i;
    UCHAR nmlk_required = 0;
    UCHAR nmlk_links = 0;
    UCHAR nmlk_link_ce_index = 0;
    UCHAR temp_rand1[BZ_AUTH_TEMP_RAND_SIZE];
    UCHAR temp_rand2[BZ_AUTH_TEMP_RAND_SIZE];
    UCHAR cof[2*LMP_BD_ADDR_SIZE];
    UCHAR bd_addr[LMP_BD_ADDR_SIZE] = {0};

    for (i = 0; i < LMP_MAX_CE_DATABASE_ENTRIES; i++)
    {
        ce_ptr = &lmp_connection_entity[i];
        if (bz_auth_is_slave(ce_ptr) ||
                !bz_auth_is_change_key_type_allowed(ce_ptr))
        {
          continue;
        }
        auth = ce_ptr->auth;
        switch(auth->super_state)
        {
            case AUTHENTICATED:
            case ENCRYPTED:
            case AUTHENTICATING:
            case AUTHENTICATED_ENCRYPTING:
            case AUTHENTICATED_CHANGING_KEY:
            case AUTHENTICATED_UPGRADE_KEY:
            case ENCRYPTED_CHANGING_KEY:
            case ENCRYPTED_UPGRADE_KEY:
            case ENCRYPTED_REFRESH_KEY:
            case ENCRYPTED_PAUSING_ENCRYPTION:
            case ENCRYPTED_PAUSED:
            case ENCRYPTED_RESUMING_ENCRYPTION:
            case ENCRYPTED_DISABLING:
#ifdef SECURE_CONN_BROADCAST
                if (auth->secure_conn_enabled)
                {
                    nmlk_link_ce_index = i;
                    nmlk_links++;
                }
                else
#endif            
                {
                    nmlk_required++;
                }
                
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
                nmlk_link_ce_index = i;
                nmlk_links++;
                break;
            default:
                break;
        }
    }

    if ((nmlk_required == 0x0) && (nmlk_links == 0x0))
    {
        return COMMAND_DISALLOWED_ERROR;
    }
    
    hci_generate_command_status_event(hci_cmd_ptr->cmd_opcode,HCI_COMMAND_SUCCEEDED);

    if (nmlk_required == 0x0)
    {
        /*
         * send HCI_Master_Link_Key_Complete_Event(success, key_type(temp_key))
         */
        bz_auth_send_master_link_key_complete_event(nmlk_link_ce_index,
                HCI_COMMAND_SUCCEEDED, 0x1);
#ifdef SECURE_CONN_BROADCAST_CHK
// dape test
RT_BT_LOG(GREEN, DAPE_TEST_LOG293, 1,1234);
#endif                
        return HCI_COMMAND_SUCCEEDED;
    }
    if (bz_auth_dev_params.mlk.state == MASTER_LINK_KEY_IDLE)
    {
        bz_auth_dev_params.mlk.n_mlk_successes = 0x0;
        /* Generate random numbers */
        ssp_rng_get(&temp_rand1[0]);
        ssp_rng_get(&temp_rand2[0]);
        ssp_rng_get(&bz_auth_dev_params.mlk.temp_rand[0]);
        ssp_rng_get(&bz_auth_dev_params.mlk.enc_rand[0]);
        /* generate master link key->temp_key */
        lp_E22(&temp_rand1[0], &temp_rand2[0], BZ_AUTH_TEMP_RAND_SIZE,
            &bd_addr[0], &bz_auth_dev_params.mlk.temp_key[0]);

        memcpy(&cof[0], &bz_auth_get_local_bd_addr(), LMP_BD_ADDR_SIZE);
        memcpy(&cof[LMP_BD_ADDR_SIZE], &cof[0], LMP_BD_ADDR_SIZE);
        lp_E3(bz_auth_dev_params.mlk.temp_key,
                &bz_auth_dev_params.mlk.enc_rand[0], cof,
                &bz_auth_dev_params.mlk.enc_key[0],
                bz_auth_dev_params.max_enc_key_size);

    }

    for (i = 0; i < LMP_MAX_CE_DATABASE_ENTRIES; i++)
    {
        ce_ptr = &lmp_connection_entity[i];
        if (bz_auth_is_slave(ce_ptr) ||
                !bz_auth_is_change_key_type_allowed(ce_ptr))
        {
          continue;
        }
        auth = ce_ptr->auth;
        switch(auth->super_state)
        {
            case AUTHENTICATED:
            case ENCRYPTED:
#ifdef SECURE_CONN_BROADCAST
                if (!auth->secure_conn_enabled)
#endif            
                {
                bz_auth_switch_to_temp_key(i);
                }
                break;
            default:
                break;
        }
    }
    bz_auth_dev_params.mlk.n_mlk_failures = 0x0;
    bz_auth_dev_params.mlk.n_mlk_links =
        (UCHAR)(bz_auth_dev_params.mlk.n_mlk_successes + nmlk_required);
    bz_auth_transition_to_mlk_state(MASTER_LINK_KEY_UPDATING);
    return HCI_COMMAND_SUCCEEDED;
}

/**
 * Handles master link key command.
 * \param hci_cmd_ptr Pointer to HCI command packet.
 * \return proper error code or HCI_COMMAND_SUCCEEDED
 */
UCHAR bz_auth_handle_master_link_key(HCI_CMD_PKT* hci_cmd_ptr)
{
    UCHAR key_flag;

    key_flag = hci_cmd_ptr->cmd_parameter[0];
    if ((key_flag != 0) && (key_flag != 1))
    {
        return INVALID_HCI_COMMAND_PARAMETERS_ERROR;
    }

    if (key_flag == 0x0)
    {
        switch (bz_auth_dev_params.mlk.state)
        {
            case MASTER_LINK_KEY_IN_USE:
                return bz_auth_handle_master_link_key_semi(hci_cmd_ptr);
            default:
                break;
        }
    }
    else
    {
        switch (bz_auth_dev_params.mlk.state)
        {
            case MASTER_LINK_KEY_IDLE:
                if(lc_get_no_of_piconets_connected() != 1)
                {
                    /* Connected in > 1 piconet. MLK is not allowed. */
                    RT_BT_LOG(GRAY, BZ_AUTH_MLK_969, 0, 0);
                    break;
                }
                if(lc_get_master_piconet_id() == SCA_PICONET_INVALID)
                {
                    /* Not master. */
                    RT_BT_LOG(GRAY, BZ_AUTH_MLK_975, 0, 0);
                    break;
                }

            case MASTER_LINK_KEY_IN_USE:
                return bz_auth_handle_master_link_key_temp(hci_cmd_ptr);
            default:
                break;
        }
    }
    return COMMAND_DISALLOWED_ERROR;
}

/**
 * Handles lmp_accepted(lmp_use_semi_permanent_key) PDU from the remote device.
 *
 * \param lmp_pdu_ptr LMP PDU packet from the remote device.
 * \param ce_index ACL Connection entity index.
 * \param can_free_pdu Set to TRUE, if the PDU can be freed by the caller.
 *                     FALSE, otherwise.
 *
 * \return None.
 */
void bz_auth_handle_use_semi_permanent_key_accepted_pdu(
        LMP_PDU_PKT* lmp_pdu_ptr, UINT16 ce_index, OUT BOOLEAN* can_free_pdu)
{
    LMP_CONNECTION_ENTITY* ce_ptr;
    BZ_AUTH_LINK_PARAMS* auth;
    LMP_TRAN_ID tid;

    ce_ptr = &lmp_connection_entity[ce_index];
    auth = ce_ptr->auth;
    tid = (LMP_TRAN_ID)LMP_GET_TRANSACTION_ID(lmp_pdu_ptr);
    if (tid != MASTER)
    {
        BZ_ASSERT(0, "Bug in the tid calculation or remote device is "
                "misbehaving");
        return;
    }

    switch (auth->sub_state)
    {
        case USE_SEMI_PER_KEY_PDU_SENT:
            bz_auth_handle_auth_completion(ce_index, HCI_COMMAND_SUCCEEDED);
            break;
        default:
            BZ_ASSERT(0, "Bug in the state machine or remote device is "
                    "misbehaving");
            break;
    }
}

/**
 * Handles lmp_temp_rand PDU from the remote device.
 *
 * \param lmp_pdu_ptr LMP PDU packet from the remote device.
 * \param ce_index ACL Connection entity index.
 * \param can_free_pdu Set to TRUE, if the PDU can be freed by the caller.
 *                     FALSE, otherwise.
 *
 * \return None.
 */
void bz_auth_handle_temp_rand_pdu(LMP_PDU_PKT *lmp_pdu_ptr, UINT16 ce_index,
        OUT BOOLEAN* can_free_pdu)
{
    LMP_CONNECTION_ENTITY* ce_ptr;
    BZ_AUTH_LINK_PARAMS* auth;
    LMP_TRAN_ID tid;

    ce_ptr = &lmp_connection_entity[ce_index];
    auth = ce_ptr->auth;
    tid = (LMP_TRAN_ID)LMP_GET_TRANSACTION_ID(lmp_pdu_ptr);
    if (bz_auth_is_master(ce_ptr)
            || tid != MASTER
            || auth->sub_state != BZ_AUTH_SUB_STATE_IDLE)
    {
        bz_auth_send_not_accepted_pdu(ce_index, LMP_TEMP_RAND_OPCODE, tid,
                PDU_NOT_ALLOWED_ERROR);
        return;
    }

    switch (auth->super_state)
    {
        case AUTHENTICATED:
        case ENCRYPTED:
            auth->txn_params.auth_role = BZ_AUTH_ROLE_RESPONDER;
            bz_auth_set_pending_pdu(auth, lmp_pdu_ptr, can_free_pdu);
            bz_auth_transition_to_sub_state(auth, RESP_AWAIT_TEMP_KEY);
            break;;
        default:
            /* send LMP not accepted. */
            bz_auth_send_not_accepted_pdu(ce_index, LMP_TEMP_RAND_OPCODE, tid,
                    PDU_NOT_ALLOWED_ERROR);
            return;
    }
    bz_auth_perform_super_state_transition(auth, HCI_MLK_TEMP_OR_LMP_TEMP_RAND);
}

/**
 * Handles lmp_temp_key PDU from the remote device.
 *
 * \param lmp_pdu_ptr LMP PDU packet from the remote device.
 * \param ce_index ACL Connection entity index.
 * \param can_free_pdu Set to TRUE, if the PDU can be freed by the caller.
 *                     FALSE, otherwise.
 *
 * \return None.
 */
void bz_auth_handle_temp_key_pdu(LMP_PDU_PKT *lmp_pdu_ptr, UINT16 ce_index,
        OUT BOOLEAN* can_free_pdu)
{
    LMP_CONNECTION_ENTITY* ce_ptr;
    BZ_AUTH_LINK_PARAMS* auth;
    LMP_TRAN_ID tid;
    UCHAR bd_addr[LMP_BD_ADDR_SIZE] = {0};
    UCHAR i;

    ce_ptr = &lmp_connection_entity[ce_index];
    auth = ce_ptr->auth;
    tid = (LMP_TRAN_ID)LMP_GET_TRANSACTION_ID(lmp_pdu_ptr);
    if (bz_auth_is_master(ce_ptr)
            || tid != MASTER
            || auth->sub_state != RESP_AWAIT_TEMP_KEY)
    {
        bz_auth_send_not_accepted_pdu(ce_index, LMP_TEMP_KEY_OPCODE, tid,
                PDU_NOT_ALLOWED_ERROR);
        return;
    }
    lp_E22(&auth->pending_pdu->payload_content[1],
            &auth->link_key[0],
            BZ_AUTH_TEMP_KEY_SIZE, &bd_addr[0], &auth->txn_params.link_key[0]);
    bz_auth_free_pending_pdu(auth);
    for (i = 0; i < BZ_AUTH_TEMP_KEY_SIZE; i++)
    {
        auth->txn_params.link_key[i] =
            (UCHAR) (lmp_pdu_ptr->payload_content[1+i]
                    ^ auth->txn_params.link_key[i]);

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
}

/**
 * Handles lmp_use_semi_permanent_key PDU from the remote device.
 *
 * \param lmp_pdu_ptr LMP PDU packet from the remote device.
 * \param ce_index ACL Connection entity index.
 * \param can_free_pdu Set to TRUE, if the PDU can be freed by the caller.
 *                     FALSE, otherwise.
 *
 * \return None.
 */
void bz_auth_handle_use_semi_permanent_key_pdu(LMP_PDU_PKT *lmp_pdu_ptr,
        UINT16 ce_index, OUT BOOLEAN* can_free_pdu)
{
    LMP_CONNECTION_ENTITY* ce_ptr;
    BZ_AUTH_LINK_PARAMS* auth;
    LMP_TRAN_ID tid;

    ce_ptr = &lmp_connection_entity[ce_index];
    auth = ce_ptr->auth;
    tid = (LMP_TRAN_ID)LMP_GET_TRANSACTION_ID(lmp_pdu_ptr);
    if (bz_auth_is_master(ce_ptr)
            || tid != MASTER
            || auth->sub_state != BZ_AUTH_SUB_STATE_IDLE)
    {
        bz_auth_send_not_accepted_pdu(ce_index,
                LMP_USE_SEMI_PERMANENT_KEY_OPCODE, tid, PDU_NOT_ALLOWED_ERROR);
        return;
    }
    switch (auth->super_state)
    {
        case TEMP_AUTHENTICATED:
        case TEMP_ENCRYPTED:
            break;
        default:
            /* send LMP not accepted. */
            bz_auth_send_not_accepted_pdu(ce_index,
                    LMP_USE_SEMI_PERMANENT_KEY_OPCODE, tid,
                    PDU_NOT_ALLOWED_ERROR);
            return;

    }
    bz_auth_send_accepted_pdu(ce_index,
            LMP_USE_SEMI_PERMANENT_KEY_OPCODE, tid);
    bz_auth_perform_super_state_transition(auth,
            HCI_MLK_SEMI_PER_OR_LMP_SEMI_PER_KEY);
    bz_auth_handle_auth_completion(ce_index, HCI_COMMAND_SUCCEEDED);
}

/**
 * Handles lmp_not_accepted(lmp_temp_rand, lmp_temp_key) PDU
 * from the remote device.
 *
 * \param lmp_pdu_ptr LMP PDU packet from the remote device.
 * \param ce_index ACL Connection entity index.
 * \param can_free_pdu Set to TRUE, if the PDU can be freed by the caller.
 *                     FALSE, otherwise.
 *
 * \return None.
 */
void bz_auth_handle_temp_pdu_not_accepted_pdu(LMP_PDU_PKT *lmp_pdu_ptr,
        UINT16 ce_index, OUT BOOLEAN* can_free_pdu)
{
    LMP_CONNECTION_ENTITY* ce_ptr;
    BZ_AUTH_LINK_PARAMS* auth;
    UCHAR lmp_reason;

    ce_ptr = &lmp_connection_entity[ce_index];
    auth = ce_ptr->auth;
    lmp_reason = lmp_pdu_ptr->payload_content[2];

    switch (auth->sub_state)
    {
#ifdef SECURE_CONN_MUTUAL_AUTH
        case SECURE_CONN_MUTUAL_SEND_AU_RAND:
#endif
        case INTR_CHALLENGED_REMOTE_HOST_MUTUAL:
            bz_auth_return_if_remote_initiated_transaction;
            break;
        default:
            BZ_ASSERT(0, "Either remote device is misbehaving or I "
                    "screwed up the state machine");
            return;
    }
    /* auth_status(failure) */
    bz_auth_handle_auth_completion(ce_index, lmp_reason);
}

/**
 * Handles lmp_not_accepted(lmp_use_semi_permanent_key) PDU
 * from the remote device.
 *
 * \param lmp_pdu_ptr LMP PDU packet from the remote device.
 * \param ce_index ACL Connection entity index.
 * \param can_free_pdu Set to TRUE, if the PDU can be freed by the caller.
 *                     FALSE, otherwise.
 *
 * \return None.
 */
void bz_auth_handle_use_semi_permanent_key_not_accepted_pdu(
        LMP_PDU_PKT *lmp_pdu_ptr, UINT16 ce_index, OUT BOOLEAN* can_free_pdu)
{
    LMP_CONNECTION_ENTITY* ce_ptr;
    BZ_AUTH_LINK_PARAMS* auth;
    UCHAR lmp_reason;

    ce_ptr = &lmp_connection_entity[ce_index];
    auth = ce_ptr->auth;
    lmp_reason = lmp_pdu_ptr->payload_content[2];

    switch (auth->sub_state)
    {
        case USE_SEMI_PER_KEY_PDU_SENT:
            bz_auth_return_if_remote_initiated_transaction;
            break;
        default:
            BZ_ASSERT(0, "Either remote device is misbehaving or I "
                    "screwed up the state machine");
            return;
    }
    /* auth_status(failure) */
    bz_auth_handle_auth_completion(ce_index, lmp_reason);
}
#endif /* COMPILE_BROADCAST_ENCRYPTION */

