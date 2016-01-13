/***************************************************************************
 Copyright (C) Mindtree Ltd.
 This module is a confidential and proprietary property of MindTree and
 a possession or use of this module requires written permission of MindTree.
 ***************************************************************************/
/********************************* Logger *************************/ 
enum { __FILE_NUM__= 18 };
/********************************* Logger *************************/

/**
 * \file bz_auth_hci.c
 *  BlueWiz Authentication module 2.0 HCI interface implementation. It
 *  contains the handlers for 2.0 authentication related HCI commands and
 *  event generators.
 *
 * \author Santhosh kumar M
 * \date 2007-08-10
 */

/* ========================= Include File Section ========================= */
#include "bz_auth.h"
#include "lmp.h"
#include "bt_fw_hci.h"
#include "bt_fw_hci_internal.h"
#include "bz_auth_internal.h"
#include "bz_auth_internal_2_1.h"
#include "bz_auth_extern_accessors.h"
#include "bz_debug.h"
#include "bz_auth_lkdb.h"
#include "bz_auth_hci.h"
#ifdef COMPILE_BROADCAST_ENCRYPTION
#include "bz_auth_mlk.h"
#endif
#include "crypto11.h"
#include "mem.h"

#include "logger.h"

/* ====================== Macro Declaration Section ======================= */


/* ==================== Structure Declaration Section ===================== */


/* ===================== Variable Declaration Section ===================== */


/* ================== Static Function Prototypes Section ================== */


/* ===================== Function Definition Section ====================== */
/**
 * Validates the give ACL connection handle and returns the corresponding ACL
 * ce_index.
 *
 * \param hci_cmd_ptr HCI Command packet containing the connection handle as
 *                    the first parameter.
 * \param pce_index Pointer to store the returned ACL ce_index.
 *
 * \return TRUE, if the ACL connection handle is valid. FALSE, otherwise.
 */
BOOLEAN bz_auth_validate_conn_handle(HCI_CMD_PKT* hci_cmd_ptr,
        OUT UINT16 *pce_index)
{
    UINT16 conn_handle;

    BT_FW_EXTRACT_16_BITS(conn_handle, &hci_cmd_ptr->cmd_parameter[0]);

    if (LMP_GET_CE_INDEX_FROM_CONN_HANDLE(conn_handle, pce_index)
            == API_SUCCESS)
    {
        return TRUE;
    }
    else
    {
        return FALSE;
    }
}

/**
 * Validates the give BD Address and returns the corresponding ACL
 * ce_index. Here, the BD address is checked to see if there is any ACL
 * connection is associated with the given BD address.
 *
 * \param hci_cmd_ptr HCI Command packet containing the BD address as
 *                    the first parameter.
 * \param pce_index Pointer to store the returned ACL ce_index.
 *
 * \return TRUE, if the BD address is valid. FALSE, otherwise.
 */
BOOLEAN bz_auth_validate_bd_addr(HCI_CMD_PKT* hci_cmd_ptr,
        OUT UINT16 *pce_index)
{
    if (LMP_GET_CE_INDEX_FROM_BD_ADDR(&hci_cmd_ptr->cmd_parameter[0],
            pce_index) == API_SUCCESS)
    {
        return TRUE;
    }
    else
    {
        return FALSE;
    }
}

/**
 * Generates the link key request event to the host.
 *
 * \param ce_index ACL connection entity index.
 *
 * \return None.
 */
void bz_auth_generate_link_key_request_event(UINT16 ce_index)
{
    UCHAR event_parameter[LMP_BD_ADDR_SIZE];
    memcpy(&event_parameter[0], lmp_connection_entity[ce_index].bd_addr,
            LMP_BD_ADDR_SIZE);
    if (!hci_generate_event(HCI_LINK_KEY_REQUEST_EVENT, event_parameter,
            LMP_BD_ADDR_SIZE))
    {
        hci_send_host_event_masked_signal(ce_index, HCI_LINK_KEY_REQUEST_EVENT);
    }
}

/**
 * Generates the pin code request event to the host.
 *
 * \param ce_index ACL connection entity index.
 *
 * \return None.
 */
void bz_auth_generate_pin_code_request_event(UINT16 ce_index)
{
    UCHAR event_parameter[LMP_BD_ADDR_SIZE];
    memcpy(&event_parameter[0], lmp_connection_entity[ce_index].bd_addr,
            LMP_BD_ADDR_SIZE);
    if (!hci_generate_event(HCI_PIN_CODE_REQUEST_EVENT, event_parameter,
            LMP_BD_ADDR_SIZE))
    {
        hci_send_host_event_masked_signal(ce_index, HCI_PIN_CODE_REQUEST_EVENT);
    }
}

/**
 * Generates authentication complete event to the host.
 *
 * \param ce_index ACL connection entity index.
 * \param status Status of the authentication procedure. It must be a valid
 *               bluetooth error code.
 *
 * \return None.
 */
void bz_auth_generate_authentication_complete_event(UINT16 ce_index,
        UCHAR status)
{
    UCHAR event_parameter[3];
    UINT16 conn_handle;

    conn_handle = lmp_connection_entity[ce_index].connection_type.connection_handle;

    event_parameter[0] = status;
    event_parameter[1] = (UCHAR)conn_handle;
    event_parameter[2] = (UCHAR)(conn_handle>>8);
    hci_generate_event(HCI_AUTHENTICATION_COMPLETE_EVENT, event_parameter, 3);
}

/**
 * Generates the link key notification event to the host.
 *
 * \param ce_index ACL connection entity index.
 *
 * \return TRUE, if the operation was successful. FALSE, otherwise.
 */
void bz_auth_generate_link_key_notification_event(UINT16 ce_index)
{
    UCHAR event_parameter[LMP_BD_ADDR_SIZE+BZ_AUTH_LINK_KEY_SIZE+1];
    BZ_AUTH_LINK_PARAMS* auth;

    auth = lmp_connection_entity[ce_index].auth;
    memcpy(&event_parameter[0], lmp_connection_entity[ce_index].bd_addr,
            LMP_BD_ADDR_SIZE);
    memcpy(&event_parameter[LMP_BD_ADDR_SIZE], auth->txn_params.link_key,
            BZ_AUTH_LINK_KEY_SIZE);
    event_parameter[LMP_BD_ADDR_SIZE+BZ_AUTH_LINK_KEY_SIZE] =
        (UCHAR)auth->txn_params.lk_type;
    hci_generate_event(HCI_LINK_KEY_NOTIFICATION_EVENT, event_parameter,
            LMP_BD_ADDR_SIZE+BZ_AUTH_LINK_KEY_SIZE+1);

    RT_BT_LOG(GREEN, HCI_MSG_LINK_LEY_NOTI_EVENT_MAC_ADDR, 8, 
             ce_index, lmp_connection_entity[ce_index].bd_addr[5], 
             lmp_connection_entity[ce_index].bd_addr[4],
             lmp_connection_entity[ce_index].bd_addr[3],
             lmp_connection_entity[ce_index].bd_addr[2],
             lmp_connection_entity[ce_index].bd_addr[1],
             lmp_connection_entity[ce_index].bd_addr[0], 
             auth->txn_params.lk_type);
 
    RT_BT_LOG(GREEN, HCI_MSG_LINK_LEY_NOTI_EVENT_KEY_CONTENT, 16, 
            auth->txn_params.link_key[15], auth->txn_params.link_key[14],
            auth->txn_params.link_key[13], auth->txn_params.link_key[12],
            auth->txn_params.link_key[11], auth->txn_params.link_key[10],
            auth->txn_params.link_key[9], auth->txn_params.link_key[8],
            auth->txn_params.link_key[7], auth->txn_params.link_key[6],
            auth->txn_params.link_key[5], auth->txn_params.link_key[4],
            auth->txn_params.link_key[3], auth->txn_params.link_key[2],
            auth->txn_params.link_key[1], auth->txn_params.link_key[0]);
}

/**
 * Generates change connection link key  complete event to the host.
 *
 * \param ce_index ACL connection entity index.
 * \param status Status of the change connection link key procedure. It must be
 *               a valid bluetooth error code.
 *
 * \return None.
 */
void bz_auth_generate_change_connection_link_key_event(UINT16 ce_index,
        UCHAR status)
{
    UCHAR event_parameter[3];
    UINT16 conn_handle;

    conn_handle = lmp_connection_entity[ce_index].connection_type.connection_handle;

    event_parameter[0] = status;
    event_parameter[1] = (UCHAR)conn_handle;
    event_parameter[2] = (UCHAR)(conn_handle>>8);
    hci_generate_event(HCI_CHANGE_CONNECTION_LINK_KEY_COMPLETE_EVENT,
            event_parameter, 3);
}

/**
 * Generates encryption change complete event to the host.
 *
 * \param ce_index ACL connection entity index.
 * \param status Status of the set connection encryption procedure. It must be
 *               a valid bluetooth error code.
 * \param enc_mode Encryption mode of the ACL connection after the set
 *                 connection encryption procedure completion.
 *
 * \return None.
 */
void bz_auth_generate_encryption_change_event(UINT16 ce_index,
        UINT8 status, UINT8 enc_mode)
{
    UCHAR event_parameter[4];
    UINT16 conn_handle;

    conn_handle = lmp_connection_entity[ce_index].connection_type.connection_handle;

    event_parameter[0] = status;
    event_parameter[1] = (UCHAR)conn_handle;
    event_parameter[2] = (UCHAR)(conn_handle>>8);
    event_parameter[3] = (UCHAR)enc_mode;
    hci_generate_event(HCI_ENCRYPTION_CHANGE_EVENT, event_parameter, 4);
}

/**
 * Generates the return link keys event to the host.
 *
 * \param bd_addr BD addresses to be included in the event packet.
 * \param keys Link keys associated with the BD addresses.
 * \param nkeys Number of keys to be included in this event packet.
 *
 * \return None.
 */
void bz_auth_generate_return_link_keys_event(
        const UCHAR* bd_addr[BZ_AUTH_LKDB_MAX_KEYS],
        const UCHAR* keys[BZ_AUTH_LKDB_MAX_KEYS], UCHAR nkeys)
{
    UCHAR event_parameter[HCI_MAX_EVENT_PARAM_LEN];
    UCHAR param_len = 1;
    register int i;

    BZ_ASSERT((nkeys * (16+6)) < HCI_MAX_EVENT_PARAM_LEN,
            "Single event packet can not hold all the link keys, the caller "
            "has to take care about it");

    if (nkeys == 0)     /* No keys to return to host */
    {
        return;
    }

    event_parameter[0] = nkeys;     /* Num_Keys */
    for (i = 0; i < nkeys; i++)
    {
        memcpy(&event_parameter[param_len], bd_addr[i], LMP_BD_ADDR_SIZE);
        param_len = (UCHAR)(param_len + LMP_BD_ADDR_SIZE);
        memset(&event_parameter[param_len], 0, BZ_AUTH_LINK_KEY_SIZE);
        param_len = (UCHAR)(param_len + BZ_AUTH_LINK_KEY_SIZE);
    }
    hci_generate_event(HCI_RETURN_LINK_KEYS_EVENT, event_parameter, param_len);
}

/**
 * Handles generation of command complete event for security related commands
 * given by \a hci_cmd_ptr. It  internally fills the command complete event
 * parameters specific to \a hci_cmd_ptr. It also takes care of filling the
 * right number of available command buffers.
 *
 * \param hci_cmd_ptr HCI command packet for which Command_Complete has to be
 *                    generated.
 * \param status A Valid bluetooth error code to indicate the status to the
 *               host. It should be set to HCI_COMMAND_SUCCEEDED when the
 *               actual command is handled in command_complete event
 *               generation handler (ie. this function).
 * \param ce_index Connection entity index related to the command. It may be
 *                 INVALID_CE_INDEX, when the \a status indicates failure.
 *
 * \return None.
 *
 * \note Command complete event can not be masked, so it need not return
 *       SUCCESS or FAILURE.
 */
void bz_auth_generate_command_complete_event(HCI_CMD_PKT* hci_cmd_ptr,
        UCHAR status, UINT16 ce_index)
{
    UCHAR event_parameter[HCI_MAX_EVENT_PARAM_LEN];
    LMP_CONNECTION_ENTITY* ce_ptr;
    UCHAR* cmd_parameter;
    UINT16 cmd_opcode;
    UCHAR param_len = 4;    /* all the cmd_completes have at least 4 bytes */

    cmd_opcode = hci_cmd_ptr->cmd_opcode;
    cmd_parameter = hci_cmd_ptr->cmd_parameter;
    if (ce_index != INVALID_CE_INDEX)
    {
        ce_ptr = &lmp_connection_entity[ce_index];
    }
    else
    {
        ce_ptr = NULL;
    }

    /* set up the global event parameter */
    event_parameter[0] = (UCHAR)bz_auth_get_num_available_cmd_buffers();
    event_parameter[1] = LSB(cmd_opcode);
    event_parameter[2] = MSB(cmd_opcode);
    event_parameter[3] = status;  /* all cmd completes have status */
    switch (cmd_opcode)
    {
        case HCI_LINK_KEY_REQUEST_REPLY_OPCODE:
        case HCI_LINK_KEY_REQUEST_NEGATIVE_REPLY_OPCODE:
        case HCI_PIN_CODE_REQUEST_REPLY_OPCODE:
        case HCI_PIN_CODE_REQUEST_NEGATIVE_REPLY_OPCODE:
        case HCI_IO_CAPABILITY_REQUEST_REPLY_OPCODE:
        case HCI_IO_CAPABILITY_REQUEST_NEGATIVE_REPLY_OPCODE:
        case HCI_USER_CONFIRMATION_REQUEST_REPLY_OPCODE:
        case HCI_USER_CONFIRMATION_REQUEST_NEGATIVE_REPLY_OPCODE:
        case HCI_USER_PASSKEY_REQUEST_REPLY_OPCODE:
        case HCI_USER_PASSKEY_REQUEST_NEGATIVE_REPLY_OPCODE:
        case HCI_REMOTE_OOB_DATA_REQUEST_REPLY_OPCODE:
        case HCI_REMOTE_OOB_DATA_REQUEST_NEGATIVE_REPLY_OPCODE:
        case HCI_SEND_KEYPRESS_NOTIFICATION_OPCODE:
            if (ce_ptr)
            {
                memcpy(&event_parameter[4], bz_auth_get_remote_bd_addr(ce_ptr),
                        LMP_BD_ADDR_SIZE);
            }
            else
            {
                /* Fill the event param with the bd addr from hci command. */
                memcpy(&event_parameter[4], &hci_cmd_ptr->cmd_parameter[0],
                        LMP_BD_ADDR_SIZE);
            }
            param_len = LMP_BD_ADDR_SIZE+4 /* the common ones */;
            break;

        case HCI_READ_AUTHENTICATION_ENABLE_OPCODE:
            event_parameter[4] = bz_auth_dev_params.auth_enable;
            param_len = 5;
            break;

        case HCI_WRITE_SIMPLE_PAIRING_MODE_OPCODE:
            if (cmd_parameter[0] == 0x01)   /* Host can't disable it */
            {
                if (bz_auth_any_connections_exist())
                {
                    event_parameter[3] = COMMAND_DISALLOWED_ERROR;
                }
                else
                {
                    bz_auth_dev_params.ssp_mode = cmd_parameter[0];
                    bz_auth_enable_ssp_ext_feature_bits();
                }
            }
            else
            {
                event_parameter[3] = INVALID_HCI_COMMAND_PARAMETERS_ERROR;
            }
            break;

        case HCI_READ_SIMPLE_PAIRING_MODE_OPCODE:
            event_parameter[4] = (UCHAR)bz_auth_dev_params.ssp_mode;
            param_len = 5;
            break;

        case HCI_WRITE_SIMPLE_PAIRING_DEBUG_MODE_OPCODE:
            if (cmd_parameter[0] < 2)
            {
                bz_auth_dev_params.ssp_debug_mode = cmd_parameter[0];
            }
            else
            {
                event_parameter[3] = INVALID_HCI_COMMAND_PARAMETERS_ERROR;
            }
            break;

        case HCI_READ_LOCAL_OOB_DATA_OPCODE:
            /* Every time we need to freshly generate the Simple_Pairing_Hash_C
             * and Simple_Pairing_Randomizer_R.
             */
            bz_auth_generate_oob_data();
            memcpy(&event_parameter[4],
                    &bz_auth_dev_params.last_generated_oob_data.C[0],
                    BZ_AUTH_CONFIRM_VALUE_SIZE);
            memcpy(&event_parameter[4+BZ_AUTH_CONFIRM_VALUE_SIZE],
                    &bz_auth_dev_params.last_generated_oob_data.R[0],
                    BZ_AUTH_OOB_SECRET_NUMBER_SIZE);
            /* Convert the Hash and Randomizer values to LSB format... All the
             * HCI layer communication is through LSB format only. But our
             * internal cryptographic functions uses MSB format.
             */
            bz_auth_convert_to_lsb(&event_parameter[4],
                    BZ_AUTH_CONFIRM_VALUE_SIZE);
            bz_auth_convert_to_lsb(&event_parameter[4+16],
                    BZ_AUTH_OOB_SECRET_NUMBER_SIZE);
            param_len = 36;
            break;

        case HCI_WRITE_AUTHENTICATION_ENABLE_OPCODE:
            if (cmd_parameter[0] < 2)
            {
                bz_auth_dev_params.auth_enable = cmd_parameter[0];
            }
            else
            {
                event_parameter[3] = INVALID_HCI_COMMAND_PARAMETERS_ERROR;
            }
            break;

#ifdef COMPILE_DEPRECATED_COMMANDS
		case HCI_READ_ENCRYPTION_MODE_OPCODE:
            event_parameter[4] = bz_auth_dev_params.enc_enable;
            param_len = 5;
            break;

        case HCI_WRITE_ENCRYPTION_MODE_OPCODE:
            if (cmd_parameter[0] < 2)
            {
                bz_auth_dev_params.enc_enable = cmd_parameter[0];
            }
            else
            {
                event_parameter[3] = INVALID_HCI_COMMAND_PARAMETERS_ERROR;
            }
            break;
#endif

        case HCI_READ_PIN_TYPE_OPCODE:
            event_parameter[4] = (UCHAR)bz_auth_dev_params.pin_type;
            param_len = 5;
            break;

        case HCI_WRITE_PIN_TYPE_OPCODE:
            if (cmd_parameter[0] < 2)
            {
                bz_auth_dev_params.pin_type = cmd_parameter[0];
            }
            else
            {
                event_parameter[3] = INVALID_HCI_COMMAND_PARAMETERS_ERROR;
            }
            break;

        case HCI_READ_STORED_LINK_KEY_OPCODE:
        {
            const UCHAR* bd_addr[BZ_AUTH_LKDB_MAX_KEYS];
            const UCHAR* keys[BZ_AUTH_LKDB_MAX_KEYS];
            UCHAR nkeys = 0;

            if (cmd_parameter[LMP_BD_ADDR_SIZE] > 0x1)
            {
                event_parameter[3] = INVALID_HCI_COMMAND_PARAMETERS_ERROR;
                break;
            }
            if (cmd_parameter[LMP_BD_ADDR_SIZE])    /* read_all_flag? */
            {
                nkeys = bz_auth_lkdb_get_all_keys(bd_addr, keys);
            }
            else if (bz_auth_lkdb_get_key(&cmd_parameter[0] /* bd_addr */,
                        &keys[0]))  /* returns key if found */
            {
                nkeys = 1;
                bd_addr[0] = &cmd_parameter[0];         /* set bd_addr */
            }
            bz_auth_generate_return_link_keys_event(bd_addr, keys, nkeys);
            /* The above event generation overwrites the global
             * "event_parameter" strcture. So we need to set it up again for
             * generating command_complete_event.
             */
            event_parameter[0] = (UCHAR)bz_auth_get_num_available_cmd_buffers();
            event_parameter[1] = LSB(cmd_opcode);
            event_parameter[2] = MSB(cmd_opcode);
            event_parameter[3] = status;  /* all cmd completes have status */
            event_parameter[4] = BZ_AUTH_LKDB_MAX_KEYS; /* Max_Num_Keys(2) */
            event_parameter[5] = 0;
            event_parameter[6] = nkeys;         /* Num_Keys_Read (2 octets) */
            event_parameter[7] = 0;
            param_len = 8;
            break;
        }
        case HCI_WRITE_STORED_LINK_KEY_OPCODE:
        {
            BZ_AUTH_LKDB_KEYS_T* keys;
            UCHAR nkeys;

            nkeys = cmd_parameter[0];       /* Num_Keys_To_Write */
            if((nkeys < 0x1)
                    || (nkeys > 0x0B))
            {
                event_parameter[3] = INVALID_HCI_COMMAND_PARAMETERS_ERROR;
                break;
            }
            keys = (BZ_AUTH_LKDB_KEYS_T*)&cmd_parameter[1];
            nkeys = bz_auth_lkdb_put_keys(keys, nkeys);

            event_parameter[4] = nkeys;     /* Num_Keys_Written (1 octet) */
            param_len = 5;
            break;
        }
        case HCI_DELETE_STORED_LINK_KEY_OPCODE:
        {
            UCHAR nkeys = 0;

            if (cmd_parameter[LMP_BD_ADDR_SIZE] > 0x1)
            {
                event_parameter[3] = INVALID_HCI_COMMAND_PARAMETERS_ERROR;
                break;
            }
            if (cmd_parameter[LMP_BD_ADDR_SIZE])    /* delete_all_flag? */
            {
                nkeys = bz_auth_lkdb_del_all_keys();
            }
            else if (bz_auth_lkdb_del_key(&cmd_parameter[0]/* bd_addr */))
            {
                nkeys = 1;
            }
            event_parameter[4] = nkeys;     /* Num_Keys_Deleted (2 octets) */
            event_parameter[5] = 0;
            param_len = 6;
            break;
        }

        case HCI_CREATE_NEW_UNIT_KEY_OPCODE:
        {
            UCHAR rand[16];

            ssp_rng_get(&rand[0]);
            lp_E21(rand, bz_auth_get_local_bd_addr(),
                    &bz_auth_dev_params.unit_key[0]);
            break;
        }
        default:
            BZ_ASSERT(0, "I don't know how to generate cmd complete for this "
                    "cmd_opcode");
            return;
    }

    hci_generate_event(HCI_COMMAND_COMPLETE_EVENT, event_parameter, param_len);
}

/**
 * Handles the HCI authentication requested command.
 *
 * \param hci_cmd_ptr HCI command packet.
 *
 * \return HCI_COMMAND_SUCCEEDED, if the operation is successfully initiated.
 *         Otherwise, a valid bluetooth error code.
 */
UCHAR bz_auth_handle_authentication_requested(HCI_CMD_PKT* hci_cmd_ptr)
{
    UINT16 ce_index;
    LMP_CONNECTION_ENTITY* ce_ptr;
    BZ_AUTH_LINK_PARAMS* auth;

    if (!bz_auth_validate_conn_handle(hci_cmd_ptr, &ce_index))
    {
        return NO_CONNECTION_ERROR;
    }

    hci_generate_command_status_event(hci_cmd_ptr->cmd_opcode,
            HCI_COMMAND_SUCCEEDED);

    ce_ptr = &lmp_connection_entity[ce_index];
    auth = ce_ptr->auth;
    switch (auth->super_state)
    {
        case UNAUTHENTICATED:
        case AUTHENTICATED:
        case ENCRYPTED:
            auth->txn_params.auth_role = BZ_AUTH_ROLE_INITIATOR;
            bz_auth_enter_link_key_requested_state(ce_index, auth,
                    INTR_LINK_KEY_REQUESTED);
            break;

        case TEMP_AUTHENTICATED:
        case TEMP_ENCRYPTED:
            auth->txn_params.auth_role = BZ_AUTH_ROLE_INITIATOR;
            /* set the link key in transaction params */
            memcpy(&auth->txn_params.link_key[0], auth->link_key,
                    BZ_AUTH_LINK_KEY_SIZE);
            /* We need not have to copy lk_type, because we won't be upgrading
             * key when TEMP_KEY in use. But we have to copy the ACO value
             * generated during authentication.
             */
            /* send lmp_au_rand */
            bz_auth_send_au_rand_pdu(ce_index, auth, SELF_DEV_TID);
            bz_auth_transition_to_sub_state(auth, INTR_CHALLENGED_REMOTE_HOST);
            break;

        default:
            bz_auth_generate_authentication_complete_event(ce_index,
                    COMMAND_DISALLOWED_ERROR);
            return HCI_COMMAND_SUCCEEDED;
    }

    bz_auth_perform_super_state_transition(auth,
            HCI_AUTH_REQ_OR_AURAND_INRAND_OR_IOCAP_REQ_AUTH_ENABLE);

    return HCI_COMMAND_SUCCEEDED;
}

/**
 * Performs the [INTR/RESP]_PIN_CODE_REQUESTED entry action and transitions to
 * [INTR/RESP]_PIN_CODE_REQUESTED sub state.
 *
 * \param ce_index ACL Connection entity index.
 * \param auth Authentication parameters related to \a ce_index.
 * \param sub_state INTR_PIN_CODE_REQUESTED or RESP_PIN_CODE_REQUESTED.
 *
 * \return None.
 */
void enter_pin_code_requested_state(UINT16 ce_index,
        BZ_AUTH_LINK_PARAMS* auth, UINT8 sub_state)
{
    bz_auth_generate_pin_code_request_event(ce_index);
    bz_auth_transition_to_sub_state(auth, sub_state);
}

/**
 * Handles the HCI change connection link key command.
 *
 * \param hci_cmd_ptr HCI command packet.
 *
 * \return HCI_COMMAND_SUCCEEDED, if the operation is successfully initiated.
 *         Otherwise, a valid bluetooth error code.
 */
UCHAR bz_auth_handle_change_connection_link_key(HCI_CMD_PKT* hci_cmd_ptr)
{
    UINT16 ce_index;
    LMP_CONNECTION_ENTITY* ce_ptr;
    BZ_AUTH_LINK_PARAMS* auth;

    if (!bz_auth_validate_conn_handle(hci_cmd_ptr, &ce_index))
    {
        return NO_CONNECTION_ERROR;
    }

    hci_generate_command_status_event(hci_cmd_ptr->cmd_opcode,
            HCI_COMMAND_SUCCEEDED);

    /* hurray we have passed the sanity checks, lets proceed :) */
    ce_ptr = &lmp_connection_entity[ce_index];
    auth = ce_ptr->auth;


#ifdef _CCH_SC_TEST_20130129_MRV_01
    ce_ptr->done_h3 = 0;
#ifdef _SECURE_CONN_TEST_LOG        
    RT_BT_LOG(YELLOW, DAPE_TEST_LOG522, 2,auth->secure_conn_enabled, ce_ptr->done_h3);
#endif
#endif

	
    switch (auth->super_state)
    {
        case AUTHENTICATED:
        case ENCRYPTED:
            auth->txn_params.auth_role = BZ_AUTH_ROLE_INITIATOR;
            /* Is the ACL link currently using unit key? */
            if (bz_auth_is_link_using_unit_key(auth))
            {
                /* We got to pair again to change the link key */
                enter_pin_code_requested_state(ce_index, auth,
                        INTR_PIN_CODE_REQUESTED);
            }
            else
            {
                /* we are using combination key, so let's change the key in
                 * the normal way.
                 */
                /* Initialize the transaction parameters: here init_key reqd */
                memcpy(&auth->txn_params.init_key[0], auth->link_key,
                        BZ_AUTH_LINK_KEY_SIZE);

                /* Send comb_key pdu (here unit_key will not be sent) and
                 * compute local comb_key contribution and store it. Also
                 * transition to corresponding sub_state.
                 */
                bz_auth_intr_send_unit_or_comb_key(ce_index, auth);
                bz_auth_transition_to_sub_state(auth,
                        INTR_CHANGING_KEY_SENT_COMB_KEY);
            }
            break;

        default:
            bz_auth_generate_change_connection_link_key_event(ce_index,
                    COMMAND_DISALLOWED_ERROR);
            return HCI_COMMAND_SUCCEEDED;
    }

    bz_auth_perform_super_state_transition(auth,
            HCI_CHANGE_CONN_LINK_KEY_LMP_UNIT_COMB_KEY);

    return HCI_COMMAND_SUCCEEDED;
}

/**
 * Handles the HCI set connection encryption command.
 *
 * \param hci_cmd_ptr HCI command packet.
 *
 * \return HCI_COMMAND_SUCCEEDED, if the operation is successfully initiated.
 *         Otherwise, a valid bluetooth error code.
 */
UCHAR bz_auth_handle_set_connection_encryption(HCI_CMD_PKT* hci_cmd_ptr)
{
    UINT16 ce_index;
    BZ_AUTH_LINK_PARAMS* auth;
    UINT8 enc_enable;

    if (!bz_auth_validate_conn_handle(hci_cmd_ptr, &ce_index))
    {
        return NO_CONNECTION_ERROR;
    }
    auth = lmp_connection_entity[ce_index].auth;
    enc_enable = hci_cmd_ptr->cmd_parameter[2];
#ifndef _TEST_NO_REFUSE_ENC_OFF    
#ifdef _SUPPORT_SECURE_CONNECTION_
    if ((auth->secure_conn_enabled) && (enc_enable == 0))
    {
        return ENCRYPTION_MODE_NOT_ACCEPTABLE_ERROR;
    }
#endif
#endif
    hci_generate_command_status_event(hci_cmd_ptr->cmd_opcode,
            HCI_COMMAND_SUCCEEDED);

    /* hurray we have passed the sanity checks, lets proceed :) */
    if (enc_enable > 1)
    {
        bz_auth_generate_encryption_change_event(ce_index,
                INVALID_HCI_COMMAND_PARAMETERS_ERROR, enc_enable);
        return HCI_COMMAND_SUCCEEDED;
    }

    switch (auth->super_state)
    {
        case AUTHENTICATED:
        case TEMP_AUTHENTICATED:
            if (!enc_enable)
            {
                /* enc_change_event (success) */
                bz_auth_generate_encryption_change_event(ce_index,
                        HCI_COMMAND_SUCCEEDED, enc_enable);
                return HCI_COMMAND_SUCCEEDED;
            }
            auth->txn_params.auth_role = BZ_AUTH_ROLE_INITIATOR;
            /* send enc_mode_req(enable) and transition to appropriate
             * sub_state.
             */
            bz_auth_send_encryption_mode_req_pdu(ce_index, auth, SELF_DEV_TID,
                    BZ_AUTH_ENCRYPTION_MODE_ON);
            break;

        case ENCRYPTED:
        case TEMP_ENCRYPTED:
            if (enc_enable)
            {
                /* enc_change_event (success) */
                bz_auth_generate_encryption_change_event(ce_index,
                        HCI_COMMAND_SUCCEEDED, enc_enable);
                return HCI_COMMAND_SUCCEEDED;
            }
            auth->txn_params.auth_role = BZ_AUTH_ROLE_INITIATOR;
            /* send enc_mode_req(disable) and transition to appropriate
             * sub_state.
             */
            bz_auth_send_encryption_mode_req_pdu(ce_index, auth, SELF_DEV_TID,
                    BZ_AUTH_ENCRYPTION_MODE_OFF);
            break;

        default:
            bz_auth_generate_encryption_change_event(ce_index,
                    COMMAND_DISALLOWED_ERROR, enc_enable);
            return HCI_COMMAND_SUCCEEDED;
    }

    bz_auth_perform_super_state_transition(auth,
            HCI_SET_ENC_OR_ENC_REQ_OR_AUTH_SUCC_ENC_ENABLED);

    return HCI_COMMAND_SUCCEEDED;
}

/**
 * Returns whether link key from the host, at this point, is expected or not.
 *
 * \param auth Authentication parameters of the ACL link.
 *
 * \return TRUE, if the link key is expected. FALSE, otherwise.
 */
BOOLEAN bz_auth_is_link_key_expected(BZ_AUTH_LINK_PARAMS* auth)
{
    return (BOOLEAN)(auth->sub_state == INTR_LINK_KEY_REQUESTED
            || auth->sub_state == RESP_LINK_KEY_REQUESTED);
}

/**
 * Perform the basic sanity checks on the common parameters of *_reply
 * commands from the host.
 *
 * \param hci_cmd_ptr HCI Command packet pointer corresponding to *_replay
 *                    command.
 * \param is_the_cmd_expected Function which returns TRUE, if the command
 *                            given by \a hci_cmd_ptr is expected now. FALSE,
 *                            otherwise.
 * \param pce_index The output connection entity index.
 * \param pce_ptr The output connection entity pointer.
 *
 * \return HCI_COMMAND_SUCCEEDED, if the sanity checks passed. Otherwise,
 *         (VALID_ERROR_CODE | ZERO_BYTE). The VALID_ERROR_CODE indicates
 *         the sanity check failure reason.
 */
UINT16 perform_sanity_checks_for_reply_commands(HCI_CMD_PKT* hci_cmd_ptr,
        BOOLEAN (*is_the_cmd_expected)(BZ_AUTH_LINK_PARAMS*),
        BOOLEAN (*is_valid_params)(HCI_CMD_PKT*),
        OUT UINT16* pce_index, OUT LMP_CONNECTION_ENTITY** pce_ptr)
{
    BZ_AUTH_LINK_PARAMS* auth;
    UCHAR status;

    /* validate basic parameters necessary to process this command */
    if (!bz_auth_validate_bd_addr(hci_cmd_ptr, pce_index))
    {
        status = NO_CONNECTION_ERROR;
        *pce_index = INVALID_CE_INDEX;
    }
    else
    {
        /* done with sanity checks, now process reply command */
        *pce_ptr = &lmp_connection_entity[*pce_index];
        auth = (*pce_ptr)->auth;

        /* do we expect this command at this point */
        if (is_the_cmd_expected(auth))
        {
            status = HCI_COMMAND_SUCCEEDED;
            if ((is_valid_params != NULL) && (!is_valid_params(hci_cmd_ptr)))
            {
                /* Wrong hci parameters, the host is trying ruin our
                 * life. I am intelligent.. hahaha :-D
                 */
                status = INVALID_HCI_COMMAND_PARAMETERS_ERROR;
            }
        }
        else
        {
            /* we don't expect the command now, the host is trying ruin our
             * life. I am intelligent.. hahaha :-D
             */
            status = COMMAND_DISALLOWED_ERROR;
        }
    }

    bz_auth_generate_command_complete_event(hci_cmd_ptr, status,
            *pce_index);

    return (UINT16)(status | HCI_COMMAND_SUCCEEDED);
}

/**
 * Performs the link key request reply action. It is called after all the link
 * key request reply command parameters are validated.
 *
 * \param ce_index ACL Connection entity index.
 * \param ce_ptr ACL Connection entity pointer.
 * \param auth Authentication parameter associated with the \a ce_index.
 * \param link_key Link key from the host.
 *
 * \return HCI_COMMAND_SUCCEEDED, if the operation is successfully initiated.
 *         Otherwise, a valid bluetooth error code.
 */
UCHAR bz_auth_perform_link_key_request_reply_action(UINT16 ce_index,
        LMP_CONNECTION_ENTITY* ce_ptr, BZ_AUTH_LINK_PARAMS* auth,
        const UCHAR link_key[16])
{
    /* Received the link key at right time, now store the link kuth_mley
     * temporarily. We initialize the trasaction parameters with the required
     * information. Here link_key and lk_type are required as they will be
     * used by send au_rand/sres functions.
     */
    memcpy(&auth->txn_params.link_key[0], link_key, BZ_AUTH_LINK_KEY_SIZE);
    /* Get the link key type from the "auth" assuming it has valid link key
     * type (It doesn't matter even if it contains wrong type, but we can not
     * afford to loose the right type -- because we copy the whole txn_params
     * on auth_success -- refer bz_auth_update_auth_params_from_txn_params())
     * We need not have to copy ACO, because it will be created everytime we
     * authenticate.
     */
    auth->txn_params.lk_type = auth->lk_type;

    RT_BT_LOG(GREEN, HCI_MSG_LINK_LEY_REQ_REPLY_KET_ENTRY, 20, 
                ce_index, auth->txn_params.lk_type, 
                auth->sub_state, auth->pending_pdu,
                auth->txn_params.link_key[0], auth->txn_params.link_key[1],
                auth->txn_params.link_key[2], auth->txn_params.link_key[3],
                auth->txn_params.link_key[4], auth->txn_params.link_key[5],
                auth->txn_params.link_key[6], auth->txn_params.link_key[7],
                auth->txn_params.link_key[8], auth->txn_params.link_key[9],
                auth->txn_params.link_key[10], auth->txn_params.link_key[11],
                auth->txn_params.link_key[12], auth->txn_params.link_key[13],
                auth->txn_params.link_key[14], auth->txn_params.link_key[15]);

    /* are we responder, then job is pretty simple */
    if (auth->sub_state == RESP_LINK_KEY_REQUESTED)
    {
#ifdef SECURE_CONN_MUTUAL_AUTH
#ifdef _SECURE_CONN_TEST_LOG
#ifdef _CCH_SC_TEST_20130129_MRV_01
RT_BT_LOG(YELLOW, DAPE_TEST_LOG522, 2,auth->secure_conn_enabled, ce_ptr->done_h3);
#else
RT_BT_LOG(YELLOW, DAPE_TEST_LOG522, 2,auth->secure_conn_enabled, auth->len_prime);
#endif
#endif
        if(auth->secure_conn_enabled)
        {
            LMP_TRAN_ID tid;
            tid = (LMP_TRAN_ID)LMP_GET_TRANSACTION_ID(auth->pending_pdu);

            bz_auth_send_au_rand_pdu(ce_index, auth, tid);

            if (!bz_auth_is_master(ce_ptr))
            {
                bz_auth_send_sres_pdu(ce_index, auth, auth->pending_pdu);
                bz_auth_free_pending_pdu(auth);
                bz_auth_transition_to_sub_state(auth, SECURE_CONN_SLV_AWAIT_SRES);
            }
            else
            {
                bz_auth_transition_to_sub_state(auth, SECURE_CONN_MAS_AWAIT_SRES);
            }

        }
        else
#endif
        {
            /* send challenge_response (sres) and free stored au_rand pdu */
            bz_auth_send_sres_pdu(ce_index, auth, auth->pending_pdu);
            bz_auth_free_pending_pdu(auth);
            /* indicate auth_status(success) */
            bz_auth_handle_auth_completion(ce_index, HCI_COMMAND_SUCCEEDED);
        }
        return HCI_COMMAND_SUCCEEDED;
    }

    /* we should be initiator to reach here.
     * don't we have any collision? damn good, we can complete straight away
     */
    if (auth->pending_pdu == NULL)
    {
        /* send lmp_au_rand (our TID) */
        bz_auth_send_au_rand_pdu(ce_index, auth, SELF_DEV_TID);
        bz_auth_transition_to_sub_state(auth, INTR_CHALLENGED_REMOTE_HOST);
        return HCI_COMMAND_SUCCEEDED;
    }

    /* being a initiator and handling collisions is pretty tough -- let's see
     * what we can do with the collided pdu.
     */
    switch (bz_auth_get_pdu_opcode(auth->pending_pdu))
    {
        case LMP_AU_RAND_OPCODE:
            if (bz_auth_is_master(ce_ptr))
            {
                /* initiate our transaction (send lmp_au_rand (our TID)) */
                bz_auth_send_au_rand_pdu(ce_index, auth, SELF_DEV_TID);
                bz_auth_transition_to_sub_state(auth,
                        INTR_CHALLENGED_REMOTE_HOST);
                /* terminate slave's transaction */
                bz_auth_send_not_accepted_pdu(ce_index, LMP_AU_RAND_OPCODE,
                        ((LMP_TRAN_ID)
                            (LMP_GET_TRANSACTION_ID(auth->pending_pdu))),
                        LMP_ERROR_TRANSACTION_COLLISION_ERROR);
                bz_auth_free_pending_pdu(auth);
            }
            else
            {
                auth->txn_params.auth_role =
                    BZ_AUTH_ROLE_INITIATOR_AS_RESPONDER;
                /* send lmp_sres (remote_TID) */
                bz_auth_send_sres_pdu(ce_index, auth, auth->pending_pdu);
                bz_auth_free_pending_pdu(auth);
                bz_auth_handle_auth_completion(ce_index, HCI_COMMAND_SUCCEEDED);
            }
            break;

        case LMP_IN_RAND_OPCODE:
            /* send pincode_request_event to host */
            bz_auth_generate_pin_code_request_event(ce_index);
            bz_auth_transition_to_sub_state(auth, INTR_PIN_CODE_REQUESTED);
            break;

        case LMP_ESCAPE4_OPCODE:
            switch (bz_auth_get_ext_pdu_opcode(auth->pending_pdu))
            {
                case LMP_IO_CAPABILITY_REQ_OPCODE:
                    /* send io_cap_requested_event to host */
                    bz_auth_generate_io_cap_request_event(ce_index);
                    bz_auth_transition_to_sub_state(auth,
                            INTR_IO_CAP_REQUESTED);
                    break;

                default:
                    BZ_ASSERT(0, "We shouldn't have stored this pdu, or "
                            "sombody forgot to make the pending_pdu = NULL");
                    bz_auth_free_pending_pdu(auth);
            }
            break;

        default:
            BZ_ASSERT(0, "We shouldn't have stored this pdu, or sombody "
                    "forgot to make the pending_pdu = NULL");
            bz_auth_free_pending_pdu(auth);
    }

    return HCI_COMMAND_SUCCEEDED;
}

/**
 * Handles link key request reply from the host.
 *
 * \param hci_cmd_ptr HCI Command Packet.
 *
 * \return HCI_COMMAND_SUCCEEDED, if the command was handled. A valid
 *         bluetooth error code, otherwise.
 */
UCHAR bz_auth_handle_link_key_request_reply(HCI_CMD_PKT* hci_cmd_ptr)
{
    UINT16 ce_index;
    LMP_CONNECTION_ENTITY* ce_ptr;
    UINT16 reason;

    reason = perform_sanity_checks_for_reply_commands(hci_cmd_ptr,
            bz_auth_is_link_key_expected, NULL, &ce_index, &ce_ptr);
    if (reason != HCI_COMMAND_SUCCEEDED)
    {
        return HCI_COMMAND_SUCCEEDED; /* Cmd_Status should not be generated */
    }

    return bz_auth_perform_link_key_request_reply_action(ce_index, ce_ptr,
            ce_ptr->auth, &hci_cmd_ptr->cmd_parameter[LMP_BD_ADDR_SIZE]);
}

/**
 * Entry action for link key requested state. It determines whether to
 * generate the link key requested event to host or use the link key stored in
 * the link key database and performs the action based on the outcome.
 *
 * \param ce_index ACL connection entity index.
 * \param auth Authentication parameter associated with the \a ce_index.
 * \param sub_state Sub state to be transitioned to after entering the super
 *        state.
 *
 * \return None.
 */
void bz_auth_enter_link_key_requested_state(UINT16 ce_index,
        BZ_AUTH_LINK_PARAMS* auth, UINT8 sub_state)
{
    LMP_CONNECTION_ENTITY* ce_ptr;
    const UCHAR* link_key;

    bz_auth_transition_to_sub_state(auth, sub_state);
    ce_ptr = &lmp_connection_entity[ce_index];

    /* do we already have the link key in the link key database? */
    if (bz_auth_lkdb_get_key(bz_auth_get_remote_bd_addr(ce_ptr), &link_key))
    {
        /* The host has already given the link key through
         * write_stored_link_key command. So lets use that link_key instead of
         * pestering the HOST for it.
         */

        /* We are going to pretend as if the host issued link_key_req_reply.
         * Anyway its functionally right thing to do.
         */
        bz_auth_perform_link_key_request_reply_action(ce_index, ce_ptr,
                auth, link_key);
    }
    else
    {
        bz_auth_generate_link_key_request_event(ce_index);
    }
}

/**
 * Performs the key missing action. This action has to be performed whenever
 * LMP_au_rand/LMP_in_rand is received but there is no link_key/pin_code either
 * with the host or controller.
 *
 * \param ce_index ACL Connection entity index.
 * \param auth Authentication parameters related to \a ce_index.
 *             auth->pending_pdu should point to the received
 *             LMP_au_rand/LMP_in_rand PDU.
 *
 * \return None.
 */
void perform_key_missing_action(UINT16 ce_index,
        BZ_AUTH_LINK_PARAMS* auth)
{
    LMP_TRAN_ID tid;

    tid = (LMP_TRAN_ID)LMP_GET_TRANSACTION_ID(auth->pending_pdu);
    bz_auth_send_not_accepted_pdu(ce_index,
            (LMP_TRAN_ID) (bz_auth_get_pdu_opcode(auth->pending_pdu)) ,
                tid, KEY_MISSING_ERROR);
    bz_auth_free_pending_pdu(auth);
}

/**
 * Performs link key request negative reply action
 *
 * \param ce_index ACL connection entity index.
 *
 * \return HCI_COMMAND_SUCCEEDED, if the command was handled. A valid
 *         bluetooth error code, otherwise.
 */
UCHAR bz_auth_perform_link_key_request_neg_reply_action(UINT16 ce_index)
{
    LMP_CONNECTION_ENTITY* ce_ptr;
    BZ_AUTH_LINK_PARAMS* auth;

    ce_ptr = &lmp_connection_entity[ce_index];
    auth = ce_ptr->auth;

    /* are we responder, then job is pretty simple */
    if (auth->sub_state == RESP_LINK_KEY_REQUESTED)
    {
        /* send lmp_not_accepted(KEY_MISSING) and free stored au_rand pdu */
        perform_key_missing_action(ce_index, auth);
        /* indicate auth_status(failed, KEY_MISSING) */
        bz_auth_handle_auth_completion(ce_index, KEY_MISSING_ERROR);

        return HCI_COMMAND_SUCCEEDED;
    }

    /* we should be initiator to reach here.
     * don't we have any collision? damn good, we can complete straight away
     */
    if (auth->pending_pdu == NULL)
    {
        if (bz_auth_is_ssp_allowed(ce_ptr))
        {
            /* send IO_cap_request_event to host */
            bz_auth_generate_io_cap_request_event(ce_index);
            bz_auth_transition_to_sub_state(auth, INTR_IO_CAP_REQUESTED);
        }
        else
        {
            /* send pin_code_request_event to host */
            enter_pin_code_requested_state(ce_index, auth,
                    INTR_PIN_CODE_REQUESTED);
        }
        return HCI_COMMAND_SUCCEEDED;
    }

    /* being a initiator and handling collisions is pretty tough -- let's see
     * what we can do with the collided pdu.
     */
    switch (bz_auth_get_pdu_opcode(auth->pending_pdu))
    {
        case LMP_AU_RAND_OPCODE:
            /* send lmp_not_accepted(KEY_MISSING) and free stored au_rand pdu */
            perform_key_missing_action(ce_index, auth);
            /* send pin_code_request_event to host */
            enter_pin_code_requested_state(ce_index, auth,
                    INTR_PIN_CODE_REQUESTED);
            break;

        case LMP_IN_RAND_OPCODE:
            /* send pin_code_request_event to host */
            enter_pin_code_requested_state(ce_index, auth,
                    INTR_PIN_CODE_REQUESTED);
            break;

        case LMP_ESCAPE4_OPCODE:
            switch (bz_auth_get_ext_pdu_opcode(auth->pending_pdu))
            {
                case LMP_IO_CAPABILITY_REQ_OPCODE:
                    /* send io_cap_requested_event to host */
                    bz_auth_generate_io_cap_request_event(ce_index);
                    bz_auth_transition_to_sub_state(auth,
                            INTR_IO_CAP_REQUESTED);
                    break;

                default:
                    BZ_ASSERT(0, "We shouldn't have stored this pdu, or "
                            "sombody forgot to make the pending_pdu = NULL");
                    bz_auth_free_pending_pdu(auth);
            }
            break;

        default:
            BZ_ASSERT(0, "We shouldn't have stored this pdu, or sombody "
                    "forgot to make the pending_pdu = NULL");
            bz_auth_free_pending_pdu(auth);
    }

    return HCI_COMMAND_SUCCEEDED;
}

/**
 * Handles link key request negative reply from the host.
 *
 * \param hci_cmd_ptr HCI Command Packet.
 *
 * \return HCI_COMMAND_SUCCEEDED, if the command was handled. A valid
 *         bluetooth error code, otherwise.
 */
UCHAR bz_auth_handle_link_key_request_neg_reply(HCI_CMD_PKT* hci_cmd_ptr)
{
    UINT16 ce_index;
    LMP_CONNECTION_ENTITY* ce_ptr;
    UINT16 reason;

    reason = perform_sanity_checks_for_reply_commands(hci_cmd_ptr,
            bz_auth_is_link_key_expected, NULL, &ce_index, &ce_ptr);
    if (reason != HCI_COMMAND_SUCCEEDED)
    {
        return HCI_COMMAND_SUCCEEDED; /* Cmd_Status should not be generated */
    }
    /* received the event at the right time, now proceed further */
    return bz_auth_perform_link_key_request_neg_reply_action(ce_index);
}

/**
 * Returns whether pin code from the host, at this point, is expected or not.
 *
 * \param auth Authentication parameters of the ACL link.
 *
 * \return TRUE, if the pin code is expected. FALSE, otherwise.
 */
BOOLEAN bz_auth_is_pin_code_expected(BZ_AUTH_LINK_PARAMS* auth)
{
    return (BOOLEAN)(auth->sub_state == INTR_PIN_CODE_REQUESTED
            || auth->sub_state == RESP_PIN_CODE_REQUESTED);
}

/**
 * Returns whether pin code parameters are valid or not.
 *
 * \param hci_cmd_ptr HCI Command Packet.
 *
 * \return TRUE, if the pin code params are valid. FALSE, otherwise.
 */
BOOLEAN bz_auth_is_valid_hci_pin_code_params(HCI_CMD_PKT* hci_cmd_ptr)
{
    if (hci_cmd_ptr->cmd_parameter[LMP_BD_ADDR_SIZE] < 0x1
            || hci_cmd_ptr->cmd_parameter[LMP_BD_ADDR_SIZE] > 0x10)
    {
        return FALSE;
    }
    return TRUE;
}

/**
 * Handles pin code request reply from the host.
 *
 * \param hci_cmd_ptr HCI Command Packet.
 *
 * \return HCI_COMMAND_SUCCEEDED, if the command was handled. A valid
 *         bluetooth error code, otherwise.
 */
UCHAR bz_auth_handle_pin_code_request_reply(HCI_CMD_PKT* hci_cmd_ptr)
{
    UINT16 ce_index;
    LMP_CONNECTION_ENTITY* ce_ptr;
    BZ_AUTH_LINK_PARAMS* auth;
    UINT16 reason;

    reason = perform_sanity_checks_for_reply_commands(hci_cmd_ptr,
            bz_auth_is_pin_code_expected, bz_auth_is_valid_hci_pin_code_params,
            &ce_index, &ce_ptr);
    if (reason != HCI_COMMAND_SUCCEEDED)
    {
        return HCI_COMMAND_SUCCEEDED; /* Cmd_Status should not be generated */
    }

    auth = ce_ptr->auth;

    /* received the pin code at right time, now store the pin temporarily
     */
    auth->txn_params.pin_len = hci_cmd_ptr->cmd_parameter[LMP_BD_ADDR_SIZE];
    memcpy(&auth->txn_params.pin[0],
            &hci_cmd_ptr->cmd_parameter[LMP_BD_ADDR_SIZE+1],
            auth->txn_params.pin_len);

    /* are we responder, then job is pretty simple */
    if (auth->sub_state == RESP_PIN_CODE_REQUESTED)
    {
        /* based on the pin_type, either send LMP_in_rand or LMP_accepted,
         * calculate init_key, switch to corresponding sub_state and free
         * the pending_pdu.
         */
        bz_auth_resp_send_in_rand_or_in_rand_accepted(ce_index, auth,
                auth->pending_pdu);
        bz_auth_free_pending_pdu(auth);

        return HCI_COMMAND_SUCCEEDED;
    }

    /* we should be initiator to reach here.
     * don't we have any collision? damn good, we can complete straight away
     */
    if (auth->pending_pdu == NULL)
    {
        /* send lmp_in_rand (our TID) and calculate init_key */
        bz_auth_send_in_rand_pdu(ce_index, auth, SELF_DEV_TID);
        bz_auth_transition_to_sub_state(auth, INTR_IN_RAND_PDU_SENT);
        return HCI_COMMAND_SUCCEEDED;
    }

    /* being a initiator and handling collisions is pretty tough -- let's see
     * what we can do with the collided pdu.
     */
    if (bz_auth_get_pdu_opcode(auth->pending_pdu) == LMP_IN_RAND_OPCODE)
    {
        if (bz_auth_is_master(ce_ptr))  /* are we master? */
        {
            /* Initiate our transaction */
            /* send lmp_in_rand(MASTER_TID) and calculate init_key */
            bz_auth_send_in_rand_pdu(ce_index, auth, MASTER_TID);
            bz_auth_transition_to_sub_state(auth, INTR_IN_RAND_PDU_SENT);

            /* Terminate slave's transaction */
            /* send lmp_not_accepted(same collision) and free stored pdu */
            bz_auth_send_not_accepted_pdu(ce_index, LMP_IN_RAND_OPCODE,
                    (LMP_TRAN_ID) LMP_GET_TRANSACTION_ID(auth->pending_pdu),
                    LMP_ERROR_TRANSACTION_COLLISION_ERROR);
            bz_auth_free_pending_pdu(auth);
        }
        else
        {
            /* AuthRole = InitiatorAsResponder */
            auth->txn_params.auth_role = BZ_AUTH_ROLE_INITIATOR_AS_RESPONDER;
            /* based on the pin_type, either send LMP_in_rand or LMP_accepted,
             * calculate init_key, switch to corresponding sub_state and free
             * the pending_pdu.
             */
            bz_auth_resp_send_in_rand_or_in_rand_accepted(ce_index, auth,
                    auth->pending_pdu);
            bz_auth_free_pending_pdu(auth);
        }
    }
    else
    {
        BZ_ASSERT(0, "We shouldn't have stored this pdu, or sombody "
                "forgot to make the pending_pdu = NULL");
        bz_auth_free_pending_pdu(auth);
    }

    return HCI_COMMAND_SUCCEEDED;
}

/**
 * Performs pin code request negative reply action
 *
 * \param ce_index ACL conection entity index.
 *
 * \return HCI_COMMAND_SUCCEEDED, if the command was handled. A valid
 *         bluetooth error code, otherwise.
 */
UCHAR bz_auth_perform_pin_code_request_neg_reply_action(UINT16 ce_index)
{
    LMP_CONNECTION_ENTITY* ce_ptr;
    BZ_AUTH_LINK_PARAMS* auth;

    ce_ptr = &lmp_connection_entity[ce_index];
    auth = ce_ptr->auth;

    /* are we initiator and no pdu collision has happened? -- the host
     * shouldn't have initiated the transaction at all... anyway let's be
     * good pals :-D.
     */
    if (auth->sub_state == INTR_PIN_CODE_REQUESTED
            && auth->pending_pdu == NULL)
    {
        /* indicate auth_status(failed, KEY_MISSING) */
        bz_auth_handle_auth_completion(ce_index, KEY_MISSING_ERROR);

        return HCI_COMMAND_SUCCEEDED;
    }

    /* we mush either be responder or initiator_with_pdu_collision to reach
     * here -- both the cases the action is same.. hahaha :)
     */
    if (auth->pending_pdu != NULL
            && bz_auth_get_pdu_opcode(auth->pending_pdu) == LMP_IN_RAND_OPCODE)
    {
        /* send lmp_not_accepted(KEY_MISSING) and free stored in_rand pdu */
        perform_key_missing_action(ce_index, auth);
        /* indicate auth_status(failed, KEY_MISSING) */
        bz_auth_handle_auth_completion(ce_index, KEY_MISSING_ERROR);

        return HCI_COMMAND_SUCCEEDED;
    }
    else
    {
        BZ_ASSERT(0, "The pending_pdu can not be NULL in the above case. "
                "If it is not NULL but the opcode is different then, either "
                "it shouldn't have been stored  or sombody forgot to make "
                "the pending_pdu = NULL (in initiator case)");
        bz_auth_free_pending_pdu(auth);
    }

    return HCI_COMMAND_SUCCEEDED;
}

/**
 * Handles pin code request negative reply from the host.
 *
 * \param hci_cmd_ptr HCI Command Packet.
 *
 * \return HCI_COMMAND_SUCCEEDED, if the command was handled. A valid
 *         bluetooth error code, otherwise.
 */
UCHAR bz_auth_handle_pin_code_request_neg_reply(HCI_CMD_PKT* hci_cmd_ptr)
{
    UINT16 ce_index;
    LMP_CONNECTION_ENTITY* ce_ptr;
    UINT16 reason;

    reason = perform_sanity_checks_for_reply_commands(hci_cmd_ptr,
            bz_auth_is_pin_code_expected, NULL, &ce_index, &ce_ptr);
    if (reason != HCI_COMMAND_SUCCEEDED)
    {
        return HCI_COMMAND_SUCCEEDED; /* Cmd_Status should not be generated */
    }

    /* received the event at the right time, now proceed further */
    return bz_auth_perform_pin_code_request_neg_reply_action(ce_index);
}

/**
 * Handles all the HCI commands related to security.
 *
 * \param hci_cmd_ptr Pointer to HCI command packet.
 *
 * \return TRUE, if the given hci command was valid security command. FALSE,
 *         otherwise.
 */
BOOLEAN bz_auth_handle_hci_security_commands(HCI_CMD_PKT *hci_cmd_ptr)
{
    UCHAR status = HCI_COMMAND_SUCCEEDED;

    switch (hci_cmd_ptr->cmd_opcode)
    {
        case HCI_LINK_KEY_REQUEST_REPLY_OPCODE:
            bz_auth_handle_link_key_request_reply(hci_cmd_ptr);
            break;

        case HCI_LINK_KEY_REQUEST_NEGATIVE_REPLY_OPCODE:
            bz_auth_handle_link_key_request_neg_reply(hci_cmd_ptr);
            break;

        case HCI_PIN_CODE_REQUEST_REPLY_OPCODE:
            bz_auth_handle_pin_code_request_reply(hci_cmd_ptr);
            break;

        case HCI_PIN_CODE_REQUEST_NEGATIVE_REPLY_OPCODE:
            bz_auth_handle_pin_code_request_neg_reply(hci_cmd_ptr);
            break;

        case HCI_IO_CAPABILITY_REQUEST_REPLY_OPCODE:
            bz_auth_handle_io_cap_request_reply(hci_cmd_ptr);
            break;

        case HCI_IO_CAPABILITY_REQUEST_NEGATIVE_REPLY_OPCODE:
            bz_auth_handle_io_cap_request_neg_reply(hci_cmd_ptr);
            break;

        case HCI_USER_CONFIRMATION_REQUEST_REPLY_OPCODE:
            bz_auth_handle_user_confirmation_request_reply(hci_cmd_ptr);
            break;

        case HCI_USER_CONFIRMATION_REQUEST_NEGATIVE_REPLY_OPCODE:
            bz_auth_handle_user_confirmation_request_neg_reply(hci_cmd_ptr);
            break;

        case HCI_USER_PASSKEY_REQUEST_REPLY_OPCODE:
            bz_auth_handle_user_passkey_request_reply(hci_cmd_ptr);
            break;

        case HCI_USER_PASSKEY_REQUEST_NEGATIVE_REPLY_OPCODE:
            bz_auth_handle_user_passkey_request_neg_reply(hci_cmd_ptr);
            break;

        case HCI_REMOTE_OOB_DATA_REQUEST_REPLY_OPCODE:
            bz_auth_handle_remote_oob_data_request_reply(hci_cmd_ptr);
            break;

        case HCI_REMOTE_OOB_DATA_REQUEST_NEGATIVE_REPLY_OPCODE:
            bz_auth_handle_remote_oob_data_request_neg_reply(hci_cmd_ptr);
            break;

        case HCI_REFRESH_ENCRYPTION_KEY_OPCODE:
            status = bz_auth_handle_refresh_encryption(hci_cmd_ptr);
            break;

        case HCI_SEND_KEYPRESS_NOTIFICATION_OPCODE:
            bz_auth_handle_key_press_notification(hci_cmd_ptr);
            break;

        case HCI_AUTHENTICATION_REQUESTED_OPCODE:
            status = bz_auth_handle_authentication_requested(hci_cmd_ptr);
            break;

        case HCI_CHANGE_CONNECTION_LINK_KEY_OPCODE:
            status = bz_auth_handle_change_connection_link_key(hci_cmd_ptr);
            break;

        case HCI_SET_CONNECTION_ENCRYPTION_OPCODE:
            status = bz_auth_handle_set_connection_encryption(hci_cmd_ptr);
            break;

#ifdef COMPILE_BROADCAST_ENCRYPTION
        case HCI_MASTER_LINK_KEY_OPCODE:
            status = bz_auth_handle_master_link_key(hci_cmd_ptr);
            break;
#endif
            /* Controller baseband commands */
        case HCI_READ_STORED_LINK_KEY_OPCODE:
        case HCI_WRITE_STORED_LINK_KEY_OPCODE:
        case HCI_DELETE_STORED_LINK_KEY_OPCODE:
        case HCI_CREATE_NEW_UNIT_KEY_OPCODE:
        case HCI_READ_PIN_TYPE_OPCODE:
        case HCI_WRITE_PIN_TYPE_OPCODE:
        case HCI_READ_AUTHENTICATION_ENABLE_OPCODE:
        case HCI_WRITE_AUTHENTICATION_ENABLE_OPCODE:
#ifdef COMPILE_DEPRECATED_COMMANDS
		case HCI_READ_ENCRYPTION_MODE_OPCODE:
        case HCI_WRITE_ENCRYPTION_MODE_OPCODE:
#endif
        case HCI_WRITE_SIMPLE_PAIRING_MODE_OPCODE:
        case HCI_READ_SIMPLE_PAIRING_MODE_OPCODE:
        case HCI_READ_LOCAL_OOB_DATA_OPCODE:
            /* Test Mode Command */
        case HCI_WRITE_SIMPLE_PAIRING_DEBUG_MODE_OPCODE:
            /* The actual work is done in command_complete_event */
            bz_auth_generate_command_complete_event(hci_cmd_ptr,
                    HCI_COMMAND_SUCCEEDED /* should be set to this value */,
                    INVALID_CE_INDEX);
            break;

        default: /* Unknown OpCode */
            return FALSE;
    }

    if (status != HCI_COMMAND_SUCCEEDED)
    {
        hci_generate_command_status_event(hci_cmd_ptr->cmd_opcode, status);
    }

    return TRUE;
}

/**
 * Handler for event masked signal. This signal is emitted whenever any
 * authentication related event can not be generated due to HCI_EVENT_MASK.
 * If the generated event request some information from the host, then this
 * handler assumes negative reply from the host and performs the action
 * accordingly.
 *
 * \param ce_index ACL connection entity index.
 * \param event_opcode HCI Event opcode that was failed to be generated to the
 *                     host due to HCI_EVENT_MASK.
 *
 * \return TRUE, if the event_opcode was handled. FALSE, otherwise.
 */
BOOLEAN bz_auth_handle_masked_event(UINT16 ce_index, UCHAR event_opcode)
{
    switch (event_opcode)
    {
        case HCI_PIN_CODE_REQUEST_EVENT:
            bz_auth_perform_pin_code_request_neg_reply_action(ce_index);
            break;
        case HCI_LINK_KEY_REQUEST_EVENT:
            bz_auth_perform_link_key_request_neg_reply_action(ce_index);
            break;
        case HCI_IO_CAPABILITY_REQUEST_EVENT:
            bz_auth_perform_io_cap_request_neg_reply_action(ce_index,
                    PAIRING_NOT_ALLOWED_ERROR);
            break;
        case HCI_USER_CONFIRMATION_REQUEST_EVENT:
            bz_auth_perform_user_confirmation_request_neg_reply_action(
                    ce_index);
            break;
        case HCI_USER_PASSKEY_REQUEST_EVENT:
            bz_auth_perform_user_passkey_request_neg_reply_action(ce_index);
            break;
        case HCI_REMOTE_OOB_DATA_REQUEST_EVENT:
            bz_auth_perform_remote_oob_data_request_neg_reply_action(ce_index);
            break;
        default:
            return FALSE;

    }
    return TRUE;
}

