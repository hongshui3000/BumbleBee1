/***************************************************************************
 Copyright (C) Mindtree Ltd.
 This module is a confidential and proprietary property of MindTree and
 a possession or use of this module requires written permission of MindTree.
 ***************************************************************************/

/**
 * \file bz_auth.h
 * BlueWiz Authentication module external interface definition.
 *
 * \author Rajan PS, Santhosh kumar M
 * \date 2007-08-10
 */

#ifndef _BZ_AUTH_H_
#define _BZ_AUTH_H_

/* ========================= Include File Section ========================= */
#include "bt_fw_types.h"
#include "bt_fw_hci_external_defines.h"
#include "lmp_external_defines.h"

/**
 * \addtogroup bz_auth_external Authentication External Interface
 * @{ */

/* ====================== Macro Declaration Section ======================= */



/* ==================== Data Types Declaration Section ==================== */
/**
 * Authentication role of the self device during any transaction.
 */
typedef enum
{
    BZ_AUTH_ROLE_INVALID,          /**< Invalid Authentication role */
    BZ_AUTH_ROLE_INITIATOR,
    BZ_AUTH_ROLE_RESPONDER,
    BZ_AUTH_ROLE_INITIATOR_AS_RESPONDER,
    LAST_ENTRY_BZ_AUTH_ROLE
} BZ_AUTH_ROLE;

/**
 * Encryption Mode.
 */
typedef enum
{
    BZ_AUTH_ENCRYPTION_MODE_OFF = 0x0,   /**< No encryption */
#ifdef _SUPPORT_SECURE_CONNECTION_
    BZ_AUTH_ENCRYPTION_MODE_ON,          /**< P-192 Encryption enabled */
    BZ_AUTH_ENCRYPTION_MODE_ON_P256,     /**< P-256 Encryption enabled */
#else
    BZ_AUTH_ENCRYPTION_MODE_ON           /**< Encryption enabled */
#endif    
} BZ_AUTH_ENCRYPTION_MODE;

/**
 * Encryption procedure used as a result of a generic call.
 */
typedef enum
{
    BZ_AUTH_ENCRYPTION_PROCEDURE_EPR,       /**< EPR Encryption Procedure */
    BZ_AUTH_ENCRYPTION_PROCEDURE_LEGACY     /**< 2.0 Encryption Procedure */
} BZ_AUTH_ENCRYPTION_PROCEDURE;

/**
 * Generic Callback Type.
 *
 * \param ce_index ACL connection entity index.
 * \param status Status of the operation for which the callback was
 *               registered.
 * \param reason Reason for failure, if any (Set only when status indicates
 *               a failure).
 * \param user_data The user data provided when the callback was registered.
 *
 * \return None.
 */
typedef void (*BZ_GENERIC_CB)(UINT16 ce_index, UINT16 status,
        UCHAR reason, void* user_data);

/**
 * Device authentication parameters pointer type (Opaque pointer).
 */
typedef struct __bz_auth_dev_params* BZ_AUTH_DEV_PARAMS_PTR;

/**
 * Link authentication parameters pointer type (Opaque pointer).
 */
typedef struct __bz_auth_link_params* BZ_AUTH_LINK_PARAMS_PTR;

/**
 * Pause Encryption Callback Type.
 *
 * \param ce_index ACL connection entity index.
 * \param status Status of the operation for which the callback was
 *               registered.
 * \param auth_role Authentication role of the local device during the pause
 *                  encryption procedure.
 * \param enc_proc Encryption procedure used for pausing the encryption.
 * \param user_data The user data provided when the callback was registered.
 *
 * \return None.
 */
typedef void (*BZ_AUTH_PAUSE_ENC_CB)(UINT16 ce_index, UINT16 status,
        UINT8 auth_role, UINT8 enc_proc, void* user_data);

/**
 * Resume Encryption Callback Type.
 *
 * \param ce_index ACL connection entity index.
 * \param status Status of the operation for which the callback was
 *               registered.
 * \param auth_role Authentication role of the local device during the resume
 *                  encryption procedure.
 * \param enc_proc Encryption procedure used for resuming the encryption.
 * \param user_data The user data provided when the callback was registered.
 *
 * \return None.
 */
typedef void (*BZ_AUTH_RESUME_ENC_CB)(UINT16 ce_index, UINT16 status,
        UINT8 auth_role, UINT8 enc_proc, void* user_data);

/**
 * Authentication completed Callback Type.
 */
typedef BZ_GENERIC_CB BZ_AUTH_COMPLETED_CB;


/* ================ Exported Variables Declaration Section ================ */



/* ============================= API Section ============================== */
void bz_auth_init(void);
void bz_auth_reset(void);
void bz_auth_init_linkparams(UINT16 ce_index);
BOOLEAN bz_auth_register_pause_encryption_callback(BZ_AUTH_PAUSE_ENC_CB cb,
        void* user_data);
BOOLEAN bz_auth_register_resume_encryption_callback(BZ_AUTH_RESUME_ENC_CB cb,
        void* user_data);
BOOLEAN bz_auth_register_auth_completed_callback(BZ_AUTH_COMPLETED_CB cb,
        void* user_data);
void bz_auth_handle_acl_connection_complete(UINT16 ce_index,
        UCHAR hci_error_code);
UINT8 bz_auth_get_encryption_mode(UINT16 ce_index);
#ifdef VER_3_0
UCHAR bz_auth_get_encryption_keysize(UINT16 ce_index);
#endif
BOOLEAN bz_auth_is_master_link_key_in_use(UINT16 ce_index);
BOOLEAN bz_auth_handle_hci_security_commands(HCI_CMD_PKT *hci_cmd_ptr);
BOOLEAN bz_auth_handle_security_pdus(LMP_PDU_PKT *lmp_pdu_ptr,
        UINT16 ce_index);
BOOLEAN bz_auth_initiate_authentication_during_connection(UINT16 ce_index);
void bz_auth_enable_link_level_encryption(UINT16 ce_index, BOOLEAN affect_bb);
void bz_auth_disable_link_level_encryption(UINT16 ce_index, BOOLEAN affect_bb);
#ifdef COMPILE_BROADCAST_ENCRYPTION
void bz_auth_enable_broadcast_encryption(UCHAR* key_dash);
void bz_auth_disable_broadcast_encryption(void);
#endif
BOOLEAN bz_auth_is_link_encrypted(UINT16 ce_index);
BOOLEAN bz_auth_pause_encryption(UINT16 ce_index);
BOOLEAN bz_auth_resume_encryption(UINT16 ce_index);
void bz_auth_calculate_dhkey(BZ_AUTH_LINK_PARAMS_PTR auth);
void bz_auth_create_auth_timers(UINT16 ce_index);
void bz_auth_reset_auth_timers(UINT16 ce_index);
void bz_auth_handle_timeout_for_auto_epr(UINT32 interval);
BOOLEAN bz_auth_handle_masked_event(UINT16 ce_index, UCHAR event_opcode);

#endif /* _BZ_AUTH_H_ */

