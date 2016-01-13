/***************************************************************************
 Copyright (C) Mindtree Ltd.
 This module is a confidential and proprietary property of MindTree and
 a possession or use of this module requires written permission of MindTree.
 ***************************************************************************/

/**
 * \file bz_auth_internal.h
 *  BlueWiz Authentication module internal interface definition.
 *
 * \author Rajan PS, Santhosh kumar M
 * \date 2007-08-10
 */

#ifndef _BZ_AUTH_INTERNAL_H_
#define _BZ_AUTH_INTERNAL_H_

/* ========================= Include File Section ========================= */
#include "bz_auth.h"
#include "bz_auth_states.h"
#include "lmp.h"
#include "crypto.h"

/**
 * \addtogroup bz_auth_internal Authentication Internal Interface
 * @{ */


/* ====================== Macro Declaration Section ======================= */
#define BZ_AUTH_LINK_KEY_SIZE                   16
#define BZ_AUTH_TEMP_RAND_SIZE                  16
#define BZ_AUTH_TEMP_KEY_SIZE                   16
#define BZ_AUTH_RAND_SIZE                       16
#define BZ_AUTH_PIN_SIZE                        16
#define BZ_AUTH_ENC_RAND_SIZE                   16
#define BZ_AUTH_MAX_ENC_KEY_SIZE                16
#define BZ_AUTH_MIN_ENC_KEY_SIZE                1
#define BZ_AUTH_SRES_SIZE                       4
#define BZ_AUTH_ACO_SIZE                        12
#define BZ_AUTH_MAX_PIN_SIZE                    16

#define BZ_AUTH_PASSKEY_SIZE                    4
#define BZ_AUTH_CONFIRM_VALUE_SIZE              16
#define BZ_AUTH_OOB_SECRET_NUMBER_SIZE          16
#define BZ_AUTH_PUBLIC_KEY_CO_ORDINATE_SIZE     24
#define BZ_AUTH_PRIVATE_KEY_SIZE                24
#define BZ_AUTH_SP_NUMBER_SIZE                  16
#define BZ_AUTH_SECRET_NUMBER_SIZE              16
#define BZ_AUTH_DHKEY_SIZE                      24
#define BZ_AUTH_CHECK_VALUE_SIZE                16
#ifdef _SUPPORT_SECURE_CONNECTION_
#define BZ_AUTH_SECURE_CONN_PUBLIC_KEY_CO_ORDINATE_SIZE 32
#endif
#ifdef _CCH_SC_ECDH_P256_ 
#define BZ_AUTH_DEV_AUTH_KEY_SIZE               16
#define BZ_AUTH_PUBLIC_KEY_CO_ORDINATE_SIZE_P256     32
#define BZ_AUTH_PRIVATE_KEY_SIZE_P256                32

#define BZ_AUTH_PUBLIC_KEY_CO_ORDINATE_SIZE_MAX      32
#define BZ_AUTH_PRIVATE_KEY_SIZE_MAX                 32

#endif
/* ==================== Data Types Declaration Section ==================== */
/**
 * Pin type of the device. Used in HCI_READ_PIN_TYPE and HCI_WRITE_PIN_TYPE.
 */
typedef enum
{
    BZ_AUTH_PIN_TYPE_VARIABLE,       /**< Host can get pin from the user */
    BZ_AUTH_PIN_TYPE_FIXED          /**< Host has a fixed pin which can't
                                          be changed */
} BZ_AUTH_PIN_TYPE;

/**
 * Authentication Link key type.
 */
typedef enum
{
    BZ_AUTH_LINK_KEY_TYPE_COMBINATION_KEY,
    BZ_AUTH_LINK_KEY_TYPE_LOCAL_UNIT_KEY,
    BZ_AUTH_LINK_KEY_TYPE_REMOTE_UNIT_KEY,
    BZ_AUTH_LINK_KEY_TYPE_DEBUG_COMBINATION_KEY,
    BZ_AUTH_LINK_KEY_TYPE_UNAUTHENTICATED_COMBINATION_KEY,
    BZ_AUTH_LINK_KEY_TYPE_AUTHENTICATED_COMBINATION_KEY,
    BZ_AUTH_LINK_KEY_TYPE_CHANGED_COMBINATION_KEY,
#ifdef _SUPPORT_SECURE_CONNECTION_
    BZ_AUTH_LINK_KEY_TYPE_UNAUTHENTICATED_COMBINATION_KEY_P256,
    BZ_AUTH_LINK_KEY_TYPE_AUTHENTICATED_COMBINATION_KEY_P256,
#endif
    
    BZ_AUTH_LINK_KEY_TYPE_INVALID
} BZ_AUTH_LINK_KEY_TYPE;

#ifdef COMPILE_BROADCAST_ENCRYPTION
/**
 * Master Link Key parameters.
 */
typedef struct
{
    BZ_AUTH_MLK_STATE state;                /**< MLK State machine */
    UCHAR temp_rand[BZ_AUTH_TEMP_RAND_SIZE];/**< Temp rand content to be sent
                                                 to remote device */
    UCHAR temp_key[BZ_AUTH_TEMP_KEY_SIZE];  /**< Temp key content to be sent to
                                                 remote device */
    UCHAR enc_rand[BZ_AUTH_ENC_RAND_SIZE];  /**< Encryption random number to be
                                                 sent to the remove device using
                                                 LMP_start_enc_pdu */
    UCHAR enc_key[BZ_AUTH_MAX_ENC_KEY_SIZE];/**< Broadcast encryption key */
    UCHAR n_mlk_links;                      /**< Number of links using MLK */
    UCHAR n_mlk_successes;                  /**< Number of successes during the
                                                 MLK procedure */
    UCHAR n_mlk_failures;                   /**< Number of failures during the
                                                 MLK procedure */
} BZ_AUTH_MLK_DATA;
#endif /* COMPILE_BROADCAST_ENCRYPTION */

/**
 * Out Of Band authentication procedure parameters.
 */
typedef struct
{
    UCHAR R[BZ_AUTH_OOB_SECRET_NUMBER_SIZE];    /**< Randomizer value */
    UCHAR C[BZ_AUTH_CONFIRM_VALUE_SIZE];        /**< Hash value */
#ifdef _CCH_SC_ECDH_P256_    
    UCHAR R_P256[BZ_AUTH_OOB_SECRET_NUMBER_SIZE];    /**< Randomizer value */
    UCHAR C_P256[BZ_AUTH_CONFIRM_VALUE_SIZE];        /**< Hash value */
#endif

	
} BZ_AUTH_OOB_DATA;

/**
 * Secure Simple Pairing (SSP) mode.
 */
typedef enum
{
    BZ_AUTH_SSP_MODE_DISABLED,
    BZ_AUTH_SSP_MODE_ENABLED
} BZ_AUTH_SSP_MODE;

/**
 * IO Capability data parameters.
 */
typedef struct
{
    UCHAR io_cap;
    UCHAR oob_data_present;
    UCHAR auth_requirements;
} BZ_AUTH_IO_CAP_DATA;

/**
 * Elliptic Curve Diffie-Hellman Public Key.
 */
typedef ssp_pukey_t BZ_AUTH_PUBLIC_KEY;

/**
 * Elliptic Curve Diffie-Hellman Private Key.
 */
typedef ssp_prkey_t BZ_AUTH_PRIVATE_KEY;

/**
 * Diffie-Hellman Key (The output symmetric key)
 */
typedef ssp_dhkey_t BZ_AUTH_DHKEY;


#ifdef _CCH_SC_ECDH_P256_
/**
 * (P256) Elliptic Curve Diffie-Hellman Public Key.
 */
typedef ssp_pukey_t_p256 BZ_AUTH_PUBLIC_KEY_P256;

/**
 * (P256) Elliptic Curve Diffie-Hellman Private Key.
 */
typedef ssp_prkey_t_p256 BZ_AUTH_PRIVATE_KEY_P256;

/**
 * (P256) Diffie-Hellman Key (The output symmetric key)
 */
typedef ssp_dhkey_t_p256 BZ_AUTH_DHKEY_P256;

typedef ssp_pukey_t_max BZ_AUTH_PUBLIC_KEY_MAX;

typedef ssp_prkey_t_max BZ_AUTH_PRIVATE_KEY_MAX;

typedef ssp_dhkey_t_max BZ_AUTH_DHKEY_MAX;

#endif

/**
 * SSP Authentication Method.
 */
typedef enum
{
    BZ_AUTH_METHOD_NC,      /**< Numeric Comparison Method */
    BZ_AUTH_METHOD_PE,      /**< Passkey Entry Method */
    BZ_AUTH_METHOD_OOB      /**< Out Of Band Method */
} BZ_AUTH_METHOD;

/**
 * Secure Simple Pairing Data.
 */
typedef struct
{
    /* These four values are required by AUTH_STAGE2 and AUTH_STAGE1 should
     * set them properly after successful completion of AUTH_STAGE1.
     * These are termed as Na, Nb, Ra, Rb in the spec MSCs ('a' denotes
     * initiator, 'b' denotes responder).
     */
    /**
     * LMP_simple_pairing_number sent to the remote device. In case of
     * PASSKEY_ENTRY method, it holds the 20th LMP_simple_pairing_number
     * content.
     */
    UCHAR local_N[BZ_AUTH_SP_NUMBER_SIZE];
    /**
     * LMP_simple_pairing_number received from the remote device. In case of
     * PASSKEY_ENTRY method, it holds the 20th LMP_simple_pairing_number
     * content.
     */
    UCHAR remote_N[BZ_AUTH_SP_NUMBER_SIZE];
    /**
     * Local Simple Pairing Randomizer(R) value. In case of:
     * NUMERIC_COMPARISON:  The content is memset to zero
     * PASSKEY_ENTRY: The content is memset to zero and local_R[12..15] is set
     *                to the passkey value with MSB of passkey in R[12]
     * OOB: The randomizer value generated during previous
     *      HCI_READ_LOCAL_OOB_DATA command execution.
     */
    UCHAR local_R[BZ_AUTH_SECRET_NUMBER_SIZE];
    /**
     * Remote Simple Pairing Randomizer(R) value. In case of:
     * NUMERIC_COMPARISON:  The content is memset to zero
     * PASSKEY_ENTRY: The content is memset to zero and local_R[12..15] is set
     *                to the passkey value with MSB of passkey in R[12]
     * OOB: The Simple_Pairing_Randomizer_R of the remote device received from
     *      the host (using HCI_REMOTE_OOB_DATA_REQUEST_EVENT).
     */
    UCHAR remote_R[BZ_AUTH_SECRET_NUMBER_SIZE];

    /* NOTE: Since the local_C is never required, it is not stored here */

    /** remote_C is used to hold LMP_simple_pairing_confirm value from the
     * remote device in case of NUMERIC_COMPARISON and PASSKEY_ENTRY methods.
     * In OOB method, it holds the Simple_Pairing_Hash_C of the remote device
     * received from the Host (using HCI_REMOTE_OOB_DATA_REQUEST_EVENT).
     */
    UCHAR remote_C[BZ_AUTH_CONFIRM_VALUE_SIZE];

    /* The following values are specific to PASSKEY_ENTRY method */
    UINT32 passkey;     /**< The auto-generated or user-provided 6 digit
                             decimal passkey (converted into binary here) */
    UCHAR i;            /**< Indicates the round number (nth round) */
    UCHAR ri;           /**< Passkey contribution to ith round (ri = 0x80, if
                             ith bit of \a passkey is 0, otherwise 0x81) */

    /* Common parameters */
#ifndef _CCH_SC_ECDH_P256_	
    UCHAR edtm_buf[2 * BZ_AUTH_PUBLIC_KEY_CO_ORDINATE_SIZE];
    BZ_AUTH_PUBLIC_KEY remote_pubkey;
    BZ_AUTH_DHKEY dhkey;
#else
    UCHAR edtm_buf[2 * BZ_AUTH_PUBLIC_KEY_CO_ORDINATE_SIZE_MAX];
    BZ_AUTH_PUBLIC_KEY_MAX remote_pubkey;
    BZ_AUTH_DHKEY_MAX dhkey;
#endif

	
    BZ_AUTH_IO_CAP_DATA local_io_cap;
    BZ_AUTH_IO_CAP_DATA remote_io_cap;
    UINT8 auth_method;
    UINT8 lk_type;
    BOOLEAN is_oob_check_succeeded;
#ifdef _SUPPORT_SECURE_CONNECTION_
    BOOLEAN is_oob256_check_succeeded;
#endif    
} BZ_AUTH_SSP_DATA;

/**
 * Registry information for callback funtion.
 */
typedef struct
{
   BZ_GENERIC_CB cb;
   void* user_data;
} BZ_AUTH_CALLBACK_REG;

/**
 * Registry information for pause encryption callback funtion.
 */
typedef struct
{
   BZ_AUTH_PAUSE_ENC_CB cb;
   void* user_data;
} BZ_AUTH_PAUSE_CALLBACK_REG;

/**
 * Registry information for resume encryption callback funtion.
 */
typedef struct
{
   BZ_AUTH_RESUME_ENC_CB cb;
   void* user_data;
} BZ_AUTH_RESUME_CALLBACK_REG;

/**
 * Transient parameters used during authentication trancations.
 */
typedef struct
{    
    UCHAR auth_role;
    UCHAR init_key[BZ_AUTH_LINK_KEY_SIZE];
    UCHAR link_key[BZ_AUTH_LINK_KEY_SIZE];
#ifdef _CCH_SC_ECDH_P256_ 
    UCHAR dev_auth_key[BZ_AUTH_DEV_AUTH_KEY_SIZE];
    UCHAR au_rand_m[16];
    UCHAR au_rand_s[16];
    UCHAR sres_m[4];
    UCHAR sres_s[4];	
#endif
    /** Local unit or comb key contribution */
    UCHAR local_uck[BZ_AUTH_LINK_KEY_SIZE];
    UCHAR lk_type;
    UCHAR local_sres[BZ_AUTH_SRES_SIZE];
    UCHAR remote_sres[BZ_AUTH_SRES_SIZE];
    UCHAR aco[BZ_AUTH_ACO_SIZE];
    UCHAR pin[BZ_AUTH_MAX_PIN_SIZE];
    UCHAR pin_len;
    LMP_ENCAPSULATED_DATA encap_data;
    BZ_AUTH_SSP_DATA ssp_data;
} BZ_AUTH_TXN_PARAMS;

/**
 * Permanent Authentication related parameters. These parameters should not
 * be reset on HCI_RESET. These parameters are initialized during the system
 * power on and changed whenever necessary.
 */
typedef struct
{
    BZ_AUTH_PUBLIC_KEY pubkey[2];       /**< Public key */
    BZ_AUTH_PRIVATE_KEY prikey[2];      /**< Private key */
#ifdef _CCH_SC_ECDH_P256_
    BZ_AUTH_PUBLIC_KEY_P256 pubkey_p256[2];       /**< Public key */
    BZ_AUTH_PRIVATE_KEY_P256 prikey_p256[2];      /**< Private key */
#endif
} BZ_AUTH_PERMANENT_PARAMS;

/**
 * Device Authentication Parameters.
 */
typedef struct __bz_auth_dev_params
{
    UCHAR auth_enable;              /**< Authentication Enable */
    UCHAR enc_enable;               /**< Encryption Enable */
    UCHAR max_enc_key_size;         /**< Max supported encryption key size */
    UCHAR min_enc_key_size;         /**< Min supported encryption key size */
    BOOLEAN unit_key_enable;        /**< Unit Key Enable (Vendor Specific) */

    UINT8 pin_type;                 /**< Host pin type */
    BZ_AUTH_OOB_DATA last_generated_oob_data;
    UINT8 ssp_mode;
    UINT8 ssp_debug_mode;

#ifdef COMPILE_BROADCAST_ENCRYPTION
    BZ_AUTH_MLK_DATA mlk;
#endif
    UCHAR unit_key[BZ_AUTH_LINK_KEY_SIZE];
    BZ_AUTH_CALLBACK_REG auth_completed_cb;
    BZ_AUTH_PAUSE_CALLBACK_REG auth_pause_enc_cb;
    BZ_AUTH_RESUME_CALLBACK_REG auth_resume_enc_cb;
#ifdef _SUPPORT_SECURE_CONNECTION_
    UCHAR secure_connection_host_enable;
#endif
} BZ_AUTH_DEV_PARAMS;

/**
 * Authentication parameters per ACL link.
 */
typedef struct __bz_auth_link_params
{
    UINT8 super_state;
    UINT8 sub_state;
    UCHAR link_key[BZ_AUTH_LINK_KEY_SIZE];
    UINT8 lk_type;
    UCHAR enc_key[BZ_AUTH_MAX_ENC_KEY_SIZE];
    UCHAR enc_key_size;
    UINT16 key_size_mask;           /**< Encryption key size mask */
    UINT8 enc_mode;

    UCHAR aco[BZ_AUTH_ACO_SIZE];
#ifdef COMPILE_BROADCAST_ENCRYPTION
    /* semi_permanent key backup */
    UCHAR semi_bk_link_key[BZ_AUTH_LINK_KEY_SIZE];
    UINT8 semi_bk_lk_type;
    UCHAR semi_bk_aco[BZ_AUTH_ACO_SIZE];
#endif
    BZ_AUTH_TXN_PARAMS txn_params;
    LMP_PDU_PKT* pending_pdu;
    TimerHandle_t tid_timer;
    UCHAR auto_epr_time_count;

#ifdef _CCH_SC_ECDH_P256_
    UCHAR len_prime;  // 6: prior to Secure Connection 8: for Secure Connection
    UCHAR enc_rand[BZ_AUTH_ENC_RAND_SIZE];  /**< Encryption random number to be
                                                 sent to the remove device using
                                                 LMP_start_enc_pdu */  
#ifdef _CCH_SC_TEST_20130129_QCA_05                                                 
    UCHAR enc_rand_remote[BZ_AUTH_ENC_RAND_SIZE];                                                 
#endif

#ifdef _CCH_SC_ECDH_P256_PAUSE_ENC
    UCHAR sc_use_enc_rand;
#endif

#endif

#ifdef _SSP_DHKEY_CALCULATE_NEW_FLOW_
    UINT8 ce_index;
#endif
#ifdef _SUPPORT_SECURE_CONNECTION_
    UCHAR secure_conn_enabled;
#endif
#ifdef _CCH_SC_TEST_20130201_ESCO_INI_FLAG
    UCHAR not_first_esco;
#endif
    
} BZ_AUTH_LINK_PARAMS;

/* ================ Exported Variables Declaration Section ================ */
extern BZ_AUTH_DEV_PARAMS bz_auth_dev_params;
extern BZ_AUTH_PERMANENT_PARAMS bz_auth_permanent_params;

/* ============================= API Section ============================== */
API_RESULT bz_auth_generate_pdu(UINT16 ce_index, UCHAR* param_list,
        UCHAR pdu_len, LMP_TRAN_ID tran_id);
void bz_auth_start_tid_timer(UINT16 ce_index, UINT32 timeout_value);
void bz_auth_tid_timer_handler(TimerHandle_t timer_handle);
void bz_auth_transition_to_sub_state(BZ_AUTH_LINK_PARAMS* auth, UINT8 next_state);
void bz_auth_perform_super_state_transition(BZ_AUTH_LINK_PARAMS* auth, UINT8 event);
void bz_auth_free_pending_pdu(BZ_AUTH_LINK_PARAMS* auth);
void bz_auth_set_pending_pdu(BZ_AUTH_LINK_PARAMS* auth, LMP_PDU_PKT* pdu,
        BOOLEAN* can_free_pdu);
BOOLEAN bz_auth_is_ssp_allowed(LMP_CONNECTION_ENTITY* ce_ptr);
BOOLEAN bz_auth_is_to_use_temp_key_for_encryption(BZ_AUTH_LINK_PARAMS* auth);
void bz_auth_send_unit_key_pdu(UINT16 ce_index, BZ_AUTH_LINK_PARAMS* auth,
        LMP_TRAN_ID tid);
void bz_auth_send_comb_key_pdu(UINT16 ce_index, BZ_AUTH_LINK_PARAMS* auth,
        LMP_TRAN_ID tid);
void bz_auth_intr_send_unit_or_comb_key(UINT16 ce_index,
        BZ_AUTH_LINK_PARAMS* auth);
void bz_auth_compute_final_key_from_pdu(UINT16 ce_index,
        BZ_AUTH_LINK_PARAMS* auth, UCHAR local_type,
        INOUT LMP_PDU_PKT* lmp_pdu_ptr);
void bz_auth_resp_send_unit_or_comb_key(UINT16 ce_index,
        BZ_AUTH_LINK_PARAMS* auth, LMP_PDU_PKT* lmp_pdu_ptr);
void bz_auth_send_in_rand_pdu(UINT16 ce_index, BZ_AUTH_LINK_PARAMS* auth,
        LMP_TRAN_ID tid);
void bz_auth_send_in_rand_accepted_pdu(UINT16 ce_index,
        BZ_AUTH_LINK_PARAMS* auth, LMP_PDU_PKT* lmp_pdu_ptr);
void bz_auth_resp_send_in_rand_or_in_rand_accepted(UINT16 ce_index,
        BZ_AUTH_LINK_PARAMS* auth, LMP_PDU_PKT* lmp_pdu_ptr);
BOOLEAN bz_auth_is_sres_same(BZ_AUTH_LINK_PARAMS* auth);
BOOLEAN bz_auth_is_remote_initiated(LMP_CONNECTION_ENTITY* ce_ptr,
        LMP_PDU_PKT *lmp_pdu_ptr);
void bz_auth_send_sres_pdu(UINT16 ce_index, BZ_AUTH_LINK_PARAMS* auth,
        LMP_PDU_PKT* lmp_pdu_ptr);
void bz_auth_send_au_rand_pdu(UINT16 ce_index, BZ_AUTH_LINK_PARAMS* auth,
        LMP_TRAN_ID tid);
void bz_auth_send_stop_encryption_pdu(UINT16 ce_index,
        BZ_AUTH_LINK_PARAMS* auth, LMP_TRAN_ID tid);
void bz_auth_send_start_encryption_pdu(UINT16 ce_index,
        BZ_AUTH_LINK_PARAMS* auth, LMP_TRAN_ID tid);
UCHAR bz_auth_local_supported_max_enc_key_size(BZ_AUTH_LINK_PARAMS* auth);
UCHAR bz_auth_local_supported_min_enc_key_size(LMP_CONNECTION_ENTITY* ce_ptr,
        BZ_AUTH_LINK_PARAMS* auth);
void bz_auth_send_encryption_mode_req_pdu(UINT16 ce_index,
        BZ_AUTH_LINK_PARAMS* auth, LMP_TRAN_ID tid, UINT8 enc_mode);
void bz_auth_send_encryption_key_size_req_pdu(UINT16 ce_index,
        BZ_AUTH_LINK_PARAMS* auth, LMP_TRAN_ID tid);
void bz_auth_send_response_to_enable_encryption_mode_req_pdu(UINT16 ce_index,
        BZ_AUTH_LINK_PARAMS* auth, LMP_PDU_PKT* lmp_pdu_ptr);
void bz_auth_send_encryption_key_size_mask_req_pdu(UINT16 ce_index);
void bz_auth_send_encryption_key_size_mask_res_pdu(UINT16 ce_index,
        BZ_AUTH_LINK_PARAMS* auth, LMP_TRAN_ID tid);
void bz_auth_send_response_to_disable_encryption_mode_req_pdu(UINT16 ce_index,
        BZ_AUTH_LINK_PARAMS* auth, LMP_PDU_PKT* lmp_pdu_ptr);
void bz_auth_update_auth_params_from_txn_params(BZ_AUTH_LINK_PARAMS* auth);
void bz_auth_handle_auth_completion(UINT16 ce_index, UCHAR status);
void bz_auth_handle_enable_enc_completion(UINT16 ce_index, UCHAR status);
void bz_auth_handle_disable_enc_completion(UINT16 ce_index, UCHAR status);
BOOLEAN bz_auth_is_epr_supported(LMP_CONNECTION_ENTITY* ce_ptr);
void bz_auth_perform_stop_encryption_action(UINT16 ce_index,
        BZ_AUTH_LINK_PARAMS* auth);
void bz_auth_perform_start_encryption_action(UINT16 ce_index,
        BZ_AUTH_LINK_PARAMS* auth);
UCHAR bz_auth_get_abs_tid(UINT16 ce_index, LMP_TRAN_ID tid);
UCHAR bz_auth_get_tid_from_auth_role(LMP_CONNECTION_ENTITY* ce_ptr,
        BZ_AUTH_LINK_PARAMS* auth);
void bz_auth_verify_challenge(UINT16 ce_index, BZ_AUTH_LINK_PARAMS* auth,
        BOOLEAN notify_link_key);
void bz_auth_send_accepted_pdu(UINT16 ce_index, UCHAR opcode, LMP_TRAN_ID tid);
void bz_auth_send_not_accepted_pdu(UINT16 ce_index, UCHAR opcode,
        LMP_TRAN_ID tid, UCHAR reason);
void bz_auth_send_accepted_ext_pdu(UINT16 ce_index, UCHAR opcode,
        UCHAR ext_opcode, LMP_TRAN_ID tid);
void bz_auth_send_not_accepted_ext_pdu(UINT16 ce_index, UCHAR opcode,
        UCHAR ext_opcode, LMP_TRAN_ID tid, UCHAR reason);

#ifdef _SUPPORT_SECURE_CONNECTION_
void bz_auth_handle_ping_req_pdu(LMP_PDU_PKT *lmp_pdu_ptr,
        UINT16 ce_index, OUT BOOLEAN* can_free_pdu);
void bz_auth_handle_ping_res_pdu(LMP_PDU_PKT *lmp_pdu_ptr,
        UINT16 ce_index, OUT BOOLEAN* can_free_pdu);
void bz_auth_send_ping_req_pdu(UINT16 ce_index, LMP_TRAN_ID tid);
void bz_auth_send_ping_res_pdu(UINT16 ce_index, LMP_TRAN_ID tid);
#endif
#ifdef SECURE_CONN_PING_EN
void bz_auth_send_max_auth_timeout_evt_timer_callback(TimerHandle_t timer_handle);
void bz_auth_start_ping_req_timer_callback(TimerHandle_t timer_handle);
void bz_auth_start_ping_req_timer(UINT16 ce_index);
void bz_auth_stop_ping_req_timer(UINT16 ce_index);
#endif
        
/**
 * Returns whether the local device is using the fixed pin or not.
 *
 * \param None.
 *
 * \return TRUE, if the local device is using the fixed pin. FALSE, otherwise.
 */
#define bz_auth_is_fixed_pin()                              \
    (bz_auth_dev_params.pin_type == BZ_AUTH_PIN_TYPE_FIXED)

/**
 * Returns whether the local device is configured to use unit key or not.
 *
 * \return TRUE, if the local device is configured to use unit key. FALSE,
 *         otherwise.
 */
#define bz_auth_is_unit_key_enabled()   (bz_auth_dev_params.unit_key_enable)

/**
 * Returns whether the ACL link is using the unit key or not.
 *
 * \param auth Authentication parameters associated with the ACL link for
 *             which the unit key usage status is required.
 *
 * \return TRUE, if the ACL link is using unit key. FALSE, otherwise.
 */
#define bz_auth_is_link_using_unit_key(auth)                \
    (auth->lk_type == BZ_AUTH_LINK_KEY_TYPE_LOCAL_UNIT_KEY  \
     || auth->lk_type == BZ_AUTH_LINK_KEY_TYPE_REMOTE_UNIT_KEY)

/**
 * Returns whether the transaction (\a lmp_pdu_ptr) is initiated by self
 * device or not.
 *
 * \param ce_ptr ACL Connection entity pointer.
 * \param lmp_pdu_ptr LMP PDU packet for which the transaction role has to be
 *                    found.
 *
 * \return TRUE, if the transaction is self/local device initiated. FALSE,
 *         otherwise.
 */
#define bz_auth_is_self_initiated(ce_ptr, lmp_pdu_ptr)      \
    (!bz_auth_is_remote_initiated(ce_ptr, lmp_pdu_ptr))

/** @} end: bz_auth_internal */

#endif /* !_BZ_AUTH_INTERNAL_H_ */

