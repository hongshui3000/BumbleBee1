/***************************************************************************
 Copyright (C) Mindtree Ltd.
 This module is a confidential and proprietary property of MindTree and
 a possession or use of this module requires written permission of MindTree.
 ***************************************************************************/

/**
 * \file bz_auth_states.h
 *  BlueWiz Authentication module related state definitions.
 *
 * \author Santhosh kumar M
 * \date 2007-08-10
 */

#ifndef _BZ_AUTH_STATES_H_
#define _BZ_AUTH_STATES_H_

#include "bz_auth_top_stt.h"

/**
 * \addtogroup bz_auth_internal
 * @{ */

/**
 * Authentication sub state machine states.
 */
typedef enum _BZ_AUTH_SUB_STATE
{
    BZ_AUTH_SUB_STATE_IDLE = 0x0,
    INTR_LINK_KEY_REQUESTED,
#ifdef SECURE_CONN_MUTUAL_AUTH
    SECURE_CONN_MAS_AWAIT_SRES,
    SECURE_CONN_SLV_AWAIT_SRES,
    SECURE_CONN_INTR_AWAIT_SRES,
    SECURE_CONN_MUTUAL_SEND_AU_RAND,
    SECURE_CONN_MUTUAL_AWAIT_AU_RAND,
#endif
    INTR_CHALLENGED_REMOTE_HOST,
    INTR_RESP_AWAIT_AU_RAND_PDU_NOT_ACCEPTED,
    INTR_AWAIT_AU_RAND_PDU_NOT_ACCEPTED,
    INTR_PIN_CODE_REQUESTED,
    INTR_IN_RAND_PDU_SENT,
    INTR_RESP_AWAIT_IN_RAND_PDU_NOT_ACCEPTED,
    INTR_SENT_COMB_KEY,
    INTR_SENT_UNIT_KEY,
    INTR_CHANGING_KEY_SENT_COMB_KEY,
    INTR_RESP_CHANGING_KEY_AWAIT_COMB_KEY_NOT_ACCEPTED,  //16
    INTR_RESP_AWAIT_CHALLENGE,
    INTR_CHALLENGED_REMOTE_HOST_MUTUAL,
    INTR_AWAIT_REMOTE_CHALLENGE,
    INTR_IO_CAP_REQUESTED,
    INTR_AWAIT_REMOTE_IO_CAP_RESPONSE,
    RESP_LINK_KEY_REQUESTED,
    RESP_PIN_CODE_REQUESTED,
    INTR_RESP_AWAIT_IN_RAND_PDU,
    RESP_IN_RAND_PDU_SENT,                             // 25
    RESP_AWAIT_UNIT_OR_COMB_KEY,
    RESP_AWAIT_CHALLENGE_MUTUAL,         // 27
    RESP_AWAIT_CHALLENGED_REMOTE_HOST,
    INTR_RESP_AWAIT_IO_CAP_REQ,
    RESP_IO_CAP_REQUESTED,
    RESP_AWAIT_TEMP_KEY,
    USE_SEMI_PER_KEY_PDU_SENT,
    /* PUBLIC_KEY_EXCHANGE, */
        PUBLIC_KEY_EXCHANGE_SENDING_ENCAPSULATED_PDU,
        PUBLIC_KEY_EXCHANGE_RECEIVING_ENCAPSULATED_PDU,
    /* AUTH_STAGE_I, */
    /*  OOB, */
            OOB_INTR_REMOTE_OOB_DATA_REQUESTED,
            OOB_INTR_AWAIT_SP_NUM,
            OOB_INTR_SENT_SP_NUM,
            OOB_RESP_REMOTE_OOB_DATA_REQUESTED,
            OOB_RESP_AWAIT_SP_NUM,
            OOB_RESP_SENT_SP_NUM,
   /*   NC, */
            NC_INTR_USER_CONFIRM_REQUESTED,
            NC_INTR_AWAIT_SP_CONFIRM,
            NC_INTR_AWAIT_SP_NUM,
            NC_INTR_SENT_SP_NUM,
            NC_RESP_USER_CONFIRM_REQUESTED,
            NC_RESP_AWAIT_DH_KEY_TO_SKIP,
            NC_RESP_AWAIT_SP_NUM,
            NC_RESP_SENT_SP_NUM,
    /*  PE, */
            PE_INTR_PASS_KEY_REQUESTED,
            PE_INTR_AWAIT_SP_NUM,
            PE_INTR_SENT_SP_CONFIRM,
            PE_INTR_SENT_SP_NUM,
            PE_RESP_PASSKEY_REQUESTED,
            PE_RESP_AWAIT_SP_CONFIRM_TO_SKIP,
            PE_RESP_AWAIT_SP_CONFIRM,
            PE_RESP_SENT_SP_CONFIRM,
            PE_RESP_SENT_SP_NUM,
    /* AUTH_STAGE_II, */
        INTR_AWAIT_DHKEY_RESP,
        INTR_AWAIT_DHKEY_CHECK,
        RESP_AWAIT_DHKEY_CHECK,
        RESP_AWAIT_DHKEY_RESP,
    /* Encryption Pause or Stop */
    AWAIT_DISABLE_ENCRYPTION_MODE_REQ,
    AWAIT_STOP_ENCRYPTION,                /* 58 */
    DISABLE_ENCRYPTION_MODE_REQ_SENT,
    STOP_ENCRYPTION_SENT,
    AWAIT_PAUSE_ENCRYPTION,
    AWAIT_PAUSE_ENCRYPTION_NOT_ACCEPTED,    /* 62 */
    AWAIT_DISABLE_ENCRYPTION_MOD_REQ_NOT_ACCEPTED,

    /* Encryption Resume or Start */
    AWAIT_ENABLE_ENCRYPTION_MODE_REQ,
    AWAIT_RESUME_ENCRYPTION,
    AWAIT_ENCRYPTION_KEY_SIZE_REQ,
    ENABLE_ENCRYPTION_MODE_REQ_SENT,
    AWAIT_START_ENCRYPTION,                  /* 68 */
    ENCRYPTION_KEY_SIZE_SENT,
    START_ENCRYPTION_SENT,
    AWAIT_ENABLE_ENCRYPTION_MOD_REQ_NOT_ACCEPTED,
    LAST_ENTRY_BZ_AUTH_SUB_STATE
} BZ_AUTH_SUB_STATE;

/**
 * Broadcast encryption state machine states.
 */
typedef enum _BZ_AUTH_MLK_STATE
{
    MASTER_LINK_KEY_IDLE = 0x0,
    MASTER_LINK_KEY_UPDATING,
    MASTER_LINK_KEY_IN_USE,
    MASTER_LINK_DISABLING,
    LAST_ENTRY_AUTH_MLK_STATE
} BZ_AUTH_MLK_STATE;

/** @} end: bz_auth_internal */

#endif /* _BZ_AUTH_STATES_H_ */

