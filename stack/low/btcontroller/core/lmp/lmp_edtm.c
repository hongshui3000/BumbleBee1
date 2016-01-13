/***************************************************************************
 Copyright (C) Mindtree Ltd.
 This module is a confidential and proprietary property of MindTree and
 a possession or use of this module requires written permission of MindTree.
 ***************************************************************************/
/********************************* Logger *************************/ 
enum { __FILE_NUM__= 52 };
/********************************* Logger *************************/

/* ========================= Include File Section ========================= */
#include "lmp.h"
#include "bz_auth_extern_accessors.h"
#include "lmp_edtm.h"
#include "bz_debug.h"
#include "mem.h"
#include "lc.h"

#include "bz_auth_internal.h"

/* ====================== Macro Declaration Section ======================= */
#define lmp_edtm_generate_pdu(c, p, l, t)                                \
    lmp_generate_pdu(c, p, l, t, LMP_NO_STATE_CHANGE)

#define lmp_edtm_send_accepted_pdu(ce_index, opcode, tid)                \
    lmp_send_lmp_accepted(ce_index, opcode, (UCHAR)(tid), LMP_NO_STATE_CHANGE)

#define lmp_edtm_send_not_accepted_pdu(ce_index, opcode, tid, reason)    \
    lmp_send_lmp_not_accepted(ce_index, opcode, (UCHAR)(tid), reason)


/* ==================== Structure Declaration Section ===================== */
/** 
 * Encapsulated Data Transport Module(EDTM) Data.
 */
typedef struct _LMP_EDTM_DATA
{
    const LMP_ENCAPSULATED_DATA* sdata;
    LMP_ENCAPSULATED_DATA_SENT_CB sent_cb;
    void* sent_cb_user_data;
    LMP_ENCAPSULATED_DATA* rdata;
    LMP_ENCAPSULATED_DATA_RECVD_CB recvd_cb;
    void* recvd_cb_user_data;
    UCHAR nbytes_sent;
    UCHAR nbytes_recvd;
} LMP_EDTM_DATA;


/* ===================== Variable Declaration Section ===================== */
static LMP_EDTM_DATA lmp_edtm_data[LMP_MAX_CE_DATABASE_ENTRIES];

#ifdef _CCH_IOT_RALINK_		
extern EFUSE_LPS_SETTING_3_S g_efuse_lps_setting_3;
#endif

/* ================== Static Function Prototypes Section ================== */


/* ===================== Function Definition Section ====================== */
void lmp_edtm_init(void)
{
    memset(lmp_edtm_data, 0x0, sizeof(lmp_edtm_data));
}

void lmp_edtm_init_linkparams(UINT16 ce_index)
{
    LMP_EDTM_DATA* edtm;

    edtm = &lmp_edtm_data[ce_index];
    lmp_connection_entity[ce_index].edtm = edtm;
    memset(edtm, 0, sizeof(LMP_EDTM_DATA));
}

void lmp_edtm_invoke_sent_cb(UINT16 ce_index, LMP_EDTM_DATA* edtm,
        UCHAR status)
{
    void* user_data;
    LMP_ENCAPSULATED_DATA_SENT_CB sent_cb;

    user_data = edtm->sent_cb_user_data;
    sent_cb = edtm->sent_cb;
    /* Reset the internal variables:
     * We do it before calling the "send_cb" because there is high
     * possiblity that the callback might again invoke EDTM for some other
     * EDTM operation (eg. edtm_receive_data).
     */
    edtm->sdata = NULL;
    edtm->sent_cb = NULL;
    edtm->sent_cb_user_data = NULL;
    edtm->nbytes_sent = 0x0;

    /* we sent the full payload */
    sent_cb(ce_index, status, status, user_data);
}

void lmp_edtm_invoke_recvd_cb(UINT16 ce_index, LMP_EDTM_DATA* edtm,
        UCHAR status)
{
    void* user_data;
    LMP_ENCAPSULATED_DATA_RECVD_CB recvd_cb;

    user_data = edtm->recvd_cb_user_data;
    recvd_cb = edtm->recvd_cb;

    /* Reset the internal variables:
     * We do it before calling the "recvd_cb" because there is high
     * possiblity that the callback might again invoke EDTM for some other
     * EDTM operation (eg. edtm_send_data).
     */
    edtm->rdata = NULL;
    edtm->recvd_cb = NULL;
    edtm->recvd_cb_user_data = NULL;
    edtm->nbytes_recvd = 0x0;

    /* we received the full payload */
    recvd_cb(ce_index, status, status, user_data);
}

void lmp_edtm_send_encapsulated_header_pdu(UINT16 ce_index,
        LMP_EDTM_DATA* edtm)
{
    UCHAR param_list[LMP_ENCAPSULATED_HEADER_LEN];

	param_list[0] = LMP_ENCAPSULATED_HEADER_OPCODE;

#ifdef _CCH_SC_TEST_20130129_QCAXXXXX

    BZ_AUTH_LINK_PARAMS* auth;
    auth = lmp_connection_entity[ce_index].auth;

if(auth->secure_conn_enabled)
{
    edtm->sdata->minor_type = 2;
}
#endif

    param_list[2] = edtm->sdata->major_type;
    param_list[3] = edtm->sdata->minor_type;
    param_list[4] = edtm->sdata->payload_length;
    edtm->nbytes_sent = 0x0;
    lmp_edtm_generate_pdu(ce_index, param_list,
            LMP_ENCAPSULATED_HEADER_LEN, (LMP_TRAN_ID) edtm->sdata->tid);
}

void lmp_edtm_send_encapsulated_payload_pdu(UINT16 ce_index,
        LMP_EDTM_DATA* edtm)
{
    UCHAR param_list[LMP_ENCAPSULATED_PAYLOAD_LEN];
    /* default to the max bytes to sent */
    UCHAR bytes_to_send = (UCHAR)
                          (edtm->sdata->payload_length - edtm->nbytes_sent);

    memset(&param_list[0], 0x0, LMP_ENCAPSULATED_PAYLOAD_LEN);
    if (bytes_to_send > (LMP_ENCAPSULATED_PAYLOAD_LEN - 2))
    {
        /* restrict it if more data to send */
        bytes_to_send = (LMP_ENCAPSULATED_PAYLOAD_LEN - 2);
    }
	param_list[0] = LMP_ENCAPSULATED_PAYLOAD_OPCODE;
    memcpy(&param_list[2], &edtm->sdata->payload[edtm->nbytes_sent],
            bytes_to_send);
    edtm->nbytes_sent = (UCHAR) ( edtm->nbytes_sent + bytes_to_send);
    lmp_edtm_generate_pdu(ce_index, param_list,
            LMP_ENCAPSULATED_PAYLOAD_LEN, (LMP_TRAN_ID) edtm->sdata->tid);
}

/** 
 * Initializes the EDTM module to start sending encapsulated data to the
 * remote device.
 * 
 * \param ce_index ACL connection entity index for which the data has to be
 *                 sent.
 * \param encap_data Encapsulated data to be sent. 
 * \param recvd_cb Callback to be invoked on completion of the send
 *                 operation.
 * \param user_data User data to be passed when invoking the callback \a
 *                  sent_cb.
 * 
 * \return TRUE, if the operation was started successfully. FALSE, otherwise.
 */
BOOLEAN lmp_edtm_send_data(UINT16 ce_index,
        const LMP_ENCAPSULATED_DATA* encap_data,
        LMP_ENCAPSULATED_DATA_SENT_CB sent_cb, void* user_data)
{
    LMP_CONNECTION_ENTITY* ce_ptr;
    LMP_EDTM_DATA* edtm;

    if (encap_data == NULL || encap_data->payload == NULL)
    {
        BZ_ASSERT(0, "I can't send NULL data in encapsulated PDU");
        return FALSE;
    }

    ce_ptr = &lmp_connection_entity[ce_index];
    edtm = ce_ptr->edtm;
    if (edtm->sdata != NULL || edtm->rdata != NULL)
    {
        BZ_ASSERT(0, "Encapsulated data transfer in progress, can't satisfy "
                "the send request now (it may be a bug also)");
        return FALSE;
    }
    edtm->sdata = encap_data;
    edtm->sent_cb = sent_cb;
    edtm->sent_cb_user_data = user_data;
    lmp_edtm_send_encapsulated_header_pdu(ce_index, edtm);

    return TRUE;
}

/** 
 * Initializes the EDTM module to start receiving encapsulated data from the
 * remote device.
 * 
 * \param ce_index ACL connection entity index for which the data has to be
 *                 received.
 * \param encap_data Indicates what kind of encapsulated data to receive and
 *                   where to store it. (\a major_type, \a minor_type, \a
 *                   payload_length) specifies the expected data type and its
 *                   length. The received data is filled in
 *                   \a encapdata->payload.
 * \param recvd_cb Callback to be invoked on completion of the receive
 *                 operation.
 * \param user_data User data to be passed when invoking the callback \a
 *                  recd_cb.
 * 
 * \return TRUE, if the operation was started successfully. FALSE, otherwise.
 */
BOOLEAN lmp_edtm_receive_data(UINT16 ce_index,
        INOUT LMP_ENCAPSULATED_DATA* encap_data,
        LMP_ENCAPSULATED_DATA_RECVD_CB recvd_cb, void* user_data)
{
    LMP_CONNECTION_ENTITY* ce_ptr;
    LMP_EDTM_DATA* edtm;

    if (encap_data == NULL || encap_data->payload == NULL)
    {
        BZ_ASSERT(0, "Where the hell I will store the received encapsulated "
                "data? Don't give me NULL pointer");
        return FALSE;
    }

    ce_ptr = &lmp_connection_entity[ce_index];
    edtm = ce_ptr->edtm;
    if (edtm->sdata != NULL || edtm->rdata != NULL)
    {
        BZ_ASSERT(0, "Encapsulated data transfer in progress, can't satisfy "
                "the receive request now (it may be a bug also)");
        return FALSE;
    }
    edtm->rdata = encap_data;
    edtm->recvd_cb = recvd_cb;
    edtm->recvd_cb_user_data = user_data;

    return TRUE;
}

/** 
 * Handles lmp_accepted(lmp_encapsulated_header) PDU from the remote device.
 * 
 * \param lmp_pdu_ptr LMP PDU packet from the remote device.
 * \param ce_index ACL Connection entity index.
 * \return None.
 */
void lmp_edtm_handle_encapsulated_header_accepted_pdu(
        LMP_PDU_PKT* lmp_pdu_ptr, UINT16 ce_index)
{
    LMP_CONNECTION_ENTITY* ce_ptr;
    LMP_EDTM_DATA* edtm;
    LMP_TRAN_ID tid;
    UINT8 diff;

    ce_ptr = &lmp_connection_entity[ce_index];
    edtm = ce_ptr->edtm;
    tid = (LMP_TRAN_ID)LMP_GET_TRANSACTION_ID(lmp_pdu_ptr);
    
#ifdef _CCH_IOT_RALINK_		
    if(g_efuse_lps_setting_3.iot_ralink_tid_no_check )
    {
        diff = FALSE;
    }
    else
#endif        
    {
        diff = (tid != edtm->sdata->tid) ? TRUE : FALSE;
    }

    if (edtm->sdata == NULL || diff)
    {
        BZ_ASSERT(0, "The remote device is mis-behaving (might be sending "
                "the accepted PDU to exploit us) or there is some "
                "problem with EDTM module");
        return;
    }
    lmp_edtm_send_encapsulated_payload_pdu(ce_index, edtm);
}

/** 
 * Handles lmp_accepted(lmp_encapsulated_payload) PDU from the remote device.
 * 
 * \param lmp_pdu_ptr LMP PDU packet from the remote device.
 * \param ce_index ACL Connection entity index.
 * \return None.
 */
void lmp_edtm_handle_encapsulated_payload_accepted_pdu(
        LMP_PDU_PKT* lmp_pdu_ptr, UINT16 ce_index)
{
    LMP_CONNECTION_ENTITY* ce_ptr;
    LMP_EDTM_DATA* edtm;
    LMP_TRAN_ID tid;
    UINT8 diff;
    
    ce_ptr = &lmp_connection_entity[ce_index];
    edtm = ce_ptr->edtm;
    tid = (LMP_TRAN_ID)LMP_GET_TRANSACTION_ID(lmp_pdu_ptr);

#ifdef _CCH_IOT_RALINK_		
    if(g_efuse_lps_setting_3.iot_ralink_tid_no_check )
    {
        diff = FALSE;
    }
    else
#endif        
    {
        diff = (tid != edtm->sdata->tid) ? TRUE : FALSE;
    }

    if (edtm->sdata == NULL || diff)
    {
        BZ_ASSERT(0, "The remote device is mis-behaving (might be sending "
                "the accepted PDU to exploit us) or there is some "
                "problem with EDTM module");
        return;
    }
    if (edtm->sdata->payload_length == edtm->nbytes_sent)
    {
        lmp_edtm_invoke_sent_cb(ce_index, edtm, HCI_COMMAND_SUCCEEDED);
    }
    else
    {
        lmp_edtm_send_encapsulated_payload_pdu(ce_index, edtm);
    }
}

/** 
 * Handles lmp_accepted for encapsulated pdus.
 * 
 * \param lmp_pdu_ptr LMP PDU packet from the remote device.
 * \param ce_index ACL Connection entity index.
 * \return TRUE, if the PDU was handled. FALSE, otherwise.
 */
BOOLEAN lmp_edtm_handle_accepted_pdu(LMP_PDU_PKT *lmp_pdu_ptr,
        UINT16 ce_index)
{
    UCHAR pdu_opcode;

    pdu_opcode = lmp_pdu_ptr->payload_content[1];

    switch (pdu_opcode)
    {
        case LMP_ENCAPSULATED_HEADER_OPCODE:
            lmp_edtm_handle_encapsulated_header_accepted_pdu(lmp_pdu_ptr,
                    ce_index);
            break;
        case LMP_ENCAPSULATED_PAYLOAD_OPCODE:
            lmp_edtm_handle_encapsulated_payload_accepted_pdu(lmp_pdu_ptr,
                    ce_index);
            break;
        default:
            return FALSE;
    }
    return TRUE;
}

/** 
 * Handles lmp_not_accepted(lmp_encapsulated_header) PDU 
 * from the remote device.
 * 
 * \param lmp_pdu_ptr LMP PDU packet from the remote device.
 * \param ce_index ACL Connection entity index.
 * \return None.
 */
void lmp_edtm_handle_encapsulated_header_not_accepted_pdu(
        LMP_PDU_PKT* lmp_pdu_ptr, UINT16 ce_index)
{
    LMP_CONNECTION_ENTITY* ce_ptr;
    LMP_EDTM_DATA* edtm;
    LMP_TRAN_ID tid;
    UCHAR lmp_reason;
    UINT8 diff;        

    ce_ptr = &lmp_connection_entity[ce_index];
    edtm = ce_ptr->edtm;
    lmp_reason = lmp_pdu_ptr->payload_content[2];
    tid = (LMP_TRAN_ID)LMP_GET_TRANSACTION_ID(lmp_pdu_ptr);

#ifdef _CCH_IOT_RALINK_		
    if(g_efuse_lps_setting_3.iot_ralink_tid_no_check )
    {
        diff = FALSE;
    }
    else
#endif        
    {
        diff = (tid != edtm->sdata->tid) ? TRUE : FALSE;
    }
    
    if (edtm->sdata == NULL || diff)
    {
        BZ_ASSERT(0, "The remote device is mis-behaving (might be sending "
                " the not_accepted PDU to exploit us) or there is some "
                "problem with EDTM module");
        return;
    }
    /* failed to sent the full payload */
    lmp_edtm_invoke_sent_cb(ce_index, edtm, lmp_reason);
}

/** 
 * Handles lmp_not_accepted(lmp_encapsulated_payload) PDU 
 * from the remote device.
 * 
 * \param lmp_pdu_ptr LMP PDU packet from the remote device.
 * \param ce_index ACL Connection entity index.
 * \return None.
 */
void lmp_edtm_handle_encapsulated_payload_not_accepted_pdu(
        LMP_PDU_PKT* lmp_pdu_ptr, UINT16 ce_index)
{
    lmp_edtm_handle_encapsulated_header_not_accepted_pdu(lmp_pdu_ptr,
            ce_index);
}

/** 
 * Handles lmp_not_accepted for encapsulated pdus.
 * 
 * \param lmp_pdu_ptr LMP PDU packet from the remote device.
 * \param ce_index ACL Connection entity index.
 * \return TRUE, if the PDU was handled. FALSE, otherwise.
 */
BOOLEAN lmp_edtm_handle_not_accepted_pdu(LMP_PDU_PKT *lmp_pdu_ptr,
        UINT16 ce_index)
{
    UCHAR pdu_opcode;

    pdu_opcode = lmp_pdu_ptr->payload_content[1];

    switch (pdu_opcode)
    {
        case LMP_ENCAPSULATED_HEADER_OPCODE:
            lmp_edtm_handle_encapsulated_header_not_accepted_pdu(
                    lmp_pdu_ptr, ce_index);
            break;
        case LMP_ENCAPSULATED_PAYLOAD_OPCODE:
            lmp_edtm_handle_encapsulated_payload_not_accepted_pdu(
                    lmp_pdu_ptr, ce_index);
            break;
        default:
            return FALSE;
    }
    return TRUE;
}

/** 
 * Handles lmp_encapsulated_header PDU from the remote device.
 * 
 * \param lmp_pdu_ptr LMP PDU packet from the remote device.
 * \param ce_index ACL Connection entity index.
 * \return None.
 */
void lmp_edtm_handle_encapsulated_header_pdu(LMP_PDU_PKT *lmp_pdu_ptr,
        UINT16 ce_index)
{
    LMP_CONNECTION_ENTITY* ce_ptr;
    LMP_EDTM_DATA* edtm;
    LMP_TRAN_ID tid;
    UINT8 diff;   

    ce_ptr = &lmp_connection_entity[ce_index];
    edtm = ce_ptr->edtm;
    tid = (LMP_TRAN_ID)LMP_GET_TRANSACTION_ID(lmp_pdu_ptr);

#ifdef _CCH_IOT_RALINK_		
    if (g_efuse_lps_setting_3.iot_ralink_tid_no_check )
    {
        diff = FALSE;
    }
    else
#endif        
    {
        diff = (tid != edtm->rdata->tid) ? TRUE : FALSE;
    }

    if (edtm->rdata == NULL
            || diff
            || edtm->rdata->major_type != lmp_pdu_ptr->payload_content[1]
            || edtm->rdata->minor_type != lmp_pdu_ptr->payload_content[2]
            || edtm->rdata->payload_length != lmp_pdu_ptr->payload_content[3])
    {
        BZ_ASSERT(0, "The remote device is mis-behaving (might be sending "
                " the encap_header PDU to exploit us) or there is some "
                "problem with EDTM module or its a TID problem or "
                "(major,minor,payload_length) type is not expected now");
        lmp_edtm_send_not_accepted_pdu(ce_index,
                LMP_ENCAPSULATED_HEADER_OPCODE, tid, PDU_NOT_ALLOWED_ERROR);
        return;
    }
    edtm->nbytes_recvd = 0x0;
    lmp_edtm_send_accepted_pdu(ce_index, LMP_ENCAPSULATED_HEADER_OPCODE, tid);
}

/** 
 * Handles lmp_encapsulated_header PDU from the remote device.
 * 
 * \param lmp_pdu_ptr LMP PDU packet from the remote device.
 * \param ce_index ACL Connection entity index.
 * \return None.
 */
void lmp_edtm_handle_encapsulated_payload_pdu(LMP_PDU_PKT *lmp_pdu_ptr,
        UINT16 ce_index)
{
    LMP_CONNECTION_ENTITY* ce_ptr;
    LMP_EDTM_DATA* edtm;
    LMP_TRAN_ID tid;
    UCHAR bytes_to_receive = 0x0;
    UINT8 diff;

    ce_ptr = &lmp_connection_entity[ce_index];
    edtm = ce_ptr->edtm;
    tid = (LMP_TRAN_ID)LMP_GET_TRANSACTION_ID(lmp_pdu_ptr);

#ifdef _CCH_IOT_RALINK_		
    if(g_efuse_lps_setting_3.iot_ralink_tid_no_check )
    {
        diff = FALSE;
    }
    else
#endif        
    {
        diff = (tid != edtm->rdata->tid) ? TRUE : FALSE;
    }


    if (edtm->rdata == NULL || diff)
    {
        BZ_ASSERT(0, "The remote device is mis-behaving (might be sending "
                " the encap_header PDU to exploit us) or there is some "
                "problem with EDTM module or its a TID problem");
        lmp_edtm_send_not_accepted_pdu(ce_index,
                LMP_ENCAPSULATED_PAYLOAD_OPCODE, tid, PDU_NOT_ALLOWED_ERROR);
        return;
    }
    /* default to the max bytes to receive */
    bytes_to_receive = (UCHAR)
                       (edtm->rdata->payload_length - edtm->nbytes_recvd);
    if (bytes_to_receive > (LMP_ENCAPSULATED_PAYLOAD_LEN - 2))
    {
        /* restrict it if more data to receive */
        bytes_to_receive = (LMP_ENCAPSULATED_PAYLOAD_LEN - 2);
    }
    memcpy(&edtm->rdata->payload[edtm->nbytes_recvd],
            &lmp_pdu_ptr->payload_content[1], bytes_to_receive);
    edtm->nbytes_recvd = (UCHAR) (edtm->nbytes_recvd + bytes_to_receive);
    lmp_edtm_send_accepted_pdu(ce_index, LMP_ENCAPSULATED_PAYLOAD_OPCODE, tid);
    if (edtm->nbytes_recvd == edtm->rdata->payload_length)
    {
        /* we received the full payload */
        lmp_edtm_invoke_recvd_cb(ce_index, edtm, HCI_COMMAND_SUCCEEDED);
    }
}

/** 
 * Handles encapsulated pdus.
 *
 * \param lmp_pdu_ptr LMP PDU packet from the remote device.
 * \param ce_index  ACL connection entity index.
 * 
 * \return TRUE, if the PDU was handled. FALSE, otherwise.
 */
BOOLEAN lmp_edtm_handle_encapsulated_pdus(LMP_PDU_PKT *lmp_pdu_ptr,
        UINT16 ce_index)
{
    switch (bz_auth_get_pdu_opcode(lmp_pdu_ptr))
    {
        case LMP_ACCEPTED_OPCODE:
            if (!lmp_edtm_handle_accepted_pdu(lmp_pdu_ptr, ce_index))
            {
                return FALSE;
            }
            break;
        case LMP_NOT_ACCEPTED_OPCODE:
            if (!lmp_edtm_handle_not_accepted_pdu(lmp_pdu_ptr, ce_index))
            {
                return FALSE;
            }
            break;
        case LMP_ENCAPSULATED_HEADER_OPCODE:
            lmp_edtm_handle_encapsulated_header_pdu(lmp_pdu_ptr, ce_index);
            break;
        case LMP_ENCAPSULATED_PAYLOAD_OPCODE:
            lmp_edtm_handle_encapsulated_payload_pdu(lmp_pdu_ptr, ce_index);
            break;
        default:
            return FALSE;
    }
	if(OS_FREE_BUFFER(lmp_pdu_buffer_pool_handle, lmp_pdu_ptr) != BT_ERROR_OK)
	{
		RT_BT_LOG(GRAY, LMP_EDTM_522, 0, 0);
	}

    return TRUE;
}

