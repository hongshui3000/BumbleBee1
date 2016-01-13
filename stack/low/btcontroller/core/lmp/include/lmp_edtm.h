/***************************************************************************
 Copyright (C) Mindtree Ltd.
 This module is a confidential and proprietary property of MindTree and
 a possession or use of this module requires written permission of MindTree.
 ***************************************************************************/

/** 
 * \file lmp_edtm.h
 *  LMP Encapsulated Data Transport Module(EDTM) interface
 * 
 * \author Rajan PS, Santhosh kumar M
 * \date 2007-09-13
 */

#ifndef _LMP_EDTM_H_
#define _LMP_EDTM_H_

/* ========================= Include File Section ========================= */
#include "bt_fw_types.h"
#include "lmp_external_defines.h"
#include "bz_auth.h"


/* ====================== Macro Declaration Section ======================= */



/* ==================== Data Types Declaration Section ==================== */
typedef struct
{
    UCHAR tid;
    UCHAR major_type;
    UCHAR minor_type;
    UCHAR payload_length;
    UCHAR* payload;
} LMP_ENCAPSULATED_DATA;

typedef BZ_GENERIC_CB LMP_ENCAPSULATED_DATA_SENT_CB;
typedef BZ_GENERIC_CB LMP_ENCAPSULATED_DATA_RECVD_CB;

typedef struct _LMP_EDTM_DATA* LMP_EDTM_DATA_PTR;   /* Opaque pointer */


/* ================ Exported Variables Declaration Section ================ */



/* ============================= API Section ============================== */
void lmp_edtm_init(void);
void lmp_edtm_init_linkparams(UINT16 ce_index);
BOOLEAN lmp_edtm_handle_encapsulated_pdus(LMP_PDU_PKT *lmp_pdu_ptr,
        UINT16 ce_index);
BOOLEAN lmp_edtm_send_data(UINT16 ce_index,
        const LMP_ENCAPSULATED_DATA* encap_data,
        LMP_ENCAPSULATED_DATA_SENT_CB sent_cb, void* user_data);
BOOLEAN lmp_edtm_receive_data(UINT16 ce_index,
        LMP_ENCAPSULATED_DATA* encap_data,
        LMP_ENCAPSULATED_DATA_RECVD_CB recvd_cb, void* user_data);

#endif /* _LMP_EDTM_H_ */
