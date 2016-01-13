/***************************************************************************
 Copyright (C) Mindtree Ltd.
 This module is a confidential and proprietary property of MindTree and
 a possession or use of this module requires written permission of MindTree.
 ***************************************************************************/
/**
 * \file bz_auth_extern_accessors.h
 *  External data structures' accessors definition.
 *
 * \author Santhosh kumar M
 * \date 2007-08-23
 */

#ifndef _BZ_AUTH_EXTERN_ACCESSORS_H_
#define _BZ_AUTH_EXTERN_ACCESSORS_H_

/* ========================= Include File Section ========================= */
#include "bz_nvconfig.h"
#include "lmp.h"
#include "bt_fw_acl_q.h"

/* ====================== Macro Declaration Section ======================= */
/**
 * Get the remote device BD_ADDR.
 *
 * \param ce_ptr Connection entity pointer of the ACL connection.
 *
 * \return BD_ADDR of the remote device.
 */
#define bz_auth_get_remote_bd_addr(ce_ptr) ((ce_ptr)->bd_addr)

/**
 * Returns the local device BD_ADDR.
 *
 * \param None.
 *
 * \return Local BD_ADDR.
 */
#define bz_auth_get_local_bd_addr() (otp_str_data.bt_bd_addr)

/**
 * Get the remote device features.
 *
 * \param ce_ptr Connection entity pointer of the ACL connection.
 *
 * \return Features supported by the remote device.
 */
#define bz_auth_get_remote_features(ce_ptr) ((ce_ptr)->feat_page0)

/**
 * Get the am_addr associated with the \a ce_ptr.
 *
 * \param ce_ptr Connection entity pointer of the ACL connection.
 *
 * \return AM_ADDR (Active Member Address) of the ACL connection given by \a
 *         ce_ptr.
 */
#define bz_auth_get_am_addr(ce_ptr) ((ce_ptr)->am_addr)


/**
 * Get the \a piconet_id associated with the \a ce_ptr.
 *
 * \param ce_ptr Connection entity pointer of the ACL connection.
 *
 * \return Piconet ID of the ACL connection given by \a ce_ptr.
 */
#define bz_auth_get_piconet_id(ce_ptr) ((ce_ptr)->phy_piconet_id)


/**
 * Returns the features page of the local device.
 *
 * \param piconet_id Piconet ID of the device for which the features page is
 *                   requested.
 *
 * \return Features page of the local device.
 *
 * \note Different piconets can have different feature bits/pages.
 */
#define bz_auth_get_local_features(piconet_id)              \
	(lmp_feature_data.feat_page0)

/**
 * Returns the extended features page of the local device.
 *
 * \param piconet_id Piconet ID of the device for which the features page is
 *                   requested.
 *
 * \return Extended features page of the local device.
 *
 * \note Different piconets can have different feature bits/pages.
 */
#define bz_auth_get_local_ext_features(piconet_id)          \
	(&lmp_feature_data.features[1])

/**
 * Returns the extended features page of the remote device.
 *
 * \param ce_ptr ACL connection entity pointer associated with the connection
 *               for which the features page is requested.
 *
 * \return Extended features page of the remote device.
 */
#define bz_auth_get_remote_ext_features(ce_ptr)     (&(ce_ptr)->features[1])
	
/**
 * Returns whether the role of the local device with respect to ACL connection
 * denoted by \a ce_ptr is slave or not.
 *
 * \param ce_ptr Connection entity pointer to the ACL connection.
 *
 * \return TRUE, if the local device role is slave. FALSE, otherwise.
 */
#define bz_auth_is_slave(ce_ptr)    (!(ce_ptr)->remote_dev_role)

/**
 * Returns whether the role of the local device with respect to ACL connection
 * denoted by \a ce_ptr is master or not.
 *
 * \param ce_ptr Connection entity pointer to the ACL connection.
 *
 * \return TRUE, if the local device role is master. FALSE, otherwise.
 */
#define bz_auth_is_master(ce_ptr)    ((ce_ptr)->remote_dev_role)


/**
 * Returns number of HCI command buffers available.
 *
 * \param None.
 *
 * \return Number of HCI command buffers available.
 */
#define bz_auth_get_num_available_cmd_buffers()                     \
    /* refer hci_generate_command_complete_event to understand this \
     * behaviour.                                                   \
     */                                                             \
    (lmp_self_device_data.num_hci_command_packets)

/**
 * Returns the basic opcode of the pdu given by \a pdu_ptr.
 *
 * \param pdu_ptr LMP PDU packet pointer.
 *
 * \return Basic opcode of the pdu.
 */
#define bz_auth_get_pdu_opcode(pdu_ptr)             \
    ((UCHAR)((pdu_ptr)->payload_content[0]>>1))

/**
 * Returns the extended opcode of the PDU given by \a pdu_ptr.
 *
 * \param pdu_ptr LMP PDU packet pointer.
 *
 * \return Extended opcode of the PDU.
 *
 * \warning It doesn't do any sanity checking on the PDU to confirm whether
 *          it is an extended PDU or not.
 */
#define bz_auth_get_ext_pdu_opcode(pdu_ptr)   ((pdu_ptr)->payload_content[1])

/**
 * Returns whether there are any ACL connections exists in the local device.
 *
 * \param None.
 *
 * \return Non zero value, if there are any connection exists in the local
 *         device. Zero, otherwise.
 */
#define bz_auth_any_connections_exist()                 \
    (lmp_self_device_data.number_of_acl_conn)

/**
 * Pause the ACL data transfer on the link associated with the \a ce_index.
 *
 * \param ce_index ACL connection entity index.
 *
 * \return None.
 */
#ifdef COMPILE_NESTED_PAUSE_RESUME
#define bz_auth_pause_data_transfer(ce_index, flag)                               \
	aclq_mark_am_addr_as_paused(                                                  \
	bz_auth_get_am_addr(&lmp_connection_entity[(ce_index)]),              \
	bz_auth_get_piconet_id(&lmp_connection_entity[(ce_index)]), flag)
#else /* COMPILE_NESTED_PAUSE_RESUME */
#define bz_auth_pause_data_transfer(ce_index)                                     \
	aclq_mark_am_addr_as_paused(                                                  \
	bz_auth_get_am_addr(&lmp_connection_entity[(ce_index)]),              \
	bz_auth_get_piconet_id(&lmp_connection_entity[(ce_index)]))
#endif /* COMPILE_NESTED_PAUSE_RESUME */

/**
 * Resume the ACL data transfer on the link associated with the \a ce_index.
 *
 * \param ce_index ACL connection entity index.
 *
 * \return None.
 */
#ifdef COMPILE_NESTED_PAUSE_RESUME
#define bz_auth_resume_data_transfer(ce_index, flag)                            \
	aclq_resume_am_addr(                                                        \
	bz_auth_get_am_addr(&lmp_connection_entity[(ce_index)]),            \
	bz_auth_get_piconet_id(&lmp_connection_entity[(ce_index)]), flag)
#else /* COMPILE_NESTED_PAUSE_RESUME */
#define bz_auth_resume_data_transfer(ce_index)                                  \
	aclq_resume_am_addr(                                                        \
	bz_auth_get_am_addr(&lmp_connection_entity[(ce_index)]),            \
	bz_auth_get_piconet_id(&lmp_connection_entity[(ce_index)]))
#endif /* COMPILE_NESTED_PAUSE_RESUME */

/* ==================== Data Types Declaration Section ==================== */


/* ================ Exported Variables Declaration Section ================ */


/* ============================= API Section ============================== */


#endif /* _BZ_AUTH_EXTERN_ACCESSORS_H_ */

