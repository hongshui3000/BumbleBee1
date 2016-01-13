/***************************************************************************
 Copyright (C) Mindtree Ltd.
 This module is a confidential and proprietary property of MindTree and
 a possession or use of this module requires written permission of MindTree.
 ***************************************************************************/

/**
 * \file
 *  Contains all the hci commands handlers related
 *  to the Bluetooth Version 1.2 - 2.1 Features.
 */




/********************************* Logger *************************/
enum { __FILE_NUM__= 4 };
/********************************* Logger *************************/

#include "mem.h"

/* Includes */
#include "bt_fw_types.h"
#include "bt_fw_hci_defines.h"
#include "lmp.h"
#include "bt_fw_hci.h"
#include "lc.h"
#include "bt_fw_hci_1_2_cmds.h"
#include "bt_fw_hci_internal.h"
#include "btc_defines.h"
#include "vendor.h"
#include "bb_driver.h"
#include "bz_debug.h"

#include "bt_fw_hci_2_1.h"
#ifdef _SUPPORT_SECURE_CONNECTION_
#include "bz_auth_internal.h"
#endif

#ifdef COMPILE_CHANNEL_ASSESSMENT
#include "lmp_ch_assessment.h"
#endif /* COMPILE_CHANNEL_ASSESSMENT */

#ifdef VER_CSA4
#include "bt_3dd.h"
#endif

#ifdef COMPILE_ESCO
extern LMP_ESCO_CONNECTION_ENTITY
       lmp_esco_connection_entity[LMP_MAX_ESCO_CONN_ENTITIES];

extern LMP_ESCO_CONNECTION_ENTITY temp_esco_connection_entity;
extern UCHAR lmp_local_dev_init_esco_param_change;
#endif /* COMPILE_ESCO */

#ifdef _BRUCE_MSBC_IS_RESTRICTED_PKT_TYPE
extern UINT16 g_voice_setting;
#endif

/**
 * Handles all the link control commands for the Bluetooth Version 1_2.
 * This function is called from the hci_task.c if the opcode
 * does not match any of the 1.1 Version opcodes.
 *
 * \param hci_cmd_ptr Pointer to HCI_CMD_PKT.
 * \return UCHAR HCI_COMMAND_SUCCEEDED or HCI error code.
 *
 */
UCHAR hci_handle_1_2_link_control_command(HCI_CMD_PKT *hci_cmd_ptr)
{
    UCHAR ret_error = HCI_COMMAND_SUCCEEDED;
    UCHAR sent_cmd_status = FALSE;
#if defined(ENABLE_SCO) || defined(COMPILE_ESCO)
    UINT16 ce_index;
    HCI_LINK_TYPE link_type;
#endif

    switch(hci_cmd_ptr->cmd_opcode)
    {
#if defined(ENABLE_SCO) || defined(COMPILE_ESCO)
        case HCI_SETUP_SYNCHRONOUS_CONNECTION_OPCODE:
        {
			//RT_BT_LOG(GREEN, BT_FW_HCI_1_2_CMDS_85, 0, 0);

            SYNC_LINK_STATUS sync_link;
            sent_cmd_status = TRUE;
            ret_error = hci_handle_setup_sync_conn_command(hci_cmd_ptr,
                    &ce_index, &link_type, FALSE, &sync_link);
            if (ret_error != HCI_COMMAND_SUCCEEDED)
            {
                if (sync_link == NEW_SYNC_LINK)
                {
                    hci_generate_synchronous_conn_complete_event(ce_index,
                            INVALID_CE_INDEX, link_type, ret_error);
                }
#ifdef COMPILE_ESCO
                else
                {
                    hci_generate_synchronous_conn_changed_event(ret_error,
                            ce_index);
                }
#endif
            }
            break;
        }

        case HCI_ACCEPT_SYNCHRONOUS_CONNECTION_REQ_OPCODE:
            sent_cmd_status = TRUE;
            ret_error = hci_handle_accept_sync_conn_req_command(hci_cmd_ptr,
                    &ce_index, &link_type, FALSE);
            if (ret_error != HCI_COMMAND_SUCCEEDED)
            {
                hci_generate_synchronous_conn_complete_event(ce_index,
                        INVALID_CE_INDEX, link_type, ret_error);
            }
            break;

        case HCI_REJECT_SYNCHRONOUS_CONNECTION_REQ_OPCODE:
            sent_cmd_status = TRUE;
            ret_error = hci_handle_reject_sync_conn_req_command(hci_cmd_ptr,
                    &ce_index, &link_type);
            if (ret_error != HCI_COMMAND_SUCCEEDED)
            {
                hci_generate_synchronous_conn_complete_event(ce_index,
                        INVALID_CE_INDEX, link_type, ret_error);
            }
            break;
#endif /* defined(ENABLE_SCO) || defined(COMPILE_ESCO) */

#ifdef COMPILE_FEATURE_REQ_EXT
        case HCI_READ_REMOTE_EXTENDED_FEATURES_OPCODE:
            ret_error = hci_handle_read_remote_ext_features_command(
                    hci_cmd_ptr, &sent_cmd_status);
            break;
#endif
        default:
            return UNKNOWN_HCI_COMMAND_ERROR;
    } /* end switch(hci_cmd_ptr->cmd_opcode) */

    if(sent_cmd_status == FALSE)
    {
        hci_generate_command_status_event(hci_cmd_ptr->cmd_opcode,
                ret_error);
    }

    return HCI_COMMAND_SUCCEEDED;
}

/**
 * Handles all the link policy commands for the Bluetooth Version 1_2.
 * This function is called from the hci_task.c if the opcode
 * does not match any of the 1.1 Version opcodes.
 *
 * \param hci_cmd_ptr Pointer to HCI_CMD_PKT.
 * \param ce_index Connection Entity Index.
 *
 * \return HCI_COMMAND_SUCCEEDED or HCI error code.
 *
 */
UCHAR hci_handle_1_2_link_policy_command(HCI_CMD_PKT *hci_cmd_ptr,
                                         UINT16 ce_index)
{
    switch(hci_cmd_ptr->cmd_opcode)
    {
        default:
                break;
    }
    return UNKNOWN_HCI_COMMAND_ERROR;
}
/**
 * Handles all the host controller and baseband for the Bluetooth Version 1_2.
 * This function is called from the hci_task.c if the opcode
 * does not match any of the 1.1 Version opcodes.
 *
 * \param hci_cmd_ptr Pointer to HCI_CMD_PKT.
 *
 * \return HCI_COMMAND_SUCCEEDED or HCI error code.
 *
 */
UCHAR hci_handle_1_2_hc_bb_command(HCI_CMD_PKT *hci_cmd_ptr)
{
    UCHAR ret_error = HCI_COMMAND_SUCCEEDED;
    switch(hci_cmd_ptr->cmd_opcode)
    {
#ifdef COMPILE_AFH_HOP_KERNEL
        case HCI_SET_AFH_HOST_CHANNEL_CLASSIFICATION_OPCODE :
             ret_error =
             hci_handle_set_afh_host_channel_classification_command(
                                                    hci_cmd_ptr);
            break;
#ifdef COMPILE_AFH_CLASSIFICATION
        case HCI_READ_AFH_CHANNEL_ASSESMENT_MODE_OPCODE :
            break;
        case HCI_WRITE_AFH_CHANNEL_ASSESMENT_MODE_OPCODE :
             ret_error = hci_handle_write_afh_channel_assessment_command(
                                                hci_cmd_ptr);
             break;
#endif /* COMPILE_AFH_CLASSIFICATION */
#endif /* COMPILE_AFH_HOP_KERNEL */

        case HCI_READ_INQUIRY_SCAN_TYPE_OPCODE :
            break;

        case HCI_READ_PAGE_SCAN_TYPE_OPCODE :
            break;

        case HCI_WRITE_INQUIRY_SCAN_TYPE_OPCODE :
            ret_error = hci_handle_write_inquiry_scan_type_command(hci_cmd_ptr);
            break;

        case HCI_READ_INQUIRY_MODE_OPCODE :
            break;

        case HCI_WRITE_INQUIRY_MODE_OPCODE :
            ret_error = hci_handle_write_inquiry_mode_command(hci_cmd_ptr);
            break;

        case HCI_WRITE_PAGE_SCAN_TYPE_OPCODE :
            ret_error = hci_handle_write_page_scan_type_command(hci_cmd_ptr);
            break;

        default:
             return UNKNOWN_HCI_COMMAND_ERROR;
    } /* end switch(hci_cmd_ptr->cmd_opcode) */
    return ret_error;
}

/**
 * Handles all the Status Parameters commands for the Bluetooth Version 1_2.
 * This function is called from the hci_task.c if the opcode
 * does not match any of the 1.1 Version opcodes.
 *
 * \param hci_cmd_ptr Pointer to HCI_CMD_PKT.
 * \param conn_handle Connection Handle
 *
 * \return HCI_COMMAND_SUCCEEDED or HCI error code.
 *
 */
UCHAR hci_handle_1_2_status_command(HCI_CMD_PKT *hci_cmd_ptr,
                                    UINT16 conn_handle)
{
    UCHAR ret_error = HCI_COMMAND_SUCCEEDED;
    switch(hci_cmd_ptr->cmd_opcode)
    {

#ifdef COMPILE_AFH_HOP_KERNEL
        case HCI_READ_AFH_CHANNEL_MAP_OPCODE:
            ret_error = hci_handle_read_afh_channel_map_command(hci_cmd_ptr, conn_handle);
            break;
#endif  /* #ifdef COMPILE_AFH_HOP_KERNEL */

        default:
#ifdef VER_3_0
            if (IS_BT30)
            {
                ret_error = HCI_HANDLE_3_0_STATUS_COMMAND(hci_cmd_ptr);
                if (ret_error != UNKNOWN_HCI_COMMAND_ERROR)
                {
                    break;
                }
            }
#endif
#ifdef VER_CSA4
            if (IS_BT_CSA4)
            {
                ret_error = HCI_HANDLE_CSA4_STATUS_COMMAND(hci_cmd_ptr);
                if (ret_error != UNKNOWN_HCI_COMMAND_ERROR)
                {
                    break;
                }
            }
#endif
            return UNKNOWN_HCI_COMMAND_ERROR;
    }/* end switch(hci_cmd_ptr->cmd_opcode) */
    return ret_error;
}

/**
 * Prepares the command complete event for all the Version 1_2 HCI commands.
 * This function is called from the hci_generate_command_complete_
 * event function in hci_events.c , this function populates the
 * parameters after the status parameter and returns the length of
 * event packet.
 *
 * \param cmd_opcode Command opcode for which event has to be generated.
 * \param pkt_param  Pointer to the event packet parameter list.
 *
 * \return The total length of all the parameters in the event packet.
 *
 */
UCHAR hci_generate_1_2_command_complete_event(UINT16 cmd_opcode,
        UCHAR *pkt_param, UINT16 ext_param)
{
    UCHAR param_length = 4;
    /* Param length is 4 because event_opcode(1),cmd_opcode(2) and status(1)
     * are populated by the calling function
     */

    switch(cmd_opcode)
    {
        case HCI_READ_INQUIRY_SCAN_TYPE_OPCODE :
            pkt_param[param_length] = lmp_self_device_data.inquiry_scan_type;
            param_length ++;
            break;

        case HCI_WRITE_INQUIRY_SCAN_TYPE_OPCODE :
            break;

        case HCI_READ_INQUIRY_MODE_OPCODE :
            pkt_param[param_length] = lmp_self_device_data.inquiry_mode;
            param_length ++;
            break;

        case HCI_WRITE_INQUIRY_MODE_OPCODE :
            break;

        case HCI_READ_PAGE_SCAN_TYPE_OPCODE :
            pkt_param[param_length] = lmp_self_device_data.page_scan_type;
            param_length ++;
            break;

        case HCI_WRITE_PAGE_SCAN_TYPE_OPCODE :
            break;

        case HCI_SET_AFH_HOST_CHANNEL_CLASSIFICATION_OPCODE :
            break;

        case HCI_READ_AFH_CHANNEL_ASSESMENT_MODE_OPCODE :
            pkt_param[param_length] =
                       lmp_self_device_data.afh_channel_assessment_mode;
            param_length ++;
            break;

        case HCI_WRITE_AFH_CHANNEL_ASSESMENT_MODE_OPCODE :
            break;

        default :
            param_length = (UCHAR)(param_length +
                    HCI_GENERATE_2_1_COMMAND_COMPLETE_EVENT(
                    cmd_opcode, pkt_param, ext_param));
            break;
    }/* end switch(cmd_opcode) */

    return (UCHAR) (param_length - 4);
}

/**
 * Handles the Write Inquiry Scan type command from
 * the host. It returns a command complete after handling the command.
 *
 * \param hci_cmd_ptr Pointer to HCI_CMD_PKT.
 *
 * \return HCI_COMMAND_SUCCEEDED or HCI error code.
 *
 */
UCHAR hci_handle_write_inquiry_scan_type_command(HCI_CMD_PKT *hci_cmd_ptr)
{
    UCHAR offset = 0;
    UCHAR inquiry_scan_type;
    UCHAR status = HCI_COMMAND_SUCCEEDED;

    inquiry_scan_type = hci_cmd_ptr->cmd_parameter[offset];
    offset++;

    /* Validate the command parameters length and value range */
    if (inquiry_scan_type > 1)
    {
        status = INVALID_HCI_COMMAND_PARAMETERS_ERROR;
    }
    else
    {
        lmp_self_device_data.inquiry_scan_type = inquiry_scan_type;

        /* Reprogram scan */
        lc_handle_scan_mode_command();
    }

    return status;
}

/**
 * Validates the params in the write inq mode
 * command and generates a command complete event with the
 * appropriate status.
 *
 * \param hci_cmd_ptr Pointer to HCI_CMD_PKT.
 *
 * \return HCI_COMMAND_SUCCEEDED or HCI error code.
 *
 */
UCHAR hci_handle_write_inquiry_mode_command(HCI_CMD_PKT *hci_cmd_ptr)
{
    UCHAR offset = 0;
    UCHAR inquiry_mode;
    UCHAR status = HCI_COMMAND_SUCCEEDED;

    inquiry_mode = hci_cmd_ptr->cmd_parameter[offset];
    offset++;

    /* Validate the command parameters length and value range */
    if(inquiry_mode > HCI_INQ_RESULT_EVENT_WITH_EIR)
    {
        status = INVALID_HCI_COMMAND_PARAMETERS_ERROR;
    }

#ifndef COMPILE_INQ_RES_EVENT_WITH_RSSI
    if(inquiry_mode == HCI_INQ_RESULT_EVENT_WITH_RSSI)
    {
        status = UNSUPPORTED_FEATURE_OR_PARAMETER_ERROR;
    }
#endif

    if(status == HCI_COMMAND_SUCCEEDED)
    {
        lmp_self_device_data.inquiry_mode = inquiry_mode;
        hci_check_and_enable_eir_recv();
    }

    return status;

}
/**
 * Validates the params in the write page scan
 * type command and generates a command complete event with the
 * appropriate status.
 *
 * \param hci_cmd_ptr Pointer to HCI_CMD_PKT.
 *
 * \return HCI_COMMAND_SUCCEEDED or HCI error code.
 *
 */
UCHAR hci_handle_write_page_scan_type_command(HCI_CMD_PKT *hci_cmd_ptr)
{
    UCHAR offset = 0;
    UCHAR page_scan_type;
    UCHAR status = HCI_COMMAND_SUCCEEDED;

    page_scan_type = hci_cmd_ptr->cmd_parameter[offset];
    offset ++;

    /* Validate the command parameters length and value range */
    if(page_scan_type >= 2)
    {
        status = INVALID_HCI_COMMAND_PARAMETERS_ERROR;
    }
    else
    {
        lmp_self_device_data.page_scan_type = page_scan_type;

        /* Reprogram scan */
        lc_handle_scan_mode_command();
    }

    return status;
}

#ifdef COMPILE_AFH_HOP_KERNEL
/**
 * Validates command parameters for
 * hci_set_afh_host_channel_classification_command and starts
 * updating of remote device AFH mask. [If device is already
 * connected with one or more 1.2 slaves]. As a slave this command
 * Generates a lmp_channel_clasifiction PDU to tell remote Master
 * about new suggested map.
 *
 * \param hci_cmd_ptr Pointer to HCI_CMD_PKT.
 *
 * \return HCI_COMMAND_SUCCEEDED or HCI error code.
 *
 */
UCHAR hci_handle_set_afh_host_channel_classification_command(
                                                HCI_CMD_PKT *hci_cmd_ptr)
{
    UCHAR afh_map[LMP_AFH_MAP_SIZE];
    UCHAR local_feature_master;
    UCHAR local_feature_slave;
    UCHAR n_min = 0;
    UCHAR count1,count2;
    UCHAR temp_var;

    //RT_BT_LOG(RED, BT_FW_HCI_1_2_CMDS_503, 0, 0);

    /* Validation of Parameters */
    /* Check for local device features */
    local_feature_master = lmp_feature_data.feat_page0[5];
    local_feature_slave = lmp_feature_data.feat_page0[4];

    if (!(local_feature_master & (AFH_CAPABLE_MASTER |
                                  AFH_CLASSIFICATION_MASTER |
                                  AFH_CLASSIFICATION_SLAVE)))
    {
        return UNSUPPORTED_FEATURE_OR_PARAMETER_ERROR;
    }

    /* Extract map from command packet to temporary buffer */
    memcpy(&afh_map[0],&hci_cmd_ptr->cmd_parameter[0],LMP_AFH_MAP_SIZE);

    /* Check Number of unknown channels in this map */
    for (count1 = 0;count1 < LMP_AFH_MAP_SIZE; count1++)
    {
        temp_var = 1;
        for(count2 = 0;count2 < 8; count2++)
        {
            if(temp_var & afh_map[count1])
            {
                n_min++;
            }
            temp_var <<= 1;
        }
    }

    /* Check for length of parameters and given length in the command packet */
    /* Check or minimum number of channels in Map */
    /* Check Map does not fall in the reserved area of channels */
    if((n_min < LMP_MIN_NUM_OF_AFH_CHANNEL_ALLOWED) ||
       (afh_map[LMP_AFH_MAP_SIZE - 1] & 0x80))
    {
        return INVALID_HCI_COMMAND_PARAMETERS_ERROR;
    }

#ifdef VER_CSA4
    if (IS_BT_CSA4)
    {
#ifdef _SUPPORT_CSB_TRANSMITTER_
        /* Save host classification for csa variable */
        bt_csb_update_host_afh_ch_classification(&afh_map[0]);
#endif
    }
#endif

    /* Save host classification in global variable */
    lmp_update_host_afh_ch_classification(&afh_map[0]);

    return HCI_COMMAND_SUCCEEDED;
}


/**
 * Validates command parameter and return appropriete
 * corresponding to connection handle.
 *
 * \param hci_cmd_ptr Pointer to HCI_CMD_PKT.
 * \param connection_handle Connection Handle.
 *
 * \return HCI_COMMAND_SUCCEEDED or HCI error code.
 *
 */
UCHAR   hci_handle_read_afh_channel_map_command(HCI_CMD_PKT *hci_cmd_ptr,
                                                UINT16 connection_handle)
{
	UINT16 ce_index;
	UCHAR status = HCI_COMMAND_SUCCEEDED;

	UCHAR local_feature_master;
	UCHAR local_feature_slave;
	UCHAR remote_feature_master;
	UCHAR remote_feature_slave;

	LMP_CONNECTION_ENTITY *ce_ptr;

	if(LMP_GET_CE_INDEX_FROM_CONN_HANDLE(connection_handle, &ce_index) !=
		API_SUCCESS)
	{
		status = NO_CONNECTION_ERROR;
		return status;
	}

	ce_ptr = &lmp_connection_entity[ce_index];

	/* Check for local and remote device features */
	local_feature_master = lmp_feature_data.feat_page0[5];
	local_feature_slave = lmp_feature_data.feat_page0[4];

	remote_feature_master = ce_ptr->feat_page0[5];
	remote_feature_slave =  ce_ptr->feat_page0[4];

	/* Validate local and remote features. */
	if (ce_ptr->remote_dev_role == MASTER)
	{
		/* Check the self device AFH feature as slave and remote device AFH
		feature as master. */

		if ( !( (local_feature_slave & AFH_CAPABLE_SLAVE) &&
			(remote_feature_master & AFH_CAPABLE_MASTER) ) )
		{
			status =  UNSUPPORTED_FEATURE_OR_PARAMETER_ERROR;
		}
	}
	else
	{
		/* Check the remote device AFH feature as slave and self device AFH
		feature as master. */

		if ( !( (local_feature_master & AFH_CAPABLE_MASTER) &&
			(remote_feature_slave & AFH_CAPABLE_SLAVE) ) )
		{
			status =  UNSUPPORTED_FEATURE_OR_PARAMETER_ERROR;
		}
	}

	return status;
}

#ifdef COMPILE_AFH_CLASSIFICATION
/**
 * Writes afh channel classification.
 *
 * \param hci_cmd_ptr Pointer to HCI_CMD_PKT.
 *
 * \return HCI_COMMAND_SUCCEEDED or HCI error code.
 *
 */
UCHAR hci_handle_write_afh_channel_assessment_command(HCI_CMD_PKT
                                                        *hci_cmd_ptr)
{
    UCHAR local_feature_master;
    UCHAR local_feature_slave;

#ifndef COMPILE_CHANNEL_ASSESSMENT
    UCHAR map[LMP_AFH_MAP_SIZE];
#endif /* COMPILE_CHANNEL_ASSESSMENT */

    /* Validation of Parameters */
    /* Check for local device features */
    local_feature_master = lmp_feature_data.feat_page0[5];
    local_feature_slave = lmp_feature_data.feat_page0[4];

    if (!(local_feature_master & (AFH_CAPABLE_MASTER |
                                  AFH_CLASSIFICATION_MASTER |
                                  AFH_CLASSIFICATION_SLAVE)))
    {
        return UNSUPPORTED_FEATURE_OR_PARAMETER_ERROR;
    }

    /* Check if the channel assesment is already disabled or enables , in that
     * case just dont do anything
     */
    if(hci_cmd_ptr->cmd_parameter[0] ==
        lmp_self_device_data.afh_channel_assessment_mode)
    {
        return HCI_COMMAND_SUCCEEDED;
    }

    /* Update afh_channel_assessment_mode */
    if(hci_cmd_ptr->cmd_parameter[0] == AFH_ENABLE)
    {
        lmp_self_device_data.afh_channel_assessment_mode = AFH_ENABLE;
    }
    else /* Disable local assessment. */
    {
        lmp_self_device_data.afh_channel_assessment_mode = AFH_DISABLE;
    }

    return HCI_COMMAND_SUCCEEDED;
}

#endif /* COMPILE_AFH_CLASSIFICATION */
#endif /* COMPILE_AFH_HOP_KERNEL */

#if defined(ENABLE_SCO) || defined(COMPILE_ESCO)
/**
 * Extracts the supported air modes from the feature bytes. Only the lower 4
 * bits are valid and the rest will be set to 0. The bit details are:
 *
 * \code
 * bit0 -> CVSD synchronous data
 * bit1 -> u-law log synchronous data
 * bit2 -> A-law log synchronous
 * bit3 -> Transparent synchronous data
 * \endcode
 *
 * \param features Feature byte array.
 *
 * \return Extracted air_mode_bitmap.
 */
UCHAR lmp_extract_air_mode_bitmap(const UCHAR* features)
{
    UCHAR air_mode_bitmap;

    air_mode_bitmap = (UCHAR)(features[2] & FEATURE_BYTE_2_AIR_MODES_MASK);
    air_mode_bitmap = (UCHAR)(air_mode_bitmap |
                      ((features[1] & FEATURE_BYTE_1_AIR_MODES_MASK) >> 5));

    return air_mode_bitmap;
}

/**
 * Extracts synchronous packet types supported by a device from its feature
 * bytes.
 *
 * \param features Feature byte array.
 *
 * \return Extracted synchronous packet types in 1.2 HCI [Setup_Sync] format.
 */
UINT16 lmp_extract_sync_pkt_types(const UCHAR* features)
{
    UINT16 sync_pkt_types = 0;

#ifdef ENABLE_SCO
    sync_pkt_types = (UINT16)((features[1] >> 3) & ALL_HCI_1_2_SCO_PKT_TYPES);
#endif

#ifdef COMPILE_ESCO
    sync_pkt_types = (UINT16)( sync_pkt_types | ((features[4] << 4) &
                          (ALL_HCI_1_2_ESCO_PKT_TYPES)) );    /* EV4 & EV5 */
    sync_pkt_types = (UINT16) ( sync_pkt_types | (((features[3] &   /* EV3 */
                                 LMP_ESCO_MANDATORY_FEATURE) >> 4)) );
    if (features[5] & EDR_ESCO_2MBPS_FEATURE)
    {
        sync_pkt_types |= HCI_2_EV3;

        if (features[5] & EDR_ESCO_3_SLOT_FEATURE)
        {
            sync_pkt_types |= HCI_2_EV5;
        }

        if (features[5] & EDR_ESCO_3MBPS_FEATURE)
        {
            sync_pkt_types |= HCI_3_EV3;
            if (features[5] & EDR_ESCO_3_SLOT_FEATURE)
            {
                sync_pkt_types |= HCI_3_EV5;
            }
        } /* if 3mbps is enabled */
    } /* if 2mbps is enabled */
#endif /* COMPILE_ESCO */

	SRI_PRINT(EXTRACTED_SYNC_PKT_TYPES, 1, sync_pkt_types);

    return sync_pkt_types;
}

/**
 * Validates the \a host_params against the Spec requirements and returns an
 * error code appropriately.
 *
 * \param ce_index ACL connection entity index.
 * \param host_params Parameters to be validated.
 *
 * \return HCI_COMMAND_SUCCEEDED, if the parameters are valid. A valid error
 *         code, otherwise.
 */
UCHAR hci_validate_sync_params(UINT16 ce_index,
        const HCI_SYNC_PARAMS* host_params)
{
    UINT16 input_coding;
    UCHAR hci_air_mode;
    UCHAR self_air_mode_bitmap;
    UCHAR rem_air_mode_bitmap;
    LMP_CONNECTION_ENTITY* ce_ptr;
    UINT8 input_data_format;
    UINT8 linear_sample_16bit;

	ce_ptr = &lmp_connection_entity[ce_index];

    hci_air_mode = (UCHAR)(host_params->voice_setting & AIR_MODE_MASK);
    input_coding = (UINT16)(host_params->voice_setting & INPUT_CODING_MASK);
    input_data_format = (host_params->voice_setting >> 6) & 0x03;
    linear_sample_16bit = (host_params->voice_setting >> 5) & 0x01;

    if (input_coding == RESERVED_FOR_FUTURE)
    {
        return INVALID_HCI_COMMAND_PARAMETERS_ERROR;
    }

	self_air_mode_bitmap =
		lmp_extract_air_mode_bitmap(lmp_feature_data.feat_page0);

	if (!lmp_is_air_mode_supported(self_air_mode_bitmap, hci_air_mode))
    {
        return INVALID_HCI_COMMAND_PARAMETERS_ERROR;
    }

    rem_air_mode_bitmap = lmp_extract_air_mode_bitmap(ce_ptr->feat_page0);
    if (!lmp_is_air_mode_supported(rem_air_mode_bitmap, hci_air_mode))
    {
        return UNSUPPORTED_REMOTE_FEATURE_ERROR;
    }

	RT_BT_LOG(WHITE,BT_VOICE_SETTING_PARAMETER,6 ,
                self_air_mode_bitmap, rem_air_mode_bitmap,
				hci_air_mode, input_coding, input_data_format, linear_sample_16bit);

    if (bb_new_program_codec(hci_air_mode, input_coding, ce_index,
                        input_data_format, linear_sample_16bit) != API_SUCCESS)
    {
        return INVALID_HCI_COMMAND_PARAMETERS_ERROR;
    }

    if (host_params->max_latency < MAX_LATENCY_MIN_VAL)
    {
        return INVALID_HCI_COMMAND_PARAMETERS_ERROR;
    }

    return HCI_COMMAND_SUCCEEDED;
}

#ifdef ENABLE_SCO
/**
 * Validates the \a host_params against the Spec requirements and returns an
 * error code appropriately (Specific for SCO).
 *
 * \param ce_index ACL connection entity index.
 * \param host_params Parameters to be validated.
 *
 * \return HCI_COMMAND_SUCCEEDED, if the parameters are valid. A valid error
 *         code, otherwise.
 */
UCHAR hci_validate_sco_params(UINT16 ce_index,
        const HCI_SYNC_PARAMS* host_params)
{
    UCHAR ret_error;

    ret_error = hci_validate_sync_params(ce_index, host_params);

    if (ret_error != HCI_COMMAND_SUCCEEDED)
    {
        return ret_error;
    }

    if ((host_params->retx_effort != 0) && (host_params->retx_effort != 0xFF))
    {
        return INVALID_HCI_COMMAND_PARAMETERS_ERROR;
    }

    if (((host_params->tx_bandwidth != 0xFFFFFFFF)
                && (host_params->tx_bandwidth != 0x1F40))
            || ((host_params->rx_bandwidth != 0xFFFFFFFF)
                && (host_params->rx_bandwidth != 0x1F40)))
    {
		RT_BT_LOG(RED, BT_FW_HCI_1_2_CMDS_975, 2,
			host_params->tx_bandwidth, host_params->rx_bandwidth);
        return INVALID_HCI_COMMAND_PARAMETERS_ERROR;
    }

    return HCI_COMMAND_SUCCEEDED;
}
#endif /* ENABLE_SCO */

#ifdef COMPILE_ESCO
/**
 * Validates the \a host_params against the Spec requirements and returns an
 * error code appropriately (Specific for ESCO).
 *
 * \param ce_index ACL connection entity index.
 * \param host_params Parameters to be validated.
 *
 * \return HCI_COMMAND_SUCCEEDED, if the parameters are valid. A valid error
 *         code, otherwise.
 */
UCHAR hci_validate_esco_params(UINT16 ce_index,
        const HCI_SYNC_PARAMS* host_params)
{
    UCHAR ret_error;

    ret_error = hci_validate_sync_params(ce_index, host_params);

    if (ret_error != HCI_COMMAND_SUCCEEDED)
    {
        return ret_error;
    }

    if((host_params->retx_effort > 2) && (host_params->retx_effort != 0xFF))
    {
        return INVALID_HCI_COMMAND_PARAMETERS_ERROR;
    }

    return HCI_COMMAND_SUCCEEDED;
}
#endif /* COMPILE_ESCO */

#ifdef COMPILE_ESCO
/**
 * Initiates the ESCO link creation procedure with the given parameters.
 *
 * \param ce_index ACL connection entity index.
 * \param host_params Parameters given by the host for this connection.
 *
 * \return HCI_COMMAND_SUCCEEDED, if the operation was successful. A valid
 *         error code, otherwise.
 */
UCHAR hci_initiate_new_esco_link(UINT16 ce_index,
        HCI_SYNC_PARAMS* host_params)
{
    UCHAR esco_am_addr = 0xFF;
    UCHAR ret_error = HCI_COMMAND_SUCCEEDED;
    UINT16 esco_ce_index;
    LMP_ESCO_CONNECTION_ENTITY* esco_ce_ptr;
    LMP_CONNECTION_ENTITY* ce_ptr;
    UCHAR parameter_list[LMP_ESCO_LINK_REQ_LEN];
    UINT8 esco_debug_count = 0;
    UINT8 my_role = 0xFF;

    do
    {
        ret_error = hci_validate_esco_params(ce_index, host_params);
        if (ret_error != HCI_COMMAND_SUCCEEDED)
        {
            esco_debug_count = 1;
            break;
        }

#ifdef ENABLE_SCO
        if(lmp_self_device_data.total_no_of_sco_conn)
        {
            esco_debug_count = 2;

          /* SCO and ESCO can not co-exist (limitation) */
            ret_error = HOST_REJECTED_LIMITED_RESOURCES_ERROR;
            break;
        }
#endif

        ret_error = lmp_check_for_allowing_new_esco_connection();

        if (ret_error != HCI_COMMAND_SUCCEEDED)
        {
            esco_debug_count = 3;
            break;
        }

        if(lmp_self_device_data.number_of_esco_connections >= LMP_MAX_ESCO_CONNECTIONS)
        {
            esco_debug_count = 4;
            ret_error = MAX_NUMBER_OF_CONNECTIONS_ERROR;
            break;
        }

        ce_ptr = &lmp_connection_entity[ce_index];

        if (ce_ptr->remote_dev_role == SLAVE)
        {
            my_role = MASTER;
            if(lmp_allocate_esco_am_addr(&esco_am_addr, &esco_ce_index,
                    ce_ptr->phy_piconet_id) == API_FAILURE)
            {
                esco_debug_count = 5;
                ret_error = MAX_NUMBER_OF_CONNECTIONS_ERROR;
                break;
            }
        }
        else
        {
            my_role = SLAVE;
            esco_ce_index = lmp_allocate_esco_entity_from_ce_database();
            if(esco_ce_index == BT_FW_ERROR)
            {
                esco_debug_count = 6;
                ret_error = MAX_NUMBER_OF_CONNECTIONS_ERROR;
                break;
            }

            lmp_esco_connection_entity[esco_ce_index].esco_handle = 0;
        }

        ce_ptr->is_esco_channel = TRUE;
        ce_ptr->esco_ce_idx = esco_ce_index;

        esco_ce_ptr = &lmp_esco_connection_entity[esco_ce_index];

        /* Update the provided data in the LMP level data structure */
        esco_ce_ptr->ce_index = ce_index;
        esco_ce_ptr->tx_bandwidth = host_params->tx_bandwidth;
        esco_ce_ptr->rx_bandwidth = host_params->rx_bandwidth;
        esco_ce_ptr->max_latency = host_params->max_latency;
        esco_ce_ptr->voice_setting = host_params->voice_setting;
        esco_ce_ptr->retransmission_effort = host_params->retx_effort;
        esco_ce_ptr->packet_type = host_params->pkt_types;

        /* Update states */
        esco_ce_ptr->status = ADDING_ESCO_LINK;
        ce_ptr->temp_ce_status = ce_ptr->ce_status;
        lmp_set_ce_status(ce_index, LMP_ADDING_ESCO_LINK);

        /* Generate viable eSCO parameters from the host given data */
        ret_error = lmp_generate_init_esco_params(esco_ce_index);
        if(ret_error != HCI_COMMAND_SUCCEEDED)
        {
            /* Rollback the changes */
            lmp_release_esco_resources(ce_index, esco_ce_index);
            lmp_set_ce_status(ce_index, ce_ptr->temp_ce_status);

            esco_debug_count = 7;
            break;
        }

        /* Populate the PDU structure */
        lmp_get_esco_params_from_ce(&parameter_list[0], esco_ce_index);

#ifdef _DAPE_TEST_BLOCK_ACL_2_SLOTS_WHEN_SCO
        UINT16 bb_reg;
        bb_reg = BB_read_baseband_register(PICONET4_INFO_REGISTER);
        bb_reg |= (UINT16)(0x0040);
        BB_write_baseband_register(PICONET4_INFO_REGISTER, bb_reg);
#endif

#ifdef _DAPE_TEST_SEND_MAX_SLOT_BEFORE_ESCO_CREATED
        lmp_self_device_data.adding_new_esco_conn = TRUE;
        lmp_send_max_slot_pdu_to_all_devices();
#endif
        if (lmp_generate_pdu(ce_index, parameter_list, LMP_ESCO_LINK_REQ_LEN,
                    SELF_DEV_TID, LMP_NO_STATE_CHANGE) != API_SUCCESS)
        {
            /* Rollback the changes */
            lmp_release_esco_resources(ce_index, esco_ce_index);
            lmp_set_ce_status(ce_index, ce_ptr->temp_ce_status);

            esco_debug_count = 8;
            ret_error = UNSPECIFIED_ERROR;
            break;
        }
    }
    while (0);

    RT_BT_LOG(RED, BT_FW_HCI_1_2_CMDS_1152, 5, ret_error, esco_debug_count,
                                        ce_index, esco_am_addr, my_role);

    return ret_error;
}

/**
 * Initiates the ESCO renegotiation procedure.
 *
 * \param esco_ce_index ESCO connection entity index.
 * \param host_params Parameters given by the host for this renegotiation.
 *
 * \return HCI_COMMAND_SUCCEEDED, if the operation was successful. A valid
 *         error code, otherwise.
 */
UCHAR hci_handle_esco_renegotiation(UINT16 esco_ce_index,
                                    HCI_SYNC_PARAMS* host_params)
{
    LMP_ESCO_CONNECTION_ENTITY* esco_ce_ptr;
    LMP_CONNECTION_ENTITY* ce_ptr;
    UCHAR parameter_list[LMP_ESCO_LINK_REQ_LEN];
    UCHAR ret_error;

    esco_ce_ptr = &lmp_esco_connection_entity[esco_ce_index];
    ce_ptr = &lmp_connection_entity[esco_ce_ptr->ce_index];
    ret_error = hci_validate_esco_params(esco_ce_ptr->ce_index, host_params);

    if (ret_error != HCI_COMMAND_SUCCEEDED)
    {
        return ret_error;
    }

    /* Tx Bandwidth, Rx Bandwidth and Voice_Setting can not be renegotiated */
    if (host_params->tx_bandwidth != esco_ce_ptr->tx_bandwidth
            || host_params->rx_bandwidth != esco_ce_ptr->rx_bandwidth
            || host_params->voice_setting != esco_ce_ptr->voice_setting)
    {
        return INVALID_HCI_COMMAND_PARAMETERS_ERROR;
    }

    /* Update the provided data in the LMP level data structure */
    temp_esco_connection_entity.tx_bandwidth = host_params->tx_bandwidth;
    temp_esco_connection_entity.rx_bandwidth = host_params->rx_bandwidth;
    temp_esco_connection_entity.max_latency = host_params->max_latency;
    temp_esco_connection_entity.voice_setting = host_params->voice_setting;
    temp_esco_connection_entity.retransmission_effort =
                                            host_params->retx_effort;
    temp_esco_connection_entity.packet_type = host_params->pkt_types;
    temp_esco_connection_entity.lt_addr = esco_ce_ptr->lt_addr;
    temp_esco_connection_entity.esco_handle = esco_ce_ptr->esco_handle;

    /* Update states */
    esco_ce_ptr->status = CHANGING_ESCO_PARAMS;
    ce_ptr->temp_ce_status = ce_ptr->ce_status;
	lmp_set_ce_status(esco_ce_ptr->ce_index, LMP_CHANGING_ESCO_PARAMS);

    lmp_local_dev_init_esco_param_change = TRUE;

    /* Generate viable eSCO parameters from the host given data */
    ret_error = lmp_generate_init_esco_params(esco_ce_index);
    if(ret_error != HCI_COMMAND_SUCCEEDED)
    {
        /* Rollback state changes */
        esco_ce_ptr->status = ESCO_CONNECTED;

		lmp_set_ce_status(esco_ce_ptr->ce_index, ce_ptr->temp_ce_status);

		lmp_local_dev_init_esco_param_change = FALSE;

        return ret_error;
    }

    /* Populate the PDU structure */
    lmp_get_esco_params_from_ce(&parameter_list[0], esco_ce_index);

    if (lmp_generate_pdu(esco_ce_ptr->ce_index, parameter_list,
                LMP_ESCO_LINK_REQ_LEN, SELF_DEV_TID, LMP_NO_STATE_CHANGE)
            != API_SUCCESS)
    {
        /* Rollback state changes */
        esco_ce_ptr->status = ESCO_CONNECTED;

		lmp_set_ce_status(esco_ce_ptr->ce_index, ce_ptr->temp_ce_status);

        lmp_local_dev_init_esco_param_change = FALSE;

        return UNSPECIFIED_ERROR;
    }

    return HCI_COMMAND_SUCCEEDED;
}
#endif /* COMPILE_ESCO */

#ifdef ENABLE_SCO
/**
 * Initiates the SCO link creation procedure with the given parameters.
 *
 * \param ce_index ACL connection entity index.
 * \param host_params Parameters given by the host for this connection.
 * \param gen_conn_complete Indicates whether to generate
 *                          Conn_Complete_Evt(TRUE) or
 *                          Sync_Conn_Complete_evt(FALSE).
 *
 * \return HCI_COMMAND_SUCCEEDED, if the operation was successful. A valid
 *         error code, otherwise.
 */
UCHAR hci_initiate_new_sco_link(UINT16 ce_index,
        HCI_SYNC_PARAMS* host_params, UCHAR gen_conn_complete)
{
    UINT16 pkt_types;
    UINT16 max_latency;
    UCHAR ret_error;
    UCHAR air_mode;

    ret_error = hci_validate_sco_params(ce_index, host_params);
    if (ret_error != HCI_COMMAND_SUCCEEDED)
    {
        return ret_error;
    }

    pkt_types = SCO_PKT_TYPE_1_2_HCI_TO_1_1_HCI(host_params->pkt_types);
    max_latency = TIMER_VAL_TO_SLOT_VAL(host_params->max_latency);
    air_mode = (UCHAR)(host_params->voice_setting & AIR_MODE_MASK);

    ret_error = lmp_initiate_new_sco_connection(ce_index, max_latency,
            pkt_types, air_mode, gen_conn_complete);

    return ret_error;
}
#endif /* ENABLE_SCO */

/**
 * Determines whether to initiate a SCO link creation procedure or ESCO link
 * creation procedure based on the parameters(\a host_params) given.
 *
 * \param ce_index ACL connection entity index.
 * \param host_params Parameters given by the host for this connection.
 * \param p_link_type Pointer to Link type (SCO_LINK or ESCO_LINK - Out
 *                    parameter).
 * \param gen_conn_complete Indicates whether to generate
 *                          Conn_Complete_Evt(TRUE) or
 *                          Sync_Conn_Complete_evt(FALSE).
 *
 * \return HCI_COMMAND_SUCCEEDED, if the operation was successful. A valid
 *         error code, otherwise.
 */
UCHAR hci_handle_new_sync_link(UINT16 ce_index,
        HCI_SYNC_PARAMS* host_params, HCI_LINK_TYPE* p_link_type,
        UCHAR gen_conn_complete)
{
    UINT16 self_pkt_types;
    UINT16 rem_pkt_types;
    UINT16 pkt_types;
    UCHAR ret_error;
    LMP_CONNECTION_ENTITY* ce_ptr;

    ce_ptr = &lmp_connection_entity[ce_index];

    self_pkt_types = lmp_extract_sync_pkt_types(&lmp_feature_data.feat_page0[0]);

    rem_pkt_types = lmp_extract_sync_pkt_types(&ce_ptr->feat_page0[0]);

    // self_pkt_types and rem_pkt_types: 1 is enable and 0 is disable
    // host_params->pkt_types according to spec(the most significant 4 bit, 0 is enable);
    /* Determine the allowed packet types for this connection */
    pkt_types = (UINT16)(self_pkt_types & rem_pkt_types
            & (host_params->pkt_types ^ HCI_ESCO_EDR_PKT_TYPE_UMASK));

    RT_BT_LOG(GREEN, BT_FW_HCI_1_2_CMDS_1332, 5,
              self_pkt_types, rem_pkt_types, host_params->pkt_types,
              pkt_types, ALL_HCI_ESCO_PKT_TYPES);

#ifndef _DAPE_TEST_NO_SET_ESCO_RETRY_TO_0
#ifdef _CCH_PCM_CHOOSE_ESCO_
    if( (sync_link_codec_state == OVER_CODEC)&&
        (sync_link_codec_type == PCM_CODEC) )
    {
        host_params->retx_effort = 0;
    }

#endif
#endif

#ifdef COMPILE_ESCO
    /* Initiate ESCO link if any eSCO pkt is selected*/
    if (pkt_types & ALL_HCI_ESCO_PKT_TYPES)
    {
        //RT_BT_LOG(GREEN, BT_FW_HCI_1_2_CMDS_1339, 0, 0);

///// dape: XOR has been operated before when deciding pkt_types
#ifndef _DAPE_TEST_CORRECT_ESCO_PKT_DECISION
        host_params->pkt_types = (UINT16)(pkt_types ^ HCI_ESCO_EDR_PKT_TYPE_UMASK);
#endif

        *p_link_type = ESCO_LINK;

        ret_error = hci_initiate_new_esco_link(ce_index, host_params);
    }
    else
#endif

#ifdef ENABLE_SCO
    /* Initiate SCO link if only SCO pkt is selected*/
    if (pkt_types & ALL_HCI_SCO_PKT_TYPES)
    {
        host_params->pkt_types = pkt_types;
        *p_link_type = SCO_LINK;
        ret_error = hci_initiate_new_sco_link(ce_index, host_params,
                gen_conn_complete);
    }
    else
#endif /* ENABLE_SCO */
    {
        *p_link_type = SCO_LINK;    /* Select some link_type */
        /* No common packet types are available */
        ret_error = INVALID_HCI_COMMAND_PARAMETERS_ERROR;
    }

    return ret_error;
}

/**
 * Handles HCI_SETUP_SYNCHRONOUS_CONNECTION_COMMAND from the host. It
 * determines whether the given set of parameters is for ESCO renegotiation or
 * for creating a new synchronous link and performs the action accordingly.It generates
 * the appropriate Status_Evt, and returns HCI_COMMAND_SUCCEEDED for valid
 * parameters or disallowed commands.Otherwise it returns the appropiate error code
 * for the invalid parameters.
 *
 * \param hci_cmd_ptr HCI Command Packet pointer.
 * \param p_ce_index Pointer to ACL Connection entity index if new conn, else
 *                   points to eSCO Connection entity. It will be used
 *                   by the caller to generate [Sync]_Conn_Complete_Evt in
 *                   case of any error, or [Sync]_Conn_Change_Evt.
 * \param p_link_type Pointer to link type. It will have either SCO_LINK or
 *                    ESCO_LINK as value. It is also used to generate event
 *                    in case of any failure.
 * \param gen_conn_complete Indicates whether to generate
 *                          Conn_Complete_Evt(TRUE) or
 *                          Sync_Conn_Complete_evt(FALSE).
 * \param sync_link. Pointer to sync_link_status. It will be set by the callee
 *                  to indicate new or existing sync connection if the function
 *                  returns an error code.
 *
 * \return HCI_COMMAND_SUCCEEDED, if the operation was successful. A valid
 *         error code, otherwise.
 */
UCHAR hci_handle_setup_sync_conn_command(HCI_CMD_PKT *hci_cmd_ptr,
        UINT16* p_ce_index, HCI_LINK_TYPE* p_link_type,
        UCHAR gen_conn_complete, SYNC_LINK_STATUS* sync_link)
{
    UINT16 connection_handle;
    UINT16 acl_ce_index;
    UINT16 sync_ce_index;
    HCI_SYNC_PARAMS host_params;
    UCHAR ret_error;
    LMP_CONNECTION_ENTITY* ce_ptr;

    *sync_link = NONE;
    BT_FW_EXTRACT_16_BITS(connection_handle, &hci_cmd_ptr->cmd_parameter[0]);

	RT_BT_LOG(GREEN, HCI_MSG_SETUP_SYNC_LINK, 1, connection_handle);

#ifdef COMPILE_ESCO
    if (lmp_get_esco_ce_index_from_conn_handle(connection_handle,
                &sync_ce_index) == API_SUCCESS) /* Is eSCO connection? */
    {
		//RT_BT_LOG(GREEN, BT_FW_HCI_1_2_CMDS_1414, 2, sync_ce_index,
        //                     lmp_esco_connection_entity[sync_ce_index].status);

        if (lmp_esco_connection_entity[sync_ce_index].status != ESCO_CONNECTED)
        {
            /* Cannot re-negotiate eSCO parameters now */
            hci_generate_command_status_event(hci_cmd_ptr->cmd_opcode,
                    COMMAND_DISALLOWED_ERROR);

            return HCI_COMMAND_SUCCEEDED;
        }

		acl_ce_index = lmp_esco_connection_entity[sync_ce_index].ce_index;
        *sync_link = EXISTING_ESCO_LINK;
    }
    else
#endif

    if(LMP_GET_CE_INDEX_FROM_CONN_HANDLE(connection_handle, &acl_ce_index)
            == API_SUCCESS)
    {
		//RT_BT_LOG(GREEN, BT_FW_HCI_1_2_CMDS_1435, 1, acl_ce_index);
        *sync_link = NEW_SYNC_LINK;
    }

#ifdef ENABLE_SCO
    else if (lmp_get_sco_ce_index_from_conn_handle(connection_handle,
                &sync_ce_index) == API_SUCCESS)
    {
		//RT_BT_LOG(GREEN, BT_FW_HCI_1_2_CMDS_1443, 1, sync_ce_index);

        /* Setup Sync Command can not be used to change the parameters
         * of the existing SCO link.
         */
        hci_generate_command_status_event(hci_cmd_ptr->cmd_opcode,
                COMMAND_DISALLOWED_ERROR);

        return HCI_COMMAND_SUCCEEDED;
    }
#endif
    else
    {
		//RT_BT_LOG(GREEN, BT_FW_HCI_1_2_CMDS_1455, 0, 0);

        /* The provided Connection Handle is not valid (SCO/ESCO/ACL) */
        hci_generate_command_status_event(hci_cmd_ptr->cmd_opcode,
                NO_CONNECTION_ERROR);

        return HCI_COMMAND_SUCCEEDED;
    }
#ifdef SECURE_CONN_NO_SCO
        BZ_AUTH_LINK_PARAMS* auth;
        auth = lmp_connection_entity[acl_ce_index].auth;
#ifdef _SECURE_CONN_TEST_LOG
        ///// dape test
        RT_BT_LOG(GREEN, DAPE_TEST_LOG522, 2,auth->super_state,
        auth->sub_state);
#endif
        if ((*sync_link) == NEW_SYNC_LINK)
        {
//            if ((auth->secure_conn_enabled) && (auth->super_state > 11))
            if ((auth->secure_conn_enabled) && (bz_auth_is_link_encrypted(acl_ce_index)))
            {
                UINT16 self_pkt_types;
                UINT16 rem_pkt_types;
                UINT16 pkt_types;
                ce_ptr = &lmp_connection_entity[acl_ce_index];

                self_pkt_types =
                    lmp_extract_sync_pkt_types(&lmp_feature_data.feat_page0[0]);
                rem_pkt_types =
                    lmp_extract_sync_pkt_types(&ce_ptr->feat_page0[0]);
                BT_FW_EXTRACT_16_BITS(host_params.pkt_types,
                                     &hci_cmd_ptr->cmd_parameter[15]);

                pkt_types = (UINT16)(self_pkt_types & rem_pkt_types
                    & (host_params.pkt_types ^ HCI_ESCO_EDR_PKT_TYPE_UMASK));

//RT_BT_LOG(BLUE, DAPE_TEST_LOG525, 6,self_pkt_types,
//rem_pkt_types, host_params.pkt_types, host_params.pkt_types ^ HCI_ESCO_EDR_PKT_TYPE_UMASK,
//pkt_types, (pkt_types & ALL_HCI_ESCO_PKT_TYPES));

                if ((!(pkt_types & ALL_HCI_ESCO_PKT_TYPES)) &&
                   (pkt_types & ALL_HCI_SCO_PKT_TYPES))
                {
                    hci_generate_command_status_event(hci_cmd_ptr->cmd_opcode,
                    HOST_REJECTED_SECURITY_REASONS_ERROR);

                    return HCI_COMMAND_SUCCEEDED;
                }
            }
        }
#endif
    hci_generate_command_status_event(hci_cmd_ptr->cmd_opcode,
            HCI_COMMAND_SUCCEEDED);

    if ((*sync_link) == EXISTING_ESCO_LINK)
    {
        *p_link_type = ESCO_LINK;
        *p_ce_index = sync_ce_index;
    }
    else
    {
        *p_link_type = SCO_LINK;
        *p_ce_index = acl_ce_index;
    }

    ce_ptr = &lmp_connection_entity[acl_ce_index];

	RT_BT_LOG(GREEN, BT_FW_HCI_1_2_CMDS_1478, 5,
		*p_link_type, acl_ce_index, ce_ptr->ce_status, *sync_link, *p_ce_index);

    switch (ce_ptr->ce_status)
    {
        case LMP_CONNECTED:
#ifdef COMPILE_SNIFF_MODE
        case LMP_SNIFF_MODE:    /* Fall through */
#endif
#ifdef _DAPE_ALLOW_SYNC_LINK_FOR_WIN8
        case LMP_ESCO_DISCONNECTING:    /* Fall through */
#endif
            break;

        default:
            return COMMAND_DISALLOWED_ERROR;
    }

#ifdef _BRUCE_TEST_SCO
    //hci_cmd_ptr->cmd_parameter[14] = 0;//no retransmission;
    hci_cmd_ptr->cmd_parameter[15] = 0xC7;//no retransmission;
    hci_cmd_ptr->cmd_parameter[16] = 0x03;//no retransmission;
#endif

    /* Extract the information from the HCI Command Packet. */
    BT_FW_EXTRACT_32_BITS(host_params.tx_bandwidth,
                                 &hci_cmd_ptr->cmd_parameter[2]);
    BT_FW_EXTRACT_32_BITS(host_params.rx_bandwidth,
                                 &hci_cmd_ptr->cmd_parameter[6]);
    BT_FW_EXTRACT_16_BITS(host_params.max_latency,
                          &hci_cmd_ptr->cmd_parameter[10]);
    BT_FW_EXTRACT_16_BITS(host_params.voice_setting,
                          &hci_cmd_ptr->cmd_parameter[12]);
    BT_FW_EXTRACT_16_BITS(host_params.pkt_types,
                                 &hci_cmd_ptr->cmd_parameter[15]);
    host_params.retx_effort = hci_cmd_ptr->cmd_parameter[14];

#ifdef _BRUCE_MSBC_IS_RESTRICTED_PKT_TYPE
        if (lmp_esco_over_codec == TRUE)
        {
            g_voice_setting = host_params.voice_setting;
        }
#endif

#ifndef BZ_2_1_2_CODEC
	//{{ added by guochunxia for debug
//	host_params.max_latency = 0x0a;
//}}
	lmp_self_device_data.voice_setting = host_params.voice_setting;
	lmp_self_device_data.lmp_air_mode =
		lmp_convert_air_mode((UCHAR)(lmp_self_device_data.voice_setting & AIR_MODE_MASK),
		HCI_LAYER, LMP_LAYER);
#endif

	RT_BT_LOG(GREEN, BT_FW_HCI_1_2_CMDS_1513, 7,
		host_params.tx_bandwidth, host_params.rx_bandwidth, host_params.max_latency,
		host_params.voice_setting, host_params.pkt_types, host_params.retx_effort,
		lmp_self_device_data.lmp_air_mode);

#ifdef SCO_REFACTORING_DEBUG
	SRI_PRINT(SETUP_SYNC_EXTRACTED_INFO,0,0);
	SRI_PRINT(TX_BANDWIDTH, 1, host_params.tx_bandwidth);
	SRI_PRINT(RX_BANDWIDTH, 1, host_params.rx_bandwidth);
	SRI_PRINT(MAX_LATENCY, 1, host_params.max_latency);
	SRI_PRINT(VOICE_SETTING,1, host_params.voice_setting);
	SRI_PRINT(RETX_EFFORT, 1, host_params.retx_effort);
	SRI_PRINT(PKT_TYPES, 1, host_params.pkt_types);
#endif

    if ((*sync_link) == NEW_SYNC_LINK)
    {
		//RT_BT_LOG(GREEN, BT_FW_HCI_1_2_CMDS_1529, 1, sync_ce_index);
		ret_error = hci_handle_new_sync_link(acl_ce_index, &host_params,
                p_link_type, gen_conn_complete);
    }
    else    /* EXISTING_ESCO_LINK */
    {
		//RT_BT_LOG(GREEN, BT_FW_HCI_1_2_CMDS_1536, 0, 0);

#ifdef COMPILE_ESCO
        ret_error = hci_handle_esco_renegotiation(sync_ce_index, &host_params);
#else
        BZ_ASSERT(0, "Only SCO is defined, then why the hell am I taking \
                      this path");
#endif
    }

    return ret_error;
}

#ifdef COMPILE_ESCO
/**
 * Releases the esco resources. It has to be called when the remote initiated
 * connection can not be satisfied. It sends LMP_not_accepted_ext for
 * LMP_esco_link_req and releases the esco resources associated with \a
 * esco_ce_index.
 *
 * \param ce_index ACL connection entity index.
 * \param esco_ce_index ESCO connection entity index.
 * \param reason Reason for rejecting the ESCO connection.
 *
 * \return None.
 */
void hci_release_esco_resources(UINT16 ce_index, UINT16 esco_ce_index,
                                UCHAR reason)
{
    /* Rollback state changes */
	lmp_set_ce_status(ce_index, lmp_connection_entity[ce_index].temp_ce_status);

	lmp_send_lmp_not_accepted_ext(ce_index, LMP_ESCAPE4_OPCODE,
		LMP_ESCO_LINK_REQ_OPCODE, REMOTE_DEV_TID, reason);
	lmp_release_esco_resources(ce_index, esco_ce_index);

	return;
}

/**
 * Initiates the SCO Connection accept procedure.
 *
 * \param ce_index ACL connection entity index.
 * \param host_params Parameters given by the host for this connection.
 *
 * \return HCI_COMMAND_SUCCEEDED, if the operation was successful. A valid
 *         error code, otherwise.
 */
UCHAR hci_handle_new_esco_conn_request(UINT16 ce_index,
                                       HCI_SYNC_PARAMS* host_params)
{
    UINT16 esco_ce_index;
    UCHAR reason = HCI_COMMAND_SUCCEEDED;
    UCHAR esco_link_req_response;
    UCHAR parameter_list[LMP_PDU_MAX_LEN];
    LMP_ESCO_CONNECTION_ENTITY* esco_ce_ptr;
#ifdef _DAPE_TEST_BLOCK_ACL_2_SLOTS_WHEN_SCO
	UINT16 bb_reg;
#endif
    if (lmp_get_esco_ce_index_from_ce_index(ce_index,
                WAITING_FOR_ACCEPT_SYNC_CONN, &esco_ce_index) != API_SUCCESS)
    {
        BZ_ASSERT(0, "Invalid ce_status or lmp_esco_connection_entity"
                     "got screwed up (host could have also misbehaved)");
        return UNSPECIFIED_ERROR;
    }

    reason = hci_validate_esco_params(ce_index, host_params);

    if (reason != HCI_COMMAND_SUCCEEDED)
    {
        hci_release_esco_resources(ce_index, esco_ce_index, reason);
        return reason;
    }

    esco_ce_ptr = &lmp_esco_connection_entity[esco_ce_index];

    esco_ce_ptr->status = ADDING_ESCO_LINK;
    esco_ce_ptr->tx_bandwidth = host_params->tx_bandwidth;
    esco_ce_ptr->rx_bandwidth = host_params->rx_bandwidth;
    esco_ce_ptr->max_latency = host_params->max_latency;
    esco_ce_ptr->voice_setting = host_params->voice_setting;
    esco_ce_ptr->retransmission_effort = host_params->retx_effort;
    esco_ce_ptr->packet_type = host_params->pkt_types;

    lmp_validate_new_esco_link_req(esco_ce_index, ce_index,
                                   &esco_link_req_response, &reason);

    if(reason == HCI_COMMAND_SUCCEEDED)
    {
        /* Check if the new set of parameters will result in esco reserved
           slots occupying the whole bandwidth.
         */
        if(lmp_check_sync_conn_bandwidth(FALSE) == API_FAILURE)
        {
            esco_link_req_response = LMP_NOT_ACCEPTED_EXT_OPCODE;
            reason = UNSUPPORTED_PARAMETER_VALUE_ERROR;
        }
    }

    switch(esco_link_req_response)
    {
        case LMP_ACCEPTED_EXT_OPCODE:
#ifdef _DAPE_TEST_BLOCK_ACL_2_SLOTS_WHEN_SCO
            bb_reg = BB_read_baseband_register(PICONET4_INFO_REGISTER);
            bb_reg |= (UINT16)(0x0040);
            BB_write_baseband_register(PICONET4_INFO_REGISTER, bb_reg);
#endif

#ifdef _DAPE_TEST_SEND_MAX_SLOT_BEFORE_ESCO_CREATED
            lmp_self_device_data.adding_new_esco_conn = TRUE;
            lmp_send_max_slot_pdu_to_all_devices();
#endif
            lmp_send_lmp_accepted_ext(ce_index, LMP_ESCAPE4_OPCODE,
                    LMP_ESCO_LINK_REQ_OPCODE, REMOTE_DEV_TID);
            break;

        case LMP_ESCO_LINK_REQ_OPCODE:
#ifdef _DAPE_TEST_BLOCK_ACL_2_SLOTS_WHEN_SCO
            bb_reg = BB_read_baseband_register(PICONET4_INFO_REGISTER);
            bb_reg |= (UINT16)(0x0040);
            BB_write_baseband_register(PICONET4_INFO_REGISTER, bb_reg);
#endif

#ifdef _DAPE_TEST_SEND_MAX_SLOT_BEFORE_ESCO_CREATED
            lmp_self_device_data.adding_new_esco_conn = TRUE;
            lmp_send_max_slot_pdu_to_all_devices();
#endif
            lmp_get_esco_params_from_ce(parameter_list, esco_ce_index);
            lmp_generate_pdu(ce_index, parameter_list, LMP_ESCO_LINK_REQ_LEN,
                    REMOTE_DEV_TID, LMP_NO_STATE_CHANGE);
            esco_ce_ptr->status = ADDING_ESCO_LINK;
            esco_ce_ptr->negotiation_count ++;
            break;

        case LMP_NOT_ACCEPTED_EXT_OPCODE:
            hci_release_esco_resources(ce_index, esco_ce_index, reason);
            return reason;

        default:
            BZ_ASSERT(0, "lmp_validate_new_esco_link_req is buggy");
            break;
    } /* end switch(esco_link_req_response) */

    return HCI_COMMAND_SUCCEEDED;
}
#endif /* COMPILE_ESCO */

#ifdef ENABLE_SCO
/**
 * Releases the sco resources. It has to be called when the remote initiated
 * connection can not be satisfied. It sends LMP_not_accepted for
 * LMP_sco_link_req and releases the sco resources associated with \a
 * sco_ce_index.
 *
 * \param ce_index ACL connection entity index.
 * \param sco_ce_index SCO connection entity index.
 * \param reason Reason for rejecting the SCO connection.
 *
 * \return None.
 */
void hci_release_sco_resources(UINT16 ce_index, UINT16 sco_ce_index,
        UCHAR reason)
{
    lmp_send_lmp_not_accepted(ce_index, LMP_SCO_LINK_REQ_OPCODE,
                              REMOTE_DEV_TID, reason);
    lmp_sco_connect_restore_ce_status(ce_index);
    lmp_free_sco_conn_entity(sco_ce_index);
}

/**
 * Initiates the SCO Connection accept procedure.
 *
 * \param ce_index ACL connection entity index.
 * \param host_params Parameters given by the host for this connection.
 * \param gen_conn_complete Indicates whether to generate
 *                          Conn_Complete_Evt(TRUE) or
 *                          Sync_Conn_Complete_evt(FALSE).
 *
 * \return HCI_COMMAND_SUCCEEDED, if the operation was successful. A valid
 *         error code, otherwise.
 */
UCHAR hci_handle_new_sco_conn_request(UINT16 ce_index,
        HCI_SYNC_PARAMS* host_params, UCHAR gen_conn_complete)
{
    UCHAR ret_error;
    UCHAR hci_air_mode;
    UINT16 max_latency;
    UINT16 sco_ce_index;
    UINT16 pkt_types;

    if (lmp_get_sco_ce_index_from_ce_index(ce_index,
                SCO_WAITING_FOR_CONN_ACCEPT, &sco_ce_index) != API_SUCCESS)
    {
		//RT_BT_LOG(GRAY, BT_FW_HCI_1_2_CMDS_1709, 0, 0);

        BZ_ASSERT(0, "Invalid ce_status or lmp_sco_connection_data"
                     "got screwed up (host could have also misbehaved)");
        return UNSPECIFIED_ERROR;
    }

    ret_error = hci_validate_sco_params(ce_index, host_params);
    if (ret_error != HCI_COMMAND_SUCCEEDED)
    {
        hci_release_sco_resources(ce_index, sco_ce_index, ret_error);
        return ret_error;
    }

    pkt_types = SCO_PKT_TYPE_1_2_HCI_TO_1_1_HCI(host_params->pkt_types);
    max_latency = TIMER_VAL_TO_SLOT_VAL(host_params->max_latency);
    hci_air_mode = (UCHAR)(host_params->voice_setting & AIR_MODE_MASK);

    ret_error = lmp_handle_new_sco_conn_req(sco_ce_index, max_latency,
            pkt_types, hci_air_mode, gen_conn_complete);

    if(ret_error != BT_FW_SUCCESS)
    {
        hci_release_sco_resources(ce_index, sco_ce_index, ret_error);
        return ret_error;
    }

    return HCI_COMMAND_SUCCEEDED;
}
#endif

/**
 * Handles HCI_ACCEPT_SYNCHRONOUS_CONNECTION_REQUEST_COMMAND from the host. It
 * determines whether this command is issued for SCO or ESCO and initiates the
 * connection accept procedure accordingly.
 *
 * \param hci_cmd_ptr HCI Command Packet pointer.
 * \param p_ce_index Pointer to ACL Connection entity index if new conn, else
 *                   points to eSCO Connection entity. It will be used
 *                   by the caller to generate [Sync]_Conn_Complete_Evt in
 *                   case of any error, or [Sync]_Conn_Change_Evt.
 * \param p_link_type Pointer to link type. It will have either SCO_LINK or
 *                    ESCO_LINK as value. It is also used to generate event
 *                    in case of any failure.
 * \param gen_conn_complete Indicates whether to generate
 *                          Conn_Complete_Evt(TRUE) or
 *                          Sync_Conn_Complete_evt(FALSE).
 *
 * \return HCI_COMMAND_SUCCEEDED, if the operation was successful. A valid
 *         error code, otherwise.
 */
UCHAR hci_handle_accept_sync_conn_req_command(HCI_CMD_PKT *hci_cmd_ptr,
        UINT16* p_ce_index, HCI_LINK_TYPE* p_link_type,
        UCHAR gen_conn_complete)
{
    UCHAR bd_addr[LMP_BD_ADDR_SIZE];
    UINT16 ce_index;
    HCI_SYNC_PARAMS host_params;
    UCHAR ret_error;
    LMP_CONNECTION_ENTITY* ce_ptr;

    /* Extract the bd_addr */
    memcpy(&bd_addr[0], &hci_cmd_ptr->cmd_parameter[0], LMP_BD_ADDR_SIZE);

    if (LMP_GET_CE_INDEX_FROM_BD_ADDR(bd_addr, &ce_index) == API_FAILURE)
    {
        hci_generate_command_status_event(hci_cmd_ptr->cmd_opcode,
                NO_CONNECTION_ERROR);

        /* To avoid generation of Sync_Conn_Complete_Event */
        return HCI_COMMAND_SUCCEEDED;
    }
#ifdef SECURE_CONN_NO_SCO
        BZ_AUTH_LINK_PARAMS* auth;
        auth = lmp_connection_entity[ce_index].auth;
        ce_ptr = &lmp_connection_entity[ce_index];
#ifdef _SECURE_CONN_TEST_LOG
        ///// dape test
        RT_BT_LOG(GREEN, DAPE_TEST_LOG522, 2,auth->super_state,
        auth->sub_state);
#endif
        if ((ce_ptr->ce_status == LMP_ADDING_SCO_LINK) ||
            (ce_ptr->ce_status == LMP_ADDING_SCO_LINK_DURING_SNIFF))
        {
            //if ((auth->secure_conn_enabled) && (auth->super_state > 11))
            if ((auth->secure_conn_enabled) && (bz_auth_is_link_encrypted(ce_index)))
            {
                hci_generate_command_status_event(hci_cmd_ptr->cmd_opcode,
                HOST_REJECTED_SECURITY_REASONS_ERROR);

                return HCI_COMMAND_SUCCEEDED;
            }
        }
#endif

    /* We have got the important parameter of the command, so generate
     * Command_Status event. In case of any other failure, we will generate
     * Sync_Conn_Complete_Event hereafter (The caller will generate it, if we
     * return any error code other than HCI_COMMAND_SUCCEEDED).
     */
    hci_generate_command_status_event(hci_cmd_ptr->cmd_opcode,
            HCI_COMMAND_SUCCEEDED);
#ifndef SECURE_CONN_NO_SCO
    *p_ce_index = ce_index;
#endif
    ce_ptr = &lmp_connection_entity[ce_index];

#ifdef _BRUCE_CVSD_IS_RESTRICTED_TRX_PKT_SAME
    hci_cmd_ptr->cmd_parameter[14]=0x07;
#endif

#ifdef _BRUCE_TEST_SCO
    //hci_cmd_ptr->cmd_parameter[18] = 0;//no retransmission;
    hci_cmd_ptr->cmd_parameter[19] = 0xC7;//no retransmission;
    hci_cmd_ptr->cmd_parameter[20] = 0x03;//no retransmission;
#endif

    OS_DELETE_TIMER(&ce_ptr->conn_accept_timer_handle);

    BT_FW_EXTRACT_32_BITS(host_params.tx_bandwidth,
                                 &hci_cmd_ptr->cmd_parameter[6]);
    BT_FW_EXTRACT_32_BITS(host_params.rx_bandwidth,
                                 &hci_cmd_ptr->cmd_parameter[10]);
    BT_FW_EXTRACT_16_BITS(host_params.max_latency,
                          &hci_cmd_ptr->cmd_parameter[14]);
    BT_FW_EXTRACT_16_BITS(host_params.voice_setting,
                          &hci_cmd_ptr->cmd_parameter[16]);
    BT_FW_EXTRACT_16_BITS(host_params.pkt_types,
                                 &hci_cmd_ptr->cmd_parameter[19]);
    host_params.retx_effort = hci_cmd_ptr->cmd_parameter[18];

#ifdef _BRUCE_MSBC_IS_RESTRICTED_PKT_TYPE
    if (lmp_esco_over_codec == TRUE)
    {
        g_voice_setting = host_params.voice_setting;
    }
#endif

#ifdef SCO_REFACTORING_DEBUG
	SRI_PRINT(ACCEPT_SYNC_EXTRACTED_INFO,0,0);
	SRI_PRINT(TX_BANDWIDTH,1, host_params.tx_bandwidth);
	SRI_PRINT(RX_BANDWIDTH,1, host_params.rx_bandwidth);
	SRI_PRINT(MAX_LATENCY,1, host_params.max_latency);
	SRI_PRINT(VOICE_SETTING,1, host_params.voice_setting);
	SRI_PRINT(RETX_EFFORT,1, host_params.retx_effort);
	SRI_PRINT(PKT_TYPES,1, host_params.pkt_types);
#endif

    switch (ce_ptr->ce_status)
    {
#ifdef COMPILE_ESCO
        case LMP_ADDING_ESCO_LINK:
            *p_link_type = ESCO_LINK;
            ret_error = hci_handle_new_esco_conn_request(ce_index,
                                                         &host_params);
            break;
#endif
#ifdef ENABLE_SCO
        case LMP_ADDING_SCO_LINK:
#ifdef COMPILE_SNIFF_MODE
        case LMP_ADDING_SCO_LINK_DURING_SNIFF:
#endif
            *p_link_type = SCO_LINK;
#ifdef _SUPPORT_SECURE_CONNECTION_
            BZ_AUTH_LINK_PARAMS* auth;
            auth = lmp_connection_entity[ce_index].auth;

            if ((auth->secure_conn_enabled) && (bz_auth_is_link_encrypted(ce_index)))
            {
                ret_error = HOST_REJECTED_SECURITY_REASONS_ERROR;
                return ret_error;
            }
#endif
            ret_error = hci_handle_new_sco_conn_request(ce_index,
                    &host_params, gen_conn_complete);
            break;
#endif /* ENABLE_SCO */

        default:
            BZ_ASSERT(0, "Why the hell am I getting this Command now");
            *p_link_type = INVALID_LINK_TYPE; /* God knows what happened!! */
            return COMMAND_DISALLOWED_ERROR;
    }

    return ret_error;
}

/**
 * Handles HCI_REJECT_SYNCHRONOUS_CONNECTION_REQUEST_COMMAND from the host. It
 * determines whether this command is issued for SCO or ESCO and initiates the
 * connection reject procedure accordingly.
 *
 * \param hci_cmd_ptr HCI Command Packet pointer.
 * \param p_ce_index Pointer to ACL Connection entity index. It will be used
 *                   by the caller to generate [Sync]_Conn_Complete_Evt in
 *                   case of any error.
 * \param p_link_type Pointer to link type. It will have either SCO_LINK or
 *                    ESCO_LINK as value. It is also used to generate the
 *                    event.
 *
 * \return HCI_COMMAND_SUCCEEDED, if the operation was successful. A valid
 *         error code, otherwise.
 */
UCHAR hci_handle_reject_sync_conn_req_command(HCI_CMD_PKT *hci_cmd_ptr,
        UINT16* p_ce_index, HCI_LINK_TYPE* p_link_type)
{
    UINT16 ce_index;
    UINT16 sync_ce_index;
    UCHAR bd_addr[LMP_BD_ADDR_SIZE];
    UCHAR reason;
    LMP_CONNECTION_ENTITY* ce_ptr;

    memcpy(&bd_addr[0], &hci_cmd_ptr->cmd_parameter[0], LMP_BD_ADDR_SIZE);
    if (LMP_GET_CE_INDEX_FROM_BD_ADDR(bd_addr, &ce_index) == API_FAILURE)
    {
        X_BZ_ASSERT(0, "Host might be misbehaving");
        hci_generate_command_status_event(hci_cmd_ptr->cmd_opcode,
                NO_CONNECTION_ERROR);

        return HCI_COMMAND_SUCCEEDED;
    }
    reason = hci_cmd_ptr->cmd_parameter[6];
    if ((reason < 0x0D) || (reason > 0x0F))
    {
        hci_generate_command_status_event(hci_cmd_ptr->cmd_opcode,
                INVALID_HCI_COMMAND_PARAMETERS_ERROR);
        return HCI_COMMAND_SUCCEEDED;
    }


    hci_generate_command_status_event(hci_cmd_ptr->cmd_opcode,
            HCI_COMMAND_SUCCEEDED);

    *p_ce_index = ce_index;
    ce_ptr = &lmp_connection_entity[ce_index];

    OS_DELETE_TIMER(&ce_ptr->conn_accept_timer_handle);

    switch(ce_ptr->ce_status)
    {
#ifdef ENABLE_SCO
        case LMP_ADDING_SCO_LINK:
#ifdef COMPILE_SNIFF_MODE
        case LMP_ADDING_SCO_LINK_DURING_SNIFF:
#endif
            *p_link_type = SCO_LINK;
			//RT_BT_LOG(GRAY, BT_FW_HCI_1_2_CMDS_1910, 0, 0);

            if (lmp_get_sco_ce_index_from_ce_index(ce_index,
                        SCO_WAITING_FOR_CONN_ACCEPT, &sync_ce_index)
                    == API_SUCCESS)
            {
                hci_release_sco_resources(ce_index, sync_ce_index, reason);
            }
            else
            {
                X_BZ_ASSERT(0, "Invalid ce_status or lmp_sco_connection_data"
                        "got screwed up (host could have also misbehaved)");
            }
            break;
#endif /* ENABLE_SCO */

#ifdef COMPILE_ESCO
        case LMP_ADDING_ESCO_LINK:
            *p_link_type = ESCO_LINK;
            if (lmp_get_esco_ce_index_from_ce_index(ce_index,
                        WAITING_FOR_ACCEPT_SYNC_CONN, &sync_ce_index)
                    == API_SUCCESS)
            {
                hci_release_esco_resources(ce_index, sync_ce_index, reason);
            }
            else
            {
                X_BZ_ASSERT(0, "Invalid ce_status or lmp_esco_connection_entity"
                        "got screwed up (host could have also misbehaved)");
            }
            break;
#endif /* COMPILE_ESCO */

        default:
            *p_link_type = SCO_LINK;    /* I like SCO */
            BZ_ASSERT(0, "God!! Dont do this to me!");
            break;
    }

    return reason;
}
#endif /* defined(ENABLE_SCO) || defined(COMPILE_ESCO) */

#ifdef COMPILE_FEATURE_REQ_EXT
/**
 * Handles the Read_remote_extended_features command.
 *
 * \param hci_cmd_ptr Pointer to HCI_CMD_PKT.
 * \param sent_cmd_status set to true if cmd_status event generated.
 *
 * \return HCI_COMMAND_SUCCEEDED or HCI error code.
 *
 */
UCHAR hci_handle_read_remote_ext_features_command(HCI_CMD_PKT *hci_cmd_ptr,
        UCHAR *sent_cmd_status)
{
    UINT16 connection_handle;
    UINT16 ce_index;
    LMP_CONNECTION_ENTITY* ce_ptr;

    BT_FW_EXTRACT_16_BITS(connection_handle, &hci_cmd_ptr->cmd_parameter[0]);
    if (LMP_GET_CE_INDEX_FROM_CONN_HANDLE(connection_handle, &ce_index)
            != API_SUCCESS)
    {
        return NO_CONNECTION_ERROR;
    }

    ce_ptr = &lmp_connection_entity[ce_index];

    if ((ce_ptr->feat_page0[7] & EXTENDED_FEATURE) == FALSE)
    {
        return UNSUPPORTED_REMOTE_FEATURE_ERROR;
    }

    UINT8 page_number = hci_cmd_ptr->cmd_parameter[2];

    if (page_number == 0)
    {
        /* requests the normal LMP features as returned by
           HCI_Read_Remote_Supported_Features */
        lmp_send_features_req_or_res_pdu(ce_index, LMP_FEATURES_REQ_OPCODE,
                SELF_DEV_TID);
        ce_ptr->lmp_expected_pdu_opcode |= lmp_get_opcode_mask(
                    LMP_FEATURES_RES_OPCODE, 0x0);

        ce_ptr->hci_cmd_bits |= REMOTE_SUP_FEA_BIT_MASK;
        ce_ptr->hci_cmd_bits  |= REMOTE_SUP_FEA_TRUE_HOST_BIT_MASK;
        return HCI_COMMAND_SUCCEEDED;
    }

#ifndef _SUPPORT_EXT_FEATURES_PAGE_2_
    ce_ptr->requested_ext_page = page_number;
#else
    ce_ptr->host_requested_ext_page = page_number;
#endif

    lmp_send_features_req_or_res_ext_pdu(ce_index, page_number,
            LMP_FEATURES_REQ_EXT_OPCODE, SELF_DEV_TID);
    ce_ptr->hci_cmd_bits |= REMOTE_EX_FEA_BIT_MASK;
    ce_ptr->hci_cmd_bits  |= REMOTE_EX_FEA_TRUE_HOST_BIT_MASK;

    return HCI_COMMAND_SUCCEEDED;
}
#endif /* COMPILE_FEATURE_REQ_EXT */

