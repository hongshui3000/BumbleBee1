/***************************************************************************
 Copyright (C) MindTree Consulting Ltd.
 This module is a confidential and proprietary property of MindTree and
 a possession or use of this module requires written permission of MindTree.
 ***************************************************************************/

/**
 * \file
 *  BT3_0 HCI Commands implementation.
 *  
 */

#ifdef VER_3_0
/********************************* Logger *************************/ 
enum { __FILE_NUM__= 73 };
/********************************* Logger *************************/

/* -----------------Includes----------------------------- */
#include "bt_fw_hci_internal.h"
#include "bt_fw_hci_3_0_cmds.h"
#include "btc_defines.h"
#include "bt_fw_hci_external_defines.h"
#include "bz_auth.h"
#include "mem.h"

UCHAR hci_epc_level_cmd_param = 0;

/* -----------------Global variables--------------------- */

/* -----------------Static functions--------------------- */

/**
 * Handles the Read Enahnced Power Transmit Power Level.
 * 
 * \param hci_cmd_ptr HCI Command pointer.
 * 
 * \return HCI Status.
 */
UCHAR hci_handle_enhanced_trans_pwr_level(HCI_CMD_PKT *hci_cmd_ptr)
{
    UINT16 conn_handle;
    UINT16 ce_index;
    UCHAR ret_error = HCI_COMMAND_SUCCEEDED;

    if (hci_cmd_ptr->param_total_length != 3)
    {
        return INVALID_HCI_COMMAND_PARAMETERS_ERROR;
    }

    BT_FW_EXTRACT_16_BITS(conn_handle, &hci_cmd_ptr->cmd_parameter[0]);
    if(LMP_GET_CE_INDEX_FROM_CONN_HANDLE(conn_handle, &ce_index)!= API_SUCCESS)
    {
        return NO_CONNECTION_ERROR;
    }

    hci_epc_level_cmd_param = hci_cmd_ptr->cmd_parameter[2];

    return ret_error;
}

#if defined(VER_CSA4) || defined(_SUPPORT_SECURE_CONNECTION_)
/**
 * Handles the Set Event Mask Page 2 Command.
 * 
 * \param hci_cmd_ptr HCI Command pointer.
 * 
 * \return HCI Status.
 */
UCHAR hci_handle_set_event_mask_page_2(HCI_CMD_PKT *hci_cmd_ptr)
{
    UCHAR ret_error = HCI_COMMAND_SUCCEEDED;

    if (hci_cmd_ptr->param_total_length != 8)
    {
        return INVALID_HCI_COMMAND_PARAMETERS_ERROR;
    }

    /* update valid event mask page 2 value */
    memcpy(lmp_self_device_data.event_mask_p2, hci_cmd_ptr->cmd_parameter, 8);

    return ret_error;
}
#endif

UCHAR hci_handle_read_encryption_key_size(HCI_CMD_PKT *hci_cmd_ptr)
{
    UINT16 conn_handle;
    UINT16 ce_index;
    UCHAR ret_error = HCI_COMMAND_SUCCEEDED;

    if (hci_cmd_ptr->param_total_length != 2)
    {
        return INVALID_HCI_COMMAND_PARAMETERS_ERROR;
    }

    BT_FW_EXTRACT_16_BITS(conn_handle, &hci_cmd_ptr->cmd_parameter[0]);
    if(LMP_GET_CE_INDEX_FROM_CONN_HANDLE(conn_handle, &ce_index)!= API_SUCCESS)
    {
        return NO_CONNECTION_ERROR;
    }

    if(bz_auth_get_encryption_mode(ce_index) ==  BZ_AUTH_ENCRYPTION_MODE_OFF)
    {
        return INSUFFICIENT_SECURITY;
    }

    return ret_error;
}

/* -----------------External functions------------------- */
/**
 * Handles all the host controller and baseband for the Bluetooth Version 3_0.
 * This function is called from the hci_task.c if the opcode
 * does not match any of the 1.1/1.2/2.1 Version opcodes.
 *
 * \param hci_cmd_ptr Pointer to HCI_CMD_PKT.
 * \param sent_event  Sets the variable if event is sent.
 *
 * \return HCI_COMMAND_SUCCEEDED or HCI error code.
 *
 */
UCHAR hci_handle_3_0_hc_bb_command(HCI_CMD_PKT *hci_cmd_ptr)
{
    UCHAR ret_error = HCI_COMMAND_SUCCEEDED;

    switch(hci_cmd_ptr->cmd_opcode)
    {
        case HCI_READ_ENHANCED_TRANSMIT_POWER_LEVEL_OPCODE:
            {
                ret_error = hci_handle_enhanced_trans_pwr_level(hci_cmd_ptr);
            }
            break;

#if defined(VER_CSA4) || defined(_SUPPORT_SECURE_CONNECTION_) 
       case HCI_SET_EVENT_MASK_PAGE_2_OPCODE:
            ret_error = hci_handle_set_event_mask_page_2(hci_cmd_ptr);
            break;
#endif

        default:
            ret_error = UNKNOWN_HCI_COMMAND_ERROR;
            break;
    }

    return ret_error;
}


UCHAR hci_handle_3_0_status_command(HCI_CMD_PKT *hci_cmd_ptr)
{
    UCHAR ret_error = HCI_COMMAND_SUCCEEDED;

    switch(hci_cmd_ptr->cmd_opcode)
    {
        case HCI_READ_ENCRYPTION_KEY_SIZE_OPCODE:
            {
                ret_error = hci_handle_read_encryption_key_size(hci_cmd_ptr);
            }
            break;

        default:
            ret_error = UNKNOWN_HCI_COMMAND_ERROR;
            break;
    }

    return ret_error;
}
#endif

