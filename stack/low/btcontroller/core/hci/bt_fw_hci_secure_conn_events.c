/***************************************************************************
 Copyright (C) Realtek Semiconductor Corp.
 This module is a confidential and proprietary property of Realtek and
 a possession or use of this module requires written permission of Realtek.
 ***************************************************************************/

/**
 * \file
 *  HCI events handlers related
 *  to the Bluetooth CSA4 Features.
 *  
 */
#ifdef _SUPPORT_SECURE_CONNECTION_
/********************************* Logger *************************/ 
enum { __FILE_NUM__= 78 };
/********************************* Logger *************************/

#include "lmp.h"
#include "lc_internal.h"
#include "bt_fw_hci_secure_conn_cmds_evts.h"
#include "bt_fw_hci_spec_defines.h"
#include "bt_fw_hci_internal.h"
#include "mem.h"
#include "bz_auth.h"
#include "bz_auth_internal.h"
#include "bt_secure_conn.h"
#include "bz_auth_internal_2_1.h"
#include "bz_auth_extern_accessors.h"

/**
 * Prepares the command complete event for BR/EDR Secure Connection HCI commands.
 * This function is called from the hci_generate_2_1_command_complete_
 * event function in bt_fw_hci_2_1_events.c , this function populates the
 * parameters after the status parameter and returns the length of
 * event packet.
 *
 * \param cmd_opcode Command opcode for which event has to be generated.
 * \param pkt_param  Pointer to the event packet parameter list.
 *
 * \return The total length of all the parameters in the event packet.
 *
 */
UCHAR hci_generate_secure_conn_command_complete_event( UINT16 cmd_opcode,
                                               UCHAR *pkt_param, 
                                               UINT16 ce_index )
{
    /** Param length is 4 because event_opcode(1),cmd_opcode(2) and status(1)
     * are populated by the calling function
     */
    UCHAR param_length = 4;
    LMP_CONNECTION_ENTITY *ce_ptr;
    if (ce_index != INVALID_CE_INDEX)
    {
        ce_ptr = &lmp_connection_entity[ce_index];
    }
    else
    {
        ce_ptr = NULL;
    }
    switch(cmd_opcode)
    {
    case  HCI_REMOTE_OOB_EXT_DATA_REQUEST_REPLY_OPCODE:
        if (ce_ptr)
        {
            memcpy(&pkt_param[4], bz_auth_get_remote_bd_addr(ce_ptr),
                    LMP_BD_ADDR_SIZE);
        }
        else
        {
            /* Fill the event param with the bd addr from hci command. */
#ifdef _DAPE_RECORD_COMMAND_POINTER_FOR_EVENT
            memcpy(&pkt_param[4], &g_record_hci_cmd_pkt_ptr->cmd_parameter[0],
                    LMP_BD_ADDR_SIZE);
#else
            memset(&pkt_param[4], 0x00,
                    LMP_BD_ADDR_SIZE);
#endif
        }
        param_length = LMP_BD_ADDR_SIZE+4 /* the common ones */;
        break;
    case  HCI_READ_SECURE_CONN_HOST_SUPPORT_OPCODE:
        pkt_param[4] = bz_auth_dev_params.secure_connection_host_enable;
        param_length = 5;
        break;

    case HCI_WRITE_SECURE_CONN_HOST_SUPPORT_OPCODE:
        break;
        
    case HCI_READ_AUTH_PAYLOAD_TIMEOUT_OPCODE:
        if (ce_ptr)
        {
            pkt_param[4] = LSB(ce_ptr->connection_type.connection_handle);
            pkt_param[5] = MSB(ce_ptr->connection_type.connection_handle);
            pkt_param[6] = LSB(ce_ptr->max_auth_interval);
            pkt_param[7] = MSB(ce_ptr->max_auth_interval);
        }
        else
        {
#ifdef _DAPE_RECORD_COMMAND_POINTER_FOR_EVENT
            pkt_param[4] = g_record_hci_cmd_pkt_ptr->cmd_parameter[0];
            pkt_param[5] = g_record_hci_cmd_pkt_ptr->cmd_parameter[1];
            pkt_param[6] = g_record_hci_cmd_pkt_ptr->cmd_parameter[2];
            pkt_param[7] = g_record_hci_cmd_pkt_ptr->cmd_parameter[3];
#else
            pkt_param[4] = LSB(ce_index);
            pkt_param[5] = MSB(ce_index);
            pkt_param[6] = 0x00;
            pkt_param[7] = 0x00;
#endif
        }
       
        param_length = 8;
        break;

    case HCI_WRITE_AUTH_PAYLOAD_TIMEOUT_OPCODE:
    case HCI_WRITE_SECURE_CONN_TEST_MODE_OPCODE:
        if (ce_ptr)
        {
            pkt_param[4] = LSB(ce_ptr->connection_type.connection_handle);
            pkt_param[5] = MSB(ce_ptr->connection_type.connection_handle);
        }
        else
        {
#ifdef _DAPE_RECORD_COMMAND_POINTER_FOR_EVENT
            pkt_param[4] = g_record_hci_cmd_pkt_ptr->cmd_parameter[0];
            pkt_param[5] = g_record_hci_cmd_pkt_ptr->cmd_parameter[1];
#else
            pkt_param[4] = LSB(ce_index);
            pkt_param[5] = MSB(ce_index);
#endif
        }
        param_length = 6;
        break;

    case HCI_READ_LOCAL_OOB_EXT_DATA_OPCODE:        
        bz_auth_generate_oob_data();
        memcpy(&pkt_param[4],
                &bz_auth_dev_params.last_generated_oob_data.C[0],
                BZ_AUTH_CONFIRM_VALUE_SIZE);
        memcpy(&pkt_param[4+BZ_AUTH_CONFIRM_VALUE_SIZE],
                &bz_auth_dev_params.last_generated_oob_data.R[0],
                BZ_AUTH_OOB_SECRET_NUMBER_SIZE);
        memcpy(&pkt_param[4+BZ_AUTH_CONFIRM_VALUE_SIZE+
                                 BZ_AUTH_OOB_SECRET_NUMBER_SIZE],
                &bz_auth_dev_params.last_generated_oob_data.C_P256[0],
                BZ_AUTH_CONFIRM_VALUE_SIZE);
        memcpy(&pkt_param[4+BZ_AUTH_CONFIRM_VALUE_SIZE+
                                 BZ_AUTH_OOB_SECRET_NUMBER_SIZE+
                                 BZ_AUTH_CONFIRM_VALUE_SIZE],
                &bz_auth_dev_params.last_generated_oob_data.R_P256[0],
                BZ_AUTH_OOB_SECRET_NUMBER_SIZE);
                
        /* Convert the Hash and Randomizer values to LSB format... All the
         * HCI layer communication is through LSB format only. But our
         * internal cryptographic functions uses MSB format.
         */
        bz_auth_convert_to_lsb(&pkt_param[4],
                BZ_AUTH_CONFIRM_VALUE_SIZE);
        bz_auth_convert_to_lsb(&pkt_param[4+16],
                BZ_AUTH_OOB_SECRET_NUMBER_SIZE);
        bz_auth_convert_to_lsb(&pkt_param[4+16+16],
                BZ_AUTH_CONFIRM_VALUE_SIZE);
        bz_auth_convert_to_lsb(&pkt_param[4+16+16+16],
                BZ_AUTH_OOB_SECRET_NUMBER_SIZE);
        param_length = 68;
        break;

    default :
        return 0xFF;
    }/* end switch(cmd_opcode) */

    return (UCHAR) (param_length - 4);
}


/**************************************************************************
 * Function     : hci_generate_authentication_payload_timeout_event
 *
 * Description  : This function is used to generate Authentication Payload 
 *                Timeout Expired Event. The Synchronization Train Complete event 
 *                indicates that the Start Synchronization Train command has 
 *                completed.
 *
 * Parameters   : connection handle
 *
 * Returns      : None
 *
 *************************************************************************/
void hci_generate_authentication_payload_timeout_expired_event(UINT16 conn_handle)
{
    UCHAR event_parameter[2];
    event_parameter[0] = LSB(conn_handle);
    event_parameter[1] = MSB(conn_handle);


    hci_generate_event(HCI_AUTH_PAYLOAD_TIMEOUT_EXPIRED_EVENT,
            event_parameter, 2);
}

#endif

