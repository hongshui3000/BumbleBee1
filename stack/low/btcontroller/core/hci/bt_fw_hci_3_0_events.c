/***************************************************************************
 Copyright (C) MindTree Consulting Ltd.
 This module is a confidential and proprietary property of MindTree and
 a possession or use of this module requires written permission of MindTree.
 ***************************************************************************/

/**
 * \file
 *  HCI events handlers related
 *  to the Bluetooth Version 3.0 Features.
 *  
 */
#ifdef VER_3_0
/********************************* Logger *************************/ 
enum { __FILE_NUM__= 74 };
/********************************* Logger *************************/

#include "lmp.h"
#include "lmp_3_0.h"
#include "bt_fw_hci_3_0_cmds.h"
#include "bt_fw_hci_spec_defines.h"

#include "bz_auth.h"
#include "mem.h"

extern UCHAR hci_epc_level_cmd_param;

/**
 * Prepares the command complete event for Version 3_0 HCI commands.
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
UCHAR hci_generate_3_0_command_complete_event( UINT16 cmd_opcode,
                                               UCHAR *pkt_param, 
                                               UINT16 ce_index )
{
    /** Param length is 4 because event_opcode(1),cmd_opcode(2) and status(1)
     * are populated by the calling function
     */
    UCHAR param_length = 4;
    CHAR power_level;
    LMP_CONNECTION_ENTITY *ce_ptr;
    UINT16 conn_handle;
    
    ce_ptr = &lmp_connection_entity[ce_index];
    conn_handle = ce_ptr->connection_type.connection_handle;

    if (pkt_param[3] == HCI_COMMAND_SUCCEEDED)
    {
        switch(cmd_opcode)
        {
        case  HCI_READ_ENHANCED_TRANSMIT_POWER_LEVEL_OPCODE:
            pkt_param[4] = conn_handle & 0xFF;
            pkt_param[5] = conn_handle >> 8;
            
            if(hci_epc_level_cmd_param == 0)
            {
                power_level = lmp_epc_get_cur_pwr_level(ce_index);
            }
            else
            {
                power_level = lmp_epc_get_max_pwr_level();
            }

            pkt_param[6] = power_level;
            pkt_param[7] = power_level;
            pkt_param[8] = power_level;
            param_length = 9;
            break;

        case HCI_READ_ENCRYPTION_KEY_SIZE_OPCODE:
            pkt_param[4] = conn_handle & 0xFF;
            pkt_param[5] = conn_handle >> 8;
            pkt_param[6] = bz_auth_get_encryption_keysize(ce_index);
            param_length = 7;
            break;

        default :
            break;
        }/* end switch(cmd_opcode) */
    }
    else
    {
        switch(cmd_opcode)
        {
        case  HCI_READ_ENHANCED_TRANSMIT_POWER_LEVEL_OPCODE:
            /* Set all values to zero. */
            memset(&pkt_param[param_length],0,5);
            param_length += 5;
            break;

        case  HCI_READ_ENCRYPTION_KEY_SIZE_OPCODE:
            /* Set all values to zero. */
            memset(&pkt_param[param_length],0,3);
            param_length += 3;
            break;

        default :
            break;
        }
    }
    return (UCHAR) (param_length - 4);
}

#endif
