/***************************************************************************
 Copyright (C) Mindtree Ltd.
 This module is a confidential and proprietary property of MindTree and
 a possession or use of this module requires written permission of MindTree.
 ***************************************************************************/

/**
 * \file
 *  HCI events handlers related
 *  to the Bluetooth Version 2.1 Features.
 *  
 *  \author Muthu Subramanian K
 *  
 */
/********************************* Logger *************************/ 
enum { __FILE_NUM__= 7 };
/********************************* Logger *************************/

/* -----------------Includes----------------------------- */
#include "bt_fw_hci_internal.h"
#include "bt_fw_hci_2_1.h"
#include "mem.h"
#include "bt_fw_hci.h"
#include "le_hci_4_0.h"

/* Move this to a suitable place */
/**
 * Used for read_inquiry_response_transmit_power_level command
 * Should be in the range of -70<= N <= 20
 */
#define HCI_INQUIRY_RES_TRANS_POWER_LEVEL           (CHAR)0

/* -----------------Global variables--------------------- */

/* -----------------Static functions--------------------- */

/* -----------------External functions------------------- */

#ifdef COMPILE_SNIFF_MODE

/**
 * Generates sniff subrating event.
 * 
 * \param status Status of the event.
 * \param conn_handle Connection handle.
 * \param ce_index CE Index (to fill latency/timeout)
 * 
 * \return None.
 */
void hci_generate_sniff_subrating_event(UCHAR status, UINT16 ce_index)
{
    UCHAR event_parameter[HCI_SNIFF_SUBRATING_EVENT_LEN];
    LMP_CONNECTION_ENTITY *ce_ptr;
    UINT16 max_transmit_latency = 0;
    UINT16 max_receive_latency = 0;
    UINT16 min_remote_timeout;
    UINT16 min_local_timeout;
    UINT16 conn_handle;
    UINT16 ssr;
    UINT16 rem_ssr;

    ce_ptr = &lmp_connection_entity[ce_index];

    conn_handle = ce_ptr->connection_type.connection_handle;
    ssr = (UCHAR)((ce_ptr->ssr_data.max_latency)/ce_ptr->sniff_interval);
    rem_ssr = ce_ptr->ssr_data.rem_max_ssr;

    if (ssr == 0)
    {
        ssr = 1;
    }
    if (rem_ssr == 0)
    {
        rem_ssr = 1;
    }

    if ((rem_ssr <= ssr) && (rem_ssr != 0))
    {
        ssr = (UINT16) ((ssr/rem_ssr)*rem_ssr);
        max_transmit_latency  = (UINT16) (ssr * ce_ptr->sniff_interval);
        max_receive_latency = (UINT16) (rem_ssr * ce_ptr->sniff_interval);
    }
    else if ((rem_ssr > ssr) && (ssr != 0))
    {
        ssr = (UINT16) ((rem_ssr/ssr)*ssr);
        max_transmit_latency = ce_ptr->ssr_data.max_latency;
        max_receive_latency  = (UINT16) (ssr * ce_ptr->sniff_interval);
    }

    min_remote_timeout   = ce_ptr->ssr_data.min_remote_timeout;
    min_local_timeout    = ce_ptr->ssr_data.rem_min_timeout;

    if (min_local_timeout < ce_ptr->ssr_data.min_local_timeout)
    {
        min_local_timeout = ce_ptr->ssr_data.min_local_timeout;
    }

    /* Check if latencies are less than sniff_interval */
    if (max_transmit_latency < ce_ptr->sniff_interval)
    {
        max_transmit_latency = ce_ptr->sniff_interval;
    }
    if(max_receive_latency < ce_ptr->sniff_interval)
    {
        max_receive_latency = ce_ptr->sniff_interval;
    }

    /* Fill in the parameters */
    event_parameter[ 0] = (UCHAR) status;

    event_parameter[ 1] = (UCHAR)(conn_handle);
    event_parameter[ 2] = (UCHAR)(conn_handle>>8);

    event_parameter[ 3] = (UCHAR)(max_transmit_latency);
    event_parameter[ 4] = (UCHAR)(max_transmit_latency>>8);

    event_parameter[ 5] = (UCHAR)(max_receive_latency);
    event_parameter[ 6] = (UCHAR)(max_receive_latency>>8);

    event_parameter[ 7] = (UCHAR)(min_remote_timeout);
    event_parameter[ 8] = (UCHAR)(min_remote_timeout>>8);

    event_parameter[ 9] = (UCHAR)(min_local_timeout);
    event_parameter[10] = (UCHAR)(min_local_timeout>>8);

    hci_generate_event(HCI_SNIFF_SUBRATING_EVENT, event_parameter,
            HCI_SNIFF_SUBRATING_EVENT_LEN);

	return;
}
#endif /* COMPILE_SNIFF_MODE */


/**
 * Generates LSTO Changed event.
 *
 * \param conn_handle        Connection Handle.
 * \param to_value           New Timeout value.
 *
 * \return None.
 */
void hci_generate_lsto_change_event(UINT16 conn_handle, UINT16 to_value)
{
    UCHAR event_parameter[HCI_LSTO_CHANGE_EVENT_LEN];
    event_parameter[0] = (UCHAR)(conn_handle);
    event_parameter[1] = (UCHAR)(conn_handle>>8);
    event_parameter[2] = (UCHAR)(to_value);
    event_parameter[3] = (UCHAR)(to_value>>8);
    
    hci_generate_event(HCI_LSTO_CHANGE_EVENT, event_parameter,
            HCI_LSTO_CHANGE_EVENT_LEN);
}

/**
 * Generates Enhanced Flush complete event.
 *
 * \param conn_handle        Connection Handle.
 *
 * \return None.
 */
void hci_generate_enhanced_flush_event(UINT16 conn_handle)
{
    UCHAR event_parameter[HCI_ENHANCED_FLUSH_COMPLETE_EVENT_LEN];
    event_parameter[0] = (UCHAR)(conn_handle);
    event_parameter[1] = (UCHAR)(conn_handle>>8);
    
    hci_generate_event(HCI_ENHANCED_FLUSH_COMPLETE_EVENT, event_parameter,
            HCI_ENHANCED_FLUSH_COMPLETE_EVENT_LEN);
}

/**
 * Generate Enhanced Inquiry Response Event.
 * 
 * \param bd_addr BD Address of the remote device.
 * \param page_scan_rep_mode Page Scan Repetition Mode.
 * \param cod Class of Device.
 * \param clk_off Clock Offset.
 * \param rssi RSSI.
 * \param eir_data EIR Response data (max. MAX_EIR_DATA_LEN).
 * 
 * \return None.
 */
void hci_generate_eir_event(UCHAR bd_addr[6],UCHAR page_scan_rep_mode,
                            UINT32 cod, UINT16 clk_off,
                            UCHAR rssi, UCHAR *eir_data, UCHAR eir_len)
{
    UCHAR event_parameter[HCI_EXTENDED_INQUIRY_RESULT_EVENT_LEN];
//	HCI_LOG_INFO(LOG_LEVEL_HIGH, EIR_EVENT, 1,eir_len);
    event_parameter[0] = 0x01;
    memcpy(&event_parameter[1], bd_addr, 6);
    event_parameter[7] = page_scan_rep_mode;
    event_parameter[8] = 0x0; /* Reserved */
    event_parameter[9] = (UCHAR)cod;
    event_parameter[10] = (UCHAR)(cod >> 8);
    event_parameter[11] = (UCHAR)(cod >> 16);
    event_parameter[12] = (UCHAR)clk_off;
    event_parameter[13] = (UCHAR)(clk_off >> 8);
    event_parameter[14] = rssi;
    /*(gordon) change avoid memcpy len<0*/
    if(eir_len < MAX_EIR_DATA_LEN)
    {
        memcpy(&event_parameter[15], eir_data, eir_len);
        memset(&event_parameter[15+eir_len], 0, MAX_EIR_DATA_LEN - eir_len);
    }else
    {
        memcpy(&event_parameter[15], eir_data, MAX_EIR_DATA_LEN);
    }

    hci_generate_event(HCI_EXTENDED_INQUIRY_RESULT_EVENT, event_parameter,
            HCI_EXTENDED_INQUIRY_RESULT_EVENT_LEN);
}

/**
 * Prepares the command complete event for Version 2_1 HCI commands.
 * This function is called from the hci_generate_1_2_command_complete_
 * event function in hci_1_2_cmds.c , this function populates the
 * parameters after the status parameter and returns the length of
 * event packet.
 *
 * \param cmd_opcode Command opcode for which event has to be generated.
 * \param pkt_param  Pointer to the event packet parameter list.
 *
 * \return The total length of all the parameters in the event packet.
 *
 */
UCHAR hci_generate_2_1_command_complete_event( UINT16 cmd_opcode,
                                               UCHAR *pkt_param, 
                                               UINT16 ext_param )
{
    /** Param length is 4 because event_opcode(1),cmd_opcode(2) and status(1)
     * are populated by the calling function
     */
    UCHAR param_length = 4;

    switch(cmd_opcode)
    {
    case HCI_READ_EXTENDED_INQUIRY_RESPONSE_OPCODE:
        pkt_param[param_length] = lmp_self_device_data.eir_data.fec_req;
        param_length++;
        /* Copy the EIR Data */
        {
            UCHAR eir_len = lmp_self_device_data.eir_data.length;
            memcpy(&pkt_param[param_length],
                   lmp_self_device_data.eir_data.data,eir_len);

            if(eir_len < MAX_EIR_DATA_LEN)
            {
                memset(&pkt_param[param_length+eir_len],0,
                       MAX_EIR_DATA_LEN - eir_len);
            }
        }
        param_length += MAX_EIR_DATA_LEN;
        break;

    case HCI_READ_INQUIRY_RES_TRANS_POWER_LEVEL_OPCODE:
        pkt_param[param_length] = (UCHAR)
                                  HCI_INQUIRY_RES_TRANS_POWER_LEVEL;
        param_length++;
        break;

#ifdef COMPILE_SNIFF_MODE
    case HCI_SNIFF_SUBRATING_OPCODE:
        /** 
         * Copy connection handle
         * Connection handle is passed thru ext_param by
         * handle_sniff_subrating_command function.
         */
        pkt_param[param_length] = ext_param & 0xFF;
        pkt_param[param_length + 1] = ext_param >> 8;
        param_length+=2;
        break;
#endif

#ifdef SCO_OVER_HCI
    case HCI_READ_DEFAULT_ERRONEOUS_DATA_REPORTING_OPCODE:
        pkt_param[param_length] =
            lmp_self_device_data.default_erroneous_data_reporting;
        param_length++;
        break;
    case HCI_WRITE_DEFAULT_ERRONEOUS_DATA_REPORTING_OPCODE:
        /* No extra parameters other than cmd_status */
        break;
#endif //#ifdef SCO_OVER_HCI

    default :
#ifdef _SUPPORT_SECURE_CONNECTION_
        if (IS_BT_SECURE_CONN)
        {
            UINT8 len;
            len = HCI_GENERATE_SECURE_CONN_COMMAND_COMPLETE_EVENT(cmd_opcode,
                    pkt_param, ext_param);
            if (len != 0xFF)
            {
                param_length += len;
                break;
            } 
        }
#endif    
#ifdef VER_CSA4
        if (IS_BT_CSA4)
        {
            UINT8 len;
            len = HCI_GENERATE_CSA4_COMMAND_COMPLETE_EVENT(cmd_opcode,
                    pkt_param, ext_param);
            if (len != 0xFF)
            {
                param_length += len;
                break;
            }            
        }
#endif

#ifdef LE_MODE_EN
        if (IS_BT40)
        {
            UINT8 len;
            len = HCI_GENERATE_4_0_COMMAND_COMPLETE_EVENT(cmd_opcode,
                    pkt_param, ext_param);
            if (len != 0xFF)
            {
                param_length += len;
                break;
            }
        }
#endif

#ifdef VER_3_0
        if (IS_BT30)
        {
            param_length = (UCHAR)(param_length
                    + HCI_GENERATE_3_0_COMMAND_COMPLETE_EVENT(cmd_opcode,
                            pkt_param, ext_param));
        }
#endif
        break;
    }/* end switch(cmd_opcode) */

    return (UCHAR) (param_length - 4);
}

/** 
 * Generates the HCI_REMOTE_HOST_SUPPORTED_FEATURES_NOTIFICAION event.
 * 
 * \param bd_addr BD Address of the remote device.
 * \param remote_host_features Remote host supported features.
 * 
 * \return None.
 */
void hci_generate_rhsfn_event(UCHAR* bd_addr, UCHAR* remote_host_features)
{
    UCHAR event_parameter[HCI_REMOTE_HOST_SUPPORTED_FEATURES_NOTIF_EVENT_LEN];
    memcpy(&event_parameter[0], bd_addr, 6);
    memcpy(&event_parameter[6], remote_host_features, 8);

    hci_generate_event(HCI_REMOTE_HOST_SUPPORTED_FEATURES_NOTIF_EVENT,
            event_parameter,
            HCI_REMOTE_HOST_SUPPORTED_FEATURES_NOTIF_EVENT_LEN);
}

