/***************************************************************************
 Copyright (C) MindTree Ltd.
 This module is a confidential and proprietary property of MindTree and
 a possession or use of this module requires written permission of MindTree.
 ***************************************************************************/

/**
 * \file
 *      Contains the HCI Events generation functions.
 *      All the functions in this file will populate the global
 *      hci_event_pkt_ptr with the respective events and then call
 *      hci_generate_event() function which will alloc a new buffer, copy
 *      event from global buffer and send a signal to HCI_Event task.
 */

/********************************* Logger *************************/
enum { __FILE_NUM__= 9 };
/********************************* Logger *************************/

/* Includes */
#include "lmp_internal.h"
#include "bt_fw_hci_internal.h"
#include "bt_fw_hci_2_1.h"
#include "mem.h"
#include "UartPrintf.h"
#ifdef LE_MODE_EN
#include "le_hci_4_0.h"
#include "le_ll.h"
#include "le_hw_reg.h"
#include "le_ll_driver.h"
#ifdef LE_HW_TEST
#include "le_ll_test.h"
#endif
#include "lmp_pdu_q.h"
#endif
#ifdef _FAKE_SECURE_CONN_
#include "bz_auth_internal.h"
#endif    
#ifdef _DAPE_TEST_GEN_FAKE_ESCO_DATA
extern UINT16 g_handle;
#endif        

#ifdef _DAPE_NO_ROLE_SW_WHEN_ROLE_SW
extern UINT16 g_role_switch_status;
#endif
#ifdef _PAUSE_SCO_FOR_LE_CONN
extern UINT8 lc_sco_pause_status;
#endif            
#ifdef _ROM_CODE_PATCHED_
#ifdef _CCH_8723_RCP_
PF_ROM_CODE_PATCH_FUNC rcp_hci_generate_inquiry_result_event = NULL;
PF_ROM_CODE_PATCH_FUNC rcp_hci_generate_command_complete_event = NULL;
#endif
PF_ROM_CODE_PATCH_FUNC rcp_hci_generate_event = NULL;
#endif

#ifdef COMPILE_ESCO
extern LMP_ESCO_CONNECTION_ENTITY
lmp_esco_connection_entity[LMP_MAX_ESCO_CONN_ENTITIES];
#endif /* COMPILE_ESCO */
#ifdef _DAPE_SLV_SUPTO_IMPROVE
extern UINT8 g_slv_tpoll_disabled;
#endif
#ifdef _DAPE_TEST_NEW_HW_BLOCK_ACL_FOR_LE
extern UINT8 g_enable_le_block_legacy;
extern UINT8 g_block_legacy_for_le_slot_num;
#endif    

/**PATCH_PENDING_ENCRY_WHILE_ROLE_SWITCH*/
#ifdef _PENDING_ENCRY_WHILE_ROLE_SWITCH
extern UINT8 g_need_pend_encry;
extern UINT8 g_encry_pending;
extern OS_SIGNAL g_signal_ptr_encry;
#endif

#ifdef _STOP_AFH_TIMER_WHILE_ROLE_SWITCH
extern UINT8 g_stop_afh_timer;
#endif

/* Temporary declaration */
INT8 get_rssi_value(UINT16 ce_index);


/**
 * Sends HCI_HOST_EVENT_MASKED_SIGNAL siganal to the event task.
 *
 * \param ce_index    Connection Entity Index.
 * \param event_opcode Opcode of the masked event.
 *
 * \return None.
 *
 */
void hci_send_host_event_masked_signal(UINT16 ce_index, UCHAR event_opcode)
{
    OS_SIGNAL sig_send;

    sig_send.type = HCI_HOST_EVENT_MASKED_SIGNAL;
    sig_send.param = (OS_ADDRESS)(UINT32) ce_index;
    sig_send.ext_param = (OS_ADDRESS) (UINT32)event_opcode;

    if (OS_SEND_SIGNAL_TO_TASK (hci_event_task_handle, sig_send) != BT_ERROR_OK)
    {
        HCI_LOG_INFO(LOG_LEVEL_LOW, ERROR_SENDING_SIGNAL,0,0);
    }

    return;
}

/**
 * Allocates a buffer and copies the event data from
 * global buffer and then sends that event to host.
 *
 * \param event_code  Event code of the event to be sent.
 * \param event_param  Event parameter buffer.
 * \param event_param_len  Length of the event parameters to be sent.
 *
 * \return None.
 *
 */
BOOLEAN hci_generate_event(UCHAR event_code, void *event_param,
        UCHAR event_param_len)
{
    HCI_EVENT_PKT *event_pkt;
    OS_SIGNAL     event_signal;

    if (!hci_pass_event_through_event_mask(event_code))
    {
        return FALSE;
    }

#ifdef VER_CSA4
#ifdef _SUPPORT_VER_4_2_
    if (IS_BT42 || IS_BT_CSA4 || IS_BT41 || IS_BT_SECURE_CONN)
#else
    if (IS_BT_CSA4 || IS_BT41 || IS_BT_SECURE_CONN)
#endif        
    {
        if (!hci_pass_event_through_event_mask_page2(event_code))
        {
            return FALSE;
        }
    }
#endif

#ifdef LE_MODE_EN
    if (IS_BT40)
    {
        if (event_code == HCI_LE_MEGA_EVENT)
        {
            UCHAR le_subevt_code = ((UCHAR *) event_param)[0];
            if (!hci_pass_event_through_le_event_mask(le_subevt_code))
            {
                return FALSE;
            }
        }

#ifdef LE_HW_TEST
        if (event_code == HCI_ENCRYPTION_CHANGE_EVENT)
        {
            ll_test_acl_trx_init();
        }
        return FALSE;
#endif
    }
#endif

#ifdef _ROM_CODE_PATCHED_
    if (rcp_hci_generate_event != NULL)
    {
        /* for update new event mask */
        if (rcp_hci_generate_event(&event_code))
        {
            return FALSE;
        }
    }
#endif

    /* Allocate buffer for the event to be sent to host */
    if(OS_ALLOC_BUFFER(tx_table[HCI_EVENT_HANDLER_TASK].pool_handle,
                       (void**)&(event_pkt)) != BT_ERROR_OK)
    {
        LMP_LOG_INFO(LOG_LEVEL_HIGH, 
            XYZ_UNABLE_TO_ALLOCATE_MEMORY_FROM_EVENT_BUFFER_POOL, 0, 0);

        return FALSE;
    }

    /* Copy event to allocated buffer */
    event_pkt->event_opcode = event_code;
    memcpy(event_pkt->event_parameter, event_param, event_param_len);
    event_pkt->param_total_length = event_param_len;

    /* Deliver event to host */
    event_signal.type = HCI_DELIVER_HCI_EVENT_SIGNAL;
    event_signal.param =(OS_ADDRESS)event_pkt;

    if (OS_ISR_SEND_SIGNAL_TO_TASK(hci_event_task_handle, 
                                    event_signal) != BT_ERROR_OK)
    {
        OS_FREE_BUFFER(tx_table[HCI_EVENT_HANDLER_TASK].pool_handle, event_pkt);

        BT_FW_HCI_ERR(UNABLE_TO_GENERATE_HCI_EVENT,1,event_code);
        RT_BT_LOG(RED, BT_FW_HCI_EVENTS_117, 0, 0);
    }
    return TRUE;
}
/**
 * Generates a Command Status Event.
 *
 * \param hci_cmd_opcode Opcode of the command for which the Command Status
 *                       Event is to generated.
 * \param cmd_status Status that is required to be sent to the upper layers
 *                   for the command issued.
 *
 * \return None.
 *
 */
void hci_generate_command_status_event(UINT16 hci_cmd_opcode,
                                       UCHAR cmd_status)
{
    UCHAR event_parameter[HCI_COMMAND_STATUS_EVENT_LEN];
    BT_FW_HCI_INF(GENERATING_COMMAND_STATUS_EVENT,0,0);

    event_parameter[0] = cmd_status;

    /* Num HCI comamnd from self dev datbase */
    if(lmp_self_device_data.num_hci_command_packets != 0)
    {
        event_parameter[1] = lmp_self_device_data.num_hci_command_packets;
    }
    else
    {
        /*
           Controller will atleast 1 command buffer (reserved)
           and one more when the handle_hci_cmd function returns.
           We can safely report '1' even thou the variable says 0.
         */
        event_parameter[1] = 1;
    }
    event_parameter[2] = LSB(hci_cmd_opcode);
    event_parameter[3] = MSB(hci_cmd_opcode);

    hci_generate_event(HCI_COMMAND_STATUS_EVENT, event_parameter,
            HCI_COMMAND_STATUS_EVENT_LEN);
}

/**
 * Sets the bits of the hci_supported_commands[64] variable. This is sent
 * to the host for HCI_READ_SUPPORTED_COMMANDS Command.
 *
 * \param hci_supported_commands Pointer to the 'hci_supported_commands'
 *                               to write to.
 *
 * \return None.
 *
 */
void hci_set_supported_commands(UCHAR *hci_supported_commands)
{
    UCHAR index;

    /* Set all as supported */
    for(index = 0; index<21; index++)
        hci_supported_commands[index] = 0xFF;

    /* Set other octets to zeros */
    for(index = 21; index<64; index++)
        hci_supported_commands[index] = 0;

    /* Clear the ones _not_ supported */
#if 0 // we support cancel create connection and cancel remote name request now 
    hci_supported_commands[ 0] &= 0x7F; /* Cancel Create Connection   */
    hci_supported_commands[ 2] &= 0xEF; /* Cancel Remote name request */
#endif
    hci_supported_commands[ 3] &= 0x03; /* Reserved                   */
    hci_supported_commands[ 4] &= 0xFE; /* Reserved                   */
    hci_supported_commands[ 8] &= 0xCF; /* Reserved                   */
    hci_supported_commands[11] &= 0x9F; /* Reserved                   */
    hci_supported_commands[12] &= 0xF3; /* Reserved                   */
    hci_supported_commands[13] &= 0x0F; /* Reserved                   */
    hci_supported_commands[14] &= 0xE8; /* Reserved                   */
    hci_supported_commands[16] &= 0x3F; /* Reserved                   */
    
    /* Clear all reserved bits */
    hci_supported_commands[17] &= 0xF7; /* Reserved                   */
    hci_supported_commands[18] &= 0x8F; /* Reserved                   */
    hci_supported_commands[20] &= 0x0C; /* Reserved                   */

    /* Disable Read_Country_Code command. */
    hci_supported_commands[15] &= ~(BIT0);
    
    /* Disable Read/Write Encryption_Mode command. */
    hci_supported_commands[8] &= ~(BIT7 | BIT6);

    /* for BQB test (BITE can check following reserved 2 bits to one) */
    hci_supported_commands[8] |= BIT5 | BIT4;

#ifndef COMPILE_PERIODIC_INQUIRY
    /* Disable Periodic_Inquiry_Mode and Exit_Periodic_Inq_mode commands. */
    hci_supported_commands[0] &= ~( BIT2 | BIT3);
#endif

#ifndef ADD_SCO
    hci_supported_commands[0] &= ~(BIT6); /* Disable HCI_ADD_SCO_CONNECTION cmd */
#endif

#ifndef ENABLE_SCO
    /* Disable HCI_READ_SCO_FLOW_CONTROL_ENABLE and
    HCI_WRITE_SCO_FLOW_CONTROL_ENABLE commands */
    hci_supported_commands[10] &= ~(BIT3 | BIT4);
#endif

#if !defined(ENABLE_SCO) && !defined(COMPILE_ESCO)
    /* Disable SETUP_SYNCHRONOUS_CONNECTION, ACCEPT_SYNCHRONOUS_CONNECTION_REQ
    and REJECT_SYNCHRONOUS_CONNECTION_REQ commands */
    hci_supported_commands[16] &= ~(BIT3 | BIT4 | BIT5);
#endif

#ifndef COMPILE_SNIFF_MODE
    /* Disable SNIFF_MODE and EXIT_SNIFF_MODE commands */
    hci_supported_commands[4] &= ~(BIT2 | BIT3);

    /* Disable SNIFF_SUBRATING command */
    hci_supported_commands[17] &= ~(BIT4);
#endif

#ifndef COMPILE_PARK_MODE
    /* Disable PARK_MODE and EXIT_PARK_MODE commands */
    hci_supported_commands[4] &= ~(BIT4 | BIT5);
#endif

#ifndef COMPILE_HOLD_MODE
    /* Disable HOLD_MODE command */
    hci_supported_commands[4] &= ~(BIT1);
    /* Disable READ_HOLD_MODE_ACTIVITY and WRITE_HOLD_MODE_ACTIVITY cmds */
    hci_supported_commands[10] &= ~(BIT0 | BIT1);
#endif

#ifndef COMPILE_ROLE_SWITCH
    /* Disable SWITCH_ROLE command */
    hci_supported_commands[5] &= ~(BIT0);
#endif

#ifndef COMPILE_FEATURE_REQ_EXT
    /* Disable READ_LOCAL_EXTENDED_FEATURES and READ_REMOTE_EXTENDED_FEATURES
    commands */
    hci_supported_commands[2] &= ~(BIT6);
    hci_supported_commands[14] &= ~(BIT6);
#endif

#ifndef TEST_MODE
    /* Disable READ_LOOPBACK_MODE, WRITE_LOOPBACK_MODE and
    ENABLE_DEVICE_UNDER_TEST_MODE commands */
    hci_supported_commands[16] &= ~(BIT0 | BIT1);
    /* The commands HCI_WRITE_SIMPLE_PAIRING_DEBUG_MODE_OPCODE is currently not
    under TEST_MODE define */
#endif

#ifndef COMPILE_AFH_HOP_KERNEL
    /* Disable SET_AFH_HOST_CHANNEL_CLASSIFICATION and READ_AFH_CHANNEL_MAP cmds */
    hci_supported_commands[12] &= ~(BIT1);
    hci_supported_commands[15] &= ~(BIT6);
#endif

#ifndef COMPILE_AFH_CLASSIFICATION
    /* Disable READ_AFH_CHANNEL_ASSESMENT_MODE and
    WRITE_AFH_CHANNEL_ASSESMENT_MODE commands */
    hci_supported_commands[14] &= ~(BIT6);
#endif

#ifndef SCO_OVER_HCI
    /* Disable READ_DEFAULT_ERRONEOUS_DATA_REPORTING and
    WRITE_DEFAULT_ERRONEOUS_DATA_REPORTING commands */
    hci_supported_commands[18] &= ~(BIT2 | BIT3);
#endif //#ifdef SCO_OVER_HCI

#ifndef COMPILE_BROADCAST_ENCRYPTION
    /* Disable MASTER_LINK_KEY command */
    hci_supported_commands[2] &= ~(BIT2);
#endif

//	hci_supported_commands[5] &= ~(BIT5);

#ifdef VER_3_0
    if (IS_BT30)
    {
        /* Enable READ_ENCRYPTION_KEY_SIZE      */
        hci_supported_commands[20] |= BIT4;
        
        /* Enable ENHANCED_POWER_TRANSMIT_LEVEL */
        hci_supported_commands[24] |= BIT0;
    }
#endif

#ifdef LE_MODE_EN
    if (IS_BT40)
    {
        /* enable Read LE Host Support and Write LE Host Support */
        hci_supported_commands[24] |= BIT5 | BIT6;

        /* enable LE Set Event Mask, LE Read Buffer Size, LE Read Local 
           Supported Features, LE Set Random Address, LE Set Advertising 
           Parameters, LE Read Advertising Channel TX Power, LE Set 
           Advertising Data */
        hci_supported_commands[25] = 0xF7;

        /* enable LE Set Scan Response Data, LE Set Advertise Enable, LE Set 
           Scan Parameters, LE Set Scan Enable, LE Create Connection, LE Create
           Connection Cancel, LE Read White List Size, LE Clear White List */
        hci_supported_commands[26] = 0xFF;

        /* enable LE Add Device To White List, LE Remove Device From White List,
           LE Connection Update, LE Set Host Channel Classification, LE Read
           Channel Map, LE Read Remote Used Features, LE Encrypt, LE Rand */
        hci_supported_commands[27] = 0xFF;

        /* enable LE Start Encryption, LE Long Term Key Request Reply, LE Long 
           Term Key Requested Negative Reply, LE Read Supported States, LE 
           Receiver Test, LE Transmitter Test, LE Test End */
        hci_supported_commands[28] = 0x7F;

#ifdef _BT4_2_DATA_EXTENSION_FEATURE_SUPPORT_
        /* LE Set Data Length, LE Read Default Data Length */
        hci_supported_commands[33] |= (BIT7 | BIT6);
        /* LE Write Default Data Length */
        hci_supported_commands[34] |= BIT0;
#endif
    }
#endif

#ifdef _SUPPORT_SECURE_CONNECTION_
    if (IS_BT_SECURE_CONN)
    {
        /* enable Remote OOB Extended Data Request Reply */
        hci_supported_commands[32] |= BIT1;

        /* enable Read Secure Connections Host Support */
        hci_supported_commands[32] |= BIT2;

        /* enable Write Secure Connections Host Support */
        hci_supported_commands[32] |= BIT3;
        
        /* enable Read Authenticated Payload Timeout */
        hci_supported_commands[32] |= BIT4;
        
        /* Write Authenticated Payload Timeout */
        hci_supported_commands[32] |= BIT5;

        /* enable Read Local OOB Extended Data */
        hci_supported_commands[32] |= BIT6;

        /* enable Write Secure Connections Test Mode */
        hci_supported_commands[32] |= BIT7;        
    }

#endif

#ifdef VER_CSA4    
    if (IS_BT_CSA4)
    {
#ifdef _SUPPORT_CSB_TRANSMITTER_    
        /* enable Set Triggered Clock Capture (bit5) */
        hci_supported_commands[30] |= BIT5;     

        /* enable Set Connectionless Slave Broadcast (bit0),
                   Start Synchronization Train (bit2),
                   Set Reserved LT_ADDR (bit4),
                   Delete Reserved LT_ADDR (bit5),
                   Set Connectionless Slave Broadcast Data (bit6) */
        hci_supported_commands[31] |= (BIT0 | BIT2 | BIT4 | BIT5 | BIT6);
        
        /* enable Write Synchronization Train Parameters (bit0) */
        hci_supported_commands[32] |= BIT0;   
#endif

#ifdef _SUPPORT_CSB_RECEIVER_
        /* enable Truncated Page (bit6) and Truncated Page Cancel (bit7) */
        hci_supported_commands[30] |= (BIT6 | BIT7);      

        /* enable Set Connectionless Slave Broadcast Receive (bit1),
                   Receive Synchronization Train (bit3),
                   Read Synchronization Train Parameters (bit7) */
        hci_supported_commands[31] |= (BIT1 | BIT3 | BIT7);       
#endif
    }
#endif

#ifdef _SUPPORT_VER_4_2_
    if (IS_BT42)
    {
#ifdef _BT4_2_DATA_EXTENSION_FEATURE_SUPPORT_
        /* LE Set Data Length, LE Read Default Data Length */
        hci_supported_commands[33] |= (BIT7 | BIT6);
        /* LE Write Default Data Length */
        hci_supported_commands[34] |= BIT0;
#endif
#ifdef _BT4_2_LE_SECURE_CONNECTIONS_SUPPORT_
        hci_supported_commands[34] |= BIT1;
        hci_supported_commands[34] |= BIT2;
#endif

#ifdef _BT4_2_PRIVACY_FEATURE_SUPPORT_
        hci_supported_commands[34] |= BIT3;
        hci_supported_commands[34] |= BIT4;
        hci_supported_commands[34] |= BIT5;
        hci_supported_commands[34] |= BIT6;
        hci_supported_commands[34] |= BIT7;
        hci_supported_commands[35] |= BIT0;
        hci_supported_commands[35] |= BIT1;
        hci_supported_commands[35] |= BIT2;
#endif
    }
#endif

#ifdef _SUPPORT_VER_5_0_
    if (IS_BT50)
    {
#ifdef _BT5_0_LE2MBPS_SUPPORT_    
        hci_supported_commands[35] |= BIT4;
        hci_supported_commands[35] |= BIT5;
        hci_supported_commands[35] |= BIT6;
        hci_supported_commands[35] |= BIT7;
        hci_supported_commands[36] |= BIT0;
#endif
    }
#endif
    return;
}

/**
 * Generates a Command Complete Event. This is used by the
 * Host Controller to transmit return status of a command and returns
 * other event parameters that are specified for the  HCI command.
 *
 * \param command_opcode Opcode of the command for which the Command
 *                       Complete Event is generated.
 * \param status Status that is required to be sent to the upper layers for
 *               the command issued.
 * \param ce_index Index to the connection entity (or will contain connection
 *                 handle when status is error - see 'else->switch').
 * \param arg Extra arguments.
 *
 * \return None.
 *
 */
void hci_generate_command_complete_event(UINT16 hci_cmd_opcode,
        UCHAR cmd_status,UINT16 ce_index, void *arg)
{
    UCHAR event_parameter[HCI_MAX_EVENT_PARAM_LEN];
    UCHAR         param_length;
    UINT16        temp_var;

    INT8         rssi_val;
#ifdef COMPILE_AFH_HOP_KERNEL
    UCHAR         done;
#endif /* COMPILE_AFH_HOP_KERNEL */
    UCHAR         tmp_index;
    LMP_CONNECTION_ENTITY *ce_ptr;
    UINT8         *ptr;

    ce_ptr = &lmp_connection_entity[ce_index];

    BT_FW_HCI_INF(GENERATING_COMAND_COMPLETE_EVENT,0,0);

    /* Num HCI comamnd from self dev database */
    if (hci_cmd_opcode == 0x00)
    {
        event_parameter[0] = 0x00 ;
    }
    else
    {
        if (lmp_self_device_data.num_hci_command_packets != 0)
        {
            event_parameter[0] = lmp_self_device_data.num_hci_command_packets;
        }
        else
        {
            /*
               Controller will atleast 1 command buffer (reserved)
               and one more when the handle_hci_cmd function returns.
               We can safely report '1' even thou the variable says 0.
             */
            event_parameter[0] = 1;
        }
    }

    event_parameter[1] = LSB(hci_cmd_opcode);
    event_parameter[2] = MSB(hci_cmd_opcode);
    event_parameter[3] = cmd_status ;
    param_length = 4;

#ifdef _ROM_CODE_PATCHED_ 
#ifdef _CCH_8723_RCP_     
    if (rcp_hci_generate_command_complete_event != NULL)
    {
        if ( rcp_hci_generate_command_complete_event((void *)&hci_cmd_opcode,
                cmd_status, ce_index, arg, event_parameter, &param_length) )
        {            
            return;
        }
    }     
#endif 
#endif

    if(cmd_status == HCI_COMMAND_SUCCEEDED)
    {
        switch(hci_cmd_opcode)
        {
        case HCI_SET_EVENT_MASK_OPCODE :
        case HCI_RESET_OPCODE :
        case HCI_INQUIRY_CANCEL_OPCODE :
        case HCI_SET_EVENT_FILTER_OPCODE :
        case HCI_WRITE_INQUIRY_SCAN_ACTIVITY_OPCODE :
        case HCI_WRITE_NUM_BROADCAST_RETRANSMISSIONS_OPCODE:
        case HCI_WRITE_SCO_FLOW_CONTROL_ENABLE_OPCODE:
        case HCI_SET_CONTROLLER_TO_HOST_FLOW_CONTROL_OPCODE:
        case HCI_HOST_BUFFER_SIZE_OPCODE:
        case HCI_WRITE_CURRENT_IAC_LAP_OPCODE:
        case HCI_HOST_NUMBER_OF_COMPLETED_PACKETS_OPCODE:

#ifdef TEST_MODE
        case HCI_WRITE_LOOPBACK_MODE_OPCODE:
        case HCI_ENABLE_DEVICE_UNDER_TEST_MODE_OPCODE:
#endif /* TEST_MODE */

            break;

        case HCI_READ_LOCAL_NAME_OPCODE :
            memcpy(&(event_parameter[4]),
                   lmp_self_device_data.local_name, LMP_MAX_NAME_LENGTH);
            if (lmp_self_device_data.local_name_len != LMP_MAX_NAME_LENGTH)
            {
                event_parameter[4+lmp_self_device_data.local_name_len] = 0x0;
            }
            param_length = 4 + LMP_MAX_NAME_LENGTH;
            break;

        case HCI_READ_CONNECTION_ACCEPT_TIMEOUT_OPCODE :
            temp_var = lmp_self_device_data.conn_accept_timeout ;
            temp_var = TIMER_VAL_TO_SLOT_VAL(temp_var) ;
            BT_FW_HCI_INF(TIMER_VALUE_CONVERTED_TO_SLOTS,1, temp_var);
            event_parameter[4] = LSB(temp_var);
            event_parameter[5] = MSB(temp_var);
            param_length = 6;

            break;

        case HCI_READ_PAGE_TIMEOUT_OPCODE :
            event_parameter[4] = LSB(lmp_self_device_data.page_timeout);
            event_parameter[5] = MSB(lmp_self_device_data.page_timeout);
            param_length = 6;
            break;

        case HCI_READ_SCAN_ENABLE_OPCODE :
            event_parameter[4] = lmp_self_device_data.scan_enable;
            param_length = 5;

            break;


        case HCI_READ_PAGE_SCAN_ACTIVITY_OPCODE :
            temp_var = lmp_self_device_data.page_scan_interval;
            event_parameter[4] = LSB(temp_var);
            event_parameter[5] = MSB(temp_var);
            temp_var = lmp_self_device_data.page_scan_window;
            event_parameter[6] = LSB(temp_var);
            event_parameter[7] = MSB(temp_var);
            param_length = 8;

            break;

        case HCI_READ_INQUIRY_SCAN_ACTIVITY_OPCODE :
            temp_var = lmp_self_device_data.inquiry_scan_interval;
            event_parameter[4] = LSB(temp_var);
            event_parameter[5] = MSB(temp_var);
            temp_var = lmp_self_device_data.inquiry_scan_window;
            event_parameter[6] = LSB(temp_var);
            event_parameter[7] = MSB(temp_var);
            param_length = 8;
            break;

        case HCI_READ_CLASS_OF_DEVICE_OPCODE:
            ptr = (UINT8*)&lmp_self_device_data.class_of_device;
            event_parameter[4] = *ptr;
            event_parameter[5] = *(ptr+1);
            event_parameter[6] = *(ptr+2);            
            param_length = 7;
            break;

        case HCI_READ_VOICE_SETTING_OPCODE :
            //RT_BT_LOG(GREEN, BT_FW_HCI_EVENTS_469, 1, 
            //            lmp_self_device_data.voice_setting);
            event_parameter[4] = LSB(lmp_self_device_data.voice_setting);
            event_parameter[5] = MSB(lmp_self_device_data.voice_setting);
            param_length = 6;
            break;

        case HCI_WRITE_LINK_SUPERVISION_TIMEOUT_OPCODE :
            event_parameter[4] = LSB(ce_ptr->connection_type.connection_handle);
            event_parameter[5] = MSB(ce_ptr->connection_type.connection_handle);
            param_length = 6 ;
            break;

        case HCI_READ_PAGE_SCAN_PERIOD_MODE_OPCODE :
            event_parameter[4] = lmp_self_device_data.page_scan_period_mode;
            param_length = 5;
            break;

        case HCI_READ_PAGE_SCAN_MODE_OPCODE :
            event_parameter[4] = lmp_self_device_data.page_scan_mode;
            param_length = 5;
            break;

        case HCI_READ_LOCAL_VERSION_INFORMATION_OPCODE :

#ifdef _DAPE_UNIFY_VERSION_FEATURE_UINT
            event_parameter[4] = lmp_self_device_data.hci_version;
            event_parameter[5] = LSB(BT_FW_HCI_REVISION);
            event_parameter[6] = MSB(BT_FW_HCI_REVISION);
            event_parameter[7] = lmp_self_device_data.lmp_version;
#else            
            if (IS_BT42)
            {
                event_parameter[4] = BT_FW_HCI_VERSION_BT42;
            }
            else if (IS_BT41)
            {
                event_parameter[4] = BT_FW_HCI_VERSION_BT41;
            }
            else if (IS_BT40)
            {
                event_parameter[4] = BT_FW_HCI_VERSION_BT40;
            }
            else if (IS_BT30)
            {
                event_parameter[4] = BT_FW_HCI_VERSION_BT30;
            }
            else
            {
                event_parameter[4] = BT_FW_HCI_VERSION_BT21_PLUS_EDR;
            }                           

            event_parameter[5] = LSB(BT_FW_HCI_REVISION);
            event_parameter[6] = MSB(BT_FW_HCI_REVISION);

            if (IS_BT42)
            {
                event_parameter[7] = BT_FW_LMP_VERSION_BT42;
            }
            else if (IS_BT41)
            {
                event_parameter[7] = BT_FW_LMP_VERSION_BT41;
            }
            else if (IS_BT40)
            {
                event_parameter[7] = BT_FW_LMP_VERSION_BT40;
            }
            else if (IS_BT30)
            {
                event_parameter[7] = BT_FW_LMP_VERSION_BT30;
            }
            else
            {
                event_parameter[7] = BT_FW_LMP_VERSION_BT21_PLUS_EDR;
            }
#endif            
            event_parameter[8] = LSB(otp_str_data.bt_manufacturer_name);
            event_parameter[9] = MSB(otp_str_data.bt_manufacturer_name);
            event_parameter[10] = LSB(BT_FW_LMP_SUBVERSION);
            event_parameter[11] = MSB(BT_FW_LMP_SUBVERSION);
            param_length = 12;
            break;

        case HCI_READ_LOCAL_SUPPORTED_FEATURES_OPCODE :
            memcpy(&( event_parameter[4]),
                   lmp_feature_data.feat_page0, LMP_FEATURES_SIZE);

            param_length += LMP_FEATURES_SIZE  ;
            break ;


#ifdef COMPILE_FEATURE_REQ_EXT
        case HCI_READ_LOCAL_EXTENDED_FEATURES_OPCODE:
            {
                UINT8 page_num = (UINT8)((UINT32) arg);
                event_parameter[4] = page_num;
                event_parameter[5] = lmp_get_max_features_page();
                lmp_fill_features_page(page_num, &event_parameter[6]);
                param_length += 10;
            }
            break;
#endif

        case HCI_READ_LOCAL_SUPPORTED_COMMANDS_OPCODE :
            hci_set_supported_commands(&(event_parameter[4]));
            param_length += 64;
            break;

        case HCI_READ_BD_ADDR_OPCODE:

            memcpy(&event_parameter[4], otp_str_data.bt_bd_addr, 
                                                    LMP_BD_ADDR_SIZE);

            param_length += LMP_BD_ADDR_SIZE;
            break;

        case HCI_READ_BUFFER_SIZE_OPCODE:
            event_parameter[4] = LSB(otp_str_data.bt_read_buffer_size);
            event_parameter[5] = MSB(otp_str_data.bt_read_buffer_size);
            event_parameter[6] = HCI_SYNCHRONOUS_DATA_PAYLOAD_SIZE;
            event_parameter[7] = LSB(BT_FW_TOTAL_ACL_PKTS_FROM_HOST);
            event_parameter[8] = MSB(BT_FW_TOTAL_ACL_PKTS_FROM_HOST);
            event_parameter[9] = LSB(BT_FW_TOTAL_SYNCHRONOUS_PKTS_FROM_HOST);
            event_parameter[10] = MSB(BT_FW_TOTAL_SYNCHRONOUS_PKTS_FROM_HOST);
            param_length = 11;

#if 0
            RT_BT_LOG(GREEN, BT_FW_HCI_EVENTS_584, 2,
                      otp_str_data.bt_read_buffer_size,
                      HCI_SYNCHRONOUS_DATA_PAYLOAD_SIZE);
#endif
            break;

#ifdef COMPILE_DEPRECATED_COMMANDS
        case HCI_READ_COUNTRY_CODE_OPCODE:
            event_parameter[4] = otp_str_data.bt_country_code;
            param_length++;
            break;
#endif

        case HCI_ROLE_DISCOVERY_OPCODE :
            event_parameter[4] = LSB(ce_ptr->connection_type.connection_handle);
            event_parameter[5] = MSB(ce_ptr->connection_type.connection_handle);
            if (ce_ptr->remote_dev_role == MASTER)
            {
                event_parameter[6] = SLAVE;
            }
            else
            {
                event_parameter[6] = MASTER;
            }
            param_length = 7;
            break;

        case HCI_WRITE_LINK_POLICY_SETTINGS_OPCODE :
            event_parameter[4] = LSB(ce_ptr->connection_type.connection_handle);
            event_parameter[5] = MSB(ce_ptr->connection_type.connection_handle);
            param_length = 6 ;
            break;

#ifndef _DISABLE_HCI_QOS_CONTROL_
        case HCI_READ_FAILED_CONTACT_COUNTER_OPCODE:
            event_parameter[4] = LSB(ce_ptr->connection_type.connection_handle);
            event_parameter[5] = MSB(ce_ptr->connection_type.connection_handle);
            event_parameter[6] = LSB(ce_ptr->failed_contact_counter);
            event_parameter[7] = MSB(ce_ptr->failed_contact_counter);
            param_length = 8;
            break;

        case HCI_RESET_FAILED_CONTACT_COUNTER_OPCODE:
            event_parameter[4] = LSB(ce_ptr->failed_contact_counter);
            event_parameter[5] = MSB(ce_ptr->failed_contact_counter);
            param_length = 6;
            break;

        case HCI_GET_LINK_QUALITY_OPCODE:
            event_parameter[4] = LSB(ce_ptr->connection_type.connection_handle);
            event_parameter[5] = MSB(ce_ptr->connection_type.connection_handle);
            event_parameter[6] = ce_ptr->link_quality ;
            param_length = 7;
            break;
#endif

        case HCI_READ_RSSI:
#ifdef LE_MODE_EN
            if ((ce_index >= LMP_MAX_CONN_HANDLES) && IS_BT40)
            {
                LL_CONN_HANDLE_UNIT *p_handle;
                
                ce_index -= LMP_MAX_CONN_HANDLES;
                p_handle = &ll_manager.conn_unit.handle[ce_index];
                rssi_val = lc_calculate_log_from_rssi(p_handle->last_rssi);
                
                event_parameter[4] = p_handle->conn_handle & 0xFF;
                event_parameter[5] = p_handle->conn_handle >> 8;
            }
            else
#endif
            {
                rssi_val = get_rssi_value(ce_index);

                event_parameter[4] = LSB(ce_ptr->connection_type.connection_handle);
                event_parameter[5] = MSB(ce_ptr->connection_type.connection_handle);
            }
            event_parameter[6] = rssi_val;
            param_length = 7;
            break;

        case HCI_READ_CLOCK_OPCODE:
            if (ce_index != 0xff)
            {
                UINT16 lcl_clk, lcl_clk_addr, lcl_accuracy, con_handle;

                if (ce_index == 0xfe)
                {
                    lcl_clk_addr = NATIVE_CLOCK1_REGISTER;
                    lcl_accuracy = 0x0;
                    con_handle = 0x0;
                }
                else
                {
                    if (ce_ptr->remote_dev_role == SLAVE)
                    {
                        con_handle = ce_ptr->connection_type.connection_handle;

                        lcl_clk_addr = NATIVE_CLOCK1_REGISTER;

                        /* Populate the accuracy,
                        this is hard-coded as 20ppm */
                        lcl_accuracy = 0x14;
                    }
                    else
                    {
                        con_handle = ce_ptr->connection_type.connection_handle;

                        lcl_clk_addr = PICONET_CLOCK1_REGISTER;

                        /* Populate the accuracy,
                        this is hard-coded as 250ppm */
                        lcl_accuracy = 0xfa;
                    }
                }

                /* Populate the connection handle. */
                event_parameter[4] = (UCHAR) con_handle;
                event_parameter[5] = (UCHAR) (con_handle >> 8);
                
                /* Populate the clock. */
                lcl_clk = BB_read_baseband_register( (UCHAR) lcl_clk_addr);
                event_parameter[6] = (UCHAR) lcl_clk;
                event_parameter[7] = (UCHAR) (lcl_clk >> 8);

                lcl_clk = BB_read_baseband_register((UCHAR)(lcl_clk_addr + 2));

                event_parameter[8] = (UCHAR) lcl_clk;
                event_parameter[9] = (UCHAR) (lcl_clk >> 8);

                /* Populate the accuracy. */
                event_parameter[10] = (UCHAR) lcl_accuracy;
                event_parameter[11] = (UCHAR) (lcl_accuracy >> 8);
            }
            else
            {
                /* No connection. */
                for (temp_var = 0; temp_var < 8; temp_var++)
                {
                    event_parameter[4 + temp_var] = 0x00;
                }
            }
            param_length = 12;
            break;

#ifdef COMPILE_AFH_HOP_KERNEL
        case HCI_READ_AFH_CHANNEL_MAP_OPCODE:

            done = FALSE;

            event_parameter[4] = LSB(ce_ptr->connection_type.connection_handle);
            event_parameter[5] = MSB(ce_ptr->connection_type.connection_handle);
            
            if (ce_ptr->afh_mode == AFH_ENABLE)
            {
                event_parameter[6] = AFH_ENABLE;

                /* Return the map at map index */
                memcpy(&event_parameter[7],
                       &ce_ptr->afh_map[0],LMP_AFH_MAP_SIZE);
                done = TRUE;
            }
            else
            {
                /*
                 * If we are not able to get a proper AFH MAP then
                 * fill the parameter with 'AFH_DISABLE' result.
                **/
                event_parameter[6] = AFH_DISABLE;

                /* Populate the invalid value which we have assume 0xff */
                memset(&event_parameter[7], 0xFF,LMP_AFH_MAP_SIZE);
            }
            event_parameter[7+LMP_AFH_MAP_SIZE-1] &= 0x7F;
            param_length = 7 + LMP_AFH_MAP_SIZE;
            break;
#endif /* COMPILE_AFH_HOP_KERNEL */

        case HCI_READ_LINK_POLICY_SETTNGS_OPCODE :
            event_parameter[4] = LSB(ce_ptr->connection_type.connection_handle);
            event_parameter[5] = MSB(ce_ptr->connection_type.connection_handle);
            event_parameter[6] = LSB(ce_ptr->link_policy_settings);
            event_parameter[7] = MSB(ce_ptr->link_policy_settings);
            param_length = 8;
            break;

        case HCI_READ_DEFAULT_LINK_POLICY_SETTNGS_OPCODE :
            event_parameter[4] = LSB(lmp_self_device_data.default_link_policy_settings);
            event_parameter[5] = MSB(lmp_self_device_data.default_link_policy_settings);
            param_length = 6;
            break;

        case HCI_READ_NUM_BROADCAST_RETRANSMISSIONS_OPCODE:
#ifdef BROADCAST_DATA
            event_parameter[4] = lmp_self_device_data.num_broadcast_retran;
#else /* BROADCAST_DATA */
            event_parameter[4] = 0x0;
#endif /* BROADCAST_DATA */
            param_length = 5;
            break;

#ifdef COMPILE_HOLD_MODE
        case HCI_READ_HOLD_MODE_ACTIVITY_OPCODE:
            event_parameter[4] = lmp_self_device_data.hold_mode_activity;
            param_length = 5;
            break;
#endif

        case HCI_FLUSH_OPCODE:
            event_parameter[4] = LSB(ce_ptr->connection_type.connection_handle);
            event_parameter[5] = MSB(ce_ptr->connection_type.connection_handle);
            param_length = 6;
            break;

#ifndef _DISABLE_HCI_HOST_FLOW_CONTROL_
        case HCI_READ_SCO_FLOW_CONTROL_ENABLE_OPCODE:
            event_parameter[4] = lmp_self_device_data.sco_flow_control_enable;
            param_length = 5;
            break;
#endif

        case HCI_READ_NUMBER_OF_SUPPORTED_IAC_OPCODE:
            event_parameter[4] = LMP_MAX_IAC_LAPS;
            param_length = 5;
            break;

        case HCI_READ_CURRENT_IAC_LAP_OPCODE:
            event_parameter[4] = lmp_self_device_data.num_supported_iac;
            param_length = 5;

            tmp_index = 0;
            while (tmp_index < lmp_self_device_data.num_supported_iac)
            {
                ptr = &event_parameter[param_length];
                ptr[0] = lmp_self_device_data.iac_lap[tmp_index][0];
                ptr[1] = lmp_self_device_data.iac_lap[tmp_index][1];
                ptr[2] = lmp_self_device_data.iac_lap[tmp_index][2];
                param_length += 3;
                tmp_index++;
            }
            break;

        case HCI_READ_TRANSMIT_POWER_LEVEL_OPCODE:
#ifdef LE_MODE_EN
            if ((ce_index >= LMP_MAX_CE_DATABASE_ENTRIES) && IS_BT40)
            {
                LL_CONN_HANDLE_UNIT *ll_handle;
                ce_index -= LMP_MAX_CE_DATABASE_ENTRIES;
                ll_handle = &ll_manager.conn_unit.handle[ce_index];

                event_parameter[4] = ll_handle->conn_handle & 0xFF;
                event_parameter[5] = ll_handle->conn_handle >> 8;
                event_parameter[6] = ll_handle->tx_power_level;
                param_length = 7;

                break;
            }
#endif

            event_parameter[4] = LSB(ce_ptr->connection_type.connection_handle);
            event_parameter[5] = MSB(ce_ptr->connection_type.connection_handle);
            event_parameter[6] = ce_ptr->transmit_power_level;
            param_length = 7;
            break;

        case HCI_READ_AUTOMATIC_FLUSH_TIMEOUT_OPCODE:
            event_parameter[4] = LSB(ce_ptr->connection_type.connection_handle);
            event_parameter[5] = MSB(ce_ptr->connection_type.connection_handle);
            event_parameter[6] = LSB(ce_ptr->flush_timeout);
            event_parameter[7] = MSB(ce_ptr->flush_timeout);
            param_length = 8;
            break;

        case HCI_WRITE_AUTOMATIC_FLUSH_TIMEOUT_OPCODE:
            event_parameter[4] = LSB(ce_ptr->connection_type.connection_handle);
            event_parameter[5] = MSB(ce_ptr->connection_type.connection_handle);
            param_length = 6;
            break;

#ifdef TEST_MODE
        case HCI_READ_LOOPBACK_MODE_OPCODE:
            if ((lmp_self_device_data.test_mode == HCI_DEVICE_UNDER_TEST_MODE))
            {
                event_parameter[4] = HCI_NO_LOOPBACK_MODE;
            }
            else
            {
                event_parameter[4] = lmp_self_device_data.test_mode;
            }
            param_length = 5;
            break;
#endif /* TEST_MODE */

        case HCI_READ_LINK_SUPERVISION_TIMEOUT_OPCODE :
            event_parameter[4] = LSB(ce_ptr->connection_type.connection_handle);
            event_parameter[5] = MSB(ce_ptr->connection_type.connection_handle);
            event_parameter[6] = LSB(ce_ptr->link_supervision_timeout);
            event_parameter[7] = MSB(ce_ptr->link_supervision_timeout);
            param_length = 8;
            break;

#ifdef _CCH_RTL8723A_B_CUT
        case HCI_CREATE_CONNECTION_CANCEL_OPCODE :
#endif
        case HCI_REMOTE_NAME_REQUEST_CANCEL_OPCODE :
            memcpy(&(event_parameter[4]), 
                    &rem_name_cancel_bd_addr[0],  LMP_BD_ADDR_SIZE);
            param_length += LMP_BD_ADDR_SIZE;
            break;
            
        default:
            param_length = (UCHAR)(4 + HCI_GENERATE_1_2_COMMAND_COMPLETE_EVENT(
                    hci_cmd_opcode, event_parameter, ce_index));
            break;
        }
    }
    else
    {
        switch(hci_cmd_opcode)
        {
        case HCI_READ_RSSI:
            event_parameter[4] = LSB(ce_index);
            event_parameter[5] = MSB(ce_index);

            /* One byte for RSSI. */
            param_length = 6;
            break;

        case HCI_FLUSH_OPCODE: /* Fall through */
        case HCI_READ_AUTOMATIC_FLUSH_TIMEOUT_OPCODE: /* Fall through */
        case HCI_WRITE_AUTOMATIC_FLUSH_TIMEOUT_OPCODE: /* Fall through */
        case HCI_READ_TRANSMIT_POWER_LEVEL_OPCODE: /* Fall through */
        case HCI_WRITE_LINK_SUPERVISION_TIMEOUT_OPCODE : /* Fall through */
        case HCI_READ_LINK_SUPERVISION_TIMEOUT_OPCODE : /* Fall through */
        case HCI_READ_FAILED_CONTACT_COUNTER_OPCODE: /* Fall through */
        case HCI_RESET_FAILED_CONTACT_COUNTER_OPCODE: /* Fall through */
        case HCI_GET_LINK_QUALITY_OPCODE: /* Fall through */
        case HCI_READ_AFH_CHANNEL_MAP_OPCODE: /* Fall through */
        case HCI_ROLE_DISCOVERY_OPCODE: /* Fall through */
        case HCI_READ_CLOCK_OPCODE: /* Fall through */
        case HCI_WRITE_LINK_POLICY_SETTINGS_OPCODE: /* Fall through */
        case HCI_READ_LINK_POLICY_SETTNGS_OPCODE:
            /*
               If there is an error,
               connection handle is returned in ce_index
             */
            event_parameter[4] = LSB(ce_index);
            event_parameter[5] = MSB(ce_index);
            param_length = 6;
            break;

#ifdef _CCH_RTL8723A_B_CUT
        case HCI_CREATE_CONNECTION_CANCEL_OPCODE :
        case HCI_REMOTE_NAME_REQUEST_CANCEL_OPCODE :
            memcpy(&(event_parameter[4]), 
                    &rem_name_cancel_bd_addr[0],  LMP_BD_ADDR_SIZE);
            param_length += LMP_BD_ADDR_SIZE;
            break;
#endif

        default:
            /* Try 2_1 opcodes */
            param_length = (UCHAR)(4 +
                       HCI_GENERATE_2_1_COMMAND_COMPLETE_EVENT(
                       hci_cmd_opcode,event_parameter,ce_index));
            break;
        }
    }

    hci_generate_event(HCI_COMMAND_COMPLETE_EVENT, event_parameter,
            param_length);

    return;
}

/**
 * Generates a Number Of Completed Packets Event and is used
 * by the Host Controller to indicate to the Host how many data packets
 * have been completed transmitted or flushed) for each connection handle
 * after the previous such event was sent to the Host.
 *
 * \param None.
 *
 * \return None.
 *
 */
void hci_generate_number_of_completed_packets_event(void)
{
    UCHAR event_parameter[HCI_MAX_EVENT_PARAM_LEN];
    UINT16 ce_index ;
    UCHAR  param_length = 0x01; /* pkt[0] will have number of handles */
    UCHAR  number_of_handles = 0x00 ;
    UCHAR  compl_pkts_param[HCI_MAX_EVENT_PARAM_LEN];

    LMP_CONNECTION_ENTITY *ce_ptr;

    /* Send number of completed packets for all the connection handles */
    for(ce_index = 0; ce_index < LMP_MAX_CE_DATABASE_ENTRIES; ce_index++)
    {
        ce_ptr = &lmp_connection_entity[ce_index];

        if(ce_ptr->ce_status != LMP_STANDBY)
        {
            if(ce_ptr->hc_num_of_completed_packets)
            {
                event_parameter[param_length] = LSB(ce_ptr->connection_type.connection_handle);
                event_parameter[param_length+1] = MSB(ce_ptr->connection_type.connection_handle);
                compl_pkts_param[param_length] = LSB(ce_ptr->hc_num_of_completed_packets);
                compl_pkts_param[param_length+1] = MSB(ce_ptr->hc_num_of_completed_packets);
                number_of_handles++;

                ce_ptr->hc_num_of_completed_packets = 0;
                param_length += 2;
            }
        }
    }

#ifdef COMPILE_ESCO
    if (lmp_self_device_data.sco_flow_control_enable)
    {
        LMP_ESCO_CONNECTION_ENTITY *esco_ce_ptr;

        /* Start scanning the esco connection entity list */
        /* Send number of completed packets for all the esco connection handles */
        for (ce_index = 0 ; ce_index < LMP_MAX_ESCO_CONN_ENTITIES ; ce_index++)
        {
            esco_ce_ptr = &lmp_esco_connection_entity[ce_index];
            ce_ptr = &lmp_connection_entity[ce_index];

            if (esco_ce_ptr->status != NO_CONNECTION)
            {
                if (esco_ce_ptr->num_of_completed_esco_packets)
                {
                    /* Number of connection handles  */
                    number_of_handles++;

                    event_parameter[param_length] = LSB(esco_ce_ptr->conn_handle);
                    event_parameter[param_length+1] = MSB(esco_ce_ptr->conn_handle);
                    compl_pkts_param[param_length] = LSB(ce_ptr->hc_num_of_completed_packets);
                    compl_pkts_param[param_length+1] = MSB(ce_ptr->hc_num_of_completed_packets);
                    param_length += 2 ;

                    /* Reset the num_of_complete_packets */
                    esco_ce_ptr->num_of_completed_esco_packets = 0;
                }
            }
        }
    }
#endif /* COMPILE_ESCO */

#ifdef BROADCAST_DATA

    /* Check for Active Slave Broadcast num complete */
    if(lmp_self_device_data.asb_num_of_completed_packets)
    {
        event_parameter[param_length] = LSB(lmp_self_device_data.bc_conn_handle);
        event_parameter[param_length+1] = MSB(lmp_self_device_data.bc_conn_handle);
        compl_pkts_param[param_length] = LSB(lmp_self_device_data.asb_num_of_completed_packets);
        compl_pkts_param[param_length+1] = MSB(lmp_self_device_data.asb_num_of_completed_packets);
        param_length += 2 ;
        number_of_handles++;
        lmp_self_device_data.asb_num_of_completed_packets = 0;
    }
    /* Check for Parked slave Broadcast num complete */
    if(lmp_self_device_data.psb_num_of_completed_packets)
    {
        event_parameter[param_length] = LSB(lmp_self_device_data.park_bc_conn_handle);
        event_parameter[param_length+1] = MSB(lmp_self_device_data.park_bc_conn_handle);
        compl_pkts_param[param_length] = LSB(lmp_self_device_data.psb_num_of_completed_packets);
        compl_pkts_param[param_length+1] = MSB(lmp_self_device_data.psb_num_of_completed_packets);
        param_length += 2 ;
        number_of_handles++;
        lmp_self_device_data.psb_num_of_completed_packets = 0;
    }
#endif /* BROADCAST_DATA */

#ifdef LE_MODE_EN
    if (IS_BT40)
    {
        /* Start scanning the LL connection entity list */
        /* Send number of completed packets for all the LL connection handles */
        for (ce_index = 0 ; ce_index < LL_MAX_CONNECTION_UNITS; ce_index++)
        {
            LL_CONN_HANDLE_UNIT *phandle;

            if (!ll_manager.conn_unit.enable)
            {
                break;
            }

            phandle = &ll_manager.conn_unit.handle[ce_index];
            if (phandle->connected && (phandle->hc_complete_pkts > 0))
            {
                event_parameter[param_length] = LSB(phandle->conn_handle);
                event_parameter[param_length + 1] = MSB(phandle->conn_handle);
                compl_pkts_param[param_length] = LSB(phandle->hc_complete_pkts);
                compl_pkts_param[param_length + 1] = 
                                                MSB(phandle->hc_complete_pkts);
                param_length += 2;
                number_of_handles++;
                phandle->hc_complete_pkts = 0;
            }
        }
    }
#endif

    if (number_of_handles)
    {
        memcpy(&event_parameter[param_length],
               &compl_pkts_param[1],number_of_handles<<1);
        param_length = (UCHAR)(param_length + (number_of_handles <<1));
        event_parameter[0] = number_of_handles ;
        hci_generate_event(HCI_NUMBER_OF_COMPLETED_PACKETS_EVENT,
                event_parameter, param_length);
    }
}
/**
 * Generates Inquiry complete event that
 * indicates that the inquiry process is complete
 *
 * \param status Status for the command complete event.
 *
 * \return None.
 *
 */
void hci_generate_inquiry_complete_event(UCHAR inquiry_status)
{
    UCHAR event_parameter[HCI_INQUIRY_COMPLETE_EVENT_LEN];
    BT_FW_HCI_INF(GENERATING_INQUIRY_COMPLETE_EVENT,0,0);

    event_parameter[0] = inquiry_status;

    /* Reset the inquiry result table */
    memset(lmp_inquiry_result_data,
           0,
           LMP_MAX_INQUIRY_RESULT_DATA * sizeof(LMP_INQUIRY_RESULT_DATA));

    hci_generate_event(HCI_INQUIRY_COMPLETE_EVENT, event_parameter,
            HCI_INQUIRY_COMPLETE_EVENT_LEN);
    lmp_periodic_inquiry = FALSE;
}
/**
 * Generates a Inquiry Result Event that indicates
 * that a bluetooth device or multiple bluetooth devices have responded
 * so far during the current Inquiry Process.
 *
 * \param inquiry_data_index Index of the LMP_INQUIRY_RESULT_DATA structure.
 * \param num_responses      Number of responses received.
 * \param rssi_flag          if TRUE generates event with RSSI.
 *
 * \return None.
 *
 */
void hci_generate_inquiry_result_event(UCHAR inquiry_data_index,
                                       UCHAR num_responses, UCHAR rssi_flag)
{
    UCHAR event_parameter[HCI_MAX_EVENT_PARAM_LEN];
    UCHAR param_length = 0x00 ;
    UCHAR count = 0x0;
    UCHAR inquiry_circular_index ;
    UCHAR event_code = HCI_INQUIRY_RESULT_EVENT;

    BT_FW_HCI_INF(GENERATING_INQUIRY_RESULT_EVENT,0,0);

    event_parameter[param_length] = num_responses;
    param_length++ ;

    inquiry_circular_index = inquiry_data_index ;


#ifdef _ROM_CODE_PATCHED_
#ifdef _CCH_8723_RCP_
// RCP for inquiry response with too small rssi value
        if (rcp_hci_generate_inquiry_result_event != NULL)
        {
            if ( rcp_hci_generate_inquiry_result_event((void *)&inquiry_data_index, num_responses, rssi_flag) )
            {
                return;
            }
        }    
#endif
#endif

    do
    {
        count++;

        memcpy(&event_parameter[param_length],
               lmp_inquiry_result_data[inquiry_circular_index].bd_addr,
               LMP_BD_ADDR_SIZE);
        param_length += LMP_BD_ADDR_SIZE;

        event_parameter[param_length] =
            lmp_inquiry_result_data[inquiry_circular_index].page_scan_repetition_mode;

        /* workaround the page scan repetition mode field to be valid if it is 
           an invalid value - austin */
        if (event_parameter[param_length] > 0x02)
        {
            event_parameter[param_length] = 0x01;
        }
        
        param_length++ ;

        event_parameter[param_length] =
            lmp_inquiry_result_data[inquiry_circular_index].page_scan_period_mode;
        param_length++ ;

        if(rssi_flag == FALSE)
        {
            event_parameter[param_length] = 0x0;
            /* Depreciated in v1.2 - set to reserved 0
            lmp_inquiry_result_data[inquiry_circular_index].page_scan_mode;
            */
            param_length++ ;
        }

        bt_fw_ultostr(& event_parameter[param_length],
                      lmp_inquiry_result_data[inquiry_circular_index].class_of_device,3);

        param_length += LMP_CLASS_OF_DEVICE_LEN ;

        event_parameter[param_length] = LSB(lmp_inquiry_result_data[inquiry_circular_index].clock_offset);
        event_parameter[param_length+1] = MSB(lmp_inquiry_result_data[inquiry_circular_index].clock_offset);

        param_length += 2 ;
        
#ifdef COMPILE_INQ_RES_EVENT_WITH_RSSI
        if (rssi_flag == TRUE)
        {
            event_code = HCI_INQUIRY_RESULT_WITH_RSSI_EVENT;
            
            event_parameter[param_length] =
                     lmp_inquiry_result_data[inquiry_circular_index].rssi;
            param_length++;
        }
#endif
        inquiry_circular_index++ ;

        inquiry_circular_index &= (LMP_MAX_INQUIRY_RESULT_DATA - 1);

    }
    while(count < num_responses );

    hci_generate_event(event_code, event_parameter, param_length);

    return;

}

/**
 * Generates a Connection Complete Event that is used to
 * indicate to the host that a new connection has been established.
 *
 * \param status           Status of the connection.
 * \param conn_handle      Connection handle.
 * \param bd_addr          Remote Bluetooth device address.
 * \param link_type        Link Type.
 * \param encryption_mode  Encryption mode
 *
 * \return None.
 *
 */
void hci_generate_connection_complete_event(UCHAR status, UINT16 conn_handle,
        UCHAR* bd_addr, UCHAR link_type, UCHAR encryption_mode)
{
    UCHAR event_parameter[11];
    UINT16 ce_index;

    BT_FW_HCI_INF(GENERATING_CONNECTION_COMPLETE_EVENT,0,0);

    if (link_type == ACL_LINK)
    {
        if (LMP_GET_CE_INDEX_FROM_CONN_HANDLE(conn_handle,&ce_index) !=
                API_SUCCESS)
        {
            BT_FW_HCI_ERR(GETTING_CE_INDEX_FROM_CONN_HANDLE_FAILED,0,0);
            return;
        }
        if (status == HCI_COMMAND_SUCCEEDED)
        {
            lmp_self_device_data.number_of_acl_conn++ ;

        }
        /* notify auth module regarding connection complete */
        bz_auth_handle_acl_connection_complete(ce_index, status);
    }

    event_parameter[0] = status ;
    event_parameter[1] = LSB(conn_handle);
    event_parameter[2] = MSB(conn_handle);
    memcpy(& event_parameter[3],bd_addr, LMP_BD_ADDR_SIZE);
    event_parameter[9] = link_type;
    event_parameter[10] = encryption_mode;

    hci_generate_event(HCI_CONNECTION_COMPLETE_EVENT, event_parameter, 11);
    
#ifdef _BRUCE_FIX_PCM_NO_ENABLE_FOR_IOT            
    if((event_parameter[0] == HCI_COMMAND_SUCCEEDED)&&
       (event_parameter[9] == 0x00))
    {
        UINT32 U32_data = VENDOR_READ(0x150);
        U32_data |= 0x10;
        VENDOR_WRITE(0x150, U32_data);
    }
#endif            

    
#ifdef _DAPE_NO_ROLE_SW_WHEN_ROLE_SW
    g_role_switch_status &= ~(BIT0 << ce_index);
#endif    
#ifdef _DAPE_TEST_GEN_FAKE_ESCO_DATA
    /* for SCO remote loopback generate data. */
    if (link_type != ACL_LINK)
    {
        g_handle = conn_handle;
        RT_BT_LOG(GREEN, DAPE_TEST_LOG293, 1,g_handle);
    }
#endif
#ifdef _FAKE_SECURE_CONN_
    BZ_AUTH_LINK_PARAMS* auth;
    auth = lmp_connection_entity[ce_index].auth;
    auth->secure_conn_enabled = 1;
    auth->len_prime = 8;
#endif
#ifndef _DAPE_TEST_FIX_NEW_PLATFORM_MUTE_ACL004_DISCONN
#ifdef _CCH_SC_ECDH_P256_
    BB_write_sc_cam_ini(ce_index);
#endif
#endif
}
/**
 * Generates a Connection Request Event that is used to indicate
 * that a new incoming connection is trying to be established.
 *
 * \param bd_addr          Remote Bluetooth device address.
 * \param class_of_device  'Class of Device' for the remote device,
 * \param link_type        SCO or ACL conenction.
 *
 * \return TRUE if success otherwise FALSE.
 *
 */
BOOLEAN hci_generate_connection_request_event ( UCHAR  *bd_addr,
        UINT32 class_of_device, UCHAR link_type)
{
    UCHAR event_parameter[10];
    UCHAR i;
    
    BT_FW_HCI_INF(GENERATING_CONNECTION_REQUEST_EVENT,0,0);

    for (i = 0 ; i < LMP_BD_ADDR_SIZE; i ++)
    {
        event_parameter[i] = bd_addr[i];
    }

    event_parameter[6] = (UCHAR)class_of_device;
    event_parameter[7] = (UCHAR)(class_of_device >> 8);
    event_parameter[8] = (UCHAR)(class_of_device >> 16);    
    event_parameter[9] = link_type ;

    return hci_generate_event(HCI_CONNECTION_REQUEST_EVENT,
            event_parameter, 10);
}

/**
 * Generates a Disconnection Complete Event that is used to
 * indicate the termination of a connection.
 *
 * \param status             Status.
 * \param connection_handle  Connection handle which was disconnected.
 * \param reason             Reason for the termination of the connection.
 *
 * \return None.
 *
 */
void hci_generate_disconnection_complete_event( UCHAR status,
        UINT16 connection_handle, UCHAR reason)
{
    UCHAR event_parameter[4];
    UINT16 ce_index;
#ifdef FIX_LE_HW_NO_BD_ADDR_CHECK
    BOOLEAN has_evt_generated = FALSE;
#endif

    BT_FW_HCI_INF(GENERATING_DISCONNECTION_COMPLETE_EVENT,0,0);
    
    /* If the host connection is being disconnected , we need to reset the
     * outstanding buffer count for which we have not received host num
     * complete because host has to flush these buffers as soon as a
     * disconnect happens.
     */
    if(LMP_GET_CE_INDEX_FROM_CONN_HANDLE(connection_handle, &ce_index) ==
            API_SUCCESS)
    {
        if(lmp_self_device_data.number_of_acl_conn)
        {
            lmp_self_device_data.number_of_acl_conn-- ;
            BT_FW_HCI_INF(NUMBER_OF_ACL_CONNECTIONS,1,
                          lmp_self_device_data.number_of_acl_conn);
        }
#ifdef _DAPE_SLV_SUPTO_IMPROVE
        if (g_slv_tpoll_disabled != 0xFF)
        {
            LMP_CONNECTION_ENTITY *ce_ptr;
            ce_ptr = &lmp_connection_entity[ce_index];
            if (ce_ptr->remote_dev_role == MASTER)
            {
                UINT8 am_addr;
                UINT8 pid;
                UINT8 lut_index;
                am_addr = ce_ptr->am_addr;
                pid = ce_ptr->phy_piconet_id;
                lut_index = lc_get_lut_index_from_phy_piconet_id(am_addr, pid);
                if (lut_index == g_slv_tpoll_disabled)
                {
                    UINT16 slv_tpoll_mask_val = 0xFFFF;
                    UINT16 slv_tpoll_mask_reg = 0xFFFF;
            
                    slv_tpoll_mask_reg = reg_SLAVE_TPOLL_MASK_REG[g_slv_tpoll_disabled - 8];
                    slv_tpoll_mask_val = BB_read_baseband_register(slv_tpoll_mask_reg);
                    BB_write_baseband_register(slv_tpoll_mask_reg, slv_tpoll_mask_val & (~BIT12));                                  
                    g_slv_tpoll_disabled = 0xFF;
                }
            }
        }
#endif    

#ifndef _DISABLE_HCI_HOST_FLOW_CONTROL_
        hci_reset_host_num_complete(connection_handle,ce_index);
#endif /* end of #ifndef _DISABLE_HCI_HOST_FLOW_CONTROL_ */

#ifdef _DAPE_NO_ROLE_SW_WHEN_ROLE_SW
        g_role_switch_status &= ~(BIT0 << ce_index);
#endif        
    }
#ifdef LE_MODE_EN
    else if (IS_BT40)
    {
        LL_CONN_HANDLE_UNIT *handle;
        handle = ll_fw_search_handle_unit_via_conn_handle(connection_handle);
        if (handle != NULL)
        {
#ifdef FIX_LE_HW_NO_BD_ADDR_CHECK
            has_evt_generated = handle->early_killed;
#endif

            ll_fw_reset_remote_conn_entry_param(handle->unit_id);
#ifdef _DAPE_CORRECT_LE_CONNECTION_CNT            
            if (ll_manager.conn_unit.connection_cnts > 0)
            {
                ll_manager.conn_unit.connection_cnts--;                                
            }            
#endif
            lmp_put_global_slot_offset(handle->unit_id + LL_HCI_MIN_CONN_HANDLE, 0);
#ifdef _DAPE_TEST_NEW_HW_BLOCK_ACL_FOR_LE
            if (g_enable_le_block_legacy)
            {
                ll_driver_block_legacy_slot_for_le(g_block_legacy_for_le_slot_num);
                g_enable_le_block_legacy = 0;
            }
#endif
#ifdef _PAUSE_SCO_FOR_LE_CONN
            bb_pause_sco(FALSE);
            bb_pause_esco(FALSE);
            lc_sco_pause_status &= ~BIT7;
#endif

#ifndef _DISABLE_HCI_HOST_FLOW_CONTROL_
            /* TODO: add hci_reset_host_num_complete relative function */
#endif /* end of #ifndef _DISABLE_HCI_HOST_FLOW_CONTROL_ */
        }
    }
#endif

#ifdef FIX_LE_HW_NO_BD_ADDR_CHECK
    if (has_evt_generated)
    {
        return;
    }
#endif
    event_parameter[0] = status;
    event_parameter[1] = LSB(connection_handle);
    event_parameter[2] = MSB(connection_handle);
    event_parameter[3] = reason;
    hci_generate_event(HCI_DISCONNECTION_COMPLETE_EVENT, event_parameter, 4);
}

/**
 * Generates a Mode Change event which is used by the Host
 * Controller to indicate to the Host that either a Hold, Park or Sniff
 * mode has been initiated or terminated.
 *
 * \param status             Status.
 * \param connection_handle  Connection handle which was disconnected.
 * \param current_mode       State of the connection is currently in.
 * \param interval           (Interval)Time specific to each state.
 *
 * \return None.
 *
 */
void hci_generate_mode_change_event(UCHAR status,
                                    UINT16 ce_index, UCHAR current_mode,
                                    UINT16 interval )
{
    UCHAR event_parameter[6];
    event_parameter[0] = status;
    event_parameter[1] = 
        LSB(lmp_connection_entity[ce_index].connection_type.connection_handle);
    event_parameter[2] = 
        MSB(lmp_connection_entity[ce_index].connection_type.connection_handle);
    event_parameter[3] = current_mode;
    event_parameter[4] = LSB(interval);
    event_parameter[5] = MSB(interval);

    hci_generate_event(HCI_MODE_CHANGE_EVENT, event_parameter, 6);

    return;
}

/**
 * Generates a remote name request complete event that
 * is used to indicate that a name request has been completed.
 *
 * \param status             Status.
 * \param ce_index           Connection Entity Index.
 *
 * \return None.
 *
 */
void hci_generate_remote_name_request_complete_event ( UCHAR status,
        UINT16 ce_index)
{
    UCHAR event_parameter[7 + LMP_MAX_NAME_LENGTH];
    BT_FW_HCI_INF(GENERATING_REMOTE_NAME_REQUEST_COMPLETE_EVENT,0,0);

    event_parameter[0] = status ;
    memcpy(& event_parameter[1],
           lmp_connection_entity[ce_index].bd_addr, LMP_BD_ADDR_SIZE );

    if (status == HCI_COMMAND_SUCCEEDED)
    {
        memcpy(& event_parameter[7],
            lmp_connection_entity[ce_index].device_name, LMP_MAX_NAME_LENGTH);
    }

    hci_generate_event(HCI_REMOTE_NAME_REQUEST_COMPLETE_EVENT, event_parameter,
            7 + LMP_MAX_NAME_LENGTH);
}

/**
 * Generates remote supported features request complete event that
 * is used to indicate that a read remote supported features has been
 * completed.
 *
 * \param status             Status.
 * \param connection_handle  Connection Handle.
 * \param remote_features    Pointer to the remote features.
 *
 * \return None.
 *
 */
void hci_generate_remote_supported_features_complete_event (
    UCHAR  status, UINT16 connection_handle,
    UCHAR *remote_features)
{
    UCHAR event_parameter[3 + LMP_FEATURES_SIZE];
    BT_FW_HCI_INF(GENERATING_REM_SUPPORTED_FEAT_COMPL_EVENT,0,0);

    event_parameter[0] = status ;
    event_parameter[1] = LSB(connection_handle);
    event_parameter[2] = MSB(connection_handle);

    memcpy(&event_parameter[3],
           remote_features, LMP_FEATURES_SIZE );;

    hci_generate_event(HCI_READ_REMOTE_SUPPORTED_FEATURES_COMPLETE_EVENT,
            event_parameter, (3 + LMP_FEATURES_SIZE));
}

/**
 * Generates a remote version information complete event.
 * The remote version infomation includes,
 * LMP version, Manufacturere name, LMP subversion.
 *
 * \param status             Status.
 * \param connection_handle  Connection Handle.
 * \param ce_index           Connection Entity Index.
 *
 * \return None.
 *
 */
void hci_generate_remote_version_information_complete_event(
    UCHAR  status, UINT16 connection_handle,
    UINT16 ce_index)
{
    UCHAR event_parameter[8];
    BT_FW_HCI_INF(GENERATING_REMOTE_VERSION_INFO_COMPLETE_EVENT,0,0);

    event_parameter[0] = status ;
    event_parameter[1] = LSB(connection_handle);
    event_parameter[2] = MSB(connection_handle);

#ifdef LE_MODE_EN
    if ((ce_index >= LMP_MAX_CE_DATABASE_ENTRIES) && IS_BT40)
    {
        ce_index -= LMP_MAX_CE_DATABASE_ENTRIES;

        event_parameter[3] = 
            ll_manager.conn_unit.handle[ce_index].remote_info.vers_nr;
        event_parameter[4] = 
            LSB(ll_manager.conn_unit.handle[ce_index].remote_info.comp_id);
        event_parameter[5] = 
            MSB(ll_manager.conn_unit.handle[ce_index].remote_info.comp_id);
        event_parameter[6] = 
            LSB(ll_manager.conn_unit.handle[ce_index].remote_info.subvers_nr);
        event_parameter[7] = 
            MSB(ll_manager.conn_unit.handle[ce_index].remote_info.subvers_nr);
    }
    else
#endif
    {
        event_parameter[3] = 
            lmp_connection_entity[ce_index].lmp_version;
        event_parameter[4] = 
            LSB(lmp_connection_entity[ce_index].rem_manuf_name);
        event_parameter[5] = 
            MSB(lmp_connection_entity[ce_index].rem_manuf_name);
        event_parameter[6] = 
            LSB(lmp_connection_entity[ce_index].lmp_subversion);
        event_parameter[7] = 
            MSB(lmp_connection_entity[ce_index].lmp_subversion);
    }

    hci_generate_event(HCI_READ_REMOTE_VERSION_INFORMATION_COMPLETE_EVENT,
            event_parameter, 8);
}

/**
 * Generates a Clock Offset information of the
 * Bluetooth device specified by the Connection_Handle event parameter.
 *
 * \param status             Status.
 * \param connection_handle  Connection Handle.
 * \param clock_offset       Clock Offset.
 *
 * \return None.
 *
 */
void hci_generate_clock_offset_complete_event(UCHAR  status,
        UINT16 connection_handle, UINT16 clock_offset)
{
    UCHAR event_parameter[5];
    BT_FW_HCI_INF(GENERATING_CLOCK_OFFSET_COMPLETE_EVENT,0,0);

    event_parameter[0] = status ;
    event_parameter[1] = LSB(connection_handle);
    event_parameter[2] = MSB(connection_handle);
    event_parameter[3] = LSB(clock_offset);
    event_parameter[4] = MSB(clock_offset);

    hci_generate_event(HCI_READ_CLOCK_OFFSET_COMPLETE_EVENT,
            event_parameter, 5);
}
/**
 * Generates a read lmp handle information of the
 * Bluetooth device specified by the Connection_Handle event parameter.
 *
 * \param status             Status.
 * \param connection_handle  Connection Handle.
 * \param clock_offset       Lmp Handle.
 *
 * \return None.
 *
 */
void hci_generate_read_lmp_handle_complete_event(UCHAR  status, UINT16 cmd_opcode,
        UINT16 connection_handle, UCHAR lmp_handle)
{
    UCHAR event_parameter[11];
    //RT_BT_LOG(GRAY, BT_FW_HCI_EVENTS_1563, 0, 0);

    /* Num HCI comamnd from self dev datbase */
    if(lmp_self_device_data.num_hci_command_packets != 0)
    {
        event_parameter[0] = lmp_self_device_data.num_hci_command_packets;
    }
    else
    {
        /*
           Controller will atleast 1 command buffer (reserved)
           and one more when the handle_hci_cmd function returns.
           We can safely report '1' even thou the variable says 0.
         */
        event_parameter[0] = 1;
    }
    event_parameter[1] = LSB(cmd_opcode);
    event_parameter[2] = MSB(cmd_opcode);
    event_parameter[3] = status ;
    event_parameter[4] = LSB(connection_handle);
    event_parameter[5] = MSB(connection_handle);
    event_parameter[6] = lmp_handle;
    event_parameter[7] = 0;
    event_parameter[8] = 0;
    event_parameter[9] = 0;
    event_parameter[10] = 0;

    hci_generate_event(HCI_COMMAND_COMPLETE_EVENT, event_parameter, 11);
}

/**
 * Generates connection packet type changed event which is
 * used to indicate that the process has completed of the Link Manager
 * changing which packet types can be used for the connection.
 *
 * \param status             Status.
 * \param connection_handle  Connection Handle.
 * \param packet_type        Packet type.
 *
 * \return None.
 *
 */
void hci_generate_connection_packet_type_changed_event(UCHAR status,
        UINT16 connection_handle, UINT16 packet_type)
{
    UCHAR event_parameter[5];
    BT_FW_HCI_INF(GENERATING_CONNECTION_PACKET_TYPE_CHANGED_EVENT,0,0);

    event_parameter[0] = status ;
    event_parameter[1] = LSB(connection_handle);
    event_parameter[2] = MSB(connection_handle);
    event_parameter[3] = LSB(packet_type);
    event_parameter[4] = MSB(packet_type);

    hci_generate_event(HCI_CONNECTION_PACKET_TYPE_CHANGED_EVENT,
            event_parameter, 5);
}

/**
 * Generates an max slots change event, which is
 * used to notify the Host about the LMP_Max_Slots parameter when
 * the value of this parameter changes. It will be sent each time
 * the maximum allowed length, in number of slots, for baseband
 * packets transmitted by the local device,
 *
 * \param connection_handle  ACL Connection Handle.
 * \param max_slots          Maximum slots for multislot packet.
 *
 * \return None.
 *
 */
void hci_generate_max_slots_change_event( UINT16 connection_handle,
        UCHAR max_slots)
{
    UCHAR event_parameter[3];
    BT_FW_HCI_INF(GENERATING_MAX_SLOTS_CHANGE_EVENT,0,0);

    event_parameter[0] = LSB(connection_handle);
    event_parameter[1] = MSB(connection_handle);
    event_parameter[2] =  max_slots;

    hci_generate_event(HCI_MAX_SLOTS_CHANGE_EVENT, event_parameter, 3);
}

#ifdef COMPILE_ROLE_SWITCH
/**
 * Generates an role change event which is used to
 * indicate that the current Bluetooth role related to the
 * particular connection has changed. This event only occurs when
 * both the remote and local Bluetooth devices have completed
 * their role change for the Bluetooth device associated with
 * the BD_ADDR event parameter.
 *
 * \param status             Status.
 * \param bd_addr            BD Address of the remoter device.
 * \param new_role           New Role - 0 => Master
 *
 * \return None.
 */
void hci_generate_role_change_event(UCHAR status, UCHAR *bd_addr,
                                    UCHAR new_role)
{
    UCHAR event_parameter[8];
    BT_FW_HCI_INF(GENERATING_ROLE_CHANGE_EVENT,0,0);

    event_parameter[0] = status ;
    memcpy(&event_parameter[1], bd_addr, LMP_BD_ADDR_SIZE);
    event_parameter[7] = new_role ;

    /**PATCH_PENDING_ENCRY_WHILE_ROLE_SWITCH*/
#ifdef _PENDING_ENCRY_WHILE_ROLE_SWITCH
    g_need_pend_encry = FALSE;
    
    if (g_encry_pending == TRUE)
    {
        LMP_PDU_PKT* pdu_pkt;
        pdu_pkt = (LMP_PDU_PKT *)g_signal_ptr_encry.param;
        pduq_queue_pdu(pdu_pkt, pdu_pkt ->piconet,  pdu_pkt->ce_index);
        lc_handle_and_check_sw_scheduler((UCHAR)((UINT32)(g_signal_ptr_encry.ext_param)));    
        g_encry_pending = FALSE;
    }
#endif
#ifdef _STOP_AFH_TIMER_WHILE_ROLE_SWITCH
    if ((g_stop_afh_timer) && (event_parameter[0]!= HCI_COMMAND_SUCCEEDED))
    {
        g_stop_afh_timer = FALSE;
    }
#endif
    hci_generate_event(HCI_ROLE_CHANGE_EVENT, event_parameter, 8);

    lmp_role_switch_data.ce_index = 0xFF;	  
}
#endif

/**
 * Generates an QoS setup complete event which is
 * used to indicate the completion of the process of the link
 * manager setting up QoS with the remote Bluetooth device
 * specified by the Connection_Handle event parameter.
 *
 * \param status             Status.
 * \param ce_index           Connection Entity Index.
 *
 * \return None.
 *
 */
void hci_generate_QoS_setup_complete_event(UCHAR status,
        UINT16 ce_index)
{
    UCHAR event_parameter[21];
    LMP_CONNECTION_ENTITY *ce_ptr;

    ce_ptr = &lmp_connection_entity[ce_index];

    event_parameter[0] = status ;
    event_parameter[1] = LSB(ce_ptr->connection_type.connection_handle);
    event_parameter[2] = MSB(ce_ptr->connection_type.connection_handle);
    event_parameter[3] = ce_ptr->QoS_flags ;
    event_parameter[4] = ce_ptr->service_type ;
    bt_fw_ultostr(& event_parameter[5], ce_ptr->token_rate, 4);
    bt_fw_ultostr(& event_parameter[9], ce_ptr->peak_bandwidth, 4);
    bt_fw_ultostr(& event_parameter[13], ce_ptr->latency, 4);
    bt_fw_ultostr(& event_parameter[17], ce_ptr->delay_variation, 4);

    hci_generate_event(HCI_QOS_SETUP_COMPLETE_EVENT, event_parameter, 21);
    return;
}


/**
* Generates an Flow specification complete event which is
* used to indicate the completion of the process of the link
* manager setting up Flow specification with the remote device.
*
* \param status             Status.
* \param flow_direction The direction of flow, for which Qos is requested.
* \param ce_index           Connection Entity Index.
*
* \return None.
*
*/
void hci_generate_flow_spec_complete_event(UCHAR status, UCHAR flow_direction,
        UINT16 ce_index)
{
    UCHAR event_parameter[22];
    LMP_CONNECTION_ENTITY *ce_ptr;
  
    ce_ptr = &lmp_connection_entity[ce_index];

    event_parameter[0] = status ;
    event_parameter[1] = LSB(ce_ptr->connection_type.connection_handle);
    event_parameter[2] = MSB(ce_ptr->connection_type.connection_handle);
    event_parameter[3] = ce_ptr->QoS_flags ; 
    event_parameter[4] = flow_direction;
    event_parameter[5] = ce_ptr->service_type ; 
    bt_fw_ultostr(& event_parameter[6], ce_ptr->token_rate, 4);
    bt_fw_ultostr(& event_parameter[10], ce_ptr->token_bucket_rate, 4);
    bt_fw_ultostr(& event_parameter[14], ce_ptr->peak_bandwidth, 4);
    bt_fw_ultostr(& event_parameter[18], ce_ptr->latency, 4);

    hci_generate_event(HCI_FLOW_SPECIFICATION_COMPLETE_EVENT,
            event_parameter, 22);
}

#ifdef TEST_MODE
/**
 * Generates loopback command Event.
 *
 * \param hci_cmd_pkt_ptr      pointer to the HCI command packet that has to
 *                             be looped back.
 *
 * \return None.
 *
 */
void hci_generate_loopback_command_event(HCI_CMD_PKT* hci_cmd_pkt_ptr)
{
    UCHAR event_parameter[HCI_MAX_EVENT_PARAM_LEN];
    UCHAR param_length;

    BT_FW_HCI_INF(GENERATING_LOOPBACK_COMMAND_EVENT,0,0);

    event_parameter[0] = LSB(hci_cmd_pkt_ptr->cmd_opcode);
    event_parameter[1] = MSB(hci_cmd_pkt_ptr->cmd_opcode);    
    event_parameter[2] = hci_cmd_pkt_ptr->param_total_length;
    memcpy(&event_parameter[3], hci_cmd_pkt_ptr->cmd_parameter,
           hci_cmd_pkt_ptr->param_total_length);
    param_length = (UCHAR)(3 + hci_cmd_pkt_ptr->param_total_length);

    hci_generate_event(HCI_LOOPBACK_COMMAND_EVENT, event_parameter,
            param_length);
}

/**
 * Generates a 'Test' Connection Complete Event to the Host.
 * This function is called during the local loopback Test Mode.
 *
 * \param status             Status.
 * \param conn_handle        Connection Handle.
 * \param bd_addr            Remote bluetooth device address.
 * \param link_type          Link Type.
 * \param encryption_mode    Encryption mode
 *
 * \return None.
 *
 */
void hci_generate_test_mode_connection_complete_event(UCHAR status,
        UINT16 conn_handle, UCHAR* bd_addr, UCHAR link_type,
        UCHAR encryption_mode )
{
    UCHAR event_parameter[11];
    BT_FW_HCI_INF(GENERATING_CONNECTION_COMPLETE_EVENT,0,0);

    event_parameter[0] = status ;
    event_parameter[1] = LSB(conn_handle);
    event_parameter[2] = MSB(conn_handle);
    memcpy(& event_parameter[3],bd_addr, LMP_BD_ADDR_SIZE);
    event_parameter[9] = link_type;
    event_parameter[10] = encryption_mode;

    hci_generate_event(HCI_CONNECTION_COMPLETE_EVENT, event_parameter, 11);
}


#endif /* TEST_MODE */

/**
 * Generates a flush occured event,
 *
 * \param conn_handle        Connection Handle.
 *
 * \return None.
 *
 */
void hci_generate_flush_occured_event(UINT16 conn_handle)
{
    UCHAR event_parameter[HCI_FLUSH_OCCURRED_EVENT_LEN];
    event_parameter[0] = (UCHAR)conn_handle;
    event_parameter[1] = (UCHAR)(conn_handle >> 8);

    hci_generate_event(HCI_FLUSH_OCCURRED_EVENT, event_parameter,
            HCI_FLUSH_OCCURRED_EVENT_LEN);

    return;
}

/**
 * Generates a h/w error occured event.
 *
 * \param hardware_code      Hardware Error Code.
 *
 * \return None.
 *
 */
void hci_generate_hw_error_event(UCHAR hardware_code)
{
    UCHAR event_parameter[HCI_HARDWARE_ERROR_EVENT_LEN];
    event_parameter[0] = hardware_code;
    hci_generate_event(HCI_HARDWARE_ERROR_EVENT, event_parameter,
            HCI_HARDWARE_ERROR_EVENT_LEN);
}

/**
 * Gets the rssi value from the radio
 *
 * \param ce_index           Connection Entity Index.
 *
 * \return Rssi value.
 *
 */
INT8 get_rssi_value(UINT16 ce_index)
{
    INT8 rssi_dbm;

    /* If RSSI read is not supported, 0 will be returned,
     * that means that the power is within the golden range.
     */
    rssi_dbm = (INT8) otp_str_data.bw_rf_max_rssi_dbm;

#ifdef COMPILE_RSSI_REPORTING
    rssi_dbm = lc_calculate_log_from_rssi((UINT16)
                                (lmp_connection_entity[ce_index].rssi) );
    LMP_LOG_INFO(LOG_LEVEL_LOW, HCI_READ_RSSI_COMMAND, 1, rssi_dbm);
#endif

    if(rssi_dbm > otp_str_data.bw_rf_max_rssi_dbm)
    {
        rssi_dbm = (CHAR) (rssi_dbm - otp_str_data.bw_rf_max_rssi_dbm);
    }
    else if(rssi_dbm < otp_str_data.bw_rf_min_rssi_dbm)
    {
        /* Indicates below the Golden Receive power range */
        rssi_dbm = (CHAR) (rssi_dbm - otp_str_data.bw_rf_min_rssi_dbm);
    }
    else
    {
        rssi_dbm = 0;
    }

    return rssi_dbm;
}

