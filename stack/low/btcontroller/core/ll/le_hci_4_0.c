enum { __FILE_NUM__= 203 };

#include "le_hci_4_0.h"
#include "le_ll.h"
#include "le_ll_driver.h"
#include "bt_fw_hci_internal.h"
#include "mem.h"
#include "lc_internal.h"
#include "lmp_internal.h"
#ifdef LE_HW_TEST
#include "le_ll_test.h"
#endif
#ifdef CONFIG_TV_POWERON
#include "tv.h"
#endif
#include "lmp_vendor_defines.h"

#ifdef _DAPE_LOW_DUTY_ADV
UINT32 bt_le_multistate_bm_ldw = 0xFFFFFFFF;
#else
UINT32 bt_le_multistate_bm_ldw = 0x1FFFFFFF;
#endif
UINT32 bt_le_multistate_bm_hdw = 0x00000000; /* some bits will be set after
                                                we support le scatternet */

UINT8 g_le_check_adv_pdu_intv_param = FALSE;
UINT8 g_le_scanner_used_channel_map = 0x07;   /* enable all advertising channels */
UINT8 g_le_initiator_used_channel_map = 0x07; /* enable all advertising channels */


#ifdef _DAPE_TEST_SLOT_OFFSET_FOR_LE
extern UCHAR  g_le_conn_interval;
extern UINT16 g_le_conn_interval_changed;
UCHAR g_le_conn_interval_set = 0;
#endif

extern UINT8 g_le_use_interval_slot_pair;
/* The LUT translation from hci adv pdu parameter to LL adv pdu type */
const UINT8 le_hci2ll_adv_pdu_type[LL_HCI_ADV_TYPE_ADV_MAX+1] =
{
    LL_ADV_PDU_TYPE_ADV_IND,
    LL_ADV_PDU_TYPE_ADV_DIRECT_IND,
    LL_ADV_PDU_TYPE_ADV_DISCOVER_IND,
    LL_ADV_PDU_TYPE_ADV_NONCONN_IND,
#ifdef _DAPE_LOW_DUTY_ADV
    LL_ADV_PDU_TYPE_ADV_DIRECT_IND
#endif
};

/* The LUT translation from LL adv pdu type to event type of adv report */
const UINT8 le_ll2hci_adv_rpt_event_type[8] =
{
    LL_HCI_ADV_REPORT_EVENT_TYPE_ADV_IND,
    LL_HCI_ADV_REPORT_EVENT_TYPE_ADV_DIRECT_IND,
    LL_HCI_ADV_REPORT_EVENT_TYPE_ADV_NONCONN_IND,
    LL_HCI_ADV_REPORT_EVENT_TYPE_INVALID,
    LL_HCI_ADV_REPORT_EVENT_TYPE_SCAN_RSP,
    LL_HCI_ADV_REPORT_EVENT_TYPE_INVALID,
    LL_HCI_ADV_REPORT_EVENT_TYPE_ADV_DISCOVER_IND,
    LL_HCI_ADV_REPORT_EVENT_TYPE_INVALID
};

#ifdef _ROM_CODE_PATCHED_
PF_ROM_CODE_PATCH_VOID rcp_hci_le_adjust_conn_req_func = NULL;
#endif

/**************************************************************************
 * Function     : hci_handle_4_0_hc_bb_command
 *
 * Description  : Handles all the host controller and baseband for the
 *                Bluetooth Version 4_0. This function is called from the
 *                hci_task.c if the opcode does not match any of the
 *                1.1/1.2/2.1/3.0 Version opcodes.
 *
 * Parameters   : hci_cmd_ptr: Pointer to HCI_CMD_PKT
 *
 * Returns      : HCI_COMMAND_SUCCEEDED or HCI error code.
 *
 *************************************************************************/
UCHAR hci_handle_4_0_hc_bb_command(HCI_CMD_PKT *hci_cmd_ptr)
{
    UCHAR ret_error = HCI_COMMAND_SUCCEEDED;
    UINT8 le_support_host_en;
    UINT8 simul_le_host_en;

    switch(hci_cmd_ptr->cmd_opcode)
    {
        case HCI_READ_LE_HOST_SUPPORTED_COMMAND_OPCODE:
            break;

        case HCI_WRITE_LE_HOST_SUPPORTED_COMMAND_OPCODE:
            if(hci_cmd_ptr->param_total_length != 2)
            {
                return INVALID_HCI_COMMAND_PARAMETERS_ERROR;
            }

            /* LE_Supported_Host */
            le_support_host_en = hci_cmd_ptr->cmd_parameter[0];
            if (le_support_host_en > 0x01)
            {
                return INVALID_HCI_COMMAND_PARAMETERS_ERROR;
            }

            /* Simultaneous_LE_Host */
            simul_le_host_en = hci_cmd_ptr->cmd_parameter[1];
            if (simul_le_host_en > 0x01)
            {
                return INVALID_HCI_COMMAND_PARAMETERS_ERROR;
            }

            /* update feature mask of page 1 */
            if (le_support_host_en)
            {
                lmp_feature_data.features[1][0] |= BIT1;
            }
            else
            {
                lmp_feature_data.features[1][0] &= ~BIT1;
            }

            if (simul_le_host_en)
            {
                lmp_feature_data.features[1][0] |= BIT2;
            }
            else
            {
                lmp_feature_data.features[1][0] &= ~BIT2;
            }
            break;

        default:
            ret_error = UNKNOWN_HCI_COMMAND_ERROR;
            break;
    }

    return ret_error;
}

/**
 * Prepares the command complete event for Version 4_0 HCI commands.
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
UCHAR hci_generate_4_0_command_complete_event( UINT16 cmd_opcode,
                                               UCHAR *pkt_param,
                                               UINT16 ext_param )
{
    UCHAR param_length = 4;
    /* Param length is 4 because event_opcode(1),cmd_opcode(2) and status(1)
     * are populated by the calling function
     */

    switch(cmd_opcode)
    {
        case HCI_READ_LE_HOST_SUPPORTED_COMMAND_OPCODE :
            pkt_param[param_length] =
                     (lmp_feature_data.features[1][0] & BIT1) ? 1 : 0;
            pkt_param[param_length + 1] =
                     (lmp_feature_data.features[1][0] & BIT2) ? 1 : 0;
            param_length += 2;
            break;

        case HCI_WRITE_LE_HOST_SUPPORTED_COMMAND_OPCODE :
            /* no more return parameters */
            break;

#ifdef _BT4_2_DATA_EXTENSION_FEATURE_SUPPORT_
        case HCI_LE_SET_DATA_LENGTH_OPCODE:
            htole16(ext_param, &pkt_param[param_length]);
            param_length += 2;
            break;

        case HCI_LE_READ_DEFAULT_DATA_LENGTH_OPCODE:
            htole16(ll_manager.conn_unit.init_max_tx_size, &pkt_param[param_length]);
            param_length += 2;
            htole16(ll_manager.conn_unit.init_max_tx_time, &pkt_param[param_length]);
            param_length += 2;
            break;

        case HCI_LE_WRITE_DEFAULT_DATA_LENGTH_OPCODE:
            break;

        case HCI_LE_READ_MAXIMUM_DATA_LENGTH_OPCODE:
            {
                UINT16 max_octets, max_time;
                if (ll_manager.conn_unit.support_dle)
                {
                    max_octets = LE_CONN_MAX_TX_SIZE_MAX;
                    max_time = LE_CONN_MAX_TX_TIME_MAX;
                }
                else
                {
                    max_octets = LE_CONN_MAX_TX_SIZE_MIN;
                    max_time = LE_CONN_MAX_TX_TIME_MIN;
                }
                /* supportedMaxTxOctets */
                htole16(max_octets, &pkt_param[param_length]);
                param_length += 2;
                /* supportedMaxTxTime */
                htole16(max_time, &pkt_param[param_length]);
                param_length += 2;
                /* supportedMaxRxOctets */
                htole16(max_octets, &pkt_param[param_length]);
                param_length += 2;
                /* supportedMaxRxTime */
                htole16(max_time, &pkt_param[param_length]);
                param_length += 2;
            }
            break;
#endif
        default :
            /* return invalid value for unknown hci command */
            return 0xFF;
    }/* end switch(cmd_opcode) */

    return (UCHAR) (param_length - 4);
}


/**************************************************************************
 * Function     : hci_4_0_handle_remote_version_information_command
 *
 * Description  : This function is used to handle HCI Remote Version
 *                Information Command for BT 4.0
 *
 * Parameters   : conn_handle: the connection handle
 *                pp_handle: the handle of LL connection unit
 *
 * Returns      : Error Code
 *
 *************************************************************************/
UCHAR hci_4_0_handle_remote_version_information_command(UINT16 conn_handle,
        LL_CONN_HANDLE_UNIT **pp_handle)
{
    LL_CONN_HANDLE_UNIT *p_handle;

    p_handle = ll_fw_search_handle_unit_via_conn_handle(conn_handle);
    *pp_handle = p_handle;

    if (p_handle == NULL)
    {
        return NO_CONNECTION_ERROR;
    }

    return HCI_COMMAND_SUCCEEDED ;
}

/**************************************************************************
 * Function     : hci_generate_le_command_complete_event_head
 *
 * Description  : This function is used to generate the head format of
 *                LE command complete event
 *
 * Parameters   : hci_cmd_opcode: the OpCode of HCI Command
 *
 * Returns      : None
 *
 *************************************************************************/
void hci_generate_le_command_complete_event_head(UINT16 hci_cmd_opcode,
        UCHAR *event_parameter)
{
    /* Num HCI comamnd from self dev database */
    if(hci_cmd_opcode == 0x00)
    {
        event_parameter[0] = 0x00 ;
    }
    else
    {
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
    }

    /* Command Opcode */
    event_parameter[1] = hci_cmd_opcode & 0xFF;
    event_parameter[2] = hci_cmd_opcode >> 8;
}

/**************************************************************************
 * Function     : hci_handle_le_invalid_command
 *
 * Description  : This function is used to handle HCI LE Invalid Command.
 *                (the opcode is invalid)
 *
 * Parameters   : hci_cmd_ptr Pointer to the command packet.
 *
 * Returns      : HCI_COMMAND_SUCCEEDED or HCI error code
 *
 *************************************************************************/
UINT8 hci_handle_le_invalid_command(HCI_CMD_PKT *hci_cmd_ptr)
{
    UCHAR event_parameter[4];
    hci_generate_le_command_complete_event_head(hci_cmd_ptr->cmd_opcode,
            event_parameter);
    event_parameter[3] = UNKNOWN_HCI_COMMAND_ERROR;
    LL_HCI_GEN_COMMAND_COMPLETE_EVENT(event_parameter, 1);
    return UNKNOWN_HCI_COMMAND_ERROR;
}

/**************************************************************************
 * Function     : hci_handle_le_set_event_mask
 *
 * Description  : This function is used to handle HCI LE Set Event Mask Command.
 *                FW only can generate masked LE events and send it to host.
 *                Please refer 7.8.1 in HCI part of BT 4.0 spec.
 *
 * Parameters   : hci_cmd_ptr Pointer to the command packet.
 *
 * Returns      : HCI_COMMAND_SUCCEEDED or HCI error code
 *
 *************************************************************************/
UINT8 hci_handle_le_set_event_mask(HCI_CMD_PKT *hci_cmd_ptr)
{
    UCHAR event_parameter[4];
    UINT8 *pCCE_returnPara = &event_parameter[3];

    /* In BT 4.0 spec, only bit [4:0] are valid in LE_Event_Mask field */
    memcpy(ll_manager.le_event_mask, hci_cmd_ptr->cmd_parameter, 8);

    /* generate command complete event */
    hci_generate_le_command_complete_event_head(hci_cmd_ptr->cmd_opcode,
            event_parameter);
    pCCE_returnPara[0] = HCI_COMMAND_SUCCEEDED;
    LL_HCI_GEN_COMMAND_COMPLETE_EVENT(event_parameter, 1);
    return HCI_COMMAND_SUCCEEDED;
}

/**************************************************************************
 * Function     : hci_handle_le_read_buffer_size
 *
 * Description  : This function is used to handle HCI LE Read Buffer Size
 *                Command. Please refer 7.8.2 in HCI part of BT 4.0 spec.
 *
 * Parameters   : hci_cmd_ptr Pointer to the command packet.
 *
 * Returns      : HCI_COMMAND_SUCCEEDED or HCI error code
 *
 *************************************************************************/
UINT8 hci_handle_le_read_buffer_size(HCI_CMD_PKT *hci_cmd_ptr)
{
    UCHAR event_parameter[7];
    UINT8 *pCCE_returnPara = &event_parameter[3];

    /* generate command complete event */
    hci_generate_le_command_complete_event_head(hci_cmd_ptr->cmd_opcode,
            event_parameter);

    /* status field - byte 0 */
    pCCE_returnPara[0] = HCI_COMMAND_SUCCEEDED;

    /* HC_LE_ACL_Data_Packet_Length field - byte 1 ~ 2 */
    pCCE_returnPara[1] = LL_HCI_H2C_MAX_ACL_PKT_SIZE & 0xFF;
    pCCE_returnPara[2] = LL_HCI_H2C_MAX_ACL_PKT_SIZE >> 8;

    /* HC_Total_Num_LE_ACL_Data_Packets field - byte 3 */
    pCCE_returnPara[3] = LL_HCI_H2C_MAX_ACL_PKT_CNT;

    LL_HCI_GEN_COMMAND_COMPLETE_EVENT(event_parameter, 4);

    return HCI_COMMAND_SUCCEEDED;
}

/**************************************************************************
 * Function     : hci_handle_le_read_local_supported_features
 *
 * Description  : This function is used to handle HCI LE Read Local Supported
 *                Features. Please refer 7.8.3 in HCI part of BT 4.0 spec.
 *
 * Parameters   : hci_cmd_ptr Pointer to the command packet.
 *
 * Returns      : HCI_COMMAND_SUCCEEDED or HCI error code
 *
 *************************************************************************/
UINT8 hci_handle_le_read_local_supported_features(HCI_CMD_PKT *hci_cmd_ptr)
{
    UCHAR event_parameter[12];
    UINT8 *pCCE_returnPara = &event_parameter[3];

    /* generate command complete event */
    hci_generate_le_command_complete_event_head(hci_cmd_ptr->cmd_opcode,
            event_parameter);

    /* status field - byte 0 */
    pCCE_returnPara[0] = HCI_COMMAND_SUCCEEDED;
#ifdef _DAPE_UNIFY_VERSION_FEATURE_UINT
    memcpy(&pCCE_returnPara[1], &ll_manager.ll_feature_support[0], 8);
#else
    /* LE_Features field - byte 1 ~ 8*/
    pCCE_returnPara[1] = (
            (ll_manager.conn_unit.support_enc)
            | (ll_manager.conn_unit.support_conn_para_req << 1)
            | (ll_manager.conn_unit.support_ext_reject_ind << 2)
            | (ll_manager.conn_unit.support_slave_init_feat_ext << 3)
            | (ll_manager.conn_unit.support_le_ping << 4)
#ifdef _BT4_2_DATA_EXTENSION_FEATURE_SUPPORT_
            | (ll_manager.conn_unit.support_dle << 5)
#endif
#ifdef _BT4_2_PRIVACY_FEATURE_SUPPORT_
            | (ll_manager.support_ll_privacy << 6)
#endif
            );

    memset(&pCCE_returnPara[2], 0, 7);    /* bit 1 ~ 63 (RFU) are 0 */
#endif
    LL_HCI_GEN_COMMAND_COMPLETE_EVENT(event_parameter, 9);

    return HCI_COMMAND_SUCCEEDED;
}

/**************************************************************************
 * Function     : hci_handle_le_set_random_address
 *
 * Description  : This function is used to handle HCI LE Set Random Address.
 *                Please refer 7.8.4 in HCI part of BT 4.0 spec.
 *
 * Parameters   : hci_cmd_ptr Pointer to the command packet.
 *
 * Returns      : HCI_COMMAND_SUCCEEDED or HCI error code
 *
 *************************************************************************/
UINT8 hci_handle_le_set_random_address(HCI_CMD_PKT *hci_cmd_ptr)
{
    UCHAR event_parameter[4];
    UINT8 *pCCE_returnPara = &event_parameter[3];

    /* copy random address (byte 0 ~ 5) from Host to local */
    memcpy(ll_manager.local_random_address, hci_cmd_ptr->cmd_parameter, 6);

    // dape added
    ll_driver_set_random_address();

    /* generate command complete event */
    hci_generate_le_command_complete_event_head(hci_cmd_ptr->cmd_opcode,
            event_parameter);
    pCCE_returnPara[0] = HCI_COMMAND_SUCCEEDED;

    LL_HCI_GEN_COMMAND_COMPLETE_EVENT(event_parameter, 1);

    return HCI_COMMAND_SUCCEEDED;
}

/**************************************************************************
 * Function     : hci_handle_le_set_advertising_parameters
 *
 * Description  : This function is used to handle HCI LE Set Advertising
 *                Parameters. Please refer 7.8.5 in HCI part of BT 4.0 spec.
 *
 * Parameters   : hci_cmd_ptr Pointer to the command packet.
 *
 * Returns      : HCI_COMMAND_SUCCEEDED or HCI error code
 *
 *************************************************************************/
UINT8 hci_handle_le_set_advertising_parameters(HCI_CMD_PKT *hci_cmd_ptr)
{
    UINT16 adv_interval_min;
    UINT16 adv_interval_max;
    UINT8 adv_type;
    UINT8 local_addr_type;
    UINT8 direct_addr_type;
    UINT8 adv_filt_policy;
    UINT8 adv_ch_map;
    UINT8 *pComPara = hci_cmd_ptr->cmd_parameter;
    UINT8 errcode = INVALID_HCI_COMMAND_PARAMETERS_ERROR;

    do
    {
        if (ll_manager.adv_unit.enable)
        {
            /* the advertising is enabled. Command Disallowed error
               code shall be used */
            errcode = COMMAND_DISALLOWED_ERROR;
            break;
        }

        /*----------------------------------------------------*/
        /*         Parser the Command Parameters              */
        /*----------------------------------------------------*/
        /* Advertising_Interval_Min - byte 0~1 */
        adv_interval_min = pComPara[0] | (pComPara[1] << 8);

        /* Advertising_Interval_Max - byte 2~3 */
        adv_interval_max = pComPara[2] | (pComPara[3] << 8);

        /* Advertising_Type - byte 4 */
        adv_type = pComPara[4];

        /* Own_Address_Type - byte 5 */
        local_addr_type = pComPara[5];

        /* Advertising_Channel_Map - byte 13 */
        adv_ch_map = pComPara[13] & 0x7;

        if ((local_addr_type > LL_ADDR_TYPE_MAX) || (adv_ch_map == 0))
        {
            break;
        }

        /* to check current state is valid or not */
        if (!ll_fw_check_multiple_states_valid(LL_STATE_ADVERTISING, adv_type))
        {
            errcode = COMMAND_DISALLOWED_ERROR;
            break;
        }

#ifdef _DAPE_LOW_DUTY_ADV
        if ((adv_type == LL_HCI_ADV_TYPE_ADV_DIRECT_IND) ||
            (adv_type == LL_HCI_ADV_TYPE_ADV_LOW_DUTY_DIRECT_IND) || local_addr_type > LL_ADDR_TYPE_RANDOM)
#else
        if (adv_type == LL_HCI_ADV_TYPE_ADV_DIRECT_IND || local_addr_type > LL_ADDR_TYPE_RANDOM)
#endif
        {
            /* directed advertising */

            /* Direct Address Type - byte 6 */
            direct_addr_type = pComPara[6];
            if (direct_addr_type > LL_ADDR_TYPE_MAX)
            {
                break;
            }

            /* Direct Address - byte 7 ~ 12 */
            /* save directed device address for directed advertising PDU */
            memcpy(ll_manager.adv_unit.direct_addr, &pComPara[7], 6);

            /* save directed device address type */
            ll_manager.adv_unit.direct_addr_type = direct_addr_type;
        }

#ifdef _DAPE_LOW_DUTY_ADV
        if (adv_type != LL_HCI_ADV_TYPE_ADV_DIRECT_IND)
#else
        else
#endif
        {
            /* non-directed advertising */
            if ((adv_type > LL_HCI_ADV_TYPE_ADV_MAX) ||
                (adv_interval_min < LL_ADV_INTERVAL_MIN) ||
                (adv_interval_min > LL_ADV_INTERVAL_MAX) ||
                (adv_interval_max < LL_ADV_INTERVAL_MIN) ||
                (adv_interval_max > LL_ADV_INTERVAL_MAX) ||
                (adv_interval_min > adv_interval_max))
            {
                break;
            }

            if ((adv_type == LL_HCI_ADV_TYPE_ADV_DISCOVER_IND) ||
                (adv_type == LL_HCI_ADV_TYPE_ADV_NONCONN_IND))
            {
                if (g_le_check_adv_pdu_intv_param &&
                    (adv_interval_min < LL_ADV_INTERVAL_NONCONN_MIN))
                {
                    break;
                }
            }

            /* Advertising Filter Policy - byte 14 */
            adv_filt_policy = pComPara[14];
            if (adv_filt_policy > LL_HCI_ADV_FILT_POLICY_MAX)
            {
                break;
            }

            /* save advertising minimum and maximum interval */
            ll_manager.adv_unit.interval_min = adv_interval_min;
            ll_manager.adv_unit.interval_max = adv_interval_max;

            /* save advertising policy */
            ll_manager.adv_unit.filter_scan_req = adv_filt_policy & 0x01;
            ll_manager.adv_unit.filter_conn_req = adv_filt_policy >> 1;
        }

        /* Advertising Channel Map - byte 13 */
        ll_manager.adv_unit.ch_map = pComPara[13];

        /* save local address type */
        ll_manager.adv_unit.local_addr_type = local_addr_type;

        /* save adv PDU type */
        ll_manager.adv_unit.hci_pdu_type = adv_type;
        ll_manager.adv_unit.pdu_type = le_hci2ll_adv_pdu_type[adv_type];

        errcode = HCI_COMMAND_SUCCEEDED;
    }
    while (0);

    LL_LOG_TRACE(WHITE, LE_MSG_LL_ADV_PARAS, 10,errcode,
     ll_manager.adv_unit.hci_pdu_type,
     ll_manager.adv_unit.pdu_type, ll_manager.adv_unit.local_addr_type,
     ll_manager.adv_unit.direct_addr_type, ll_manager.adv_unit.interval_min,
     ll_manager.adv_unit.interval_max, ll_manager.adv_unit.ch_map,
     ll_manager.adv_unit.filter_scan_req, ll_manager.adv_unit.filter_conn_req);

    /* generate command complete event */
    UCHAR event_parameter[4];
    UINT8 *pCCE_returnPara = &event_parameter[3];
    hci_generate_le_command_complete_event_head(hci_cmd_ptr->cmd_opcode,
            event_parameter);
    pCCE_returnPara[0] = errcode;
    LL_HCI_GEN_COMMAND_COMPLETE_EVENT(event_parameter, 1);

    if (errcode == HCI_COMMAND_SUCCEEDED)
    {
        lmp_update_lps_para();
    }

    return errcode;
}

/**************************************************************************
 * Function     : hci_handle_le_read_advertising_channel_tx_power
 *
 * Description  : This function is used to handle HCI Read Advertising Channel
 *                Tx Power Command. Please refer 7.8.6 in HCI part of BT 4.0
 *                spec.
 *
 * Parameters   : hci_cmd_ptr Pointer to the command packet.
 *
 * Returns      : HCI_COMMAND_SUCCEEDED or HCI error code
 *
 *************************************************************************/
UINT8 hci_handle_le_read_advertising_channel_tx_power(HCI_CMD_PKT *hci_cmd_ptr)
{
    UCHAR event_parameter[5];
    UINT8 *pCCE_returnPara = &event_parameter[3];
    UINT8 index = ll_manager.adv_tx_power_idx;
    INT8 tx_power_dbm = 0;

    if (index < le_tx_power_max_index)
    {
        tx_power_dbm = -3 * (le_tx_power_max_index - index);
    }

    hci_generate_le_command_complete_event_head(hci_cmd_ptr->cmd_opcode,
            event_parameter);

    /* status field - byte 0 */
    pCCE_returnPara[0] = HCI_COMMAND_SUCCEEDED;

    /* transmit power level - byte 1 */
    pCCE_returnPara[1] = (UINT8)tx_power_dbm;

    LL_HCI_GEN_COMMAND_COMPLETE_EVENT(event_parameter, 2);

    return HCI_COMMAND_SUCCEEDED;
}

/**************************************************************************
 * Function     : hci_handle_le_set_advertising_data
 *
 * Description  : This function is used to handle HCI LE Set Advertising Data.
 *                Please refer 7.8.7 in HCI part of BT 4.0 spec.
 *
 * Parameters   : hci_cmd_ptr Pointer to the command packet.
 *
 * Returns      : HCI_COMMAND_SUCCEEDED or HCI error code
 *
 *************************************************************************/
UINT8 hci_handle_le_set_advertising_data(HCI_CMD_PKT *hci_cmd_ptr)
{
    UCHAR event_parameter[4];
    UINT8 *pComPara = hci_cmd_ptr->cmd_parameter;
    UINT8 adv_data_len = pComPara[0];
    UINT8 errcode = HCI_COMMAND_SUCCEEDED;
    UINT8 *pCCE_returnPara = &event_parameter[3];

    if (adv_data_len > LL_ADV_DATA_LEN_MAX)
    {
        errcode = INVALID_HCI_COMMAND_PARAMETERS_ERROR;
    }
    else
    {
        /* update advertising data length and data content */
        ll_manager.adv_unit.adv_len = adv_data_len;
        if (adv_data_len > 0)
        {
            memcpy(ll_manager.adv_unit.adv_buf, &pComPara[1], adv_data_len);
        }
    }

    /* generate command complete event */
    hci_generate_le_command_complete_event_head(hci_cmd_ptr->cmd_opcode,
            event_parameter);
    pCCE_returnPara[0] = errcode;
    LL_HCI_GEN_COMMAND_COMPLETE_EVENT(event_parameter, 1);

    return errcode;
}

/**************************************************************************
 * Function     : hci_handle_le_set_scan_response_data
 *
 * Description  : This function is used to handle HCI LE Set Scan Response Data.
 *                Please refer 7.8.8 in HCI part of BT 4.0 spec.
 *
 * Parameters   : hci_cmd_ptr Pointer to the command packet.
 *
 * Returns      : HCI_COMMAND_SUCCEEDED or HCI error code
 *
 *************************************************************************/
UINT8 hci_handle_le_set_scan_response_data(HCI_CMD_PKT *hci_cmd_ptr)
{
    UCHAR event_parameter[4];
    UINT8 *pComPara = hci_cmd_ptr->cmd_parameter;
    LL_ADVERTISER_UNIT *pAdv = &ll_manager.adv_unit;
    UINT8 scan_rsp_data_len = pComPara[0];
    UINT8 errcode = HCI_COMMAND_SUCCEEDED;
    UINT8 *pCCE_returnPara = &event_parameter[3];

    if (scan_rsp_data_len > LL_SCAN_RESPONSE_DATA_LEN_MAX)
    {
        errcode = INVALID_HCI_COMMAND_PARAMETERS_ERROR;
    }
    else
    {
        /* update scan response data length and data content */
        pAdv->scan_rsp_len = scan_rsp_data_len;
        if (scan_rsp_data_len > 0)
        {
            memcpy(pAdv->scan_rsp_buf, &pComPara[1], scan_rsp_data_len);
        }
    }

    /* generate command complete event */
    hci_generate_le_command_complete_event_head(hci_cmd_ptr->cmd_opcode,
            event_parameter);
    pCCE_returnPara[0] = errcode;
    LL_HCI_GEN_COMMAND_COMPLETE_EVENT(event_parameter, 1);

    return errcode;
}

/**************************************************************************
 * Function     : hci_handle_le_set_advertising_enable
 *
 * Description  : This function is used to handle HCI LE Set Advertising Enable.
 *                Please refer 7.8.9 in HCI part of BT 4.0 spec.
 *
 * Parameters   : hci_cmd_ptr Pointer to the command packet.
 *
 * Returns      : HCI_COMMAND_SUCCEEDED or HCI error code
 *
 *************************************************************************/
UINT8 hci_handle_le_set_advertising_enable(HCI_CMD_PKT *hci_cmd_ptr)
{
    UCHAR event_parameter[4];
    UINT8 adv_enable = hci_cmd_ptr->cmd_parameter[0];
    UINT8 errcode = HCI_COMMAND_SUCCEEDED;
    UINT8 *pCCE_returnPara = &event_parameter[3];

    do
    {
        if (adv_enable > 0x01)
        {
            errcode = INVALID_HCI_COMMAND_PARAMETERS_ERROR;
            break;
        }

        if (adv_enable)
        {
            /* to check current state is valid or not */
            if (!ll_fw_check_multiple_states_valid(LL_STATE_ADVERTISING,
                                      ll_manager.adv_unit.hci_pdu_type))
            {
                errcode = COMMAND_DISALLOWED_ERROR;
                break;
            }
        }

        ll_manager.adv_unit.enable = adv_enable;

        if (adv_enable)
        {
            /* enable advertising mode */
#ifdef _CCH_RETENTION_FLOW_FOR_DLPS_LE_
            ll_driver_enable_advertising(FALSE);
#else
            ll_driver_enable_advertising();
#endif
        }
        else
        {
            /* disable advertising mode */
            ll_driver_disable_advertising();
        }

#ifdef MINICARD_BT_LED_CONTROL
        BT_LED_WPAN_ON();
#endif
    }
    while (0);

    /* generate command complete event */
    hci_generate_le_command_complete_event_head(hci_cmd_ptr->cmd_opcode,
            event_parameter);
    pCCE_returnPara[0] = errcode;
    LL_HCI_GEN_COMMAND_COMPLETE_EVENT(event_parameter, 1);

    if (errcode == HCI_COMMAND_SUCCEEDED)
    {
        lmp_update_lps_para();
    }

    return errcode;
}

/**************************************************************************
 * Function     : hci_handle_le_set_scan_parameters
 *
 * Description  : This function is used to handle HCI LE Set Scan Parameters.
 *                Please refer 7.8.10 in HCI part of BT 4.0 spec.
 *
 * Parameters   : hci_cmd_ptr Pointer to the command packet.
 *
 * Returns      : HCI_COMMAND_SUCCEEDED or HCI error code
 *
 *************************************************************************/
UINT8 hci_handle_le_set_scan_parameters(HCI_CMD_PKT *hci_cmd_ptr)
{
    UCHAR event_parameter[4];
    UINT8 scan_type;
    UINT16 scan_interval;
    UINT16 scan_window;
    UINT8 local_addr_type;
    UINT8 scan_filter_policy;
    UINT8 *pComPara = hci_cmd_ptr->cmd_parameter;
    UINT8 errcode = HCI_COMMAND_SUCCEEDED;
    UINT8 *pCCE_returnPara = &event_parameter[3];

    do
    {
        if (ll_manager.scan_unit.enable)
        {
            /* the advertising is enabled. Command Disallowed error
               code shall be used */
            errcode = COMMAND_DISALLOWED_ERROR;
            break;
        }

        /*----------------------------------------------------*/
        /*         Parser the Command Parameters              */
        /*----------------------------------------------------*/
        /* LE_Scan_Type - byte 0 */
        scan_type = pComPara[0];

        /* LE_Scan_Interval - byte 1~2 */
        scan_interval = pComPara[1] | (pComPara[2] << 8);

        /* LE_Scan_Window - byte 3~4 */
        scan_window = pComPara[3] | (pComPara[4] << 8);

        /* Own_Address_Type  - byte 5 */
        local_addr_type = pComPara[5];

        /* Scanning_Filter_Policy - byte 6 */
        scan_filter_policy = pComPara[6];

        if ((scan_type > 0x01) ||
            (scan_interval < LL_SCAN_INTERVAL_MIN) ||
            (scan_interval > LL_SCAN_INTERVAL_MAX) ||
            (scan_window < LL_SCAN_WINDOW_MIN) ||
            (scan_window > LL_SCAN_INTERVAL_MAX) ||
            (scan_window > scan_interval) ||
            (local_addr_type > LL_ADDR_TYPE_MAX) ||
#ifdef _BT4_2_PRIVACY_FEATURE_SUPPORT_
            (scan_filter_policy > 0x03)) /* supported Extended Scanner Filter policies */
#else
            (scan_filter_policy > 0x01))
#endif
        {
            errcode = INVALID_HCI_COMMAND_PARAMETERS_ERROR;
            break;
        }

        /* to check current state is valid or not */
        if (!ll_fw_check_multiple_states_valid(LL_STATE_SCANNING ,scan_type))
        {
            errcode = COMMAND_DISALLOWED_ERROR;
            break;
        }

#ifdef _DAPE_TEST_NEW_HW_LE_UNCONN_STATE_HIGHER_THAN_LEGACY_UNCONN
        if ((scan_interval - scan_window) < LEGACY_INQUIRY_RESERVED_12SLOT * 12)
        {
            scan_interval = scan_interval + (LEGACY_INQUIRY_RESERVED_12SLOT * 12);
        }
#endif
        /* save valid scan paramteres */
        ll_manager.scan_unit.local_addr_type = local_addr_type;
        ll_manager.scan_unit.filter_policy = scan_filter_policy;
        ll_manager.scan_unit.active_scan = scan_type;
        ll_manager.scan_unit.interval = scan_interval;
        ll_manager.scan_unit.window = scan_window;
        ll_manager.scan_unit.ch_map = g_le_scanner_used_channel_map;
    }
    while (0);

    /* generate command complete event */
    hci_generate_le_command_complete_event_head(hci_cmd_ptr->cmd_opcode,
            event_parameter);
    pCCE_returnPara[0] = errcode;
    LL_HCI_GEN_COMMAND_COMPLETE_EVENT(event_parameter, 1);

    LL_LOG_TRACE(WHITE, LE_MSG_LL_SCAN_PARAS, 6,
                 ll_manager.scan_unit.active_scan,
                 ll_manager.scan_unit.local_addr_type,
                 ll_manager.scan_unit.interval,
                 ll_manager.scan_unit.window,
                 ll_manager.scan_unit.ch_map,
                 ll_manager.scan_unit.filter_policy);

    if (errcode == HCI_COMMAND_SUCCEEDED)
    {
        lmp_update_lps_para();
    }

    return errcode;
}

/**************************************************************************
 * Function     : hci_handle_le_set_scan_enable
 *
 * Description  : This function is used to handle HCI LE Set Scan Enable.
 *                Please refer 7.8.11 in HCI part of BT 4.0 spec.
 *
 * Parameters   : hci_cmd_ptr Pointer to the command packet.
 *
 * Returns      : HCI_COMMAND_SUCCEEDED or HCI error code
 *
 *************************************************************************/
UINT8 hci_handle_le_set_scan_enable(HCI_CMD_PKT *hci_cmd_ptr)
{
    UCHAR event_parameter[4];
    UINT8 scan_enable;
    UINT8 filter_duplicates;
    UINT8 errcode = HCI_COMMAND_SUCCEEDED;
    UINT8 *pCCE_returnPara = &event_parameter[3];

    /*----------------------------------------------------*/
    /*         Parser the Command Parameters              */
    /*----------------------------------------------------*/
    /* LE_Scan_Enable - byte 0 */
    scan_enable = hci_cmd_ptr->cmd_parameter[0];

    /* Filter_Duplicates - byte 1 */
    filter_duplicates = hci_cmd_ptr->cmd_parameter[1];


    if ((scan_enable > 0x01) || (filter_duplicates > 0x01))
    {
        errcode = INVALID_HCI_COMMAND_PARAMETERS_ERROR;
    }
    else
    {
        ll_manager.scan_unit.filter_duplicate = filter_duplicates;
        ll_manager.scan_unit.enable = scan_enable;

#ifdef _DAPE_FIX_HW_NO_LE_SCAN_N_CONN_UPDT_SAME_TIME
        /* if connection update is in procedure, pending scan. */
        if ((ll_manager.conn_unit.conn_updt_entry == LL_MAX_CONNECTION_UNITS) &&
            (ll_manager.conn_unit.chm_updt_entry == LL_MAX_CONNECTION_UNITS))

        {
            if (scan_enable)
            {
                // enable scanning
#ifdef _CCH_RETENTION_FLOW_FOR_DLPS_LE_
                ll_driver_enable_scanning(FALSE);
#else
                ll_driver_enable_scanning();
#endif
            }
            else
            {
                // disable scanning
                ll_driver_disable_scanning();
                ll_driver_clear_all_duplicated_flags_in_list(LL_WHITE_LIST_TYPE);
                ll_driver_clear_all_duplicated_flags_in_list(LL_BLACK_LIST_TYPE);
            }
        }
        else
        {
            RT_BT_LOG(RED, DAPE_TEST_LOG540, 4, scan_enable,
                     ll_manager.conn_unit.conn_updt_entry,
                     ll_manager.conn_unit.chm_updt_entry,
                     ll_manager.scan_unit.filter_duplicate);
        }
#else

#ifdef _SUPPORT_MSFT_BT_HCI_EXTENSION_FINAL_SPEC_
        UINT8 ori_filter_policy = ll_manager.scan_unit.filter_policy;

        if ((ll_rssi_manager.msft_monitor_rssi_mode ==
                    LE_MSFT_MONITOR_RSSI_VIA_CONN_HANDLE_MODE_MSFT) &&
                        (ll_rssi_manager.le_adv_monitor_type > 0))
        {
            if (scan_enable)
            {
                if (ll_manager.scan_unit.filter_policy)
                {
                    ll_manager.scan_unit.sw_filter_policy = 1;
                    ll_manager.scan_unit.filter_policy = 0;
                }
            }
            else
            {
                ll_manager.scan_unit.sw_filter_policy = 0;
            }
        }
#endif

        if (scan_enable)
        {
#ifdef CONFIG_TV_POWERON
            if (efuse_tv_poweron->power_on_tv_en)
            {
                tv_poweron_set_le_scan(FALSE);
            }
#endif
            // enable scanning
#ifdef _CCH_RETENTION_FLOW_FOR_DLPS_LE_
            ll_driver_enable_scanning(FALSE);
#else
            ll_driver_enable_scanning();
#endif

#ifdef _SUPPORT_MSFT_BT_HCI_EXTENSION_FINAL_SPEC_
            ll_manager.scan_unit.filter_policy = ori_filter_policy;
#endif
        }
        else
        {
            // disable scanning
            ll_driver_disable_scanning();
            ll_driver_clear_all_duplicated_flags_in_list(LL_WHITE_LIST_TYPE);
            ll_driver_clear_all_duplicated_flags_in_list(LL_BLACK_LIST_TYPE);
#ifdef CONFIG_TV_POWERON
            if (efuse_tv_poweron->power_on_tv_en)
            {
                tv_poweron_set_le_scan(TRUE);
            }
#endif
        }
#endif
#ifdef MINICARD_BT_LED_CONTROL
        BT_LED_WPAN_ON();
#endif
    }

    /* generate command complete event */
    hci_generate_le_command_complete_event_head(hci_cmd_ptr->cmd_opcode,
            event_parameter);
    pCCE_returnPara[0] = errcode;
    LL_HCI_GEN_COMMAND_COMPLETE_EVENT(event_parameter, 1);

    /* we need to generate zero or more LE Advertising Reports are generated by
       the controller based on advertising packets received and the duplicate
       filtering */
    LL_LOG_TRACE(WHITE, LE_MSG_LL_SCAN_EN, 2, scan_enable, filter_duplicates);

    if (errcode == HCI_COMMAND_SUCCEEDED)
    {
        lmp_update_lps_para();
    }

    return errcode;
}

/**************************************************************************
 * Function     : hci_handle_le_create_connection
 *
 * Description  : This function is used to handle HCI LE Create Connection.
 *                Please refer 7.8.12 in HCI part of BT 4.0 spec.
 *
 * Parameters   : hci_cmd_ptr Pointer to the command packet.
 *
 * Returns      : HCI_COMMAND_SUCCEEDED or HCI error code
 *
 *************************************************************************/
UINT8 hci_handle_le_create_connection(HCI_CMD_PKT *hci_cmd_ptr)
{
    UINT16 scan_interval;
    UINT16 scan_window;
    UINT8 filt_policy;
    UINT8 peer_addr_type;
    UINT8 own_addr_type;
    UINT8 conn_entry = LL_MAX_CONNECTION_UNITS;
    UINT16 conn_interval_min;
    UINT16 conn_interval_max;
    UINT16 conn_latency;
    UINT16 supervision_timeout;
    UINT16 ce_length_min;
    UINT16 ce_length_max;
    UINT8 *pComPara = hci_cmd_ptr->cmd_parameter;
    UINT8 errcode = INVALID_HCI_COMMAND_PARAMETERS_ERROR;
    LL_INITIATOR_UNIT *initiator = &ll_manager.initiator_unit;
    UINT8 err_case = 0;

    do
    {
        if (initiator->enable)
        {
            /* the initiator is enabled. Command Disallowed error
               code shall be used */
            errcode = COMMAND_DISALLOWED_ERROR;
            break;
        }

        /*----------------------------------------------------*/
        /*         Parser the Command Parameters              */
        /*----------------------------------------------------*/
        /* LE Scan Interval - byte 0~1 */
        scan_interval = pComPara[0] | (pComPara[1] << 8);

        /* LE Scan Window - byte 2~3 */
        scan_window = pComPara[2] | (pComPara[3] << 8);

        /* filter policy - byte 4
           0x00: no use white list, use peer_address_type, peer_address
           0x01: use white list, ignore peer_address_type, peer_address */
        filt_policy = pComPara[4];

        /* peer address type - byte 5 */
        peer_addr_type = pComPara[5];

        /* peer address - byte 6~11 */

        /* own address type - byte 12 */
        own_addr_type = pComPara[12];

        /* Conn_Interval_Min - byte 13~14 */
        conn_interval_min = pComPara[13] | (pComPara[14] << 8);

        /* Conn_Interval_Max - byte 15~16 */
        conn_interval_max = pComPara[15] | (pComPara[16] << 8);

        /* Conn_Latency - byte 17~18 */
        conn_latency = pComPara[17] | (pComPara[18] << 8);

        /* Supervision_Timeout - byte 19~20 */
        supervision_timeout = pComPara[19] | (pComPara[20] << 8);

        /* Minimum_CE_Length - byte 21~22 */
        ce_length_min = pComPara[21] | (pComPara[22] << 8);

        /* Maximum_CE_Length - byte 23~24 */
        ce_length_max = pComPara[23] | (pComPara[24] << 8);
        if ((scan_interval < LL_INITIATOR_SCAN_INTERVAL_MIN) ||
            (scan_interval > LL_INITIATOR_SCAN_INTERVAL_MAX) ||
            (scan_window < LL_INITIATOR_SCAN_WINDOW_MIN) ||
            (scan_window > LL_INITIATOR_SCAN_WINDOW_MAX) ||
            (scan_window > scan_interval))
        {
            err_case = 1;
            break;
        }

        if ((filt_policy > 0x01) ||
#ifdef _BT4_2_PRIVACY_FEATURE_SUPPORT_
                (peer_addr_type > 0x03) || (own_addr_type > 0x03))
#else
                (peer_addr_type > 0x01) || (own_addr_type > 0x01))
#endif
        {
            err_case = 2;
            break;
        }

        if ((conn_interval_min < LL_CONN_INTERVAL_MIN) ||
            (conn_interval_min > LL_CONN_INTERVAL_MAX) ||
            (conn_interval_max < LL_CONN_INTERVAL_MIN) ||
            (conn_interval_max > LL_CONN_INTERVAL_MAX) ||
            (conn_interval_min > conn_interval_max))
        {
            err_case = 3;
            break;
        }

        if (conn_latency > LL_CONN_LATENCY_MAX)
        {
            err_case = 4;
            break;
        }

#ifndef _LE_SUPPORT_CSA3_LE_ERRATA_
        if ((supervision_timeout < LL_SUPERVISION_TO_MIN) ||
            (supervision_timeout > LL_SUPERVISION_TO_MAX) ||
            (supervision_timeout <= (conn_interval_max >> 3)))
        {
            /* supervision_timeout (unit:10ms) in milliseconds shall be larger
               than the conn_interval_max (unit:1.25ms) in milliseconds */
            err_case = 5;
            break;
        }
#else
        if ((supervision_timeout < LL_SUPERVISION_TO_MIN) ||
            (supervision_timeout > LL_SUPERVISION_TO_MAX) ||
            (supervision_timeout <= (((conn_interval_max >> 3) << 1) * (1 + conn_latency))))
        {
            /* The Supervision_Timeout in milliseconds shall be larger
               than (1 + Conn_Latency) Conn_Interval_Max * 2,
               where Conn_Interval_Max is given in milliseconds. */
            err_case = 6;
            break;
        }
#endif
#ifdef _3DD_TV_CORRECT_HOST_ERROR_
#ifdef CONFIG_TV_POWERON
        /* (dape) If power on TV LE scanner is enabled, then shorten scan interval of
           LE initiator.
           This will help LE initiator to get connected with remote device. (or LE scanner
           will lower the probability that LE initiator hit adv pkt.)*/
        if (tv_poweron.scan.enable)
        {
            if (scan_window < (scan_interval>>2))
            {
                scan_interval = scan_window <<2;
            }
        }
#endif
#endif
#ifdef _DAPE_TEST_NEW_HW_LE_UNCONN_STATE_HIGHER_THAN_LEGACY_UNCONN
        if ((scan_interval - scan_window) < LEGACY_INQUIRY_RESERVED_12SLOT * 12)
        {
            scan_interval = scan_interval + (LEGACY_INQUIRY_RESERVED_12SLOT * 12);
        }
#endif
        /* ce_length(ms) should be no longer than conn_interval(ms) -1.25ms*/
        if ((ce_length_max < ce_length_min) ||
            ((ce_length_min >> 1) > (conn_interval_max - 1)))
        {
            err_case = 7;
            break;
        }

        if (conn_latency != 0)
        {
            UINT16 conn_intv_val;
            conn_intv_val = (supervision_timeout << 3) / (conn_latency + 1);
            if (conn_interval_min > conn_intv_val)
            {
                /* slave latency shall be an integer beteen 0 and
                   (supervision timeout / conn_interval) - 1 */
                err_case = 8;
                break;
            }
        }

        /* to check current state is valid or not */
        if (!ll_fw_check_multiple_states_valid(LL_STATE_INITIATING, 0))
        {
            errcode = COMMAND_DISALLOWED_ERROR;
            break;
        }

        /* get a free connection entry */
        conn_entry = ll_fw_get_free_conn_entry();
        if (conn_entry == LL_MAX_CONNECTION_UNITS)
        {
            /* no free connection, so we report it to host */
            errcode = MAX_NUMBER_OF_CONNECTIONS_ERROR;
            LL_LOG_TRACE(RED, LE_MSG_NO_FREE_CONN_ENTRY, 0, 0);
            break;
        }

        initiator->access_address = ll_fw_generate_access_address();
        initiator->scan_interval = scan_interval;
        initiator->scan_window = scan_window;
        initiator->filter_policy = filt_policy;
        initiator->remote_addr_type = peer_addr_type;
        memcpy(initiator->remote_addr, &pComPara[6], 6);
        initiator->local_addr_type = own_addr_type;
        initiator->conn_interval_min = conn_interval_min;
        initiator->conn_interval_max = conn_interval_max;
        /* dape added. To choose a conn_interval to be a muptiple
           of ((g_le_use_interval_slot_pair+1)*2) slots.*/
        initiator->conn_interval = conn_interval_max -
                       (conn_interval_max % (g_le_use_interval_slot_pair + 1));

        if (initiator->conn_interval < conn_interval_min)
        {
            initiator->conn_interval = conn_interval_max -
                  (conn_interval_max % ((g_le_use_interval_slot_pair>>1) + 1));
#ifdef _DAPE_CHOOSE_LE_INTV_BY_CONTROLLER
            if (!otp_str_data.bt_auto_choose_le_intv)
#endif
            {
                if (initiator->conn_interval < conn_interval_min)
                {
                    initiator->conn_interval = conn_interval_min;
                }
            }
        }
        initiator->conn_latency = conn_latency;
        initiator->ch_map = g_le_initiator_used_channel_map;
        initiator->supervision_to = supervision_timeout;
#ifdef _DAPE_SET_CE_LENGTH_NON_ZERO
        /* this is used to set ce_length that win8 doesn't set this. */
        if (ce_length_max == 0)
        {
            ce_length_max = (initiator->conn_interval - 1) << 1;
        }
        if (ce_length_min == 0)
        {
            ce_length_min = ce_length_max;
        }
#endif
        initiator->ce_len_min = ce_length_min;
        initiator->ce_len_max = ce_length_max;
        //initiator->ce_len = ce_length_min;
        /* dape added. ce_length(ms) should be no longer than
           conn_interval(ms) -1.25ms*/
        initiator->ce_len = MIN(ce_length_min,
                                (initiator->conn_interval - 1) << 1);

        /* compute transmit window offset (0 to connInterval)*/
        initiator->tx_win_offset = initiator->conn_interval >> 1;

        /* compute transmit window size
           (1.25ms ~ min(10 ms ~ (connInterval - 1.25ms))) */
        initiator->tx_win_size = MIN(8, initiator->conn_interval - 1);

        initiator->entry_id = conn_entry;
        initiator->enable = 1;

        errcode = HCI_COMMAND_SUCCEEDED;
    }
    while (0);

    /* generate command status event */
    hci_generate_command_status_event(hci_cmd_ptr->cmd_opcode, errcode);


    if (errcode == HCI_COMMAND_SUCCEEDED)
    {

#ifdef _DAPE_TEST_SLOT_OFFSET_FOR_LE
        UINT16 conn_interval_to_be_set = 0;
        /* dape added. To set all connections to the same interval. */
        if (IS_LE_CONN_INTV_AUTO_UPDATE_EN)
        {
            /* If IS_LE_CONN_INTV_DEFAULT_EN is not enabled,
               and this is the first connection, then use this interval
               as global connection interval. */
            if ((!g_le_conn_interval_set)&& (!IS_LE_CONN_INTV_DEFAULT_EN))
            {
                g_le_conn_interval = initiator->conn_interval;
                g_le_conn_interval_set = 1;
            }
            /* If IS_LE_CONN_INTV_DEFAULT_EN is enabled, then use
               LE_CONN_INTV_DEFAULT_VALUE as global connection interval. */
            if (IS_LE_CONN_INTV_DEFAULT_EN)
            {
                conn_interval_to_be_set = LE_CONN_INTV_DEFAULT_VALUE;
        	}
            else
            {
                conn_interval_to_be_set = g_le_conn_interval;
            }
            /* If initiator's current interval is not the same as the
               conn_interval_to_be_set, then enable g_le_conn_interval_changed
               for indication that we should send
               connection_update_complete_event to host when receiving
               conn_interrupt. */
            if (initiator->conn_interval != conn_interval_to_be_set)
            {
                RT_BT_LOG(RED, DAPE_TEST_LOG425, 3, conn_entry,
                            initiator->conn_interval, conn_interval_to_be_set);
                initiator->conn_interval = g_le_conn_interval;
                g_le_conn_interval_changed |= (1 << conn_entry);
            }
        }
#endif

#ifdef _ROM_CODE_PATCHED_
        /* adjust initiator parameters via rom code patch */
        if (rcp_hci_le_adjust_conn_req_func != NULL)
        {
            rcp_hci_le_adjust_conn_req_func();
        }
#endif

        /* dape added to check all ce intervals */
        ll_fw_check_all_slave_interval();
        ll_manager.conn_unit.tx_win_offset = initiator->tx_win_offset;

        /* create connection */
        ll_driver_create_connection(conn_entry);

#ifdef MINICARD_BT_LED_CONTROL
        BT_LED_WPAN_ON();
#endif
    }

    LL_LOG_TRACE(WHITE, LE_MSG_CREATE_CONN_PARA0, 13,
                 errcode, err_case, initiator->access_address,
                 initiator->scan_interval, initiator->scan_window,
                 initiator->filter_policy, initiator->remote_addr_type,
                 initiator->remote_addr[0], initiator->remote_addr[1],
                 initiator->remote_addr[2], initiator->remote_addr[3],
                 initiator->remote_addr[4], initiator->remote_addr[5]);

    LL_LOG_TRACE(WHITE, LE_MSG_CREATE_CONN_PARA1, 12,
                 initiator->local_addr_type, initiator->ch_map,
                 initiator->conn_interval_min, initiator->conn_interval_max,
                 initiator->conn_interval, initiator->tx_win_offset,
                 initiator->tx_win_size, initiator->conn_latency,
                 initiator->supervision_to, initiator->ce_len_min,
                 initiator->ce_len_max, initiator->ce_len);

    /* we need to create LE_Connection_Complete_Event when a connection is
       create or the connection creation procedure is cancelled */
    return errcode;
}

/**************************************************************************
 * Function     : hci_handle_le_create_connection_cancel
 *
 * Description  : This function is used to handle HCI LE Create Connection
 *                Cancel. Please refer 7.8.13 in HCI part of BT 4.0 spec.
 *
 * Parameters   : hci_cmd_ptr Pointer to the command packet.
 *
 * Returns      : HCI_COMMAND_SUCCEEDED or HCI error code
 *
 *************************************************************************/
UINT8 hci_handle_le_create_connection_cancel(HCI_CMD_PKT *hci_cmd_ptr)
{
    UINT8 errcode;
    UINT8 send_le_conn_complete_event = FALSE;

    /* this command shall only be issued after the LE_Create_Connection command
       has been issued, a Command Status event has been received for the LE
       Create Connection command and before the LE Connection Complete event */
    if (ll_manager.initiator_unit.enable)
    {
        ll_driver_create_connection_cancel();

        if (!ll_manager.initiator_unit.enable)
        {
            send_le_conn_complete_event = TRUE;
        }
        errcode = HCI_COMMAND_SUCCEEDED;
    }
    else
    {
        /* the LE_Create_Connection_Cancel cmmand is sent to the controller
           without a preceding LE_Create_Connection command */
        errcode = COMMAND_DISALLOWED_ERROR;
    }

    /* generate command complete event */
    UCHAR event_parameter[4];
    hci_generate_le_command_complete_event_head(hci_cmd_ptr->cmd_opcode,
            event_parameter);
    UINT8 *pCCE_returnPara = &event_parameter[3];
    pCCE_returnPara[0] = errcode;
    LL_HCI_GEN_COMMAND_COMPLETE_EVENT(event_parameter, 1);

    /* we shall sent a LE Connection Complete event with the error code
       Unknown Connection Identifier(0x02)if the cancellation was successful */
    if (send_le_conn_complete_event)
    {
#ifdef _BT4_2_PRIVACY_FEATURE_SUPPORT_
        UINT16 le_event_mask = ll_manager.le_event_mask[0] | (ll_manager.le_event_mask[1] << 8);
        if (le_event_mask & LE_ENHANCED_CONNECTION_COMPLETE_EVENT_MASK)
        {
            hci_generate_le_enhanced_connection_complete_event(NO_CONNECTION_ERROR, NULL);
        }
        else
        {
            hci_generate_le_connection_complete_event(NO_CONNECTION_ERROR, NULL);
        }
#else
        hci_generate_le_connection_complete_event(NO_CONNECTION_ERROR, NULL);
#endif
        LL_INITIATOR_UNIT *initiator = &ll_manager.initiator_unit;
        UINT8 conn_entry_id;
        conn_entry_id = initiator->entry_id;
        ll_manager.conn_unit.bmActiveHandle &= ~ (1 << conn_entry_id);
    }

    return HCI_COMMAND_SUCCEEDED;
}

/**************************************************************************
 * Function     : hci_handle_le_read_white_list_size
 *
 * Description  : This function is used to handle HCI LE Read White List Size
 *                Command. Please refer 7.8.14 in HCI part of BT 4.0 spec.
 *
 * Parameters   : hci_cmd_ptr Pointer to the command packet.
 *
 * Returns      : HCI_COMMAND_SUCCEEDED or HCI error code
 *
 *************************************************************************/
UINT8 hci_handle_le_read_white_list_size(HCI_CMD_PKT *hci_cmd_ptr)
{
    UCHAR event_parameter[5];
    UINT8 *pCCE_returnPara = &event_parameter[3];

    hci_generate_le_command_complete_event_head(hci_cmd_ptr->cmd_opcode,
            event_parameter);

    /* status field - byte 0 */
    pCCE_returnPara[0] = HCI_COMMAND_SUCCEEDED;

    /* white_list_size - byte 1 */
    pCCE_returnPara[1] = LL_MAX_WHITE_LIST_SIZE;

    LL_HCI_GEN_COMMAND_COMPLETE_EVENT(event_parameter, 2);

    return HCI_COMMAND_SUCCEEDED;
}

/**************************************************************************
 * Function     : hci_handle_le_clear_white_list
 *
 * Description  : This function is used to handle HCI LE Clear White List
 *                Command. Please refer 7.8.15 in HCI part of BT 4.0 spec.
 *
 * Parameters   : hci_cmd_ptr Pointer to the command packet.
 *
 * Returns      : HCI_COMMAND_SUCCEEDED or HCI error code
 *
 *************************************************************************/
UINT8 hci_handle_le_clear_white_list(HCI_CMD_PKT *hci_cmd_ptr)
{
    UINT8 errcode = COMMAND_DISALLOWED_ERROR;

    do
    {
        if (ll_manager.adv_unit.enable &&
            (ll_manager.adv_unit.filter_scan_req ||
             ll_manager.adv_unit.filter_conn_req))
        {
            /* advertising filter policy uses the white list and
               advertising is enabled */
            break;
        }

        if (ll_manager.scan_unit.enable &&
            ll_manager.scan_unit.filter_policy)
        {
            /* scanning filter policy uses the white list and
               scanning is enabled */
            break;
        }

        if (ll_manager.initiator_unit.enable &&
            ll_manager.initiator_unit.filter_policy)
        {
            /* Initiator filter policy uses the white list and an
               LE_Create_Connection command is outstanding */
            break;
        }

        /* clear white list */
        ll_driver_reset_dev_list(LL_WHITE_LIST_TYPE);

        errcode = HCI_COMMAND_SUCCEEDED;
    }
    while (0);

    /* generate command complete event */
    UCHAR event_parameter[4];
    UINT8 *pCCE_returnPara = &event_parameter[3];
    hci_generate_le_command_complete_event_head(hci_cmd_ptr->cmd_opcode,
            event_parameter);
    pCCE_returnPara[0] = errcode;
    LL_HCI_GEN_COMMAND_COMPLETE_EVENT(event_parameter, 1);

    return errcode;
}

/**************************************************************************
 * Function     : hci_handle_le_add_device_to_white_list
 *
 * Description  : This function is used to handle HCI LE Add Device To White
 *                List Command. Please refer 7.8.16 in HCI part of BT 4.0 spec.
 *
 * Parameters   : hci_cmd_ptr Pointer to the command packet.
 *
 * Returns      : HCI_COMMAND_SUCCEEDED or HCI error code
 *
 *************************************************************************/
UINT8 hci_handle_le_add_device_to_white_list(HCI_CMD_PKT *hci_cmd_ptr)
{
    UINT8 errcode = COMMAND_DISALLOWED_ERROR;
    UINT8 *cmd_para = hci_cmd_ptr->cmd_parameter;
    UINT8 list_entry;

    do
    {
        if (cmd_para[0] > 0x01)
        {
            /* invalid address type */
            errcode = INVALID_HCI_COMMAND_PARAMETERS_ERROR;
            break;
        }

        if (ll_manager.adv_unit.enable &&
            (ll_manager.adv_unit.filter_scan_req ||
             ll_manager.adv_unit.filter_conn_req))
        {
            /* advertising filter policy uses the white list and
               advertising is enabled */
            break;
        }

        if (ll_manager.scan_unit.enable &&
            ll_manager.scan_unit.filter_policy)
        {
            /* scanning filter policy uses the white list and
               scanning is enabled */
            break;
        }

        if (ll_manager.initiator_unit.enable &&
            ll_manager.initiator_unit.filter_policy)
        {
            /* Initiator filter policy uses the white list and an
               LE_Create_Connection command is outstanding */
            break;
        }

        UINT8 addr_type = cmd_para[0];
        UINT8 *addr = &cmd_para[1];
        if (ll_driver_search_dev_from_list(LL_WHITE_LIST_TYPE, addr_type,
                addr) >= LL_MAX_WHITE_LIST_SIZE)
        {
#ifdef _BT4_2_PRIVACY_FEATURE_SUPPORT_
            list_entry = ll_driver_add_dev_to_list(LL_WHITE_LIST_TYPE, addr_type, addr, NULL, NULL);
#else
            list_entry = ll_driver_add_dev_to_list(LL_WHITE_LIST_TYPE, addr_type, addr);
#endif

            if (LL_MAX_WHITE_LIST_SIZE == list_entry)
            {
                /* no free entries to add white list. In test spec HCI1.TS.4.0.0a
                 - TP/CIN/BV-06-C, we shall set error code 0x07 in this case */
                errcode = MEMORY_FULL_ERROR;
                break;
            }
        }

        errcode = HCI_COMMAND_SUCCEEDED;
    }
    while (0);

    /* generate command complete event */
    UCHAR event_parameter[4];
    UINT8 *pCCE_returnPara = &event_parameter[3];
    hci_generate_le_command_complete_event_head(hci_cmd_ptr->cmd_opcode,
            event_parameter);
    pCCE_returnPara[0] = errcode;
    LL_HCI_GEN_COMMAND_COMPLETE_EVENT(event_parameter, 1);

    return errcode;
}

/**************************************************************************
 * Function     : hci_handle_le_remove_device_from_white_list
 *
 * Description  : This function is used to handle HCI LE Remote Device From
 *                White List Command. Please refer 7.8.17 in HCI part of BT 4.0
 *                spec.
 *
 * Parameters   : hci_cmd_ptr Pointer to the command packet.
 *
 * Returns      : HCI_COMMAND_SUCCEEDED or HCI error code
 *
 *************************************************************************/
UINT8 hci_handle_le_remove_device_from_white_list(HCI_CMD_PKT *hci_cmd_ptr)
{
    UINT8 errcode = COMMAND_DISALLOWED_ERROR;
    UINT8 addr_type;

    do
    {
        addr_type = hci_cmd_ptr->cmd_parameter[0];
        if (addr_type > 0x01)
        {
            /* invalid address type */
            errcode = INVALID_HCI_COMMAND_PARAMETERS_ERROR;
            break;
        }

        if (ll_manager.adv_unit.enable &&
            (ll_manager.adv_unit.filter_scan_req ||
             ll_manager.adv_unit.filter_conn_req))
        {
            /* advertising filter policy uses the white list and
               advertising is enabled */
            break;
        }

        if (ll_manager.scan_unit.enable &&
            ll_manager.scan_unit.filter_policy)
        {
            /* scanning filter policy uses the white list and
               scanning is enabled */
            break;
        }

        if (ll_manager.initiator_unit.enable &&
            ll_manager.initiator_unit.filter_policy)
        {
            /* Initiator filter policy uses the white list and an
               LE_Create_Connection command is outstanding */
            break;
        }

        /* begin to remove device from white list */
        ll_driver_remove_dev_from_list(LL_WHITE_LIST_TYPE,
                                  addr_type, &hci_cmd_ptr->cmd_parameter[1]);

        errcode = HCI_COMMAND_SUCCEEDED;
    }
    while (0);

    /* generate command complete event */
    UCHAR event_parameter[4];
    UINT8 *pCCE_returnPara = &event_parameter[3];
    hci_generate_le_command_complete_event_head(hci_cmd_ptr->cmd_opcode,
            event_parameter);
    pCCE_returnPara[0] = errcode;
    LL_HCI_GEN_COMMAND_COMPLETE_EVENT(event_parameter, 1);

    return errcode;

}

/**************************************************************************
 * Function     : hci_handle_le_connection_update
 *
 * Description  : This function is used to handle HCI LE Connection Update
 *                Command. Please refer 7.8.18 in HCI part of BT 4.0 spec.
 *
 * Parameters   : hci_cmd_ptr Pointer to the command packet.
 *
 * Returns      : HCI_COMMAND_SUCCEEDED or HCI error code
 *
 *************************************************************************/
UINT8 hci_handle_le_connection_update(HCI_CMD_PKT *hci_cmd_ptr)
{
    UINT16 conn_handle;
    UINT16 conn_interval_min;
    UINT16 conn_interval_max;
    UINT16 conn_interval;
    UINT16 conn_latency;
    UINT16 supervision_timeout;
    UINT16 ce_length_min;
    UINT16 ce_length_max;
    UINT16 ce_length;
    UINT8 *pComPara = hci_cmd_ptr->cmd_parameter;
    UINT8 errcode = INVALID_HCI_COMMAND_PARAMETERS_ERROR;
    LL_CONN_HANDLE_UNIT *pHandle;

    do
    {
#ifdef _DAPE_FIX_LE_CONN_UPDATE
        WR_LE_REG(0x22, 0);
#endif
        if (!ll_manager.conn_unit.enable || !ll_manager.conn_unit.master)
        {
            /* not setup a connection or we are not master role */
            errcode = COMMAND_DISALLOWED_ERROR;
            break;
        }

        /*----------------------------------------------------*/
        /*         Parser the Command Parameters              */
        /*----------------------------------------------------*/
        /* Connection_Handle - byte 0~1 */
        conn_handle = pComPara[0] | (pComPara[1] << 8);

        /* Connection Interval Min - byte 2~3 */
        conn_interval_min = pComPara[2] | (pComPara[3] << 8);

        /* Connection Interval Max  - byte 4~5 */
        conn_interval_max = pComPara[4] | (pComPara[5] << 8);

#ifdef _CCH_8821B_TEST_LE_CONN_UPD
        conn_interval_max = (conn_interval_max<<2);
#endif

        /* Connection Latency - byte 6~7 */
        conn_latency = pComPara[6] | (pComPara[7] << 8);

        /* Supervision Timeout - byte 8~9 */
        supervision_timeout = pComPara[8] | (pComPara[9] << 8);

#ifdef _CCH_8821B_TEST_LE_CONN_UPD
        supervision_timeout = (supervision_timeout<<2);
#endif

        /* Minimum CE Length - byte 10~11 */
        ce_length_min = pComPara[10] | (pComPara[11] << 8);

        /* Maximum CE Length - byte 12~13 */
        ce_length_max = pComPara[12] | (pComPara[13] << 8);

        if ((conn_handle > LL_HCI_MAX_CONNECTION_HANDLE) ||
            (conn_interval_min < LL_CONN_INTERVAL_MIN) ||
            (conn_interval_min > LL_CONN_INTERVAL_MAX) ||
            (conn_interval_max < LL_CONN_INTERVAL_MIN) ||
            (conn_interval_max > LL_CONN_INTERVAL_MAX) ||
            (conn_interval_min > conn_interval_max))
        {
            break;
        }

        if (conn_latency > LL_CONN_LATENCY_MAX)
        {
            /* In fact, the valid range of this field is between 0x0000 and
               0x03E8 in BT4.0 HCI spec. But the LL spec defines slave latency
               can not be larger than 500.  Therefore, FW sets maximum value is
               500 here - austin */
            break;
        }

#ifndef _LE_SUPPORT_CSA3_LE_ERRATA_
        if ((supervision_timeout < LL_SUPERVISION_TO_MIN) ||
            (supervision_timeout > LL_SUPERVISION_TO_MAX) ||
            (supervision_timeout <= (conn_interval_max >> 3)))
        {
            /* supervision_timeout (unit:10ms) in milliseconds shall be larger
               than the conn_interval_max (unit:1.25ms) in milliseconds */
            break;
        }
#else
        if ((supervision_timeout < LL_SUPERVISION_TO_MIN) ||
            (supervision_timeout > LL_SUPERVISION_TO_MAX) ||
            (supervision_timeout <= (((conn_interval_max >> 3) << 1) * (1 + conn_latency))))
        {
            /* The Supervision_Timeout in milliseconds shall be larger
               than (1 + Conn_Latency) Conn_Interval_Max * 2,
               where Conn_Interval_Max is given in milliseconds. */
            break;
        }
#endif

        if (ce_length_max < ce_length_min)
        {
            break;
        }

        if (conn_latency != 0)
        {
            UINT16 conn_intv_val;
            conn_intv_val = (supervision_timeout << 3) / (conn_latency + 1);
            if (conn_interval_min > conn_intv_val)
            {
                /* slave latency shall be an integer beteen 0 and
                   (supervision timeout / conn_interval) - 1 */
                break;
            }
        }

        pHandle = ll_fw_search_handle_unit_via_conn_handle(conn_handle);

        if (pHandle != NULL)
        {
            if (pHandle->llc.pend_conn_updt)
            {
                /* one connection req update is processing. we should disallow
                   this commaand (or pend this command) ? - austin */
                errcode = COMMAND_DISALLOWED_ERROR;
                break;
            }
            /* dape added. To choose a conn_interval to be a muptiple
               of ((g_le_use_interval_slot_pair+1)*2) slots.*/
            conn_interval = conn_interval_max -
                   (conn_interval_max % (g_le_use_interval_slot_pair + 1));

            if (conn_interval < conn_interval_min)
            {
                conn_interval = conn_interval_max -
                (conn_interval_max % ((g_le_use_interval_slot_pair>>1) + 1));
                if (conn_interval < conn_interval_min)
                {
                    conn_interval = conn_interval_min;
                }
            }
#ifdef _DAPE_SET_CE_LENGTH_NON_ZERO
            /* this is used to set ce_length that win8 doesn't set this. */
            if (ce_length_max == 0)
            {
                ce_length_max = (conn_interval - 1) << 1;
            }
            if (ce_length_min == 0)
            {
                ce_length_min = ce_length_max;
            }
#endif
            ce_length = MIN(ce_length_min,
                                  (conn_interval - 1) << 1);

#ifdef _DAPE_LE_NO_2ND_CONN_UPDT_WHEN_PARA_SAME
            /* To reduce the update time to reduce the anchor point collision. */
            if (((pHandle->ce_interval) == conn_interval) &&
                ((pHandle->slave_latency) == conn_latency) &&
                ((pHandle->supervision_to) == supervision_timeout) &&
                ((pHandle->ce_length) == ce_length))
            {
                errcode = HCI_COMMAND_SUCCEEDED;
                hci_generate_command_status_event(hci_cmd_ptr->cmd_opcode, errcode);

                LL_TASK_PARA_U task_param;
                OS_SIGNAL sig_send;

                /* generate connection update complete event to host */
                task_param.Dword = 0;
                task_param.lmp_s.sub_type = LL_TASK_HANDLE_CONN_UPDT_COMP;
                task_param.lmp_s.conn_entry = conn_handle - LL_HCI_MIN_CONN_HANDLE;
                task_param.lmp_s.status = HCI_COMMAND_SUCCEEDED;
                sig_send.type = LL_GEN_HCI_EVENT_SIGNAL;
                sig_send.param = (OS_ADDRESS)task_param.Dword;
                OS_SEND_SIGNAL_TO_TASK (lmp_task_handle, sig_send);
#ifdef _DAPE_PRINT_MORE_LOG_INFO
                RT_BT_LOG(BLUE, DAPE_TEST_LOG570, 5, conn_handle,
                (pHandle->ce_interval), (pHandle->ce_length),
                (pHandle->slave_latency),
                (pHandle->supervision_to));
#endif

                return errcode;
            }
#endif
            /* update connection parameters*/
            pHandle->conn_updt_blk.ce_interval_min = conn_interval_min;
            pHandle->conn_updt_blk.ce_interval_max = conn_interval_max;
            pHandle->conn_updt_blk.ce_interval = conn_interval;
            pHandle->conn_updt_blk.slave_latency = conn_latency;
            pHandle->conn_updt_blk.supervision_to = supervision_timeout;

            pHandle->conn_updt_blk.ce_length_min = ce_length_min;
            pHandle->conn_updt_blk.ce_length_max = ce_length_max;
            //pHandle->conn_updt_blk.ce_length = ce_length_min;
            /* dape added. CE length shold be no longer than
               ce_interval - 1.25 ms */
            pHandle->conn_updt_blk.ce_length = ce_length;

            /* compute transmit window offset (0 to new connInterval)*/
            pHandle->conn_updt_blk.tx_win_offset = conn_interval_min >> 1;
            /* compute transmit window size =
               1.25ms ~ MIN(10 ms,(connInterval - 1.25ms)) */
            pHandle->conn_updt_blk.tx_win_size = MIN(8, conn_interval_min - 1);

#ifdef _DAPE_KEEP_SLOT_OFST_SAME_WHEN_CONN_UPDT
            pHandle->conn_updt_blk.tx_win_offset = 0;
            /* original is set to 1 (Atheros method). Change to
               MIN(8, conn_interval_min - 1) for safe. */
            pHandle->conn_updt_blk.tx_win_size = MIN(8, conn_interval_min - 1);
            pHandle->conn_updt_blk.tx_win_size_ofst = 0;
#endif
#ifdef _DAPE_TEST_CHG_LE_UPDT_WIN_OFST_FOR_CSR
            /* change for CSR */
            if (pHandle->conn_updt_blk.tx_win_offset >= pHandle->supervision_to)
            {
                pHandle->conn_updt_blk.tx_win_offset = pHandle->supervision_to >> 1;
            }
#endif
            errcode = HCI_COMMAND_SUCCEEDED;

            /* generate command status event */
            hci_generate_command_status_event(hci_cmd_ptr->cmd_opcode, errcode);
#ifdef _DAPE_TEST_SEND_CONN_UPT_COMPLETE_EVT_WHEN_HOST_REQ
            pHandle->hci_cmd_bm.conn_updt = 1;
#endif
#ifdef _BT4_1_SUPPORT_LL_CONN_PARAM_REQ
            if (IS_BT41)
            {
                pHandle->conn_param_req_blk.Interval_Min = conn_interval_min;
                pHandle->conn_param_req_blk.Interval_Max = conn_interval_max;
                pHandle->conn_param_req_blk.Latency = conn_latency;
                pHandle->conn_param_req_blk.Timeout = supervision_timeout;

                if (pHandle->llc.type != LLC_PROCEDURE_TYPE_NONE)
                {
                    /* need to wait after previous LLC procedure has completed */
                    pHandle->llc.pend_conn_param_req = TRUE;
                }
                else
                {
                    llc_schedule_connection_parameter_procedure(pHandle);
                }
            }
            else
#endif
            {
            /* TODO: prepare connection update procedure */

            if (pHandle->llc.type != LLC_PROCEDURE_TYPE_NONE)
            {
                /* need to wait after previous LLC procedure has completed */
                pHandle->llc.pend_conn_updt = TRUE;
            }
            else
            {
                llc_schedule_connection_update_procedure(pHandle);
            }
            }
            return errcode;
        }
        else
        {
            /* this code shall be re-name "UNKNOWN_CONNECTION_IDENTIFIER" */
            errcode = NO_CONNECTION_ERROR;
        }
    }
    while (0);

    /* generate command status event */
    hci_generate_command_status_event(hci_cmd_ptr->cmd_opcode, errcode);

    /* we need to create LE_Connection_Complete_Event when a connection is
       create or the connection creation procedure is cancelled */

    return errcode;
}

/**************************************************************************
 * Function     : hci_handle_le_set_host_channel_classification
 *
 * Description  : This function is used to handle HCI LE Set Host Channel
 *                Classification Command. Please refer 7.8.19 in HCI part of
 *                BT 4.0 spec.
 *
 * Parameters   : hci_cmd_ptr Pointer to the command packet.
 *
 * Returns      : HCI_COMMAND_SUCCEEDED or HCI error code
 *
 *************************************************************************/
UINT8 hci_handle_le_set_host_channel_classification(HCI_CMD_PKT *hci_cmd_ptr)
{
    UINT8 *pComPara = hci_cmd_ptr->cmd_parameter;
    UINT8 errcode;
    UINT8 i;
    LL_CONN_HANDLE_UNIT *pHandle;
    UINT8 n_min = 0;

    do
    {
        if ((LL_MULTI_STATES_SUPPORTED_BITMAP_LOW_WORD &
                (BIT18 | BIT19 | BIT24 | BIT25 | BIT28)) == 0)
        {
            /* we do not support master role */
            errcode = COMMAND_DISALLOWED_ERROR;
            break;
        }

        /*----------------------------------------------------*/
        /*         Parser the Command Parameters              */
        /*----------------------------------------------------*/
        /* Channel Bitmap (channel 32~36) - byte 4 */
        /* bit 37,38,39 are reserved and shall be zero */
        if (pComPara[4] & 0xE0)
        {
            errcode = INVALID_HCI_COMMAND_PARAMETERS_ERROR;
            break;
        }

        /* Check Number of unknown channels in this map */
        for (i = 0; i < LL_MAX_DATA_CHANNEL_NUMBER; i++)
        {
            if (pComPara[i >> 3] & BIT(i & 0x07))
            {
                n_min++;
            }
        }

        /* Check for length of parameters and given length in the command packet */
        /* Check or minimum number of channels in Map */
        /* Check Map does not fall in the reserved area of channels */
        if (n_min < LL_MIN_DATA_CHANNEL_NUMBER)
        {
            errcode = INVALID_HCI_COMMAND_PARAMETERS_ERROR;
            break;
        }

        /* copy updated channel bitmap from Host */
        memcpy(ll_manager.conn_unit.updt_ch_map, pComPara, 5);

        errcode = HCI_COMMAND_SUCCEEDED;
    }
    while (0);

    /* generate command complete event */
    UCHAR event_parameter[4];
    UINT8 *pCCE_returnPara = &event_parameter[3];
    UINT8 count = 0;

    hci_generate_le_command_complete_event_head(hci_cmd_ptr->cmd_opcode,
            event_parameter);
    pCCE_returnPara[0] = errcode;
    LL_HCI_GEN_COMMAND_COMPLETE_EVENT(event_parameter, 1);

    if (errcode != HCI_COMMAND_SUCCEEDED)
    {
        /* do not schedule le channel map update procedure */
        return errcode;
    }

    /* TODO: we need to create LE_Connection_Map_Req and send to every slaves */
    for (i = 0; i < LL_MAX_CONNECTION_UNITS; i++)
    {
        if (ll_manager.conn_unit.bmActiveHandle & (1 << i))
        {
            pHandle = &ll_manager.conn_unit.handle[i];
            if ((count == 0) && (pHandle->llc.type == LLC_PROCEDURE_TYPE_NONE))
            {
                llc_schedule_channel_map_update_procedure(pHandle);
                count++;
            }
            else
            {
                pHandle->llc.pend_ch_map_updt = TRUE;
            }
        }
    }

    return errcode;
}

/**************************************************************************
 * Function     : hci_handle_le_read_channel_map
 *
 * Description  : This function is used to handle HCI LE Read Channel Map
 *                Map Command. Please refer 7.8.20 in HCI part of BT 4.0 spec.
 *
 * Parameters   : hci_cmd_ptr Pointer to the command packet.
 *
 * Returns      : HCI_COMMAND_SUCCEEDED or HCI error code
 *
 *************************************************************************/
UINT8 hci_handle_le_read_channel_map(HCI_CMD_PKT *hci_cmd_ptr)
{
    UCHAR event_parameter[HCI_MAX_EVENT_PARAM_LEN];
    UINT16 connection_handle;
    UINT8 *pComPara = hci_cmd_ptr->cmd_parameter;
    UINT8 errcode = INVALID_HCI_COMMAND_PARAMETERS_ERROR;
    UINT8 *pCCE_returnPara = &event_parameter[3];
    UINT8 return_para_len = 1;

    do
    {
        /*----------------------------------------------------*/
        /*         Parser the Command Parameters              */
        /*----------------------------------------------------*/
        /* Connection_Handle - byte 0~1 */
        connection_handle = pComPara[0] | (pComPara[1] << 8);

        if (connection_handle > LL_HCI_MAX_CONNECTION_HANDLE)
        {
            break;
        }

        LL_CONN_HANDLE_UNIT *pHandle;
        pHandle = ll_fw_search_handle_unit_via_conn_handle(connection_handle);

        if (pHandle != NULL)
        {
            errcode = HCI_COMMAND_SUCCEEDED;

            /* fill byte 2~3 for connection_handle */
            pCCE_returnPara[1] = connection_handle;
            pCCE_returnPara[2] = connection_handle >> 8;

            /* fill byte 4~8 for Channel_Map */
#ifdef _DAPE_MOVE_LE_SLAVE_PARAMETER_TO_CAM
            UINT8 cam_entry = 0;
            if (ll_manager.conn_unit.master)
            {
                cam_entry = pHandle->unit_id;
            }
            UINT32 chmap_dw[2];
            UINT8 dw_addr = LE_CAM_ENTRY_BASE(cam_entry) +
                            LE_CAM_ADDR_CH_MAP;

            chmap_dw[0] = ll_driver_read_cam(dw_addr);
            chmap_dw[1] = ll_driver_read_cam(dw_addr + 1) & 0x1F;
            memcpy(pHandle->bmChMap, chmap_dw, 5);

#else
            /* fill byte 4~8 for Channel_Map */
            if (ll_manager.conn_unit.master)
            {
                UINT32 chmap_dw[2];
                UINT8 dw_addr = LE_CAM_ENTRY_BASE(pHandle->unit_id) +
                                LE_CAM_ADDR_CH_MAP;

                chmap_dw[0] = ll_driver_read_cam(dw_addr);
                chmap_dw[1] = ll_driver_read_cam(dw_addr + 1) & 0x1F;
                memcpy(pHandle->bmChMap, chmap_dw, 5);
            }
            else
            {
                UINT16 chmap_w[3];
                chmap_w[0] = RD_LE_REG(LE_REG_SLAVE_CH_MAP_L);
                chmap_w[1] = RD_LE_REG(LE_REG_SLAVE_CH_MAP_M);
                chmap_w[2] = RD_LE_REG(LE_REG_SLAVE_CH_MAP_H) & 0x1F;
                memcpy(pHandle->bmChMap, chmap_w, 5);
            }
#endif

            memcpy(&pCCE_returnPara[3], pHandle->bmChMap, 5);
            return_para_len += 7;
        }
        else
        {
            /* this code shall be re-name "UNKNOWN_CONNECTION_IDENTIFIER" */
            errcode = NO_CONNECTION_ERROR;
        }
    }
    while (0);

    /* generate command complete event */
    hci_generate_le_command_complete_event_head(hci_cmd_ptr->cmd_opcode,
            event_parameter);
    pCCE_returnPara[0] = errcode;
    LL_HCI_GEN_COMMAND_COMPLETE_EVENT(event_parameter, return_para_len);
    return errcode;
}

/**************************************************************************
 * Function     : hci_handle_le_read_remote_used_features
 *
 * Description  : This function is used to handle HCI LE Read Remote Used
 *                Features Command. Please refer 7.8.21 in HCI part of BT 4.0
 *                spec.
 *
 * Parameters   : hci_cmd_ptr Pointer to the command packet.
 *
 * Returns      : HCI_COMMAND_SUCCEEDED or HCI error code
 *
 *************************************************************************/
UINT8 hci_handle_le_read_remote_used_features(HCI_CMD_PKT *hci_cmd_ptr)
{
    UINT16 connection_handle;
    UINT8 *pComPara = hci_cmd_ptr->cmd_parameter;
    UINT8 errcode = INVALID_HCI_COMMAND_PARAMETERS_ERROR;
    LL_CONN_HANDLE_UNIT *pHandle;

    do
    {
#ifdef _SUPPORT_VER_4_1_
        if (!ll_manager.conn_unit.enable || ((!IS_BT41) && (!IS_BT42) && (!ll_manager.conn_unit.master)))
#else
        if (!ll_manager.conn_unit.enable || !ll_manager.conn_unit.master)
#endif
        {
            /* not setup a connection or we are not master role */
            errcode = COMMAND_DISALLOWED_ERROR;
            break;
        }

        /*----------------------------------------------------*/
        /*         Parser the Command Parameters              */
        /*----------------------------------------------------*/
        /* Connection_Handle - byte 0~1 */
        connection_handle = pComPara[0] | (pComPara[1] << 8);

        if (connection_handle > LL_HCI_MAX_CONNECTION_HANDLE)
        {
            break;
        }

        pHandle = ll_fw_search_handle_unit_via_conn_handle(connection_handle);

        if (pHandle != NULL)
        {
            errcode = HCI_COMMAND_SUCCEEDED;

            /* generate command status event */
            hci_generate_command_status_event(hci_cmd_ptr->cmd_opcode, errcode);
            pHandle->hci_cmd_bm.remote_feature = 1;

            /* TODO: we shall send a LE Read Remote Used Features Complete Event
               to the Host after the controller has completed the procedure to
               determine the remote features */

            if (pHandle->llc.type != LLC_PROCEDURE_TYPE_NONE)
            {
                /* we need to wait the completion of previous LLC procedure */
                pHandle->llc.pend_feature_ex = TRUE;
            }
            else
            {
                llc_schedule_feature_exchange_procedure(pHandle);
            }

            return errcode;
        }
        else
        {
            /* this code shall be re-name "UNKNOWN_CONNECTION_IDENTIFIER" */
            errcode = NO_CONNECTION_ERROR;
        }
    }
    while (0);

    /* generate command status event */
    hci_generate_command_status_event(hci_cmd_ptr->cmd_opcode, errcode);

    return errcode;
}

/**************************************************************************
 * Function     : hci_handle_le_encrypt
 *
 * Description  : This function is used to handle HCI LE Encrypt Command.
 *                Please refer 7.8.22 in HCI part of BT 4.0 spec.
 *
 * Parameters   : hci_cmd_ptr Pointer to the command packet.
 *
 * Returns      : HCI_COMMAND_SUCCEEDED or HCI error code
 *
 *************************************************************************/
UINT8 hci_handle_le_encrypt(HCI_CMD_PKT *hci_cmd_ptr)
{
    UCHAR event_parameter[20];
    UINT8 errcode = HCI_COMMAND_SUCCEEDED;
    UINT8 *pCCE_returnPara = &event_parameter[3];
    UINT16 key[8];
    UINT16 plaint_text[8];
    UINT16 encrypted_data[8];

    /*----------------------------------------------------*/
    /*         Parser the Command Parameters              */
    /*----------------------------------------------------*/
    /* Key - byte 0 ~ 15 */
    memcpy((UINT8*)key, hci_cmd_ptr->cmd_parameter, 16);

    /* Plaintext_Data - byte 16 ~ 31 */
    memcpy((UINT8*)plaint_text, hci_cmd_ptr->cmd_parameter + 16, 16);

    /* generate Encrypted_Data */
    ll_driver_get_session_key(key, plaint_text, encrypted_data);

    /* generate command complete event */
    hci_generate_le_command_complete_event_head(hci_cmd_ptr->cmd_opcode,
            event_parameter);
    pCCE_returnPara[0] = errcode;                            /* status field */
    memcpy(pCCE_returnPara + 1, (UINT8*)encrypted_data, 16); /* encrypted_data*/
    LL_HCI_GEN_COMMAND_COMPLETE_EVENT(event_parameter, 17);

    return errcode;
}

/**************************************************************************
 * Function     : hci_handle_le_rand
 *
 * Description  : This function is used to handle HCI LE Rand Command.
 *                Please refer 7.8.23 in HCI part of BT 4.0 spec.
 *
 * Parameters   : hci_cmd_ptr Pointer to the command packet.
 *
 * Returns      : HCI_COMMAND_SUCCEEDED or HCI error code
 *
 *************************************************************************/
UCHAR hci_handle_le_rand(HCI_CMD_PKT *hci_cmd_ptr)
{
    UCHAR event_parameter[12];
    UINT8 errcode = HCI_COMMAND_SUCCEEDED;
    UINT8 *pCCE_returnPara = &event_parameter[3];
    UINT32 random_number[2];

    /* generate 8 byte Random Number */
    LL_DRIVER_GEN_32BIT_RANDOM(random_number[0]);
    LL_DRIVER_GEN_32BIT_RANDOM(random_number[1]);

    /* generate command complete event */
    hci_generate_le_command_complete_event_head(hci_cmd_ptr->cmd_opcode,
            event_parameter);
    pCCE_returnPara[0] = errcode;                          /* status field */
    memcpy(pCCE_returnPara + 1, (UINT8*)random_number, 8); /* random_number */
    LL_HCI_GEN_COMMAND_COMPLETE_EVENT(event_parameter, 9);
    return errcode;
}

/**************************************************************************
 * Function     : hci_handle_le_start_encryption
 *
 * Description  : This function is used to handle HCI LE Start Encryption
 *                Command. Please refer 7.8.24 in HCI part of BT 4.0 spec.
 *
 * Parameters   : hci_cmd_ptr Pointer to the command packet.
 *
 * Returns      : HCI_COMMAND_SUCCEEDED or HCI error code
 *
 *************************************************************************/
UCHAR hci_handle_le_start_encryption(HCI_CMD_PKT *hci_cmd_ptr)
{
    UINT16 connection_handle;
    UINT8 *pComPara = hci_cmd_ptr->cmd_parameter;
    UINT8 errcode = INVALID_HCI_COMMAND_PARAMETERS_ERROR;
    LL_CONN_HANDLE_UNIT *pHandle;

    do
    {
        if (!ll_manager.conn_unit.enable || !ll_manager.conn_unit.master)
        {
            /* not setup a connection or we are not master role */
            errcode = COMMAND_DISALLOWED_ERROR;
            break;
        }

        /*----------------------------------------------------*/
        /*         Parser the Command Parameters              */
        /*----------------------------------------------------*/
        /* Connection_Handle - byte 0~1 */
        connection_handle = pComPara[0] | (pComPara[1] << 8);

        if (connection_handle > LL_HCI_MAX_CONNECTION_HANDLE)
        {
            break;
        }

        pHandle = ll_fw_search_handle_unit_via_conn_handle(connection_handle);

        if (pHandle != NULL)
        {
            errcode = HCI_COMMAND_SUCCEEDED;

            /* generate command status event */
            hci_generate_command_status_event(hci_cmd_ptr->cmd_opcode, errcode);

            /* copy random_number (8 bytes) + encrypted_diversifier (2 bytes) +
               long_term_key (16 bytes) to local */
            memcpy(pHandle->encrypt_blk.rand, &pComPara[2], 26);

            if (pHandle->llc.type != LLC_PROCEDURE_TYPE_NONE)
            {
                /* we need to wait the completion of previous LLC procedure */
                pHandle->llc.pend_start_encrypt = TRUE;
            }
            else
            {
                llc_schedule_encryption_procedure(pHandle);
#ifdef _TP_SEC_SLA_BI05C_LOWER_TESTER
                {
                    llc_schedule_version_exchange_procedure(pHandle);
                }
#endif
            }

            return errcode;
        }
        else
        {
            /* this code shall be re-name "UNKNOWN_CONNECTION_IDENTIFIER" */
            errcode = NO_CONNECTION_ERROR;
        }
    }
    while (0);

    /* generate command status event */
    hci_generate_command_status_event(hci_cmd_ptr->cmd_opcode, errcode);

    return errcode;
}

/**************************************************************************
 * Function     : hci_handle_le_long_term_key_request_reply
 *
 * Description  : This function is used to handle HCI LE Long Term Key Request
 *                Reply Command. Please refer 7.8.25 in HCI part of BT 4.0 spec.
 *
 * Parameters   : hci_cmd_ptr Pointer to the command packet.
 *
 * Returns      : HCI_COMMAND_SUCCEEDED or HCI error code
 *
 *************************************************************************/
UCHAR hci_handle_le_long_term_key_request_reply(HCI_CMD_PKT *hci_cmd_ptr)
{
    UCHAR event_parameter[6];
    UINT8 errcode = INVALID_HCI_COMMAND_PARAMETERS_ERROR;
    UINT8 *pCCE_returnPara = &event_parameter[3];
    UINT8 *pComPara = hci_cmd_ptr->cmd_parameter;
    UINT16 connection_handle;
    LL_ENCRYPT_BLK *pEnc_blk;

    do
    {
        /*----------------------------------------------------*/
        /*         Parser the Command Parameters              */
        /*----------------------------------------------------*/
        /* Connection_Handle - byte 0~1 */
        connection_handle = pComPara[0] | (pComPara[1] << 8);

        if (connection_handle > LL_HCI_MAX_CONNECTION_HANDLE)
        {
            break;
        }

        LL_CONN_HANDLE_UNIT *pHandle;
        pHandle = ll_fw_search_handle_unit_via_conn_handle(connection_handle);

        if (pHandle != NULL)
        {
            errcode = HCI_COMMAND_SUCCEEDED;

            /* long term key data - byte 2 ~ 17 */
            memcpy(pHandle->encrypt_blk.ltk, hci_cmd_ptr->cmd_parameter + 2, 16);

            if (LLC_START_ENC_STATE_BEGIN == pHandle->encrypt_blk.start_enc_state)
            {
                /* generate session key and install it to the cam */
                pEnc_blk = &pHandle->encrypt_blk;
                ll_driver_get_session_key((UINT16*)pEnc_blk->ltk,
                                        (UINT16*)pEnc_blk->skd_m,
                                        (UINT16*)pEnc_blk->sesskey);
            }
		     pHandle->long_term_key_got = 1;
        }
        else
        {
            /* this code shall be re-name "UNKNOWN_CONNECTION_IDENTIFIER" */
            errcode = NO_CONNECTION_ERROR;
        }
    }
    while (0);

    /* generate command complete event */
    hci_generate_le_command_complete_event_head(hci_cmd_ptr->cmd_opcode,
            event_parameter);
    pCCE_returnPara[0] = errcode;                    /* status field */
    pCCE_returnPara[1] = connection_handle & 0xFF;
    pCCE_returnPara[2] = connection_handle >> 8;
    LL_HCI_GEN_COMMAND_COMPLETE_EVENT(event_parameter, 3);
    return errcode;
}

/**************************************************************************
 * Function     : hci_handle_le_long_term_key_requested_negative_reply
 *
 * Description  : This function is used to handle HCI LE Long Term Key Requested
 *                Negative Reply Command. Please refer 7.8.26 in HCI part of
 *                BT 4.0 spec.
 *
 * Parameters   : hci_cmd_ptr Pointer to the command packet.
 *
 * Returns      : HCI_COMMAND_SUCCEEDED or HCI error code
 *
 *************************************************************************/
UCHAR hci_handle_le_long_term_key_requested_negative_reply(HCI_CMD_PKT *hci_cmd_ptr)
{
    UCHAR event_parameter[6];
    UINT8 errcode = INVALID_HCI_COMMAND_PARAMETERS_ERROR;
    UINT8 *pCCE_returnPara = &event_parameter[3];
    UINT8 *pComPara = hci_cmd_ptr->cmd_parameter;
    UINT16 connection_handle;
    LL_CTRL_PDU_PAYLOAD *pTxPkt;

    do
    {
        /*----------------------------------------------------*/
        /*         Parser the Command Parameters              */
        /*----------------------------------------------------*/
        /* Connection_Handle - byte 0~1 */
        connection_handle = pComPara[0] | (pComPara[1] << 8);

        if (connection_handle > LL_HCI_MAX_CONNECTION_HANDLE)
        {
            break;
        }

        LL_CONN_HANDLE_UNIT *pHandle;
        pHandle = ll_fw_search_handle_unit_via_conn_handle(connection_handle);

        if (pHandle != NULL)
        {
            /* host can not provide a long term key for this handle */
            errcode = HCI_COMMAND_SUCCEEDED;

            if (LLC_START_ENC_STATE_BEGIN == pHandle->encrypt_blk.start_enc_state)
            {
                /* fix correct long term key request negative reply handle */
                if (pHandle->encrypt_blk.pause_enc_state == LLC_PAUSE_ENC_STATE_END)
                {
                    /* send ll_terminate pdu to remote device */
                    ll_handle_acl_disconnect(pHandle->conn_handle,
                                             KEY_MISSING_ERROR, FALSE);
                }
                else
                {
                    /* send LL_REJECT_IND to remote device */
#ifdef _TP_SEC_MAS_BV11C_LOWER_TESTER
                    pTxPkt = llc_generate_ll_reject_ind_ext(LL_LENGTH_REQ, KEY_MISSING_ERROR);

#else
                    pTxPkt = llc_generate_ll_reject_ind(KEY_MISSING_ERROR);
#endif
                    llc_append_pdu_to_tx_list(pTxPkt, pHandle->unit_id);
#ifdef _TP_SEC_MAS_BV11C_LOWER_TESTER
                    llc_check_sent_llc_pdu_request(pHandle, LL_REJECT_IND_EXT);
#else
                    llc_check_sent_llc_pdu_request(pHandle, LL_REJECT_IND);
#endif
                }
                pHandle->encrypt_blk.start_enc_state = LLC_START_ENC_STATE_IDLE;
            }
        }
        else
        {
            /* this code shall be re-name "UNKNOWN_CONNECTION_IDENTIFIER" */
            errcode = NO_CONNECTION_ERROR;
        }
    }
    while (0);

    /* generate command complete event */
    hci_generate_le_command_complete_event_head(hci_cmd_ptr->cmd_opcode,
            event_parameter);
    pCCE_returnPara[0] = errcode;                    /* status field */
    pCCE_returnPara[1] = connection_handle & 0xFF;
    pCCE_returnPara[2] = connection_handle >> 8;
    LL_HCI_GEN_COMMAND_COMPLETE_EVENT(event_parameter, 3);
    return errcode;
}

/**************************************************************************
 * Function     : hci_handle_le_read_supported_states
 *
 * Description  : This function is used to handle HCI LE Read Supported States
 *                Command. Please refer 7.8.27 in HCI part of BT 4.0 spec.
 *
 * Parameters   : hci_cmd_ptr Pointer to the command packet.
 *
 * Returns      : HCI_COMMAND_SUCCEEDED or HCI error code
 *
 *************************************************************************/
UCHAR hci_handle_le_read_supported_states(HCI_CMD_PKT *hci_cmd_ptr)
{
    UCHAR event_parameter[12];
    UINT8 *pCCE_returnPara = &event_parameter[3];
    UINT32 multistate_ldw = LL_MULTI_STATES_SUPPORTED_BITMAP_LOW_WORD;
    UINT32 multistate_hdw = LL_MULTI_STATES_SUPPORTED_BITMAP_HIGH_WORD;

    hci_generate_le_command_complete_event_head(hci_cmd_ptr->cmd_opcode,
            event_parameter);

    /* status field - byte 0 */
    pCCE_returnPara[0] = HCI_COMMAND_SUCCEEDED;

    /* LE_States - byte 1 ~ 8 */
    memcpy(pCCE_returnPara + 1, (UINT8*)&multistate_ldw, 4);
    memcpy(pCCE_returnPara + 5, (UINT8*)&multistate_hdw, 4);

    LL_HCI_GEN_COMMAND_COMPLETE_EVENT(event_parameter, 9);

    return HCI_COMMAND_SUCCEEDED;
}

/**************************************************************************
 * Function     : hci_handle_le_receiver_test
 *
 * Description  : This function is used to handle HCI LE Receiver Test Command.
 *                Please refer 7.8.28 in HCI part of BT 4.0 spec.
 *
 * Parameters   : hci_cmd_ptr Pointer to the command packet.
 *
 * Returns      : HCI_COMMAND_SUCCEEDED or HCI error code
 *
 *************************************************************************/
UCHAR hci_handle_le_receiver_test(HCI_CMD_PKT *hci_cmd_ptr)
{
#ifdef _BT5_0_LE2MBPS_SUPPORT_
    UINT8 errcode = INVALID_HCI_COMMAND_PARAMETERS_ERROR;
    errcode = hci_handle_le_receiver_test_general(hci_cmd_ptr, 0);
#else
    UCHAR event_parameter[4];
    UINT8 errcode = INVALID_HCI_COMMAND_PARAMETERS_ERROR;
    UINT8 *pCCE_returnPara = &event_parameter[3];
    UINT8 rx_frequency;
    LE_REG_S_SET reg;

    /*----------------------------------------------------*/
    /*         Parser the Command Parameters              */
    /*----------------------------------------------------*/
    /* RX_Frequency - byte 0 */
    rx_frequency = hci_cmd_ptr->cmd_parameter[0];

    if (rx_frequency <= LL_MAX_RF_CHANNEL_INDEX)
    {
        ll_manager.cur_mode = LL_MODE_RX_TEST;
        ll_manager.test_unit.rf_channel = rx_frequency;
        ll_manager.test_unit.num_of_rx_pkts = 0;
        ll_manager.test_unit.num_of_rx_pkts_crc_err = 0;
        errcode = HCI_COMMAND_SUCCEEDED;

        /* disable whitenning and keep duplicated rx packet */
        reg.value = RD_LE_REG(LE_REG_RX_PKT_ERR_CTRL);
        reg.rx_pkt_err_ctrl.whiten_dis = 1;
        reg.rx_pkt_err_ctrl.pkt_dup_kept = 1;
        reg.rx_pkt_err_ctrl.crc_err_kept = 1;
        WR_LE_REG(LE_REG_RX_PKT_ERR_CTRL, reg.value);

        /* configure RF Channel */
        reg.value = RD_LE_REG(LE_REG_MODEM_CONTROL);
        reg.modem_ctrl.test_mode_rf_ch = rx_frequency;
#ifdef _BT5_0_LE2MBPS_SUPPORT_
        reg.modem_ctrl.TM_Midx = 0;
        reg.modem_ctrl.TM_PhyRate = 0;
#endif
        WR_LE_REG(LE_REG_MODEM_CONTROL, reg.value);

#ifdef _DAPE_SUPPORT_CHG_AA_IN_LE_TEST_MODE
        if (ll_manager.test_unit.chg_aa)
        {
            WR_LE_REG(LE_REG_TEST_MODE_ACCESS_ADDR_L, ll_manager.test_unit.access_addr_l);
            WR_LE_REG(LE_REG_TEST_MODE_ACCESS_ADDR_H, ll_manager.test_unit.access_addr_h);
        }
        else
        {
            /* as spec. defines (p.2628/2684) LE test packets shall have '10010100100000100110111010001110'
            (in transmission order) as the synchronization word.*/
            WR_LE_REG(LE_REG_TEST_MODE_ACCESS_ADDR_L, 0x4129);
            WR_LE_REG(LE_REG_TEST_MODE_ACCESS_ADDR_H, 0x7176);
        }
#endif

        /* configure LE Receiver Test mode */
        reg.value = RD_LE_REG(LE_REG_RF_TEST_CONTROL);
        reg.rf_test_ctrl.rf_test_rx = TRUE;
        reg.rf_test_ctrl.rf_test_mode_en =  TRUE;
        WR_LE_REG(LE_REG_RF_TEST_CONTROL, reg.value);
    }

    LL_LOG_TRACE(GRAY, HCI_LE_RX_TEST_PARAM, 2, rx_frequency, errcode);

    /* generate command complete event */
    hci_generate_le_command_complete_event_head(hci_cmd_ptr->cmd_opcode,
            event_parameter);
    pCCE_returnPara[0] = errcode;  /* status field */
    LL_HCI_GEN_COMMAND_COMPLETE_EVENT(event_parameter, 1);
#endif    
    return errcode;
}

/**************************************************************************
 * Function     : hci_handle_le_transmitter_test
 *
 * Description  : This function is used to handle HCI LE Transmitter Test
 *                Command. Please refer 7.8.29 in HCI part of BT 4.0 spec.
 *
 * Parameters   : hci_cmd_ptr Pointer to the command packet.
 *
 * Returns      : HCI_COMMAND_SUCCEEDED or HCI error code
 *
 *************************************************************************/
UCHAR hci_handle_le_transmitter_test(HCI_CMD_PKT *hci_cmd_ptr)
{
#ifdef _BT5_0_LE2MBPS_SUPPORT_
    UINT8 errcode = INVALID_HCI_COMMAND_PARAMETERS_ERROR;
    errcode = hci_handle_le_transmitter_test_general(hci_cmd_ptr, 0);
#else
    UCHAR event_parameter[4];
    UINT8 errcode = INVALID_HCI_COMMAND_PARAMETERS_ERROR;
    UINT8 *pCCE_returnPara = &event_parameter[3];
    UINT8 tx_frequency = 0xFF;
    UINT8 length_of_test_data = 0xFF;
    UINT8 packet_payload = 0xFF;
    UINT16 opcode = hci_cmd_ptr->cmd_opcode;
    LE_REG_S_SET reg;

    do
    {
        /*----------------------------------------------------*/
        /*         Parser the Command Parameters              */
        /*----------------------------------------------------*/
        /* TX_Frequency - byte 0 */
        tx_frequency = hci_cmd_ptr->cmd_parameter[0];
        if (tx_frequency > LL_MAX_RF_CHANNEL_INDEX)
        {
            break;
        }

        /* Length_Of_Test_Data - byte 1 */
        length_of_test_data = hci_cmd_ptr->cmd_parameter[1];
        if (length_of_test_data > LL_MAX_TEST_DATA_LEN)
        {
            break;
        }

        /* Packet_Payload - byte 2 */
        packet_payload = hci_cmd_ptr->cmd_parameter[2];
        if (packet_payload > LL_TEST_PKT_TYPE_MAX)
        {
            break;
        }

#ifdef LE_TX_TEST_MODE_WORKAROUND_CHANGE_TX_TYPE
        /* because hw can keep rx state to let transmit nothing,
           we need to reset it via software reset in workaround way */
        hci_handle_reset_command();
#endif

        ll_manager.cur_mode = LL_MODE_TX_TEST;
        ll_manager.test_unit.rf_channel = tx_frequency;
        ll_manager.test_unit.payload_len = length_of_test_data;
        ll_manager.test_unit.payload_type = packet_payload;
        ll_manager.test_unit.num_of_rx_pkts = 0;
        ll_manager.test_unit.num_of_rx_pkts_crc_err = 0;
        errcode = HCI_COMMAND_SUCCEEDED;
#ifdef _DAPE_SUPPORT_LE_TX_PKT_MODE
        reg.value = RD_LE_REG(LE_REG_TX_TEST_CNT_MODE);
        reg.le_tx_test_cnt_mode_ctrl.counter_mode_en = ll_manager.test_unit.tx_cnt_mode_en;
        reg.le_tx_test_cnt_mode_ctrl.tx_pkt_cnt = ll_manager.test_unit.tx_pkt_cnt;
        WR_LE_REG(LE_REG_TX_TEST_CNT_MODE, reg.value);
#endif
#ifdef _DAPE_SUPPORT_CHG_AA_IN_LE_TEST_MODE
        if (ll_manager.test_unit.chg_aa)
        {
            WR_LE_REG(LE_REG_TEST_MODE_ACCESS_ADDR_L, ll_manager.test_unit.access_addr_l);
            WR_LE_REG(LE_REG_TEST_MODE_ACCESS_ADDR_H, ll_manager.test_unit.access_addr_h);
        }
        else
        {
            /* as spec. defines (p.2628/2684) LE test packets shall have '10010100100000100110111010001110'
            (in transmission order) as the synchronization word.*/
            WR_LE_REG(LE_REG_TEST_MODE_ACCESS_ADDR_L, 0x4129);
            WR_LE_REG(LE_REG_TEST_MODE_ACCESS_ADDR_H, 0x7176);
        }
#endif
        /* disable whitenning */
        reg.value = RD_LE_REG(LE_REG_RX_PKT_ERR_CTRL);
        reg.rx_pkt_err_ctrl.whiten_dis = 1;
        WR_LE_REG(LE_REG_RX_PKT_ERR_CTRL, reg.value);

        /* configure RF Channel */
        reg.value = RD_LE_REG(LE_REG_MODEM_CONTROL);
        reg.modem_ctrl.test_mode_rf_ch = tx_frequency;
#ifdef _BT5_0_LE2MBPS_SUPPORT_
        reg.modem_ctrl.TM_PhyRate = 0;
#endif
        WR_LE_REG(LE_REG_MODEM_CONTROL, reg.value);

        /* configure LE Transmitter Test mode, payload type, payload length */
        reg.value = RD_LE_REG(LE_REG_RF_TEST_CONTROL);
        reg.rf_test_ctrl.rf_test_rx = FALSE;
        reg.rf_test_ctrl.rf_test_mode_en = TRUE;
        reg.rf_test_ctrl.prbs_mode = TRUE;
#ifdef _DAPE_SUPPORT_LE_TEST_MODE_TX_RANDOM
        /* Originally this is 0. If we want to change the value we change by vendor command. */
        reg.rf_test_ctrl.prbs_fix = ll_manager.test_unit.prbs_fix;
#endif
        reg.rf_test_ctrl.payload_len = length_of_test_data;
        reg.rf_test_ctrl.payload_type = packet_payload;
        WR_LE_REG(LE_REG_RF_TEST_CONTROL, reg.value);
    }
    while (0);

    LL_LOG_TRACE(GRAY, HCI_LE_TX_TEST_PARAM, 4, tx_frequency,
                 length_of_test_data, packet_payload, errcode);

    /* generate command complete event */
    hci_generate_le_command_complete_event_head(opcode, event_parameter);
    pCCE_returnPara[0] = errcode;  /* status field */
    LL_HCI_GEN_COMMAND_COMPLETE_EVENT(event_parameter, 1);
#endif    
    return errcode;
}

/**************************************************************************
 * Function     : hci_handle_le_test_end
 *
 * Description  : This function is used to handle HCI LE Test End Command.
 *                Please refer 7.8.30 in HCI part of BT 4.0 spec.
 *
 * Parameters   : hci_cmd_ptr Pointer to the command packet.
 *
 * Returns      : HCI_COMMAND_SUCCEEDED or HCI error code
 *
 *************************************************************************/
UCHAR hci_handle_le_test_end(HCI_CMD_PKT *hci_cmd_ptr)
{
    UCHAR event_parameter[6];
    UINT8 errcode = HCI_COMMAND_SUCCEEDED;
    UINT8 *pCCE_returnPara = &event_parameter[3];
    UINT16 number_of_packets = 0; /* default value for transmitter test mode */
    UINT16 number_of_tx_packets = 0;
    UINT16 number_of_packets_crc_err = 0;
    LE_REG_S_SET reg;
    UINT8 current_mode;

    current_mode = ll_manager.cur_mode;

    if (ll_manager.cur_mode == LL_MODE_TX_TEST)
    {
        reg.value = RD_LE_REG(LE_REG_NUM_OF_TX_PKT_IN_RF_TEST);
        number_of_tx_packets = reg.value;
#ifdef _DAPE_SUPPORT_LE_TX_PKT_MODE
        if (ll_manager.test_unit.tx_cnt_mode_en)
        {
            reg.value = RD_LE_REG(LE_REG_TX_TEST_CNT_MODE);

            ll_manager.test_unit.pkt_cnt_tx_busy = reg.le_tx_test_cnt_mode_ctrl.pkt_cnt_tx_busy;
            if (ll_manager.test_unit.pkt_cnt_tx_busy)
            {
                RT_BT_LOG(RED, DAPE_TEST_LOG583, 2, ll_manager.test_unit.pkt_cnt_tx_busy, reg.value);
            }
        }
#endif

    }
    else
    {
        number_of_packets = ll_manager.test_unit.num_of_rx_pkts;
        number_of_packets_crc_err = ll_manager.test_unit.num_of_rx_pkts_crc_err;
    }

    ll_manager.cur_mode = LL_MODE_NORMAL;
    /* (20141208) dape mark this operation to make others can read this value until LE test start next time. */
    //ll_manager.test_unit.num_of_rx_pkts_crc_err = 0;

    /* Enable whitenning and dont keep duplicate rx pkt */
    reg.value = RD_LE_REG(LE_REG_RX_PKT_ERR_CTRL);
    reg.rx_pkt_err_ctrl.whiten_dis = 0;
    reg.rx_pkt_err_ctrl.pkt_dup_kept = 0;
    reg.rx_pkt_err_ctrl.crc_err_kept = 0;
    WR_LE_REG(LE_REG_RX_PKT_ERR_CTRL, reg.value);

    /* Exit LE RF Test mode */
    reg.value = RD_LE_REG(LE_REG_RF_TEST_CONTROL);
    reg.rf_test_ctrl.rf_test_mode_en = FALSE;
    WR_LE_REG(LE_REG_RF_TEST_CONTROL, reg.value);

    LL_LOG_TRACE(GRAY, HCI_LE_TEST_END_PARAM_CRC_ERR, 5,
                            current_mode, errcode,
                            number_of_packets, number_of_tx_packets,
                            number_of_packets_crc_err);

    /* generate command complete event */
    hci_generate_le_command_complete_event_head(hci_cmd_ptr->cmd_opcode,
            event_parameter);
    pCCE_returnPara[0] = errcode;                   /* status field */
    pCCE_returnPara[1] = number_of_packets & 0xFF;  /* number_of_packets */
    pCCE_returnPara[2] = number_of_packets >> 8;    /* number_of_packets */
    LL_HCI_GEN_COMMAND_COMPLETE_EVENT(event_parameter, 3);
    return errcode;
}

#ifdef _BT4_2_DATA_EXTENSION_FEATURE_SUPPORT_
UINT8 hci_handle_le_set_data_length(HCI_CMD_PKT *hci_cmd)
{
    UINT8 status;
    BOOLEAN need_schedule_data_len_update = TRUE;
    HCI_LE_SET_DATA_LENGTH_CMD_PARAM *cmd_param =
            (HCI_LE_SET_DATA_LENGTH_CMD_PARAM *) hci_cmd->cmd_parameter;
    UINT16 conn_handle = letoh16(cmd_param->conn_handle);
    UINT16 tx_size = letoh16(cmd_param->tx_octets);
    UINT16 tx_time = letoh16(cmd_param->tx_time);

    LL_CONN_HANDLE_UNIT *chu = ll_fw_search_handle_unit_via_conn_handle(
            conn_handle);

    do {
        if (chu == NULL)
        {
            status = NO_CONNECTION_ERROR;
            break;
        }

#ifdef _BT4_2_DLE_CHECK_REMOTE_FEATURE_
        if (!chu->support_dle)
        {
            /* skip this flow if remote device and local device both are not
               support data length extension (austin) */
            status = UNSUPPORTED_REMOTE_FEATURE_ERROR;
            break;
        }
#endif

        if (!ll_fw_validate_ll_length_param(tx_size, tx_time))
        {
            status = INVALID_HCI_COMMAND_PARAMETERS_ERROR;
            break;
        }

        if (chu->llc.type != LLC_PROCEDURE_TYPE_NONE)
        {
            /* If more than one length update request, we just pend one. */
            chu->llc.pend_data_len_updt = TRUE;
            need_schedule_data_len_update = FALSE;
        }

        UINT16 old_tx_size = chu->data_len_updt.local_max_tx_size;
        UINT16 old_tx_time = chu->data_len_updt.local_max_tx_time;
        chu->data_len_updt.local_max_tx_size = tx_size;
        chu->data_len_updt.local_max_tx_time = tx_time;
        if (need_schedule_data_len_update
                && (old_tx_size != tx_size || old_tx_time != tx_time))
        {
            llc_schedule_data_length_update_procedure(chu,
                    LLC_DATA_LEN_UPDATE_STATE_START_REQ);
        }
        status = HCI_COMMAND_SUCCEEDED;
    }
    while (0);

    hci_generate_command_complete_event(HCI_LE_SET_DATA_LENGTH_OPCODE, status,
            conn_handle, NULL);
    return status;
}

UINT8 hci_handle_le_read_default_data_length(HCI_CMD_PKT *hci_cmd)
{
    hci_generate_command_complete_event(HCI_LE_READ_DEFAULT_DATA_LENGTH_OPCODE,
            HCI_COMMAND_SUCCEEDED, 0, NULL);
    return HCI_COMMAND_SUCCEEDED;
}

UINT8 hci_handle_le_write_default_data_length(HCI_CMD_PKT *hci_cmd)
{
    UINT8 status;
    HCI_LE_WRITE_DEFAULT_DATA_LENGTH_CMD_PARAM *cmd_param =
            (HCI_LE_WRITE_DEFAULT_DATA_LENGTH_CMD_PARAM *) hci_cmd->cmd_parameter;
    UINT16 conn_init_max_tx_size = letoh16(cmd_param->tx_octets);
    UINT16 conn_init_max_tx_time = letoh16(cmd_param->tx_time);
    if (!ll_fw_validate_ll_length_param(conn_init_max_tx_size,
            conn_init_max_tx_time))
    {
        status = INVALID_HCI_COMMAND_PARAMETERS_ERROR;
        goto gen_hci_event;
    }

    ll_manager.conn_unit.init_max_tx_size = conn_init_max_tx_size;
    ll_manager.conn_unit.init_max_tx_time = conn_init_max_tx_time;
    status = HCI_COMMAND_SUCCEEDED;

gen_hci_event:
    hci_generate_command_complete_event(HCI_LE_WRITE_DEFAULT_DATA_LENGTH_OPCODE,
            status, 0, NULL);
    return status;
}

UINT8 hci_handle_le_read_maximum_data_length(HCI_CMD_PKT *hci_cmd)
{
    hci_generate_command_complete_event(HCI_LE_READ_MAXIMUM_DATA_LENGTH_OPCODE,
            HCI_COMMAND_SUCCEEDED, 0, NULL);
    return HCI_COMMAND_SUCCEEDED;
}

#define HCI_LE_SET_DATA_LENGTH_HDLR hci_handle_le_set_data_length
#define HCI_LE_READ_DEFAULT_DATA_LENGTH_HDLR hci_handle_le_read_default_data_length
#define HCI_LE_WRITE_DEFAULT_DATA_LENGTH_HDLR hci_handle_le_write_default_data_length
#define HCI_LE_READ_MAXIMUM_DATA_LENGTH hci_handle_le_read_maximum_data_length
#else
#define HCI_LE_SET_DATA_LENGTH_HDLR hci_handle_le_invalid_command
#define HCI_LE_READ_DEFAULT_DATA_LENGTH_HDLR hci_handle_le_invalid_command
#define HCI_LE_WRITE_DEFAULT_DATA_LENGTH_HDLR hci_handle_le_invalid_command
#define HCI_LE_READ_MAXIMUM_DATA_LENGTH hci_handle_le_invalid_command
#endif /* _BT4_2_DATA_EXTENSION_FEATURE_SUPPORT_ */

#ifdef _BT4_2_PRIVACY_FEATURE_SUPPORT_

static inline void hci_handle_dump_resolving_list(void)
{
#ifdef _BT4_2_PRIVACY_FEATURE_SUPPORT_MSG_
    int i = 0;
    LL_RESOLVING_LIST_ENTRY *list = ll_manager.resolving_list.item;
    for (i = 0;i < LL_MAX_RESOLVING_LIST_SIZE;i ++)
    {
        UINT32 *localIRK = list[i].Local_IRK;
        UINT32 *peerIRK = list[i].Peer_IRK;
        UINT16 *addr = (UINT16 *)list[i].addr;
        UINT8 type = list[i].type;

        if (list[i].valid)
        {
            LL_LOG_TRACE(YELLOW, MSG_RESOLVING_LIST_DUMP, 14,
                i, 0xdeadbeef, localIRK[3], localIRK[2], localIRK[1], localIRK[0], peerIRK[3], peerIRK[2], peerIRK[1], peerIRK[0], addr[2], addr[1], addr[0], type);
        }
    }
#endif /* _BT4_2_PRIVACY_FEATURE_SUPPORT_MSG_ */

    ll_driver_dump_hw_resolving_list();
}

/**************************************************************************
 * Function     : hci_handle_le_add_device_to_resolving_list_command
 *
 * Description  : This function is used to handle HCI Add Device to Resolving
 *                List Command. Please refer 7.8.38 in HCI part of BT 4.2 spec.
 *
 * Parameters   : hci_cmd_ptr Pointer to the command packet.
 *
 * Returns      : HCI_COMMAND_SUCCEEDED or HCI error code
 *
 *************************************************************************/
UINT8 hci_handle_le_add_device_to_resolving_list_command(HCI_CMD_PKT *hci_cmd_ptr)
{
    UINT8 errcode = COMMAND_DISALLOWED_ERROR;
    UINT8 *cmd_para = hci_cmd_ptr->cmd_parameter;
    UINT8 list_entry;

    do
    {
        if (!ll_manager.support_ll_privacy)
        {
            errcode = UNSUPPORTED_REMOTE_FEATURE_ERROR;
            break;
        }

        if (cmd_para[0] > 0x01)
        {
            /* invalid address type */
            errcode = INVALID_HCI_COMMAND_PARAMETERS_ERROR;
            break;
        }
        else if (cmd_para[0] == 0x1)
        {
            if ((cmd_para[6] & 0xC0) != 0xC0)
            {
                /* invalid address (should be static address) */
                errcode = INVALID_HCI_COMMAND_PARAMETERS_ERROR;
                break;
            }
        }

        if (ll_manager.address_resolution_enable && (ll_manager.adv_unit.enable || ll_manager.scan_unit.enable || ll_manager.initiator_unit.enable))
        {
            /* command cannot be used when address translation is enabled and
             * - Advertising is enabled
             * - Scanning is enabled
             * - Create connection command is outstanding
             */
            break;
        }

        UINT8 addr_type = cmd_para[0];
        UINT8 *addr = &cmd_para[1];
        if (ll_driver_search_dev_from_list(LL_RESOLVING_LIST_TYPE, addr_type,
                                           addr) >= LL_MAX_RESOLVING_LIST_SIZE)
        {
            UINT8 *peer_irk = &cmd_para[7];
            UINT8 *local_irk = &cmd_para[23];

            list_entry = ll_driver_add_dev_to_list(LL_RESOLVING_LIST_TYPE,
                                                   addr_type, addr, local_irk, peer_irk);

            if (LL_MAX_RESOLVING_LIST_SIZE == list_entry)
            {
                /* When a Controller cannot add a device to the resolving list
                 * because the list is full,
                 * it shall respond with error code 0x07 (Memory Capacity Exceeded).
                 */
                errcode = MEMORY_FULL_ERROR;
                break;
            }
        }

        hci_handle_dump_resolving_list();

        errcode = HCI_COMMAND_SUCCEEDED;
    }
    while (0);

    /* generate command complete event */
    UCHAR event_parameter[4];
    UINT8 *pCCE_returnPara = &event_parameter[3];
    hci_generate_le_command_complete_event_head(hci_cmd_ptr->cmd_opcode, event_parameter);
    pCCE_returnPara[0] = errcode;
    LL_HCI_GEN_COMMAND_COMPLETE_EVENT(event_parameter, 1);

    return errcode;
}

/**************************************************************************
 * Function     : hci_handle_le_remove_device_from_resolving_list_command
 *
 * Description  : This function is used to handle HCI LE Remote Device From
 *                Resolving List Command. Please refer 7.8.39 in HCI part of BT 4.2
 *                spec.
 *
 * Parameters   : hci_cmd_ptr Pointer to the command packet.
 *
 * Returns      : HCI_COMMAND_SUCCEEDED or HCI error code
 *
 *************************************************************************/
UINT8 hci_handle_le_remove_device_from_resolving_list_command(HCI_CMD_PKT *hci_cmd_ptr)
{
    UINT8 errcode = COMMAND_DISALLOWED_ERROR;
    UINT8 addr_type;
    BOOLEAN is_found = FALSE;

    do
    {
        if (!ll_manager.support_ll_privacy)
        {
            errcode = UNSUPPORTED_REMOTE_FEATURE_ERROR;
            break;
        }

        addr_type = hci_cmd_ptr->cmd_parameter[0];
        if (addr_type > 0x01)
        {
            /* invalid address type */
            errcode = INVALID_HCI_COMMAND_PARAMETERS_ERROR;
            break;
        }

        if (ll_manager.address_resolution_enable && (ll_manager.adv_unit.enable || ll_manager.scan_unit.enable || ll_manager.initiator_unit.enable))
        {
            /* command cannot be used when address translation is enabled and
             * - Advertising is enabled
             * - Scanning is enabled
             * - Create connection command is outstanding
             */
            break;
        }

        /* begin to remove device from resolving list */
        is_found = ll_driver_remove_dev_from_list(LL_RESOLVING_LIST_TYPE,
                   addr_type, &hci_cmd_ptr->cmd_parameter[1]);

        if (is_found)
            errcode = HCI_COMMAND_SUCCEEDED;
        else
            errcode = NO_CONNECTION_ERROR;

        hci_handle_dump_resolving_list();
    }
    while (0);

    /* generate command complete event */
    UCHAR event_parameter[4];
    UINT8 *pCCE_returnPara = &event_parameter[3];
    hci_generate_le_command_complete_event_head(hci_cmd_ptr->cmd_opcode, event_parameter);
    pCCE_returnPara[0] = errcode;
    LL_HCI_GEN_COMMAND_COMPLETE_EVENT(event_parameter, 1);

    return errcode;

}


/**************************************************************************
 * Function     : hci_handle_le_clear_resolving_list_command
 *
 * Description  : This function is used to handle HCI LE Clear Resolving List
 *                Command. Please refer 7.8.40 in HCI part of BT 4.2 spec.
 *
 * Parameters   : hci_cmd_ptr Pointer to the command packet.
 *
 * Returns      : HCI_COMMAND_SUCCEEDED or HCI error code
 *
 *************************************************************************/
UINT8 hci_handle_le_clear_resolving_list_command(HCI_CMD_PKT *hci_cmd_ptr)
{
    UINT8 errcode = COMMAND_DISALLOWED_ERROR;

    do
    {
        if (!ll_manager.support_ll_privacy)
        {
            errcode = UNSUPPORTED_REMOTE_FEATURE_ERROR;
            break;
        }

        if (ll_manager.address_resolution_enable && (ll_manager.adv_unit.enable || ll_manager.scan_unit.enable || ll_manager.initiator_unit.enable))
        {
            /* command cannot be used when address translation is enabled and
             * - Advertising is enabled
             * - Scanning is enabled
             * - Create connection command is outstanding
             */
            break;
        }

        /* clear resolving list */
        ll_driver_reset_dev_list(LL_RESOLVING_LIST_TYPE);

        hci_handle_dump_resolving_list();

        errcode = HCI_COMMAND_SUCCEEDED;
    }
    while (0);

    /* generate command complete event */
    UCHAR event_parameter[4];
    UINT8 *pCCE_returnPara = &event_parameter[3];
    hci_generate_le_command_complete_event_head(hci_cmd_ptr->cmd_opcode, event_parameter);
    pCCE_returnPara[0] = errcode;
    LL_HCI_GEN_COMMAND_COMPLETE_EVENT(event_parameter, 1);

    return errcode;
}

/**************************************************************************
 * Function     : hci_handle_le_read_resolving_list_size_command
 *
 * Description  : This function is used to handle HCI LE Read Resolving List Size
 *                Command. Please refer 7.8.41 in HCI part of BT 4.2 spec.
 *
 * Parameters   : hci_cmd_ptr Pointer to the command packet.
 *
 * Returns      : HCI_COMMAND_SUCCEEDED or HCI error code
 *
 *************************************************************************/
UINT8 hci_handle_le_read_resolving_list_size_command(HCI_CMD_PKT *hci_cmd_ptr)
{
    UCHAR event_parameter[5];
    UINT8 *pCCE_returnPara = &event_parameter[3];

    hci_generate_le_command_complete_event_head(hci_cmd_ptr->cmd_opcode, event_parameter);

    /* status field - byte 0 */
    if (ll_manager.support_ll_privacy)
    {
        pCCE_returnPara[0] = HCI_COMMAND_SUCCEEDED;
    }
    else
    {
        pCCE_returnPara[0] = UNSUPPORTED_REMOTE_FEATURE_ERROR;
    }

    /* resolving_list_size - byte 1 */
    pCCE_returnPara[1] = LL_MAX_RESOLVING_LIST_SIZE;

    LL_HCI_GEN_COMMAND_COMPLETE_EVENT(event_parameter, 2);

    return HCI_COMMAND_SUCCEEDED;
}

/**************************************************************************
 * Function     : hci_handle_le_read_peer_resolvable_address_command
 *
 * Description  : This function is used to handle HCI LE Read Peer Resolvable
 *                Address Command. Please refer 7.8.42 in HCI part of BT 4.2 spec.
 *
 * Parameters   : hci_cmd_ptr Pointer to the command packet.
 *
 * Returns      : HCI_COMMAND_SUCCEEDED or HCI error code
 *
 *************************************************************************/
UINT8 hci_handle_le_read_peer_resolvable_address_command(HCI_CMD_PKT *hci_cmd_ptr)
{
    UINT8 errcode = COMMAND_DISALLOWED_ERROR;
    UINT8 *cmd_para = hci_cmd_ptr->cmd_parameter;
    UCHAR event_parameter[10] = {[0 ... 9] = 0};
    UINT8 *pCCE_returnPara = &event_parameter[3];

    do
    {
        if (!ll_manager.support_ll_privacy)
        {
            errcode = UNSUPPORTED_REMOTE_FEATURE_ERROR;
            break;
        }

        if (cmd_para[0] > 0x01)
        {
            /* invalid address type */
            errcode = INVALID_HCI_COMMAND_PARAMETERS_ERROR;
            break;
        }

        UINT8 idx = ll_driver_search_dev_from_list(LL_RESOLVING_LIST_TYPE, cmd_para[0], &cmd_para[1]);

        if (idx < LL_MAX_RESOLVING_LIST_SIZE)
        {
            memcpy(&pCCE_returnPara[1], ll_manager.resolving_list.item[idx].peer_rpa, 6);
            errcode = HCI_COMMAND_SUCCEEDED;
        }
        else
        {
            errcode = NO_CONNECTION_ERROR;
        }
    }
    while (0);

    hci_generate_le_command_complete_event_head(hci_cmd_ptr->cmd_opcode, event_parameter);

    /* status field - byte 0 */
    pCCE_returnPara[0] = errcode;

    LL_HCI_GEN_COMMAND_COMPLETE_EVENT(event_parameter, 7);

    return HCI_COMMAND_SUCCEEDED;
}

/**************************************************************************
 * Function     : hci_handle_le_read_local_resolvable_address_command
 *
 * Description  : This function is used to handle HCI LE Read Local Resolvable
 *                Address Command. Please refer 7.8.43 in HCI part of BT 4.2 spec.
 *
 * Parameters   : hci_cmd_ptr Pointer to the command packet.
 *
 * Returns      : HCI_COMMAND_SUCCEEDED or HCI error code
 *
 *************************************************************************/
UINT8 hci_handle_le_read_local_resolvable_address_command(HCI_CMD_PKT *hci_cmd_ptr)
{
    UINT8 errcode = COMMAND_DISALLOWED_ERROR;
    UINT8 *cmd_para = hci_cmd_ptr->cmd_parameter;
    UCHAR event_parameter[10] = {[0 ... 9] = 0};
    UINT8 *pCCE_returnPara = &event_parameter[3];

    do
    {
        if (!ll_manager.support_ll_privacy)
        {
            errcode = UNSUPPORTED_REMOTE_FEATURE_ERROR;
            break;
        }

        if (cmd_para[0] > 0x01)
        {
            /* invalid address type */
            errcode = INVALID_HCI_COMMAND_PARAMETERS_ERROR;
            break;
        }

        UINT8 idx = ll_driver_search_dev_from_list(LL_RESOLVING_LIST_TYPE, cmd_para[0], &cmd_para[1]);

        if (idx < LL_MAX_RESOLVING_LIST_SIZE)
        {
            memcpy(&pCCE_returnPara[1], ll_manager.resolving_list.item[idx].local_rpa, 6);
            errcode = HCI_COMMAND_SUCCEEDED;
        }
        else
        {
            errcode = NO_CONNECTION_ERROR;
        }
    }
    while (0);

    hci_generate_le_command_complete_event_head(hci_cmd_ptr->cmd_opcode, event_parameter);

    /* status field - byte 0 */
    pCCE_returnPara[0] = errcode;

    LL_HCI_GEN_COMMAND_COMPLETE_EVENT(event_parameter, 7);

    return HCI_COMMAND_SUCCEEDED;
}

/**************************************************************************
 * Function     : hci_handle_le_set_address_resolution_enable_command
 *
 * Description  : This function is used to handle HCI LE Set Address Resolution
 *                Command. Please refer 7.8.44 in HCI part of BT 4.2 spec.
 *
 * Parameters   : hci_cmd_ptr Pointer to the command packet.
 *
 * Returns      : HCI_COMMAND_SUCCEEDED or HCI error code
 *
 *************************************************************************/
UINT8 hci_handle_le_set_address_resolution_enable_command(HCI_CMD_PKT *hci_cmd_ptr)
{
    UINT8 errcode = COMMAND_DISALLOWED_ERROR;
    BOOLEAN is_addr_resolution_on = hci_cmd_ptr->cmd_parameter[0];

    do
    {
        if (!ll_manager.support_ll_privacy)
        {
            errcode = UNSUPPORTED_REMOTE_FEATURE_ERROR;
            break;
        }

        if (ll_manager.adv_unit.enable || ll_manager.scan_unit.enable || ll_manager.initiator_unit.enable)
        {
            /* This command can be used at any time except when:
             * - Advertising is enabled
             * - Scanning is enabled
             * - Create connection command is outstanding
             */
            break;
        }

        ll_driver_set_address_resolution_enable(is_addr_resolution_on);

        errcode = HCI_COMMAND_SUCCEEDED;
    }
    while (0);

    /* generate command complete event */
    UCHAR event_parameter[4];
    UINT8 *pCCE_returnPara = &event_parameter[3];
    hci_generate_le_command_complete_event_head(hci_cmd_ptr->cmd_opcode, event_parameter);
    pCCE_returnPara[0] = errcode;
    LL_HCI_GEN_COMMAND_COMPLETE_EVENT(event_parameter, 1);

    return errcode;
}

static UINT32 rpa_timeout_sec = 0;

static void hci_handle_rpa_timeout(TimerHandle_t timer)
{
    if (rpa_timeout_sec == 0)
    {
        ll_driver_set_resolvable_address_regenerate();
        rpa_timeout_sec = ll_manager.RPA_timeout;
    }
    else
    {
        rpa_timeout_sec -= OS_MAX_BUSY_PERIOD;
    }

    if (rpa_timeout_sec > OS_MAX_BUSY_PERIOD)
    {
        OS_CREATE_TIMER(ONESHOT_TIMER, ll_manager.RPA_timer,
                hci_handle_rpa_timeout, NULL, 0);

        OS_START_TIMER(*ll_manager.RPA_timer, OS_MAX_BUSY_PERIOD * 1000);
    }
    else
    {
        OS_CREATE_TIMER(ONESHOT_TIMER, ll_manager.RPA_timer,
                hci_handle_rpa_timeout, NULL, 0);

        OS_START_TIMER(*ll_manager.RPA_timer, rpa_timeout_sec * 1000);

        rpa_timeout_sec = 0;
    }
}

void ll_driver_set_resolvable_address_regenerate_timeout(TimerHandle_t timer)
{
    ll_driver_set_resolvable_address_regenerate();
}

/**************************************************************************
 * Function     : hci_handle_le_set_resolvable_private_address_timeout_command
 *
 * Description  : This function is used to handle HCI LE Set Resolvable Private Address Timeout
 *                Command. Please refer 7.8.45 in HCI part of BT 4.2 spec.
 *
 * Parameters   : hci_cmd_ptr Pointer to the command packet.
 *
 * Returns      : HCI_COMMAND_SUCCEEDED or HCI error code
 *
 *************************************************************************/
UINT8 hci_handle_le_set_resolvable_private_address_timeout_command(HCI_CMD_PKT *hci_cmd_ptr)
{
    UINT8 errcode = COMMAND_DISALLOWED_ERROR;

    do
    {
        if (!ll_manager.support_ll_privacy)
        {
            errcode = UNSUPPORTED_REMOTE_FEATURE_ERROR;
            break;
        }

        ll_manager.RPA_timeout = hci_cmd_ptr->cmd_parameter[0] + (hci_cmd_ptr->cmd_parameter[1] << 8);

        /* Range for N: 0x0001 ¡V 0xA1B8 (1 sec ¡V approximately 11.5 hours) */
        if (ll_manager.RPA_timeout > 0xA1B8)
            break;

        if (*ll_manager.RPA_timer != NULL)
        {
            OS_DELETE_TIMER(ll_manager.RPA_timer);
        }

        if (ll_manager.RPA_timeout == 0)
        {
            errcode = INVALID_HCI_COMMAND_PARAMETERS_ERROR;
            break;
        }

        /* set RPA timeout timer */
        if (ll_manager.RPA_timeout > OS_MAX_BUSY_PERIOD)
        {
            rpa_timeout_sec = ll_manager.RPA_timeout;

            OS_CREATE_TIMER(ONESHOT_TIMER, ll_manager.RPA_timer,
                    hci_handle_rpa_timeout, NULL, 0);

            OS_START_TIMER(*ll_manager.RPA_timer, OS_MAX_BUSY_PERIOD * 1000);
        }
        else
        {
            OS_CREATE_TIMER(PERIODIC_TIMER, ll_manager.RPA_timer,
                    ll_driver_set_resolvable_address_regenerate_timeout, NULL, 0);

            OS_START_TIMER(*ll_manager.RPA_timer, ll_manager.RPA_timeout * 1000);
        }

        errcode = HCI_COMMAND_SUCCEEDED;
    }
    while (0);

    /* generate command complete event */
    UCHAR event_parameter[4];
    UINT8 *pCCE_returnPara = &event_parameter[3];
    hci_generate_le_command_complete_event_head(hci_cmd_ptr->cmd_opcode, event_parameter);
    pCCE_returnPara[0] = errcode;
    LL_HCI_GEN_COMMAND_COMPLETE_EVENT(event_parameter, 1);

    return errcode;
}

#define HCI_LE_ADD_DEVICE_TO_RESOLVING_LIST           hci_handle_le_add_device_to_resolving_list_command
#define HCI_LE_REMOVE_DEVICE_FROM_RESOLVING_LIST      hci_handle_le_remove_device_from_resolving_list_command
#define HCI_LE_CLEAR_RESOLVING_LIST                   hci_handle_le_clear_resolving_list_command
#define HCI_LE_READ_RESOLVING_LIST_SIZE               hci_handle_le_read_resolving_list_size_command
#define HCI_LE_READ_PEER_RESOLVABLE_ADDRESS           hci_handle_le_read_peer_resolvable_address_command
#define HCI_LE_READ_LOCAL_RESOLVABLE_ADDRESS          hci_handle_le_read_local_resolvable_address_command
#define HCI_LE_SET_ADDRESS_RESOLUTION_ENABLE          hci_handle_le_set_address_resolution_enable_command
#define HCI_LE_SET_RESOLVABLE_PRIVATE_ADDRESS_TIMEOUT hci_handle_le_set_resolvable_private_address_timeout_command
#else
#define HCI_LE_ADD_DEVICE_TO_RESOLVING_LIST           hci_handle_le_invalid_command
#define HCI_LE_REMOVE_DEVICE_FROM_RESOLVING_LIST      hci_handle_le_invalid_command
#define HCI_LE_CLEAR_RESOLVING_LIST                   hci_handle_le_invalid_command
#define HCI_LE_READ_RESOLVING_LIST_SIZE               hci_handle_le_invalid_command
#define HCI_LE_READ_PEER_RESOLVABLE_ADDRESS           hci_handle_le_invalid_command
#define HCI_LE_READ_LOCAL_RESOLVABLE_ADDRESS          hci_handle_le_invalid_command
#define HCI_LE_SET_ADDRESS_RESOLUTION_ENABLE          hci_handle_le_invalid_command
#define HCI_LE_SET_RESOLVABLE_PRIVATE_ADDRESS_TIMEOUT hci_handle_le_invalid_command
#endif /* _BT4_2_PRIVACY_FEATURE_SUPPORT_ */

#ifdef _BT4_2_LE_SECURE_CONNECTIONS_SUPPORT_

UINT8 hci_handle_le_read_local_p256_public_key(HCI_CMD_PKT *hci_cmd_ptr)
{
    UINT8 errcode = HCI_COMMAND_SUCCEEDED;

    /* generate command status event */
    hci_generate_command_status_event(hci_cmd_ptr->cmd_opcode, errcode);

    LL_TASK_PARA_U task_param;
    OS_SIGNAL sig_send;

    /* generate LE Read Local P-256 Public Key Complete event to host */
    task_param.Dword = 0;
    task_param.lmp_s.sub_type = LL_TASK_HANDLE_READ_LOCAL_P256_PUBLIC_KEY_COMP;
    task_param.lmp_s.conn_entry = 0;
    task_param.lmp_s.status = HCI_COMMAND_SUCCEEDED;
    sig_send.type = LL_GEN_HCI_EVENT_SIGNAL;
    sig_send.param = (OS_ADDRESS)task_param.Dword;
    OS_SEND_SIGNAL_TO_TASK (lmp_task_handle, sig_send);

    return errcode;
}

extern void bz_auth_convert_to_pubkey_internal_format(const UCHAR buf[BZ_AUTH_PUBLIC_KEY_CO_ORDINATE_SIZE_MAX*2],
        BZ_AUTH_PUBLIC_KEY_MAX* pubkey, UCHAR len_prime);
static ssp_pukey_t_max remote_pubkey;

UINT8 hci_handle_le_generate_dhkey(HCI_CMD_PKT *hci_cmd_ptr)
{
    UINT8 errcode = HCI_COMMAND_SUCCEEDED;

    /* generate command status event */
    hci_generate_command_status_event(hci_cmd_ptr->cmd_opcode, errcode);

    bz_auth_convert_to_pubkey_internal_format(&hci_cmd_ptr->cmd_parameter[0], &remote_pubkey, 8);

    LL_TASK_PARA_U task_param;
    OS_SIGNAL sig_send;

    /* generate LE DHKey Generation Complete event to host */
    task_param.Dword = 0;
    task_param.lmp_s.sub_type = LL_TASK_HANDLE_GENERATE_DHKEY_COMP;
    task_param.lmp_s.conn_entry = 0;
    task_param.lmp_s.status = HCI_COMMAND_SUCCEEDED;
    sig_send.type = LL_GEN_HCI_EVENT_SIGNAL;
    sig_send.param = (OS_ADDRESS)task_param.Dword;
    OS_SEND_SIGNAL_TO_TASK (lmp_task_handle, sig_send);

    return errcode;
}

#define HCI_LE_READ_LOCAL_P256_PUBLIC_KEY             hci_handle_le_read_local_p256_public_key
#define HCI_LE_GENERATE_DHKEY                         hci_handle_le_generate_dhkey

#else
#define HCI_LE_READ_LOCAL_P256_PUBLIC_KEY             hci_handle_le_invalid_command
#define HCI_LE_GENERATE_DHKEY                         hci_handle_le_invalid_command
#endif
#ifdef _BT5_0_LE2MBPS_SUPPORT_
UCHAR hci_handle_le_receiver_test_general(HCI_CMD_PKT *hci_cmd_ptr, UINT8 enhanced)
{
    UCHAR event_parameter[4];
    UINT8 errcode = INVALID_HCI_COMMAND_PARAMETERS_ERROR;
    UINT8 *pCCE_returnPara = &event_parameter[3];
    UINT8 rx_frequency;
    LE_REG_S_SET reg;
#ifdef _BT5_0_LE2MBPS_SUPPORT_
    UINT8 phy;
    UINT8 modulation_idx;
#endif

    /*----------------------------------------------------*/
    /*         Parser the Command Parameters              */
    /*----------------------------------------------------*/
    /* RX_Frequency - byte 0 */
    rx_frequency = hci_cmd_ptr->cmd_parameter[0];
#ifdef _BT5_0_LE2MBPS_SUPPORT_
    if (enhanced)
    {
        phy = hci_cmd_ptr->cmd_parameter[1];
        modulation_idx = hci_cmd_ptr->cmd_parameter[2];
    }
    else
    {
        phy = 1;
        modulation_idx = 0;
    }
#endif
    if (rx_frequency <= LL_MAX_RF_CHANNEL_INDEX)
    {
        ll_manager.cur_mode = LL_MODE_RX_TEST;
        ll_manager.test_unit.rf_channel = rx_frequency;
        ll_manager.test_unit.num_of_rx_pkts = 0;
        ll_manager.test_unit.num_of_rx_pkts_crc_err = 0;
        errcode = HCI_COMMAND_SUCCEEDED;

        /* disable whitenning and keep duplicated rx packet */
        reg.value = RD_LE_REG(LE_REG_RX_PKT_ERR_CTRL);
        reg.rx_pkt_err_ctrl.whiten_dis = 1;
        reg.rx_pkt_err_ctrl.pkt_dup_kept = 1;
        reg.rx_pkt_err_ctrl.crc_err_kept = 1;
        WR_LE_REG(LE_REG_RX_PKT_ERR_CTRL, reg.value);

        /* configure RF Channel */
        reg.value = RD_LE_REG(LE_REG_MODEM_CONTROL);
        reg.modem_ctrl.test_mode_rf_ch = rx_frequency;
#ifdef _BT5_0_LE2MBPS_SUPPORT_
        if (enhanced)
        {
            reg.modem_ctrl.TM_Midx = 0;  /* (dape) Yilin says this parameter shouldn't matter.
                                            So FW don't change this. */
            reg.modem_ctrl.TM_PhyRate = 1;
        }
        else
        {
            reg.modem_ctrl.TM_Midx = 0;
            reg.modem_ctrl.TM_PhyRate = 0;
        }
#endif
        WR_LE_REG(LE_REG_MODEM_CONTROL, reg.value);

#ifdef _DAPE_SUPPORT_CHG_AA_IN_LE_TEST_MODE
        if (ll_manager.test_unit.chg_aa)
        {
            WR_LE_REG(LE_REG_TEST_MODE_ACCESS_ADDR_L, ll_manager.test_unit.access_addr_l);
            WR_LE_REG(LE_REG_TEST_MODE_ACCESS_ADDR_H, ll_manager.test_unit.access_addr_h);
        }
        else
        {
            /* as spec. defines (p.2628/2684) LE test packets shall have '10010100100000100110111010001110'
            (in transmission order) as the synchronization word.*/
            WR_LE_REG(LE_REG_TEST_MODE_ACCESS_ADDR_L, 0x4129);
            WR_LE_REG(LE_REG_TEST_MODE_ACCESS_ADDR_H, 0x7176);
        }
#endif

        /* configure LE Receiver Test mode */
        reg.value = RD_LE_REG(LE_REG_RF_TEST_CONTROL);
        reg.rf_test_ctrl.rf_test_rx = TRUE;
        reg.rf_test_ctrl.rf_test_mode_en =  TRUE;
        WR_LE_REG(LE_REG_RF_TEST_CONTROL, reg.value);
    }

    LL_LOG_TRACE(GRAY, HCI_LE_RX_TEST_PARAM_ENHANCED, 4, rx_frequency, errcode);

    /* generate command complete event */
    hci_generate_le_command_complete_event_head(hci_cmd_ptr->cmd_opcode,
            event_parameter);
    pCCE_returnPara[0] = errcode;  /* status field */
    LL_HCI_GEN_COMMAND_COMPLETE_EVENT(event_parameter, 1);

    return errcode;

}
UCHAR hci_handle_le_transmitter_test_general(HCI_CMD_PKT *hci_cmd_ptr, UINT8 enhanced)
{
    UCHAR event_parameter[4];
    UINT8 errcode = INVALID_HCI_COMMAND_PARAMETERS_ERROR;
    UINT8 *pCCE_returnPara = &event_parameter[3];
    UINT8 tx_frequency = 0xFF;
    UINT8 length_of_test_data = 0xFF;
    UINT8 packet_payload = 0xFF;
    UINT16 opcode = hci_cmd_ptr->cmd_opcode;
    LE_REG_S_SET reg;
#ifdef _BT5_0_LE2MBPS_SUPPORT_
    UINT8 phy = 0;
#endif

    do
    {
        /*----------------------------------------------------*/
        /*         Parser the Command Parameters              */
        /*----------------------------------------------------*/
        /* TX_Frequency - byte 0 */
        tx_frequency = hci_cmd_ptr->cmd_parameter[0];
        if (tx_frequency > LL_MAX_RF_CHANNEL_INDEX)
        {
            break;
        }

        /* Length_Of_Test_Data - byte 1 */
        length_of_test_data = hci_cmd_ptr->cmd_parameter[1];
        if (length_of_test_data > LL_MAX_TEST_DATA_LEN)
        {
            break;
        }

        /* Packet_Payload - byte 2 */
        packet_payload = hci_cmd_ptr->cmd_parameter[2];
        if (packet_payload > LL_TEST_PKT_TYPE_MAX)
        {
            break;
        }
#ifdef _BT5_0_LE2MBPS_SUPPORT_
        if (enhanced)
        {
            phy = hci_cmd_ptr->cmd_parameter[3];
            /* dape: I decided not to limit the parameter for 
               ASIC to accept new parameters easier. */
            /*   
            if (phy > xxx)
            {
                break;
            } */           
        }
        else
        {
            phy = 1;
        }
#endif        
#ifdef LE_TX_TEST_MODE_WORKAROUND_CHANGE_TX_TYPE
        /* because hw can keep rx state to let transmit nothing,
           we need to reset it via software reset in workaround way */
        hci_handle_reset_command();
#endif

        ll_manager.cur_mode = LL_MODE_TX_TEST;
        ll_manager.test_unit.rf_channel = tx_frequency;
        ll_manager.test_unit.payload_len = length_of_test_data;
        ll_manager.test_unit.payload_type = packet_payload;
        ll_manager.test_unit.num_of_rx_pkts = 0;
        ll_manager.test_unit.num_of_rx_pkts_crc_err = 0;
        errcode = HCI_COMMAND_SUCCEEDED;
#ifdef _DAPE_SUPPORT_LE_TX_PKT_MODE
        reg.value = RD_LE_REG(LE_REG_TX_TEST_CNT_MODE);
        reg.le_tx_test_cnt_mode_ctrl.counter_mode_en = ll_manager.test_unit.tx_cnt_mode_en;
        reg.le_tx_test_cnt_mode_ctrl.tx_pkt_cnt = ll_manager.test_unit.tx_pkt_cnt;
        WR_LE_REG(LE_REG_TX_TEST_CNT_MODE, reg.value);
#endif
#ifdef _DAPE_SUPPORT_CHG_AA_IN_LE_TEST_MODE
        if (ll_manager.test_unit.chg_aa)
        {
            WR_LE_REG(LE_REG_TEST_MODE_ACCESS_ADDR_L, ll_manager.test_unit.access_addr_l);
            WR_LE_REG(LE_REG_TEST_MODE_ACCESS_ADDR_H, ll_manager.test_unit.access_addr_h);
        }
        else
        {
            /* as spec. defines (p.2628/2684) LE test packets shall have '10010100100000100110111010001110'
            (in transmission order) as the synchronization word.*/
            WR_LE_REG(LE_REG_TEST_MODE_ACCESS_ADDR_L, 0x4129);
            WR_LE_REG(LE_REG_TEST_MODE_ACCESS_ADDR_H, 0x7176);
        }
#endif
        /* disable whitenning */
        reg.value = RD_LE_REG(LE_REG_RX_PKT_ERR_CTRL);
        reg.rx_pkt_err_ctrl.whiten_dis = 1;
        WR_LE_REG(LE_REG_RX_PKT_ERR_CTRL, reg.value);

        /* configure RF Channel */
        reg.value = RD_LE_REG(LE_REG_MODEM_CONTROL);
        reg.modem_ctrl.test_mode_rf_ch = tx_frequency;
#ifdef _BT5_0_LE2MBPS_SUPPORT_
        if (enhanced)
        {
            reg.modem_ctrl.TM_PhyRate = 1;
        }
        else
        {
            reg.modem_ctrl.TM_PhyRate = 0;
        }
#endif
        WR_LE_REG(LE_REG_MODEM_CONTROL, reg.value);

        /* configure LE Transmitter Test mode, payload type, payload length */
        reg.value = RD_LE_REG(LE_REG_RF_TEST_CONTROL);
        reg.rf_test_ctrl.rf_test_rx = FALSE;
        reg.rf_test_ctrl.rf_test_mode_en = TRUE;
        reg.rf_test_ctrl.prbs_mode = TRUE;
#ifdef _DAPE_SUPPORT_LE_TEST_MODE_TX_RANDOM
        /* Originally this is 0. If we want to change the value we change by vendor command. */
        reg.rf_test_ctrl.prbs_fix = ll_manager.test_unit.prbs_fix;
#endif
        reg.rf_test_ctrl.payload_len = length_of_test_data;
        reg.rf_test_ctrl.payload_type = packet_payload;
        WR_LE_REG(LE_REG_RF_TEST_CONTROL, reg.value);
    }
    while (0);

    LL_LOG_TRACE(GRAY, HCI_LE_TX_TEST_PARAM_ENHANCED, 5, tx_frequency,
                 length_of_test_data, packet_payload, phy, errcode);

    /* generate command complete event */
    hci_generate_le_command_complete_event_head(opcode, event_parameter);
    pCCE_returnPara[0] = errcode;  /* status field */
    LL_HCI_GEN_COMMAND_COMPLETE_EVENT(event_parameter, 1);
    return errcode;
}
UINT8 hci_handle_le_read_phy_command(HCI_CMD_PKT *hci_cmd_ptr)
{
    UCHAR event_parameter[4];
    hci_generate_le_command_complete_event_head(hci_cmd_ptr->cmd_opcode,
            event_parameter);
    event_parameter[3] = UNKNOWN_HCI_COMMAND_ERROR;
    LL_HCI_GEN_COMMAND_COMPLETE_EVENT(event_parameter, 1);
    return UNKNOWN_HCI_COMMAND_ERROR;
}
UINT8 hci_handle_le_set_default_phy_command(HCI_CMD_PKT *hci_cmd_ptr)
{
    UCHAR event_parameter[4];
    hci_generate_le_command_complete_event_head(hci_cmd_ptr->cmd_opcode,
            event_parameter);
    event_parameter[3] = UNKNOWN_HCI_COMMAND_ERROR;
    LL_HCI_GEN_COMMAND_COMPLETE_EVENT(event_parameter, 1);
    return UNKNOWN_HCI_COMMAND_ERROR;
}
UINT8 hci_handle_le_set_phy_command(HCI_CMD_PKT *hci_cmd_ptr)
{
    UCHAR event_parameter[4];
    hci_generate_le_command_complete_event_head(hci_cmd_ptr->cmd_opcode,
            event_parameter);
    event_parameter[3] = UNKNOWN_HCI_COMMAND_ERROR;
    LL_HCI_GEN_COMMAND_COMPLETE_EVENT(event_parameter, 1);
    return UNKNOWN_HCI_COMMAND_ERROR;
}
UINT8 hci_handle_le_enhanced_rx_test_command(HCI_CMD_PKT *hci_cmd_ptr)
{
    UINT8 errcode = INVALID_HCI_COMMAND_PARAMETERS_ERROR;
#ifdef _BT5_0_LE2MBPS_SUPPORT_
    errcode = hci_handle_le_receiver_test_general(hci_cmd_ptr, 1);
#endif
    return errcode;
}
UINT8 hci_handle_le_enhanced_tx_test_command(HCI_CMD_PKT *hci_cmd_ptr)
{
RT_BT_LOG(GREEN, DAPE_TEST_LOG293, 1, 8888);
    UINT8 errcode = INVALID_HCI_COMMAND_PARAMETERS_ERROR;
#ifdef _BT5_0_LE2MBPS_SUPPORT_
    errcode = hci_handle_le_transmitter_test_general(hci_cmd_ptr, 1);
#endif
    return errcode;
}


#define HCI_LE_READ_PHY_HDLR                          hci_handle_le_read_phy_command    
#define HCI_LE_SET_DEFAULT_PHY_HDLR                   hci_handle_le_set_default_phy_command
#define HCI_LE_SET_PHY_HDLR                           hci_handle_le_set_phy_command
#define HCI_LE_ENHANCED_RX_TEST_HDLR                  hci_handle_le_enhanced_rx_test_command
#define HCI_LE_ENHANCED_TX_TEST_HDLR                  hci_handle_le_enhanced_tx_test_command

#else
#define HCI_LE_READ_PHY_HDLR                          hci_handle_le_invalid_command    
#define HCI_LE_SET_DEFAULT_PHY_HDLR                   hci_handle_le_invalid_command
#define HCI_LE_SET_PHY_HDLR                           hci_handle_le_invalid_command
#define HCI_LE_ENHANCED_RX_TEST_HDLR                  hci_handle_le_invalid_command
#define HCI_LE_ENHANCED_TX_TEST_HDLR                  hci_handle_le_invalid_command
#endif
/* The function table of HCI LE Control Command Handling */
UINT8 (*(hci_handle_le_control_command[])) (HCI_CMD_PKT *) =
{
    hci_handle_le_invalid_command,                          /* ocf 0x00 */
    hci_handle_le_set_event_mask,                           /* ocf 0x01 */
    hci_handle_le_read_buffer_size,                         /* ocf 0x02 */
    hci_handle_le_read_local_supported_features,            /* ocf 0x03 */
    hci_handle_le_invalid_command,                          /* ocf 0x04 */
    hci_handle_le_set_random_address,                       /* ocf 0x05 */
    hci_handle_le_set_advertising_parameters,               /* ocf 0x06 */
    hci_handle_le_read_advertising_channel_tx_power,        /* ocf 0x07 */
    hci_handle_le_set_advertising_data,                     /* ocf 0x08 */
    hci_handle_le_set_scan_response_data,                   /* ocf 0x09 */
    hci_handle_le_set_advertising_enable,                   /* ocf 0x0A */
    hci_handle_le_set_scan_parameters,                      /* ocf 0x0B */
    hci_handle_le_set_scan_enable,                          /* ocf 0x0C */
    hci_handle_le_create_connection,                        /* ocf 0x0D */
    hci_handle_le_create_connection_cancel,                 /* ocf 0x0E */
    hci_handle_le_read_white_list_size,                     /* ocf 0x0F */
    hci_handle_le_clear_white_list,                         /* ocf 0x10 */
    hci_handle_le_add_device_to_white_list,                 /* ocf 0x11 */
    hci_handle_le_remove_device_from_white_list,            /* ocf 0x12 */
    hci_handle_le_connection_update,                        /* ocf 0x13 */
    hci_handle_le_set_host_channel_classification,          /* ocf 0x14 */
    hci_handle_le_read_channel_map,                         /* ocf 0x15 */
    hci_handle_le_read_remote_used_features,                /* ocf 0x16 */
    hci_handle_le_encrypt,                                  /* ocf 0x17 */
    hci_handle_le_rand,                                     /* ocf 0x18 */
    hci_handle_le_start_encryption,                         /* ocf 0x19 */
    hci_handle_le_long_term_key_request_reply,              /* ocf 0x1A */
    hci_handle_le_long_term_key_requested_negative_reply,   /* ocf 0x1B */
    hci_handle_le_read_supported_states,                    /* ocf 0x1C */
    hci_handle_le_receiver_test,                            /* ocf 0x1D */
    hci_handle_le_transmitter_test,                         /* ocf 0x1E */
    hci_handle_le_test_end,                                 /* ocf 0x1F */
    hci_handle_le_invalid_command,                          /* ocf 0x20 */
    hci_handle_le_invalid_command,                          /* ocf 0x21 */
    HCI_LE_SET_DATA_LENGTH_HDLR,                            /* ocf 0x22 */
    HCI_LE_READ_DEFAULT_DATA_LENGTH_HDLR,                   /* ocf 0x23 */
    HCI_LE_WRITE_DEFAULT_DATA_LENGTH_HDLR,                  /* ocf 0x24 */
    HCI_LE_READ_LOCAL_P256_PUBLIC_KEY,                      /* ocf 0x25 */
    HCI_LE_GENERATE_DHKEY,                                  /* ocf 0x26 */
    HCI_LE_ADD_DEVICE_TO_RESOLVING_LIST,                    /* ocf 0x27 */
    HCI_LE_REMOVE_DEVICE_FROM_RESOLVING_LIST,               /* ocf 0x28 */
    HCI_LE_CLEAR_RESOLVING_LIST,                            /* ocf 0x29 */
    HCI_LE_READ_RESOLVING_LIST_SIZE,                        /* ocf 0x2A */
    HCI_LE_READ_PEER_RESOLVABLE_ADDRESS,                    /* ocf 0x2B */
    HCI_LE_READ_LOCAL_RESOLVABLE_ADDRESS,                   /* ocf 0x2C */
    HCI_LE_SET_ADDRESS_RESOLUTION_ENABLE,                   /* ocf 0x2D */
    HCI_LE_SET_RESOLVABLE_PRIVATE_ADDRESS_TIMEOUT,          /* ocf 0x2E */
    HCI_LE_READ_MAXIMUM_DATA_LENGTH,                        /* ocf 0x2F */
    HCI_LE_READ_PHY_HDLR,                                   /* ocf 0x30 */    
    HCI_LE_SET_DEFAULT_PHY_HDLR,                            /* ocf 0x31 */
    HCI_LE_SET_PHY_HDLR,                                    /* ocf 0x32 */
    HCI_LE_ENHANCED_RX_TEST_HDLR,                           /* ocf 0x33 */
    HCI_LE_ENHANCED_TX_TEST_HDLR,                           /* ocf 0x34 */
};


/**************************************************************************
 * Function     : hci_execute_4_0_hc_le_control_command
 *
 * Description  : This function is used to handle all HCI LE Commands.
 *
 * Parameters   : hci_cmd_ptr Pointer to the command packet.
 *
 * Returns      : HCI_COMMAND_SUCCEEDED or HCI error code
 *
 *************************************************************************/
UINT8 hci_execute_4_0_hc_le_control_command(HCI_CMD_PKT *hci_cmd_ptr)
{
    UCHAR ret_error;
    UCHAR ocf = hci_cmd_ptr->cmd_opcode & 0x3FF;

    LL_LOG_TRACE(YELLOW, HCI_LE_CONTROL_CMD_NEW, 20,
                                ocf, hci_cmd_ptr->param_total_length,
                                hci_cmd_ptr->cmd_parameter[0],
                                hci_cmd_ptr->cmd_parameter[1],
                                hci_cmd_ptr->cmd_parameter[2],
                                hci_cmd_ptr->cmd_parameter[3],
                                hci_cmd_ptr->cmd_parameter[4],
                                hci_cmd_ptr->cmd_parameter[5],
                                hci_cmd_ptr->cmd_parameter[6],
                                hci_cmd_ptr->cmd_parameter[7],
                                hci_cmd_ptr->cmd_parameter[8],
                                hci_cmd_ptr->cmd_parameter[9],
                                hci_cmd_ptr->cmd_parameter[10],
                                hci_cmd_ptr->cmd_parameter[11],
                                hci_cmd_ptr->cmd_parameter[12],
                                hci_cmd_ptr->cmd_parameter[13],
                                hci_cmd_ptr->cmd_parameter[14],
                                hci_cmd_ptr->cmd_parameter[15],
                                hci_cmd_ptr->cmd_parameter[16],
                                hci_cmd_ptr->cmd_parameter[17]);

    /* dape: I think this belowing code should depend on our support spec.*/
    if (ocf > HCI_LE_MAX_OCF)
    {
        /* it is invalid command. we modify the ocf to be zero then enter
           function table for patchable or extension - austin */
        ocf = 0x00;
    }

    ret_error = (*(hci_handle_le_control_command[ocf]))(hci_cmd_ptr);

    return ret_error;
}


/**************************************************************************
 * Function     : hci_pass_event_through_le_event_mask
 *
 * Description  : This function is used to check whether a particular Mega
 *                Event is to be generated or not (uses the LE event mask).
 *
 * Parameters   : subevent_opcode: Subevent Opcode
 *
 * Returns      : TRUE or FALSE
 *
 *************************************************************************/
UINT8 hci_pass_event_through_le_event_mask(UCHAR subevent_opcode)
{
    UINT8 idx = (subevent_opcode - 1) >> 3;
    UINT8 mask = 1 << ((subevent_opcode - 1) & 0x07);
    if (ll_manager.le_event_mask[idx] & mask)
    {
        return TRUE;
    }
    else
    {
        return FALSE;
    }
}

/**************************************************************************
 * Function     : hci_generate_le_connection_complete_event
 *
 * Description  : This function is used to generate LE Connection Complete
 *                Event. This event indicates to the Host which issued a
 *                LE_Create_Connection command and received a Command Status
 *                event if the connection establishment was failed or successful
 *                (Refer 7.7.65.1 on page 797 in HCI 4.0 spec)
 *
 * Parameters   : status: the status of the event parameters
 *                pHandle: the pointer of handle
 *
 * Returns      : None
 *
 *************************************************************************/
#ifdef _BT4_2_PRIVACY_FEATURE_SUPPORT_
void hci_generate_le_connection_complete_basic(UINT8 status,
        LL_CONN_HANDLE_UNIT *pHandle, BOOLEAN isEnhanced)
{
    UCHAR event_parameter[LE_ENHANCED_CONN_COMPLETE_EVENT_PARA_SIZE] = {[0 ... (LE_ENHANCED_CONN_COMPLETE_EVENT_PARA_SIZE - 1)] = 0};
    LE_CONNECTION_COMPLETE_EVENT_PARA_S *pParam = (LE_CONNECTION_COMPLETE_EVENT_PARA_S *)event_parameter;
    UINT32 offset = 1;
    UINT32 full_event_sz = isEnhanced ? LE_ENHANCED_CONN_COMPLETE_EVENT_PARA_SIZE : LE_CONN_COMPLETE_EVENT_PARA_SIZE;

    pParam->Subevent_Code = isEnhanced ? HCI_LE_ENHANCED_CONNECTION_COMPLETE_SUBEVENT : HCI_LE_CONNECTION_COMPLETE_SUBEVENT;

    pParam->Status = status;

    if (status == HCI_COMMAND_SUCCEEDED)
    {
        pParam->Connection_Handle = pHandle->conn_handle;

        pParam->Role = ll_manager.conn_unit.master ? 0 : 1;

        pParam->Peer_Address_Type = pHandle->remote_info.addr_type;

        pParam->Peer_Address[0] = pHandle->remote_info.u2addr[0];
        pParam->Peer_Address[1] = pHandle->remote_info.u2addr[1];
        pParam->Peer_Address[2] = pHandle->remote_info.u2addr[2];

        if (isEnhanced)
        {
            /* This is only valid when the Own_Address_Type (from the
               HCI_LE_Create_Connection or HCI_LE_Set_Advertising_Parameters
               commands) is set to 0x02 or 0x03
            */
            BOOLEAN is_local_RPA_valid = (pParam->Role == 0) ? (ll_manager.initiator_unit.local_addr_type > 0x01) : (ll_manager.adv_unit.local_addr_type > 0x01);

            if (ll_manager.resolving_list.item[ll_manager.local_IRK_idx].valid_local_IRK && is_local_RPA_valid)
            {
                UINT16 *Local_Resolvable_Private_Address = (UINT16 *)&pParam->Peer_Address[2 + offset];
                Local_Resolvable_Private_Address[0] = ll_manager.resolving_list.item[ll_manager.local_IRK_idx].local_rpa[0];
                Local_Resolvable_Private_Address[1] = ll_manager.resolving_list.item[ll_manager.local_IRK_idx].local_rpa[1];
                Local_Resolvable_Private_Address[2] = ll_manager.resolving_list.item[ll_manager.local_IRK_idx].local_rpa[2];

#ifdef _BT4_2_PRIVACY_FEATURE_SUPPORT_INFO_
                if (ll_manager.show_resolvable_addr)
                {
                    LL_LOG_TRACE(BLUE, MSG_LE_PRIVACY_LOCAL_RPA, 4, ll_manager.local_IRK_idx, Local_Resolvable_Private_Address[2], Local_Resolvable_Private_Address[1], Local_Resolvable_Private_Address[0]);
                }
#endif
            }
            offset += 3;

            /* This is only valid for Peer_Address_Type 0x02 and 0x03 */
            BOOLEAN is_peer_RPA_valid = (pParam->Peer_Address_Type > 0x01);

            if (ll_manager.resolving_list.item[ll_manager.local_IRK_idx].valid_peer_IRK && is_peer_RPA_valid)
            {
                UINT16 *Peer_Resolvable_Private_Address = (UINT16 *)&pParam->Peer_Address[2 + offset];
                Peer_Resolvable_Private_Address[0] = ll_manager.resolving_list.item[ll_manager.local_IRK_idx].peer_rpa[0];
                Peer_Resolvable_Private_Address[1] = ll_manager.resolving_list.item[ll_manager.local_IRK_idx].peer_rpa[1];
                Peer_Resolvable_Private_Address[2] = ll_manager.resolving_list.item[ll_manager.local_IRK_idx].peer_rpa[2];

#ifdef _BT4_2_PRIVACY_FEATURE_SUPPORT_INFO_
                if (ll_manager.show_resolvable_addr)
                {
                    LL_LOG_TRACE(BLUE, MSG_LE_PRIVACY_PEER_RPA, 4, ll_manager.local_IRK_idx, Peer_Resolvable_Private_Address[2], Peer_Resolvable_Private_Address[1], Peer_Resolvable_Private_Address[0]);
                }
#endif
            }
            offset += 3;
        }

        /* set Conn_Interval */
        *((UINT16 *)&pParam->Peer_Address[2 + offset]) = pHandle->ce_interval;
        offset ++;

        /* set Conn_Latency */
        *((UINT16 *)&pParam->Peer_Address[2 + offset]) = pHandle->slave_latency;
        offset ++;

        /* set Supervision_Timeout */
        *((UINT16 *)&pParam->Peer_Address[2 + offset]) = pHandle->supervision_to;
        offset ++;

        if (ll_manager.conn_unit.master)
        {
            /* set Master_Clock_Accuracy to 0 bcuz of master role */
            *((UINT8 *)&pParam->Peer_Address[2 + offset]) = 0;
        }
        else
        {
            /* set Master_Clock_Accuracy for slave */
            *((UINT8 *)&pParam->Peer_Address[2 + offset]) = ll_manager.conn_unit.masterSCA;

            /* dape added. set slave ce_length */
            WR_LE_REG(LE_REG_CONN_CE_LEN, pHandle->ce_length);

            if (pHandle->slave_latency > 0)
            {
                /* enable slave latency */
                LE_REG_S_SET reg;
                reg.value = RD_LE_REG(LE_REG_SLAVE_WIN_WIDENING_L);
                reg.slave_win_widen_l.sub_en = TRUE;
                WR_LE_REG(LE_REG_SLAVE_WIN_WIDENING_L, reg.value);
            }
        }
    }
    else
    {
        /* fill invalid fields to zero */
        memset(event_parameter + 2, 0, full_event_sz - 2);

        //if (ll_manager.initiator_unit.enable)
        /* dape: ll_manager.initiator_unit.enable is disabled
           when fw calls ll_driver_create_connection_cancel().
           So we can not generate event base on this variable.
           Since there are no other errors except these two
           status: NO_CONNECTION_ERROR &
           DIRECTED_ADVERTISING_TIMEOUT, we can use this to
           distinguish the state. */
        if (status == NO_CONNECTION_ERROR)
        {
            LL_INITIATOR_UNIT *pinit = &ll_manager.initiator_unit;
            pParam->Peer_Address_Type = pinit->remote_addr_type;
            pParam->Peer_Address[0] = pinit->u2rem_addr[0];
            pParam->Peer_Address[1] = pinit->u2rem_addr[1];
            pParam->Peer_Address[2] = pinit->u2rem_addr[2];
            pinit->enable = 0;
        }

        //if (ll_manager.adv_unit.enable)
        /* dape: use status instead of ll_manager.adv_unit.enable
           or it may cause error when controller is in multi-state,
           such as adv+initating. */
        if (status == DIRECTED_ADVERTISING_TIMEOUT)
        {
            LL_ADVERTISER_UNIT *padv = &ll_manager.adv_unit;

            if (padv->pdu_type == LL_ADV_PDU_TYPE_ADV_DIRECT_IND)
            {
                pParam->Peer_Address_Type = padv->direct_addr_type;
                pParam->Peer_Address[0] = padv->u2dir_addr[0];
                pParam->Peer_Address[1] = padv->u2dir_addr[1];
                pParam->Peer_Address[2] = padv->u2dir_addr[2];
            }
            pParam->Role = SLAVE;
            padv->enable = 0;
        }
    }

#ifdef _DAPE_PRINT_MORE_LOG_INFO
#ifdef _DAPE_MOVE_LE_SLAVE_PARAMETER_TO_CAM
    // print access address for slave
    UINT32 conn_aa;
    UINT8 cam_entry = 0;
    if(ll_manager.conn_unit.master)
    {
        cam_entry = pHandle->unit_id;
    }
    UINT8 dw_addr;
    dw_addr = LE_CAM_ENTRY_BASE(cam_entry) + LE_CAM_ADDR_AA;
    conn_aa = ll_driver_read_cam(dw_addr);
#else
    // print access address for slave
    UINT32 conn_aa;
    if(ll_manager.conn_unit.master)
    {
        UINT8 dw_addr;
        dw_addr = LE_CAM_ENTRY_BASE(pHandle->unit_id) + LE_CAM_ADDR_AA;
        conn_aa = ll_driver_read_cam(dw_addr);
    }
    else
    {
        conn_aa = ((RD_LE_REG(LE_REG_SLAVE_AA_L))|(RD_LE_REG(LE_REG_SLAVE_AA_H)<<16));
    }
#endif
    if (pHandle != NULL)
    {
        LL_LOG_TRACE(WHITE, LE_MSG_CONN_COMPLETE_EVENT_NEW, 7, status,
                     pHandle->unit_id, pHandle->conn_handle,
                ll_manager.conn_unit.master, conn_aa, pHandle->ce_interval, pHandle->slave_latency);
    }
#else
    if (pHandle != NULL)
    {
        LL_LOG_TRACE(WHITE, LE_MSG_CONN_COMPLETE_EVENT, 4, status,
                     pHandle->unit_id, pHandle->conn_handle,
                     ll_manager.conn_unit.master);
    }
#endif

    LL_HCI_GEN_LE_MEGA_EVENT(event_parameter, full_event_sz);
}
#else
void hci_generate_le_connection_complete_event(UINT8 status,
        LL_CONN_HANDLE_UNIT *pHandle)
{
    UCHAR event_parameter[LE_CONN_COMPLETE_EVENT_PARA_SIZE];
    LE_CONNECTION_COMPLETE_EVENT_PARA_S *pPara;
    pPara = (LE_CONNECTION_COMPLETE_EVENT_PARA_S *)event_parameter;

    pPara->Subevent_Code = HCI_LE_CONNECTION_COMPLETE_SUBEVENT;
    pPara->Status = status;
    if (status == HCI_COMMAND_SUCCEEDED)
    {
        pPara->Connection_Handle = pHandle->conn_handle;
        pPara->Role = ll_manager.conn_unit.master ? 0 : 1;
        pPara->Peer_Address_Type = pHandle->remote_info.addr_type;
        pPara->Peer_Address[0] = pHandle->remote_info.u2addr[0];
        pPara->Peer_Address[1] = pHandle->remote_info.u2addr[1];
        pPara->Peer_Address[2] = pHandle->remote_info.u2addr[2];
        pPara->Conn_Interval = pHandle->ce_interval;
        pPara->Conn_Latency = pHandle->slave_latency;
        pPara->Supervision_Timeout = pHandle->supervision_to;
        if (ll_manager.conn_unit.master)
        {
            pPara->Master_Clock_Accuracy = 0;
        }
        else
        {
            pPara->Master_Clock_Accuracy = ll_manager.conn_unit.masterSCA;
            /* dape added. set slave ce_length */
            WR_LE_REG(LE_REG_CONN_CE_LEN, pHandle->ce_length);

            if (pHandle->slave_latency > 0)
            {
                /* enable slave latency */
                LE_REG_S_SET reg;
                reg.value = RD_LE_REG(LE_REG_SLAVE_WIN_WIDENING_L);
                reg.slave_win_widen_l.sub_en = TRUE;
                WR_LE_REG(LE_REG_SLAVE_WIN_WIDENING_L, reg.value);
            }
        }
    }
    else
    {
        /* fill invalid fields to zero */
        memset(event_parameter + 2, 0, LE_CONN_COMPLETE_EVENT_PARA_SIZE - 2);

        //if (ll_manager.initiator_unit.enable)
        /* dape: ll_manager.initiator_unit.enable is disabled
           when fw calls ll_driver_create_connection_cancel().
           So we can not generate event base on this variable.
           Since there are no other errors except these two
           status: NO_CONNECTION_ERROR &
           DIRECTED_ADVERTISING_TIMEOUT, we can use this to
           distinguish the state. */
        if (status == NO_CONNECTION_ERROR)
        {
            LL_INITIATOR_UNIT *pinit = &ll_manager.initiator_unit;
            pPara->Peer_Address_Type = pinit->remote_addr_type;
            pPara->Peer_Address[0] = pinit->u2rem_addr[0];
            pPara->Peer_Address[1] = pinit->u2rem_addr[1];
            pPara->Peer_Address[2] = pinit->u2rem_addr[2];
            pinit->enable = 0;
        }

        //if (ll_manager.adv_unit.enable)
        /* dape: use status instead of ll_manager.adv_unit.enable
           or it may cause error when controller is in multi-state,
           such as adv+initating. */
        if (status == DIRECTED_ADVERTISING_TIMEOUT)
        {
            LL_ADVERTISER_UNIT *padv = &ll_manager.adv_unit;

            if (padv->pdu_type == LL_ADV_PDU_TYPE_ADV_DIRECT_IND)
            {
                pPara->Peer_Address_Type = padv->direct_addr_type;
                pPara->Peer_Address[0] = padv->u2dir_addr[0];
                pPara->Peer_Address[1] = padv->u2dir_addr[1];
                pPara->Peer_Address[2] = padv->u2dir_addr[2];
            }
            pPara->Role = SLAVE;
            padv->enable = 0;
        }
    }
#ifdef _DAPE_PRINT_MORE_LOG_INFO
#ifdef _DAPE_MOVE_LE_SLAVE_PARAMETER_TO_CAM
    // print access address for slave
    UINT32 conn_aa;
    UINT8 cam_entry = 0;
    if(ll_manager.conn_unit.master)
    {
        cam_entry = pHandle->unit_id;
    }
    UINT8 dw_addr;
    dw_addr = LE_CAM_ENTRY_BASE(cam_entry) + LE_CAM_ADDR_AA;
    conn_aa = ll_driver_read_cam(dw_addr);
#else
    // print access address for slave
    UINT32 conn_aa;
    if(ll_manager.conn_unit.master)
    {
        UINT8 dw_addr;
        dw_addr = LE_CAM_ENTRY_BASE(pHandle->unit_id) + LE_CAM_ADDR_AA;
        conn_aa = ll_driver_read_cam(dw_addr);
    }
    else
    {
        conn_aa = ((RD_LE_REG(LE_REG_SLAVE_AA_L))|(RD_LE_REG(LE_REG_SLAVE_AA_H)<<16));
    }
#endif
    if (pHandle != NULL)
    {
        LL_LOG_TRACE(WHITE, LE_MSG_CONN_COMPLETE_EVENT_NEW, 5, status,
                pHandle->unit_id, pHandle->conn_handle,
                ll_manager.conn_unit.master, conn_aa);
    }
#else
    if (pHandle != NULL)
    {
        LL_LOG_TRACE(WHITE, LE_MSG_CONN_COMPLETE_EVENT, 4, status,
                pHandle->unit_id, pHandle->conn_handle,
                ll_manager.conn_unit.master);
    }
#endif

    LL_HCI_GEN_LE_MEGA_EVENT(event_parameter, LE_CONN_COMPLETE_EVENT_PARA_SIZE);
}
#endif /* _BT4_2_PRIVACY_FEATURE_SUPPORT_ */

/**************************************************************************
 * Function     : hci_generate_le_advertising_report_event
 *
 * Description  : This function is used to generate LE Advertising Report
 *                Event. LE Advertising Report event indicates that a Bluetooth
 *                device or multiple Bluetooth devices have responded to an
 *                active scan or received some information during a passive
 *                scan. The Controller may queue these advertising reports
 *                and send information from multiple devices in one LE
 *                Advertising Report event.
 *                (Refer 7.7.65.2 on page 800 in HCI 4.0 spec)
 *
 * Parameters   : None
 *
 * Returns      : None
 *
 *************************************************************************/
void hci_generate_le_advertising_report_event(void)
{
    UCHAR event_parameter[HCI_MAX_EVENT_PARAM_LEN];
    LL_ADV_REPORT_UNIT *rpt;
    LE_ADVERTISING_REPORT_EVENT_PARA_S *pPara;
    UINT8 i;
    UINT16 offset = 0;
    UINT8 num_of_rpt;
    rpt = &ll_manager.scan_unit.report;
    num_of_rpt = rpt->num_of_reports;
    pPara = (LE_ADVERTISING_REPORT_EVENT_PARA_S *)event_parameter;

    if (num_of_rpt == 0)
    {
        /* avoid to generate invalid event parameters for host */
        return;
    }

    /* fill subevent code field */
    pPara->Subevent_Code = rpt->is_direct_adv_report ? HCI_LE_DIRECT_ADVERTISING_REPORT_SUBEVENT : HCI_LE_ADVERTISING_REPORT_SUBEVENT;

    /* fill number of reports field */
    pPara->Num_Reports = num_of_rpt;

    /* fill every advertising information to the report */
    for (i = 0; i < num_of_rpt; i++)
    {
        /* fill event type field */
        pPara->Event_Type[offset] =
            le_ll2hci_adv_rpt_event_type[rpt->event_type[i] & 0x7];
        offset++;

        /* fill address_type field */
        pPara->Event_Type[offset] = rpt->addr_type[i];
        offset++;

        /* fill address field */
        memcpy(&pPara->Event_Type[offset], &rpt->addr[i][0], 6);
        offset += 6;

#ifdef _BT4_2_PRIVACY_FEATURE_SUPPORT_
        if (rpt->is_direct_adv_report)
        {
            /* fill direct addr type */
            pPara->Event_Type[offset] = rpt->direct_addr_type[i];
            offset ++;

            /* fill direct addr */
            memcpy(&pPara->Event_Type[offset], &rpt->direct_addr[i][0], 6);
            offset += 6;
        }
        else
        {
            /* fill length field */
            pPara->Event_Type[offset] = rpt->data_len[i];
            offset++;

            /* fill data field */
            if (rpt->data_len[i] > 0)
            {
                memcpy(&pPara->Event_Type[offset], &rpt->data_buf[i][0],
                       rpt->data_len[i]);
                offset += rpt->data_len[i];
            }
        }
#else
        /* fill length field */
        pPara->Event_Type[offset] = rpt->data_len[i];
        offset++;

        /* fill data field */
        if (rpt->data_len[i] > 0)
        {
            memcpy(&pPara->Event_Type[offset], &rpt->data_buf[i][0],
                   rpt->data_len[i]);
            offset += rpt->data_len[i];
        }
#endif /* _BT4_2_PRIVACY_FEATURE_SUPPORT_ */

        /* fill rssi field */
        pPara->Event_Type[offset] = rpt->rssi[i];
        offset++;
    }

    /* reset number of reports*/
    rpt->num_of_reports = 0;

    LL_HCI_GEN_LE_MEGA_EVENT(event_parameter, 2 + offset);
}

/**************************************************************************
 * Function     : hci_generate_le_connection_update_complete_event
 *
 * Description  : This function is used to generate LE Connection Update
 *                Complete Event. This event is used to indicate that the
 *                Controller process to update the connection has completed.
 *                On a slave, if no connection parameters are updated, then
 *                this event shall not be issued. On a master, this event shall
 *                be issued if the Connection_Update command was sent.
 *                (Refer 7.7.65.3 on page 801 in HCI 4.0 spec)
 *
 * Parameters   : status: the status of the event parameters
 *                pHandle: the pointer of handle
 *
 * Returns      : None
 *
 *************************************************************************/
void hci_generate_le_connection_update_complete_event(UINT8 status,
        LL_CONN_HANDLE_UNIT *pHandle)
{
    UCHAR event_parameter[LE_CONN_UPDT_COMPLETE_EVENT_PARA_SIZE];
    LE_CONNECTION_UPDATE_COMPLETE_EVENT_PARA_S *pPara;
    pPara = (LE_CONNECTION_UPDATE_COMPLETE_EVENT_PARA_S *)event_parameter;

    /* TODO: this event can not be issued autonomously by the slave role */

    pPara->Subevent_Code = HCI_LE_CONNECTION_UPDATE_COMPLETE_SUBEVENT;
    pPara->Status = status;
    pPara->Connection_Handle = pHandle->conn_handle;
    pPara->Conn_Interval = pHandle->ce_interval;
    pPara->Conn_Latency = pHandle->slave_latency;
    pPara->Supervision_Timeout = pHandle->supervision_to;

    LL_HCI_GEN_LE_MEGA_EVENT(event_parameter,
            LE_CONN_UPDT_COMPLETE_EVENT_PARA_SIZE);
}

/**************************************************************************
 * Function     : hci_generate_le_read_remote_used_features_complete_event
 *
 * Description  : This function is used to generate LE Read Remote Used Features
 *                Complete Event. This event is used to indicate the completion
 *                of the process of the Controller obtaining the used features
 *                of the remote Bluetooth device specified by the
 *                Connection_Handle event parameter.
 *                (Refer 7.7.65.4 on page 801 in HCI 4.0 spec)
 *
 * Parameters   : status: the status of the event parameters
 *                pHandle: the pointer of handle
 *
 * Returns      : None
 *
 *************************************************************************/
void hci_generate_le_read_remote_used_features_complete_event(UINT8 status,
        LL_CONN_HANDLE_UNIT *pHandle)
{
    UCHAR event_parameter[LE_READ_REMOTE_USED_FEATURES_COMPLETE_EVENT_PARA_SIZE];
    LE_READ_REMOTE_USED_FEATURES_COMPLETE_EVENT_PARA_S *pPara;
    pPara = (LE_READ_REMOTE_USED_FEATURES_COMPLETE_EVENT_PARA_S *)event_parameter;

    /* TODO: this event can not be issued autonomously by the slave role */

    pPara->Subevent_Code = HCI_LE_READ_REMOTE_USED_FEATURES_COMPLETE_SUBEVENT;
    pPara->Status = status;
    pPara->Connection_Handle = pHandle->conn_handle;
    pPara->u2LE_Features[0] = pHandle->remote_info.u2le_features[0];
    pPara->u2LE_Features[1] = pHandle->remote_info.u2le_features[1];
    pPara->u2LE_Features[2] = pHandle->remote_info.u2le_features[2];
    pPara->u2LE_Features[3] = pHandle->remote_info.u2le_features[3];

    LL_HCI_GEN_LE_MEGA_EVENT(event_parameter,
            LE_READ_REMOTE_USED_FEATURES_COMPLETE_EVENT_PARA_SIZE);
}

/**************************************************************************
 * Function     : hci_generate_le_long_term_key_request_event
 *
 * Description  : This function is used to generate LE Long Term Key Request
 *                Event. The LE Long Term Key Requestevent indicates that the
 *                master device is attempting to encrypt or re-encrypt the link
 *                and is requesting the Long Term Key from the Host.
 *                (Refer 7.7.65.5 on page 804 in HCI 4.0 spec)
 *
 * Parameters   : pHandle: the pointer of handle
 *
 * Returns      : None
 *
 *************************************************************************/
void hci_generate_le_long_term_key_request_event(LL_CONN_HANDLE_UNIT *pHandle)
{
    UCHAR event_parameter[LE_LONG_TERM_KEY_REQUEST_EVENT_PARA_SIZE];
    LE_LONG_TERM_KEY_REQUEST_EVENT_PARA_S *pPara;
    BOOLEAN result;

    pPara = (LE_LONG_TERM_KEY_REQUEST_EVENT_PARA_S *)event_parameter;
// dape modified for LE
    pPara->Subevent_Code = HCI_LE_LONG_TERM_KEY_REQUEST_SUBEVENT;
    pPara->u1Connection_Handle[0] = pHandle->conn_handle & 0xFF;
    pPara->u1Connection_Handle[1] = pHandle->conn_handle >> 8;
    memcpy(pPara->u1Random_Number, pHandle->encrypt_blk.rand, 8);
    pPara->u1Encrypted_Diversifier[0] = pHandle->encrypt_blk.ediv[0];
    pPara->u1Encrypted_Diversifier[1] = pHandle->encrypt_blk.ediv[1];

    result = LL_HCI_GEN_LE_MEGA_EVENT(event_parameter,
            LE_LONG_TERM_KEY_REQUEST_EVENT_PARA_SIZE);

#ifdef LE_HW_TEST
    ll_test_trigger_one_shot_event(LL_TEST_CASE_LTK_REQUEST_REPLY, 0);
#endif

    if (result == FALSE)
    {
        /* host has masked LE Long Term Key Request event */
        if (LLC_START_ENC_STATE_BEGIN == pHandle->encrypt_blk.start_enc_state)
        {
            LL_CTRL_PDU_PAYLOAD *pTxPkt;
            pHandle->encrypt_blk.start_enc_state = LLC_START_ENC_STATE_IDLE;

            /* send LL_REJECT_IND to remote device */
            pTxPkt = llc_generate_ll_reject_ind(KEY_MISSING_ERROR);
            llc_append_pdu_to_tx_list(pTxPkt, pHandle->unit_id);
            llc_check_sent_llc_pdu_request(pHandle, LL_REJECT_IND);
        }
    }
}

#ifdef _BT4_2_DATA_EXTENSION_FEATURE_SUPPORT_
void hci_generate_le_data_length_change_event(LL_CONN_HANDLE_UNIT *chu)
{
    LE_DATA_LENGTH_CHANGE_EVT_PARAM evt_param;
    evt_param.subevt_code = HCI_LE_DATA_LENGTH_CHANGE_SUBEVENT;
    htole16(chu->conn_handle, evt_param.conn_handle);
    htole16(chu->data_len_updt.max_tx_size, evt_param.max_tx_octets);
    htole16(chu->data_len_updt.max_tx_time, evt_param.max_tx_time);
    htole16(chu->data_len_updt.max_rx_size, evt_param.max_rx_octets);
    htole16(chu->data_len_updt.max_rx_time, evt_param.max_rx_time);
    LL_HCI_GEN_LE_MEGA_EVENT(&evt_param, sizeof (evt_param));
}
#endif
#ifdef _BT4_2_LE_SECURE_CONNECTIONS_SUPPORT_

extern void bz_auth_convert_to_pubkey_edtm_format_p256(
        const BZ_AUTH_PUBLIC_KEY_P256* pubkey, UCHAR buf[BZ_AUTH_PUBLIC_KEY_CO_ORDINATE_SIZE_P256*2]);

void hci_generate_le_read_local_p256_public_key_complete_event(void)
{
    UCHAR event_parameter[LE_READ_LOCAL_P256_PUBLIC_KEY_COMPLETE_PARA_SIZE];
    ssp_prkey_t_p256 priv;
    ssp_pukey_t_p256 pub;

    event_parameter[0] = HCI_LE_READ_LOCAL_P256_PUBLIC_KEY_COMPLETE_SUBEVENT;
    event_parameter[1] = HCI_COMMAND_SUCCEEDED;

    ssp_get_ecdh_keypair_p256(priv, &pub);

    bz_auth_convert_to_pubkey_edtm_format_p256(&pub, &event_parameter[2]);

    LL_HCI_GEN_LE_MEGA_EVENT(event_parameter, LE_READ_LOCAL_P256_PUBLIC_KEY_COMPLETE_PARA_SIZE);
}

void hci_generate_le_generate_dhkey_complete_event(void)
{
    UCHAR event_parameter[LE_GENERATE_DHKEY_COMPLETE_EVENT_PARA_SIZE];

    event_parameter[0] = HCI_LE_GENERATE_DHKEY_COMPLETE_SUBEVENT;
    event_parameter[1] = HCI_COMMAND_SUCCEEDED;

    ssp_p256(bz_auth_permanent_params.prikey_p256[0], &remote_pubkey, &event_parameter[2]);

    convert_to_lsb(&event_parameter[2], 32);

    LL_HCI_GEN_LE_MEGA_EVENT(event_parameter, LE_GENERATE_DHKEY_COMPLETE_EVENT_PARA_SIZE);
}

#endif
