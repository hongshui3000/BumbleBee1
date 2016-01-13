enum { __FILE_NUM__= 207 };

#include "le_hci_4_0.h"
#include "common_utils.h"
#include "le_ll_test.h"
#include "bt_fw_hci_external_defines.h"
#include "le_hw_reg.h"
#include "le_ll_driver.h"
#include "mint_os.h"
#include "bt_fw_os.h"
#include "bb_driver.h"
#include "bzdma.h"
#include "le_ll.h"
#include "bz_log_defines.h"


LL_DEV_ADDR_LIST_ENTRY ll_test_addr_entry[LL_MAX_WHITE_LIST_SIZE];
TIMER_ID ll_test_tid_timer = OS_INVALID_HANDLE; /* software timer id */
LL_DEV_ADDR_LIST_ENTRY ll_test_rem_dev_addr_entry;
HCI_CMD_PKT local_hci_cmd_pkt;
LL_TEST_UNIT ll_test_unit;

void ll_test_create_connection(void)
{
    /* create connection parameters */
    local_hci_cmd_pkt.cmd_opcode = HCI_LE_CREATE_CONNECTION_OPCODE;
    local_hci_cmd_pkt.param_total_length = 25;
    local_hci_cmd_pkt.cmd_parameter[0] = LSB(LL_TEST_INIT_SCAN_INTERVAL);
    local_hci_cmd_pkt.cmd_parameter[1] = MSB(LL_TEST_INIT_SCAN_INTERVAL);
    local_hci_cmd_pkt.cmd_parameter[2] = LSB(LL_TEST_INIT_SCAN_WINDOW);
    local_hci_cmd_pkt.cmd_parameter[3] = MSB(LL_TEST_INIT_SCAN_WINDOW); 
    local_hci_cmd_pkt.cmd_parameter[4] = LL_TEST_INIT_FILTER_POLICY;
    local_hci_cmd_pkt.cmd_parameter[5] = ll_test_rem_dev_addr_entry.type;
    local_hci_cmd_pkt.cmd_parameter[6] = ll_test_rem_dev_addr_entry.addr[0];
    local_hci_cmd_pkt.cmd_parameter[7] = ll_test_rem_dev_addr_entry.addr[1];
    local_hci_cmd_pkt.cmd_parameter[8] = ll_test_rem_dev_addr_entry.addr[2];
    local_hci_cmd_pkt.cmd_parameter[9] = ll_test_rem_dev_addr_entry.addr[3];
    local_hci_cmd_pkt.cmd_parameter[10] = ll_test_rem_dev_addr_entry.addr[4];
    local_hci_cmd_pkt.cmd_parameter[11] = ll_test_rem_dev_addr_entry.addr[5];
    local_hci_cmd_pkt.cmd_parameter[12] = LL_TEST_OWN_ADDR_TYPE;
    local_hci_cmd_pkt.cmd_parameter[13] = LSB(LL_TEST_CONN_INTERVAL_MIN);
    local_hci_cmd_pkt.cmd_parameter[14] = MSB(LL_TEST_CONN_INTERVAL_MIN);
    local_hci_cmd_pkt.cmd_parameter[15] = LSB(LL_TEST_CONN_INTERVAL_MAX);
    local_hci_cmd_pkt.cmd_parameter[16] = MSB(LL_TEST_CONN_INTERVAL_MAX);
    local_hci_cmd_pkt.cmd_parameter[17] = LSB(LL_TEST_CONN_LATENCY);
    local_hci_cmd_pkt.cmd_parameter[18] = MSB(LL_TEST_CONN_LATENCY);
    local_hci_cmd_pkt.cmd_parameter[19] = LSB(LL_TEST_SUPERVISION_TIMEOUT);
    local_hci_cmd_pkt.cmd_parameter[20] = MSB(LL_TEST_SUPERVISION_TIMEOUT);
    local_hci_cmd_pkt.cmd_parameter[21] = LSB(LL_TEST_MIN_CE_LENGTH);
    local_hci_cmd_pkt.cmd_parameter[22] = MSB(LL_TEST_MIN_CE_LENGTH);    
    local_hci_cmd_pkt.cmd_parameter[23] = LSB(LL_TEST_MAX_CE_LENGTH);
    local_hci_cmd_pkt.cmd_parameter[24] = MSB(LL_TEST_MAX_CE_LENGTH);    
    hci_handle_le_create_connection(&local_hci_cmd_pkt);
}

void ll_test_cancel_connection(void)
{
    /* cancel connection parameters */
    local_hci_cmd_pkt.cmd_opcode = HCI_LE_CREATE_CONNECTION_CANCEL_OPCODE;
    local_hci_cmd_pkt.param_total_length = 0;    
    hci_handle_le_create_connection_cancel(&local_hci_cmd_pkt);
}

void ll_test_disconnect_connection(void)
{
    UINT8 sent_event_flag;

    /* disconnect parameters */
    local_hci_cmd_pkt.cmd_opcode = HCI_DISCONNECT_OPCODE;
    local_hci_cmd_pkt.param_total_length = 3;     
    local_hci_cmd_pkt.cmd_parameter[0] = LL_HCI_MIN_CONN_HANDLE;  
    local_hci_cmd_pkt.cmd_parameter[1] = 0;      
    local_hci_cmd_pkt.cmd_parameter[2] = CONNECTION_TERMINATED_USER_ERROR;  

    hci_handle_disconnect_command(&local_hci_cmd_pkt, &sent_event_flag );
}

void ll_test_start_encryption(void)
{
    UINT8 i;
    UINT32_S u4buf[3];
        
    /* enable advertising parameters */
    local_hci_cmd_pkt.cmd_opcode = HCI_LE_START_ENCRYPTION_OPCODE;
    local_hci_cmd_pkt.param_total_length = 28;
    local_hci_cmd_pkt.cmd_parameter[0] = LL_HCI_MIN_CONN_HANDLE;  
    local_hci_cmd_pkt.cmd_parameter[1] = 0;   

    LL_DRIVER_GEN_32BIT_RANDOM(u4buf[0].u4Byte);
    LL_DRIVER_GEN_32BIT_RANDOM(u4buf[1].u4Byte);    
    LL_DRIVER_GEN_32BIT_RANDOM(u4buf[2].u4Byte);  

    /* generate 8 bytes random number */
    for (i = 0; i < 8; i++)
    {
        local_hci_cmd_pkt.cmd_parameter[i + 2] = u4buf[i >> 2].u1Byte[i & 0x03];  
    }
    
    /* generate 2 bytes encrypted_diversifier */
    local_hci_cmd_pkt.cmd_parameter[10] = u4buf[2].u1Byte[0];
    local_hci_cmd_pkt.cmd_parameter[11] = u4buf[2].u1Byte[1];    

    /* generate 16 bytes long term key */
    for (i = 0; i < 16; i++)
    {
        local_hci_cmd_pkt.cmd_parameter[12 + i] = i;
    }

    hci_handle_le_start_encryption(&local_hci_cmd_pkt);
}

void ll_test_ltk_request_reply(void)
{
    UINT8 i;

    /* enable advertising parameters */
    local_hci_cmd_pkt.cmd_opcode = HCI_LE_LONG_TERM_KEY_REQUEST_REPLY_OPCODE;
    local_hci_cmd_pkt.param_total_length = 18;
    local_hci_cmd_pkt.cmd_parameter[0] = LL_HCI_MIN_CONN_HANDLE;  
    local_hci_cmd_pkt.cmd_parameter[1] = 0;      

    /* generate 16 bytes long term key */
    for (i = 0; i < 16; i++)
    {
        local_hci_cmd_pkt.cmd_parameter[2 + i] = i;
    }

    hci_handle_le_long_term_key_request_reply(&local_hci_cmd_pkt);
}

void ll_test_enable_advertisier(void)
{
    UINT8 i;

    /* set advertising data buffer */    
    local_hci_cmd_pkt.cmd_opcode = HCI_LE_SET_ADVERTISING_DATA_OPCODE;
    local_hci_cmd_pkt.cmd_parameter[0] = LL_TEST_ADV_DATA_LEN;
    for (i = 0; i < LL_TEST_ADV_DATA_LEN; i++)
    {
        local_hci_cmd_pkt.cmd_parameter[i + 1] = i;    
    }
    local_hci_cmd_pkt.param_total_length = LL_TEST_ADV_DATA_LEN + 1;    
    hci_handle_le_set_advertising_data(&local_hci_cmd_pkt);

    /* set scan response data buffer */
    local_hci_cmd_pkt.cmd_opcode = HCI_LE_SET_SCAN_RESPONSE_DATA_OPCODE;
    local_hci_cmd_pkt.cmd_parameter[0] = LL_TEST_SCAN_RSP_DATA_LEN;
    for (i = 0; i < LL_TEST_SCAN_RSP_DATA_LEN; i++)
    {
        local_hci_cmd_pkt.cmd_parameter[i + 1] = LL_TEST_SCAN_RSP_DATA_LEN - i;    
    }
    hci_handle_le_set_scan_response_data(&local_hci_cmd_pkt);

    /* set advertising parameters */   
    local_hci_cmd_pkt.cmd_opcode = HCI_LE_SET_ADVERTISING_PARAMETERS_OPCODE;
    local_hci_cmd_pkt.param_total_length = 15;    
    local_hci_cmd_pkt.cmd_parameter[0] = LSB(LL_TEST_ADV_INTERVAL_MIN); 
    local_hci_cmd_pkt.cmd_parameter[1] = MSB(LL_TEST_ADV_INTERVAL_MIN); 
    local_hci_cmd_pkt.cmd_parameter[2] = LSB(LL_TEST_ADV_INTERVAL_MAX); 
    local_hci_cmd_pkt.cmd_parameter[3] = MSB(LL_TEST_ADV_INTERVAL_MAX);
    local_hci_cmd_pkt.cmd_parameter[4] = LL_TEST_ADV_TYPE;
    local_hci_cmd_pkt.cmd_parameter[5] = LL_TEST_OWN_ADDR_TYPE;
    local_hci_cmd_pkt.cmd_parameter[6] = LL_TEST_DIRECT_ADDR_TYPE;
    local_hci_cmd_pkt.cmd_parameter[7] = 0;
    local_hci_cmd_pkt.cmd_parameter[8] = 0;
    local_hci_cmd_pkt.cmd_parameter[9] = 0;
    local_hci_cmd_pkt.cmd_parameter[10] = 0;
    local_hci_cmd_pkt.cmd_parameter[11] = 0;
    local_hci_cmd_pkt.cmd_parameter[12] = 0;
    local_hci_cmd_pkt.cmd_parameter[13] = LL_TEST_ADV_CHAN_MAP;
    local_hci_cmd_pkt.cmd_parameter[14] = LL_TEST_ADV_FILTER_POLICY;    
    hci_handle_le_set_advertising_parameters(&local_hci_cmd_pkt);

    /* enable advertising parameters */
    local_hci_cmd_pkt.cmd_opcode = HCI_LE_SET_ADVERTISING_ENABLE_OPCODE;
    local_hci_cmd_pkt.param_total_length = 1;
    local_hci_cmd_pkt.cmd_parameter[0] = 1;
    hci_handle_le_set_advertising_enable(&local_hci_cmd_pkt);
}

void ll_test_disable_advertisier(void)
{
    /* enable advertising parameters */
    local_hci_cmd_pkt.cmd_opcode = HCI_LE_SET_ADVERTISING_ENABLE_OPCODE;
    local_hci_cmd_pkt.param_total_length = 1;
    local_hci_cmd_pkt.cmd_parameter[0] = 0;
    hci_handle_le_set_advertising_enable(&local_hci_cmd_pkt);
}

void ll_test_enable_scanning(void)
{
    /* set LE scan parameters command */    
    local_hci_cmd_pkt.cmd_opcode = HCI_LE_SET_SCAN_PARAMETERS_OPCODE;
    local_hci_cmd_pkt.param_total_length = 7;
    local_hci_cmd_pkt.cmd_parameter[0] = LL_TEST_SCAN_TYPE;
    local_hci_cmd_pkt.cmd_parameter[1] = LSB(LL_TEST_SCAN_INTERVAL);
    local_hci_cmd_pkt.cmd_parameter[2] = MSB(LL_TEST_SCAN_INTERVAL);    
    local_hci_cmd_pkt.cmd_parameter[3] = LSB(LL_TEST_SCAN_WINDOW);
    local_hci_cmd_pkt.cmd_parameter[4] = MSB(LL_TEST_SCAN_WINDOW); 
    local_hci_cmd_pkt.cmd_parameter[5] = LL_TEST_OWN_ADDR_TYPE; 
    local_hci_cmd_pkt.cmd_parameter[6] = LL_TEST_SCAN_FILTER_POLICY;   
    hci_handle_le_set_scan_parameters(&local_hci_cmd_pkt);

    /* set LE scan enable command */
    local_hci_cmd_pkt.cmd_opcode = HCI_LE_SET_SCAN_ENABLE_OPCODE;
    local_hci_cmd_pkt.param_total_length = 2;
    local_hci_cmd_pkt.cmd_parameter[0] = 1;
    local_hci_cmd_pkt.cmd_parameter[1] = LL_TEST_SCAN_FILTER_DUPLICATES;        
    hci_handle_le_set_scan_enable(&local_hci_cmd_pkt);
}

void ll_test_disable_scanning(void)
{
    /* set LE scan enable command */
    local_hci_cmd_pkt.cmd_opcode = HCI_LE_SET_SCAN_ENABLE_OPCODE;
    local_hci_cmd_pkt.param_total_length = 2;
    local_hci_cmd_pkt.cmd_parameter[0] = 0;
    local_hci_cmd_pkt.cmd_parameter[1] = LL_TEST_SCAN_FILTER_DUPLICATES;        
    hci_handle_le_set_scan_enable(&local_hci_cmd_pkt);
}

void ll_test_add_white_list(UINT8 AddrType, UINT8 *pAddr)
{
    UINT8 i;
    
    /* LE add device to white list command */
    local_hci_cmd_pkt.cmd_opcode = HCI_LE_ADD_DEVICE_TO_WHITE_LIST_OPCODE;
    local_hci_cmd_pkt.param_total_length = 7;
    local_hci_cmd_pkt.cmd_parameter[0] = AddrType;
    for (i = 0; i < 6; i++)
    {
        local_hci_cmd_pkt.cmd_parameter[1 + i] = pAddr[i];
    }
    hci_handle_le_add_device_to_white_list(&local_hci_cmd_pkt);
}

void ll_test_remove_white_list(UINT8 AddrType, UINT8 *pAddr)
{
    UINT8 i;
    
    /* LE remove device from white list command */
    local_hci_cmd_pkt.cmd_opcode = HCI_LE_REMOVE_DEVICE_FROM_WHITE_LIST_OPCODE;
    local_hci_cmd_pkt.param_total_length = 7;
    local_hci_cmd_pkt.cmd_parameter[0] = AddrType;
    for (i = 0; i < 6; i++)
    {
        local_hci_cmd_pkt.cmd_parameter[1 + i] = pAddr[i];
    }
    hci_handle_le_remove_device_from_white_list(&local_hci_cmd_pkt); 
}

void ll_test_reset_white_list(void)
{
    LL_DEV_ADDR_LIST_ENTRY addr_entry_tmp;  
    UINT8 cam_addr_wl_base = LE_CAM_WHITE_LIST_BASE;
    UINT8 cam_addr;
    UINT8 i;
    
    /* LE reset white list command */
    local_hci_cmd_pkt.cmd_opcode = HCI_LE_CLEAR_WHITE_LIST_OPCODE;
    local_hci_cmd_pkt.param_total_length = 0;    
    hci_handle_le_clear_white_list(&local_hci_cmd_pkt);      

    for (i = 0; i < LL_MAX_WHITE_LIST_SIZE; i++)
    {
        cam_addr = cam_addr_wl_base + (i << 1);      
        addr_entry_tmp.DWord[0] = ll_driver_read_cam(cam_addr);
        addr_entry_tmp.DWord[1] = ll_driver_read_cam(cam_addr + 1);

        LL_LOG_TRACE(GRAY, LE_MSG_CHK_WHITE_LIST_IN_CAM, 4, i, cam_addr,
                     addr_entry_tmp.DWord[0], addr_entry_tmp.DWord[1]);  
    }
}


void ll_test_white_list_set(void)
{    
    UINT32_S u32tmp[2];
    UINT8 i;
    
    /* init test addr entry via random register */
    for (i = 0; i < LL_MAX_WHITE_LIST_SIZE; i++)
    {
        LL_DRIVER_GEN_32BIT_RANDOM(u32tmp[0].u4Byte);
        LL_DRIVER_GEN_32BIT_RANDOM(u32tmp[1].u4Byte);        
        ll_test_addr_entry[i].type = u32tmp[0].u2Byte[0];
        *(UINT16*)&ll_test_addr_entry[i].addr[0] = u32tmp[0].u2Byte[1];
        *(UINT16*)&ll_test_addr_entry[i].addr[2] = u32tmp[1].u2Byte[0];        
        *(UINT16*)&ll_test_addr_entry[i].addr[4] = u32tmp[1].u2Byte[1];
    }

    for (i = 0; i < LL_MAX_WHITE_LIST_SIZE; i++)
    {
        LL_LOG_TRACE(GRAY, LE_MSG_ADD_DEV_LIST, 8, 
            i, ll_test_addr_entry[i].type, 
            ll_test_addr_entry[i].addr[0], ll_test_addr_entry[i].addr[1], 
            ll_test_addr_entry[i].addr[2], ll_test_addr_entry[i].addr[3], 
            ll_test_addr_entry[i].addr[4], ll_test_addr_entry[i].addr[5]);

        ll_test_add_white_list(ll_test_addr_entry[i].type, 
                               ll_test_addr_entry[i].addr);        
    }   
}

void ll_test_reset_black_list(void)
{
    LL_DEV_ADDR_LIST_ENTRY addr_entry_tmp;  
    UINT8 cam_addr_wl_base = LE_CAM_BLACK_LIST_BASE;
    UINT8 cam_addr;
    UINT8 i;

    ll_driver_reset_dev_list(LE_MSG_ADD_DEV_LIST);

    for (i = 0; i < LL_MAX_WHITE_LIST_SIZE; i++)
    {
        cam_addr = cam_addr_wl_base + (i << 1);      
        addr_entry_tmp.DWord[0] = ll_driver_read_cam(cam_addr);
        addr_entry_tmp.DWord[1] = ll_driver_read_cam(cam_addr + 1);

        LL_LOG_TRACE(GRAY, LE_MSG_CHK_WHITE_LIST_IN_CAM, 4, i, cam_addr,
                     addr_entry_tmp.DWord[0], addr_entry_tmp.DWord[1]);  
    }
}

void ll_test_black_list_set(void)
{    
    UINT32_S u32tmp[2];
    UINT8 i;
    
    /* init test addr entry via random register */
    for (i = 0; i < LL_MAX_BLACK_LIST_SIZE; i++)
    {
        LL_DRIVER_GEN_32BIT_RANDOM(u32tmp[0].u4Byte);
        LL_DRIVER_GEN_32BIT_RANDOM(u32tmp[1].u4Byte);        
        ll_test_addr_entry[i].type = u32tmp[0].u2Byte[0];
        *(UINT16*)&ll_test_addr_entry[i].addr[0] = u32tmp[0].u2Byte[1];
        *(UINT16*)&ll_test_addr_entry[i].addr[2] = u32tmp[1].u2Byte[0];        
        *(UINT16*)&ll_test_addr_entry[i].addr[4] = u32tmp[1].u2Byte[1];
    }

    for (i = 0; i < LL_MAX_BLACK_LIST_SIZE; i++)
    {
        LL_LOG_TRACE(GRAY, LE_MSG_ADD_DEV_LIST, 8, 
            i, ll_test_addr_entry[i].type, 
            ll_test_addr_entry[i].addr[0], ll_test_addr_entry[i].addr[1], 
            ll_test_addr_entry[i].addr[2], ll_test_addr_entry[i].addr[3], 
            ll_test_addr_entry[i].addr[4], ll_test_addr_entry[i].addr[5]);

        ll_driver_add_dev_to_list(LL_BLACK_LIST_TYPE, 
                                  ll_test_addr_entry[i].type, 
                                  ll_test_addr_entry[i].addr);        
    }   
}


void ll_test_sw_timer_callback(TIMER_ID timer_handle, void *index)
{
    UINT8 test_case = (UINT8)((UINT32)index);

    /* free software timer */
    if (ll_test_tid_timer != OS_INVALID_HANDLE)
    {
        OS_DELETE_TIMER(&ll_test_tid_timer);
    }

    switch (test_case)
    {
    case LL_TEST_CASE_DISABLE_ADVERTISING:
        ll_test_disable_advertisier();
#if 1
        /* test scanning state enable */
        ll_test_enable_scanning();

        /* test scanning state disable */
        ll_test_trigger_one_shot_event(LL_TEST_CASE_DISABLE_SCANNING, 4000);         
#endif
        break;
        
    case LL_TEST_CASE_DISABLE_SCANNING:
        ll_test_disable_scanning();
        break;
        
    case LL_TEST_CASE_CANCEL_CONNECTION_REQ:
        ll_test_cancel_connection();
        break;

    case LL_TEST_CASE_DISCONNET_REQ:
        ll_test_disconnect_connection();
        break;

    case LL_TEST_CASE_GEN_TX_DATA_PKT:
        ll_test_generate_tx_data_pkt();
        break;

    case LL_TEST_CASE_START_ENCRPT:
        ll_test_start_encryption();
        break;

    case LL_TEST_CASE_LTK_REQUEST_REPLY:
        ll_test_ltk_request_reply();
        break;
        
    default:
        break;
    }
}

void ll_test_trigger_one_shot_event(UINT32 test_case, UINT32 time_ms)
{    
    if (ll_test_tid_timer != OS_INVALID_HANDLE)
    {
        OS_DELETE_TIMER(&ll_test_tid_timer);
    }

    /* Create a tid timer */
    OS_CREATE_TIMER((UCHAR)ONESHOT_TIMER, &ll_test_tid_timer,
                   (OS_TIMEOUT_HANDLER)ll_test_sw_timer_callback,
                   (void *)((UINT32)test_case), 0);

    OS_START_TIMER(ll_test_tid_timer, time_ms);
}

void ll_test_acl_trx_init(void)
{
    ll_test_unit.acl_tx_len = 1;
    ll_test_unit.acl_rx_len = 1;                

    ll_test_unit.acl_tx_start_value = 0;
    ll_test_unit.acl_rx_start_value = 0;
    ll_test_unit.acl_rx_pkt_cnt_pass = 0;
    ll_test_unit.acl_rx_pkt_cnt_err = 0;
    ll_test_trigger_one_shot_event(LL_TEST_CASE_GEN_TX_DATA_PKT, 2000); 
}

void ll_test_main_adv_function(void)
{
    ll_test_unit.le_ce_cnt = 0; 

    /* test advertising state enable */
    ll_test_enable_advertisier();    
}

void ll_test_main_scan_function(void)
{
    ll_test_unit.le_ce_cnt = 0; 

    /* test scanning state enable */
    ll_test_enable_scanning();
}

void ll_test_main_initiator_function(void)
{
    ll_test_unit.le_ce_cnt = 0; 

    ll_test_rem_dev_addr_entry.type = 0;
    ll_test_rem_dev_addr_entry.addr[0] = 0x87;
    ll_test_rem_dev_addr_entry.addr[1] = 0x99;
    ll_test_rem_dev_addr_entry.addr[2] = 0x23;
    ll_test_rem_dev_addr_entry.addr[3] = 0x86;
    ll_test_rem_dev_addr_entry.addr[4] = 0x11;
    ll_test_rem_dev_addr_entry.addr[5] = 0x00;

    /* test initiator state enable */
    ll_test_create_connection();

//  ll_test_trigger_one_shot_event(LL_TEST_CASE_CANCEL_CONNECTION_REQ, 4000);
//  ll_test_trigger_one_shot_event(LL_TEST_CASE_DISCONNET_REQ, 8000);
}

void ll_test_generate_tx_data_pkt(void)
{
    UINT8 i;    
    LL_HCI_ACL_DATA_PKT *pkt_buf;

    /* allocate a data pkt buffer from le acl data pool */
    if (OS_GET_FREE_BUFFERS(le_acl_data_to_ll_pool_id) > 0)
    {
        OS_ALLOC_BUFFER(le_acl_data_to_ll_pool_id, (void **)(&pkt_buf));

        pkt_buf->connection_handle = LL_HCI_MIN_CONN_HANDLE;
        pkt_buf->packet_boundary_flag = (ll_test_unit.acl_tx_start_value & 0x01)? 
                                        L_CH_L2CAP_NON_FLUSH : L_CH_L2CAP_CONT;
        pkt_buf->broadcast_flag = 0;
        pkt_buf->acl_data_total_length = ll_test_unit.acl_tx_len;
        pkt_buf->next = NULL;

        for (i = 0; i < ll_test_unit.acl_tx_len; i++)
        {
            pkt_buf->hci_acl_data_pkt[i] = 
                                (ll_test_unit.acl_tx_start_value + i) & 0xff;
        }
        
        hci_td_handle_received_packet(HCI_TRANSPORT_LE_ACL_DATA_PKT_TYPE,
                                      pkt_buf, ll_test_unit.acl_tx_len + 4, 0);    

        /* set new value for next tx pkt */
        ll_test_unit.acl_tx_len++;
        if (ll_test_unit.acl_tx_len > LL_TEST_MAX_DATA_PKT_SIZE)
        {
            ll_test_unit.acl_tx_len = 1;    
        }

        ll_test_unit.acl_tx_start_value++;

        ll_test_trigger_one_shot_event(LL_TEST_CASE_GEN_TX_DATA_PKT, 10);  
    }
    else
    {
        ll_test_trigger_one_shot_event(LL_TEST_CASE_GEN_TX_DATA_PKT, 1000);        
    }    
}

void ll_test_receive_rx_data_pkt(LL_HCI_ACL_DATA_PKT *pkt_buf)
{
    UINT8 i;
    UINT8 err_cnt = 0;
    UINT8 exp_data;
    UINT8 actual_data;
    
    for (i = 0; i < ll_test_unit.acl_rx_len; i++)
    {
        exp_data = (ll_test_unit.acl_rx_start_value + i) & 0xff;
        actual_data = pkt_buf->hci_acl_data_pkt[i];
        
        if (exp_data != actual_data)
        {
            if (err_cnt < 3)
            {
                LL_LOG_TRACE(RED, LE_MSG_DATA_CH_RX_DATA_ERR_CHK, 4, 
                    ll_test_unit.acl_rx_len, i, exp_data, actual_data);   
            }
            
            err_cnt++;
        }
    }   

    if (err_cnt)
    {
        ll_test_unit.acl_rx_pkt_cnt_err++;
    }
    else
    {
        ll_test_unit.acl_rx_pkt_cnt_pass++;
    }

    LL_LOG_TRACE(GRAY, LE_MSG_DATA_CH_RX_DATA_PKT_RPT, 5, 
             ll_test_unit.acl_rx_len, err_cnt, ll_test_unit.le_ce_cnt,
             ll_test_unit.acl_rx_pkt_cnt_err, ll_test_unit.acl_rx_pkt_cnt_pass);        

    /* set new value for next tx pkt */
    ll_test_unit.acl_rx_len++;
    if (ll_test_unit.acl_rx_len > LL_TEST_MAX_DATA_PKT_SIZE)
    {
        ll_test_unit.acl_rx_len = 1;
    }
    ll_test_unit.acl_rx_start_value++;
    
    OS_FREE_BUFFER(le_acl_data_to_host_pool_id, pkt_buf);
}

void ll_test_main_function(void)
{
    /* clear exit scanning state interrupt */
//    WR_LE_REG(LE_REG_RF_TEST_CONTROL, (1 << 15) | (0 << 14) | (30 << 8));    
    //WR_LE_REG(LE_REG_RF_TEST_CONTROL, (1 << 15) | (1 << 14));  
#if 0
    /* test white list access */
    ll_test_white_list_set();
    ll_test_reset_white_list();    

    /* test black list access */
    ll_test_black_list_set();
    ll_test_reset_black_list();

    /* test advertising state enable */
    ll_test_enable_advertisier();

    /* test advertising state disable */
    ll_test_trigger_one_shot_event(LL_TEST_CASE_DISABLE_ADVERTISING, 4000);

#if 0
    /* test scanning state enable */
    ll_test_enable_scanning();

    /* test scanning state disable */
    ll_test_trigger_one_shot_event(LL_TEST_CASE_DISABLE_SCANNING, 4000);        
#endif

#else

    UINT8 rvalue;
    rvalue = RD_LE_REG(0x38);

    if (rvalue == 1)
    {
        ll_test_main_scan_function();
    }
    else if (rvalue == 2)
    {
        ll_test_main_initiator_function();
    }
    else 
    {
        ll_test_main_adv_function();
    }
#endif

}

