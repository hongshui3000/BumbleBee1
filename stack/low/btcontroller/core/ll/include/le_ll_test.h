#ifndef LE_LL_TEST_H_
#define LE_LL_TEST_H_

#define TEST_LL_SECURITY_FTP

#ifdef TEST_LL_SECURITY_FTP
#define LL_TEST_MAX_DATA_PKT_SIZE       LL_MAX_ACL_SECURITY_PKT_SIZE
#else
#define LL_TEST_MAX_DATA_PKT_SIZE       LL_MAX_ACL_UNSECURITY_PKT_SIZE
#endif

#define LL_TEST_ADV_DATA_LEN            15
#define LL_TEST_SCAN_RSP_DATA_LEN       31
#ifdef FOR_SIMULATION
#define LL_TEST_ADV_INTERVAL_MIN        80 //0.625 ms * 320 = 200 ms
#define LL_TEST_ADV_INTERVAL_MAX        100 //0.625 ms * 1610 = 1000 ms 
#else
#define LL_TEST_ADV_INTERVAL_MIN        320 //0.625 ms * 320 = 200 ms
#define LL_TEST_ADV_INTERVAL_MAX        1610 //0.625 ms * 1610 = 1000 ms 
#endif

#define LL_TEST_ADV_TYPE                LL_HCI_ADV_TYPE_ADV_IND
#define LL_TEST_OWN_ADDR_TYPE           LL_ADDR_TYPE_PUBLIC
#define LL_TEST_DIRECT_ADDR_TYPE        LL_ADDR_TYPE_PUBLIC
#define LL_TEST_ADV_CHAN_MAP            0x07
#define LL_TEST_ADV_FILTER_POLICY       LL_HCI_ADV_FILT_POLICY_ANY_SCAN_ANY_CONN

#define LL_TEST_SCAN_TYPE               1 /* 1: active scan 0: passive scan */
#define LL_TEST_SCAN_INTERVAL           0x1000 // 0.625 ms * 4096 = 2.56 s
#define LL_TEST_SCAN_WINDOW             0x0800 // 0.625 ms * 2048 = 1.28 s
#define LL_TEST_SCAN_FILTER_POLICY      0
#define LL_TEST_SCAN_FILTER_DUPLICATES  0

#ifdef FOR_SIMULATION
#define LL_TEST_INIT_SCAN_INTERVAL      30 // 0.625 ms * 4096 = 2.56 s    
#define LL_TEST_INIT_SCAN_WINDOW        20 // 0.625 ms * 2048 = 1.28 s
#else
#define LL_TEST_INIT_SCAN_INTERVAL      0x1000 // 0.625 ms * 4096 = 2.56 s    
#define LL_TEST_INIT_SCAN_WINDOW        0x0800 // 0.625 ms * 2048 = 1.28 s
#endif

#define LL_TEST_INIT_FILTER_POLICY      0

#ifdef FOR_SIMULATION
#define LL_TEST_CONN_INTERVAL_MIN       20 // 1.25 ms * 2048 = 2.56 s
#else
#define LL_TEST_CONN_INTERVAL_MIN       0x0200 // 1.25 ms * 2048 = 2.56 s
#endif
#define LL_TEST_CONN_INTERVAL_MAX       0x0400 // 1.25 ms * 0x0C80 = 4 s
#define LL_TEST_CONN_LATENCY            0
#define LL_TEST_SUPERVISION_TIMEOUT     1000   // 10 ms * 1000 = 10 s
#ifdef FOR_SIMULATION
#define LL_TEST_MIN_CE_LENGTH           30    // 0.625 ms * 10 = 62.5 ms
#else
#define LL_TEST_MIN_CE_LENGTH           100    // 0.625 ms * 10 = 62.5 ms
#endif
#define LL_TEST_MAX_CE_LENGTH           1000   // 0.625 ms * 1000 = 625 ms 

enum LL_TEST_CASE_TYPE_
{
    LL_TEST_CASE_DISABLE_ADVERTISING,
    LL_TEST_CASE_DISABLE_SCANNING,
    LL_TEST_CASE_CANCEL_CONNECTION_REQ,
    LL_TEST_CASE_DISCONNET_REQ,
    LL_TEST_CASE_GEN_TX_DATA_PKT,
    LL_TEST_CASE_START_ENCRPT,
    LL_TEST_CASE_LTK_REQUEST_REPLY,    
};

typedef struct LL_TEST_UNIT_ {
    UINT8 acl_tx_len;
    UINT8 acl_tx_start_value;
    UINT8 acl_rx_len;
    UINT8 acl_rx_start_value;
    UINT32 acl_rx_pkt_cnt_pass;
    UINT32 acl_rx_pkt_cnt_err;
    UINT32 le_ce_cnt; 
} LL_TEST_UNIT, *PLL_TEST_UNIT;

extern LL_TEST_UNIT ll_test_unit;

void ll_test_enable_advertisier(void);
void ll_test_disable_advertisier(void);
void ll_test_enable_scanning(void);
void ll_test_enable_scanning(void);
void ll_test_main_function(void);
void ll_test_trigger_one_shot_event(UINT32 test_case, UINT32 time_ms);
void ll_test_generate_tx_data_pkt(void);
void ll_test_acl_trx_init(void);
void ll_test_receive_rx_data_pkt(LL_HCI_ACL_DATA_PKT *pkt_buf);
void ll_test_start_encryption(void);
void ll_test_ltk_request_reply(void);

#endif

