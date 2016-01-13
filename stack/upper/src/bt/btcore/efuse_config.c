#include <efuse_config.h>
#include <flags.h>
#include <diag.h>
#include <trace_binary_mod.h>

//#pragma anon_unions


OTP_STRUCT otp_str_data;

void efuse_config_load(void)
{

#if 1
/************************************************ Upper Stack start ***********************************/

/**
        {0x5b,0x08,0xb8,0x00,0xb4,0x3c,0x64,0x00,
          0xef,0x99,0x23,0x4c,0xe0,0x00,0x80,0x08,
          0x08,0x00,0x08,0x08,0xd4,0x00,0x78,0x00,
          0x3c,0x00,0xd0,0x00,0x22,0x22,0x22,0x22,
          0xaa,0x00,0x00
        }

*/
    otp_str_data.gEfuse_UpperStack_s.upper_stack_en = 1;
    otp_str_data.gEfuse_UpperStack_s.num_link_don = 1;
    otp_str_data.gEfuse_UpperStack_s.num_link_doff = 1;
    otp_str_data.gEfuse_UpperStack_s.role_master_link = 0;
    otp_str_data.gEfuse_UpperStack_s.role_slave_link = 1;
    otp_str_data.gEfuse_UpperStack_s.support_exchange_mtu = 0;
    otp_str_data.gEfuse_UpperStack_s.att_max_mtu_size = 0xB8; //BT_LOCAL_MAX_MTU_SIZE;//0xB8
    otp_str_data.gEfuse_UpperStack_s.gatt_max_service_count = 8;

    otp_str_data.gEfuse_UpperStack_s.ota_timeout_total = 180;//180s, timeout will cause NVIC_SystemReset().
    otp_str_data.gEfuse_UpperStack_s.ota_timeout_wait4_conn = 60;//60s, time from enter ota mode to connection established, need to reset when connected.
    otp_str_data.gEfuse_UpperStack_s.ota_timeout_wait4_image_transfer = 100;//100s, time from connection established to ota finish,timeout will cause NVIC_SystemReset().

    otp_str_data.gEfuse_UpperStack_s.ota_use_randon_address = 0;
    otp_str_data.gEfuse_UpperStack_s.ota_adv_random_address[0] = 0xEF;
    otp_str_data.gEfuse_UpperStack_s.ota_adv_random_address[1] = 0x99;
    otp_str_data.gEfuse_UpperStack_s.ota_adv_random_address[2] = 0x23;
    otp_str_data.gEfuse_UpperStack_s.ota_adv_random_address[3] = 0x4C;
    otp_str_data.gEfuse_UpperStack_s.ota_adv_random_address[4] = 0xE0;
    otp_str_data.gEfuse_UpperStack_s.ota_adv_random_address[5] = 0x00;
    
    
    otp_str_data.gEfuse_UpperStack_s.dg3_en = 0;
    otp_str_data.gEfuse_UpperStack_s.num_le_data_chan_don = 0;
    otp_str_data.gEfuse_UpperStack_s.num_le_data_chan_doff = 0;
    otp_str_data.gEfuse_UpperStack_s.support_key_store = 0;
    otp_str_data.gEfuse_UpperStack_s.gatt_max_tx_wsize = BT_GATT_DS_WSIZE_COUNT;

    otp_str_data.gEfuse_UpperStack_s.UpsteamShortBufferCount = BT_US_BUFFER_COUNT;//BT_US_SHORT_BUFFER_COUNT;
    otp_str_data.gEfuse_UpperStack_s.UpsteamShortBufferLength =BT_US_BUFFER_SIZE;// BT_US_SHORT_BUFFER_SIZE;
    otp_str_data.gEfuse_UpperStack_s.UpstreamLongBufferCount = 0;//BT_US_LONG_BUFFER_COUNT;
    otp_str_data.gEfuse_UpperStack_s.UpstreamLongBufferLength = 120;//BT_US_LONG_BUFFER_SIZE;

    otp_str_data.gEfuse_UpperStack_s.DownsteamShortBufferCount =12;// BT_DS_SHORT_BUFFER_COUNT;
    otp_str_data.gEfuse_UpperStack_s.DownsteamShortBufferLength =212;// BT_DS_SHORT_BUFFER_SIZE;
    otp_str_data.gEfuse_UpperStack_s.DownstreamLongBufferCount =BT_DS_BUFFER_COUNT;// BT_DS_LONG_BUFFER_COUNT;
    otp_str_data.gEfuse_UpperStack_s.DownstreamLongBufferLength =BT_US_BUFFER_SIZE;// BT_DS_LONG_BUFFER_SIZE;
    otp_str_data.gEfuse_UpperStack_s.traceMask =  HCI_TRACE_MASK_ERROR|
                                    L2CAP_TRACE_MASK_ERROR|
                                    GATT_TRACE_MASK_ERROR|    
                                    BTSECMAN_TRACE_MASK_ERROR|
                                    BLUEFACE_TRACE_MASK_ERROR|BLUEFACE_TRACE_MASK_TRACE|
                                    BLUEAPI_TRACE_MASK_ERROR|BLUEAPI_TRACE_MASK_TRACE|BLUEAPI_TRACE_MASK_MESSAGE|
                                    RFCOMM_TRACE_MASK_ERROR| 
                                    SDP_TRACE_MASK_ERROR
                                    ;
    otp_str_data.gEfuse_UpperStack_s.le_data_max_tx_wsize = 0;
    otp_str_data.gEfuse_UpperStack_s.gattServerCCCbitsCount = 8;


    otp_str_data.gEfuse_UpperStack_s.num_data_psm = 6;
    otp_str_data.gEfuse_UpperStack_s.num_data_rfcom_dlci_don = 2;
    otp_str_data.gEfuse_UpperStack_s.num_data_rfcom_dlci_doff = 2;

    otp_str_data.gEfuse_UpperStack_s.num_l2c_chan_don = 3;
    otp_str_data.gEfuse_UpperStack_s.num_l2c_chan_doff = 0;

    otp_str_data.gEfuse_UpperStack_s.bqb_en = 0;


/************************************************ Upper Stack End ***********************************/
#endif
}
