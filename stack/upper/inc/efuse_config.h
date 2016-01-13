/*
 *  Routines to access hardware
 *
 *  Copyright (c) 2013 Realtek Semiconductor Corp.
 *
 *  This module is a confidential and proprietary property of RealTek and
 *  possession or use of this module requires written permission of RealTek.
 */

#ifndef _EFUSE_CONFIG_H_
#define _EFUSE_CONFIG_H_
#include <rtl_types.h>

#include <blueapi_types.h>

typedef struct BT_EFUSE_UPPERSTACK_CFG_
{
    uint8_t upper_stack_en: 1;
    uint8_t num_link_don: 2;
    uint8_t num_link_doff: 2;
    uint8_t role_master_link: 1;
    uint8_t role_slave_link: 1;
    uint8_t support_exchange_mtu: 1;

    uint8_t gatt_max_service_count;
    uint16_t att_max_mtu_size;

    uint8_t ota_timeout_total;
    uint8_t ota_timeout_wait4_conn;
    uint8_t ota_timeout_wait4_image_transfer;

    uint8_t ota_use_randon_address:1;
    uint8_t ota_adv_with_image_version:1;
    uint8_t ota_with_encryption_data:1;
    uint8_t ota_with_encryption_aes_type:2;
    uint8_t dlps_check_timer:1;
    uint8_t ota_reserved:2;
    
    uint8_t ota_adv_random_address[6];


    uint8_t dg3_en:1;
    uint8_t num_le_data_chan_don: 3;
    uint8_t num_le_data_chan_doff: 3;
    uint8_t support_key_store:1;
    uint8_t gatt_max_tx_wsize;

    uint8_t UpsteamShortBufferCount;
    uint8_t UpstreamLongBufferCount;
    uint8_t DownsteamShortBufferCount;
    uint8_t DownstreamLongBufferCount;
    uint16_t UpsteamShortBufferLength;
    uint16_t UpstreamLongBufferLength;
    uint16_t DownsteamShortBufferLength;
    uint16_t DownstreamLongBufferLength;
    uint32_t traceMask;
    uint8_t le_data_max_tx_wsize;
    uint8_t gattServerCCCbitsCount;

    uint8_t num_data_psm: 4;           //randy add 
    uint8_t num_data_rfcom_dlci_don:2;
    uint8_t num_data_rfcom_dlci_doff:2;

    uint8_t num_l2c_chan_don: 4;
    uint8_t num_l2c_chan_doff: 4;

    uint8_t  bqb_en : 1;
    uint8_t  reserved1 : 7;

    uint8_t  reserved2[3];
} BT_EFUSE_UPPERSTACK_CFG;

/*
fix me: suppose to be union
*/
typedef struct otp_struct
{
	//union{
   uint8_t gEfuse_UpperStack_d8[37];      //EFUSE[B4~D7]
   BT_EFUSE_UPPERSTACK_CFG gEfuse_UpperStack_s;
	//};
} OTP_STRUCT;



void efuse_config_load(void);


#endif
