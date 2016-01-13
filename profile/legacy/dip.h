/**
 *****************************************************************************************
 *               Copyright(c) 2015, Realtek Semiconductor Corporation. All rights reserved.
 *****************************************************************************************
 * @file       dip.h
 * @brief     Head file for device id profile
 * @detail
 * @author   kyle_xu
 * @date      2015-12-07
 * @version   v0.1
 * ****************************************************************************************
 */

#ifndef __DIP_H__
#define __DIP_H__

#ifdef  __cplusplus
extern "C" {
#endif      /* __cplusplus */

bool dip_Init(uint16_t vendor_id, uint16_t id_source, uint16_t product_id, uint16_t product_version);

#ifdef  __cplusplus
}
#endif      /*  __cplusplus */
#endif
