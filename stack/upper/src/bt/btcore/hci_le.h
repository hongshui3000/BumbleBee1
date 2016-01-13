/**
********************************************************************************************************
Copyright (c) 2015, Realtek Semiconductor Corporation. All rights reserved.
*********************************************************************************************************
* @file       hci_le.h
* @brief     HCI Protocol Layer (Low Energy)
* @details   
*
* @author  	gordon
* @date      	2015-07-13
* @version	v0.1
*/

#ifndef __HCI_LE_H
#define __HCI_LE_H

#include <flags.h>
#include <rtl_types.h>


#ifdef __cplusplus
extern "C" {
#endif

/** Function declarations */
void hciLEHandleDisconnectionComplete(uint16_t Handle, uint16_t Reason);
void hciLEProcessCommandComplete(uint8_t * fp, uint16_t response, uint16_t status);
void hciLEProcessCommandStatus(uint16_t response, uint16_t status);
void hciLEProcessEventPacket(uint8_t * fp, uint16_t pos, uint16_t len);


#ifdef __cplusplus
}
#endif

#endif
