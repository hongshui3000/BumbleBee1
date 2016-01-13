/**
*****************************************************************
*	Copyright (c) 2015, Realtek Semiconductor Corporation. All rights reserved.
*****************************************************************
* @file       hci_llif.h
* @brief     hci layer interface
* @details   
*
* @author   	gordon
* @date      	2015-07-13
* @version	v0.1
*/

#if !defined(__HCI_LLIF_H)
#define      __HCI_LLIF_H

#include <flags.h>
#include <os_message.h>

void hciLLOpen(void);
void hciLLClose(void);
void hciLLWrite(MESSAGE_P pMsg);

#if UPPER_STACK_USE_VIRTUAL_HCI
void hciLLResponse(PVOID pBuffer);
#endif

#endif /**< !defined(__HCI_LLIF_H) */
