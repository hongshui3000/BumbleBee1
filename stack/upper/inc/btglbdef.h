/**
********************************************************************************************************
Copyright (c) 2015, Realtek Semiconductor Corporation. All rights reserved.
*********************************************************************************************************
* @file        btglbdef.h
* @brief      Bluetooth specific global definitions
* @details   
*
* @author   	gordon
* @date      	2015-07-13
* @version	v0.1
*/

#ifndef __BTGLBDEF_H
#define __BTGLBDEF_H

#include <rtl_types.h>

#ifdef __cplusplus
extern "C" {
#endif


/** windows/comtask/L1ASYNC-COMAPI need 4 additional bytes offset */
#define BT_L1_HCI_DS_OFFSET 4


/** Header Length for HCI including HCI Header Type */
#define EVENT_HDR_LENGTH 3 /**< packet type (1), event type (1), length (1) */
#define ACL_HDR_LENGTH   5 /**< packet type (1), handle (2), length (2) */
#define SCO_HDR_LENGTH   4 /**< packet type (1), handle (2), length (1) */
#define CMD_HDR_LENGTH   4 /**< packet type (1), command (2), length (1) */

#define L2CAP_CLOSE_DELAY   15000      /**< delay in ms before L2CAP clears down the HCI link when no open connection */



#ifdef __cplusplus
}
#endif

#endif /**< __BTGLBDEF_H */
