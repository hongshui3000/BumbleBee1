/***************************************************************************
 Copyright (C) MindTree Ltd.
 This module is a confidential and proprietary property of MindTree and
 a possession or use of this module requires written permission of MindTree.
 ***************************************************************************/

#ifndef _BZ_APPL_H_
#define _BZ_APPL_H_

/**
 * \file bz_appl.h
 *  Bluetooth Application related interface definitions.
 *
 * \author Santhosh kumar M
 * \date 2008-01-07
 */

/* ========================= Include File Section ========================= */
#include "platform.h"
#include "bz_config.h"


/* ====================== Macro Declaration Section ======================= */

#define MAX_HCI_CMD_LEN         256
#define MAX_HCI_ACL_LEN         (((HCI_ACL_DATA_REPORT_SIZE/4)+1) * 4)

#define MB_EXT_SIZE         4
#define MB0_SIZE            (260 + MB_EXT_SIZE) /* mem buf */
#define MB1_SIZE            (760 + MB_EXT_SIZE) /* For Profile Msgs and for
                                                   HCI ACL to BB */

/* The minimum requirement of hci-lc/bb layer buffer size is 258bytes => 260
 * bytes. The event parameter can have up to 255 bytes (e.g. Remote Name
 * Inquiry => 1+6+248 (status+bdaddress+name). The event header requires 2
 * bytes - opcode + len. So the total required length of hci event/cmd is
 * 255+7 => 257 bytes. In our implementation we use 1 bytes for message type.
 * So the total required length of BT_MEM_BUF_TYPE_0 buffer is 258bytes =>
 * 260bytes(round off to 4 bytes) We need to define
 */
#define MB0_NUM             8 /* 10 */
#define MB1_NUM             2 /* 4  */

#define BT_MEM_BUF_TYPE_0   0  /* used by BT_alloc_mem_ucos as a source of a
                                  medium buffer. Also used by a Host router
                                  (hci_send_packet_to_host()) in the Profile
                                  mode */
#define BT_MEM_BUF_TYPE_1   1  /* for host events, Host router
                                  (hci_transport_write_data() in Profile mode.
                                  Used for User_API_handle_received_char() */

#define BT_MEM_BUF_TYPE_NUM 2

/* Types of Mem buffers */
#define BT_OS_MEM_BUF_TYPE_MAX  BT_MEM_BUF_TYPE_NUM
#define BT_OS_MSG_TYPE_MAX      BT_MEM_BUF_TYPE_NUM

#ifndef MINT_OS_EMBEDDED
#define BT_FW_STK_APPL_HEAP_REQ           \
    (MB0_NUM*(MB0_SIZE+MEM_POOL_ITEM_SZ)) + \
    (MB1_NUM*(MB1_SIZE+MEM_POOL_ITEM_SZ))
#else
#define BT_FW_STK_APPL_HEAP_REQ  0x0
#endif

#ifndef MINT_OS_EMBEDDED
#define BT_FW_STK_APPL_MEM_POOL_REQ  BT_OS_MEM_BUF_TYPE_MAX
#else
#define BT_FW_STK_APPL_MEM_POOL_REQ  0x0
#endif

#define LED_ON          0x01
#define LED_OFF         0x00
#define BLINK_MODE     0
#define STOP_BLINKING 0x03
#define START_BLINKING 0x04
#define BLINK_1 0   /* Double Blinking */
#define BLINK_2 1   /* Fast Blinking */
#define BLINK_3 2   /* Slow Blinking */
#define BLINK_4 3   /* Middle Speed Blinking */
#define BLINK_5 4   /* Tripple Blinking */
#define BLINK_6 5   /* Change fast blinking OFF */
#define BLINK_7 6
#define BLINK_8 7


/* ==================== Data Types Declaration Section ==================== */
#ifdef BU9460
enum BT_MSG_TYPE_T {
    BT_MSG_HCI = 0x01,              /* 0x01 ommand/event */
    BT_MSG_ACL,                     /* data packet */
    BT_HOST_EVT,                    /* event is sent to Host Poll task from Write Queue */     
    BT_MSG_UL_TIMEOUT,               /* Timeout */    
    
    /* USER DEFINED  MESSAGE SIGNALS */
    BT_MSG_GPIO_TAP_0,              /* GPIO 0 Tap */
    BT_MSG_GPIO_PRESS_0,            /* GPIO 0 Press */
    BT_MSG_GPIO_REL_0,            /* GPIO 0 Long Press/Release */
    BT_MSG_GPIO_TAP_1,              /* GPIO 1 Tap */
    BT_MSG_GPIO_PRESS_1,            /* GPIO 1 Press */
    BT_MSG_GPIO_REL_1,            /* GPIO 1 Long Press/Release */
    BT_MSG_GPIO_TAP_2,              /* GPIO 2 Tap */
    BT_MSG_GPIO_PRESS_2,             /* GPIO 2 Press */
    BT_MSG_GPIO_REL_2,             /* GPIO 2 Long Press/Release */
    BT_MSG_GPIO_TAP_3,              /* GPIO 2 Tap */
    BT_MSG_GPIO_PRESS_3,             /* GPIO 2 Press */
    BT_MSG_GPIO_REL_3,             /* GPIO Long Press/Release */
/* seike add */
	BT_MSG_GPIO_RELEASE             /* cheack release */
/* seike add end */
};
#else
enum BT_MSG_TYPE_T {
    BT_MSG_HCI = 0x01,              /* 0x01 ommand/event */
    BT_MSG_ACL,                     /* data packet */
    BT_HOST_EVT,                    /* event is sent to Host Poll task from Write Queue */     
    BT_MSG_UL_TIMEOUT,               /* Timeout */    

    BT_MSG_GPIO_TAP_0,              /* GPIO 0 Tap */
    BT_MSG_GPIO_PRESS_0,            /* GPIO 0 Press */
    BT_MSG_GPIO_REL_0,            /* GPIO 0 Long Press/Release */
    BT_MSG_GPIO_TAP_1,              /* GPIO 1 Tap */
    BT_MSG_GPIO_PRESS_1,            /* GPIO 1 Press */
    BT_MSG_GPIO_REL_1,            /* GPIO 1 Long Press/Release */
    BT_MSG_GPIO_TAP_2,              /* GPIO 2 Tap */
    BT_MSG_GPIO_PRESS_2,             /* GPIO 2 Press */
    BT_MSG_GPIO_REL_2,             /* GPIO 2 Long Press/Release */
    BT_MSG_GPIO_TAP_3,              /* GPIO 2 Tap */
    BT_MSG_GPIO_PRESS_3,             /* GPIO 2 Press */
    BT_MSG_GPIO_REL_3,             /* GPIO Long Press/Release */
};
#endif

/* ============================= API Section ============================== */

#endif  /* _BZ_APPL_H_ */

