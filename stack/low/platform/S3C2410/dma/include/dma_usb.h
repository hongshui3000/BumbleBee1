/***************************************************************************
 Copyright (C) RealTek Ltd.
 This module is a confidential and proprietary property of RealTek and
 a possession or use of this module requires written permission of RealTek.
 ***************************************************************************/

#ifndef __DMA_USB_H__
#define __DMA_USB_H__
//#include "Bt_fw_config.h"
#include "DataType.h"
#include "platform.h"
#include "bt_fw_config.h"
//#ifdef _ENABLE_BTON_POWER_SAVING_ 
#if (defined (_ENABLE_BTON_POWER_SAVING_) || defined(_RTK8723_UART_INIT_))
#include "power_control.h"
#endif


#define BT_DMA_REG_BASE_ADDR        				0x40055000

/*
 * Bits |	++Descriptin++
 * 15:0 |	ACL TX buffer start address, must be 4-byte aligned.
 */
#define ACL_TX_BUF_START_ADDR_REG       				0x00

/*
 * Bits |	++Descriptin++
 * 15:0 |	ACL RX descriptor ring base address, must be 4-byte aligned.
**/
#define ACL_RX_DES_RING_BASE_REG        				0x04

/*
 * Bits |	++Descriptin++
 * 15:0 |	Synchronous TX buffer start address, must be 4-byte aligned.
**/
#define SCO_TX_BUF_START_ADDR_REG       				0x08

/*
 * Bits |	++Descriptin++
 * 15:0 |	Synchronous RX descriptor ring base address, must be 4-byte aligned.
**/
#define SCO_RX_DES_RING_BASE_REG        				0x0C

/*
 * Bits |	++Descriptin++
 * 15:0 |	Command buffer start address, must be 4-byte aligned.
**/
#define CMD_TX_BUF_START_ADDR_REG       				0x10

/*
 * Bits |	++Descriptin++
 * 15:0 |	Event descriptor ring base address, must be 4-byte aligned.
**/
#define EVENT_DES_RING_BASE_REG         				0x14

/*
 * Bits |	++Descriptin++
 * 15:0 |   LE ACL TX buffer start address, must be 4-byte aligned
**/
#define LE_ACL_TX_BUF_START_ADDR_REG    				0x18

/*
 * Bits |	++Descriptin++
 * 31:24|   packet offset (unit in 8 byte)from packet entry boundary for ACL tx (from 8723d)
 * 23:21|   Reserved
 * 20:16|	Total number of ACL TX data packets supported, maximum is 16. 
 * 11:0 |	ACL TX buffer entry size for each packet storage, must be 4-byte aligned. Maximum is 4092.
**/
#define ACL_TX_BUF_SIZE_REG             				0x20

/*
* Bits  |	++Descriptin++
* 31:24 |   packet offset (unit in 8 byte)from packet entry boundary for SCO tx (from 8723d)
* 23:21 |   Reserved
* 20:16 |	Total number of synchronous TX data packets supported, maximum is 16.
* 8:0   |	Synchronous TX buffer entry size for each packet storage, must be 4-byte aligned. Maximum 260.
**/
#define SCO_TX_BUF_SIZE_REG             				0x24

/*
 * Bits |	++Descriptin++
 * 31:24|   packet offset (unit in 8 byte)from packet entry boundary for CMD tx (from 8723d)
 * 23:19|   Reserved
 * 18:16|	Total number of command packets supported, maximum is 8.
 * 8:0  |	Command TX buffer entry size for each packet storage, must be 4-byte aligned. Maximum is 260.
**/
#define CMD_BUF_SIZE_REG                				0x28

/*
 * Bits |	++Descriptin++
 * 31:24|   packet offset (unit in 8 byte)from packet entry boundary for LE ACL tx (from 8723d)
 * 23:21|   Reserved 
 * 20:16|	Total number of LE ACL TX data packets supported, maximum is 16.
 * 15:12|   Reserved
 * 11:0 |	LE buffer size is fixed to be 2048 bytes. (after 8723d)
 * 8:0  |   LE buffer size is fixed to be 64 bytes.   (before 8723d)
**/
#define LE_BUF_SIZE_REG                 				0x2C

/*
 * Bits |	++Descriptin++
 * 15:0 |	ACL TX buffer free notification
**/
#define ACL_TX_BUF_FREE_REG             				0x30

/*
 * Bits |	++Descriptin++
 * 15:0 |	Synchronous TX buffer free notification
**/
#define SCO_TX_BUF_FREE_REG             				0x34

/*
 * Bits |	++Descriptin++
 * 31   |	FW write 1 to notify DMA that synchronous RX descriptor ring has valid data. 
 * 30   |	FW write 1 to notify DMA that ACL RX descriptor ring has valid data
 * 29   |	FW write 1 to notify DMA that event descriptor ring has valid data.
 * 28:8 |	Reserved
 * 7:0  |	Command buffer free notification. 
**/
#define CMD_TX_BUF_FREE_RX_START_CTL_REG				0x38

/*
 * Bits |	++Descriptin++
 * 15:0 |	LE ACL TX buffer free notification
**/
#define LE_TX_BUF_FREE_REG              				0x3C

/*
 * Bits |	++Descriptin++
 * 3:0  |	FW read this register to obtain incoming ACL TX packet index
**/
#define ACL_TX_ORDER_FIFO_REG           				0x40

/*
* Bits  |	++Descriptin++
 * 3:0  |	FW read this register to obtain incoming synchronous TX packet index
**/
#define SCO_TX_ORDER_FIFO_REG           				0x44

/*
 * Bits |	++Descriptin++
 * 3:0  |	FW read this register to obtain incoming command packet index
**/
#define CMD_TX_ORDER_FIFO_REG           				0x48

/*
 * Bits |	++Descriptin++
 * 3:0  |	FW read this register to obtain incoming LE ACL TX packet index
**/
#define LE_ACLT_TX_ORDER_FIFO_REG   					0x4C

/*
 * Bits |	++Descriptin++
 * 31   |	Iso out token timeout status
 * 30   |	Iso in token timeout status
 * 26   |	Evt token timeout status 
 * 24:22|	Number of incoming ACL TX packets.
 * 21:19|	Number of incoming command packets.
 * 18:14|	Number od incoming LE ACL TX packets.
 * 13:8 |	Reserved
 * 7    |	USB Iso alternative setting changed interrupt status.
 * 6    |	LE ACL TX packet arrive interrupt status.
 * 5    |	Synchronous RX done interrupt status.
 * 4    |	ACL RX done interrupt status. 
 * 3    |	Event RX done interrupt status.
 * 2    |	Synchronous TX packet arrive interrupt status.
 * 1    |	ACL TX packet arrive interrupt status.
 * 0    |	Command packet arrive interrupt status.
**/
#define ISR_STATUS_REG                  				0x50

/*
 * Bits |	++Descriptin++
 * 31   |	Iso out token timeout interrupt enable
 * 30   |	Iso in token timeout interrupt enable
 * 26   |	Evt token timeout interrupt enable 
 * 7    |	USB Iso alternative setting changed interrupt enable
 * 6    |	LE ACL TX packet arrive interrupt enable
 * 5    |	Synchronous RX done interrupt enable.
 * 4    |	ACL RX done interrupt enable.
 * 3    |	Event RX done interrupt enable.
 * 2    |	Synchronous TX packet arrive interrupt enable.
 * 1    |	ACL TX packet arrive interrupt enable.
 * 0    |	Command packet arrive interrupt enable.
**/
#define ISR_ENABLE_REG                  				0x54

/*
 * Bits |	**Descriptin**
 * 31   |	FW write 1 to reset DMA, all pending transfers will be dropped, FIFO pointers and registers will be reset to default values.
 * 30:27|	Reserved
 * 26   |	Control when will Synchronous RX done interrupt trigger.
 * 25   |	Control when will ACL RX done interrupt trigger.
 * 24   |	Control when will Event RX done interrupt trigger.
 * 23:20|	Reserved
 * 19:16|	Control number of accumulated incoming synchronous TX packets that will trigger interrupt.
 * 15:9 |	Reserved
 * 8    |   LE data length extension enable, default 0 (from 8723d)
 * 7    |   select tx hcicmdlen from the length field of usb control transfer's setup stage (1) or data stage (0)
 * 6    |	Enable LE ACL TX packet Buffer
 * 5    |	PCIe Vendor Message filed enable
 * 4    |	Control whether synchronous packets TX and RX DMA operations are turned on or not.
 * 3    |	Current DMA TX operation state.
 * 2    |	After FW change this bit, it must read DMA_TX_ST to know when DMA actually changes state.
 * 1    |	Current DMA RX operation state.
 * 0    |	After FW change this bit, it must read DMA_RX_ST to know when DMA actually changes state.
**/
#define DMA_CTL_REG                     				0x58


/*
* 31:12|   Reserved
* 11:0 |   Largeest connection handle value of LE ACL.
**/
#define LE_CONNECTION_HANDLE_UPPER_REG                  0x5C

/*
 * Bits |	++Descriptin++
 * 31:29|	The value of alternate setting in USB isochronous interface descriptor. 
 * 28   |	Enable fileter for connection handle 2.
 * 27:16|	Connection handle 2 for HCI DMA filter.
 * 15:13|	HCI synchronous packet boundary error recovery mode setting
 * 12   |	Enable fileter for connection handle 1.
 * 11:0 |	Connection handle 1 for HCI DMA filter.
**/
#define SCO_CONNECTION_HANDLE_1_AND_2_REG   				0x60

/*
 * Bits    |    ++Descriptin++
 * 31:29|      Reserved 
 * 28      |     Enable fileter for connection handle of LE.
 * 27:16|	Smallest connection handle value of LE ACL.
 * 15:13|	Reserved
 * 12     |       Enable fileter for connection handle 3.
 * 11:0 |	Connection handle 3 for HCI DMA filter.
**/
#define SCO_CONNECTION_HANDLE_3_REG         				0x64

/*	Added by Wallice for ehanced HCI DMA.	2011/12/06	*/
#define ACL_TX_HCI_SIZE_LIMIT_REG         					0x68

#define LE_TX_HCI_SIZE_LIMIT_REG         					0x6C

#define USB_ACL_TX_LENERR_EARLY_PKTINDEX_REG         	0x78

/*	End Added by Wallice for ehanced HCI DMA.	2011/12/06	*/

#ifdef _SUPPORT_USB_TIMEOUT_INTERRUPT_
#define USB_TOKEN_TIMEOUT_REG                               0x7C
#endif

#define DMA_UART_STS_REG                                0x20

#define DMA_UART_SOFEWARE_RESET_REG                     0x150  // struct: DMA_UART_SOFEWARE_RESET_REG_S


/* sie reg define */
#ifdef _SUPPORT_USB_TIMEOUT_INTERRUPT_
#define RX_EVT_REQ_LEN_REG          0x50    // 16 bit
#define SIE_BUFF_OK_REG             0x52    // 8 bit
#define RX_EVT_BUF_W_ADDR_REG       0x53    // 8 bit

#define RX_ACL_REQ_LEN_REG          0x74    // 16 bits
#define RX_ACL_BUF_W_ADDR_LOW_REG   0x76    // 9 bit
#define RX_SCO_REQ_LEN_REG          0x78    // 16 bits
#define RX_SCO_BUF_W_ADDR_LOW_REG   0x7A    // 9 bit
#endif
/* sie reg end */


typedef PF_TP_PKT_RECD_CB      USB_RX_PKT_FUNC_PTR;
typedef PF_TP_TX_COMPLETED_CB  USB_TX_COMP_FUNC_PTR;

#ifdef IS_BTSOC
#define NO_CACHE_ABLE_MASK 0x00000000
#else
#define NO_CACHE_ABLE_MASK 0xa0000000
#endif

#define EVT_NOTIFY_HW                   				0x20000000
#define ACL_NOTIFY_HW                   				0x40000000
#define SCO_NOTIFY_HW                   				0x80000000

#define CLEAN_EVT_RXDONE                              0x08
#define CLEAN_ACL_RXDONE                              0x10
#define CLEAN_SCO_RXDONE                              0x20
#define CLEAN_ALTERNATING_SETTING                     0x80
#define CLEAN_WAKE_UP_ISR                             0x100

#define IS_TX                                          1 /* host side */
#define IS_RX                                          0 /* host side */

#define NO_NEED_POOL_ID                                     0xFF

enum SCO_CON_HANDLE_FILTER_TYPE{
    CON_HANDLE_1,
    CON_HANDLE_2,
    CON_HANDLE_3,    
};

/*
 * DMA Read/Write register macro
 **/
#define DMA_DWORD_READ(y)           RD_32BIT_IO(BT_DMA_REG_BASE_ADDR, y)
#define DMA_DWORD_WRITE(y, z)     	WR_32BIT_IO(BT_DMA_REG_BASE_ADDR, y, z)
#define DMA_WORD_READ(y)         	RD_16BIT_IO(BT_DMA_REG_BASE_ADDR, y)
#define DMA_WORD_WRITE(y, z)     	WR_16BIT_IO(BT_DMA_REG_BASE_ADDR, y, z)
#define DMA_BYTE_READ(y)         	RD_8BIT_IO(BT_DMA_REG_BASE_ADDR, y)
#define DMA_BYTE_WRITE(y, z)     	WR_8BIT_IO(BT_DMA_REG_BASE_ADDR, y, z)

#define BT_DMA_UART_BASE_ADDRESS     0x40056000

/*
 * UART Read/Write register macro
 **/
#define UART_DWORD_READ(y)          RD_32BIT_IO(BT_DMA_UART_BASE_ADDRESS, y)
#define UART_DWORD_WRITE(y, z)     	WR_32BIT_IO(BT_DMA_UART_BASE_ADDRESS, y, z)
#define UART_WORD_READ(y)         	RD_16BIT_IO(BT_DMA_UART_BASE_ADDRESS, y)
#define UART_WORD_WRITE(y, z)     	WR_16BIT_IO(BT_DMA_UART_BASE_ADDRESS, y, z)
#define UART_BYTE_READ(y)         	RD_8BIT_IO(BT_DMA_UART_BASE_ADDRESS, y)
#define UART_BYTE_WRITE(y, z)     	WR_8BIT_IO(BT_DMA_UART_BASE_ADDRESS, y, z)

/*
 * 0x1: USB
 * 0x2: PCIE
 * 0x3: UART
 **/
#ifdef _ENABLE_UART_DMA_
#define INTERFACE_TYPE        3
#elif _ENABLE_PCIE_DMA_
#define INTERFACE_TYPE        2
#else
#define INTERFACE_TYPE        1
#endif
#define INIT_FIRST            1
#define INIT_RUN_TIME         2
#define INIT_FROM_DLPS        3

#define USB_INTERFACE         1
#define PCIE_INTERFACE        2
#define UART_INTERFACE        3

#define ENABLE_WAKE_UP_ISR    1
#define DISABLE_WAKE_UP_ISR   2

#define HOST_SLEEP_QUEUE_PACKET_NUM                     EVT_RX_DES_NUM-1

/* The definition of HCI DMA ISR Function Code */
enum HCI_DMA_ISR_FUNC_OPCODE {
    HCID_ISR_OP_CMD_TX,
    HCID_ISR_OP_SCO_TX,        
    HCID_ISR_OP_ACL_TX,
#ifdef LE_MODE_EN
    HCID_ISR_OP_LE_ACL_TX,
#endif
    HCID_ISR_OP_EVENT_RX,
    HCID_ISR_OP_ACL_RX,
    HCID_ISR_OP_SCO_RX,
    HCID_ISR_OP_ISO_ALT_CHG,
#ifdef _ENABLE_32K_CLK_WAKE_UP_ISR_    
    HCID_ISR_OP_32K_CLK_WKUP,
#endif    
#ifdef _SUPPORT_USB_TIMEOUT_INTERRUPT_
    HCID_ISR_OP_TIMEOUT_HANDLE,
#endif
    HCID_ISR_INVALID_OPCODE
};

#ifdef _DAPE_TEST_FIX_DMA_HANG
////// CMD_TX_BUF_FREE_RX_START_CTL_REG 0x38
/*
 * Bits |	++Descriptin++
 * 31   |	FW write 1 to notify DMA that synchronous RX descriptor ring has valid data. 
 * 30   |	FW write 1 to notify DMA that ACL RX descriptor ring has valid data
 * 29   |	FW write 1 to notify DMA that event descriptor ring has valid data.
 * 28:8 |	Reserved
 * 7:0  |	Command buffer free notification. 
**/

typedef union CMD_TX_BUF_FREE_RX_START_CTL_REG_S{
	UINT32 d32;
	struct
	{
        UINT32 cmd_tx_buf_free_nofification     :8;  // bit0~7
        UINT32 reserved_8_28                    :21; // bit8~28
        UINT32 evt_rx_start_notification        :1;  // bit29
        UINT32 acl_rx_start_notification        :1;  // bit30
        UINT32 sco_rx_start_notification        :1;  // bit31
	}b;	
}CMD_TX_BUF_FREE_RX_START_CTL_REG_TYPE;
#endif

typedef union CHIP_FUN_AND_INTERFACE_S{
     UINT32 d32;
     struct
     {
        UINT32 bt_fun_sts:1;            //bit0, copied from Vendor 0x44[25](bt_fen_sts)
        UINT32 bt_interface:4;          //bit1~4, derived from Vendor 0x50[31:30](hci_sel)
#ifdef _8821A_BTON_DESIGN_                                
        UINT32 rsvd1:5;                 //bit5~9
#else
        UINT32 gps_fun_sts:1;           //bit5
        UINT32 gps_interface:4;         //bit6~9
#endif  

#ifndef _SUPPORT_3BITS_HCI_SELECTION_
        UINT32 hci_sel:2;               //bit10~11, copied from Vendor 0x50[31:30](hci_sel)
        UINT32 reserved_12_31:20;       //bit12~31
#else
        UINT32 hci_sel:3;               //bit10~12, copied from page0 0xxF4[14:12](hci_sel)
        UINT32 reserved_13_31:19;       //bit13~31
#endif
     }b;
}CHIP_FUN_AND_INTERFACE_S_TYPE;

typedef union DMA_ADDR_REG{
	UINT32 d32;
	struct
	{
		UINT32 addr    	: 16;   //hardware register address is 4 bytes alignment	
		UINT32 reserved	: 16;
	}b;	
}DMA_ADDR_REG_TYPE;

/*
 * 0-11  bits: ACL TX buffer entry size
 * 12-15 bits: reserved
 * 16-20 bits: ACL TX packet number
 * 21-31 bits: reserved
**/
typedef union DMA_ACL_TX_BUF_SIZE{
	UINT32 d32;
	struct
	{
		UINT32 acl_tx_buf_entry_size    : 12;
		UINT32 reserved_12_15           : 4;
		UINT32 acl_tx_pkt_num           : 5;
#ifndef _NEW_HCIDMA_FROM_V1_3_
		UINT32 reserved_21_31           : 11;
#else
		UINT32 reserved_21_23           : 3;
        UINT32 acl_tx_pkt_offset        : 8;
#endif
	}b;
}ACL_TX_BUF_SIZE_TYPE;

/*
 * 0-8   bits: SCO TX buffer entry size
 * 9-15  bits: Reserved
 * 16-20 bits: SCO TX packet number
 * 21-31 bits: Reserved
**/
typedef union DMA_SCO_TX_BUF_SIZE{
	UINT32 d32;
	struct
	{
		UINT32 sco_tx_buf_entry_size    : 9;
		UINT32 reserved_15_9            : 7;
		UINT32 sco_tx_pkt_num           : 5;
#ifndef _NEW_HCIDMA_FROM_V1_3_
		UINT32 reserved_21_31           : 11;
#else
        UINT32 reserved_21_23           : 3;
        UINT32 sco_tx_pkt_offset        : 8;
#endif
	}b;
}SCO_TX_BUF_SIZE_TYPE;

/*
 * 0-8   bits: Command TX buffer entry size
 * 9-15  bits: Reserved
 * 16-18 bits: Command TX packet number
 * 19-31 bits: Reserved
**/
typedef union DMA_CMD_TX_BUF_SIZE{
	UINT32 d32;
	struct
	{
		UINT32 cmd_tx_buf_entry_size    : 9;
		UINT32 reserved_15_9            : 7;
		UINT32 cmd_tx_pkt_num           : 3;
#ifndef _NEW_HCIDMA_FROM_V1_3_
		UINT32 reserved_19_31           : 13;
#else
        UINT32 reserved_19_23           : 5;
        UINT32 cmd_tx_pkt_offset        : 8;
#endif
	}b;
}CMD_TX_BUF_SIZE_TYPE;

/*
 * 0-8   bits: Command TX buffer entry size
 * 9-15  bits: Reserved
 * 16-18 bits: Command TX packet number
 * 19-31 bits: Reserved
**/
typedef union DMA_LE_ACL_TX_BUF_SIZE{
	UINT32 d32;
	struct
	{
#ifndef _NEW_HCIDMA_FROM_V1_3_
		UINT32 le_tx_buf_entry_size       : 9;
		UINT32 reserved_15_9              : 7;
#else
        UINT32 le_tx_buf_entry_size       : 12;
        UINT32 reserved_15_12             : 4;
#endif
		UINT32 le_acl_tx_pkt_num          : 5;

#ifndef _NEW_HCIDMA_FROM_V1_3_
		UINT32 reserved_21_31             : 11;
#else
        UINT32 reserved_21_23             : 3;
        UINT32 le_tx_pkt_offset           : 8;
#endif
	}b;
}LE_ACL_TX_BUF_SIZE_TYPE;

/*
 * 0-15  bits: General TX buffer free bit map
 * 16-31 bits: Reserved
**/
typedef union GEN_TX_BUF_FREE{
	UINT32 d32;
	struct
	{
		UINT32 tx_buf_free      : 16;
		UINT32 reserved         : 16;
	}b;
}GEN_TX_BUF_FREE_TYPE;

/*
 * 0-3   bits: Command TX buffer free bt map
 * 4-28  bits: Reserved
 * 29    bit : Event RX notification bit
 * 30    bit : ACL RX notification bit
 * 31    bit : SCO RX notification bit
**/
typedef union CMD_TX_BUF_FREE_AND_RX_START{
	UINT32 d32;
	struct
	{
		UINT32 cmd_tx_buf_free_notification     :4;
		UINT32 reserved_4_28                    :25;
		UINT32 evt_rx_start_notification        :1;
		UINT32 acl_rx_start_notification        :1;
		UINT32 sco_rx_start_notification        :1;
	}b;
}CMD_TX_BUF_FREE_AND_RX_START_TYPE;

typedef union ACL_TX_PKT_ORDER_FIFO{
	UINT32 d32;
	struct
	{
		UINT32 acl_tx_order_fifo_port   				:4;/*bit[3:0]*/
/*	Updated by Wallice for HCI DMA Enhancement.	2011/11/28	*/		
//		UINT32 reserved_4_31            					:28;/*bit[31:4]*/
		UINT32 USB_ACL_HdrLen_Longer_F            	:1;/*bit[4]*/
		UINT32 USB_ACL_HdrLen_Shorter_F            	:1;/*bit[5]*/
		UINT32 reserved_6_31            				:26;/*bit[31:6]*/
/*	End Updated by Wallice for HCI DMA Enhancement.	2011/11/28	*/		
	}b;
}ACL_TX_PKT_ORDER_FIFO_TYPE;

typedef union SCO_TX_PKT_ORDER_FIFO{
	UINT32 d32;
	struct
	{
		UINT32 sco_tx_order_fifo_port   :4;
		UINT32 sco_tx_err_sts           :3;
		UINT32 reserved_7_31            :25;
	}b;
}SCO_TX_PKT_ORDER_FIFO_TYPE;


typedef union CMD_TX_PKT_ORDER_FIFO{
	UINT32 d32;
	struct
	{
		UINT32 cmd_tx_order_fifo_port   :2;
		UINT32 reserved_2_31            :30;
	}b;
}CMD_TX_PKT_ORDER_FIFO_TYPE;

typedef union LE_ACL_TX_PKT_ORDER_FIFO{
	UINT32 d32;
	struct
	{
		UINT32 LE_tx_order_fifo_port   :4;
/*	Updated by Wallice for HCI DMA Enhancement.	2011/11/28	*/		
//		UINT32 reserved_4_31            					:28;/*bit[31:4]*/
		UINT32 USB_LE_HdrLen_Longer_F            	:1;/*bit[4]*/
		UINT32 USB_LE_HdrLen_Shorter_F            	:1;/*bit[5]*/
		UINT32 reserved_6_31            				:26;/*bit[31:6]*/
/*	End Updated by Wallice for HCI DMA Enhancement.	2011/11/28	*/		
	}b;
}LE_ACL_TX_PKT_ORDER_FIFO_TYPE;

typedef union ISR_STATUS{
	UINT32 d32;
	struct
	{
		UINT32 cmd_tx_int_sts       :1; /* bit[0] */
		UINT32 acl_tx_int_sts       :1; /* bit[1] */
		UINT32 sco_tx_int_sts       :1; /* bit[2] */
		UINT32 evt_rx_int_sts       :1; /* bit[3] */
		UINT32 acl_rx_int_sts       :1; /* bit[4] */
		UINT32 sco_rx_int_sts       :1; /* bit[5] */
		UINT32 le_acl_tx_int_sts    :1; /* bit[6] */
		UINT32 iso_alt_ch_int_sts   :1; /* bit[7] */
		UINT32 wake_up_int_sts      :1; /* bit[8] */
/*	Updated by Wallice for HCI_DMA enhancement.	2011/11/28	*/		
//		UINT32 reserved_9_13        :5; /* bit[13:9] */
		UINT32 HCI_ACK_Err_int_sts      				:1; /* bit[9] */
		UINT32 USB_ACL_Tx_LongPktErr_int_sts      	:1; /* bit[10] */
		UINT32 USB_LE_Tx_LongPktErr_int_sts      		:1; /* bit[11] */
		UINT32 USB_BulkOut_Arrive_int_sts      		:1; /* bit[12] */
		UINT32 USB_CMD_Tx_PktErr_int_sts      		:1; /* bit[13] */
/*	End Updated by Wallice for HCI_DMA enhancement.	2011/11/28	*/		
		UINT32 le_acl_num_in_tx_pkt :5; /* bit[18:14] */
		UINT32 cmd_num_in_tx_pkt    :3; /* bit[21:19] */
#ifndef _SUPPORT_USB_TIMEOUT_INTERRUPT_        
		UINT32 acl_num_in_tx_pkt    :5; /* bit[26:22] */
		UINT32 sco_num_in_tx_pkt    :5; /* bit[31:27]*/
#else
        UINT32 acl_num_in_tx_pkt        :3; /* bit[24:22] */
		UINT32 reserved_b25      		:1; /* bit[25] */
        UINT32 evt_timeout_int_sts       :1; /* bit[26] */
        UINT32 sco_num_in_tx_pkt        :3; /* bit[29:27]*/
        UINT32 iso_in_timeout_int_sts    :1; /* bit[30] */
        UINT32 iso_out_timeout_int_sts   :1; /* bit[31] */        
#endif
	}b;
}ISR_STATUS_TYPE;

typedef union ISR_CONTRL{
	UINT32 d32;
	struct
	{
		UINT32 cmd_tx_int_en      :1;  /* bit[0] */
		UINT32 acl_tx_int_en      :1;  /* bit[1] */
		UINT32 sco_tx_int_en      :1;  /* bit[2] */
		UINT32 evt_rx_int_en      :1;  /* bit[3] */
		UINT32 acl_rx_int_en      :1;  /* bit[4] */
		UINT32 sco_rx_int_en      :1;  /* bit[5] */
		UINT32 le_acl_tx_int_en   :1;  /* bit[6] */
		UINT32 iso_alt_ch_int_en  :1;  /* bit[7] */
		UINT32 wake_up_int_en     :1;  /* bit[8] */		
/*	Updated by Wallice for HCI_DMA enhancement.	2011/11/28	*/		
//		UINT32 reserved_9_31      						:23; /* bit[31:9] */
		UINT32 HCI_ACK_Err_int_en      				:1; /* bit[9] */
		UINT32 USB_ACL_Tx_LongPktErr_int_en      	:1; /* bit[10] */
		UINT32 USB_LE_Tx_LongPktErr_int_en      		:1; /* bit[11] */
		UINT32 USB_BulkOut_Arrive_int_en      		:1; /* bit[12] */
		UINT32 USB_CMD_Tx_PktErr_int_en      	        :1; /* bit[13] */
#ifndef _SUPPORT_USB_TIMEOUT_INTERRUPT_        
		UINT32 reserved_14_31      					:18; /* bit[31:12] */
#else
		UINT32 reserved_14_25      		:12; /* bit[25:14] */
        UINT32 evt_timeout_int_en       :1; /* bit[26] */
		UINT32 reserved_27_29      		:3; /* bit[29:27] */            
        UINT32 iso_in_timeout_int_en    :1; /* bit[30] */
        UINT32 iso_out_timeout_int_en   :1; /* bit[31] */
#endif        
/*	End Updated by Wallice for HCI_DMA enhancement.	2011/11/28	*/		
	}b;
}ISR_CONTRL_TYPE;

typedef union DMA_CONTRL{
	UINT32 d32;
	struct
	{
		UINT32 dma_rx_en              :1;  /* bit[0] */
		UINT32 dma_rx_st              :1;  /* bit[1] */
		UINT32 dma_tx_en              :1;  /* bit[2] */
		UINT32 dma_tx_st              :1;  /* bit[3] */
		UINT32 sco_on                 :1;  /* bit[4] */
#ifndef _REMOVE_HCI_PCIE_        
		UINT32 pcie_msg_en            :1;  /* bit[5] */
#else
        UINT32 rsvd                   :1;  /* bit[5] */
#endif
		UINT32 le_acl_tx_pkt_buf_en   :1;  /* bit[6] */
        UINT32 usb_hcicmdlen_sel      :1;  /* bit[7] */
#ifndef _NEW_HCIDMA_FROM_V1_3_        
        UINT32 reserved_8_15          :8;  /* bit[15:8] */  
#else
        UINT32 le_data_ext_en         :1;  /* bit[8] */
		UINT32 reserved_9_15          :7;  /* bit[15:9] */
#endif
		UINT32 sco_tx_int_thr         :4;  /* bit[19:16] */
		UINT32 reserved_20_23         :4;  /* bit[23:20] */
		UINT32 evt_rx_int_mode        :1;  /* bit[24] */
		UINT32 acl_rx_int_mode        :1;  /* bit[25] */
		UINT32 sco_rx_int_mode        :1;  /* bit[26] */
		UINT32 reserved_27_28         :2;  /* bit[28:27] */
		UINT32 fw_set_error           :1;  /* bit[29] */		
		UINT32 fw_set_ready           :1;  /* bit[30] */
		UINT32 dma_soft_rst           :1;  /* bit[31] */
	}b;
}DMA_CONTRL_TYPE;

typedef union SCO_CON_HANDLE_1_AND_2{
	UINT32 d32;
	struct
	{
		UINT32 con_handle1      :12; /* bit[11:0] */
		UINT32 con_handle1_vld  :1;  /* bit[12] */
		UINT32 recover_mode     :3;  /* bit[15:13] */
		UINT32 con_handle2      :12; /* bit[27:16] */
		UINT32 con_handle2_vld  :1;  /* bit[28] */
		UINT32 iso_al_setting   :3;  /* bit[31:29] */
	}b;
}SCO_CON_HANDLE_1_AND_2_TYPE;

typedef union SCO_CON_HANDLE_3{
	UINT32 d32;
	struct
	{
		UINT32 con_handle3        :12;
		UINT32 con_handle3_vld    :1;
		UINT32 reserved_13_15     :3;
		UINT32 le_con_handle      :12;
		UINT32 le_con_han_value   :1;
		UINT32 reserved_29_31     :3;
	}b;
}SCO_CON_HANDLE_3_TYPE;


typedef union DMA_RX_DES{
	UINT32 d32;
	struct
	{
		UINT32 seg_start_addr   :16;
		UINT32 seg_len          :12;
		UINT32 resevered_28     :1;
		UINT32 last_seg         :1;
		UINT32 end              :1;
		UINT32 own              :1;
	}b;
}DMA_RX_DES_TYPE;


typedef struct DMA_RX_DES_DW1{
	UINT32 payload_start_add      :16;
#ifndef _REMOVE_HCI_PCIE_    
	UINT32 pcie_vender_spec       :8;
#else
    UINT32 reserved_16_23         :8;
#endif
#ifdef _UART_H5	
	UINT32 reserved_24_28         :5;
	UINT32 h5_unrel                    :1;
#else
	UINT32 reserved_24_29         :6;
#endif
	UINT32 end                    :1;
	UINT32 own                    :1;
}DMA_RX_DES_DW1_TYPE;

typedef struct DMA_RX_DES_ACL_DW2{
	UINT32 handle                 :12;
	UINT32 pb_flag                :2;
	UINT32 bc_flag                :2;
	UINT32 data_total_length      :16;
}DMA_RX_DES_ACL_DW2_TYPE;

typedef struct DMA_RX_DES_SCO_DW2{
	UINT32 handle             :12;
	UINT32 pkt_sta_flag       :2;
	UINT32 reserved_14_15     :2;
	UINT32 data_total_length  :8;
	UINT32 reserved_24_31     :8;
}DMA_RX_DES_SCO_DW2_TYPE;

typedef struct DMA_RX_DES_EVT_DW2{
	UINT32 event_code         :8;
	UINT32 event_pkt_length   :8;
	UINT32 reserved_16_31     :16;
}DMA_RX_DES_EVT_DW2_TYPE;

typedef struct DMA_NEW_RX_DES{
    union{
        UINT32                    rx_des_dw1_u32;
        DMA_RX_DES_DW1_TYPE       rx_des_dw1;
    };
    union{
        UINT32                    rx_des_dw2_u32;
        DMA_RX_DES_ACL_DW2_TYPE   rx_des_acl_dw2;
        DMA_RX_DES_SCO_DW2_TYPE   rx_des_sco_dw2;
        DMA_RX_DES_EVT_DW2_TYPE   rx_des_evt_dw2;
    };
}DMA_NEW_RX_DES_TYPE;


typedef struct DMA_MANAGER{
//	UINT32  cmd_tx_index;
//	UINT32  sco_tx_index;
//	UINT32  acl_tx_index;
    UINT8   evt_rx_w_ind;
    UINT8   evt_rx_r_ind;	
    UINT8   acl_rx_w_ind;
    UINT8   acl_rx_r_ind;
    UINT8   sco_rx_w_ind;
    UINT8   sco_rx_r_ind;
    UINT8   evt_available_num;
    UINT8   acl_available_num;
    UINT8   sco_available_num;    
    UINT32  dma_evt_rx_free_addr[EVT_RX_DES_NUM];
    UINT32  dma_acl_rx_free_addr[ACL_RX_DES_NUM];
    UINT32  dma_sco_rx_free_addr[SCO_RX_DES_NUM];	
}DMA_MANAGER_TYPE;

typedef union EFUSE_POW_PARA_S{
    UINT16 d16;
    struct 
    {
        UINT16 lps_timer_en               :1; //bit 0
        UINT16 bt_core_ldo_standyby_en    :1; //bit 1 (EXPIRED after RTL8821A/RTL8723B)
        UINT16 usb_sie_ny_en              :1; //bit 2
        UINT16 efuse_pow_option_en        :1; // bit 3
        UINT16 force_mask_wifi_ch_en      :1; //bit 4
        UINT16 pta_antenna_status         :1; //bit 5  
        UINT16 uart_hci_reset_en          :1; //bit6
#ifdef _8821A_BTON_DESIGN_ 
        UINT16 pcm_pull_low_en            :1; //bit7
        UINT16 pcm_pull_low_act_0_3       :4; //bit8~11
        UINT16 pcm_pull_low_pdn_sus_0_3   :4; //bit12~15     
#else
        UINT16 reserved_7_15              :9; //bit 7~15
#endif        
    }b;
}EFUSE_POW_PARA_S_TYPE;

typedef union GENERAL_CONTROL_S{
    UINT16 d16;
    struct
    {
        UINT16 gpio_wake_up_fun_en        :1; //bit 0
        UINT16 gpio_wake_up_type          :2; //bit 1~2
        UINT16 gpio_led_fun_en            :1; //bit 3
        UINT16 gpio_wake_up_polarity      :1; //bit 4
        UINT16 uart_h5_wakeup_en          :1; //bit 5
        UINT16 led_polarity               :1; //bit 6
        UINT16 led_idle_glitter           :1; //bit 7
        UINT16 led_vendor_en              :1; //bit 8
#ifdef _8821A_BTON_DESIGN_
        UINT16 gpio_wake_up_opendrain     :1; //bit 9
#else
        UINT16 afe_ldo_hw_ctrl_en         :1; //bit 9 (EXPIRED after RTL8821A/RTL8723B)
#endif        
        UINT16 log_uart_rx_en             :1; //bit10
        UINT16 pdn_delay_time_unit        :1; //bit11 
#ifdef _8821A_BTON_DESIGN_
        UINT16 pdn_sus_fa_recov_en        :1; //bit12, default 0 (1 is recommended), 1: recovery after execute power-down for false-alarm case) 
        UINT16 pdn_sus_fa_recov_delay     :3; //bit13~15, default 0, (2 is recommended), delay =  5us * 2^x
#else        
        UINT16 reserved_12_15              :4; //bit 12~15
#endif        
    }b;
}GENERAL_CONTROL_S_TYPE;

typedef struct RX_QUEUE_PKT_S{
    UINT8   *buf;
    UINT8   type;
    UINT16  len;
}RX_QUEUE_PKT_S_TYPE;

typedef struct HOST_SLEEP_PKT_MANAGER_S{
    RX_QUEUE_PKT_S_TYPE queue_man[HOST_SLEEP_QUEUE_PACKET_NUM];
    UINT32  queue_index;
}HOST_SLEEP_PKT_MANAGER_S_TYPE;


#ifdef _SUPPORT_POLLING_BASED_LPM_L1_

#define SIE_REG_0x10 0x10
#define SIE_REG_0x52 0x52
#define SIE_REG_0x65 0x65
#define SIE_REG_0x7A 0x7A


typedef union USB_SIE_0x65_REG_S {
    UINT8 d8;
    struct 
    {
        UINT8 do_not_write_0        :2; //bit1,0
        UINT8 usb_bcd_201           :1; //bit2
        UINT8 besl_en               :1; //bit3
        UINT8 baseline_besl_valid   :1; //bit4          
        UINT8 deepsleep_valid       :1; //bit5
        UINT8 do_not_write_1        :2; //bit7,6        

    }b;
}USB_SIE_0x65_REG_S_TYPE;

typedef union USB_SIE_0x7A_REG_S {
    UINT8 d8;
    struct 
    {

        UINT8 baseline_besl_value   :4; //bit[3:0]       
        UINT8 deep_besl_value       :4; //bit[7:4]

    }b;
}USB_SIE_0x7A_REG_S_TYPE;


typedef union USB_SIE_0x52_REG_S {
    UINT8 d8;
    struct 
    {
        UINT8 ep0_buf_ok            :1; //bit0    
        UINT8 ep1_buf_ok            :1; //bit1
        UINT8 ep2_buf_ok            :1; //bit2
        UINT8 project_depend_0      :1; //bit3
        UINT8 project_depend_1      :1; //bit4          
        UINT8 project_depend_2      :1; //bit5
        UINT8 project_depend_3      :1; //bit6        
        UINT8 project_depend_4      :1; //bit7

    }b;
}USB_SIE_0x52_REG_S_TYPE;

typedef union LPM_POLL_CONTROL_S_ {
    UINT8 d8;
    struct
    {
        UINT8 lpm_en:1;            // bit 0 => LPM Disable: 0, LPM Enable: 1               
        UINT8 lpm_sw_fsm_en:1;     // bit 1 => Support USB LPM operation FSM by BT FW
                                   // 1: Yes, 0: NO  
        UINT8 lpm_sw_res_policy:1; // bit 2 => USB LPM token response policy, 
                                   // 1: always NYET, 
                                   // 0: return ACK when rx fifo is empty during a sample period
        UINT8 lpm_sample_period:5; // bit[7:3] => sample period (units: ms)
    }b;
} LPM_POLL_CONTROL;
 
#endif


BOOLEAN dma_init(UINT8 init_type);

BOOLEAN S3C2410Usb_Dma(HCI_TRANSPORT_PKT_TYPE pkt_type, UCHAR *buf, UINT16 len, UINT32 free_index);
void dma_tx_fifo_order_free(UINT16 free_bitmap, UINT8 type);
UINT16 dma_tx_fifo_pkt_free(void *ppkt, UINT8 type);
void switch_hci_dma_setting(UINT8 mode);
void wake_up_interrupt_switch(UINT8 mode);
void enable_filter_sco_connection(UINT8 sco_index, 
              UINT8 is_enable, UINT8 con_handle, UINT8 mode);

#ifdef _SUPPORT_NEW_USB_LPM_L1_DATA_PATH_150629_
UINT8 hci_dma_usb_check_usb_lpm_l1_then_queue_notification(UINT8 pkt_type);
UINT8 hci_dma_usb_check_and_allow_enter_usb_lpm_l1(void);
UINT8 hci_dma_usb_check_and_dequeue_notification(void);
#ifdef _ALLOW_ENTER_USB_LPM_L1_VIA_HW_TIMER2_
extern UINT8 g_usb_lpm_l1_hw_timer_is_running;
#endif
#endif

#ifdef _SUPPORT_POLLING_BASED_LPM_L1_
BOOLEAN checkRxDescAllFree(UINT8 type);
void set_lpm_l1_token_reponse(UINT8 response);
UINT8 get_lpm_l1_token_reponse(void);
enum{L1_NYET=0, L1_ACK=1};
enum {CHECK_BY_CNT = 0, CHECK_BY_OWN = 1};

#endif

void clear_sco_buffer();

#ifdef _DMA_LOOPBACK_TEST_ 
BOOLEAN Usb_Dma_RX(HCI_TRANSPORT_PKT_TYPE pkt_type, UCHAR *buf, UINT16 len, UINT8 free_index);
#endif

#ifdef _RTL_DBG_UTILITY_
BOOLEAN rtk_vender_command(UINT8* in_buf);
#endif
#ifdef _RX_CMD_DBG_
void cmd_rx_test_step_1();
void cmd_rx_test_step_2();
#endif
#ifdef _RX_ACL_DBG_
void acl_rx_test_step_1();
void acl_rx_test_step_2();
#endif
#ifdef _RX_SCO_DBG_
void sco_rx_test_step_1();
void sco_rx_test_step_2();
#endif
#ifdef _TX_CMD_DBG_
void cmd_tx_test(UINT8* in_buf, UINT16 pkt_len);
#endif
#ifdef _TX_SCO_DBG_
void sco_tx_test(UINT8* in_buf, UINT16 pkt_len);
#endif
#ifdef _TX_ACL_DBG_
void acl_tx_test(UINT8* in_buf, UINT16 pkt_len);
#endif
#ifdef _TX_LE_ACL_DBG_
void le_acl_tx_test(UINT8* in_buf, UINT16 pkt_len);
#endif
void dma_des_init();

void dma_des_allocate(POOL_ID dma_trx_pool_id, POOL_ID tx_pool_id,
								UINT8 is_tx, void**des_p, UINT32 buf_num);

#ifdef _DMA_LOOPBACK_TEST_ 
void set_rx_des(DMA_RX_DES_TYPE *rx_des, UINT32 addr, UINT16 len, UINT8 fifo_index);
#endif
#ifdef _USB_DUMP_REG_
void dump_usb_dma_register();
void dump_dma_tx_des();
#endif

extern USB_RX_PKT_FUNC_PTR    RxPktFunc;
extern USB_TX_COMP_FUNC_PTR   TxCompFunc;
extern DMA_MANAGER_TYPE       dma_man;

extern CHIP_FUN_AND_INTERFACE_S_TYPE g_fun_interface_info;
extern UINT32 g_baudRate;
extern UINT32 g_uart_clk;

extern UINT8 wakeup_isr_en;

extern UINT8 enable_hci_loopback_mode;

extern UINT8 g_host_state;

extern UINT8 alternatset_change_tx_clear;
extern UINT8 alternatset_change_rx_clear;

extern UINT8 g_uart_h5_fw_allow;

extern UINT8 g_wake_first_time;
#ifdef _YL_RTL8723A_B_CUT
extern UINT8 g_host_wake_bt;
#endif

#endif

