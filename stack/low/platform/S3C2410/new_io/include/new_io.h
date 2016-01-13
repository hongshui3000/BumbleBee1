/***************************************************************************
 Copyright (C) Realtek
 ***************************************************************************/
#ifndef _NEW_IO_H_
#define _NEW_IO_H_

#include "DataType.h"
#include "power_control.h"


/* Indirected Access Definition for System On Area */
typedef struct VENDOR_REG_INDIRECT_CTRL_S_ {
    UINT32 offset:16;               /* bit[15:0] Register Offset (R/W) */
    UINT32 rsvd:10;                 /* bit[25:16]: reserved */
    UINT32 start_bit:1;             /* bit[26]: start bit => request HW to 
                                        start Indirected Control Access. 
                                        Note: To enable the command, FW 
                                        must set this bit to one then 
                                        clear this bit to zero. (R/W)*/
    UINT32 byte_access_type:2;      /* bit[28:27] byte_access_type 
                                       => the access type for one/two/four 
                                          bytes access (R/W)
                                        0 is one byte access, 
                                        1 is two types access, 
                                        2 is four bytes access, 
                                        3 is reserved */
    UINT32 rw_ctrl:1;               /* bit[29]: rw_ctrl 
                                        => the read/write control bit. 
                                           1 is write and 0 is read (R/W) */
    UINT32 err:1;                   /* bit[30]: error bit 
                                        => inidicate this access is error or 
                                           not. 1 is error and 0 is OK. (RO)*/
    UINT32 busy:1;                  /* bit[31]: busy bit 
                                        => inidicate the access is busy or 
                                         idle. 1 is busy and 0 is idle. (RO)*/
} VENDOR_REG_INDIRECT_CTRL_S;


#define VENDOR_INDIRECT_CTRL_REG     0x18
#define VENDOR_INDIRECT_DATA_REG     0x1C


typedef union PAGE0_REG_16B_S_ {
    UINT16 d16;    
    struct 
    {
        UINT16 EEDO:1;              /* [16] R/W	0 */
        UINT16 EEDI:1;              /* [17] R/W	0 */
        UINT16 EESK:1;              /* [18] R/W	0 */
        UINT16 EECS:1;              /* [19] R/W	0 */
        UINT16 EERPOMSEL:1;         /* [20] R/W	0 */
        UINT16 AUTOLOAD_SUS:1;      /* [21] R/W	1 */
        UINT16 EEM1_0:2;            /* [23:22] R/W 0 */	        
        UINT16 VPDIDX:8;            /* [31:24] R/W 0 */
    }reg0a;
}PAGE0_REG_16B_S;

#if defined(_BT_ONLY_) || defined(_SUPPORT_COMBO_RW_PAGE0_)
void indirect_access_write_syson_reg(UINT16 offset, UINT32 wdata, UINT8 type);
UINT32 indirect_access_read_syson_reg(UINT16 offset, UINT8 type);
void indirect_access_write_two_byte_syson_reg(UINT16 offset, UINT16 wdata);
UINT8 indirect_access_read_one_byte_syson_reg(UINT16 offset);
UINT16 indirect_access_read_two_byte_syson_reg(UINT16 offset);

#define WR_8BIT_SYSON_IO(offset, value)  indirect_access_write_syson_reg(offset, value, 0)
// temp solution, to be check
#ifdef _FONGPIN_CHECK_TODO_
#define WR_16BIT_SYSON_IO(offset, value) indirect_access_write_syson_reg(offset, value, 1)
#else
#define WR_16BIT_SYSON_IO(offset, value) indirect_access_write_two_byte_syson_reg(offset, value)
#endif
#define WR_32BIT_SYSON_IO(offset, value) indirect_access_write_syson_reg(offset, value, 2)
#define RD_8BIT_SYSON_IO(offset)         indirect_access_read_one_byte_syson_reg(offset)
#define RD_16BIT_SYSON_IO(offset)        indirect_access_read_two_byte_syson_reg(offset)
#define RD_32BIT_SYSON_IO(offset)        indirect_access_read_syson_reg(offset, 2)
#else
#define WR_8BIT_SYSON_IO(offset, value)
#define WR_16BIT_SYSON_IO(offset, value)
#define WR_32BIT_SYSON_IO(offset, value)
#define RD_8BIT_SYSON_IO(offset)
#define RD_16BIT_SYSON_IO(offset)
#define RD_32BIT_SYSON_IO(offset)
#endif


UINT8 read_sie_by_page0(UINT8 u8Offset);
void write_sie_by_page0(UINT8 u8Offset, UINT8 u8Value);


#ifdef _NEW_BTON_DESIGN_AFTER_RTL8821B_TC_
#ifndef _REMOVE_INDIRECT_READ_PAGE0_
UINT8 safe_indirect_read_combo_byte_syson_reg(UINT16 offset);   // only valid in 8703B& 8822B
UINT8 indirect_read_combo_byte_syson_reg(UINT16 offset);        // only valid in 8703B& 8822B
UINT16 indirect_read_combo_word_syson_reg(UINT16 offset);       // only valid in 8703B& 8822B
UINT32 indirect_read_combo_dword_syson_reg(UINT16 offset);      // only valid in 8703B& 8822B
#endif
#ifdef _SUPPORT_FW_INDIRECT_READ_SIE_
UINT16 indirect_read_sie(UINT8 type, UINT8 u8Addr);
UINT16 safe_indirect_read_sie(UINT8 type, UINT8 u8wAddr);
#endif

#ifndef _REMOVE_INDIRECT_READ_PAGE0_
void indirect_access_read_syson_test();
#endif

void indirect_access_read_sie_test();
void ThirdGenModemPiAccessTest();

#ifndef _REMOVE_INDIRECT_READ_PAGE0_
#define RD_8BIT_COMBO_SYSON_IO(offset)         safe_indirect_read_combo_byte_syson_reg(offset)
#define RD_16BIT_COMBO_SYSON_IO(offset)        indirect_read_combo_word_syson_reg(offset)
#define RD_32BIT_COMBO_SYSON_IO(offset)        indirect_read_combo_dword_syson_reg(offset)
#endif

#define VENOR_REG_READ_SIE 0x358
enum{READ_SIE_BYTE=0, READ_SIE_WORD};
#endif


#ifdef _NEW_BTON_DESIGN_AFTER_RTL8821B_TC_
UINT16 indirect_access_read_32k_reg(UINT16 offset);
void indirect_access_write_32k_reg(UINT16 offset, UINT16 wdata);

#define RD_16BIT_32K_SYSON_IO(offset)          indirect_access_read_32k_reg(offset)
#define WR_16BIT_32K_SYSON_IO(offset, wdata)   indirect_access_write_32k_reg(offset, wdata)
#endif

#if defined(_FONGPIN_TEST_TIMER_POLLING_REG_)
void timer_polling_io_dbg();
#endif

//#define PAGE0_REG_0xF5 0xF5
//#define GET_HCI_INFO_FROM_PAGE0             RD_8BIT_COMBO_SYSON_IO(PAGE0_REG_0xF5)>>4
#if 0
typedef union PAGE0_REG_0xF5_BYTE_READ_S_ {
    UINT8 d8;    
    struct 
    {
        UINT8 bt_no_use_1_0         :2; /* [1:0] wifi reserved */
        UINT8 usb_operation_mode    :1; /* [2] */
        UINT8 bt_no_use_3           :1; /* [3] wifi reserved */
        UINT8 hci_selection         :3; /* [6:4] hci selection  */
        UINT8 bt_no_use_7           :1; /* [7] wifi reserved */	        
    }b;
}PAGE0_REG_0xF5_BYTE_READ_S;
#endif
#endif

