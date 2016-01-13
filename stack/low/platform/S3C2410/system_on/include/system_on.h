#ifndef _SYSTEM_ON_H_
#define _SYSTEM_ON_H_

#include "platform.h"
#include "DataType.h"
#include "mint_os.h"


// macro region
#define PAGE0_REG_8703B_FINAL_HCI_SEL   0xF4    // 2bits[5:4]
#define PAGE0_REG_8822B_FINAL_HCI_SEL   0xF5    // 3bits[6:4]
#define PAGE0_REG_HW_ID           0xFC    // 8bits[7:0]
#define PAGE0_REG_CHIP_VER        0xF1    // 4bits[7:4]
#define PAGE0_REG_SIE_CTRL          0xE0

/*
00, sdio-uart
01, usb
10 pic-uart
11 pci-usb
21C: f4[13:12] = 0x0F[5:4], bit6 force 0
*/


/*
  8723A : 0x01
  8188E : 0x02
  8881A : 0x03 
  8812A : 0x04                               
  8821A : 0x05
  8723B : 0x06
  8192E : 0x07
  8813A : 0x08
  8821B : 0x09 (8821C)
  8822B : 0x0A
  8703B : 0x0B
  8188F : 0x0C
  8192F : 0x0D
  8723D : 0x0F
*/
enum COMBO_CHIP_HW_ID{CHIP_ID_8822B = 0x0A, CHIP_ID_8703B = 0x0B, CHIP_ID_8723D = 0x0F, CHIP_ID_8821C = 0x09 };

typedef union PAGE0_REG_0xF5_BYTE_READ_S_ {
    //2 8822B only
    UINT8 d8;    
    struct 
    {
        UINT8 bt_no_use_1_0         :2; /* [1:0] wifi reserved */
        UINT8 usb_operation_mode    :1; /* [2] */
        UINT8 bt_no_use_3           :1; /* [3] wifi reserved */
        UINT8 hci_selection_3bit    :3; /* [6:4] hci selection  */
        UINT8 bt_no_use_7           :1; /* [7] wifi reserved */	        
    }b;
}PAGE0_REG_0xF5_BYTE_READ_S;


typedef union PAGE0_REG_0xF4_BYTE_READ_S_ {
    //2 8703B only
    UINT8 d8;    
    struct 
    {
        UINT8 bt_no_use_3_0         :4; /* [3:0] wifi reserved */
        UINT8 hci_selection_2bit    :2; /* [5:4] hci selection  */
        UINT8 bt_no_use_7_6         :2; /* [7:6] wifi reserved */	        
    }b;
}PAGE0_REG_0xF4_BYTE_READ_S;


typedef union PAGE0_REG_0xE0_SIE_CTRL_S_ {
    //2 8723D & 8821C
    UINT32 d32;    
    struct 
    {
        UINT32 write_data            :8; /* [7:0] write data to sie */
        UINT32 read_data             :8; /* [15:8] read data from sie  */
        UINT32 sie_address           :8; /* [23:16] address of sie */
        UINT32 sie_bypass_ioreg_if   :1; /* [24] SIE Bypass IOREG interface.  */
        UINT32 sie_write_en          :1; /* [25] Write Enable of SIE interface.  */
        UINT32 rsvd                  :5; /* [30:26] reserved.  */
        UINT32 do_not_modify         :1; /* [31] do_not_modify.  */
    }b;
}PAGE0_REG_0xE0_SIE_CTRL_S;



// function
UINT8 getHwId(void);
UINT8 getchipver(void);
UINT8 getfinalhci(void);
//void update_lmp_version();

#endif//_SYSTEM_ON_H_

