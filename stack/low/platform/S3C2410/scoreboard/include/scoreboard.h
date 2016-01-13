#ifndef _SCOREBOARD_H_
#define _SCOREBOARD_H_

#include "platform.h"
#include "DataType.h"
#include "mint_os.h"


// macro region
#define SAFE_CLEAR_SCOREBOARD_INT \
{ \
    UINT8 u8data; \
    u8data = VENDOR_BYTE_READ(BTON_DLPS_CONTROL_REGISTER+1); \
    u8data |= BIT7;\
    VENDOR_BYTE_WRITE(BTON_DLPS_CONTROL_REGISTER+1, u8data); \
}

typedef struct _EFUSE_SCOREBOARD_AND_USB_TO_INIT_S_ {
// refer to vendor 0xbc
    UINT8 wl2bt_int_msk:1;          // bit0; 1: "accept" wl interrupt, 0: msk wl interrupt
    UINT8 bt_int_to_wl:1;           // bit1; 1: "assert" an interrupt to wl, 0: donothing, wl should polling page0
    //UINT8 reserved:6;               // reserved
    UINT8 evt_timeout_en:1;          // bit2; 
    UINT8 iso_in_timeout_en:1;          // bit3; 
    UINT8 iso_out_timeout_en:1;          // bit4;
    UINT8 usb_token_timeout_fun_en:1;          // bit5; 
    UINT8 usb_timeout_fun_policy_sel:2;          // bit6,7;  
} EFUSE_SCOREBOARD_AND_USB_TO_INIT_S;


// function
void scoreboard_init();
void wl_bt_scoreboard_interrupt_handler();
void timer1_disbale_scoreboard_int();
void write_scoreboard_data(UINT32 write_data);

#endif//_SCOREBOARD_H_

