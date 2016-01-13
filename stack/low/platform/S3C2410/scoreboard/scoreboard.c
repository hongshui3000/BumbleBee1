/***************************************************************************
 Copyright (C) RealTek Ltd.
 This module is a confidential and proprietary property of RealTek and
 a possession or use of this module requires written permission of RealTek.
 ***************************************************************************/

enum { __FILE_NUM__= 215 };

#include "scoreboard.h"
#include "platform.h"
#include "power_control.h"
#include "UartPrintf.h"
#include "logger.h"
#include "DataType.h"

#ifdef _SUPPORT_WL_BT_SCOREBOARD_

#ifdef _ROM_CODE_PATCHED_
PF_ROM_CODE_PATCH_FUNC rcp_scoreboard_intrhandler_func = NULL;
#endif


UINT16 g_u16disable_scoreboard_int_timer_cnt = 0;


void scoreboard_init()
{
    BTON_SCOREBOARD_REG_S_TYPE reg_coreboard;
    EFUSE_SCOREBOARD_AND_USB_TO_INIT_S efuse_scoreboard_init;

    *(UINT8*)&efuse_scoreboard_init = otp_str_data.efuse_misc_interrupt_d8;
    reg_coreboard.d32 = VENDOR_READ(BTON_DLPS_CONTROL_REGISTER);
   
    //reg_coreboard.wl2bt_int_msk = 1;// ToDo: set efuse content
    reg_coreboard.wl2bt_int_msk = efuse_scoreboard_init.wl2bt_int_msk;
    reg_coreboard.bt_int_to_wl = efuse_scoreboard_init.bt_int_to_wl;
        
    VENDOR_WRITE(BTON_DLPS_CONTROL_REGISTER, reg_coreboard.d32);
    
}/*end of scoreboard_init*/

/* scoreboard write data function */
void write_scoreboard_data(UINT32 write_data)
{
	BTON_SCOREBOARD_REG_S_TYPE reg_scoreboard;
	reg_scoreboard.R_LOP_ECORE = 1;
	reg_scoreboard.wl2bt_int_msk = 1;
	reg_scoreboard.bt_int_to_wl = 1;
	reg_scoreboard.wlbt_sts = (write_data & 0x7fff)<<16;
	RT_BT_LOG(GREEN, SCOREBOARD_DATA_WRITE, 1 , reg_scoreboard.wlbt_sts);
	VENDOR_WRITE(BTON_DLPS_CONTROL_REGISTER, reg_scoreboard.d32);
}
void wl_bt_scoreboard_interrupt_handler()
{
    
    BTON_SCOREBOARD_REG_S_TYPE reg_coreboard;
 
    reg_coreboard.d32 = VENDOR_READ(BTON_DLPS_CONTROL_REGISTER);
    
#ifdef _FONGPIN_TEST_SCOREBOARD_LOG_    
    RT_BT_LOG(BLUE, SCOREBOARD_REG, 1, reg_coreboard.d32);
#endif

#ifdef _ROM_CODE_PATCHED_
        if (rcp_scoreboard_intrhandler_func != NULL)
        {
            if(rcp_scoreboard_intrhandler_func((void*)&reg_coreboard.d32))
            {
                return;
            }
        }    
#endif

    //clear interrupt
    SAFE_CLEAR_SCOREBOARD_INT;

}/*end of wl_bt_scoreboard_interrupt_handler*/


void timer1_disbale_scoreboard_int(void)
{
    // bit[0]: wl2bt_int_msk, 1: can receive wl interrupt
    // bit[1]: bt2bt_mode, 1: interrupt, 0: polling
    // bit[2]: enable evt timeout interrupt, default off(0)
    // bit[3]: enable iso in timeout interrupt, default off(0)                                            
    // bit[4]: enable iso out timeout interrupt, default off(0)
    // bit[5]: enable USB_token_timeout_en to SIE, default off(0)


    if(otp_str_data.efuse_misc_interrupt_d8 & 0x20)
    {
        if(g_u16disable_scoreboard_int_timer_cnt == 512)
         {
            UINT8 u8Value = DMA_BYTE_READ(USB_TOKEN_TIMEOUT_REG);
            DMA_BYTE_WRITE(USB_TOKEN_TIMEOUT_REG,u8Value|BIT0);// enable timeout function
            g_u16disable_scoreboard_int_timer_cnt = 0;
         }
        g_u16disable_scoreboard_int_timer_cnt++;
    }
    
}

 
#endif


