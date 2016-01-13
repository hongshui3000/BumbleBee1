#if (defined(_GPIO_POWER_SEQUENCE_ENABLE))

enum { __FILE_NUM__= 104 };

#include "platform.h"
#include "gpio.h"
#include "logger.h"
#include "mint_os.h"
#include "power_control.h"
#include "dma_usb.h"
#include "lmp.h"
#include "lc.h"
#include "lmp_vendor_defines.h"

#ifdef _3DD_FUNCTION_SUPPORT_
#include "bt_3dd.h"
#endif

#include "new_io.h"

#ifdef _SUPPORT_BT_CTRL_FM_
#include "fm.h"
#endif

#ifdef _GPIO_POWER_SEQUENCE_ENABLE
UINT8  suspend_state = 0;
#endif

#if defined (_SUPPORT_INFO_FROM_SYSTEM_ON_) && defined(_SUPPORT_BT_CTRL_FM_)
extern UINT8 g_u8support_fm;
#endif


TimerHandle_t gpio_wake_up_tid_timer = NULL;

#ifdef _ROM_CODE_PATCHED_
PF_ROM_CODE_PATCH_VOID rcp_GpioInit = NULL;
PF_ROM_CODE_PATCH_FUNC rcp_GpioIntrHandler = NULL;
PF_ROM_CODE_PATCH_FUNC rcp_process_power_gpio_set = NULL;
PF_ROM_CODE_PATCH_FUNC rcp_gpio_power_on_check = NULL;
PF_ROM_CODE_PATCH_FUNC rcp_gpio_wake_up_host = NULL;
#ifdef _8821A_BTON_DESIGN_
PF_ROM_CODE_PATCH_FUNC rcp_set_gpio_bt_wake_host = NULL;
PF_ROM_CODE_PATCH_FUNC rcp_get_gpio_bt_wake_host = NULL;
PF_ROM_CODE_PATCH_FUNC rcp_gpio_wake_up_timer_callback_func = NULL;
#else
PF_ROM_CODE_PATCH_FUNC rcp_get_wake_up_pin = NULL;
#endif
#ifdef _DAPE_8723_RCP_
//PF_ROM_CODE_PATCH_VOID rcp_usb_remote_wakeup_gpio11_recv_power_dn_func = NULL;    
//PF_ROM_CODE_PATCH_VOID rcp_usb_remote_wakeup_gpio11_recv_suspend_func = NULL;    
#endif
#ifdef _YL_RTL8723A_B_CUT
PF_ROM_CODE_PATCH_FUNC rcp_gpio_power_on_check_end_func = NULL;
//PF_ROM_CODE_PATCH_FUNC rcp_gpiointr_usb_suspend_func = NULL;
#endif
#endif

#if defined(_ROM_CODE_PATCHED_) && defined( _ENABLE_VENDOR_GPIO_INTERRUPT_)
PF_ROM_CODE_PATCH_FUNC rcp_vendor_gpio_intrhandler_func = NULL;
#endif



#ifdef _YL_RTL8723A_B_CUT
#ifdef _YL_LPS
extern EFUSE_LPS_SETTING_1_S g_efuse_lps_setting_1;
extern EFUSE_LPS_SETTING_2_S g_efuse_lps_setting_2;
extern EFUSE_LPS_SETTING_3_S g_efuse_lps_setting_3;

#ifdef _CCH_RTL8723A_B_CUT
// _CCH_ECO_LPS_
extern EFUSE_LPS_SETTING_4_S g_efuse_lps_setting_4;
#endif

#endif
#endif

#ifdef _SUPPORT_USB_LOG_ENABLE_        
extern UINT8 g_usb_misc;
#endif

#ifdef _SUPPORT_TIMER1_SHUTDOWN_LOG_
UINT16 g_u16shutdown_log_timer_cnt = 0;
#endif

#ifdef _FONGPIN_TEST_MUX_DEV_WAKEUP_HOST_BY_GPIO13_
extern BTON_POW_CTRL_REG_S_TYPE pow_ctrl_int_rec;
#endif
#ifndef _IS_ASIC_
#ifdef _YL_GPIO_POW_TEST_FORCE_EFUSE
void gpio_pow_test_force_efuse(void)
{
    EFUSE_POW_SETTING_1_S efuse_pow_setting_1;
    EFUSE_POW_SETTING_2_S efuse_pow_setting_2;
    EFUSE_POW_SETTING_3_S efuse_pow_setting_3;
    *(UINT8*)&efuse_pow_setting_1 = otp_str_data.efuse_pow_setting_1_d8;    
    *(UINT8*)&efuse_pow_setting_2 = otp_str_data.efuse_pow_setting_2_d8;    
    *(UINT8*)&efuse_pow_setting_3 = otp_str_data.efuse_pow_setting_3_d8;  

    efuse_pow_setting_3.fast_gpio_power_on_check = 1;
    
    otp_str_data.efuse_pow_setting_1_d8 = *(UINT8*)&efuse_pow_setting_1;    
    otp_str_data.efuse_pow_setting_2_d8 = *(UINT8*)&efuse_pow_setting_2;    
    otp_str_data.efuse_pow_setting_3_d8 = *(UINT8*)&efuse_pow_setting_3;               
}
#endif
#endif

/*
#define GPIO7_BIT                                   0x80
#define GPIO_INT_EN_REG                       0x30	        //interrupt enable register
#define GPIO_INT_MASK_REG                   0x34	 //interrupt mask register
#define GPIO_INT_TYPE_LEVEL_REG         0x38	        //0:level_sensitive, 1:edge_sensitive
#define GPIO_INT_POLARITY_REG             0x3C	 //0:low active, 1:high active
#define GPIO_INT_STATUS_REG                0x40	 //interrupt status
#define GPIO_RAW_STATUS_REG              0x44	 //raw status
#define GPIO_DEBOUNCE_REG                  0x48	 //remove glitches
#define GPIO_PORTA_EOI_REG                 0x4C	 //clear interrupt
#define GPIO_LS_SYNC				       0x60	 //level sensitive interrupt sync to pclk_intr
*/
/*
port A: (DesignWare GPIO description)
GPIO 0: BT Disable
GPIO 1: GPS Disable(old info, error message), HOST_WAKE_BT
GPIO 2: USB suspend
GPIO 3: PCI-E suspend
GPIO 4: LED0
GPIO 5: LED1 Or RF Dbg
GPIO 6: (before 8821a) Switch antenna to BT or WIFI
        (bt only) 3dg sync
GPIO 7: (bt only) 3dg sel
*/
void GpioInit(void)
{
                
#ifdef _GPIO_POWER_SEQUENCE_ENABLE
    BTON_INTERFACE_CTRL_REG_S_TYPE bton_inter_info;
    POW_OPTION_AND_32K_CTRL_S_TYPE bt_option_reg;
    GENERAL_CONTROL_S_TYPE bt_general_ctrl;
    //pcie and usb suspend all rising edge
    UINT8 gpio_polarity_mask = 0x0C;
#ifdef _8821A_BTON_DESIGN_    
    UINT32 int_mask = 0;
    UINT32 int_en = 0;
    UINT32 gpio_out_en = 0;
    UINT32 output_value = 0;
#else
    UINT32 int_mask = 0xf0;
    UINT32 int_en = 0x0f;
    UINT32 gpio_out_en = 0;
    UINT32 output_value = 0;
#endif


#ifdef _ROM_CODE_PATCHED_
    if (rcp_GpioInit != NULL)
    {
        rcp_GpioInit();
        return;
    }
#endif

    //the setting value loaded from efuse
    bt_general_ctrl.d16 = otp_str_data.general_control;

    suspend_state = 0; // TODO: unused global variable!

    bt_option_reg.d32 = VENDOR_READ(BTON_POW_OPTION_AND_32K_CTRL);
    bton_inter_info.d32 = VENDOR_READ(BTON_INTERFACE_CTRL_REG);

    // 1) PCIE, USB multi or mix mode; if uart isn't be enabled, gpio can be uset to power
    //    function. Otherwise, it can't be used to power function
    // 2) SDIO-UART mode; No matter what uart function setting, gpio all can be used to
    //    power function.
    if ((!bton_inter_info.b.hci_uart_fen_sts) || (!bt_option_reg.b.hci_selection))
    {
//        RT_BT_LOG(BLUE,INITI_GPIO_FOR_POWER,0,0);
        //Check bt gpio polarity
        if (bton_inter_info.b.polarity_bt_gpio_sts)
            gpio_polarity_mask |=  0x1;

#ifndef _8821A_BTON_DESIGN_
        //Check gps gpio polarity
#ifdef _YL_RTL8723A_B_CUT
        if (bton_inter_info.b.polarity_gps_gpio_sts)
            gpio_polarity_mask |=  0x2;
#else
        if (bton_inter_info.b.polarity_bt_gpio_sts)
            gpio_polarity_mask |=  0x2;
#endif

        //RTK8723S (SDIO UART mode) doesn't care USB and PCIE suspend signal
        if (g_fun_interface_info.b.hci_sel == RTK8723_S)
        {
            int_mask = 0xFE;
            int_en = 0x01;
            if (bt_general_ctrl.b.gpio_wake_up_fun_en)
            {
                get_wake_up_pin(&gpio_out_en,&output_value,NULL);
                //Set Gpio for output
                GPIO_WRITE(GPIO_PORTA_DATA_DREC_REG, gpio_out_en);
                //Set Gpio to default value
                GPIO_WRITE(GPIO_PORTA_DATA_REG, output_value);
#ifdef _YL_RTL8723A_B_CUT
                //  TODO: 
                // Set EFUSE bt_general_ctrl.b.gpio_wake_up_polarity to make initial output value as High or Low (according to Host)
                // Then suitable value is set at wake_up_host(): level or pulse
#endif                
            }
        }
#else
#ifndef _SUPPORT_3BITS_HCI_SELECTION_
#ifndef _SUPPORT_2BIT_HCI_AND_PCIE_UART_
    if (g_fun_interface_info.b.hci_sel == RTK8723_S)
#else
    if ((g_fun_interface_info.b.hci_sel == PACKAGE_SDIO_UART)
        ||(g_fun_interface_info.b.hci_sel == PACKAGE_PCIE_UART))
#endif
#else
    if((g_fun_interface_info.b.hci_sel == PACKAGE_SDIO_UART)
        || (g_fun_interface_info.b.hci_sel == PACKAGE_PCIE_UART)
#ifdef _RTL8822B_SPECIFIC_        
        ||(g_fun_interface_info.b.hci_sel == PACKAGE_MPCIE_UART)
#endif
    )
        
#endif            
        { // bt = uart
/*        
#ifndef _SUPPORT_BT_CTRL_FM_        
            int_mask = 0xFE;
            int_en = 0x01;
#else 
            int_mask = 0xF6;
            int_en = 0x09;
#endif
*/

#if defined(_SUPPORT_INFO_FROM_SYSTEM_ON_) && defined(_SUPPORT_BT_CTRL_FM_)  
            if(g_u8support_fm == 0)
#endif                
            {
                int_mask = 0xFE;
                int_en = 0x01;                
            }
#if defined(_SUPPORT_INFO_FROM_SYSTEM_ON_)  && defined(_SUPPORT_BT_CTRL_FM_)
            else
#endif 
            {
#ifdef _SUPPORT_BT_CTRL_FM_             
                int_mask = 0xF6;
                int_en = 0x09;
#endif 

            }
                 




        }
        else
        {
            // bt = usb
            /*
            int_mask = 0xF2;
            int_en = 0x0D;
            */
            // remove bit3 pcie-sus
            int_mask = 0xFA;
            int_en = 0x05;            
        }
        
        EFUSE_POW_SETTING_1_S efuse_pow_setting_1;
        *(UINT8*)&efuse_pow_setting_1 = otp_str_data.efuse_pow_setting_1_d8;
        if (!(efuse_pow_setting_1.force_hw_pdn_en || bton_inter_info.b.gpio_hw_pdn_sts))
        {
            int_mask |= BIT0;
            int_en &= ~BIT0;
        }        

        // TODO: as follows
        /*

        // BIT1, 
        // Note: level/edge and polarity should also programmed
        // Note: LPS-wakeup should also programmed
        if (BTGPIO13 INTR EN) 
        {}
        
        // BIT7,
        // Note: level/edge and polarity should also programmed
        if (BT_3DG_SEL INTR EN) 
        {}

        */

#ifdef _3DD_FUNCTION_SUPPORT_
        if (IS_SUPPORT_3DG_APP)
        {
            /* open 3dg sync and 3dg sel interrupt */
            int_mask &= ~BIT6;
            int_en |= BIT6;
        }
#endif
        
#ifdef _8821A_BTON_DESIGN_     
        EFUSE_POW_SETTING_3_S efuse_pow_setting_3;
        *(UINT8*)&efuse_pow_setting_3 = otp_str_data.efuse_pow_setting_3_d8;

        if (!efuse_pow_setting_3.dont_change_gpio_settings_at_init)
        {
            GPIO_WRITE(GPIO_PORTA_DATA_REG, output_value);
            GPIO_WRITE(GPIO_PORTA_DATA_DREC_REG, gpio_out_en);
        }
        
        if (bt_general_ctrl.b.gpio_wake_up_fun_en)
        {
            set_gpio_bt_wake_host(0);
            //  TODO: 
            // Set EFUSE bt_general_ctrl.b.gpio_wake_up_polarity to make initial output value as High or Low (according to Host)
            // Then suitable value is set at wake_up_host(): level or pulse
        }
#else            

        // NOTE: should also refer to gpio_power_on_check(), which may overwrite the setting
        if (bt_general_ctrl.b.gpio_wake_up_fun_en)
        {
            get_wake_up_pin(&gpio_out_en,&output_value,NULL);
            //Set Gpio for output
            GPIO_WRITE(GPIO_PORTA_DATA_DREC_REG, gpio_out_en);
            //Set Gpio to default value
            GPIO_WRITE(GPIO_PORTA_DATA_REG, output_value);
#ifdef _YL_RTL8723A_B_CUT
            //  TODO: 
            // Set EFUSE bt_general_ctrl.b.gpio_wake_up_polarity to make initial output value as High or Low (according to Host)
            // Then suitable value is set at wake_up_host(): level or pulse
#endif                
        }        
#endif            
#endif

#ifdef _CCH_RETENTION_FLOW_FOR_DLPS_
        if(sleep_mode_param.dlps_restore_flow == FALSE)
#endif
        {
//        RT_BT_LOG(BLUE,LMP_PAYLOAD_INFO,1,gpio_polarity_mask);
            RT_BT_LOG(BLUE,INITI_GPIO_FOR_POWER_2,1,gpio_polarity_mask);
        }

#ifndef _IS_ASIC_
#ifdef _YL_GPIO_POW_TEST_FORCE_EFUSE
        RT_BT_LOG(YELLOW, YL_DBG_HEX_1, 1,
                    otp_str_data.efuse_pow_setting_1_d8 +
                    (otp_str_data.efuse_pow_setting_2_d8<<8) +
                    (otp_str_data.efuse_pow_setting_3_d8<<16)); 
#endif
#endif
        
        
#ifdef _8821A_BTON_DESIGN_
        /*  Writing to the gpio_porta_eoi register has no effect on level-sensitive interrupts. If
         *  level-sensitive interrupts cause the processor to interrupt, then the ISR can poll the gpio_rawint status
         *  register until the interrupt source disappears, or it can write to the gpio_intmask register to mask the
         *  interrupt before exiting the ISR. If the ISR exits without masking or disabling the interrupt prior to
         *  exiting, then the level-sensitive interrupt repeatedly requests an interrupt until the interrupt is cleared at
         *  the source. */
        GPIO_ISR_STS_REG_TYPE gpio_int_type;
        gpio_int_type.d32 = 0x0F;
        
#ifdef _3DD_FUNCTION_SUPPORT_
        if (IS_SUPPORT_3DG_APP)
        {
            gpio_int_type.b.bt_3dg_sync = 1; /* edge trigger */
            gpio_polarity_mask |= BIT6; /* rising edge active */
        }
#endif

        if (efuse_pow_setting_3.gpio_pdn_intr_lvl_trig_en)
        {
            gpio_int_type.b.bt_pon = 0;
        }
        if (efuse_pow_setting_3.gpio_sus_intr_lvl_trig_en)
        {
            gpio_int_type.b.usb_suspend = 0;
#ifndef _REMOVE_HCI_PCIE_
            gpio_int_type.b.pcie_suspend = 0;
#endif
        }

#if defined(_SUPPORT_BT_CTRL_FM_) && defined(_REMOVE_HCI_PCIE_)        
        if (efuse_pow_setting_3.gpio_fm_en_intr_lvl_trig_en)//0x36
        {
            gpio_int_type.b.fm_pad_enable_edge_trig = 0;
        }
#endif
  

        if (!efuse_pow_setting_3.dont_change_gpio_settings_at_init)
        {        
            GPIO_WRITE(GPIO_INT_TYPE_LEVEL_REG, gpio_int_type.d32);	      // 1: edge / 0:level sensitive
        }
#else
        GPIO_WRITE(GPIO_INT_TYPE_LEVEL_REG, 0x0F);	              // 1:edge sensitive
#endif

#ifdef _8821A_BTON_DESIGN_
        if (!efuse_pow_setting_3.dont_change_gpio_settings_at_init)
#endif        
        {
            GPIO_WRITE(GPIO_INT_POLARITY_REG, gpio_polarity_mask);    // 1:rising edge active (active-high detection)
            GPIO_WRITE(GPIO_LS_SYNC, 0x1);                            //sync to pclk_intr
            GPIO_WRITE(GPIO_DEBOUNCE_REG, 0x0F);                      //enable debounce
            GPIO_WRITE(GPIO_PORTA_EOI_REG, 0xFF);                     //clear edge interrupt
            GPIO_WRITE(GPIO_INT_MASK_REG, int_mask);                  //only unmask GPIO 0~ 3 intr
            GPIO_WRITE(GPIO_INT_EN_REG, int_en);                      //only enable GPIO 0~3 intr
        }
    }
    else
    {
        RT_BT_LOG(BLUE,GPIO_FOR_UART,0,0);        
    }
#else
    GPIO_WRITE(GPIO_INT_TYPE_LEVEL_REG, 0x80);	//1:edge sensitive

    if(radio_sw_on)
    {//BT is on when init
        GPIO_WRITE(GPIO_INT_POLARITY_REG, 0x00);//0:falling edge active
    }
    else
    {//BT is off when init
        GPIO_WRITE(GPIO_INT_POLARITY_REG, 0x80);//1:rising edge active
    }

    GPIO_WRITE(GPIO_LS_SYNC, 0x1);				//sync to pclk_intr
    GPIO_WRITE(GPIO_DEBOUNCE_REG, 0x80);		//enable debounce
    GPIO_WRITE(GPIO_PORTA_EOI_REG, 0xff);		//clear edge interrupt
    GPIO_WRITE(GPIO_INT_MASK_REG, 0x7f);		//only unmask GPIO7 intr
    GPIO_WRITE(GPIO_INT_EN_REG, 0x80);			//only enable GPIO7 intr
#endif


#ifdef _FONGPIN_TEST_MUX_DEV_WAKEUP_HOST_BY_GPIO13_
BTON_POW_CTRL_REG_S_TYPE pow_ctrl;
pow_ctrl.d32 = VENDOR_READ(BTON_POW_CTRL_REG);
pow_ctrl.b.bt_gpio13_wk_wakeup_en = 1;
pow_ctrl.b.bt_gpio13_wk_wakeup_polarity = 1;
VENDOR_WRITE(BTON_POW_CTRL_REG,pow_ctrl.d32);

pow_ctrl.d32 = VENDOR_READ(BTON_POW_CTRL_REG);
#ifndef _REMOVE_INDIRECT_READ_PAGE0_
UINT8 Page0_0x65 = RD_8BIT_COMBO_SYSON_IO(0x65);// check bit 3, 7 = 1,0
UINT8 Page0_0x4e = RD_8BIT_COMBO_SYSON_IO(0x4e);// check bit 6 = 1
#else
UINT8 Page0_0x65 = RD_8BIT_SYSON_IO(0x65);// check bit 3, 7 = 1,0
UINT8 Page0_0x4e = RD_8BIT_SYSON_IO(0x4e);// check bit 6 = 1
#endif
RT_BT_LOG(BLUE, YL_DBG_HEX_5, 5,Page0_0x4e, Page0_0x65, pow_ctrl_int_rec.b.bt_gpio13_wk_wakeup_en,pow_ctrl.b.bt_gpio13_wk_wakeup_en, pow_ctrl.b.bt_gpio13_wk_wakeup_polarity );
#endif



}
    
SECTION_ISR_LOW UINT8 gpio_bt_pwr_down_isr(GPIO_BT_ISR_MANAGER_S *argu)
{
    UINT8 bit_polarity_sts;
    UINT16 pdn_delay_time;
    GENERAL_CONTROL_S_TYPE bt_general_ctrl;    

    bt_general_ctrl.d16 = otp_str_data.general_control;    
    pdn_delay_time = otp_str_data.pdn_delay_time_b0;

    argu->state |= BIT0;
    //RT_BT_LOG(RED,BT_PDN_ENABLE,0,0);

    if (!process_power_gpio_set( 0x1, &bit_polarity_sts, FALSE))
    {           
       argu->state |= BIT1;
       //RT_BT_LOG(RED, BT_PDN_FALSE_ALARM,0,0);
       return TRUE;
    }
   
#ifdef _ENABLE_USB_REMOTE_WAKEUP_GPIO11
#ifdef _ROM_CODE_PATCHED_
#ifdef _DAPE_8723_RCP_
//    if (rcp_usb_remote_wakeup_gpio11_recv_power_dn_func != NULL)
//    {
//        rcp_usb_remote_wakeup_gpio11_recv_power_dn_func();
//    }
#endif	
#endif
#endif

    if (bit_polarity_sts == argu->bton_inter_info.b.polarity_bt_gpio_sts) 
    {
        //set bt power down bit
        argu->pow_ctrl.b.bt_hwpdn_en = 1;
#ifndef _8821A_BTON_DESIGN_
        if (!g_fun_interface_info.b.gps_fun_sts)
        {
            pow_ctrl.b.gps_hwpdn_en = 1;
        }
#endif
        argu->state |= BIT2;
        //RT_BT_LOG(BLUE,BTCORE_SET_PDN,0,0);
        
        //for hw work around; need delay some time
        if ( bt_general_ctrl.b.pdn_delay_time_unit == USEC)
        {
            argu->state |= BIT3;              
            pf_delay_us(pdn_delay_time);
        }
        else
        {
            pf_delay(pdn_delay_time);
        }
        
    }
    else 
    {            
        //clear bt power down bit
        argu->pow_ctrl.b.bt_hwpdn_en = 0;
#ifndef _8821A_BTON_DESIGN_
        if (!g_fun_interface_info.b.gps_fun_sts)
        {
            pow_ctrl.b.gps_hwpdn_en = 0;
        }
#endif
        //RT_BT_LOG(BLUE,BTCORE_CLEAR_PDN,0,0);
    }

    return FALSE;
}

#ifndef _8821A_BTON_DESIGN_
SECTION_ISR_LOW UINT8 gpio_gps_pwr_down_isr(GPIO_BT_ISR_MANAGER_S *argu)
{
    UINT8 bit_polarity_sts;
    UINT16 pdn_delay_time;
    GENERAL_CONTROL_S_TYPE bt_general_ctrl;    

    bt_general_ctrl.d16 = otp_str_data.general_control;    
    pdn_delay_time = otp_str_data.pdn_delay_time_b0;

    argu->state |= BIT4;  
    //RT_BT_LOG(RED,GPS_PDN_ENABLE,0,0);

    if (!process_power_gpio_set( 0x2, &bit_polarity_sts, FALSE))
    {			
        argu->state |= BIT5;  
        //RT_BT_LOG(RED, GPS_PDN_FALSE_ALARM,0,0);
        return TRUE;
    }

    if (bit_polarity_sts == bton_inter_info.b.polarity_gps_gpio_sts) 
    {
        //set bt power down bit
        argu->pow_ctrl.b.gps_hwpdn_en = 1;
        
        if (!g_fun_interface_info.b.bt_fun_sts)
        {
            argu->pow_ctrl.b.bt_hwpdn_en = 1;
        }
        argu->state |= BIT6; 
        //RT_BT_LOG(BLUE,BTCORE_SET_PDN,0,0);
        
        if ( bt_general_ctrl.b.pdn_delay_time_unit == USEC)
        {
            pf_delay_us(pdn_delay_time);
        }
        else
        {
            pf_delay(pdn_delay_time);
        }
    }
    else 
    {            
        //clear bt power down bit
        argu->pow_ctrl.b.gps_hwpdn_en = 0;

        if (!g_fun_interface_info.b.bt_fun_sts)
        {
            argu->pow_ctrl.b.bt_hwpdn_en = 0;
        }
        argu->state |= BIT7; 
        //RT_BT_LOG(BLUE,BTCORE_CLEAR_PDN,0,0);
    }
    return FALSE;
}    
#endif

SECTION_ISR_LOW UINT8 gpio_usb_suspend_isr(GPIO_BT_ISR_MANAGER_S *argu)
{
    UINT8 bit_polarity_sts;
    GENERAL_CONTROL_S_TYPE bt_general_ctrl;    

    bt_general_ctrl.d16 = otp_str_data.general_control;    

    /*  Added by Wallice Su for USB LPM.    2012/03/01  */
    BTON_USB_LPM_REG_S_TYPE bton_USB_LPM_reg;

    argu->state |= BIT8; 
#ifndef _8821A_BTON_DESIGN_            
    RT_BT_LOG(RED,USB_SUSPEND_ENABLE,0,0);
#endif
    if (!process_power_gpio_set( 0x4, &bit_polarity_sts, TRUE))
    {           
        argu->state |= BIT9; 
        //RT_BT_LOG(RED, USB_SUS_FALSE_ALARM,0,0);
        return TRUE;
    }

    if (g_fun_interface_info.b.bt_interface == UART_INTERFACE)
    {
        /* move here to avoid no clear gpio interrupt */
        return TRUE;
    }

#ifdef _ROM_CODE_PATCHED_
#ifdef _YL_RTL8723A_B_CUT
    /*
    TODO: optionally enable remote wakeup by Vendor Command or EFUSE opt_str_data
    TODO: maintain suitable/enough  information for USB/PCIE RemoteWakeup Function Decisions, e.g. EVT filtering, resuming, ...
    TODO: set suitable scan enable and parameters (by PATCH)
    if(bt_pow_man.b.usb_remote_wakeup_en) // determined by EFUSE and Vendor Command 
    {
        if (bit_polarity_sts)
        {
            g_host_state = 1;
        }
        else
        {
            g_host_state = 0;
            if(argu->bt_pow_man.b.usb_suspend_hci_rx_buffer_en)
            {
                if (argu->bt_pow_man.b.usb_suspend_hci_rx_resch_opt == 1)
                {
                    by direct call
                }
                else if (argu->bt_pow_man.b.usb_suspend_hci_rx_resch_opt == 2)
                {
                    by timer callback (x+1)*10ms
                }
                else
                {
                    // NONE
                }                 
            }
        }
        pow_ctrl.b.bt_sus_en = 0;
        if ((g_fun_interface_info.b.hci_sel == RTK8723_U) ||
            (!g_fun_interface_info.b.gps_fun_sts))
        {
            argu->pow_ctrl.b.gps_sus_en = 0; *** ?????? required???
        }            
        VENDOR_WRITE(BTON_POW_CTRL_REG, argu->pow_ctrl.d32);
        return TRUE;
    }
    */
    
    /* yilinli, 
     *  HOST ACT => SUS:
     *      update g_host_state, g_host_wake_bt, g_wake_first_time
     *      read remote wakeup feature
     *      decide r_LOP_EXTL = 0 (other r_LOP_xxx = ?)
     *      if (remote_wakeup_feature SET and enabled)
     *             bt_sus_en = 1 (gps_sus_en = ?)
     *      else
     *             bt_sus_en = 0 (gps_sus_en = ?)
     *      decide SCAN or not
     *
     * HOST SUS ==> ACT:
     *     update g_host_state, g_host_wake_bt, g_wake_first_time
     *     r_LOP_EXTL = 0 (other r_LOP_xxx = ?)
     *     bt_sus_en = 0 (gps_sus_en = ?)
     *     kill SCAN or not ??
     *     (optional) if(g_hci_buf_rx)
     *           (opt.) re-sch
     *           g_hci_buf_rx = 0
      */
//        if(rcp_gpiointr_usb_suspend_func != NULL)
//        {
//            if(rcp_gpiointr_usb_suspend_func((void *)&bit_polarity_sts, &argu->gpio_status, &argu->pow_ctrl))
//                return;
//        }
#endif
#endif
    
    if (bit_polarity_sts) 
    {
        //set suspend bit

#if defined(_SUPPORT_USB_SS_REMOTE_WAKEUP_ON_BT_) || \
    defined(_SUPPORT_USB_LPM_L1_REMOTE_WAKEUP_ON_BT_)
        g_host_state = 1;
#endif                

/*  Updated by Wallice Su for USB LPM.  2012/03/15  */                
//                argu->pow_ctrl.b.bt_sus_en = 1;
        argu->pow_ctrl.b.bt_sus_en = 0;

#if 0				
#ifndef _8821A_BTON_DESIGN_
        //USB multi or gps disable: fw need to set gps suspend enable
        if ((g_fun_interface_info.b.hci_sel == RTK8723_U) ||
            (!g_fun_interface_info.b.gps_fun_sts))
        {
            argu->pow_ctrl.b.gps_sus_en = 1;
        }
#endif                
#endif

/*  End Updated by Wallice Su for USB LPM.  2012/03/15  */                

/*  Added by Wallice Su for USB LPM.    2012/03/01  */
        bton_USB_LPM_reg.d32 = VENDOR_READ(BTON_USB_LPM);

#ifndef _SUPPORT_POLLING_BASED_LPM_L1_
        if (bton_USB_LPM_reg.b.Suspend_Mode)//L1 Suspend
        {
            if (bton_USB_LPM_reg.b.LPM_Allow)
            {
                //L1 suspend
                argu->state |= BIT16; 
                //RT_BT_LOG(BLUE, MSG_USBLPM_SUSPEND_L1, 0,0);

                //HIRD/BESL Value
                if (bton_USB_LPM_reg.b.HIRD_BESL_Ind)
                {
                    argu->state |= BIT17; 
                    //RT_BT_LOG(BLUE, MSG_USBLPM_SUSPEND_L1_HIRDBESL_VAL, 2, bton_USB_LPM_reg.b.HIRD_BESL_Ind, bton_USB_LPM_reg.b.HIRD_BESL_Val);                   
                }
            }
            else
            {
                //Error, it should not happen
                argu->state |= BIT18; 
                //RT_BT_LOG(BLUE, MSG_USBLPM_SUSPEND_L1_ERR, 0, 0);
            }
#ifdef _SUPPORT_FW_INDIRECT_READ_SIE_
            UINT8 u8sieValue;
            u8sieValue = safe_indirect_read_sie(0, 0x10);

            if(u8sieValue&BIT6)
            //if (bton_USB_LPM_reg.b.RMT_WkUp)
            {
                //L1 RemoteWakeup enable
                argu->state |= BIT19;
                RT_BT_LOG(WHITE, FONGPIN_SIE_REG,2,0x10, u8sieValue);

                usb_remote_wakeup();
                //RT_BT_LOG(BLUE, MSG_USBLPM_SUSPEND_L1_RMTWKUP_EN, 0, 0);
                
            }
            else
#endif                
            {
                //L1 RemoteWakeup disable
                argu->state |= BIT20; 
                //RT_BT_LOG(BLUE, MSG_USBLPM_SUSPEND_L1_RMTWKUP_DIS, 0, 0);
            }
        }
        else//L2 Suspend
#endif
       
        {
            argu->state |= BIT21; 
            //RT_BT_LOG(BLUE, MSG_USBLPM_SUSPEND_L2, 0,0);    
            
            argu->pow_ctrl.b.bt_sus_en = 1; // original suspend, D3

#ifndef _8821A_BTON_DESIGN_
            //USB multi or gps disable: fw need to set gps suspend enable
            if ((g_fun_interface_info.b.hci_sel == RTK8723_U) ||
                (!g_fun_interface_info.b.gps_fun_sts))
            {
                argu->pow_ctrl.b.gps_sus_en = 1;
            }
#endif   

            //L2 suspend
            if (bton_USB_LPM_reg.b.L2RMT_WkUp)
            {
                //L2 RemoteWakeup enable
                argu->state |= BIT22;
#ifdef _SUPPORT_USB_LOG_ENABLE_
                if(g_usb_misc&BIT0)
                    RT_BT_LOG(BLUE, MSG_USBLPM_SUSPEND_L2_RMTWKUP_EN, 0, 0);
#endif

#if defined(_SUPPORT_USB_SS_REMOTE_WAKEUP_ON_BT_)
                /* allow remote wakeup from selected suspend */
                argu->pow_ctrl.b.bt_sus_en = 0;
#endif
            }
            else
            {
                //L2 RemoteWakeup disable
                argu->state |= BIT23;
#ifdef _SUPPORT_USB_LOG_ENABLE_        
        if(g_usb_misc&BIT0)                
                RT_BT_LOG(BLUE, MSG_USBLPM_SUSPEND_L2_RMTWKUP_DIS, 0, 0);
#endif        
            }
        }   
/*  End Added by Wallice Su for USB LPM.    2012/03/01  */

        argu->state |= BIT10; 
        //RT_BT_LOG(BLUE,BTCORE_SET_PDN,0,0);
    }
    else 
    {            
#if defined(_SUPPORT_USB_SS_REMOTE_WAKEUP_ON_BT_) || \
    defined(_SUPPORT_USB_LPM_L1_REMOTE_WAKEUP_ON_BT_)
        g_host_state = 0;
#endif                

        //clear bt suspend bit
        argu->pow_ctrl.b.bt_sus_en = 0;
#ifndef _8821A_BTON_DESIGN_                
        if ((g_fun_interface_info.b.hci_sel == RTK8723_U) ||
            (!g_fun_interface_info.b.gps_fun_sts))
        {
            argu->pow_ctrl.b.gps_sus_en = 0;
        }
#endif                
        argu->state |= BIT11; 
#ifdef _SUPPORT_USB_LOG_ENABLE_         
        if(g_usb_misc&BIT0)
            RT_BT_LOG(BLUE,BTCORE_CLEAR_PDN,0,0);
#endif
    }    
    return FALSE;
}

#if defined(_SUPPORT_BT_CTRL_FM_) && defined(_REMOVE_HCI_PCIE_)
SECTION_ISR_LOW UINT8 gpio_fm_pad_en_isr(GPIO_BT_ISR_MANAGER_S *argu)
{
    UINT8 bit_polarity_sts;

    argu->state |= BIT12; 


    if (!process_power_gpio_set( 0x8, &bit_polarity_sts, FALSE))
    {
        argu->state |= BIT13; 

        return TRUE;
    }


    if (g_fun_interface_info.b.bt_interface == USB_INTERFACE)
    {
        // illegal
        return TRUE;
    }

    if (bit_polarity_sts) // 0, !0
    {
        // should i check FM alive?
        fm_ctrl.hw_enable = 1;
        argu->state |= BIT14;     		   
        //RT_BT_LOG(BLUE,FONGPIN_FM_PAD_ENABLE,1,bit_polarity_sts);
     }
    else 
    {
        fm_ctrl.hw_enable = 0;
        argu->state |= BIT15;  
        //RT_BT_LOG(BLUE,FONGPIN_FM_PAD_ENABLE,1,bit_polarity_sts);
    }
    //fm_get_ready();
    //RT_BT_LOG(BLUE,YL_DBG_HEX_1,1,fm_get_ready());
    //RT_BT_LOG(BLUE,FONGPIN_FM_PAD_ENABLE,1,bit_polarity_sts);
    return FALSE;
}

#endif

#ifndef _REMOVE_HCI_PCIE_
SECTION_ISR_LOW UINT8 gpio_pcie_suspend_isr(GPIO_BT_ISR_MANAGER_S *argu)
{
    UINT8 bit_polarity_sts;

    argu->state |= BIT12; 
    //RT_BT_LOG(RED,USB_SUSPEND_ENABLE,0,0);

    if (!process_power_gpio_set( 0x8, &bit_polarity_sts, FALSE))
    {
        argu->state |= BIT13; 
        //RT_BT_LOG(RED, PCIE_SUS_FALSE_ALARM,0,0);
        return TRUE;
    }

    if (g_fun_interface_info.b.bt_interface == UART_INTERFACE)
    {
        /* move here to avoid no clear gpio interrupt */
        return TRUE;
    }

    if (bit_polarity_sts) 
    {
#ifndef _8821A_BTON_DESIGN_
        //set suspend bit
        argu->pow_ctrl.b.gps_sus_en = 1;
#endif
#ifndef _SUPPORT_3BITS_HCI_SELECTION_ /* dape(svn13680): since there is no "EE" need. We marked this. */
#ifndef _SUPPORT_2BIT_HCI_AND_PCIE_UART_
        //PCIE multi or bt disable: fw need to set gps suspend enable
        if ((g_fun_interface_info.b.hci_sel == RTK8723_EE) ||
            (!g_fun_interface_info.b.bt_fun_sts))
        {
            argu->pow_ctrl.b.bt_sus_en = 1;
        }
#else
    // do nothing
#endif
#endif

        argu->state |= BIT14;     		   
        //RT_BT_LOG(BLUE,BTCORE_SET_PDN,0,0);
     }
    else 
    {			  
#ifndef _8821A_BTON_DESIGN_            
        //clear suspend bit
        argu->pow_ctrl.b.gps_sus_en = 0;
#endif
#ifndef _SUPPORT_3BITS_HCI_SELECTION_ /* dape(svn13680): since there is no "EE" need. We marked this. */
#ifndef _SUPPORT_2BIT_HCI_AND_PCIE_UART_
        //PCIE multi or gps disable: fw need to set gps suspend enable
        if ((g_fun_interface_info.b.hci_sel == RTK8723_EE) ||
            (!g_fun_interface_info.b.bt_fun_sts))
        {
            argu->pow_ctrl.b.bt_sus_en = 0;
        }
#else
    // do nothing
#endif
#endif  

        argu->state |= BIT15;  
        //RT_BT_LOG(BLUE,BTCORE_CLEAR_PDN,0,0);
    }  
    return FALSE;
}

#endif


#ifdef _3DD_FUNCTION_SUPPORT_
SECTION_ISR_LOW UINT8 gpio_3dd_sync_isr(GPIO_BT_ISR_MANAGER_S *argu)
{
    UINT8 bit_polarity_sts;

    argu->state |= BIT24; 

    if (!process_power_gpio_set(BIT6, &bit_polarity_sts, TRUE))
    {
        return TRUE;
    }

    argu->state |= BIT25; 
#ifdef _DAPE_TEST_USE_GPIO1_AS_FRAME_SYNC
#ifdef _DAPE_FRAME_SYNC_PROCESS_TO_BG

    OS_SIGNAL sig_send;
    sig_send.type = ISR_EXT_FRAME_SYNC_INTR;  
    OS_ISR_SEND_SIGNAL_TO_TASK(isr_extended_task_handle, sig_send); 

#endif
#endif

    if (bit_polarity_sts)
    {                   
        argu->state |= BIT26; 
#ifndef _DAPE_TEST_USE_GPIO1_AS_FRAME_SYNC
#ifdef VER_CSA4
#ifdef _SUPPORT_CSB_TRANSMITTER_
        bt_csb_driver_handle_ext_clock_capture_by_edge_trigger();
#endif
#endif
#endif
    }
    
    return FALSE;
}
#endif

void auto_detach_enable_link_timer_task(void *mode, uint32_t time_ms)
{
    auto_detach_enable_link_timer((UINT8)(UINT32) mode, (UINT16) time_ms);
}

SECTION_ISR_LOW void GpioIntrHandler(void)
{
    GPIO_BT_ISR_MANAGER_S gpio_argu;
    
    gpio_argu.state = 0;   
    gpio_argu.bton_inter_info.d32 = VENDOR_READ(BTON_INTERFACE_CTRL_REG);
    gpio_argu.gpio_status.d32 = GPIO_READ(GPIO_INT_STATUS_REG);
    gpio_argu.pow_ctrl.d32 = VENDOR_READ(BTON_POW_CTRL_REG);
    gpio_argu.pow_option.d32 = VENDOR_READ(BTON_POW_OPTION_AND_32K_CTRL);
       
#ifdef _ROM_CODE_PATCHED_
    if (rcp_GpioIntrHandler != NULL)
    {
        /* we can handle all GPIO patch in this function */
        if (rcp_GpioIntrHandler(&gpio_argu))
        {
            return;
        }
    }
#endif

    do 
    {
        //BT Power Down interrupt
        if (gpio_argu.gpio_status.b.bt_pon)
        {
            if (gpio_bt_pwr_down_isr(&gpio_argu))
            {
                break;
            }
        }
        
#ifndef _8821A_BTON_DESIGN_
        //GPS Power down interrupt
        if (gpio_argu.gpio_status.b.gps_pon)
        {
            if (gpio_gps_pwr_down_isr(&gpio_argu))
            {
                break;
            }            
        }
#endif
        //handle usb suspend
        if (gpio_argu.gpio_status.b.usb_suspend)
        {
            if (gpio_usb_suspend_isr(&gpio_argu))
            {
                break;
            }                
        }


#if defined(_SUPPORT_BT_CTRL_FM_) && defined(_REMOVE_HCI_PCIE_)
    //handle fm enable and disable (pad_fm_en)
#if defined(_SUPPORT_INFO_FROM_SYSTEM_ON_)
    if(g_u8support_fm == 1)
#endif        
    {
        if (gpio_argu.gpio_status.b.fm_pad_enable_edge_trig)
        {
            if (gpio_fm_pad_en_isr(&gpio_argu))
            {
                break;
            }             
        }
    }
#if 0
	//in order print msg: GPIO INT MSG: -- State:5000
	//in order print msg: GPIO INT MSG: -- State:9000	
    if (!(gpio_argu.gpio_status.d32 & ~BIT3))
    {
        /* only fm interrupt occur !! */
        return;
    }
#endif
#endif
#ifndef _REMOVE_HCI_PCIE_
    //handle Pcie suspend and resume
    if (gpio_argu.gpio_status.b.pcie_suspend)
    {
        if (gpio_pcie_suspend_isr(&gpio_argu))
        {
            break;
        }             
    }
#endif


        
#if defined _3DD_FUNCTION_SUPPORT_ || defined (_DAPE_TEST_USE_GPIO1_AS_FRAME_SYNC)
#ifndef _3DD_FUNCTION_SUPPORT_
        if (_DAPE_TEST_USE_GPIO1_AS_FRAME_SYNC)
#endif            
        {
            if (gpio_argu.gpio_status.b.bt_3dg_sync)
            {
                if (gpio_3dd_sync_isr(&gpio_argu))
                {
                    break;
                }                  
            }

            if (!(gpio_argu.gpio_status.d32 & ~BIT6))
            {
                /* only 3dg interrupt occur !! */
                return;
            }
        }
#endif

        if (gpio_argu.pow_ctrl.b.bt_hwpdn_en)
        {                        
#ifdef _SUPPORT_AUTO_DETACH_LINK_ 
            lmp_self_device_data.scan_enable = 0;
            lc_start_write_scan_mode(0);
            if (auto_detach_pend_terminate_all_remote_links_from_isr() > 0)
            {
#ifndef _SUPPORT_3BITS_HCI_SELECTION_
#ifndef _SUPPORT_2BIT_HCI_AND_PCIE_UART_
                if (g_fun_interface_info.b.hci_sel != RTK8723_U)
#else
                if (g_fun_interface_info.b.hci_sel != PACKAGE_USB_USB)
#endif
#else
                if (g_fun_interface_info.b.hci_sel != PACKAGE_USB_USB)
#endif
                {
                    gpio_argu.pow_ctrl.b.bt_hwpdn_en = 0;

                    BaseType_t high_pri_task_woken = pdFALSE;
                    /* do not need to trigger detach timer in 8723AU,
                       the callback function will enable hw power down */
                    xTimerPendFunctionCallFromISR(
                            auto_detach_enable_link_timer_task,
                            (void *) AUTO_DETACH_PROCESS_HW_RADIO_OFF, 1500,
                            &high_pri_task_woken);
                    portYIELD_FROM_ISR(high_pri_task_woken);
                }              
            }
#endif

#ifndef _SUPPORT_3BITS_HCI_SELECTION_
#ifndef _SUPPORT_2BIT_HCI_AND_PCIE_UART_
            if (g_fun_interface_info.b.hci_sel == RTK8723_U)
#else
            if (g_fun_interface_info.b.hci_sel == PACKAGE_USB_USB)
#endif
#else
            if (g_fun_interface_info.b.hci_sel == PACKAGE_USB_USB)
#endif
                
            {      
                /* do not enter hw power down in 8723AS-VAU or 8723AU */                    
                gpio_argu.pow_ctrl.b.bt_hwpdn_en = 0; 
#ifndef _8821A_BTON_DESIGN_              
                gpio_argu.pow_ctrl.b.gps_hwpdn_en = 0;   
#endif
            }
        }
       
#ifdef _YL_RTL8723A_B_CUT
        EFUSE_POW_SETTING_1_S efuse_pow_setting_1;
        *(UINT8*)&efuse_pow_setting_1 = otp_str_data.efuse_pow_setting_1_d8;

#if !defined(_IS_ASIC_)
#ifdef _DAPE_IOT_FOR_TOSHIBA
        efuse_pow_setting_1.init_bt_never_close_sram_pwr = 1;
#endif        
#endif

        if (efuse_pow_setting_1.init_bt_never_close_sram_pwr)
        {
            /* for some reason, maybe we do not enter power down or 
               suspend mdoe - austin */
#ifdef _8821A_BTON_DESIGN_
            gpio_argu.pow_ctrl.b.bt_hwpdn_en = 0;
            gpio_argu.pow_ctrl.b.bt_sus_en = 0;
#else
            gpio_argu.pow_ctrl.d32 &= ~(BIT23|BIT22|BIT21|BIT20);
#endif            
        }
#endif
#ifdef _8821A_BTON_DESIGN_
        EFUSE_POW_SETTING_2_S efuse_pow_setting_2;
        *(UINT8*)&efuse_pow_setting_2 = otp_str_data.efuse_pow_setting_2_d8;
        if (efuse_pow_setting_2.gpiointr_ignore_sus)
        {
            gpio_argu.pow_ctrl.b.bt_sus_en = 0;
        }
#endif

#ifdef _YL_USB_DYNAMIC_LPS_XTAL_EN_
        {
            EFUSE_POW_SETTING_4_S efuse_pow_setting_4;
            *(UINT8*)&efuse_pow_setting_4 = otp_str_data.efuse_pow_setting_4_d8;
            if ((efuse_pow_setting_4.usb_dynamic_lps_xtal_en) && (g_fun_interface_info.b.bt_interface == USB_INTERFACE))
            {
                // SUSPEND Interrupt occurs
                if (gpio_argu.gpio_status.b.usb_suspend)
                {
                    // Host(USB-SIE) enter SUSPEND and BT Keep Alive
                    if ((g_host_state==1) && (!gpio_argu.pow_ctrl.b.bt_hwpdn_en) && (!gpio_argu.pow_ctrl.b.bt_sus_en))
                    {
                        lps_lop_setting_set_xtal_en(0);
                    }
                    // Host(USB-SIE) RESUME
                    else if (g_host_state==0)
                    {
                        lps_lop_setting_set_xtal_en(1);
                    }
                }
            }
        }
#endif

        execute_bton_entering_pdn_sus(&gpio_argu.pow_ctrl);
        
    }
    while (0);  

    RT_BT_LOG(BLUE, MSG_GPIO_INTR, 5, gpio_argu.state, gpio_argu.bton_inter_info.d32,
                                      gpio_argu.gpio_status.d32, gpio_argu.pow_ctrl.d32, 
                                      gpio_argu.pow_option.d32);    
}


BOOLEAN process_power_gpio_set( UINT8 bit_offset,UINT8 * bit_polarity_sts, 
                                        UINT8 no_delay)
{
    UINT32 isr_polarity_sts;

#ifdef _ROM_CODE_PATCHED_
#ifdef _YL_RTL8723A_B_CUT
    /*
     * preferred processing:
     *      set as level-triggered
     *      debouncing and read raw value
     *      if fake report
     *          return FALSE;
     *      else
     *          change the active-level if needed
     *          return TRUE;
     *  Q: ???if(!(gpio_raw_status.d32 >> check_offset))
     */
#endif
    if (rcp_process_power_gpio_set != NULL)
    {
        UINT8 result = 0;
        if (rcp_process_power_gpio_set((void *)&bit_offset, 
                                            bit_polarity_sts, 
                                            no_delay,
                                            &result))
        {
            return result;
        }
    }
#endif

    UINT8 is_rising_edge = FALSE;
    UINT8 is_detect_high = FALSE;
    
    //Sofeware GPIO Debunce	    
    if (!no_delay)
    {
        pf_delay(otp_str_data.pdn_delay_time_b1);
    }
    
    //clear interrupt
    GPIO_WRITE(GPIO_PORTA_EOI_REG, bit_offset); 

    isr_polarity_sts = GPIO_READ(GPIO_INT_POLARITY_REG);

    if (isr_polarity_sts & bit_offset)
    {
        
        //RT_BT_LOG(WHITE, FONGPIN_IS_RISING_EDGE, 0,0); 
       is_rising_edge = TRUE;
    }
    else
    {
        //RT_BT_LOG(WHITE, FONGPIN_IS_NOT_RISING_EDGE, 0,0);
    }
    
    if (GPIO_READ(GPIO_EXT_PORTA_REG) & bit_offset)
    {
        //RT_BT_LOG(WHITE, FONGPIN_IS_DETECT_HIGH, 0,0);    
        is_detect_high = TRUE;
    }
    else
    {
        //RT_BT_LOG(WHITE, FONGPIN_IS_NOT_DETECT_HIGH, 0,0);
    }   

    if ((is_detect_high != is_rising_edge)) /* to execute XOR logic */
    {
        //EFUSE_POW_SETTING_3_S efuse_pow_setting_3;
        //*(UINT8*)&efuse_pow_setting_3 = otp_str_data.efuse_pow_setting_3_d8;

        //if (efuse_pow_setting_3.gpiointr_no_break) /* yilinli */
        //{
        //    return TRUE;
        //}

        //if (efuse_pow_setting_3.gpiointr_bypass_debunce == 0) /* yilinli */
        //{
             return FALSE; //fake report
        //}

        /* mark by austin -- many system bug in 8723b after 
           we do not use sw swbounce, because some can become glitch for hw   */
    }
    
    //restore polarity status for checking if it need to be power down
    *bit_polarity_sts = (isr_polarity_sts & bit_offset);
    isr_polarity_sts ^= bit_offset;    
    GPIO_WRITE(GPIO_INT_POLARITY_REG, isr_polarity_sts);

    //RT_BT_LOG(BLUE,GPIO_INT_POLARITY_STS,1,isr_polarity_sts);

    return TRUE;
}

void gpio_power_on_check(void)
{
    BTON_INTERFACE_CTRL_REG_S_TYPE bton_inter_info;
    GPIO_PIN_S_TYPE gpio_pin;
    BTON_POW_CTRL_REG_S_TYPE pow_ctrl;

#ifdef _YL_RTL8723A_B_CUT    
    BTON_UART_INFO_REG_S_TYPE bton_uart_info;
    EFUSE_POW_SETTING_1_S efuse_pow_setting_1;
    *(UINT8*)&efuse_pow_setting_1 = otp_str_data.efuse_pow_setting_1_d8;	
#else
     /**** unexpected for RTL8723A B_CUT ****/
#endif

    UINT8 pw_execute = 0;
    UINT16 state = 0;

#ifdef _ROM_CODE_PATCHED_
    if (rcp_gpio_power_on_check != NULL)
    {
        if (rcp_gpio_power_on_check(NULL))
        {
            return;
        }
    }
#endif

    bton_inter_info.d32 = VENDOR_READ(BTON_INTERFACE_CTRL_REG);
    gpio_pin.d32 = GPIO_READ(GPIO_EXT_PORTA_REG);
    pow_ctrl.d32 = VENDOR_READ(BTON_POW_CTRL_REG);
#ifndef _8821A_BTON_DESIGN_
    RT_BT_LOG(BLUE,MSG_PDN_GPIO_POLARTIY, 3, gpio_pin.d32,
                                       bton_inter_info.b.polarity_bt_gpio_sts,
                                       bton_inter_info.b.polarity_gps_gpio_sts);
#else                                       
    bton_uart_info.d32 = VENDOR_READ(BTON_UART_INFO_REG);
#endif

    //Check BT power down
    if (g_fun_interface_info.b.bt_fun_sts)
    {
        state |= BIT0;

#ifdef _YL_RTL8723A_B_CUT
#ifdef _8821A_BTON_DESIGN_ 
        if (    (efuse_pow_setting_1.bt_pdn_power_on_check_dis == 0) &&
                (gpio_pin.b.bt_pdn == bton_inter_info.b.polarity_bt_gpio_sts)    )
#else
        if (    (pow_ctrl.b.bt_gpio_wakeup_sts || efuse_pow_setting_1.init_bt_pdn_check_wakeupsts_dis) &&
                (efuse_pow_setting_1.init_bt_pdn_check_dis == 0) &&
                (gpio_pin.b.bt_pdn == bton_inter_info.b.polarity_bt_gpio_sts)    )
#endif
#else
        /**** unexpected for RTL8723A B_CUT ****/
        if (pow_ctrl.b.bt_gpio_wakeup_sts &&
            (gpio_pin.b.bt_pdn == bton_inter_info.b.polarity_bt_gpio_sts))
#endif            
        {
#ifdef _8821A_BTON_DESIGN_        
            if ( (efuse_pow_setting_1.force_hw_pdn_en || bton_inter_info.b.gpio_hw_pdn_sts) )
#endif            
            {
                state |= BIT1;
                
                //RT_BT_LOG(RED, BTCORE_SET_PDN, 0, 0);
                pow_ctrl.b.bt_hwpdn_en = 1;
#ifndef _8821A_BTON_DESIGN_
                if (!g_fun_interface_info.b.gps_fun_sts)
                {
                    pow_ctrl.b.gps_hwpdn_en = 1;
                    state |= BIT2;

                    //RT_BT_LOG(RED,GPS_STATUS_DISABLE, 0, 0);
                }
#endif                
                pw_execute = 1;
            }
        }                      
    }
#ifdef _8821A_BTON_DESIGN_
    else /* when g_fun_interface_info.b.bt_fun_sts = 0, BTON Should keep in IDLE STATE */
    {
        if (efuse_pow_setting_1.bt_fun_sts_power_on_check_en)
        {
            state |= BIT2;
            pow_ctrl.b.bt_hwpdn_en = 1;
            pw_execute = 1;
        }
    }
#endif

#ifndef _8821A_BTON_DESIGN_
    //Check GPS power down
    if (g_fun_interface_info.b.gps_fun_sts)
    {
        state |= BIT3;

        if (pow_ctrl.b.gps_gpio_wakeup_sts &&
            (gpio_pin.b.gps_pdn == bton_inter_info.b.polarity_gps_gpio_sts))
        {
                state |= BIT4;
                //RT_BT_LOG(RED, GPS_PDN_ENABLE, 0, 0);
                pow_ctrl.b.gps_hwpdn_en = 1;
                if (!g_fun_interface_info.b.bt_fun_sts)
                {
                    pow_ctrl.b.bt_hwpdn_en = 1;
                    state |= BIT5;
                    //RT_BT_LOG(RED,BT_STATUS_DISABLE, 0, 0);
                }
                pw_execute = 1;
        }        
    }
#endif

    //Check USB suspend; low active
#ifdef _8821A_BTON_DESIGN_
    if (gpio_pin.b.usb_suspend && (efuse_pow_setting_1.bt_suspend_power_on_check_dis == 0))
#else    
    if (gpio_pin.b.usb_suspend)
#endif    
    {
        state |= BIT6;
        //RT_BT_LOG(RED, USB_SUSPEND_ENABLE, 0, 0);

        //set suspend bit
        pow_ctrl.b.bt_sus_en = 1; // TODO: need to check hci_sel ??
#ifndef _8821A_BTON_DESIGN_        
        //USB multi or gps disable: fw need to set gps suspend enable
        if ((g_fun_interface_info.b.hci_sel == RTK8723_U) ||
            (!g_fun_interface_info.b.gps_fun_sts))
        {
            state |= BIT7;
                pow_ctrl.b.gps_sus_en = 1;
            //RT_BT_LOG(RED,GPS_STATUS_DISABLE, 0, 0);
        }        
#else
        if (efuse_pow_setting_1.bt_sus_power_on_check_execute)
        {
            pw_execute = 1;
        }
#endif

    }

    //Check PCIE suspend; low active
#ifdef _8821A_BTON_DESIGN_

#ifndef _REMOVE_HCI_PCIE_
    if (gpio_pin.b.pcie_suspend && (efuse_pow_setting_1.bt_suspend_power_on_check_dis == 0))
#else    
    if (efuse_pow_setting_1.bt_suspend_power_on_check_dis == 0)
#endif        
#else    
    if (gpio_pin.b.pcie_suspend)
#endif    
    {
        state |= BIT8;

#ifndef _8821A_BTON_DESIGN_
        //RT_BT_LOG(RED, PCIE_SUSPEND_ENABLE, 0, 0);
        //set suspend bit
        pow_ctrl.b.gps_sus_en = 1;
#endif        
        //PCIE multi or gps disable: fw need to set gps suspend enable
#ifndef _8821A_BTON_DESIGN_
        if ((g_fun_interface_info.b.hci_sel == RTK8723_EE) ||
            (!g_fun_interface_info.b.bt_fun_sts))
#else
#ifndef _REMOVE_HCI_PCIE_

        if ((g_fun_interface_info.b.hci_sel == RTK8723_EE))
#endif            
#endif
#ifndef _REMOVE_HCI_PCIE_
        {
            state |= BIT9;
            pow_ctrl.b.bt_sus_en = 1;
            //RT_BT_LOG(RED,BT_STATUS_DISABLE, 0, 0);

#ifdef _8821A_BTON_DESIGN_         
            if (efuse_pow_setting_1.bt_sus_power_on_check_execute)
            {
                pw_execute = 1;
            }
#endif
        }
#endif        
    }

#ifdef _CCH_RETENTION_FLOW_FOR_DLPS_
    if(sleep_mode_param.dlps_restore_flow == FALSE)
#endif
    {
#ifdef _8821A_BTON_DESIGN_
        RT_BT_LOG(BLUE,MSG_CHK_GPIO_PWRON3,5 , state, gpio_pin.d32, pow_ctrl.d32, bton_inter_info.d32, bton_uart_info.d32);
#else
        RT_BT_LOG(BLUE,MSG_CHK_GPIO_PWRON,3, state, gpio_pin.d32, pow_ctrl.d32);
#endif  
    }

    if (pw_execute)
    {

#if 0
#ifdef _8821A_BTON_DESIGN_        
        EFUSE_POW_SETTING_2_S efuse_pow_setting_2;
        *(UINT8*)&efuse_pow_setting_2 = otp_str_data.efuse_pow_setting_2_d8;
        if ((pow_ctrl.b.bt_hwpdn_en))
        {
            if (efuse_pow_setting_2.avoid_pdn_with_sus)
            {
                pow_ctrl.b.bt_sus_en = 0;
            }                            
            execute_bton_entering_pdn_sus(BTON_ENTERING_PDN);
        }
        else if ((pow_ctrl.b.bt_sus_en))
        {
            execute_bton_entering_pdn_sus(BTON_ENTERING_SUS);
        }               
#endif    
        // NOTE: It will also W1C LPS Wakeup Interrupt Status at Vendor0x40
        VENDOR_WRITE(BTON_POW_CTRL_REG, pow_ctrl.d32); 
#else
#ifdef _8821A_BTON_DESIGN_        
        execute_bton_entering_pdn_sus(&pow_ctrl);
#endif
#endif

    }

#ifdef _YL_RTL8723A_B_CUT
    //2 Note: it would overlap the setting in GpioInit()
#ifdef _8821A_BTON_DESIGN_
    EFUSE_POW_SETTING_2_S efuse_pow_setting_2;
    *(UINT8*)&efuse_pow_setting_2 = otp_str_data.efuse_pow_setting_2_d8;
    if (efuse_pow_setting_2.power_on_check_init_power_var)
#else
    if (g_efuse_lps_setting_2.power_on_check_init_power_var) // TODO: To be modified
#endif    
    {
        if (pow_ctrl.b.bt_sus_en)
        {
            g_host_state = 1;
            g_host_wake_bt = 0;
            g_wake_first_time = 0;
        }        
#ifdef _8821A_BTON_DESIGN_
        if (efuse_pow_setting_2.gpio_host_wake_bt_en)
#else
        if (g_efuse_lps_setting_2.gpio_host_wake_bt_en)
#endif
        {
            g_host_wake_bt = get_gpio_host_wake_bt();
        }
        /* No effect When bt_general_ctrl.b.gpio_wake_up_type == 0 */
        /* bt_general_ctrl.b.gpio_wake_up_type serve as Enable and Pin Select */        
        /* the polarity of gpio_bt_wake_host is controlled by bt_general_ctrl.b.gpio_wake_up_polarity*/
#ifdef _8821A_BTON_DESIGN_    
        if (efuse_pow_setting_2.bt_wake_host_init_opt)
        {
            set_gpio_bt_wake_host(g_host_state);
        }
        else
        {   
            set_gpio_bt_wake_host(!g_host_state);
        }
#else
        set_gpio_bt_wake_host(g_host_state);
#endif
    }
#endif    

#ifdef _ROM_CODE_PATCHED_
#ifdef _YL_RTL8723A_B_CUT
    /* ylilinli
      * 1. if (bt dis and gps dis)    hwpwn_en? sus_en?
      * 2. g_host_wake_bt initialization
      * 3. set GPS_DIS_N as host_wake_bt related variables 
      * 4. change edge-trigger to level-triggered
      * 5. need also modify process_power_gpio_set() as level-triggered
      * 6. need also modify GpioIntrHandler: change active-level then clear interrupt status to avoid extra triggering
      * 7. initialize 
      */
    if (rcp_gpio_power_on_check_end_func != NULL)
    {
        rcp_gpio_power_on_check_end_func((void*)(&gpio_pin) );
    }
    
#endif
#endif    

}

//2 Note: When the conrtrol received the packet and  it need to wake up host
//2    fw need to call this function.
BOOLEAN gpio_wake_up_host(void)
{
    GENERAL_CONTROL_S_TYPE bt_general_ctrl;   

#ifdef _ROM_CODE_PATCHED_
    if (rcp_gpio_wake_up_host != NULL)
    {
        UINT8 result = 0;
        rcp_gpio_wake_up_host((void *)&result);
        return result;
    }
#endif

    //the setting value loaded from efuse
    bt_general_ctrl.d16 = otp_str_data.general_control;

    if (g_host_state && (bt_general_ctrl.b.gpio_wake_up_fun_en))
    {
#ifdef _8821A_BTON_DESIGN_    
        set_gpio_bt_wake_host(1);
#else
        UINT32 output_value = 0;
        get_wake_up_pin(NULL,NULL,&output_value);
        GPIO_WRITE(GPIO_PORTA_DATA_REG, output_value);
#endif

        //Set timer callback function
        //Need to wake up host
        gpio_wake_up_trigger_one_shot_event((UINT32)otp_str_data.gpio_wake_up_time);
        
        return FALSE;
    }
    else
    {
        return TRUE;
    }
}


#ifndef _8821A_BTON_DESIGN_
/* output_en: the GPIO OE (direction) to be to GPIO_PORTA_DATA_DREC_REG
 * init_value: the inactive value to be programmed to GPIO_PORTA_DATA_REG
 * execute_value: the active value to be programmed to GPIO_PORTA_DATA_REG */
void get_wake_up_pin(UINT32 *output_en,UINT32 *init_value, UINT32 *execute_value)
{
    GENERAL_CONTROL_S_TYPE bt_general_ctrl;
    UINT32 gpio_out_en;
    UINT8 index = 0;

#ifdef _ROM_CODE_PATCHED_
    if (rcp_get_wake_up_pin != NULL)
    {
        rcp_get_wake_up_pin((void *)output_en, init_value, execute_value);
        return;
    }
#endif

    gpio_out_en = GPIO_READ(GPIO_PORTA_DATA_DREC_REG);

    //the setting value loaded from efuse
    bt_general_ctrl.d16 = otp_str_data.general_control;

    switch (bt_general_ctrl.b.gpio_wake_up_type)
    {
        case 1://enable GPS GPIO for wake up 
            gpio_out_en |= BIT1;
            index = 1;
            break;
        case 2://enable LED0 for wake up 
            gpio_out_en |= BIT4;
            index = 4;
            break;
        case 3://enable LED1 for wake up
            gpio_out_en |= BIT5;
            index = 5;
            break;
        default:
            break;
    }

    *output_en = gpio_out_en;
    
#ifdef _8821A_BTON_DESIGN_
    if (bt_general_ctrl.b.gpio_wake_up_value_from_reg0)
    {
        gpio_out_en = GPIO_READ(GPIO_PORTA_DATA_REG);
        gpio_out_en |= (0x1 << index);
    }
#endif
    if (bt_general_ctrl.b.gpio_wake_up_polarity)
    {
        // polarity = 1: init_value = 0, execute_value = 1
        *init_value = gpio_out_en & (~(0x1 << index));
        *execute_value = gpio_out_en;
    }
    else
    {
        // polarity = 0: init_value = 1, execute_value = 0
        *init_value = gpio_out_en;
        *execute_value = gpio_out_en & (~(0x1 << index));        
    }
}
#endif

#ifdef _SUPPORT_POLLING_BASED_LPM_L1_
UINT8 usb_read_L1_remote_wakeup_feature(void)
{
    UINT8 u8sieValue;


    u8sieValue = (UINT8)safe_indirect_read_sie(READ_SIE_BYTE, SIE_REG_0x10);

    if(u8sieValue&BIT6)
    {
        return 1;
    }

    return 0;
    
}

UINT8 is_usb_lpm_l1_state(void)
{
    // return 1: L1, 0: dont care
    BTON_USB_LPM_REG_S_TYPE bton_0x20;
    bton_0x20.d32 = VENDOR_READ(BTON_USB_LPM);    
    return bton_0x20.b.Suspend_Mode; 
}

UINT8 is_usb_suspend_state(void)
{
    // return 1: L2, 0: dont care
    BTON_USB_LPM_REG_S_TYPE bton_0x20;
    bton_0x20.d32 = VENDOR_READ(BTON_USB_LPM);    
    return bton_0x20.b.USB_Suspend_Ind; 
}


#endif


void gpio_wake_up_trigger_one_shot_event(UINT32 time_ms)
{
    if (gpio_wake_up_tid_timer != NULL)
    {
        OS_DELETE_TIMER(&gpio_wake_up_tid_timer);
    }

    /* Create a tid timer */
    OS_CREATE_TIMER(ONESHOT_TIMER, &gpio_wake_up_tid_timer,
            gpio_wake_up_timer_callback, NULL, 0);

    OS_START_TIMER(gpio_wake_up_tid_timer, time_ms);
}

void gpio_wake_up_timer_callback(TimerHandle_t timer_handle)
{
    DEF_CRITICAL_SECTION_STORAGE;

#ifdef _ROM_CODE_PATCHED_
    if (rcp_gpio_wake_up_timer_callback_func != NULL)
    {
        if (rcp_gpio_wake_up_timer_callback_func(timer_handle))
        {
            return;
        }
    }
#endif


#ifdef _8821A_BTON_DESIGN_
    set_gpio_bt_wake_host(0);
#else
    UINT32 initial_value = 0;
    get_wake_up_pin(NULL,&initial_value,NULL);
    //Set the gpio to default value
    GPIO_WRITE(GPIO_PORTA_DATA_REG, initial_value);
#endif    
    MINT_OS_ENTER_CRITICAL();
    g_host_state = 0;
    MINT_OS_EXIT_CRITICAL();
}

#ifdef _YL_RTL8723A_B_CUT
#ifdef _8821A_BTON_DESIGN_
UINT8 usb_read_d2_remote_wakeup_feature(void)
{
/*
    ??? to be modified
    GPIO_PIN_S_TYPE gpio_pin;
    UINT8 remote_wakeup_feature;
    gpio_pin.d32 = GPIO_READ(GPIO_EXT_PORTA_REG);
    remote_wakeup_feature = gpio_pin.b.usb_remote_wakeup_en;
    return remote_wakeup_feature;
*/
    // Modified after RTL8821A-MP //
    BTON_USB_LPM_REG_S_TYPE bton_0x20;
    bton_0x20.d32 = VENDOR_READ(BTON_USB_LPM);    
    return bton_0x20.b.L2RMT_WkUp;
}



/* return value:
 *   0: USB interface is ACTIVE
 *   1: USB interface is SUSPEND
 */
UINT8 usb_read_suspend_status(void)
{
    GPIO_PIN_S_TYPE gpio_pin;
    gpio_pin.d32 = GPIO_READ(GPIO_EXT_PORTA_REG);
//	RT_BT_LOG(GRAY, CCH_DBG_106, 2,gpio_pin.d32, gpio_pin.b.usb_suspend);
    return gpio_pin.b.usb_suspend;
}

#endif
/* return:
 *       1: ON, 
 *       0: OFF        */
UINT8 get_gpio_host_wake_bt(void)
{
    GPIO_PIN_S_TYPE gpio_pin;
    UINT8 gpio_host_wake_bt;

#ifdef _ROM_CODE_PATCHED_
    if (rcp_get_gpio_bt_wake_host != NULL)
    {
        return rcp_get_gpio_bt_wake_host(NULL);
    }
#endif    
    
    gpio_pin.d32 = GPIO_READ(GPIO_EXT_PORTA_REG);
#ifndef _8821A_BTON_DESIGN_  
    gpio_host_wake_bt = gpio_pin.b.gps_pdn;
#else
    gpio_host_wake_bt = gpio_pin.b.gpio13_host_wake_bt; //workaround by austin: do not know the actual design
#endif
#ifdef _8821A_BTON_DESIGN_
    EFUSE_POW_SETTING_2_S efuse_pow_setting_2;
    *(UINT8*)&efuse_pow_setting_2 = otp_str_data.efuse_pow_setting_2_d8;
    if (efuse_pow_setting_2.gpio_host_wake_bt_polarity)
#else
    if (g_efuse_lps_setting_2.gpio_host_wake_bt_polarity)
#endif    
    {
        // Active High
        return gpio_host_wake_bt;
    }
    else
    {
        // Active Low
        return (!gpio_host_wake_bt);    
    }
}

/* parameter value:
 *       1: ON, 
 *       0: OFF, and Initialization
 */
void set_gpio_bt_wake_host(UINT8 value)
{

#ifdef _8821A_BTON_DESIGN_    

    GENERAL_CONTROL_S_TYPE bt_general_ctrl;

#ifdef _ROM_CODE_PATCHED_
    if (rcp_set_gpio_bt_wake_host != NULL)
    {
        if (rcp_set_gpio_bt_wake_host((void *)&value))
        {
            return;
        }
    }
#endif

    //the setting value loaded from efuse
    bt_general_ctrl.d16 = otp_str_data.general_control;

    if (bt_general_ctrl.b.gpio_wake_up_type == 1)
    {   /* BT WAKEUP HOST by GPIO14 */
        /* Active High, Polarity is determined by SYSTEM TOP */
        BTON_POW_CTRL_REG_S_TYPE pow_ctrl;
        pow_ctrl.d32 = VENDOR_READ(BTON_POW_CTRL_REG);
        pow_ctrl.b.resume_to_sie_en = value;
        bton_pow_ctrl_avoid_int_sts_w1c(&pow_ctrl);
        VENDOR_WRITE(BTON_POW_CTRL_REG, pow_ctrl.d32);
    }
    else
    {
        UINT8 level = (bt_general_ctrl.b.gpio_wake_up_polarity==value) ? 1 : 0;
        UINT8 oe = (!bt_general_ctrl.b.gpio_wake_up_opendrain) || value;    
    
//        GPIO_POUT_S_TYPE gpio_out;
//        GPIO_POUT_S_TYPE gpio_out_en;
//        gpio_out.d32 = GPIO_READ(GPIO_PORTA_DATA_REG);  
//        gpio_out_en.d32 = GPIO_READ(GPIO_PORTA_DATA_DREC_REG);
        
        switch (bt_general_ctrl.b.gpio_wake_up_type)
        {
            case 2://enable LED0 for wake up 
                /* bt_general_ctrl.b.gpio_wake_up_polarity = 1: active high
                 * bt_general_ctrl.b.gpio_wake_up_polarity = 0: active low
                 */
                if (oe)
                {
                    bt_led_control(0, level);
                }
                else
                {
                    bt_led_control(0, OUTPUT_HI_Z);
                }
//                gpio_out.b.led0 = ((bt_general_ctrl.b.gpio_wake_up_polarity==value) ? 1 : 0);
//                gpio_out_en.b.led0 = 1;
                break;
            case 3://enable LED1 for wake up
                if (oe)
                {
                    bt_led_control(1, level);
                }
                else
                {
                    bt_led_control(1, OUTPUT_HI_Z);
                }
//                gpio_out.b.led1 = ((bt_general_ctrl.b.gpio_wake_up_polarity==value) ? 1 : 0);
//                gpio_out_en.b.led1 = 1;
                break;
            default:
                return;
        }
//        GPIO_WRITE(GPIO_PORTA_DATA_REG, gpio_out.d32);
//        GPIO_WRITE(GPIO_PORTA_DATA_DREC_REG, gpio_out_en.d32);
    }

#else
    
    UINT8 index;
    GENERAL_CONTROL_S_TYPE bt_general_ctrl;
    bt_general_ctrl.d16 = otp_str_data.general_control;
    
    if (bt_general_ctrl.b.gpio_wake_up_type) 
    {
        switch (bt_general_ctrl.b.gpio_wake_up_type)
        {
            case 1://enable GPS GPIO for wake up 
                index = 1;
                break;
            case 2://enable LED0 for wake up 
                index = 4;
                break;
            case 3://enable LED1 for wake up
                index = 5;
                break;
            default:
                return;
        }
        if(bt_general_ctrl.b.gpio_wake_up_polarity)
        {
            // Active High
            if(value)
                gpio_output_high(index);
            else
                gpio_output_low(index);
        }
        else
        {
            // Active Low
            if(value)
                gpio_output_low(index);
            else
                gpio_output_high(index);            
        }
    }
#endif
}
#endif
#ifndef _IS_ASIC_ // TODO: to be implemented in PATCH or by HCI_VENDOR_CMD
void dbg_gpio_init(void)
{
    UINT32 gpio_out_en;
    UINT32 gpio_out_value;

    gpio_out_en = GPIO_READ(GPIO_PORTA_DATA_DREC_REG);
    gpio_out_en |= BIT5;    
    GPIO_WRITE(GPIO_PORTA_DATA_DREC_REG, gpio_out_en);

    gpio_out_value = GPIO_READ(GPIO_PORTA_DATA_REG);
    gpio_out_value &= (~BIT5);
    GPIO_WRITE(GPIO_PORTA_DATA_REG, gpio_out_value);

}
#endif
void gpio_output_control(UINT8 index, UINT8 mode)
{
    UINT32 output_value;

    GENERAL_CONTROL_S_TYPE bt_general_ctrl;

    //the setting value loaded from efuse
    bt_general_ctrl.d16 = otp_str_data.general_control;
        
    if ((!bt_general_ctrl.b.gpio_led_fun_en) || 
            (!bt_general_ctrl.b.gpio_wake_up_fun_en))
    {
        if ((index < 8) && (index >3))
        {
            DEF_CRITICAL_SECTION_STORAGE;
            MINT_OS_ENTER_CRITICAL();
            output_value = GPIO_READ(0x00);            
            if (mode == GPIO_OUTPUT_HIGH)
            {
                if (!(output_value & (1 << index)))
                {
                    /* Low to Hight */
                    GPIO_WRITE(GPIO_PORTA_DATA_REG, output_value | (1 << index));
                }
            }
            else if (mode == GPIO_OUTPUT_LOW)
            {
                if (output_value & (1 << index))
                {
                    /* Hight to Low */
                    GPIO_WRITE(GPIO_PORTA_DATA_REG, output_value & ~(1 << index));
                }
            }
            else
            {
                GPIO_WRITE(GPIO_PORTA_DATA_REG, output_value ^ (1 << index));
                GPIO_WRITE(GPIO_PORTA_DATA_REG, output_value);                
            }
            MINT_OS_EXIT_CRITICAL();
        }
        else
        {
            RT_BT_LOG(RED, GPIO_INDEX_CAN_NOT_USE, 1, index);
        }
    }
}

#ifndef _IS_ASIC_ // TODO: to be implemented in PATCH or by HCI_VENDOR_CMD
void gpio_init_for_la()
{
#ifdef _YL_LPS_COMBO_PLATFORM
    GPIO_WRITE(GPIO_PORTA_DATA_DREC_REG, GPIO_READ(GPIO_INT_MASK_REG) & 0xFF);
#else
    GPIO_WRITE(GPIO_PORTA_DATA_DREC_REG,0xff);
#endif    
    GPIO_WRITE(GPIO_PORTA_DATA_REG,0x0);
}
#endif

void gpio_one_pull_low(UINT8 index)
{
    UINT8 bm_gpio_dir;
    UINT8 bm_gpio_out;
    UINT8 bm_index;

    if (index < 8)
    {
        bm_index = 1 << index;
        
        DEF_CRITICAL_SECTION_STORAGE;
        MINT_OS_ENTER_CRITICAL();
        bm_gpio_dir = GPIO_READ(GPIO_PORTA_DATA_DREC_REG);
        if (!(bm_gpio_dir & bm_index))
        {
            /* input -> output */
            GPIO_WRITE(GPIO_PORTA_DATA_DREC_REG, bm_gpio_dir | bm_index);
        }
        bm_gpio_out = GPIO_READ(GPIO_PORTA_DATA_REG);
        if (bm_gpio_out & bm_index)
        {
            GPIO_WRITE(GPIO_PORTA_DATA_REG, bm_gpio_out & ~bm_index);
        }
        MINT_OS_EXIT_CRITICAL();
    }
}
void gpio_one_pull_high(UINT8 index)
{
    UINT8 bm_gpio_dir;
    UINT8 bm_gpio_out;    
    UINT8 bm_index;

    if (index < 8)
    {
        bm_index = 1 << index;

        DEF_CRITICAL_SECTION_STORAGE;
        MINT_OS_ENTER_CRITICAL();
        bm_gpio_dir = GPIO_READ(GPIO_PORTA_DATA_DREC_REG);
        if (!(bm_gpio_dir & bm_index))
        {
            /* input -> output */
            GPIO_WRITE(GPIO_PORTA_DATA_DREC_REG, bm_gpio_dir | bm_index);
        }
        bm_gpio_out = GPIO_READ(GPIO_PORTA_DATA_REG);
        if (!(bm_gpio_out & bm_index))
        {
            GPIO_WRITE(GPIO_PORTA_DATA_REG, bm_gpio_out | bm_index);
        }
        MINT_OS_EXIT_CRITICAL();
    }    
}

void gpio_one_pull_pulse(UINT8 index)
{
    UINT32 bm_gpio_dir;
    UINT32 bm_gpio_out;
    UINT32 bm_index;

    if (index < 8)
    {
        bm_index = 1 << index;

        DEF_CRITICAL_SECTION_STORAGE;
        MINT_OS_ENTER_CRITICAL();
        bm_gpio_dir = GPIO_READ(GPIO_PORTA_DATA_DREC_REG);
        if (!(bm_gpio_dir & bm_index))
        {
            /* input -> output */
            GPIO_WRITE(GPIO_PORTA_DATA_DREC_REG, bm_gpio_dir | bm_index);
        }
        
        bm_gpio_out = GPIO_READ(GPIO_PORTA_DATA_REG);
        GPIO_WRITE(GPIO_PORTA_DATA_REG, bm_gpio_out ^ bm_index);        
        GPIO_WRITE(GPIO_PORTA_DATA_REG, bm_gpio_out);
        MINT_OS_EXIT_CRITICAL();
    }
}



#endif /*end of defined(_GPIO_POWER_SEQUENCE_ENABLE) */

/**
 * @brief Set BT_GPIO[20:0] to bton domain.
 *
 * This function sets BT_GPIO[x] to bton domain if bit[x] of \p mask is 1.
 * @param mask  Bitmap of pins to set. bit[x] corresponds to BT_GPIO[x].
 */
void gpio_ctrl_set_bton(UINT32 mask)
{
    BTON_GPIO_CTRL0_REG reg = bton_gpio_ctrl0_reg_read();
    CLR_BIT(reg.d32, mask);
    bton_gpio_ctrl0_reg_write(reg);
}

/**
 * @brief Set pin in BT_GPIO[20:0] to off domain
 *
 * This functions set BT_GPIO[x] to off domain if bit[x] of \p mask is 1.
 * @param mask  Bitmap of pins to set. bit[x] corresponds to BT_GPIO[x].
 */
void gpio_ctrl_set_off_domain(UINT32 mask)
{
    BTON_GPIO_CTRL0_REG reg = bton_gpio_ctrl0_reg_read();
    SET_BIT(reg.d32, mask);
    bton_gpio_ctrl0_reg_write(reg);
}

#ifndef _SUPPORT_SEPARATE_BTGPIO_RW_REG_
/**
 * @brief Output value of BT_GPIO[20:0].
 *
 * Bit[30:10] records the output of BT_GPIO[20:0] to avoid overwriting the
 * original output of BT_GPIO[20:0].
 */
UINT32 gpio_ctrl_out_val;
#endif

/**
 * @brief Set input/output for BT_GPIO[20:0].
 *
 * This function sets the I/O flag of BT_GPIO[x] to bit[x] of \p io_flag
 * if bit[x] of \p mask is 1.
 * @param mask  Bitmap of pins to set.
 * @param io_flag  Input/output flags to set (0: Input; 1: output).
 * ex,set gpio2 as output, u should mask = 0x04
 */
void gpio_ctrl_set_in_out(UINT32 mask, UINT32 io_flag)
{
    UINT32 mask_10_0 = ((mask & 0x7ff) << 21);
    UINT32 io_flag_10_0 = ((io_flag & 0x7ff) << 21);
    UINT32 mask_20_11 = (((mask >> 11) & 0x3ff) | 0xfffffc00);
#ifndef _SUPPORT_SEPARATE_BTGPIO_RW_REG_    
    UINT32 io_flag_20_11 = (((io_flag >> 11) & 0x3ff) | gpio_ctrl_out_val);
#else
    UINT32 gpio_ctrl_out_val = ((bton_gpio_ctrl1_reg_read()).d32)&0xfffffc00;// to get original out value
    UINT32 io_flag_20_11 = (((io_flag >> 11) & 0x3ff) | gpio_ctrl_out_val);
#endif
    BTON_GPIO_CTRL0_REG reg0 = bton_gpio_ctrl0_reg_read();
    BTON_GPIO_CTRL1_REG reg1 = bton_gpio_ctrl1_reg_read();
    reg0.d32 = MERGE_BIT(reg0.d32, io_flag_10_0, mask_10_0);
    reg1.d32 = MERGE_BIT(reg1.d32, io_flag_20_11, mask_20_11);
    bton_gpio_ctrl0_reg_write(reg0);
    bton_gpio_ctrl1_reg_write(reg1);
}

UINT8 gpio_ctrl_read_gpio(UINT8 pos)
{
    // get "one" gpio input "value", not bitmap
#ifndef _SUPPORT_SEPARATE_BTGPIO_RW_REG_
    // 8703B & 8822B
    BTON_GPIO_CTRL1_REG reg = bton_gpio_ctrl1_reg_read();
    return EXTRACT_BIT(reg.d32 >> 10, pos);
#else
#ifdef _RTL8723D_SPECIFIC_  
    // 8723D
    BTON_GPIO_CTRL2_REG reg = bton_gpio_ctrl2_reg_read();
    return EXTRACT_BIT(reg.d32, pos);
#endif
#ifdef _RTL8821C_SPECIFIC_  
    // 8821C
    BTON_GPIO_CTRL2_REG reg = bton_gpio_ctrl2_reg_read();
    if(pos>=18)
    {
        return EXTRACT_BIT(reg.d32, pos+1);
    }
    else
    {
        return EXTRACT_BIT(reg.d32, pos);
    }
        
#endif

#endif
}

void gpio_ctrl_write_gpio(UINT8 pos, UINT8 val)
{
#ifndef _SUPPORT_SEPARATE_BTGPIO_RW_REG_
    BTON_GPIO_CTRL1_REG reg = bton_gpio_ctrl1_reg_read();
    UINT32 mask = (GEN_BIT_MASK(pos) << 10);
    UINT32 new_val = (((val & 0x1) << (pos)) << 10);
    gpio_ctrl_out_val = MERGE_BIT(gpio_ctrl_out_val, new_val, mask);
    reg.d32 = MERGE_BIT(reg.d32, gpio_ctrl_out_val, 0xfffffc00);
    bton_gpio_ctrl1_reg_write(reg);
#else
    BTON_GPIO_CTRL1_REG reg = bton_gpio_ctrl1_reg_read();
    UINT32 mask = (GEN_BIT_MASK(pos) << 10);
    UINT32 new_val = (((val & 0x1) << (pos)) << 10);
    reg.d32 = MERGE_BIT(reg.d32, new_val, mask);
    bton_gpio_ctrl1_reg_write(reg);
#endif /* End of #ifndef _SUPPORT_SEPARATE_BTGPIO_RW_REG  */
}



#ifdef _SUPPORT_TIMER1_SHUTDOWN_LOG_
void timer1_shutdown_log(void)
{
    if(IS_AUTO_SHUTDOWN_BT_LOG)
    {
        if(g_u16shutdown_log_timer_cnt == 1024)
         {
            if(IS_LOG_ENABLED_WHEN_POWER_ON)
            {
                SWITCH_BTON_GPIO_TO_ON_DOMAIN(BTON_GPIOMAP_08);
                //ENABLE_OFF_DOMAIN_LOG_OUTPUT();
                off_domain_log_control(0);
            }
        
            g_u16shutdown_log_timer_cnt = 0;
         }
        g_u16shutdown_log_timer_cnt++;
    }

}
#endif

#ifdef _SUPPORT_POWERON_ENABLE_LOG_MUX_
#ifndef _SUPPORT_BTGPIO10_AS_LOG1_
void off_domain_log_control(UINT8 enable)
{
    UINT32 dword;
    // control log0, before 8723d
    dword = VENDOR_READ(REG_VEN_GPIO_CTRL);
    if(enable != 0)
    {
        dword |= BIT29;
    }
    else
    {
        dword &= (~BIT29);
    }
    VENDOR_WRITE(REG_VEN_GPIO_CTRL, dword);

}

#else
void off_domain_log_control(UINT8 control)
{
    UINT32 dword;
    // control log0, before 8723d
    // BIT29 = gpio8 log(loguart_sel_0), BIT28 = gpio10 log(loguart_sel_1)
    dword = VENDOR_READ(REG_VEN_GPIO_CTRL);

    switch(control)
    {
        case LOG_OFF:
           dword &= (~(BIT29|BIT28));
        break;
        
        case OUTPUT_LOG0:// original log
            dword |= BIT29; // enable log0
            dword &= (~(BIT28));// disable log1
        break;
        
        case OUTPUT_LOG1:
            dword |= BIT28; // enable log0
            dword &= (~(BIT29));// disable log1
        break;
        
        default:
           dword &= (~(BIT29|BIT28));            
        break;

    }
    VENDOR_WRITE(REG_VEN_GPIO_CTRL, dword);

}

#endif /* _SUPPORT_BTGPIO10_AS_LOG1_ */
#endif /* _SUPPORT_POWERON_ENABLE_LOG_MUX_ */


#ifdef _ENABLE_VENDOR_GPIO_INTERRUPT_
void vendor_gpio_interrupt_handler()
{


    BTOFF_VEN_GPIO_REG_INT ven_gpio_int;
    ven_gpio_int.d32 = VENDOR_READ(VND_GPIO_INT);

     if( (ven_gpio_int.d32&0x000000FF) == 0 )
        return;
     
#ifdef _FONGPIN_TEST_VENDOR_GPIO_    
    RT_BT_LOG(BLUE, VENDOR_GPIO_INTERRUPT, 2, VENDOR_READ(VND_GPIO_CTRL), ven_gpio_int.d32);
#endif

#ifdef _ROM_CODE_PATCHED_
        if (rcp_vendor_gpio_intrhandler_func != NULL)
        {
            if(rcp_vendor_gpio_intrhandler_func((void*)&ven_gpio_int.d32))
            {
                return;
            }
        }    
#endif
    

    //clear interrupt
    ven_gpio_int.d32 |= ven_gpio_int.d32;
    VENDOR_WRITE(VND_GPIO_INT, ven_gpio_int.d32);

}
#endif


