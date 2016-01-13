/***************************************************************************
 Copyright (C) RealTek Ltd.
 This module is a confidential and proprietary property of RealTek and
 a possession or use of this module requires written permission of RealTek.
 ***************************************************************************/

#ifndef _POWER_CONTROL_H_
#define _POWER_CONTROL_H_

#include "DataType.h"
#include "platform.h"
#include "bt_fw_config.h"
#include "logger.h"
#include "bb_driver.h"
#include "dma_usb.h"


#define CPU_FILTER_MASK                    (0x3F << 10)

#define BT_SYS_CTRL_REG                    0x00  //struct: BT_SYS_CTRL_REG_S


#ifdef _2801_BTON_DESIGN_
#define BTON_32K_DET_REG                   0x08  //struct: 
#endif

/*	Added by Wallice Su for USB LPM.	2012/02/29	*/
#define BTON_USB_LPM                       0x20 //struct: BTON_USB_LPM_REG_S
/*	End Added by Wallice Su for USB LPM.	2012/02/29	*/

#define BTON_POW_CTRL_REG                  0x40  //struct: BTON_POW_CTRL_REG_S
#define BTON_INTERFACE_CTRL_REG            0x44  //struct: BTON_INTERFACE_CTRL_REG_S
// TODO: WARNING: 32K calibration should be added (Vendor Register 0x48 0x4C related) !!!! 
#ifndef _CCH_REMOVE_THERMAL_METER_
#define BTON_THERMAL_METER_CTRL_REG        0x48  // struct:THERMAL_METER_CTRL_S
#endif
#define BTON_TIMER_CTRL_REG                0x4C // struct: TIME_CTRL_REG_S
#define BTON_POW_OPTION_AND_32K_CTRL       0x50  //struct: POW_OPTION_AND_32K_CTRL_S


#ifdef _8821A_BTON_DESIGN_
#ifdef _NEW_BTON_DESIGN_AFTER_RTL8821B_TC_
#define BTON_54_REG                        0x54  // struct: BTON_54_REG_S
#define BTON_58_REG                        0x58  // struct: BTON_58_REG_S
#define BTON_A0_REG                        0xA0  // struct: BTON_A0_REG_S
#define BTON_A4_REG                        0xA4  // struct: BTON_A4_REG_S
#else
#define BTON_32K_TEST_REG                  0x54  // struct: BTON_32K_TEST_REG_S
#define BTON_32K_CSR0_REG                  0x58  // struct: BTON_32K_CSR0_REG_S
#define BTON_32K_NEW_CTRL1_REG             0xA0  // struct: BTON_32K_NEW_CTRL1_REG_S
#define BTON_32K_NEW_CTRL2_REG             0xA4  // struct: BTON_32K_NEW_CTRL2_REG_S
// TODO: (yl) modify FW to fit new 8821A 32K design
#endif
#endif


#ifdef _NEW_BTON_DESIGN_AFTER_RTL8821B_TC_
#define BTON_5C_REG                        0x5C  // struct: BTON_5C_REG_S
#define BTON_70_REG                        0x70  // struct: BTON_70_REG_S
#define BTON_74_REG                        0x74  // struct: BTON_74_REG_S
#define BTON_A8_REG                        0xA8  // struct: BTON_A8_REG_S
#define BTON_AC_REG                        0xAC  // struct: BTON_AC_REG_S
#else
#define BTON_32K_CTRL_REG                  0x5C  // struct: BTON_32K_CTRL_REG_S
#define BTON_CORE_AFE_LDO_CTRL_REG         0x74 // struct: CORE_AND_AFE_LDO_CTRL_S
#ifndef _8821A_BTON_DESIGN_
#define BTON_UART_SPI_AFE_CTRL_REG         0x70 // struct: BTON_UART_SPI_AFE_CTRL_REG_S
#else
#define BTON_AFE_PLL_CTRL_0_1_REG          0x70 // struct: BTON_AFE_PLL_CTRL_0_1_REG_S
#define BTON_AFE_PLL_CTRL_3_4_REG          0xA8 // struct: BTON_AFE_PLL_CTRL_3_4_REG_S
#endif

#ifdef _8821A_BTON_DESIGN_
#define BTON_BT_RFE_PAD_CTRL_REG           0xAC // struct: BTON_BT_RFE_PAD_CTRL_REG_S
#endif
#endif

#ifdef _NEW_BTON_DESIGN_AFTER_RTL8703B_TC_
#define BTON_BT_RETENTION_REG              0x35C  // struct: BTON_BT_RETENTION_REG_S

#define PON_F00_REG                        0xF00  // struct: PON_F00_REG_S
#endif


#define BTON_BT_LECMD_PUSH_REG             0x460  // struct: BTON_BT_LECMD_PUSH_REG_S


#ifdef _NEW_BTON_DESIGN_AFTER_RTL8821B_TC_
// 32k indirect address map
#define BTON_INDIR_32K_00_REG              0x00
#define BTON_INDIR_32K_01_REG              0x01
#define BTON_INDIR_32K_02_REG              0x02
#define BTON_INDIR_32K_03_REG              0x03
#define BTON_INDIR_32K_04_REG              0x04
#define BTON_INDIR_32K_05_REG              0x05
#define BTON_INDIR_32K_06_REG              0x06
#define BTON_INDIR_32K_07_REG              0x07
#define BTON_INDIR_32K_08_REG              0x08
#define BTON_INDIR_32K_09_REG              0x09
#define BTON_INDIR_32K_0A_REG              0x0A
#define BTON_INDIR_32K_0B_REG              0x0B
#define BTON_INDIR_32K_0C_REG              0x0C
#define BTON_INDIR_32K_0D_REG              0x0D
#define BTON_INDIR_32K_0E_REG              0x0E
#define BTON_INDIR_32K_0F_REG              0x0F
#define BTON_INDIR_32K_10_REG              0x10
#define BTON_INDIR_32K_11_REG              0x11
#endif

#ifdef _UART_H5
#define BTON_UART_INFO_REG                 0x7C // struct: BTON_UART_INFO_REG_S
#endif

#ifndef _SUPPORT_3BITS_HCI_SELECTION_ /* obly for 8822b v6*/
#ifndef _SUPPORT_2BIT_HCI_AND_PCIE_UART_ /* enable it when 8723D, 8821C v6 */
#define RTK8723_S                                       0x00 //SDIO_UART mode no GPS
#define RTK8723_U                                       0x01 //USB multiple  mode
#define RTK8723_EE                                      0x02 // PCIE multiple mode
#define RTK8822B_ME                                     0x02 // mPCIE-USB mode, from 8822b
#define RTK8723_E                                       0x03 // PCIE-USB mode


#define COMBO_INF_SDIO_UART                             0x00
#define COMBO_INF_USB_MULTI                             0x01
#define COMBO_INF_PCIE_MULTI                            0x02
#define COMBO_INF_PCIE_USB                              0x03

enum package_hci{/* PACKAGE_WIFI_INTERFACE_BT_INTERFCE */
                    PACKAGE_SDIO_UART = 0, 
                    PACKAGE_USB_USB = 1,
                    PACKAGE_MPCIE_USB = 2,
                    PACKAGE_PCIE_USB = 3};


#else
#define RTK8723_S                                       0x00 //SDIO_UART mode no GPS
#define RTK8723_U                                       0x01 //USB multiple  mode
#define RTK8723_EE                                      0x02 // PCIE multiple mode
#define RTK8822B_ME                                     0x02 // mPCIE-USB mode, from 8822b
#define RTK8723_E                                       0x03 // PCIE-USB mode


#define COMBO_INF_SDIO_UART                             0x00
#define COMBO_INF_USB_MULTI                             0x01
#define COMBO_INF_PCIE_MULTI                            0x02
#define COMBO_INF_PCIE_USB                              0x03

enum package_hci{/* PACKAGE_WIFI_INTERFACE_BT_INTERFCE */
                    /* for 8723d, 8821c v6 */
                    PACKAGE_SDIO_UART = 0, 
                    PACKAGE_USB_USB = 1,
                    PACKAGE_PCIE_UART = 2,
                    PACKAGE_PCIE_USB = 3};


#endif
#else

enum package_hci{/* PACKAGE_WIFI_INTERFACE_BT_INTERFCE */
#ifdef _RTL8821C_SPECIFIC_
                        /* for 8723d, 8821c v6 */
                        PACKAGE_SDIO_UART = 0, 
                        PACKAGE_USB_USB = 1,
                        PACKAGE_PCIE_UART = 2,
                        PACKAGE_PCIE_USB = 3};
#endif

#ifdef _RTL8822B_SPECIFIC_
                    /* only for 8822b v6 */
                    PACKAGE_SDIO_UART = 0, 
                    PACKAGE_USB_USB = 1,
                    PACKAGE_MPCIE_USB = 2,
                    PACKAGE_PCIE_USB = 3,
                    PACKAGE_PCIE_UART = 6,
                    PACKAGE_MPCIE_UART = 7};
#endif

#endif /* _SUPPORT_3BITS_HCI_SELECTION_  */

#define ENABLE      1
#define DISABLE     0
#define POSITIVE     ENABLE
#define NEGATIVE     DISABLE
#define ONE_ANTENNA  0
#define DUAL_ANTENNA 1
#define MSEC         0 //ms 
#define USEC         1 //us     


#define GPS_GPIO_USE_WAKE_UP  0x01
#define LED0_GPIO_USE_WAKE_UP 0x02
#define LED1_GPIO_USE_WAKE_UP 0x03

#define RETRY_INDEX         4
#define WAKE_UP_POLL_TIME   2


#ifdef LPS_NEW_CAL_MODE
#define NEW_CAL_100PPM    4
#define NEW_CAL_250PPM    10
#endif

#define GPIO_WAKE_OPTION(bit, value, on) ((on) ? (value << bit) : 0)

#define POWER_ENABLE_BIT(bit, on)  ((on) ? (1 << (bit)) : 0) 

//All chip power option
#ifdef _8821A_BTON_DESIGN_
#define EF_SOP_PDN_32K_EN   POWER_ENABLE_BIT(13,DISABLE)
#define EF_SOP_ERCK         POWER_ENABLE_BIT(12,DISABLE)
#define EF_SOP_ACKF         POWER_ENABLE_BIT(11,DISABLE)
#define EF_LOP_ACKF         POWER_ENABLE_BIT(10,DISABLE)
#endif
#define SOP_ESWR     POWER_ENABLE_BIT(9,ENABLE)
#define SOP_PWMM     POWER_ENABLE_BIT(8,DISABLE)
#define SOP_EECK     POWER_ENABLE_BIT(7,ENABLE)
#define SOP_EXTL     POWER_ENABLE_BIT(6,DISABLE)
#define SOP_EBNG     POWER_ENABLE_BIT(5,ENABLE)
#define LOP_ESWR     POWER_ENABLE_BIT(4,ENABLE)
#define LOP_PWMM     POWER_ENABLE_BIT(3,DISABLE)
#define LOP_EECK     POWER_ENABLE_BIT(2,ENABLE)
#define LOP_EXTL     POWER_ENABLE_BIT(1,DISABLE)
#define LOP_EBNG     POWER_ENABLE_BIT(0,ENABLE)

//power switch
#ifdef _8821A_BTON_DESIGN_
#define EF_PCM_PULL_LOW_PDN_SUS_3           POWER_ENABLE_BIT(15,DISABLE)
#define EF_PCM_PULL_LOW_PDN_SUS_2           POWER_ENABLE_BIT(14,DISABLE)
#define EF_PCM_PULL_LOW_PDN_SUS_1           POWER_ENABLE_BIT(13,DISABLE)
#define EF_PCM_PULL_LOW_PDN_SUS_0           POWER_ENABLE_BIT(12,DISABLE)
#define EF_PCM_PULL_LOW_ACT_3               POWER_ENABLE_BIT(11,DISABLE)
#define EF_PCM_PULL_LOW_ACT_2               POWER_ENABLE_BIT(10,DISABLE)
#define EF_PCM_PULL_LOW_ACT_1               POWER_ENABLE_BIT(9,DISABLE)
#define EF_PCM_PULL_LOW_ACT_0               POWER_ENABLE_BIT(8,DISABLE)
#define EF_PCM_PULL_LOW_EN                  POWER_ENABLE_BIT(7,DISABLE)
#endif
#define UART_HCI_RESET_EN                POWER_ENABLE_BIT(6,DISABLE)
#define PTA_ANTENNA_STATUS               POWER_ENABLE_BIT(5,DUAL_ANTENNA)
#define FORCE_MASK_WIFI_CH               POWER_ENABLE_BIT(4,DISABLE)
#define EFUSE_POWER_OPTION_EN            POWER_ENABLE_BIT(3,DISABLE)
#define USB_SIE_NY                       POWER_ENABLE_BIT(2,DISABLE)
#define BT_CORE_LDO_STANDYBY             POWER_ENABLE_BIT(1,DISABLE)

#ifdef _YL_LPS_EFUSE_FORCE
#define LPS_TIMER                        POWER_ENABLE_BIT(0,ENABLE)
#else
#define LPS_TIMER                        POWER_ENABLE_BIT(0,DISABLE)
#endif

//General control option
#ifdef _8821A_BTON_DESIGN_
#define EF_PDN_SUS_FA_RECOV_DELAY        (2<<13)
#define EF_PDN_SUS_FA_RECOV_EN           POWER_ENABLE_BIT(12,ENABLE)
#endif
#define PDN_DELAY_TIME_UNIT              POWER_ENABLE_BIT(11,MSEC)
#define LOG_UART_RX_EN                   POWER_ENABLE_BIT(10,DISABLE)
#ifdef _8821A_BTON_DESIGN_
#define EF_GPIO_WAKE_OPENDRAIN           POWER_ENABLE_BIT(9,ENABLE)
#else
#define AFE_LDO_HW_EN                    POWER_ENABLE_BIT(9,ENABLE)
#endif
#define UART_H5_WAKEUP_EN                POWER_ENABLE_BIT(5,DISABLE)
#define WAKE_UP_GPIO_POLARITY            POWER_ENABLE_BIT(4,POSITIVE)
#define GPIO_FOR_LED_FUN_EN              POWER_ENABLE_BIT(3,DISABLE)
#define WAKE_UP_GPIO_TYPE                GPIO_WAKE_OPTION(1, GPS_GPIO_USE_WAKE_UP, DISABLE)
#define GPIO_FOR_WAKE_UP_EN              POWER_ENABLE_BIT(0,DISABLE)

#ifdef _8821A_BTON_DESIGN_
#define EFUSE_POWER_OPTION \
           LOP_EBNG    | LOP_EXTL    | LOP_EECK    | LOP_PWMM | LOP_ESWR | \
           SOP_EBNG    | SOP_EXTL    | SOP_EECK    | SOP_PWMM | SOP_ESWR | \
           EF_LOP_ACKF | EF_SOP_ACKF | EF_SOP_ERCK | EF_SOP_PDN_32K_EN
#else
#define EFUSE_POWER_OPTION \
           LOP_EBNG | LOP_EXTL | LOP_EECK | LOP_PWMM | LOP_ESWR | \
           SOP_EBNG | SOP_EXTL | SOP_EECK | SOP_PWMM | SOP_ESWR
#endif

#ifdef _8821A_BTON_DESIGN_
#define EFUSE_POWER_PARAMETER \
           LPS_TIMER | BT_CORE_LDO_STANDYBY | EFUSE_POWER_OPTION_EN | \
           FORCE_MASK_WIFI_CH | PTA_ANTENNA_STATUS |UART_HCI_RESET_EN |\
           EF_PCM_PULL_LOW_EN | \
           EF_PCM_PULL_LOW_ACT_0 | EF_PCM_PULL_LOW_ACT_1 | EF_PCM_PULL_LOW_ACT_2 | EF_PCM_PULL_LOW_ACT_3 |\
           EF_PCM_PULL_LOW_PDN_SUS_0 | EF_PCM_PULL_LOW_PDN_SUS_1 | EF_PCM_PULL_LOW_PDN_SUS_2 | EF_PCM_PULL_LOW_PDN_SUS_3           
#else
#define EFUSE_POWER_PARAMETER \
           LPS_TIMER | BT_CORE_LDO_STANDYBY | EFUSE_POWER_OPTION_EN | \
           FORCE_MASK_WIFI_CH | PTA_ANTENNA_STATUS |UART_HCI_RESET_EN
#endif

#ifdef _8821A_BTON_DESIGN_
#define GENERAL_CONTROL_OPTION \
           GPIO_FOR_WAKE_UP_EN | WAKE_UP_GPIO_TYPE | GPIO_FOR_LED_FUN_EN | \
           UART_H5_WAKEUP_EN | \
           EF_GPIO_WAKE_OPENDRAIN | LOG_UART_RX_EN | PDN_DELAY_TIME_UNIT | \
           EF_PDN_SUS_FA_RECOV_DELAY | EF_PDN_SUS_FA_RECOV_EN
#else
#define GENERAL_CONTROL_OPTION \
           GPIO_FOR_WAKE_UP_EN | WAKE_UP_GPIO_TYPE | GPIO_FOR_LED_FUN_EN | \
           UART_H5_WAKEUP_EN | \
           AFE_LDO_HW_EN | LOG_UART_RX_EN |PDN_DELAY_TIME_UNIT
#endif
#define CONTROL_TIME_PARAMETER  ((WAKE_UP_POLL_TIME<<4)|RETRY_INDEX)
           

/*
 * bit
 * 0:1:         System clock:
 *                  00b: 5MHz; 01b:10MHz; 10b:20MHz; 11b:40MHz
 * 2:            Bluewiz reset:
 *                  defaut: 0,reset state
 *                  1: disable reset
 * 3:4:
 * 5:            Modem reset:
 *                  defaut: 0,reset state
 *                  1: disable reset 
 * 6:7:
 * 8:            The extern clock gat of bluewiz:
 *                  defaut: 0,gating state
 *                  1: disable gating 
 * 9:            CPU clock 40MHz switch:
 *                  defaut: 0, 32K mode
 *                  1: enable 40MHz mode 
 * 10:           Exist bluewiz seep sleep mode:
 *                  defaut: 0
 *                  1: force leaving deep sleep mode 
 * 11:31:   
**/
typedef union BT_SYS_CTRL_REG_S {
    UINT32 d32;
    struct 
    {
        UINT32 sys_clk                          :2; //bit0~1
        UINT32 bluewiz_rst                      :1; //bit2
        UINT32 reserved_3_4                     :2; //bit3~4
        UINT32 modem_rst                        :1; //bit5
        UINT32 reserved_6_7                     :2; //bit6~7
        UINT32 external_bluewiz_clk_gat         :1; //bit8
        UINT32 cpu_40mhz_en                     :1; //bit9
        UINT32 exist_deep_sleep_mode            :1; //bit10
        UINT32 reserved_11_31                   :21; //bit 11~31
    }b;
}BT_SYS_CTRL_REG_S_TYPE;


/*	Added by Wallice Su for USB LPM.	2012/02/29	*/
#define BTON_USB_LPM				0x20 //struct: BTON_USB_LPM_REG_S
typedef union BTON_USB_LPM_REG_S {
    UINT32 d32;
    struct 
    {
        UINT32 BESL_Thrd                        :4; //bit0~3    
        UINT32 HIRD_Thrd                        :4; //bit4~7    
        UINT32 LPM_Allow                        :1; //bit8      
        UINT32 reserved_9_15                    :7; //bit9~15
        UINT32 Suspend_Mode                     :1; //bit16, L1(if 1), L2:0, 03b:lpm_active
        UINT32 L2RMT_WkUp                       :1; //bit17
        UINT32 HIRD_BESL_Val                    :4; //bit18~21,
        UINT32 HIRD_BESL_Ind                    :1; //bit22
        UINT32 USB_Suspend_Ind                  :1; //bit23: 1:L2 suspend, 0:normal or L1:check bit16
        UINT32 reserved_24_31                   :8; //bit24~31
//normal: USB_Suspend_Ind = 0,Suspend_Mode  = dont care
//L1: USB_Suspend_Ind = 1, Suspend_Mode = 1
//L2: USB_Suspend_Ind = 1, Suspend_Mode = 0
    }b;
}BTON_USB_LPM_REG_S_TYPE;
/*	End Added by Wallice Su for USB LPM.	2012/02/29	*/

typedef union BTON_POW_CTRL_REG_S{ /* Vendor 0x40 */
    UINT32  d32;
    struct 
    {
        UINT32 mailbox_wakeup_en        :1; //bit0, Enable wakeup by mailbox
        UINT32 bluewiz_wakeup_en        :1; //bit1, Enable wakeup by Bluewiz
        UINT32 uart0_wakeup_en          :1; //bit2, Enable wakeup by UART0
        UINT32 usb_wakeup_en            :1; //bit3, Enable wakeup by USB data in
#ifdef _8821A_BTON_DESIGN_
        UINT32 bt_gpio13_wk_wakeup_en      :1; //bit4, 1: enable BT_GPIO13(HOST_WAKE_BT) LPS wakeup
#else        
        UINT32 gps_gpio_wakeup_en       :1; //bit4, Enable wakeup by GPS GPIO power down
#endif        
        UINT32 bt_gpio_wakeup_en        :1; //bit5, Enable wakeup by BT GPIO power down
        UINT32 cal_thermeter_intr_en    :1; //bit6, Enable thermal meter interrupt
        UINT32 cal_timer_intr_en        :1; //bit7, Enable 32KHz timer interrupt
        UINT32 mailbox_wakeup_sts       :1; //bit8, W1C, Status of mailbox wakeup
        UINT32 bluewiz_wakeup_sts       :1; //bit9, W1C, Status of Bluewiz wakeup interrupt
        UINT32 uart0_wakeup_sts         :1; //bit10, W1C, Status of UART0 wakeup interrupt
        UINT32 usb_wakeup_sts           :1; //bit11, W1C, Status of USB data in wakeup interrupt
#ifdef _8821A_BTON_DESIGN_
        UINT32 bt_gpio13_wk_wakeup_sts      :1; //bit12, W1C, interrrupt status of BT_GPIO13(HOST_WAKE_BT)
#else
        UINT32 gps_gpio_wakeup_sts      :1; //bit12, W1C, Status of GPS GPIO power on LPS-wakeup interrupt
#endif        
        UINT32 bt_gpio_wakeup_sts       :1; //bit13, W1C, Status of BT GPIO power on wakeup interrupt
        UINT32 cal_thermeter_intr_sts   :1; //bit14, W1C, Status of thermal meter interrupt
        UINT32 cal_timer_intr_sts       :1; //bit15, W1C, Status of 32KHz timer interrupt

#ifndef _REMOVE_HCI_PCIE_
        UINT32 pcie_sus_intr_sts        :1; //bit16, W1C, Status of PCIE suspend wakeup interrupt
#else

#ifdef _SUPPORT_BT_CTRL_FM_
        UINT32 fm_wake_bt_int_sts        :1; //bit16, W1C, Status of FM pad enable interrupt
#else
        UINT32 just_for_v5_test_0        :1;//bit16
#endif
#endif /*end of _REMOVE_HCI_PCIE_ */

/*        
#ifndef  _REMOVE_HCI_PCIE_
        UINT32 pcie_sus_intr_sts        :1; //bit16, W1C, Status of PCIE suspend wakeup interrupt
#endif
#if defined(_REMOVE_HCI_PCIE_) && defined(_SUPPORT_BT_CTRL_FM_)
        UINT32 fm_wake_bt_int_sts        :1; //bit16, W1C, Status of FM pad enable interrupt
#endif
*/

        UINT32 usb_sus_intr_sts         :1; //bit17, W1C, Status of USB suspend wakeup interrupt


/*        
#ifndef  _REMOVE_HCI_PCIE_        
        UINT32 pcie_sus_intr_en         :1; //bit18, Enable PCIE suspend wakeup interrupt
#endif
#if defined(_REMOVE_HCI_PCIE_) && defined(_SUPPORT_BT_CTRL_FM_)
        UINT32 fm_wake_bt_int_en         :1; //bit18, Enable FM pad enable interrupt
#endif
*/
#ifndef  _REMOVE_HCI_PCIE_        
            UINT32 pcie_sus_intr_en         :1; //bit18, Enable PCIE suspend wakeup interrupt
#else
    
#ifdef _SUPPORT_BT_CTRL_FM_
            UINT32 fm_wake_bt_int_en         :1; //bit18, Enable FM pad enable interrupt
#else
            UINT32 just_for_v5_test_1        :1;//bit18
#endif
    
#endif /* end of _REMOVE_HCI_PCIE_ */



        UINT32 usb_sus_intr_en          :1; //bit19, Enable USB suspend wakeup interrupt
#ifdef _8821A_BTON_DESIGN_
        UINT32 bt_gpio13_wk_wakeup_polarity:1; //bit20, 1: positive edge, 0: negedge, 
#else
        UINT32 gps_sus_en               :1; //bit20, Enable GPS suspend
#endif        
        UINT32 bt_sus_en                :1; //bit21, Enable BT suspend
#ifdef _8821A_BTON_DESIGN_
        UINT32 wifi_onoff_intr_sts      :1; //bit22, W1C, Interrupt to inform BT the change of WiFi status 
#else
        UINT32 gps_hwpdn_en             :1; //bit22, Enable GPS power down
#endif        
        UINT32 bt_hwpdn_en              :1; //bit23, Enable BT power down
        UINT32 uart0_resume_en          :1; //bit24, Enable resume once UART0 toggle
        UINT32 resume_to_sie_en         :1; //bit25, Enable resume to SIE; Also control BT_WAKE_HOST(GPIO14) at 8821AS/AU
#ifdef _8821A_BTON_DESIGN_   
        UINT32 wifi_onoff_intr_en       :1; //bit26, 1: Enable Interrupt to inform BT the change of WiFi status
        UINT32 wake_on_lan_en           :1; //bit27, resume by BT_DIS_N (GPIO11) (Enable Wake on LAN)
#else
        UINT32 wake_on_lan_en           :2; //bit26 ~ 27, Enable Wake on LAN
#endif        
        UINT32 gating_xtal2rf_en        :1; //bit28, Enable XTAL to RF clock gating
        UINT32 gating_xtal2afe_en       :1; //bit29, Enable XTAL to AFE clock gating
        UINT32 lps_mode_en              :1; //bit30, sysclk_32k_en, Enable BT core to 32KHz low power mode
        UINT32 gating_hci_clk_en        :1; //bit31, Enable gated hci clock
    }b;
}BTON_POW_CTRL_REG_S_TYPE;

typedef union BTON_INTERFACE_CTRL_REG_S{ /* Vendor 0x44 */
    UINT32 d32;
    struct
    {
#ifdef _8821A_BTON_DESIGN_    
        UINT32 reg_uart_info_h5_sts_d16 :16; //[15:0] Used as H5 status backup
                                            //Value restored when recovery from power-down/ suspend
        UINT32 rsvd_for_future          :8; //[23:16]Value restored when recovery from power-down/ suspend
#else
        UINT32 uart_baudrate_record     :24;//bit0~23, used as UART baud rate register
                                            //Value restored when recovery from power-down/ suspend
#endif                                            
#ifdef _8821A_BTON_DESIGN_
        UINT32 p_gpio11                 :1; //bit24, BT_DIS_N (GPIO11) Value
#else
        UINT32 pcie_gps_fen_sts         :1; //bit24, Status of GPS with PCIE interface
#endif        
        UINT32 bt_fen_sts               :1; //bit25, Status of BT function, (PCIE_BT | USB_BT | UART_BT)
#ifdef _8821A_BTON_DESIGN_        
        UINT32 gpio_hw_pdn_sts          :1; //bit26, Status of EFUSE: HW PDN by BT_DIS_N (GPIO11)
#else
        UINT32 usb_gps_fen_sts          :1; //bit26, Status of GPS with USB interface
#endif
        UINT32 gpio_reset_rf_sts        :1; //bit27, Status of EFUSE: RF reset by BT_DIS_N (GPIO11)
                                            // TODO: to be verified
        UINT32 hci_uart_fen_sts         :1; //bit28, Status of UART HCI from EFUSE
#ifdef _8821A_BTON_DESIGN_        
        UINT32 wifi_onoff_raw_sts       :1; //bit29, WiFi ON/OFF RAW Status 
#else
        UINT32 polarity_gps_gpio_sts    :1; //bit29, Polarity of GPS GPIO power down pin
#endif
        UINT32 polarity_bt_gpio_sts     :1; //bit30, Polarity of BT power down pin - GPIO11 (BT_DIS_N)
        UINT32 clk_ana_sts              :1; //bit31, Status of CLK_ANA, 0: 500KHz; 1: 12MHz
    }b;
}BTON_INTERFACE_CTRL_REG_S_TYPE;


/*
 *   bit
 * 15:8:      When bluewiz wake up, it will wait this value.
 *              Then, it will switch bluewiz to 40MHz
 *  7:3:       no use
 *     2:       0: 32.768k
 *               1: 32k
 *     1:       1) Gating 40Mz for bluewiz
 *               2) Switch bluewiz to 32K
 *               3) According to 0x6E(Wakeup instant register), 
 *                   bluewiz used 32KHz clock to count
 *     0:       1) Gating 40Mz for bluewiz
 *               2) According to 0x6E(Wakeup instant register), 
 *                   bluewiz used 40MHz clock to count
**/
typedef union BB_POWER_CONTROL_REG_S{
    UINT16 d16;
    struct
    {
        UINT16 enable_sleep               :1; //bit 0
        UINT16 enter_deep_sleep           :1; //bit 1
        UINT16 lpo_freq_sel               :1; //bit 2
        UINT16 bb_clk_freq_minus          :5; //bit3-7
        UINT16 osc_startup_delay          :8; //bit8-15
    }b;
}BB_POWER_CONTROL_REG_S_TYPE;


typedef union POW_OPTION_AND_32K_CTRL_S{ /* Vendor 0x50 */
    UINT32 d32;
    struct
    {
        UINT32 reserved_0_13                :14; //[13:0] CLKGEN_0[13:0]	Reserved 
        UINT32 clkgen_0                     :2; //[15:14] CLKGEN_0[15:14]	32kHz OSC current bias. 00:45nA ,11:180nA
        UINT32 bandgap_mbias_lps_en         :1; //[16] LPS option to enable Bandgap/Mbias
        UINT32 xtal_lps_en                  :1; //[17] LPS option to enable XTAL
        UINT32 efuse_power_clk_lps_en       :1; //[18] LPS option to enable Efuse Cell 1.2 power and Loader Clock
        UINT32 swr_2_pwm_lps_en             :1; //[19] LPS option to enable SWR to PFM mode
        UINT32 swr_lps_en                   :1; //[20] LPS option to enable SWR
        UINT32 bandgap_mbias_sus_en         :1; //[21] Suspend option to enable Bandgap/Mbias
        UINT32 xtal_sus_en                  :1; //[22] Suspend option to enable XTAL
        UINT32 efuse_power_clk_sus_en       :1; //[23] Suspend option to enable Efuse Cell 1.2 power and Loader Clock
        UINT32 swr_2_pwm_sus_en             :1; //[24] Suspend option to enable SWR to PFM mode
        UINT32 swr_sus_en                   :1; //[25] Suspend option to enable SWR
        UINT32 btdbg_gpio_sel               :2; //[27:26] BTDBG_GPIO selection
        UINT32 uart0_sus_en                 :1; //[28] Enable UART0 suspend by firmware
#ifdef _8821A_BTON_DESIGN_ 
        UINT32 GPIOFUNC_I_13_debunc         :1; //[29] BT_GPIO13(HOST_WAKE_BT) RAW Status
#else
        UINT32 reserved_29                  :1; //[29]
#endif        
        UINT32 hci_selection                :2; //[31:30]: map to package_wifi_bt

    }b;
}POW_OPTION_AND_32K_CTRL_S_TYPE;

typedef union TIME_CTRL_REG_S { /* Vendor 0x4C */  // TODO: to be modified
    UINT32 d32;
    struct
    {
        UINT32 cal_timer_period             :23; //bit 0:22, Period for hardware calibration by 32KHz counter
        UINT32 cal_timer_en                 :1; //bit 23, Enable 32KHz timer
        UINT32 timer_hw_cal_en              :1; //bit 24, Enable calibration by 32KHz timer
        UINT32 debug_sel                    :4; //bit 25:28, Selection of BTON debug signals
#ifdef _8821A_BTON_DESIGN_ 
        UINT32 rsvd1                        :2; //bit 29:30
#else
        UINT32 eecs_pin_sel                 :1; //bit 29
        UINT32 trxiq_pin_sel                :1; //bit 30
#endif        
        UINT32 timer_hw_cal_sts             :1; //bit 31, TStatus of calibration by timer
    }b;
}TIME_CTRL_REG_S_TYPE;


#ifdef _8821A_BTON_DESIGN_

#ifdef _NEW_BTON_DESIGN_AFTER_RTL8821B_TC_

typedef union BTON_54_REG_S{ /* Vendor 0x54 */
    UINT32 d32;
    struct
    {
        UINT32 bt_symon_rdata:8;      //[7:0]   R   (Read data of wifi page0)
        UINT32 bt_symon_addr:10;      //[17:8]  R/W (Access wifi page0 registers)
        UINT32 RSVD:14;               //[31:18]         
    };
}BTON_54_REG_S_TYPE;


typedef union BTON_58_REG_S{ /* Vendor 0x58 */
    UINT32 d32;
    struct
    {
        UINT32 BT_ANAPAR_PLL_9_0:10;  //[9:0]   R   (dummy)
        UINT32 BT_ANAPAR_PLL5_11_10:2;//[11:10] R   (reg_mcco<1:0>)
        UINT32 BT_ANAPAR_PLL5_12:1;   //[12]    R   (reg_icp_double)
        UINT32 BT_ANAPAR_PLL5_15_13:3;//[15:13] R   (dummy)
        UINT32 COMP_GM_CUR:2;         //[17:16] R   (comparator gm cell current)
        UINT32 COMP_LATCH_CUR:2;      //[19:18] R   (comparator latch cell current)
        UINT32 COMP_LOAD_CUR:2;       //[21:20] R   (comparator loading cell current)
        UINT32 LDO_V18ADJ:4;          //[25:22] R   (32K internal ldo)
        UINT32 FREQ_SEL:2;            //[27:26] R   (Analog to dsm的頻率)
        UINT32 RSVD:4;                //[31:28] R

    };
}BTON_58_REG_S_TYPE;


typedef union BTON_5C_REG_S{ /* Vendor 0x5C */
    UINT32 d32;
    struct
    {
#ifdef _NEW_BTON_DESIGN_AFTER_RTL8703B_TC_
        UINT32 rsvd:23;               //[22:0] R/W
#else    
        UINT32 ind_32k_wdata:16;      //[15:0]  R/W   (32k register write date. Read data)
        UINT32 ind_32k_addr:6;        //[21:16] R/W   (32k register address)
        UINT32 ind_32k_rw:1;          //[22]    W     (1: write 32k register)
#endif        
        UINT32 Reg_se0_rst_intr:1;    //[23]    R/W   (Usb se0 reset interrupt enable)
        UINT32 TEMP_COMP:4;           //[27:24] R/W   (tank平坦區)
        UINT32 reg_POW_CKGEN_32:1;    //[28]    R/W
        UINT32 St_se0_rst:1;          //[29]    R/W1C (Usb se0 reset interrupt status)
        UINT32 st_cal_fail_intr:1;    //[30]    R/W1C (Calibration fail Wakeup interrupt status)
        UINT32 reg_cal_fail_intr:1;   //[31]    R/W   (Calibration fail Wakeup interrupt enable)        
    };
}BTON_5C_REG_S_TYPE;


typedef union BTON_70_REG_S{ /* Vendor 0x70 */
    UINT32 d32;
    struct
    {
        UINT32 dummy_0:1;             //[0]     R/W    
        UINT32 pow_pll:1;             //[1]     R/W   (1: power on PLL)
        UINT32 reg_cp_bias:3;         //[4:2]   R/W   (pll CP current selection)
        UINT32 reg_kvco:1;            //[5]     R/W   (pll VCO KVCO select)
        UINT32 reg_c3_set:2;          //[7:6]   R/W   (Set C3)
        UINT32 reg_cp_set:2;          //[9:8]   R/W   (Set CP)
        UINT32 reg_cs_set:2;          //[11:10] R/W   (Set CS)
        UINT32 reg_r3_set:3;          //[14:12] R/W   (Set r3)
        UINT32 reg_mbias:1;           //[15]    R/W   (0: default mbias current 1: 1.5X)
        UINT32 reg_rs_set:3;          //[18:16] R/W   (Set rs)	
        UINT32 reg_dogenb:1;          //[19]    R/W   (0: enable watch dog 1:disable)
        UINT32 dummy_20:1;            //[20]    R/W   
        UINT32 reg_ck_5m_en:1;        //[21]    R/W   (1:enable)
        UINT32 reg_fref_edge:1;       //[22]    R/W   (pll referemce clk edge select 1:negative 0:postive)
        UINT32 reg_testen:1;          //[23]    R/W   (1: enable testing pll feedback clk)
        UINT32 reg_ck_mon_en:1;       //[24]    R/W   (1:Enable mon sel)
        UINT32 reg_ck_mon_sel:3;      //[27:25] R/W   (Set ck_monh clk)
        UINT32 pow_ldo:1;             //[28]    R/W   (1: power on LDO for PLL)
        UINT32 reg_ldo_sel:2;         //[30:29] R/W   (LDO output voltage)
        UINT32 reg_ck_480m_en:1;      //[31]    R/W   (1:enable)
    };
}BTON_70_REG_S_TYPE;

typedef union BTON_74_REG_S{ /* Vendor 0x74 */
    UINT32 d32;
    struct
    {
        UINT32 reg_cp_offset:3;       //[2:0]   R/W   (pll CP offset current)
        UINT32 reg_ck_da_en:1;        //[3]     R/W   (1:enable da ck)
        UINT32 reg_ck_ad_en:1;        //[4]     R/W   (1:enable ad ck)
        UINT32 reg_ck_48m_en:1;       //[5]     R/W   (1:enable 48m ck)
        UINT32 reg_ck_dig_en:1;       //[6]     R/W   (1:enable digital ck)	
        UINT32 reg_ck_ad_div:3;       //[9:7]   R/W   (001:160MHz)
        UINT32 reg_ck_da_div:3;       //[12:10] R/W   (100:80MHz)
        UINT32 reg_ck_dig_div:2;      //[14:13] R/W   (00:40MHz/01:80MHz/10:160MHz)
        UINT32 reg_ck_120m_en:1;      //[15]    R/W   (1:enable)
   
        UINT32 r_use_ext_32k:1;       //[16]    R/W   (0), 1: remove pull-down, valid for RTL8821A-MP, RTL8723B, NOT for RTL2801
        UINT32 wake_by_cts:1;         //[17]
        UINT32 r_use_rts_pin:1;       //[18]    R/W   (0), 1: remove pull-down and output enable, valid for RTL8821A-MP, RTL8723B, NOT for RTL2801
        UINT32 en_bton_nonactive_intr:1; //[19] R/W    1: enable bton interrupt to CPU at non-ACTIVE state
                                         //            0: disable bton interrupt to CPU at non-ACTIVE state (default)
        UINT32 r_ext_32k_sel:1;       //[20]    R/W    1: 32k from UART_RTS, 0: from GPIO6
        UINT32 en_bton_cal_intr_wakeup:1;//[21] R/W    1: power state will return to ACTIVE when cal_timer_intr and cal_tmeter_intr happen
                                         //            0: power state will not return to ACTIVE when cal_timer_intr and cal_tmeter_intr happen (default)
        UINT32 hw_filter_wake:2;      //[22]    R/W   
        UINT32 wake_host_en:2;        //[23]    R      (Status of wake_host_en from SIE)    
        UINT32 rsvd2:2;               //[25:24]
        UINT32 PLL_CKRDY_BT:1;        //[26]    R      (Status of UART/SPI PLL clock ready)
        UINT32 Uart_pll_en:1;         //[27]    R      (Status of UART PLL enable by hardware or firmware (can NOT be used as spi mode flag))
        UINT32 AFE_ldo_en:1;          //[28]    R      (Status of AFE LDO enable by hardware)
        UINT32 St_btafe_iso:1;        //[29]    R      (Status of BT AFE iso)
        UINT32 rsvd3:1;               //[30]       
        UINT32 Reg_BT_USB_NYEN:1;     //[31]    R/W    (Enable BT USB NYEN)
    };
}BTON_74_REG_S_TYPE;


typedef union BTON_A0_REG_S{ /* Vendor 0xA0 */
    UINT32 d32;
    struct
    {
        UINT32 is_boot_from_dlps:1;     /* bit[0]: is cpu boot from dlps ? */
        UINT32 is_rom_code_patch:1;     /* bit[1]: is rom code patch ? */
        UINT32 is_fw_trig_wdg_to:1;     /* bit[2]: does fw trigger watchdog timeout ? */
        UINT32 is_h5_link_reset:1;      /* bit[3]: does h5 link reset ? */
        UINT32 is_send_patch_end_evt:1; /* bit[4]: does we send patch end event ? */
        UINT32 is_pwl_on:1;             /* bit[5]: IS_PWL_ON */		
        UINT32 been_lps:1;              /* bit[6]: ever been in lps */
        UINT32 been_dlps:1;             /* bit[7]: ever been in dlps */		
        UINT32 patch_end_evt_index:8;   /* bit[15:8]: the index of patch end event */
        UINT32 is_hci_uart_man_sram_valid:1; /* bit[16]: is hci uart sram still valid ? */
        UINT32 rsvd2:15;                /* bit[31:17]: reserved */         
    };
}BTON_A0_REG_S_TYPE;

typedef union BTON_A4_REG_S{ /* Vendor 0xA4 */
    UINT32 d32;
    struct
    {
        UINT32 RSVD:32;               //[31:0]         
    };
}BTON_A4_REG_S_TYPE;


typedef union BTON_A8_REG_S{ /* Vendor 0xA8 */
    UINT32 d32;
    struct
    {
        UINT32 RFE_SEL_SDM:1;         //[0]     R/W   (pll referemce clk select)  
        UINT32 reg_psen:1;            //[1]     R/W   (PLL phase shift enable)
        UINT32 reg_ps:3;              //[4:2]   R/W   (decide clock phase when reg_ps_enb = 1)
        UINT32 BB_DBG_SEL_AFE_SDM:4;  //[8:5]   R/W   (SDM debug selection)
        UINT32 DIVN_SDM:6;            //[14:9]  R/W   (SDM divider sel)
        UINT32 ORDER_SDM:1;           //[15]    R/W   (SDM order)
        UINT32 F0F_SDM:13;            //[28:16] R/W   (SDM divider sel)
        UINT32 F0N_SDM:3;             //[31:29] R/W   (SDM divider sele)        
    };
}BTON_A8_REG_S_TYPE;

typedef union BTON_AC_REG_S{ /* Vendor 0xAC */
    UINT32 d32;
    struct
    {
        UINT32 BT_RFE_CTRL:6;       //[5:0] Drive value control: {LNAON_A, LNAON_G, PAPE_A, PAPE_G, DPDT_P, DPDT_N}
        UINT32 BT_DPDT_SR:1;        //[6] Slew rate control
        UINT32 BT_DPDT_E2:1;        //[7] Drive strength control
        UINT32 BT_PAPE_SR:1;        //[8] Slew rate control
        UINT32 BT_PAPE_E2:1;        //[9] Drive strength control
        UINT32 BT_LNAON_SR:1;       //[10] Slew rate control
        UINT32 BT_LNAON_E2:1;       //[11] Drive strength control
        UINT32 BT_RFE_OE:1;         //[12] 1: output, 0: input
        UINT32 BT_DPDT_GNT:1;       //[13] PAD is controlled by BT
        UINT32 BT_PAPE_GNT:1;       //[14] PAD is controlled by BT
        UINT32 BT_LNAON_GNT:1;      //[15] PAD is controlled by BT
        UINT32 r_LOP_ACKF:1;        //[16] Enable ANA_CLK High Freq @ LPS
        UINT32 reg_HSIC_WOL_SEND:1; //[17] ???
        UINT32 reg_HSIC_GPIO_SEND:1;//[18] ???
        UINT32 reg_HSIC_NO_POWER:1; //[19] ??? enable external SUSPEND from GPIO13 ???
        UINT32 bts0_bb_sel_s1_trx:1;//[20] for RTL8723B RFC; (0) use S0, (1) use S1
        UINT32 reserved:3;          //[23:21] 
        UINT32 r_SOP_ERCK:1;        //[24] 
        UINT32 r_SOP_ACKF:1;        //[25] Enable ANA_CLK High Freq @ SUS
        UINT32 rsvd:2;	            //[27:26]
        UINT32 BT_G3_G0_PLEN:4;	    //[31:28] control weakly pull-low of GPIO[3:0] (PINMUX with PCM/I2S) to avoid leakage
    };
}BTON_AC_REG_S_TYPE;

#else

typedef union BTON_32K_TEST_REG_S{ /* Vendor 0x54 */
    UINT32 d32;
    struct
    {
        UINT32 tm_RCAL_32k:15;      //[14:0]   // Note: 15bit in new cal mode 
        UINT32 test_mode_32k:1;     //[15] 
        UINT32 center_cnt_fref:16;  //[31:16]  // Note: Only 10bit in new cal mode
    };
}BTON_32K_TEST_REG_S_TYPE;

typedef union BTON_32K_CSR0_REG_S{ /* Vendor 0x58 */
    UINT32 d32;
    struct
    {
        UINT32 inc_step:4;          //[3:0]
        UINT32 bs_start_bit:4;      //[7:4] 
        UINT32 num_settle:4;        //[11:8]
        UINT32 criter0:2;           //[13:12]
        UINT32 criter1:2;           //[15:14]
        UINT32 num_32k_cyc:6;       //[21:16]  // Note: Only 2bit in new cal mode
        UINT32 cal_mode:1;          //[22]
        UINT32 inc_mode:1;          //[23]
        UINT32 kt_lim:6;            //[29:24]
        UINT32 RCAL_h:2;            //[31:30]
    };
}BTON_32K_CSR0_REG_S_TYPE;

typedef union BTON_32K_CTRL_REG_S{ /* Vendor 0x5C */
    UINT32 d32;
    struct
    {
        UINT32 RCAL:15;             //[14:0] R
        UINT32 reserved:1;          //[15]
        UINT32 reg_POW_CKGEN_32:1;  //[16]
        UINT32 OSC32K_RES_COMP:2;   //[18:17]
        UINT32 OSC32K_DBG_SEL:1;    //[19]
        UINT32 cal_en:1;            //[20] Note: usage is little different with RTL8723A
        UINT32 cnt_read_mode:1;     //[21]
        UINT32 LOCK:2;              //[23:22] R
        UINT32 kt_cnt:6;            //[29:24] R
        UINT32 st_cal_fail_intr:1;  //[30] R/W1C, Calibration fail Wakeup interrupt status
        UINT32 reg_cal_fail_intr:1; //[31] Calibration fail Wakeup interrupt enable
    };
}BTON_32K_CTRL_REG_S_TYPE;



typedef union BTON_32K_NEW_CTRL1_REG_S{ /* Vendor 0xA0 */
    UINT32 d32;
    struct
    {
        UINT32 cnt_fref_rep_0:16;   //[15:0] R
        UINT32 cnt_fref_rep_1:16;   //[31:16] R
    };
}BTON_32K_NEW_CTRL1_REG_S_TYPE;
typedef union BTON_32K_NEW_CTRL2_REG_S{ /* Vendor 0xA4 */
    UINT32 d32;
    struct
    {
        UINT32 cnt_fref_rep_2:16;   //[15:0] R
        UINT32 cnt_fref_rep_3:16;   //[31:16] R
    };
}BTON_32K_NEW_CTRL2_REG_S_TYPE;


#endif
#endif


#ifdef _NEW_BTON_DESIGN_AFTER_RTL8703B_TC_

typedef union PON_F00_REG_S{ /* Vendor 0xF00 */
    UINT32 d32;
    struct
    {
        UINT32 ind_32k_wdata:16;      //[15:0]  R/W   (32k register write date. Read data)
        UINT32 ind_32k_addr:6;        //[21:16] R/W   (32k register address)
        UINT32 ind_32k_rw:1;          //[22]    W     (1: write 32k register)
        UINT32 rsvd:9;                //[31:23] R/W        
    };
}PON_F00_REG_S_TYPE;

#endif


#if defined(_CCH_RETENTION_FLOW_FOR_DLPS_) || defined(_SUPPORT_WL_BT_SCOREBOARD_)
typedef union BTON_DLPS_CONTROL_AND_SCOREBOARD_REG_S{ /* Vendor 0xBC */
    UINT32 d32;
    struct
    {
        UINT32 R_LOP_ECORE:1;       //[0] R/W
        UINT32 rsvd:13;             //[13:1] R/W
        UINT32 wl2bt_int_msk:1;     // [14] W
        UINT32 wl2bt_int_clr:1;     // [15] R/W, interrupt occure, W1C
        UINT32 wlbt_sts:15;         // [30:16]R: wl2bt_sts, W: bt2wl_sts
        UINT32 bt_int_to_wl:1;      // [31]: R/W, 1: asset int to wl, 0: do nothing
    };
}BTON_DLPS_CONTROL_REG_S_TYPE, BTON_SCOREBOARD_REG_S_TYPE;
#endif

#ifndef _NEW_BTON_DESIGN_AFTER_RTL8821B_TC_
typedef union CORE_AND_AFE_LDO_CTRL_S{ /* Vendor 0x74 */
    UINT32 d32;
    struct
    {
#ifdef _8821A_BTON_DESIGN_      
        UINT32 en_ck48m:1;                  //[0] default 0,
        UINT32 en_ckdig:1;                  //[1] default 1, 
        UINT32 D_AD:5;                      //[6:2] (FW dont't set this)
        UINT32 D_DA:5;                      //[11:7] (FW dont't set this)
        UINT32 D_DIG:2;                     //[13:12] (FW dont't set this)
        UINT32 en_ck120m:1;                 //[14] default 0, (Controlled by FW|HW) (for UART 120M mode: should be enabled by FW)
        UINT32 DUMMY:1;                     //[15]
#else
        UINT32 ldo_en                          :1; //bit 0
        UINT32 precharge_vref                  :1; //bit 1
        UINT32 sw_cap                          :2; // bit 2:3
        UINT32 vcm_ldo                         :2; // bit 4:5
        UINT32 reserved_6_15                   :10; // bit 6:15
#endif        
#ifdef _8821A_BTON_DESIGN_      
#ifdef _NEW_RTS_EXT32K_CTRL_ // TODO: to be modified/sync-ed
        UINT32 r_use_ext_32k:1;             //[16] (0), 1: remove pull-down, valid for RTL8821A-MP, RTL8723B, NOT for RTL2801
#else
        UINT32 rsvd1:1;                     //[16]
#endif        
#else        
        UINT32 core_ldo_standby_en:1;       //[16]
#endif        
        UINT32 core_ldo_obuf:1;             //[17]
#ifdef _8821A_BTON_DESIGN_    
#ifdef _NEW_RTS_EXT32K_CTRL_ // TODO: to be modified/sync-ed
        UINT32 r_use_rts_pin:1;             //[18] (0), 1: remove pull-down and output enable, valid for RTL8821A-MP, RTL8723B, NOT for RTL2801
#else
        UINT32 rsvd18:1;                    //[18]
#endif        
        UINT32 en_bton_nonactive_intr:1;    //[19]   1: enable bton interrupt to CPU at non-ACTIVE state
                                            //      0: disable bton interrupt to CPU at non-ACTIVE state (default)
        UINT32 rsvd20:1;                    //[20]
        UINT32 en_bton_cal_intr_wakeup:1;   //[21]   1: power state will return to ACTIVE when cal_timer_intr and cal_tmeter_intr happen
                                            //      0: power state will not return to ACTIVE when cal_timer_intr and cal_tmeter_intr happen (default)
#else
        UINT32 core_ldo_vadj:4;             //[21:18]
#endif
#ifdef _8821A_BTON_DESIGN_      
        UINT32 rsvd2:4;                     //[25:22]
#else                
        UINT32 afe_ldo_hw_ctr_en:1;         //[22], Read-Only
        UINT32 afe_ldo_iso_en:1;            //[23], Read-Only
        UINT32 uart_spi_pll_iso_en:1;       //[24], Read-Only
        UINT32 core_ldo_standby_sts:1;      //[25], Read-Only
#endif        
        UINT32 uart_spi_pll_ready_sts:1;    //[26], Read-Only
        UINT32 uart_pll_en_by_hw_sts:1;     //[27], Read-Only
        UINT32 uart_pll_ldo_en_by_hw_sts:1; //[28], Read-Only
        UINT32 afe_iso_sts:1;               //[29], Read-Only
#ifdef _8821A_BTON_DESIGN_     
        UINT32 rsvd3:1;                     //[30]
#else
        UINT32 afe_ldo_en_sts                  :1; //bit 30
#endif        
        UINT32 usb_lpm_ny_en:1;             //[31]
    }b;
}CORE_AND_AFE_LDO_CTRL_S_TYPE;



#ifndef _8821A_BTON_DESIGN_
#ifdef _RTK8723_UART_INIT_
// TODO: Need to double-check
typedef union BTON_UART_SPI_AFE_CTRL_REG_S{
    UINT32 d32;
    struct
    {
        UINT32 pow_ldo          :1; //bit 0; AFE_LDO1[0]
        UINT32 pow_pre_ldo      :1; //bit 1; AFE_LDO1[1]
        UINT32 sw_cap           :2; //bit 2~3; AFE_LDO1[3:2]
        UINT32 vcm_ldo          :2; //bit 4~5; AFE_LDO1[5:4]
        UINT32 dummy_6_15       :10; //bit 6~15; AFE_LDO1[15:6]
        UINT32 pow_pll          :1; //bit 16; AFE_PLL[0]
        UINT32 cp_bias          :3; //bit 17~19; AFE_PLL[3:1]
        UINT32 fref_sel         :1; //bit 20; AFE_PLL[4]
        UINT32 dogenb           :1; //bit 21; AFE_PLL[5]
        UINT32 dummy_22         :1; //bit 22; AFE_PLL[6]
        UINT32 fref_edge        :1; //bit 23; AFE_PLL[7]
        UINT32 d                :5; //bit 24~28; AFE_PLL[12:8]
        UINT32 sel              :2; //bit 29~30; AFE_PLL[14:13]
        UINT32 dummy_31         :1; //bit 31; AFE_PLL[15]
    }b;
}BTON_UART_SPI_AFE_CTRL_REG_S_TYPE;
#endif
#else

typedef union BTON_AFE_PLL_CTRL_0_1_REG_S{ /* Vendor 0x70 */
    UINT32 d32;
    struct
    {
        UINT32 pow:1;           //[0] 1: power on PLL, Controlled by HW
        UINT32 reg_cp_bia:3;    //[3:1]
        UINT32 reg_kvco:2;      //[5:4]
        UINT32 reg_c3_set:2;    //[7:6]
        UINT32 reg_cp_set:2;    //[9:8]
        UINT32 reg_cs_set:2;    //[11:10]
        UINT32 reg_r3_set:3;    //[14:12]
        UINT32 DUMMY1:1;        //[15]
        UINT32 reg_rs_set:3;    //[18:16]
        UINT32 reg_dogb:1;      //[19]
        UINT32 reg_lpfen:1;     //[20]
        UINT32 reg_vref_sel:1;  //[21]
        UINT32 reg_fref_edge:1; //[22]
        UINT32 reg_testen:1;    //[23]
        UINT32 reg_en_mon:1;    //[24]
        UINT32 reg_monpll:3;    //[27:25]
        UINT32 POW_LDO:1;       //[28] Enable AFE LDO, Controlled by HW
        UINT32 SREF:2;          //[30:29]
        UINT32 DUMMY2:1;        //[31]
    };
}BTON_AFE_PLL_CTRL_0_1_REG_S_TYPE;


typedef union BTON_AFE_PLL_CTRL_3_4_REG_S{ /* Vendor 0xA8 */
    UINT32 d32;
    struct
    {
        UINT32 RFE_SEL_SDM:1;           //[0] 1: power on PLL, Controlled by HW
        UINT32 BB_RFE_SEL_AFE_SDM:4;    //[4:1] pll referemce clk select AFE, NOTE: FW don't set this
        UINT32 BB_DBG_SEL_AFE_SDM:4;    //[8:5] SDM debug selection
        UINT32 DIVN:6;                  //[14:9] SDM divider sel
        UINT32 DUMMY:1;                 //[15] dummy
        UINT32 F0F:13;                  //[28:16] SDM divider sel
        UINT32 F0N:3;                   //[31:29] SDM divider sel
    };
}BTON_AFE_PLL_CTRL_3_4_REG_S_TYPE;

typedef union BTON_BT_RFE_PAD_CTRL_REG_S{ /* Vendor 0xAC */
    UINT32 d32;
    struct
    {
        UINT32 BT_RFE_CTRL:6;       //[5:0] Drive value control: {LNAON_A, LNAON_G, PAPE_A, PAPE_G, DPDT_P, DPDT_N}
        UINT32 BT_DPDT_SR:1;        //[6] Slew rate control
        UINT32 BT_DPDT_E2:1;        //[7] Drive strength control
        UINT32 BT_PAPE_SR:1;        //[8] Slew rate control
        UINT32 BT_PAPE_E2:1;        //[9] Drive strength control
        UINT32 BT_LNAON_SR:1;       //[10] Slew rate control
        UINT32 BT_LNAON_E2:1;       //[11] Drive strength control
        UINT32 BT_RFE_OE:1;         //[12] 1: output, 0: input
        UINT32 BT_DPDT_GNT:1;       //[13] PAD is controlled by BT
        UINT32 BT_PAPE_GNT:1;       //[14] PAD is controlled by BT
        UINT32 BT_LNAON_GNT:1;      //[15] PAD is controlled by BT
        UINT32 r_LOP_ACKF:1;        //[16] Enable ANA_CLK High Freq @ LPS
        UINT32 reg_HSIC_WOL_SEND:1; //[17] ???
        UINT32 reg_HSIC_GPIO_SEND:1;//[18] ???
        UINT32 reg_HSIC_NO_POWER:1; //[19] ??? enable external SUSPEND from GPIO13 ???
        UINT32 bts0_bb_sel_s1_trx:1;//[20] for RTL8723B RFC; (0) use S0, (1) use S1
        UINT32 reserved:3;          //[23:21] 
        UINT32 r_SOP_ERCK:1;        //[24] 
        UINT32 r_SOP_ACKF:1;        //[25] Enable ANA_CLK High Freq @ SUS
        UINT32 rsvd:2;	            //[27:26]
        UINT32 BT_G3_G0_PLEN:4;	    //[31:28] control weakly pull-low of GPIO[3:0] (PINMUX with PCM/I2S) to avoid leakage
    };
}BTON_BT_RFE_PAD_CTRL_REG_S_TYPE;
#endif
#endif


#ifdef _NEW_BTON_DESIGN_AFTER_RTL8703B_TC_

typedef union BTON_BT_RETENTION_REG_S{ /* Retention register */
    UINT32 d32;
    struct
    {
        UINT32 modem_retention:8;         //[7:0]   R/W	 W: 0xc3 => backup 0x3c => restore 0xa5=> purge 
                                          //             R bit[0]: 1 for enable 0 for done (backup/restore)
        UINT32 rsvd_0:8;                  //[15:8]  R/W
        UINT32 bluewiz_retention:12;      //[27:16] R/W	
        UINT32 modem_ren_rdy:1;           //[28]    R MODEM retention ready
        UINT32 rsvd_1:1;                  //[29]    R
        UINT32 bz_ren_rdy:1;              //[30]    R Bluewiz retention ready.
        UINT32 rsvd_2:1;                  //[31]    R 
    };
}BTON_BT_RETENTION_REG_TYPE;

#endif

#ifdef _NEW_BTON_DESIGN_AFTER_RTL8821B_TC_

typedef union BTON_INDIR_32K_00_REG_S{ /* Indirect Access by 0x5C.Bit[22] 32k_rw: register offset: 0x00 */
    UINT16 d16;
    struct
    {
        UINT16 rsvd:2;                    //[1:0]   R/W	 
        UINT16 bs_start_bit:4;            //[5:2]   R/W
        UINT16 cal_mode:2;                //[7:6]   R/W	
        UINT16 cnt_read_mode:1;           //[8]     R/W
        UINT16 inc_mode:1;                //[9]     R/W
        UINT16 reg_hw_cal_en:1;           //[10]    R/W
        UINT16 reg_reset_n:1;             //[11]    R/W
        UINT16 dsm_fo_sel:1;              //[12]    R/W
        UINT16 dsm_fin:2;                 //[14:13] R/W
        UINT16 sw_cal_en:1;               //[15]    R/W       
    };
}BTON_INDIR_32K_00_REG_S_TYPE;

typedef union BTON_INDIR_32K_01_REG_S{ /* Indirect Access by 0x5C.Bit[22] 32k_rw: register offset: 0x01 */
    UINT16 d16;
    struct
    {
        UINT16 center_cnt_fref_15_0:16;   //[15:0]  R/W	        
    };
}BTON_INDIR_32K_01_REG_S_TYPE;

typedef union BTON_INDIR_32K_02_REG_S{ /* Indirect Access by 0x5C.Bit[22] 32k_rw: register offset: 0x02 */
    UINT16 d16;
    struct
    {
        UINT16 kt_lim:6;                  //[5:0]   R/W	
        UINT16 num_32k_cyc:8;             //[13:6]  R/W	
        UINT16 rsvd:2;                    //[15:14] R/W			
    };
}BTON_INDIR_32K_02_REG_S_TYPE;

typedef union BTON_INDIR_32K_03_REG_S{ /* Indirect Access by 0x5C.Bit[22] 32k_rw: register offset: 0x03 */
    UINT16 d16;
    struct
    {
        UINT16 tm_RCAL:15;                //[14:0]  R/W	       
        UINT16 test_mode:1;               //[15]    R/W	
    };
}BTON_INDIR_32K_03_REG_S_TYPE;

typedef union BTON_INDIR_32K_04_REG_S{ /* Indirect Access by 0x5C.Bit[22] 32k_rw: register offset: 0x04 */
    UINT16 d16;
    struct
    {
        UINT16 inc_step:4;                //[3:0]   R/W  
        UINT16 num_settle:4;              //[7:4]   R/W
        UINT16 center_cnt_fref_21_16:6;   //[13:8]  R/W	
        UINT16 rsvd:2;                    //[15:14] R/W	
    };
}BTON_INDIR_32K_04_REG_S_TYPE;

typedef union BTON_INDIR_32K_05_REG_S{ /* Indirect Access by 0x5C.Bit[22] 32k_rw: register offset: 0x05 */
    UINT16 d16;
    struct
    {
        UINT16 tm_N_int:10;               //[9:0]   R/W 
        UINT16 rsvd:6;                    //[15:10] R/W        
    };
}BTON_INDIR_32K_05_REG_S_TYPE;

typedef union BTON_INDIR_32K_06_REG_S{ /* Indirect Access by 0x5C.Bit[22] 32k_rw: register offset: 0x06 */
    UINT16 d16;
    struct
    {
        UINT16 tm_N_frac:15;              //[14:0]  R/W	   
        UINT16 rsvd:1;                    //[15]    R/W        
    };
}BTON_INDIR_32K_06_REG_S_TYPE;

typedef union BTON_INDIR_32K_07_REG_S{ /* Indirect Access by 0x5C.Bit[22] 32k_rw: register offset: 0x07 */
    UINT16 d16;
    struct
    {
        UINT16 N_frac_os:15;              //[14:0]  R/W
        UINT16 rsvd:1;                    //[15]    R/W        
    };
}BTON_INDIR_32K_07_REG_S_TYPE;


typedef union BTON_INDIR_32K_08_REG_S{ /* Indirect Access by 0x5C.Bit[22] 32k_rw: register offset: 0x08 */
    UINT16 d16;
    struct
    {
        UINT16 criter0:10;                //[9:0]   R/W
        UINT16 RCAL_h:4;                  //[13:10] R/W
        UINT16 dbg_sel:1;                 //[14]    R/W   
        UINT16 rsvd:1;                    //[15]    R/W
    };
}BTON_INDIR_32K_08_REG_S_TYPE;

typedef union BTON_INDIR_32K_09_REG_S{ /* Indirect Access by 0x5C.Bit[22] 32k_rw: register offset: 0x09 */
    UINT16 d16;
    struct
    {
        UINT16 RCAL:15;                   //[14:0]  R
        UINT16 rsvd:1;                    //[15]    R
    };
}BTON_INDIR_32K_09_REG_S_TYPE;

typedef union BTON_INDIR_32K_0A_REG_S{ /* Indirect Access by 0x5C.Bit[22] 32k_rw: register offset: 0x0A */
    UINT16 d16;
    struct
    {	
        UINT16 Cnt_fref_rep_0_15_0:16;    //[15:0]  R   
    };
}BTON_INDIR_32K_0A_REG_S_TYPE;


typedef union BTON_INDIR_32K_0B_REG_S{ /* Indirect Access by 0x5C.Bit[22] 32k_rw: register offset: 0x0B */
    UINT16 d16;
    struct
    {	
        UINT16 Cnt_fref_rep_1_15_0:16;    //[15:0]  R  
    };
}BTON_INDIR_32K_0B_REG_S_TYPE;


typedef union BTON_INDIR_32K_0C_REG_S{ /* Indirect Access by 0x5C.Bit[22] 32k_rw: register offset: 0x0C */
    UINT16 d16;
    struct
    {	
        UINT16 Cnt_fref_rep_2_15_0:16;    //[15:0]  R   
    };
}BTON_INDIR_32K_0C_REG_S_TYPE;


typedef union BTON_INDIR_32K_0D_REG_S{ /* Indirect Access by 0x5C.Bit[22] 32k_rw: register offset: 0x0D */
    UINT16 d16;
    struct
    {	
        UINT16 Cnt_fref_rep_3_15_0:16;    //[15:0]  R
    };
}BTON_INDIR_32K_0D_REG_S_TYPE;


typedef union BTON_INDIR_32K_0E_REG_S{ /* Indirect Access by 0x5C.Bit[22] 32k_rw: register offset: 0x0E */
    UINT16 d16;
    struct
    {
        UINT16 Cnt_fref_rep_0_21_16:6;    //[5:0]   R          
        UINT16 Cnt_fref_rep_3_21_16:6;    //[11:6]  R
        UINT16 rsvd:4;                    //[15:12] R
    };
}BTON_INDIR_32K_0E_REG_S_TYPE;


typedef union BTON_INDIR_32K_0F_REG_S{ /* Indirect Access by 0x5C.Bit[22] 32k_rw: register offset: 0x0F */
    UINT16 d16;
    struct
    {
        UINT16 N_int:10;                  //[9:0]   R
        UINT16 Cnt_fref_rep_1_19_16:4;    //[13:10] R
        UINT16 rsvd:2;                    //[15:14] R        
    };
}BTON_INDIR_32K_0F_REG_S_TYPE;


typedef union BTON_INDIR_32K_10_REG_S{ /* Indirect Access by 0x5C.Bit[22] 32k_rw: register offset: 0x10 */
    UINT16 d16;
    struct
    {
        UINT16 N_frac:15;                 //[14:0]  R     
        UINT16 rsvd:1;                    //[15]    R        
    };
}BTON_INDIR_32K_10_REG_S_TYPE;


typedef union BTON_INDIR_32K_11_REG_S{ /* Indirect Access by 0x5C.Bit[22] 32k_rw: register offset: 0x11 */
    UINT16 d16;
    struct
    {
        UINT16 Cnt_fref_rep_2_21_16:6;    //[5:0]   R   
        UINT16 kt_cnt:6;                  //[11:6]  R  
        UINT16 LOCK:2;                    //[13:12] R   
        UINT16 rsvd:2;                    //[15:14] R/W	        
    };
}BTON_INDIR_32K_11_REG_S_TYPE;
#endif





#ifdef _YL_RTL8723A_B_CUT
typedef struct EFUSE_POW_SETTING_1_S_ {
#ifdef _8821A_BTON_DESIGN_ 
    UINT8 bt_pdn_power_on_check_dis:1;          // [0] default 0, 0: check GPIO11(BT_DIS_N) in gpio_power_on_check();
    UINT8 bt_suspend_power_on_check_dis:1;      // [1] default 0, 0: check and execute SUSPEND Status in gpio_power_on_check()
    UINT8 init_bt_never_close_sram_pwr:1;       // [2] default 0, 1: Never close sram power fpr some reason
    UINT8 bt_fun_sts_power_on_check_en:1;       // [3] default 0, 1: check BT Functioin Status (OFF==>POWER-DOWN) in gpio_power_on_check() 
    // TODO: to be replaced by BTON R Reg
    UINT8 force_hw_pdn_en:1;                    // [4] default 0, To assume EFUSE_HW_PDN_EN is 1 regardless Vendor0x44[26]
                                                // Vendor 0x44[27]: 0/1: GPIO11 acts as HWPDN/RFOFF Function
    UINT8 turn_off_ck120m_at_lps:1;             // [5] default 0, 1: ck120m off when entering LPS, ck120m on when exiting from LPS
    UINT8 lop_sop_low_power_setting_en:1;       // [6] default 0, 1 is preferred; 1: init r_SOP_xxx and r_LOP_xxx for LOW POWER CONSUMPTION, then dynamically change the setting
    UINT8 bt_sus_power_on_check_execute:1;      // [7] default 0, 1: execulte bt_sus_en at gpio_power_on_check()
#else
    UINT8 init_bt_pdn_check_wakeupsts_dis:1;    // 1: do NOT check pow_ctrl.b.bt_gpio_wakeup_sts in gpio_power_on_check();
    UINT8 init_bt_pdn_check_dis:1;                     // 1: do NOT check bt_pdn status in gpio_power_on_check();
    UINT8 init_bt_never_close_sram_pwr:1;       // [2] 1: Never close sram power fpr some reason
    UINT8 rsvd:5;                               // [7:3]
#endif
} EFUSE_POW_SETTING_1_S;

#ifdef _8821A_BTON_DESIGN_
typedef struct EFUSE_POW_SETTING_2_S_ {
    UINT8 hci_reset_init_power_var:1;           // [0] 1b; (default: 0);  1: initialize power-related variable at HCI_RESET
    UINT8 power_on_check_init_power_var:1;      // [1] 1b; (default: 0);  1: initialize power-related variable at REBOOT gpio_power_on_check 
    UINT8 gpio_host_wake_bt_en:1;               // [2] 1b; (default: 0);  1: enable GPIO output
    UINT8 gpio_host_wake_bt_polarity:1;         // [3] 1b; (default: 0); 1: active high, 0: active low
    UINT8 disable_force_usb_r_lop_extl_en:1;    // [4] 1b; (default: 0); valid when lop_sop_low_power_setting_en = 0, 
                                                //                   0: force r_LOP_EXTL = 1 for USB interface
    UINT8 bt_wake_host_init_opt:1;              // [5] 1b; (default: 0); 1: initialized as g_host_state; 0: (!g_host_state)
    UINT8 gpiointr_ignore_sus:1;                // [6] 1b; (default: 0); 1: ignore GpioIntrHandler() bt_sus_en
    UINT8 avoid_pdn_with_sus:1;                 // [7] 1b; (default: 0 => 1); 1: ignore GpioIntrHandler() bt_sus_en when bt_pdn_en
} EFUSE_POW_SETTING_2_S;
typedef struct EFUSE_POW_SETTING_3_S_ {
    UINT8 gpio_pdn_intr_lvl_trig_en:1;          //[0] 1b; (default 0); 1: set as BT_DIS_N as level trigger, 0: edge trigger
    UINT8 gpio_sus_intr_lvl_trig_en:1;          //[1] 1b; (default 0 => 1); 1: set as USB/PCI SUS as level trigger, 0: edge trigger
    UINT8 gpiointr_bypass_debunce:1;            //[2] 1b; (default 0 => 1); 1: do NOT debunce, process_power_gpio_set() always return TRUE
    UINT8 gpiointr_no_break:1;                  //[3] 1b; (default 0); 1: do NOT "break" after debunce check; to solve the problem of lossing PDN-signal, valid when gpiointr_no_break = 0
#ifndef _SUPPORT_BT_CTRL_FM_     
    UINT8 rsvd:1;                               // [4] 1b;
#else    
    UINT8 gpio_fm_en_intr_lvl_trig_en:1;          //[4] 1b; (default 0 => 1); 1: set as fm en as level trigger, 0: edge trigger
#endif    
//    UINT8 fast_gpio_power_on_check:1;           //[4] 1b; (default 1); 1: move gpio_power_on_check for power-down as fast as possible
    UINT8 record_bton_pre_boot_state:1;         //[5] 1b; (default 1); 1: record the bton pre-boot state; (0) just power-on (1) from bt_hwpdn_en (2) from bt_sus_en (3) bt_hwpdn_en+bt_sus_en
    UINT8 dont_change_gpio_settings_at_init:1;  //[6] 1b; (default 0); 1: use the value at reset and PHY_Init, 0: as before 
    UINT8 enable_gpio_power_on_check_at_est_log:1; //[7] 1b; (default 0); 1: enable background gpio power-down/suspend check at baud_est_log()
} EFUSE_POW_SETTING_3_S;
typedef struct EFUSE_POW_SETTING_4_S_ {
    UINT8 fast_gpio_power_on_check2b:2;         // [1:0] 2b; (default 1); 1: move gpio_power_on_check for power-down as fast as possible; 2: after lc_init_radio() (for execute EFUSE Register Write(PHY_Init))
    UINT8 usb_dynamic_lps_xtal_en:1;            // [2] 1b; (default 0); 1: dynamic enable/disable lop_xtal_enable for LPS when USB suspend
    UINT8 rsvd:5;                               // [7:3] 
} EFUSE_POW_SETTING_4_S;
typedef struct EFUSE_POW_SETTING_5_S_ { /* Used for PATCH/ConfigFile */
    UINT8 patch_gpio_wake_host_type           :3; //bit[2:0] 0: NONE, 1: GPIO14, 2: LED0, 3: LED1, 4~7: Reserved for extension
    UINT8 patch_gpio_wake_host_polarity       :1; //bit[3] 0: default active low, 1: default active high
    UINT8 patch_gpio_wake_host_opendrain      :1; //bit[4] 0: push-pull, 1: open-drain(drive the output in active duration)
    UINT8 patch_gpio_wake_host_duration       :2; //bit[6:5] active duration = 20ms*2^x
    UINT8 patch_gpio_wake_host_test_mode      :1; //bit[7] FOR TEST-ONLY, 1: At Boot, Generate 4 cycles of alternating ACTIVE/INACTIVE pulses of 1ms duration 1ms
} EFUSE_POW_SETTING_5_S;

#endif

#else
     /**** unexpected for RTL8723A B_CUT ****/
#endif

#ifdef _UART_H5
typedef union BTON_UART_INFO_REG_S{ /* Vendor 0x7C */
    UINT32 d32;
    struct
    {
#ifdef _8821A_BTON_DESIGN_ 
        UINT32 reg_uart_info_baud_d28:28;   //[27:0] used as UART baudrate backdup
                                            //Value restored when recovery from power-down/ suspend
        UINT32 rsvd_for_future:2;           //[29:28] 
                                            //Value restored when recovery from power-down/ suspend
        UINT32 bton_pre_boot_state:2;       //[31:30] used as BTON pre-Boot State record: (0) just power-on (1) from bt_hwpdn_en (2) from bt_sus_en (3) bt_hwpdn_en+bt_sus_en
                                            //Value restored when recovery from power-down/ suspend
#else    
        UINT32 reg_uart_info_d16                  :16; //bit 0~15, used as H5 status backup
        UINT32 reserved_16_30                     :15; //bit 16~30
        UINT32 reg_uart_pll_en                    :1; //bit 31
#endif        
    }b;
}BTON_UART_INFO_REG_S_TYPE;
#endif


typedef union EFUSE_POW_OPTION_S{
    UINT16 d16;
    struct
    {
        UINT16 bandgap_mbias_lps_en      :1; //bit 0
        UINT16 xtal_lps_en               :1; //bit 1
        UINT16 efuse_power_clk_lps_en    :1; //bit 2
        UINT16 swr_2_pwm_lps_en          :1; //bit 3
        UINT16 swr_lps_en                :1; //bit 4
        UINT16 bandgap_mbias_sus_en      :1; //bit 5
        UINT16 xtal_sus_en               :1; //bit 6
        UINT16 efuse_power_clk_sus_en    :1; //bit 7
        UINT16 swr_2_pwm_sus_en          :1; //bit 8
        UINT16 swr_sus_en                :1; //bit 9
#ifdef _8821A_BTON_DESIGN_
        UINT16 lop_ackf                  :1; //bit 10
        UINT16 sop_ackf                  :1; //bit 11
        UINT16 sop_erck                  :1; //bit 12
        UINT16 sop_pdn_32k_en            :1; //bit 13
        UINT16 rsvd                      :2; //bit 14~15
#else
        UINT16 reserved_10_15            :6; //bit 10~15
#endif        
    }b;
}EFUSE_POW_OPTION_S_TYPE;

#ifndef _CCH_REMOVE_THERMAL_METER_
typedef union THERMAL_METER_CTRL_S{ /* Vendor 0x48 */ // TODO: to be modified
    UINT32 d32;
    struct
    {
        UINT32 thermal_timer_period:12;     //[11:0] Timer period to read thermal meter counting by 32KHZ clock; 
                                            //reg_tmeter_timer needs to be bigger than 12'd6
        UINT32 cur_power_state:4;           //[15:12] R, BTON Current power state
        UINT32 thermal_delta:6;             //[21:16] Temperature delta to enable 32KHz calibration
        UINT32 thermal_timer_en:1;          //[22] Enable thermal meter
        UINT32 thermal_timer_hw_en:1;       //[23] Enable calibration by thermal meter
        UINT32 thermal_value:6;             //[29:24] R, Temp value
        UINT32 thermal_valid:1;             //[30] R, Status of Thermal meter
        UINT32 thermalhw_cal_status:1;      //[31] R, Status of calibration by thermal meter
    }b;
}THERMAL_METER_CTRL_S_TYPE;
#endif


#ifdef _ENABLE_BTON_POWER_SAVING_
void rlx4081_isr_sw_unmask();
void rlx4081_isr_sw_mask();
void enable_lps_mode_setting();
void disable_lps_mode_setting();
void pow_ctrl_intr_handle();
void enter_lps_mode();
#ifdef _YL_LPS
void execute_lps_mode_procedure_6128(UINT8 bz_wake, UINT16 bz_wake_time);
//void exit_lps_mode_procedure_6128(UINT8 bz_wake, UINT16 bz_wake_time);
void enter_lps_mode_6128(UINT8 bluewiz_wakeup);

#ifdef _8821A_BTON_DESIGN_
void bton_32k_ctrl_avoid_int_sts_w1c(UINT32 *bton_32k_ctrl_d32_ptr);
#endif
void bton_pow_ctrl_avoid_int_sts_w1c(BTON_POW_CTRL_REG_S_TYPE *pow_ctrl);
void bton_clr_wakeup_sts(void);

#ifndef _YL_LPS_COMBO_PLATFORM
void execute_exit_lps_mode_procedure_6128(void);
#endif
#ifdef _YL_LPS_TEST
void lps_test_at_keep_alive_event(void);
#endif
#endif

#if !defined(_MOVE_TO_PATCH_) && !defined(SPI_FLASH_BOOT) // TODO:  to be modified/implemented in PATCH
void enable_32k_timer_setting(UINT8 is_on);
#endif
void enable_uart_suspend();
void execute_lps_mode_procedure(UINT8 bz_wake, UINT16 bz_wake_time);
void power_init();
BOOLEAN execute_wakeup_procedure(HCI_TRANSPORT_PKT_TYPE pkt_type,
                                         UCHAR *buf, UINT16 len);
void wake_up_host();
void hci_wake_up_trigger_one_shot_event(UINT32 time_ms);
void hci_wake_up_timer_callback(TimerHandle_t timer_handle);
#ifdef _8821A_BTON_DESIGN_      
#if 0
void execute_bton_entering_pdn_sus(UINT8 option);
#define BTON_ENTERING_PDN 1
#define BTON_ENTERING_SUS 2
#else
void execute_bton_entering_pdn_sus(BTON_POW_CTRL_REG_S_TYPE *pow_ctrl_ptr);
#endif
void bton_set_pcm_pull_low(UINT8 option);
#define PCM_PULL_OPTION_ACT_ST 1
#define PCM_PULL_OPTION_PDN_SUS_ST 2
#else
void trun_on_off_afe_ldo(UINT8 on_off);
#endif

void lps_lop_setting_set_xtal_en(UINT8 val);
#ifndef _REDUCE_LPS_AFTER_RTL8703B_
void lps_handle_timer2_mode_sel0(void);
#endif
void lps_handle_timer2_mode_sel1(void);

void pow_timer_lps();

#if !defined(_MOVE_TO_PATCH_) && !defined(SPI_FLASH_BOOT) // TODO:  to be modified/implemented in PATCH
void hw_timer_cal_func(UINT8 is_on, 
                              UINT8 cal_hw_en,
                              UINT8 wake_lps_en,
                              UINT32 timer_counter);
#endif        
#if !defined(_MOVE_TO_PATCH_) && !defined(SPI_FLASH_BOOT) // TODO:  to be modified/implemented in PATCH
void hw_thermal_cal_func(UINT8 thermal_en, 
                                 UINT8 hw_cal_en, 
                                 UINT8 diff, 
                                 UINT16 timer_period);
#endif

#ifdef _2801_BTON_DESIGN_
void bton_32k_cal_ini();
#endif

void bton_32k_cal_en();
UINT8 check_in_range(UINT32 big, UINT32 small, UINT32 diff);
UINT8 bton_32k_cal_chk_lock();


#ifdef _ENABLE_USB_REMOTE_WAKEUP 
void usb_remote_wakeup(void);
#if defined(_FONGPIN_TEST_USBCV_REMOTEWAKEUP_BY_GPIO0_)
void test_usb_remotewakeup_by_detect_gpio0_high();
#endif

#endif

#ifdef _ACTIVE_CHECK_WIFI_ON_OFF_
UINT8 check_wifi_alive(void);
#endif

#ifdef _MODEM_LNA_CONSTRAINT_
#define CHECK_WIFI_ALIVE_FOR_PTA ((VENDOR_READ(BTON_INTERFACE_CTRL_REG) & BIT29)!=0)
#endif

#endif
#endif

