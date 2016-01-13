#ifndef _GPIO_H_
#define _GPIO_H_

#if defined(_GPIO_POWER_SEQUENCE_ENABLE)

#include "DataType.h"
#include "power_control.h"

#ifdef _ENABLE_VENDOR_GPIO_INTERRUPT_
void vendor_gpio_interrupt_handler();
#endif


#define GPIO7_BIT					0x80
#define GPIO_PORTA_DATA_REG         0x00    //DesigeWare GPIO A Data Value
#define GPIO_PORTA_DATA_DREC_REG    0x04    //Direction or DesignWare GPIO A: 0: input, 1: output
#define GPIO_INT_EN_REG				0x30	//interrupt enable register	
#define GPIO_INT_MASK_REG			0x34	//interrupt mask register
#define GPIO_INT_TYPE_LEVEL_REG		0x38	//0:level_sensitive, 1:edge_sensitive, set 0x38 and 0x3C together
#define GPIO_INT_POLARITY_REG		0x3C	//0:low active /falling edge, 1:high active / rising edge; , set 0x38 and 0x3C together 
#define GPIO_INT_STATUS_REG			0x40	//interrupt status
#define GPIO_RAW_STATUS_REG			0x44	//raw status
#define GPIO_DEBOUNCE_REG			0x48	//remove glitches
#define GPIO_PORTA_EOI_REG			0x4C	//clear interrupt
#define GPIO_EXT_PORTA_REG			0x50	//gpio pin value
#define GPIO_LS_SYNC				0x60	//level sensitive interrupt sync to pclk_intr

#define GPIO_OUTPUT_HIGH            0
#define GPIO_OUTPUT_LOW             1
#define GPIO_OUTPUT_PULSE           2

#define IDLE_MODE                   0
#define TRX_MODE                    1
#define QUERY_MODE                  2

typedef union GPIO_ISR_STS_REG{ /* defines the DWGPIO input interrupt usage */
    UINT32 d32;
    struct
    {
        UINT32 bt_pon:1;                //[0]
#ifdef _8821A_BTON_DESIGN_        
        UINT32 gpio13_host_wake_bt:1;   //[1]                
#else
        UINT32 gps_pon:1;               //[1]
#endif        
        UINT32 usb_suspend:1;           //[2]

#ifndef _REMOVE_HCI_PCIE_ 
        UINT32 pcie_suspend:1;          //[3]
#else
#ifdef _SUPPORT_BT_CTRL_FM_
		UINT32 fm_pad_enable_edge_trig:1;          //[3]
#else
		UINT32 just_for_v5_test: 1;//[3]
#endif  /* end of _SUPPORT_BT_CTRL_FM__*/
#endif  /* end of _REMOVE_HCI_PCIE_*/



#ifdef _8821A_BTON_DESIGN_                
//        UINT32 rsvd_2:3;                //[6:4]
        UINT32 led0:1;                  //[4] from RTL8821A-MP
        UINT32 led1:1;                  //[5] from RTL8821A-MP, Not used in RTL2801 
        UINT32 bt_3dg_sync:1;           //[6] from RTL8821A-TEST
        UINT32 rsvd_3:25;               //[31:7]
#else
        UINT32 vender_4_31:28;          //[31:4]
#endif
    }b;
}GPIO_ISR_STS_REG_TYPE;

typedef union GPIO_PIN_S{ /* defines the DWGPIO input usage */
    UINT32 d32;
    struct
    {
        UINT32 bt_pdn:1;                //[0]
#ifdef _8821A_BTON_DESIGN_        
        UINT32 gpio13_host_wake_bt:1;   //[1]                   
#else
        UINT32 gps_pdn:1;               //[1]
#endif
        UINT32 usb_suspend:1;           //[2]

#ifndef _REMOVE_HCI_PCIE_       
        UINT32 pcie_suspend:1;          //[3]
#else

#ifdef _SUPPORT_BT_CTRL_FM_
		UINT32 fm_pad_enable_edge_trig:1;          //[3]
#else
		UINT32 just_for_v5_test:1;          //[3]
#endif

#endif/* end of _REMOVE_HCI_PCIE_*/



#ifdef _8821A_BTON_DESIGN_                
//        UINT32 usb_remote_wakeup_en:1;   //[4], from SIE
        UINT32 led0:1;                  //[4] from RTL8821A-MP
        UINT32 led1:1;                  //[5] from RTL8821A-MP, Not used in RTL2801
        UINT32 bt_3dg_sync:1;           //[6] from RTL8821A-TEST
        UINT32 rsvd_2:25;               //[31:7]
#else
        UINT32 led0:1;                  //[4]
        UINT32 led1:1;                  //[5]
        UINT32 reserved_6_31:26;        //[31:6]
#endif
    }b;
}GPIO_PIN_S_TYPE;

#ifdef _SUPPORT_BT_CONTROL_PCM_MUX_
typedef union PARK_PCM_GPIO_PIN_S{ /* define the PCM mux gpio state after disconnection */
    UINT16 d16;
    struct
    {
        UINT16 gpio0_switch_to_off:1;   //[0], 0 = on, 1 = off
        UINT16 gpio1_switch_to_off:1;   //[1]                   
        UINT16 gpio2_switch_to_off:1;   //[2]
        UINT16 gpio3_switch_to_off:1;   //[3]
        UINT16 gpio0_is_output:1;       //[4], 0:input, 1: output
        UINT16 gpio1_is_output:1;       //[5]
        UINT16 gpio2_is_output:1;       //[6]
        UINT16 gpio3_is_output:1;       //[7]
        UINT16 gpio0_output_value:1;   //[8]
        UINT16 gpio1_output_value:1;   //[9]
        UINT16 gpio2_output_value:1;   //[10]
        UINT16 gpio3_output_value:1;   //[11]        
        UINT16 restore_pcm_mux_state_sel:1;//[12]
        UINT16 rsvd:3;               //[15:13]
    }b;
}PARK_PCM_GPIO_PIN_S_TYPE;
#endif


#ifdef _8821A_BTON_DESIGN_
typedef union GPIO_POUT_S{ /* defines the DWGPIO output usage */ 
    UINT32 d32;
    struct
    {
        UINT32 rsvd_1:4;                //[3:0]
        UINT32 led0:1;                  //[4]
        UINT32 led1:1;                  //[5], Not used in RTL2801
        UINT32 reserved_6_31:26;        //[31:6]
    }b;
}GPIO_POUT_S_TYPE;
#endif

typedef struct GPIO_BT_ISR_MANAGER_S_ {
    UINT32 state;
    BTON_POW_CTRL_REG_S_TYPE pow_ctrl;
    GPIO_ISR_STS_REG_TYPE gpio_status;
    BTON_INTERFACE_CTRL_REG_S_TYPE bton_inter_info;
    POW_OPTION_AND_32K_CTRL_S_TYPE pow_option;    
} GPIO_BT_ISR_MANAGER_S;

#ifdef _SUPPORT_POLLING_BASED_LPM_L1_
UINT8 usb_read_L1_remote_wakeup_feature(void);
UINT8 is_usb_lpm_l1_state(void);
UINT8 is_usb_suspend_state(void);
#endif

void gpio_init_for_la();
void gpio_one_pull_low(UINT8 index);
void gpio_one_pull_high(UINT8 index);
void gpio_one_pull_pulse(UINT8 index);
void GpioInit(void);
SECTION_ISR_LOW void GpioIntrHandler(void);
BOOLEAN process_power_gpio_set( UINT8 bit_offset,UINT8 * bit_polarity_sts, 
                                                              UINT8 no_delay);
void gpio_power_on_check(void);
BOOLEAN gpio_wake_up_host(void);
#ifndef _8821A_BTON_DESIGN_
void get_wake_up_pin(UINT32 *output_en,UINT32 *init_value, UINT32 *execute_value);
#endif
void gpio_wake_up_trigger_one_shot_event(UINT32 time_ms);
void gpio_wake_up_timer_callback(TimerHandle_t timer_handle);
void dbg_gpio_init(void);
void gpio_output_control(UINT8 index, UINT8 is_high);

UINT8 usb_read_d2_remote_wakeup_feature(void);
UINT8 usb_read_suspend_status(void);

#ifdef _YL_RTL8723A_B_CUT
UINT8 get_gpio_host_wake_bt(void);
void set_gpio_bt_wake_host(UINT8 value);
#endif


#ifdef _SUPPORT_POWERON_ENABLE_LOG_MUX_
void off_domain_log_control(UINT8 enable);
enum{ LOG_OFF = 0, OUTPUT_LOG0 = 1, OUTPUT_LOG1 = 2};// for 8723D, LOG0 = BTGPIO8, LOG1 = BTGPIO10
#endif



#define gpio_force_output_init()                    gpio_init_for_la()
#define gpio_output_high(index)                     gpio_one_pull_high(index)
#define gpio_output_low(index)                      gpio_one_pull_low(index)
#define gpio_output_pulse(index)                    gpio_one_pull_pulse(index)

/*
 * the GPIO marco is used for BT_GPIO[3:0] 
 *  note: must set SW GPIO mode first after set 
 *  wifi register (0x66[10] | 0x66[2:0] = 4'h4) 
 */
#ifdef _BT_ONLY_
#define SET_BT_GPIO_MODE()  \
{ \
    UINT16 temp = RD_16BIT_SYSON_IO(0x66); \
    temp &= 0xFBF8; \
    temp |= 0x0004; \
    WR_16BIT_SYSON_IO(0x66, temp); \
}
#else
/* must set BT gpio mode via wifi driver or wifi efuse */
#define SET_BT_GPIO_MODE()
#endif

#define REG_VEN_GPIO_CTRL   0x38
#define REG_VEN_GPIO_CTRL_HIGH_BYTE_3   0x3b

#ifdef _DAPE_ENABLE_FPGA_GPIO_TRIGGER
#define SET_BT_GPIO_OUTPUT_3_0(gpio_o_3_0) \
{ \
    UINT32 dword; \
    dword = VENDOR_READ(REG_VEN_GPIO_CTRL); \
    dword &= ~(0x00000f00); \
    dword |= 0x000f0000; \
    dword |= ((((UINT32)gpio_o_3_0) & 0xf) << 8); \
    dword |= BIT29; \
    dword |= BIT30; \
    VENDOR_WRITE(REG_VEN_GPIO_CTRL, dword); \
}

#define SET_BT_GPIO_OUTPUT_HIGH(index) \
{ \
    UINT32 dword; \
    dword = VENDOR_READ(REG_VEN_GPIO_CTRL); \
    dword |= 0x101 << (8 + (index)); \
    dword |= BIT30; \
    VENDOR_WRITE(REG_VEN_GPIO_CTRL, dword); \
}

#define SET_BT_GPIO_OUTPUT_LOW(index) \
{ \
    UINT32 dword; \
    dword = VENDOR_READ(REG_VEN_GPIO_CTRL); \
    dword |= (1 << (16 + (index))); \
    dword &= ~((UINT32)(1 << (8 + (index)))); \
    dword |= BIT29; \
    dword |= BIT30; \
    VENDOR_WRITE(REG_VEN_GPIO_CTRL, dword); \
}
#else
#define SET_BT_GPIO_OUTPUT_3_0(gpio_o_3_0) \
{ \
    UINT32 dword; \
    dword = VENDOR_READ(REG_VEN_GPIO_CTRL); \
    dword &= ~(0x00000f00); \
    dword |= 0x000f0000; \
    dword |= ((((UINT32)gpio_o_3_0) & 0xf) << 8); \
    VENDOR_WRITE(REG_VEN_GPIO_CTRL, dword); \
}

#define SET_BT_GPIO_OUTPUT_HIGH(index) \
{ \
    UINT32 dword; \
    dword = VENDOR_READ(REG_VEN_GPIO_CTRL); \
    dword |= 0x101 << (8 + (index)); \
    VENDOR_WRITE(REG_VEN_GPIO_CTRL, dword); \
}

#define SET_BT_GPIO_OUTPUT_LOW(index) \
{ \
    UINT32 dword; \
    dword = VENDOR_READ(REG_VEN_GPIO_CTRL); \
    dword |= (1 << (16 + (index))); \
    dword &= ~((UINT32)(1 << (8 + (index)))); \
    VENDOR_WRITE(REG_VEN_GPIO_CTRL, dword); \
}
#endif


#endif /* end of defined(_GPIO_POWER_SEQUENCE_ENABLE) */

#endif


/*========== section:  Bton gpio [21:0]===========*/

typedef union BTON_GPIO_CTRL0_REG_
{
    struct
    {
        UINT32 bton_gpio0_sel : 1;
        UINT32 bton_gpio1_sel : 1;
        UINT32 bton_gpio2_sel : 1;
        UINT32 bton_gpio3_sel : 1;
        UINT32 bton_gpio4_sel : 1;
        UINT32 bton_gpio5_sel : 1;
        UINT32 bton_gpio6_sel : 1;
        UINT32 bton_gpio7_sel : 1;
        UINT32 bton_gpio8_sel : 1;
        UINT32 bton_gpio9_sel : 1;
        UINT32 bton_gpio10_sel : 1;
        UINT32 bton_gpio11_sel : 1;
        UINT32 bton_gpio12_sel : 1;
        UINT32 bton_gpio13_sel : 1;
        UINT32 bton_gpio14_sel : 1;
        UINT32 bton_gpio15_sel : 1;
        UINT32 bton_gpio16_sel : 1;
        UINT32 bton_gpio17_sel : 1;
        UINT32 bton_gpio18_sel : 1;
        UINT32 bton_gpio19_sel : 1;
        UINT32 bton_gpio20_sel : 1;
        UINT32 gpio0_out_en : 1;
        UINT32 gpio1_out_en : 1;
        UINT32 gpio2_out_en : 1;
        UINT32 gpio3_out_en : 1;
        UINT32 gpio4_out_en : 1;
        UINT32 gpio5_out_en : 1;
        UINT32 gpio6_out_en : 1;
        UINT32 gpio7_out_en : 1;
        UINT32 gpio8_out_en : 1;
        UINT32 gpio9_out_en : 1;
        UINT32 gpio10_out_en : 1;
    };
    UINT32 d32;
} BTON_GPIO_CTRL0_REG;

// REG_BTON_REG_CTRL[31:21] = gpio_e[10:0], gpio output enable
// REG_BTON_REG_CTRL[20:0] = bton_gpio_sel_b, 0: select bton gpio. 1: select off domain gpio
#define REG_BTON_REG_CTRL0 0x48

static inline BTON_GPIO_CTRL0_REG bton_gpio_ctrl0_reg_read()
{
    BTON_GPIO_CTRL0_REG reg = { .d32 = VENDOR_READ(REG_BTON_REG_CTRL0) };
    return reg;
}

static inline void bton_gpio_ctrl0_reg_write(BTON_GPIO_CTRL0_REG reg)
{
    VENDOR_WRITE(REG_BTON_REG_CTRL0, reg.d32);
}


typedef union BTON_GPIO_CTRL1_REG_
{
    struct
    {
        UINT32 gpio11_out_en : 1;
        UINT32 gpio12_out_en : 1;
        UINT32 gpio13_out_en : 1;
        UINT32 gpio14_out_en : 1;
        UINT32 gpio15_out_en : 1;
        UINT32 gpio16_out_en : 1;
        UINT32 gpio17_out_en : 1;
        UINT32 gpio18_out_en : 1;
        UINT32 gpio19_out_en : 1;
        UINT32 gpio20_out_en : 1;
#ifndef _SUPPORT_SEPARATE_BTGPIO_RW_REG_        
        UINT32 gpio0 : 1;
        UINT32 gpio1 : 1;
        UINT32 gpio2 : 1;
        UINT32 gpio3 : 1;
        UINT32 gpio4 : 1;
        UINT32 gpio5 : 1;
        UINT32 gpio6 : 1;
        UINT32 gpio7 : 1;
        UINT32 gpio8 : 1;
        UINT32 gpio9 : 1;
        UINT32 gpio10 : 1;
        UINT32 gpio11 : 1;
        UINT32 gpio12 : 1;
        UINT32 gpio13 : 1;
        UINT32 gpio14 : 1;
        UINT32 gpio15 : 1;
        UINT32 gpio16 : 1;
        UINT32 gpio17 : 1;
        UINT32 gpio18 : 1;
        UINT32 gpio19 : 1;
        UINT32 gpio20 : 1;
#else
        UINT32 gpio0_o : 1;
        UINT32 gpio1_o : 1;
        UINT32 gpio2_o : 1;
        UINT32 gpio3_o : 1;
        UINT32 gpio4_o : 1;
        UINT32 gpio5_o : 1;
        UINT32 gpio6_o : 1;
        UINT32 gpio7_o : 1;
        UINT32 gpio8_o : 1;
        UINT32 gpio9_o : 1;
        UINT32 gpio10_o : 1;
        UINT32 gpio11_o : 1;
        UINT32 gpio12_o : 1;
        UINT32 gpio13_o : 1;
        UINT32 gpio14_o : 1;
        UINT32 gpio15_o : 1;
        UINT32 gpio16_o : 1;
        UINT32 gpio17_o : 1;
        UINT32 gpio18_o : 1;
        UINT32 gpio19_o : 1;
        UINT32 gpio20_o : 1;
#endif
        UINT32 no_use : 1;

    };
    UINT32 d32;
} BTON_GPIO_CTRL1_REG;

#ifndef _SUPPORT_SEPARATE_BTGPIO_RW_REG
// REG_BTON_REG_CTRL[30:10] = gpio_o/gpio_i, Write as gpio output, read as gpio input
// REG_BTON_REG_CTRL[9:0] = gpio_e[20:11], gpio output enable
#else
// REG_BTON_REG_CTRL[30:10] = gpio_o, Write as gpio output, read as what you output
// REG_BTON_REG_CTRL[9:0] = gpio_e[20:11], gpio output enable
#endif
#define REG_BTON_REG_CTRL1 0xB4

static inline BTON_GPIO_CTRL1_REG bton_gpio_ctrl1_reg_read()
{
    BTON_GPIO_CTRL1_REG reg = { .d32 = VENDOR_READ(REG_BTON_REG_CTRL1)};
    return reg;
}

static inline void bton_gpio_ctrl1_reg_write(BTON_GPIO_CTRL1_REG reg)
{
    VENDOR_WRITE(REG_BTON_REG_CTRL1, reg.d32);
}


#ifdef _SUPPORT_SEPARATE_BTGPIO_RW_REG_ 
typedef union BTON_GPIO_CTRL2_REG_
{
    struct
    {
        UINT32 gpio0_i : 1;
        UINT32 gpio1_i : 1;
        UINT32 gpio2_i : 1;
        UINT32 gpio3_i : 1;
        UINT32 gpio4_i : 1;
        UINT32 gpio5_i : 1;
        UINT32 gpio6_i : 1;
        UINT32 gpio7_i : 1;
        UINT32 gpio8_i : 1;
        UINT32 gpio9_i : 1;
        UINT32 gpio10_i : 1;
        UINT32 gpio11_i : 1;
        UINT32 gpio12_i : 1;
        UINT32 gpio13_i : 1;
        UINT32 gpio14_i : 1;
        UINT32 gpio15_i : 1;
        UINT32 gpio16_i : 1;
        UINT32 gpio17_i : 1;
#ifdef _RTL8723D_SPECIFIC_         
        UINT32 gpio18_i : 1;// bit 18
        UINT32 gpio19_i : 1;
        UINT32 gpio20_i : 1;
        UINT32 reserved : 11;
#endif        
#ifdef _RTL8821C_SPECIFIC_        
        UINT32 gpio23_wakeup : 1;
        UINT32 gpio18_i : 1; // bit 19
        UINT32 gpio19_i : 1;
        UINT32 gpio20_i : 1;
        UINT32 reserved : 10;
#endif        

    };
    UINT32 d32;
} BTON_GPIO_CTRL2_REG;

// REG_BTON_REG_CTRL2[20:0] = gpio_i[20:0], gpio input value
#define REG_BTON_REG_CTRL2 0x54

static inline BTON_GPIO_CTRL2_REG bton_gpio_ctrl2_reg_read()
{
    BTON_GPIO_CTRL2_REG reg = { .d32 = VENDOR_READ(REG_BTON_REG_CTRL2)};
    return reg;
}
/*
static inline void bton_gpio_ctrl2_reg_write(BTON_GPIO_CTRL2_REG reg)
{
    VENDOR_WRITE(REG_BTON_REG_CTRL2, reg.d32);
}
*/
#endif



#ifdef _SUPPORT_TIMER1_SHUTDOWN_LOG_
void timer1_shutdown_log();
#endif

void gpio_ctrl_set_bton(UINT32 mask);
void gpio_ctrl_set_off_domain(UINT32 mask);
void gpio_ctrl_set_in_out(UINT32 mask, UINT32 io_flag);
UINT8 gpio_ctrl_read_gpio(UINT8 pos);
void gpio_ctrl_write_gpio(UINT8 pos, UINT8 val);

#define SWITCH_BTON_GPIO_TO_ON_DOMAIN(gpio_map) gpio_ctrl_set_bton(gpio_map)
#if 0
#define SWITCH_BTON_GPIO_TO_ON_DOMAIN(gpio_map) \
{ \
    UINT32 dword; \
    dword = VENDOR_READ(REG_BTON_REG_CTRL0); \
    dword &= (~gpio_map);\
    VENDOR_WRITE(REG_BTON_REG_CTRL0, dword); \
}
#endif

#define SWITCH_BTON_GPIO_TO_OFF_DOMAIN(gpio_map) gpio_ctrl_set_off_domain(gpio_map)
#if 0
#define SWITCH_BTON_GPIO_TO_OFF_DOMAIN(gpio_map) \
{ \
    UINT32 dword; \
    dword = VENDOR_READ(REG_BTON_REG_CTRL0); \
    dword |= gpio_map;\
    VENDOR_WRITE(REG_BTON_REG_CTRL0, dword); \
}
#endif


#define ENABLE_OFF_DOMAIN_LOG_OUTPUT() \
{ \
    UINT32 dword; \
    dword = VENDOR_READ(REG_VEN_GPIO_CTRL); \
    dword |= BIT29; \
    VENDOR_WRITE(REG_VEN_GPIO_CTRL, dword); \
}

#define DISABLE_OFF_DOMAIN_LOG_OUTPUT() \
{ \
    UINT32 dword; \
    dword = VENDOR_READ(REG_VEN_GPIO_CTRL); \
    dword &= (~BIT29); \
    VENDOR_WRITE(REG_VEN_GPIO_CTRL, dword); \
}



#define BTON_GPIOMAP_00 BIT0
#define BTON_GPIOMAP_01 BIT1
#define BTON_GPIOMAP_02 BIT2
#define BTON_GPIOMAP_03 BIT3
#define BTON_GPIOMAP_04 BIT4
#define BTON_GPIOMAP_05 BIT5
#define BTON_GPIOMAP_06 BIT6
#define BTON_GPIOMAP_07 BIT7
#define BTON_GPIOMAP_08 BIT8
#define BTON_GPIOMAP_09 BIT9
#define BTON_GPIOMAP_10 BIT10
#define BTON_GPIOMAP_11 BIT11
#define BTON_GPIOMAP_12 BIT12
#define BTON_GPIOMAP_13 BIT13
#define BTON_GPIOMAP_14 BIT14
#define BTON_GPIOMAP_15 BIT15
#define BTON_GPIOMAP_16 BIT16
#define BTON_GPIOMAP_17 BIT17
#define BTON_GPIOMAP_18 BIT18
#define BTON_GPIOMAP_19 BIT19
#define BTON_GPIOMAP_20 BIT20

typedef struct EFUSE_BTON_GPIO_SETTING_0_ {
    // not used, i will remove it
    UINT8 enable_gpio8_as_log_when_power_on:1;           // [0] enable bton gpio8 as bt log function, gpio switch to off domain 
    UINT8 rsv:7;
} EFUSE_BTON_GPIO_SETTING_0;

/*========== section:  Bt on gpio [21:0] end ===========*/


/*========== section:  vendor gpio [8:0] start ===========*/
#define VND_GPIO_CTRL 0x38  // Vendor GPIO Control Register
#define VND_GPIO_INT 0x3C  // Vendor GPIO Interrupt Register



typedef union BTOFF_VEN_GPIO_CTRL_REG_
{
    struct
    {
        UINT32 Reg_gpio3_0_sel  : 2; // valid after 8703B
        UINT32 Loguart_sel      : 1;     // valid after 8703B
        UINT32 loguart_sel_1    : 1;   // valid after 8723D
        UINT32 rsvd             : 1;
        UINT32 ven_gpio8_oe  : 1;
        UINT32 ven_gpio8_o   : 1;
        UINT32 ven_gpio8_i   : 1; //b24
        UINT32 ven_gpio7_oe  : 1;
        UINT32 ven_gpio6_oe  : 1;
        UINT32 ven_gpio5_oe  : 1;
        UINT32 ven_gpio4_oe  : 1;
        UINT32 ven_gpio3_oe  : 1;
        UINT32 ven_gpio2_oe  : 1;
        UINT32 ven_gpio1_oe  : 1;
        UINT32 ven_gpio0_oe  : 1; //b16
        UINT32 ven_gpio7_o   : 1;
        UINT32 ven_gpio6_o   : 1;
        UINT32 ven_gpio5_o   : 1;
        UINT32 ven_gpio4_o   : 1;
        UINT32 ven_gpio3_o   : 1;
        UINT32 ven_gpio2_o   : 1;
        UINT32 ven_gpio1_o   : 1;
        UINT32 ven_gpio0_o   : 1; // BIT8
        UINT32 ven_gpio7_i   : 1;
        UINT32 ven_gpio6_i   : 1;
        UINT32 ven_gpio5_i   : 1;
        UINT32 ven_gpio4_i   : 1;
        UINT32 ven_gpio3_i   : 1;
        UINT32 ven_gpio2_i   : 1;
        UINT32 ven_gpio1_i   : 1;        
        UINT32 ven_gpio0_i   : 1; // BIT0

    };
    UINT32 d32;
} BTOFF_VEN_GPIO_CTRL_REG;

typedef union BTOFF_VEN_GPIO_INT_REG_
{
    struct
    {
        UINT32 ven_gpio7_int_edge   : 1; // change interrupt edge, 
        UINT32 ven_gpio6_int_edge   : 1; // 1 = high level or rising edge trigger, 0 = low level or falling edge trigger
        UINT32 ven_gpio5_int_edge   : 1; 
        UINT32 ven_gpio4_int_edge   : 1;
        UINT32 ven_gpio3_int_edge   : 1;
        UINT32 ven_gpio2_int_edge   : 1;
        UINT32 ven_gpio1_int_edge   : 1;
        UINT32 ven_gpio0_int_edge   : 1;
        UINT32 ven_gpio7_int_mode   : 1; // 1 = level trigger, 0 = edge trigger
        UINT32 ven_gpio6_int_mode   : 1;
        UINT32 ven_gpio5_int_mode   : 1;
        UINT32 ven_gpio4_int_mode   : 1;
        UINT32 ven_gpio3_int_mode   : 1;
        UINT32 ven_gpio2_int_mode   : 1;
        UINT32 ven_gpio1_int_mode   : 1;
        UINT32 ven_gpio0_int_mode   : 1; //b16
        UINT32 ven_gpio7_int_en     : 1;    //change interrupt enable
        UINT32 ven_gpio6_int_en     : 1;//i2c_interrupt, 8723d
        UINT32 ven_gpio5_int_en     : 1;
        UINT32 ven_gpio4_int_en     : 1;
        UINT32 ven_gpio3_int_en     : 1;
        UINT32 ven_gpio2_int_en     : 1; 
        UINT32 ven_gpio1_int_en     : 1;
        UINT32 ven_gpio0_int_en     : 1; // BIT8
        UINT32 ven_gpio7_int_sts    : 1; // change interrupt status. Write 1 clear
        UINT32 ven_gpio6_int_sts    : 1;
        UINT32 ven_gpio5_int_sts    : 1;
        UINT32 ven_gpio4_int_sts    : 1;
        UINT32 ven_gpio3_int_sts    : 1;
        UINT32 ven_gpio2_int_sts    : 1;
        UINT32 ven_gpio1_int_sts    : 1;        
        UINT32 ven_gpio0_int_sts    : 1; // BIT0

    };
    UINT32 d32;
} BTOFF_VEN_GPIO_REG_INT;



/*========== section:  vendor gpio [8:0] end ===========*/


