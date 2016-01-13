/***************************************************************************
 Copyright (C) MindTree Ltd.
 This module is a confidential and proprietary property of MindTree and
 a possession or use of this module requires written permission of MindTree.
 ***************************************************************************/

#ifndef _PLATFORM_H_
#define _PLATFORM_H_

/**
 * \file platform.h
 *  Platform interface (which meets the 'BlueWiz Reference Platform Interface')
 *  implemented by S3C2410 platform.
 *
 * \author Santhosh kumar M
 * \date 2006-09-15
 */

#include "compiler.h"
#include "common_utils.h"
#include "DataType.h"
#include "rlx4181.h"
#include "FreeRTOS.h"
#include "task.h"

#include "bt_fw_types.h"

/** Imported interfaces are marked with this Macro */
#define PF_IMPORT
/** Exported interfaces are marked with this Macro */
#define PF_EXPORT

extern UINT32 cpu_clk;

/* Masks off all bits but the VECTACTIVE bits in the ICSR register. */
#define portVECTACTIVE_MASK (0xFFUL)
#define IN_ISR()    ((portNVIC_INT_CTRL_REG & portVECTACTIVE_MASK) != 0)


/* ================== C Prefix Setting MACRO ========================== */
#define SECTION(_name)          __attribute__ ((__section__(_name)))
#define _PACKED_                __attribute__ ((packed))
#define MIPS16                  __attribute__((mips16))
#define MIPS32                  __attribute__((nomips16))
#define FAR_CODE_MIPS16         __attribute__((far_code, mips16))
#define FAR_CODE                __attribute__((far_code, nomips16))

#define SECTION_SRAM            SECTION(".bss.sram")
#define SECTION_HEAP            SECTION(".heapsection")

#ifdef _ENABLE_RETENTION_FLOW_FOR_DLPS_
#define SECTION_LOW_HEAP        SECTION(".low_heapsection")
#define SECTION_LOW_BSS         SECTION(".low_bss")
#else
#define SECTION_LOW_HEAP
#define SECTION_LOW_BSS
#endif

#if defined(SPI_FLASH_BOOT)
    #if defined(SPI_CRITICAL_CODE_IN_SRAM)
#define SECTION_ISR             SECTION(".isr.text")
#define SECTION_LE_ISR
#define SECTION_ISR_LOW
    #else
#define SECTION_ISR
#define SECTION_LE_ISR
#define SECTION_ISR_LOW
    #endif
#else
#define SECTION_ISR             SECTION(".isr.text")
#define SECTION_LE_ISR          SECTION(".isr.text")
#define SECTION_ISR_LOW         SECTION(".isr.text")
#endif

#ifdef _ROM_CODE_PATCHED_
/* Section Settings about .text region of Patch Code */
#define SECTION_PATCH_TEXT_HEAD SECTION(".rom_patch_head_text")
#define SECTION_PATCH_TEXT      SECTION(".rom_patch_text")
#define SECTION_PATCH_DRO       SECTION(".rom_patch_rodata")
#define SECTION_PATCH_DRO_SVN   SECTION(".rodata1_svn")
#define SECTION_PATCH_DRO_TAIL  SECTION(".rodata1_tail")

/* Section Settings about .bss region of Patch Code in DMEM */
#define SECTION_PATCH_DBSS      SECTION(".rom_patch_dmem_bss")
#define SECTION_PATCH_LOW_DBSS  SECTION(".rom_patch_dmem_low_bss")

/* Section Settings about .bss region of Patch Code in SRAM */
#define SECTION_PATCH_SBSS_PRAM  SECTION(".rom_patch_sram_bss_pram")
#define SECTION_PATCH_SBSS       SECTION(".rom_patch_sram_bss")
#define SECTION_PATCH_LOW_SBSS   SECTION(".rom_patch_sram_low_bss")
#else
#define SECTION_PATCH_TEXT_HEAD
#define SECTION_PATCH_TEXT
#define SECTION_PATCH_DRO
#define SECTION_PATCH_DRO_SVN
#define SECTION_PATCH_DRO_TAIL
#define SECTION_PATCH_DBSS
#define SECTION_PATCH_LOW_DBSS
#define SECTION_PATCH_SBSS_PRAM  SECTION(".bss.sram")
#define SECTION_PATCH_SBSS       SECTION(".bss.sram")
#define SECTION_PATCH_LOW_SBSS   SECTION(".bss.sram")
#endif

/* ========================== Timer Section ============================ */
#ifdef TIMER_TICK_10_MSEC
#define PF_TIMER_TICKS_PER_SEC      100     /* once in every 10 ms */
#else
#define PF_TIMER_TICKS_PER_SEC      1000    /* once in every ms */
#endif

#define PF_TIMER_TICKS_PER_1000MS    1
#define PF_TIMER_TICKS_PER_100MS      10
#define PF_TIMER_TICKS_PER_10MS       100
#define PF_TIMER_TICKS_PER_MS           1000

#ifdef _CCH_RETENTION_FLOW_FOR_DLPS_
void pf_timer_init(UINT8 dlps_flow) PF_EXPORT;
#else
void pf_timer_init(void) PF_EXPORT;
#endif
void timer_var_init() PF_EXPORT;
void timer_on_off(UINT8 timer_id, UINT32 usec, UINT8 is_on) PF_EXPORT;
void timer_update_expired_time(UINT8 timer_id, UINT32 usec) PF_EXPORT;

#ifndef IS_BTSOC
void pf_timer_handle_tick(void) PF_IMPORT;  // implement in mint_os_timer.c
                                            // send signal to timer task
#endif

typedef void (*PF_ISR_EXT_BOTTOM_HALF_FUNC)(void *argu);


/* ======================= Miscellaneous Section ======================== */
void pf_delay_us(UINT32 us) PF_EXPORT;
void pf_delay(UINT32 ms) PF_EXPORT;
UINT32 pf_get_rng_seed(void) PF_EXPORT;

#define pf_define_critical      uint32_t cpu_sr = 0; /* init by austin */
#define pf_enter_critical()     do { cpu_sr = taskENTER_CRITICAL_FROM_ISR(); barrier(); } while (0)
#define pf_exit_critical()      do { barrier(); taskEXIT_CRITICAL_FROM_ISR(cpu_sr); } while (0)

/* ========================== PF_TASK Section =========================== */
/* "pf_task" is required to implement S3C2410 UART and USB driver. So enabling
 * it.
 */
#ifndef _DONT_USE_LOG_UART_TX_INT_
#define ENABLE_PLATFORM_TASK_HOOK
#endif

#ifdef ENABLE_PLATFORM_TASK_HOOK
BOOLEAN pf_task_send_signal(INT32 signal, void* arg);
void pf_task(INT32 signal, void* arg);
#endif

#ifdef _CP3_COUNTER_SUPPORTED_
void reset_cp3_counter(void);
void stop_cp3_counter(UINT8 index);
#endif

/* ========================= Transport Section ========================= */
typedef enum
{
    HCI_TRANSPORT_CMD_PKT_TYPE=0x1,
	HCI_TRANSPORT_ACL_DATA_PKT_TYPE,
    HCI_TRANSPORT_SYNC_DATA_PKT_TYPE,
	HCI_TRANSPORT_EVENT_PKT_TYPE,
	HCI_TRANSPORT_LE_ACL_DATA_PKT_TYPE, /* proprietary definition, only use in
	                                       FW processing flow - austin */
} HCI_TRANSPORT_PKT_TYPE;

typedef void (*PF_TP_INIT_COMPLETED_CB)(void);

typedef void (*PF_TP_PKT_RECD_CB)(HCI_TRANSPORT_PKT_TYPE pkt_type,
        UCHAR* buf, UINT16 len, UINT32 fifo_index);

typedef void (*PF_TP_BYTES_RECD_CB)(UCHAR* buf, UINT16 len);

typedef void (*PF_UART_DBG_RECD_CB) (UCHAR* buf, UINT16 len);

typedef void (*PF_TP_TX_COMPLETED_CB)(UCHAR* buf, UINT16 len, UINT8 type);

typedef BOOLEAN (*PF_HCI_TRANSPORT_WRITE_FN)(HCI_TRANSPORT_PKT_TYPE type,
                                               UCHAR* buf, UINT16 len, UINT32 free_index);
typedef void(*PF_HCI_ISO_OUT_EP_EN_FN)(UINT8 enable);
typedef void (*PF_SWITCH_HCI_DMA_PARAMTER_FN)(UINT8 mode);
typedef void (*PF_INTERFACE_WAKE_UP_INTERRUPT_FN)(UINT8 mode);
typedef void (*PF_ENABLE_USB_INTERFACE_SCO_FILTER_FN)(UINT8 sco_index,
              UINT8 is_enable, UINT8 con_handle, UINT8 mode);
//====================== HCI DMA  	============================
/*
 * The platform provide this founction to free hardware DMA fifo.
 ------------------------------------------------------------------*/
typedef void (*PF_TX_FIFO_ORDER_FREE_FN) (UINT16 free_bitmap, UINT8 type);
extern PF_TX_FIFO_ORDER_FREE_FN pf_tx_fifo_order_free;
#define 	ACL_BUF_TYPE       0x1
#define 	SCO_BUF_TYPE       0x2
#define 	CMD_BUF_TYPE       0x3

void pf_hci_transport_init(void) PF_EXPORT;

extern PF_HCI_TRANSPORT_WRITE_FN pf_hci_transport_write;
extern PF_SWITCH_HCI_DMA_PARAMTER_FN pf_switch_hci_dma_parameter;
extern PF_INTERFACE_WAKE_UP_INTERRUPT_FN pf_interface_wake_up_interrupt;
extern PF_ENABLE_USB_INTERFACE_SCO_FILTER_FN pf_enable_usb_interface_sco_filter;

#define PF_HCI_TRANSPORT_HW_ERROR_SYNC_LOST                  0x01
#define PF_HCI_TRANSPORT_HW_ERROR_WRONG_PARAM_FOR_ACL_DATA   0x02
#define PF_HCI_TRANSPORT_HW_ERROR_WRONG_PARAM_FOR_SYNC_DATA  0X03
#define PF_HCI_TRANSPORT_HW_ERROR_NO_RESOURCE_AVAILABLE      0X04

//void pf_hci_transport_indicate_error(UCHAR error_code) PF_IMPORT;

SECTION_ISR_LOW void USB_DMA_IntrHandler();
SECTION_ISR_LOW void UartIntrHandler(void);

#ifdef ENABLE_LOGGER
void pf_logger_transport_init(PF_TP_INIT_COMPLETED_CB init_completed_cb,
        PF_TP_TX_COMPLETED_CB tx_completed_cb,
        PF_UART_DBG_RECD_CB uart_dbg_recd_cb) PF_EXPORT;
void pf_os_trigger_logger_task(void) PF_EXPORT;
void pf_os_trigger_uar_dbg_task(void) PF_EXPORT;
void pf_service_logger(void) PF_IMPORT;
#endif  //ENABLE_LOGGER

/* =========================== BB_DRIVER Section ======================== */
#define BB_BASE_ADDR    0x40050000
#define BB_REG_ADDR(offset) (BB_BASE_ADDR + (offset))
SECTION_ISR void baseband_interrupt_handler(void);

#define BT_SOC_VENDOR_BASE_ADDR     0x40006000
#define BT_SOC_VENDOR_ADDR(offs)    (BT_SOC_VENDOR_BASE_ADDR + (offs))

typedef struct BTMAC_ISR_STATUS_REG_
{
    UINT32 timer_0_1_int : 1;
    UINT32 bt_bluewiz_int : 1;
    UINT32 bt_hcidma_int : 1;
    UINT32 bt_bzdma_int : 1;
    UINT32 bt_crossbar_int : 1;
    UINT32 reserved : 27;
} BTMAC_ISR_STATUS_REG;
#define BTMAC_ISR_STATUS_REG_ADDR   BT_SOC_VENDOR_ADDR(0x10)

__STATIC_INLINE BTMAC_ISR_STATUS_REG btmac_isr_status_reg_read(void)
{
    return RD_REG_MMIO(BTMAC_ISR_STATUS_REG, BTMAC_ISR_STATUS_REG_ADDR);
}

__STATIC_INLINE void btmac_isr_status_reg_write(BTMAC_ISR_STATUS_REG reg)
{
    WR_REG_MMIO(BTMAC_ISR_STATUS_REG, BTMAC_ISR_STATUS_REG_ADDR, reg);
}

/* =========================== GPIO section ===========================*/
void GpioInit() PF_EXPORT;
#define GPIO_BASE_ADDRESS 0x40001000
#define GPIO_READ(GPIO_offset) \
	((UINT32)*((volatile UINT32*)(GPIO_BASE_ADDRESS+(GPIO_offset))))
#define GPIO_WRITE(GPIO_offset, Value) \
	((*((volatile UINT32*)(GPIO_BASE_ADDRESS + (GPIO_offset)))) = (Value))


#ifdef MINICARD_BT_LED_CONTROL
void bt_led_control(UINT8 id, UINT8 level);
#if defined (_FIX_8703B_LED_SHARE_WITH_WIFI_)&&defined(_SUPPORT_INFO_FROM_SYSTEM_ON_)
void bt_wifi_share_led_8703b_eco(UINT8 id);
#endif

//#define OUTPUT_LOW      0       /* output low */
//#define OUTPUT_HIGH     1       /* output high */
//#define OUTPUT_HI_Z     2       /* output Hi-Z or input */

enum{   OUTPUT_LOW = 0,     /* output low */
        OUTPUT_HIGH = 1,    /* output high */
        OUTPUT_HI_Z = 2};   /* output Hi-Z or input */

extern UINT8 g_wpan_led_num;

#define BT_LED_WPAN_INIT() \
{ \
    if (IS_SUPPORT_WPAN_LED) \
    { \
        g_wpan_led_num = (IS_WPAN_LED_USE_LED0) ? 0 : 1; \
    } \
}

#define BT_LED_WPAN_ON() \
{ \
    if (IS_SUPPORT_WPAN_LED) \
    { \
        bt_led_control(g_wpan_led_num, OUTPUT_LOW); \
    } \
}

#define BT_LED_WPAN_OFF() \
{ \
    if (IS_SUPPORT_WPAN_LED) \
    { \
        bt_led_control(g_wpan_led_num, OUTPUT_HI_Z); \
    } \
}
#endif

/* ========================== CLK32K section ===========================*/
#ifdef _GPIO_POWER_SEQUENCE_ENABLE
#define MODE_40M    1
#define MODE_32K    2
#if (defined(_ENABLE_32K_CLK_WAKE_UP_ISR_) || defined(_GPIO_POWER_SEQUENCE_ENABLE))
extern UINT16 power_ctr_reg;
extern UINT32 sys_clk_reg;
#endif
#endif

/* =========================== vendor section ===========================*/
/* note: vender reg can be access by 1, 2, 4 byte */
/* note: should be 1-byte aligned, 2-byte aligned, 4-byte aligned */
#define VENDOR_BASE_ADDRESS 0x40058000
#define VENDOR_REG_ADDR(offset) (VENDOR_BASE_ADDRESS + (offset))
#define VENDOR_READ(Vendor_offset) \
	((UINT32)*((volatile UINT32*)(VENDOR_BASE_ADDRESS+(Vendor_offset))))
#define VENDOR_WRITE(Vendor_offset, Value) \
	((*((volatile UINT32*)(VENDOR_BASE_ADDRESS + (Vendor_offset)))) = (Value))
#define VENDOR_BYTE_READ(Vendor_offset) \
	(*((volatile UINT8*)(VENDOR_BASE_ADDRESS+(Vendor_offset))))
#define VENDOR_BYTE_WRITE(Vendor_offset, Value) \
	((*((volatile UINT8*)(VENDOR_BASE_ADDRESS + (Vendor_offset)))) = (Value))
#define VENDOR_U32_REG_UPDATE(Vendor_offset, Mask, Value) \
    {UINT32 temp; \
     temp = VENDOR_READ(Vendor_offset) & ~(Mask);\
     VENDOR_WRITE(Vendor_offset, (temp | ((Value) & (Mask))));}
#define VENDOR_U8_REG_UPDATE(Vendor_offset, Mask, Value) \
    {UINT8 temp; \
     temp = VENDOR_BYTE_READ(Vendor_offset) & ~(Mask);\
     VENDOR_BYTE_WRITE(Vendor_offset, (temp | ((Value) & (Mask))));}

/* ================ Baseband clock Control ============================== */
#ifdef POWER_SAVE_FEATURE
#include "S3C2410_bt_power_ctrl.h"
#define pf_set_dsm_exit()       S3C2410_set_dsm_exit()
#define pf_clear_dsm_exit()     S3C2410_clear_dsm_exit()
#else
#define pf_set_dsm_exit()
#define pf_clear_dsm_exit()
#endif /* POWER_SAVE_FEATURE */

/* ====================== Debug Utilities Section ========================= */
/** @} end: bzrp_leds_section */

/* ====================== Rom Code Patch Section  ======================== */
extern UINT8 __rom_sram_start_addr;
#define ROM_CODE_PAGE_START_ADDRESS   ((UINT32)(&__rom_sram_start_addr))

#ifndef _MOVE_SOME_FW_SRAM_SIGNATURES_TO_BTON_REG_
extern SECTION_SRAM UINT32 g_sram_rom_code_patch;
extern SECTION_SRAM UINT32 g_sram_patch_end_event;
extern SECTION_SRAM UINT32 g_sram_check_wdg_to_reason;
#endif

#ifdef _YL_RTL8723A_B_CUT
extern SECTION_SRAM UINT32 g_sram_patch_uart_status;
#endif

#ifndef _MOVE_SOME_FW_SRAM_SIGNATURES_TO_BTON_REG_
extern SECTION_SRAM UINT32 g_sram_check_h5_linkreset;

#define SRAM_ADDR_ROM_CODE_PATCH            &g_sram_rom_code_patch
#define SRAM_ADDR_PATCH_END_EVENT           &g_sram_patch_end_event
#define SRAM_ADDR_CHECK_WDG_TO_REASON       &g_sram_check_wdg_to_reason
#define SRAM_ADDR_CHECK_H5_LINKRESET        &g_sram_check_h5_linkreset
#endif

#ifdef _YL_RTL8723A_B_CUT
#define SRAM_ADDR_PATCH_UART_STATUS         &g_sram_patch_uart_status
#endif

#define ROM_CODE_PATCH_SIGNATURE            0x548723AE
#define PATCH_END_EVENT_SIGNATURE           0x00C0FFEE
#define FW_TRIG_WDG_TO_SIGNATURE            0x23799732
#define H5_LINKRESET_SIGNATURE              0x937A3B58

#ifdef _YL_RTL8723A_B_CUT
#define SIGN_PATCH_UART_STATUS(uart_status_d32) \
    {*(UINT32*)SRAM_ADDR_PATCH_UART_STATUS = uart_status_d32;}
#endif

#ifndef _MOVE_SOME_FW_SRAM_SIGNATURES_TO_BTON_REG_
#define SIGN_ROM_CODE_PATCH_SIGNATURE \
    {(*(UINT32*)SRAM_ADDR_ROM_CODE_PATCH) = ROM_CODE_PATCH_SIGNATURE;}

#define ERASE_ROM_CODE_PATCH_SIGNATURE \
    {*(UINT32*)SRAM_ADDR_ROM_CODE_PATCH = 0;}

#define IS_ROM_CODE_PATCHED \
    (*(UINT32*)SRAM_ADDR_ROM_CODE_PATCH == ROM_CODE_PATCH_SIGNATURE)

#define SIGN_PATCH_END_EVENT_SIGNATURE(index) \
    {*(UINT32*)SRAM_ADDR_PATCH_END_EVENT = ((index) << 24) | PATCH_END_EVENT_SIGNATURE;}

#define ERASE_PATCH_END_EVENT_SIGNATURE \
    {*(UINT32*)SRAM_ADDR_PATCH_END_EVENT = 0;}

#define IS_SEND_PATCH_END \
    (((*(UINT32*)SRAM_ADDR_PATCH_END_EVENT) & 0xFFFFFF) == PATCH_END_EVENT_SIGNATURE)

#define GET_PATCH_END_INDEX \
    ((*(UINT32*)SRAM_ADDR_PATCH_END_EVENT) >> 24)

#define SIGN_FW_TRIG_WDG_TO_SIGNATURE \
    {*(UINT32*)SRAM_ADDR_CHECK_WDG_TO_REASON = FW_TRIG_WDG_TO_SIGNATURE;}

#define ERASE_FW_TRIG_WDG_TO_SIGNATURE \
    {*(UINT32*)SRAM_ADDR_CHECK_WDG_TO_REASON = 0;}

#define IS_FW_TRIG_WDG_TO \
    (*(UINT32*)SRAM_ADDR_CHECK_WDG_TO_REASON == FW_TRIG_WDG_TO_SIGNATURE)

#define SIGN_H5_LINKRESET_SIGNATURE \
    {*(UINT32*)SRAM_ADDR_CHECK_H5_LINKRESET = H5_LINKRESET_SIGNATURE;}

#define ERASE_H5_LINKRESET_SIGNATURE \
    {*(UINT32*)SRAM_ADDR_CHECK_H5_LINKRESET = 0;}

#define IS_H5_LINKRESET \
    (*(UINT32*)SRAM_ADDR_CHECK_H5_LINKRESET == H5_LINKRESET_SIGNATURE)
#else

/* struct: BTON_A0_REG_S */

#define SIGN_ENTER_DLPS_MODE_SIGNATURE \
        VENDOR_U32_REG_UPDATE(0xA0, BIT0, BIT0)  /* Sign Enter DLPS (Auto Clear by fw)*/

#define SIGN_ROM_CODE_PATCH_SIGNATURE \
        VENDOR_U32_REG_UPDATE(0xA0, BIT1, BIT1)

#define ERASE_ROM_CODE_PATCH_SIGNATURE \
        VENDOR_U32_REG_UPDATE(0xA0, BIT1, 0)

#define IS_ROM_CODE_PATCHED \
        (VENDOR_READ(0xA0) & BIT1)

#define SIGN_PATCH_END_EVENT_SIGNATURE(index) \
        VENDOR_U32_REG_UPDATE(0xA0, 0xff10, ((index) << 8) | BIT4)

#define ERASE_PATCH_END_EVENT_SIGNATURE \
        VENDOR_U32_REG_UPDATE(0xA0, BIT4, 0)

#define IS_SEND_PATCH_END \
        (VENDOR_READ(0xA0) & BIT4)

#define GET_PATCH_END_INDEX \
        ((VENDOR_READ(0xA0) >> 8) & 0xff)

#define SIGN_FW_TRIG_WDG_TO_SIGNATURE \
        VENDOR_U32_REG_UPDATE(0xA0, BIT2, BIT2)

#define ERASE_FW_TRIG_WDG_TO_SIGNATURE \
        VENDOR_U32_REG_UPDATE(0xA0, BIT2, 0)

#define IS_FW_TRIG_WDG_TO \
        (VENDOR_READ(0xA0) & BIT2)

#ifdef _BRUCE_WDG_TIMEOUT_DEBUG
#define IS_PWL_ON \
            (VENDOR_READ(0xA0) & BIT5)
#define SIGN_PWL_ON_SIGNATURE \
            VENDOR_U32_REG_UPDATE(0xA0, BIT5, BIT5)
#endif

#define SIGN_H5_LINKRESET_SIGNATURE \
        VENDOR_U32_REG_UPDATE(0xA0, BIT3, BIT3)

#define ERASE_H5_LINKRESET_SIGNATURE \
        VENDOR_U32_REG_UPDATE(0xA0, BIT3, 0)

#define IS_H5_LINKRESET \
        (VENDOR_READ(0xA0) & BIT3)

#define SIGN_ENTER_LPS_MODE_FLAG \
        VENDOR_U32_REG_UPDATE(0xA0, BIT6, BIT6)    /* Sign Enter LPS for Debug (No Clear by fw) */

#define SIGN_ENTER_DLPS_MODE_FLAG \
        VENDOR_U32_REG_UPDATE(0xA0, BIT7, BIT7)    /* Sign Enter DLPS for Debug (No Clear by fw)*/
#endif

#ifdef _ROM_CODE_PATCHED_

typedef void (*PF_ROM_CODE_PATCH_VOID)(void);
typedef UINT8 (*PF_ROM_CODE_PATCH_FUNC)(void *buf, ...);

//bzdma.c
#ifdef _ROM_CODE_PATCHED_
extern PF_ROM_CODE_PATCH_FUNC rcp_bzdma_update_fw_rptr_of_ble_data_ring_fifo;
#endif

extern PF_ROM_CODE_PATCH_VOID rcp_rf_iqk_func;
extern PF_ROM_CODE_PATCH_FUNC rcp_hci_vendor_cmd_func;
//extern PF_ROM_CODE_PATCH_FUNC rcp_hci_vendor_host_enter_sleep_mode;
extern PF_ROM_CODE_PATCH_FUNC rcp_hci_handle_vendor_write_bb_reg_cmd;
extern PF_ROM_CODE_PATCH_FUNC rcp_hci_vendor_generate_cmd_complete_func;
extern PF_ROM_CODE_PATCH_FUNC rcp_ll_hit_conn_adv_isr_func;
extern PF_ROM_CODE_PATCH_VOID rcp_hci_le_adjust_conn_req_func;

//vector.c
extern PF_ROM_CODE_PATCH_FUNC rcp_bt_interrupt_handler;

//lmp_pdu_q.c
extern PF_ROM_CODE_PATCH_FUNC rcp_pduq_get_next_active_pdu;

//power_control.c
extern PF_ROM_CODE_PATCH_FUNC rcp_execute_lps_mode_procedure;
extern PF_ROM_CODE_PATCH_FUNC rcp_pow_ctrl_intr_handle;
extern PF_ROM_CODE_PATCH_VOID rcp_enter_lps_mode;
extern PF_ROM_CODE_PATCH_VOID rcp_power_init;
extern PF_ROM_CODE_PATCH_FUNC rcp_execute_wakeup_procedure;
extern PF_ROM_CODE_PATCH_VOID rcp_wake_up_host;
#ifndef _8821A_BTON_DESIGN_
extern PF_ROM_CODE_PATCH_FUNC rcp_trun_on_off_afe_ldo;
#else
extern PF_ROM_CODE_PATCH_FUNC rcp_execute_bton_entering_pdn_sus; // yilinli
#endif
#ifdef _CCH_8723_RCP_
extern PF_ROM_CODE_PATCH_FUNC rcp_pow_timer_lps;
#endif
#ifdef _CCH_8821_RCP_
extern PF_ROM_CODE_PATCH_VOID rcp_bton_32k_cal_en;
extern PF_ROM_CODE_PATCH_FUNC rcp_bton_32k_cal_chk_lock;
extern PF_ROM_CODE_PATCH_FUNC rcp_lps_handle_timer2_mode_sel1;
#endif

#ifdef _CCH_2801_RCP_
extern PF_ROM_CODE_PATCH_VOID rcp_bton_32k_cal_ini;
#endif

//gpio.c
extern PF_ROM_CODE_PATCH_VOID rcp_GpioInit;
extern PF_ROM_CODE_PATCH_FUNC rcp_GpioIntrHandler;
extern PF_ROM_CODE_PATCH_FUNC rcp_process_power_gpio_set;
extern PF_ROM_CODE_PATCH_FUNC rcp_gpio_power_on_check;
extern PF_ROM_CODE_PATCH_FUNC rcp_gpio_wake_up_host;
#ifdef _8821A_BTON_DESIGN_
extern PF_ROM_CODE_PATCH_FUNC rcp_set_gpio_bt_wake_host; // yilinli, RTL8821A
extern PF_ROM_CODE_PATCH_FUNC rcp_get_gpio_bt_wake_host; // yilinli, RTL8821A
extern PF_ROM_CODE_PATCH_FUNC rcp_gpio_wake_up_timer_callback_func; // yilinli
#else
extern PF_ROM_CODE_PATCH_FUNC rcp_get_wake_up_pin;
#endif
#ifdef _DAPE_8723_RCP_
//extern PF_ROM_CODE_PATCH_VOID rcp_usb_remote_wakeup_gpio11_recv_power_dn_func;
//extern PF_ROM_CODE_PATCH_VOID rcp_usb_remote_wakeup_gpio11_recv_suspend_func;
#endif
#ifdef _YL_RTL8723A_B_CUT
extern PF_ROM_CODE_PATCH_FUNC rcp_gpio_power_on_check_end_func; // yilinli
//extern PF_ROM_CODE_PATCH_FUNC rcp_gpiointr_usb_suspend_func; //yilinli
#endif

//mailbox.c
extern PF_ROM_CODE_PATCH_FUNC rcp_mailbox_handle_command;
extern PF_ROM_CODE_PATCH_FUNC rcp_mailbox_interrupt_handler;
extern PF_ROM_CODE_PATCH_FUNC rcp_mailbox_handle_mp_extra_command;

//pta.c
//extern PF_ROM_CODE_PATCH_FUNC rcp_pta_profile_manage;

//main.c
extern PF_ROM_CODE_PATCH_VOID rcp_main_hci_uart_init_func;  //yilinli
extern PF_ROM_CODE_PATCH_VOID rcp_hw_reg_reinit_func;

//bt_fw_hci_hcbb_info.c
#ifdef _YL_RTL8723A_B_CUT
extern PF_ROM_CODE_PATCH_FUNC rcp_hci_handle_reset_command_func; // yilinli
#endif
extern PF_ROM_CODE_PATCH_VOID rcp_hci_reset_dma_init_func; // yilinli, RTL8821A

//bt_fw_hci_cmds.c
#ifdef _CCH_8723_RCP_
extern PF_ROM_CODE_PATCH_FUNC rcp_hci_handle_sniff_mode_command_get_slot; // cch
#endif
#ifdef _DAPE_8723_RCP_
extern PF_ROM_CODE_PATCH_FUNC rcp_hci_handle_accept_conn_func;
#endif

//bt_fw_hci_tasks.c
#ifndef _REDUCE_LPS_AFTER_RTL8703B_
#ifdef _CCH_8723_RCP_
extern PF_ROM_CODE_PATCH_FUNC rcp_hci_handle_remote_name_request_cancel_command; // cch
extern PF_ROM_CODE_PATCH_FUNC rcp_hci_execute_link_control_command_packet_case; // cch
#endif
#endif
extern PF_ROM_CODE_PATCH_FUNC rcp_hci_command_handle; //austin

//lmp.c
#ifdef _CCH_8723_RCP_
extern PF_ROM_CODE_PATCH_FUNC rcp_lmp_exit_sniff_mode;// cch
#endif
#ifdef _CCH_8821_RCP_
extern PF_ROM_CODE_PATCH_FUNC rcp_lmp_release_am_addr_ppi;
#endif


//lmp_afh.c
#ifdef _DAPE_8723_RCP_
extern PF_ROM_CODE_PATCH_FUNC rcp_lmp_handle_afh_instant_when_master_func;
#endif

//lmp_utils.c
#ifdef _CCH_8723_RCP_
extern PF_ROM_CODE_PATCH_FUNC rcp_lmp_validate_sniff_parms;  // cch
extern PF_ROM_CODE_PATCH_FUNC rcp_lmp_validate_sniff_parms_1;  // cch
extern PF_ROM_CODE_PATCH_FUNC rcp_lmp_get_global_slot_offset;  // cch
extern PF_ROM_CODE_PATCH_FUNC rcp_lmp_update_global_slot_offset;  // cch
#endif


#ifdef _CCH_8821_RCP_
extern PF_ROM_CODE_PATCH_VOID rcp_lmp_stop_regular_sw_timers;
extern PF_ROM_CODE_PATCH_VOID rcp_lmp_start_regular_sw_timers;
#endif

//lmp_sco.c
#ifdef _CCH_8723_RCP_
extern PF_ROM_CODE_PATCH_FUNC rcp_lmp_generate_sco_params_rej; // cch
#ifndef _REDUCE_LPS_AFTER_RTL8703B_
extern PF_ROM_CODE_PATCH_FUNC rcp_lmp_handle_remove_sco_link_request_pdu; // cch
#endif
#endif

//lmp_datamgr.c
#ifdef _CCH_8723_RCP_
extern PF_ROM_CODE_PATCH_VOID rcp_lmp_update_lps_para; // cch
#endif
extern PF_ROM_CODE_PATCH_FUNC rcp_sup_to_pwa_func; // dape

//lmp_pdu.c
#ifdef _DAPE_8723_RCP_
extern PF_ROM_CODE_PATCH_FUNC rcp_lmp_handle_conn_req_func;
#endif

//lmp_sync_links.c
#ifdef _CCH_8723_RCP_
extern PF_ROM_CODE_PATCH_FUNC rcp_lmp_validate_esco_link_req_pdu_start; // cch
#endif

#ifdef _CCH_NEW_SPEC_RCP_
extern PF_ROM_CODE_PATCH_FUNC rcp_lmp_create_esco_connection;
#endif

//bz_auth_lmp.c
#ifdef _CCH_8723_RCP_
extern PF_ROM_CODE_PATCH_FUNC  rcp_bz_auth_handle_au_rand_pdu_subcase; // cch
extern PF_ROM_CODE_PATCH_FUNC  rcp_bz_auth_handle_sres_pdu_subcase; // cch
#endif

//lmp_tasks.c
#ifdef _CCH_8723_RCP_
extern PF_ROM_CODE_PATCH_FUNC  rcp_lmp_handle_hardware_level_connection_end; // cch
#endif

//lc.c
extern PF_ROM_CODE_PATCH_FUNC rcp_lc_start_sniff_head_func;      //yilinli
extern PF_ROM_CODE_PATCH_FUNC rcp_lc_start_sniff_end_func;      //yilinli
extern PF_ROM_CODE_PATCH_FUNC rcp_lc_calculate_tolerance_func;  //yilinli
extern PF_ROM_CODE_PATCH_FUNC rcp_lc_program_sniff_sm_mode_func;   //yilinli
extern PF_ROM_CODE_PATCH_FUNC rcp_lc_sniff_sm_tune_wakeup_instant_func;  //yilinli
extern PF_ROM_CODE_PATCH_FUNC rcp_lc_handle_sm_intr_func;  //yilinli
extern PF_ROM_CODE_PATCH_FUNC rcp_lc_program_lps_mode_func;  //yilinli
extern PF_ROM_CODE_PATCH_FUNC rcp_lc_program_lps_mode_end_func;  //yilinli
extern PF_ROM_CODE_PATCH_FUNC rcp_lc_get_least_sniff_interval;
extern PF_ROM_CODE_PATCH_FUNC rcp_lc_update_pkts_allowed;

//lc_rf.c
extern PF_ROM_CODE_PATCH_VOID rcp_lc_init_radio_phy_func; //yilinli, RTL8821A
extern PF_ROM_CODE_PATCH_FUNC rcp_rtk_read_modem_radio_reg; //yilinli, RTL8723B/BTONLY
extern PF_ROM_CODE_PATCH_FUNC rcp_rtk_write_modem_radio_reg; //yilinli, RTL8723B/BTONLY
#ifdef _NEW_MODEM_PI_ACCESS_
extern PF_ROM_CODE_PATCH_FUNC rcp_rtk_read_modem_radio_reg_pi; //yilinli, RTL8723B/BTONLY
extern PF_ROM_CODE_PATCH_FUNC rcp_rtk_write_modem_radio_reg_pi; //yilinli, RTL8723B/BTONLY
#endif
#ifdef _NEW_RFC_PI_ACCESS_
extern PF_ROM_CODE_PATCH_FUNC rcp_rtk_read_rfc_reg_pi;
extern PF_ROM_CODE_PATCH_FUNC rcp_rtk_write_rfc_reg_pi;
#endif

#ifdef _YL_MODEM_RSSI_MAPPING
extern PF_ROM_CODE_PATCH_FUNC rcp_lc_modem_rssi_mapping; //yilinli,
extern PF_ROM_CODE_PATCH_FUNC rcp_lc_set_modem_lna_constraint_on_off; //yilinli,
extern PF_ROM_CODE_PATCH_FUNC rcp_rtl8821_btrf_lok; //yilinli,
extern PF_ROM_CODE_PATCH_FUNC rcp_rtl8723_btfr_SetTxGainTable; //yilinli
extern PF_ROM_CODE_PATCH_FUNC rcp_rtl8723_btrf_TxPowerTrack; //yilinli
extern PF_ROM_CODE_PATCH_FUNC rcp_rtl8723_btrf_UpdateThermalValueTimer; //yilinli
extern PF_ROM_CODE_PATCH_FUNC rcp_lc_psd_modem_init; //yilinli
#endif

#ifdef _CCH_8723_RCP_
extern PF_ROM_CODE_PATCH_FUNC rcp_lc_exit_sniff_mode;// cch
extern PF_ROM_CODE_PATCH_FUNC rcp_lc_kill_paging;// cch
#ifndef _DISABLE_HOLD_MODE_
//extern PF_ROM_CODE_PATCH_FUNC rcp_lc_program_hold_sm_mode;// cch
#endif
#ifndef _DISABLE_PARK_MODE_
//extern PF_ROM_CODE_PATCH_FUNC rcp_lc_program_park_sm_mode;// cch
#endif
extern PF_ROM_CODE_PATCH_VOID rcp_lc_baseband_reset;// cch
extern PF_ROM_CODE_PATCH_FUNC rcp_lps_period_state_machine;// cch
extern PF_ROM_CODE_PATCH_FUNC rcp_lps_period_state_enter_lps;// cch
#endif
#ifdef _CCH_8821_RCP_
extern PF_ROM_CODE_PATCH_FUNC rcp_lc_check_lps_for_link;
extern PF_ROM_CODE_PATCH_FUNC rcp_lc_check_lps_for_idle;
extern PF_ROM_CODE_PATCH_VOID rcp_lc_check_lps_for_resume;
extern PF_ROM_CODE_PATCH_FUNC rcp_lc_lps_double_check_dsm_cond;
extern PF_ROM_CODE_PATCH_FUNC rcp_lc_lps_double_check_dsm_cond_after_clr_sts;
extern PF_ROM_CODE_PATCH_FUNC rcp_lc_check_lps_task_queue;
extern PF_ROM_CODE_PATCH_FUNC rcp_lc_program_lps_upd_val;
extern PF_ROM_CODE_PATCH_FUNC rcp_lc_program_lps_check_cal;
extern PF_ROM_CODE_PATCH_FUNC rcp_lc_program_lps_upd_par;
#endif

#ifdef _CCH_8821B_TC_RCP_
extern PF_ROM_CODE_PATCH_FUNC rcp_lps_period_state_machine_fast;
#endif

#ifdef _CCH_8703B_RCP_
extern PF_ROM_CODE_PATCH_VOID rcp_dlps_save_cpu_to_partial_on_then_enter_deep_lps_mode;
extern PF_ROM_CODE_PATCH_VOID rcp_dlps_save_io_mem_to_partial_on;
extern PF_ROM_CODE_PATCH_VOID rcp_dlps_restore_io_mem_from_partial_on;
extern PF_ROM_CODE_PATCH_VOID rcp_dlps_restore_io_mem_from_partial_on_after_clk_rdy;
extern PF_ROM_CODE_PATCH_FUNC rcp_lps_period_state_machine_mid;

#endif

//lc_task.c
#ifdef _DAPE_8723_RCP_
extern PF_ROM_CODE_PATCH_FUNC rcp_lc_kill_scan_status_func;
extern PF_ROM_CODE_PATCH_FUNC rcp_lc_start_page_func;
extern PF_ROM_CODE_PATCH_FUNC rcp_lc_start_inq_func;
#endif
//lc_isr.c
extern PF_ROM_CODE_PATCH_FUNC rcp_lc_sniff_att_start_intr_head_func;   //yilinli
extern PF_ROM_CODE_PATCH_FUNC rcp_lc_sniff_att_start_intr_end_func;  //yilinli
extern PF_ROM_CODE_PATCH_FUNC rcp_lc_sniff_att_end_intr_head_func;  //yilinli
extern PF_ROM_CODE_PATCH_FUNC rcp_lc_sniff_att_end_intr_end_func;  //yilinli
extern PF_ROM_CODE_PATCH_FUNC rcp_lc_sniff_cont_count_modify_and_supto_func;  //yilinli
extern PF_ROM_CODE_PATCH_FUNC rcp_rx_interrupt_get_esco_ce_index_func;  //yilinli
extern PF_ROM_CODE_PATCH_FUNC rcp_rx_interrupt_valid_ce_index_func; //yilinli
extern PF_ROM_CODE_PATCH_FUNC rcp_rx_interrupt_2nd_half_func;
extern PF_ROM_CODE_PATCH_FUNC rcp_set_packet_type_in_dont_wait_ack_rx;
extern PF_ROM_CODE_PATCH_FUNC rcp_BB_handle_psd_end_intr_func; //yilinli
extern PF_ROM_CODE_PATCH_FUNC rcp_new_afh_fill_backup_reg_and_send_signal_func; //yilinli
extern PF_ROM_CODE_PATCH_FUNC rcp_lc_log_data_for_channel_assessment;
extern PF_ROM_CODE_PATCH_VOID rcp_bb_rx_interrupt;

#ifdef _CCH_8723_RCP_
extern PF_ROM_CODE_PATCH_FUNC rcp_BB_handle_tx_interrupt_lps_reset;// cch
extern PF_ROM_CODE_PATCH_FUNC rcp_BB_handle_rx_interrupt_lps_reset;// cch
extern PF_ROM_CODE_PATCH_FUNC rcp_BB_handle_end_of_sniff_attemp_interrupt;// cch
#endif
#ifdef _DAPE_8723_RCP_
extern PF_ROM_CODE_PATCH_FUNC rcp_lc_send_packet_for_le_func;
extern PF_ROM_CODE_PATCH_FUNC rcp_bb_tx_intr_no_pkt_scheduled_func;
extern PF_ROM_CODE_PATCH_FUNC rcp_dual_mode_switch_in_tx_func;
#endif
#ifdef _CCH_8821_RCP_
extern PF_ROM_CODE_PATCH_FUNC rcp_bb_scan_end_intr;
#endif

//lc_scan.c
#ifdef _CCH_8821_RCP_
extern PF_ROM_CODE_PATCH_FUNC rcp_lc_start_write_scan_mode;
#endif
extern PF_ROM_CODE_PATCH_FUNC rcp_lc_retrieve_page_scan_func;


// lc_utils.c
extern PF_ROM_CODE_PATCH_FUNC rcp_lc_decide_packet_type_change;
extern PF_ROM_CODE_PATCH_FUNC rcp_lc_get_exp_transfer_time;
extern PF_ROM_CODE_PATCH_FUNC rcp_lc_check_and_enable_scans_in_scatternet_func; // yilinli

//lc_sync_links.c
#ifdef _CCH_8723_RCP_
extern PF_ROM_CODE_PATCH_FUNC rcp_lc_handle_connect_sco_end;// cch
extern PF_ROM_CODE_PATCH_FUNC rcp_lc_make_esco_connection_end;// cch
extern PF_ROM_CODE_PATCH_FUNC rcp_lc_handle_sco_rx_interrupt;// cch
extern PF_ROM_CODE_PATCH_FUNC rcp_lc_handle_sco_erroneous_data;// cch
extern PF_ROM_CODE_PATCH_FUNC rcp_lc_handle_esco_rx_packet;// cch
#endif
#ifdef _CCH_NEW_SPEC_RCP_
extern PF_ROM_CODE_PATCH_FUNC rcp_lc_make_esco_connection;
extern PF_ROM_CODE_PATCH_FUNC rcp_lc_make_esco_connection_OverCodec;
extern PF_ROM_CODE_PATCH_FUNC rcp_lc_make_sco_connection_OverCodec;
#endif

extern PF_ROM_CODE_PATCH_FUNC rcp_bz_auth_handle_auth_completion;

//bt_fw_hci_events.c
#ifdef _CCH_8723_RCP_
extern PF_ROM_CODE_PATCH_FUNC rcp_hci_generate_inquiry_result_event;// cch
extern PF_ROM_CODE_PATCH_FUNC rcp_hci_generate_command_complete_event;// cch
#endif
extern PF_ROM_CODE_PATCH_FUNC rcp_hci_generate_event;

//lmp_pdu.c
#ifndef _REDUCE_LPS_AFTER_RTL8703B_
#ifdef _CCH_8723_RCP_
extern PF_ROM_CODE_PATCH_FUNC rcp_lmp_handle_max_slot_req_pdu;// cch
#endif
#endif

// ll_isr.c
#ifdef _DAPE_8723_RCP_
extern PF_ROM_CODE_PATCH_FUNC rcp_ll_update_slot_in_ce_end_func;
extern PF_ROM_CODE_PATCH_FUNC rcp_ll_ce_begin_func;
#endif
#ifdef _CCH_8821_RCP_
extern PF_ROM_CODE_PATCH_VOID rcp_ll_cleanup_rx_status_ae_end;
#endif
#ifdef _CCH_8821B_TC_RCP_
extern PF_ROM_CODE_PATCH_VOID rcp_ll_handle_event_end_interrupt;
#endif
extern PF_ROM_CODE_PATCH_FUNC rcp_ll_isr_func;
extern PF_ROM_CODE_PATCH_FUNC rcp_ll_handle_scan_start_intr_func;
extern PF_ROM_CODE_PATCH_FUNC rcp_ll_early_func;


//le_ll.c
extern PF_ROM_CODE_PATCH_FUNC rcp_ll_handle_adv_channel_pdu;

//le_llc.c
extern PF_ROM_CODE_PATCH_FUNC rcp_ll_update_instant_func;
extern PF_ROM_CODE_PATCH_FUNC rcp_llc_sent_llc_pdu_ack_recd_func;
extern PF_ROM_CODE_PATCH_FUNC rcp_ll_generate_conn_para_req_func;

//le_ll_driver.c
extern PF_ROM_CODE_PATCH_FUNC rcp_ll_data_cmd_decision;
extern PF_ROM_CODE_PATCH_FUNC rcp_ll_driver_wake_from_slave_latency;

// le_ll_general.c
#ifdef _DAPE_8723_RCP_
extern PF_ROM_CODE_PATCH_FUNC rcp_ll_update_slot_decision_func;
#ifdef _BT4_1_SUPPORT_LL_CONN_PARAM_REQ
extern PF_ROM_CODE_PATCH_FUNC rcp_ll_decide_connection_parameters_func;
#endif
#endif
//power_control.c
extern PF_ROM_CODE_PATCH_FUNC rcp_execute_lps_mode_procedure_6128_func;  //yilinli
extern PF_ROM_CODE_PATCH_FUNC rcp_enter_lps_mode_6128_func;  //yilinli
extern PF_ROM_CODE_PATCH_FUNC rcp_pow_ctrl_intr_handle_end_func;  //yilinli
#ifdef _8821A_BTON_DESIGN_
extern PF_ROM_CODE_PATCH_FUNC rcp_enable_lps_mode_setting_func; // yilinli
extern PF_ROM_CODE_PATCH_FUNC rcp_disable_lps_mode_setting_func; // yilinli
extern PF_ROM_CODE_PATCH_FUNC rcp_bton_clr_wakeup_sts_func; //yilinli
#else
extern PF_ROM_CODE_PATCH_VOID rcp_enable_lps_mode_setting_func; // yilinli
#endif
//h5.c
extern PF_ROM_CODE_PATCH_FUNC rcp_hci_uart_h5_isr_func;  //yilinli
extern PF_ROM_CODE_PATCH_FUNC rcp_hci_uart_h5_timing_adapt_func;  //yilinli
extern PF_ROM_CODE_PATCH_VOID rcp_hci_uart_h5_backup_func;  //yilinli
extern PF_ROM_CODE_PATCH_VOID rcp_hci_uart_h5_restore_func;  //yilinli
extern PF_ROM_CODE_PATCH_FUNC rcp_hci_uart_h5_poll_wake_callback_func;  //yilinli
extern PF_ROM_CODE_PATCH_FUNC rcp_hci_uart_h5_go_sleep_func;  //yilinli
extern PF_ROM_CODE_PATCH_FUNC rcp_hci_uart_h5_go_lps_func;  //yilinli
extern PF_ROM_CODE_PATCH_FUNC rcp_hci_uart_h4h5_config_func;  //yilinli
extern PF_ROM_CODE_PATCH_FUNC rcp_hci_uart_vendor_set_baud_func;  //yilinli
extern PF_ROM_CODE_PATCH_FUNC rcp_hci_uart_send_long_break_callback_func;  //yilinli
extern PF_ROM_CODE_PATCH_FUNC rcp_hci_uart_h5_wakeup_utility_func;  //yilinli
extern PF_ROM_CODE_PATCH_FUNC rcp_hci_uart_change_baudrate_event_func;  //yilinli
extern PF_ROM_CODE_PATCH_FUNC rcp_hci_uart_h4_err_event_func;  //yilinli
extern PF_ROM_CODE_PATCH_FUNC rcp_hci_uart_falling_cnt_isr_func;  //yilinli
extern PF_ROM_CODE_PATCH_FUNC rcp_hci_uart_baud_est_init_preprocessing_func;  //yilinli
extern PF_ROM_CODE_PATCH_FUNC rcp_hci_uart_baud_est_init_postprocessing_func;  //yilinli
#ifdef _CCH_8723_RCP_
extern PF_ROM_CODE_PATCH_VOID rcp_hci_uart_h5_isr_lps_reset; // cch
#endif
extern PF_ROM_CODE_PATCH_FUNC rcp_hci_uart_baud_recovery_func;

//usb_dma.c
extern PF_ROM_CODE_PATCH_FUNC rcp_hci_dma_intr_h4_error_func;  //yilinli
#ifdef _CCH_8723_RCP_
extern PF_ROM_CODE_PATCH_FUNC rcp_USB_DMA_IntrHandler_lps_reset; // cch
#endif
extern PF_ROM_CODE_PATCH_FUNC rcp_usb_dma_intrhandler_func;   //yilinli
extern PF_ROM_CODE_PATCH_FUNC rcp_S3C2410Usb_Dma_wake_host; //yilinli
#ifdef _YL_RTL8723A_B_CUT
//extern PF_ROM_CODE_PATCH_FUNC rcp_S3C2410Usb_Dma_func; // yilinli
#endif

//bb_driver.c
extern PF_ROM_CODE_PATCH_FUNC rcp_bb_write_baseband_register_func;  //yilinli

//bt_fw_acl_q.c
extern PF_ROM_CODE_PATCH_FUNC rcp_aclq_get_next_acl_pkt;

//lmp_ch_assessment.c
#ifdef COMPILE_CHANNEL_ASSESSMENT
extern PF_ROM_CODE_PATCH_FUNC rcp_lmp_handle_channel_assess_timer_expiry_func; //yilinli
extern PF_ROM_CODE_PATCH_FUNC rcp_lmp_handle_channel_classify_timer_expiry_func; //yilinli
extern PF_ROM_CODE_PATCH_FUNC rcp_lmp_handle_channel_classify_timer_expiry_II_func; //yilinli
extern PF_ROM_CODE_PATCH_FUNC rcp_lmp_generate_afh_map_for_device_RTK_func; //yilinli
extern PF_ROM_CODE_PATCH_FUNC rcp_lmp_generate_afh_map_func; //yilinli
extern PF_ROM_CODE_PATCH_FUNC rcp_lmp_generate_afh_map_post_func; //yilinli
extern PF_ROM_CODE_PATCH_FUNC rcp_lmp_generate_afh_map_for_device_RTK_post_func; //yilinli
extern PF_ROM_CODE_PATCH_FUNC rcp_modem_psd_man_refresh_at_la_period_timer_func; //yilinli
extern PF_ROM_CODE_PATCH_FUNC rcp_modem_psd_man_init_func; //yilinli
extern PF_ROM_CODE_PATCH_FUNC rcp_lmp_generate_afh_map_by_modem_psd_func; //yilinli
extern PF_ROM_CODE_PATCH_FUNC rcp_lmp_generate_afh_map_check_and_handle_few_channels_func; //yilinli
extern PF_ROM_CODE_PATCH_FUNC rcp_lmp_classify_channel_quality_RTK_func; //yilinli
extern PF_ROM_CODE_PATCH_FUNC rcp_lmp_cal_afh_channel_quality_RTK_generator_func; //yilinli
// morgan add 20120823 for recover mechanism
extern PF_ROM_CODE_PATCH_FUNC rcp_lmp_generate_afh_recovery_subroutine_func;
extern PF_ROM_CODE_PATCH_FUNC rcp_lmp_generate_afh_map_for_AFH_RECOVERY_func;
extern PF_ROM_CODE_PATCH_FUNC rcp_lmp_generate_afh_map_front_half_func;
extern PF_ROM_CODE_PATCH_FUNC rcp_lmp_generate_afh_map_from_PGR_RTK_func;
//extern PF_ROM_CODE_PATCH_FUNC rcp_lmp_decide_RECOVERY_func;
//extern PF_ROM_CODE_PATCH_FUNC rcp_lmp_decide_no_wifi_RECOVERY_func;
extern PF_ROM_CODE_PATCH_FUNC rcp_lmp_generate_afh_map_for_AFH_RECOVERY_part1_func;
extern PF_ROM_CODE_PATCH_FUNC rcp_lmp_generate_afh_map_by_psd_recovery_func;
#ifndef _BT_ONLY_
extern PF_ROM_CODE_PATCH_FUNC rcp_lmp_generate_afh_map_by_wlan_psd_func;
#endif
extern PF_ROM_CODE_PATCH_FUNC rcp_lmp_modem_psd_scan_period_timer_expiry; //yilinli
extern PF_ROM_CODE_PATCH_FUNC rcp_lmp_modem_psd_scan_tdm_timer_expiry; //yilinli
extern PF_ROM_CODE_PATCH_FUNC rcp_lmp_modem_psd_afh_map_gen_timer_expiry; //yilinli

extern PF_ROM_CODE_PATCH_FUNC rcp_lmp_piconet_start_afh_map_updation;
#endif

//logger.c
extern PF_ROM_CODE_PATCH_FUNC rcp_log;

#ifdef _SUPPORT_SECURE_CONNECTION_
// bz_auth_internal.h
extern PF_ROM_CODE_PATCH_FUNC rcp_bz_auth_start_ping_timer;
extern PF_ROM_CODE_PATCH_FUNC rcp_rx_interrupt_secure_conn_func;
#endif

#ifdef _BRUCE_8821B_PLC_RCP_
extern PF_ROM_CODE_PATCH_FUNC rcp_plc_init_func;
extern PF_ROM_CODE_PATCH_VOID rcp_lc_handle_connect_sco_func;
extern PF_ROM_CODE_PATCH_VOID rcp_lc_make_esco_connection_func;
extern PF_ROM_CODE_PATCH_VOID rcp_lc_kill_sco_connection_over_codec_func;
extern PF_ROM_CODE_PATCH_VOID rcp_lc_kill_esco_connect_over_codec_func;
extern PF_ROM_CODE_PATCH_FUNC rcp_lc_handle_esco_instant_interrupt_new;
extern PF_ROM_CODE_PATCH_FUNC rcp_lc_handle_sco_instant_interrupt;
extern PF_ROM_CODE_PATCH_FUNC rcp_bz_isoch_send_data_to_host;
#endif


#ifdef MWS_ENABLE
// mws.c
extern PF_ROM_CODE_PATCH_FUNC rcp_mws_init_func_1;
extern PF_ROM_CODE_PATCH_FUNC rcp_mws_init_func_2;
extern PF_ROM_CODE_PATCH_FUNC rcp_mws_16_bit_mode;
extern PF_ROM_CODE_PATCH_FUNC rcp_mws_8_bit_mode;
extern PF_ROM_CODE_PATCH_VOID rcp_mws_baudrate_config;
extern PF_ROM_CODE_PATCH_FUNC rcp_mws_frame_sync_protection_func;
extern PF_ROM_CODE_PATCH_FUNC rcp_mws_write_bb_frame_sync_updt_func;
extern PF_ROM_CODE_PATCH_FUNC rcp_mws_frame_sync_update_value_for_next_time_func;

// mws_isr.c
extern PF_ROM_CODE_PATCH_FUNC rcp_mws_int_handler;

#endif
#ifdef _SUPPORT_BT_CONTROL_PCM_MUX_
extern PF_ROM_CODE_PATCH_FUNC rcp_restore_pcm_mux_state_when_disconnect;
//extern PF_ROM_CODE_PATCH_FUNC rcp_switch_pcm_mux_after_con_established;
extern PF_ROM_CODE_PATCH_FUNC rcp_switch_pcm_mux_after_con_established_pcmoutctrl;
#endif

#ifdef _SUPPORT_VER_4_1_
/* bb_driver_4_1.c */
extern PF_ROM_CODE_PATCH_FUNC rcp_bb_set_piconet_adjust_func;
extern PF_ROM_CODE_PATCH_FUNC rcp_bb_prepare_clk_adj_broadcast_func;

/* lmp_4_1.c*/
extern PF_ROM_CODE_PATCH_FUNC rcp_lmp_validate_clk_adj_param_func;
#endif

extern PF_ROM_CODE_PATCH_FUNC rcp_bt_led_control;

extern PF_ROM_CODE_PATCH_FUNC rcp_dbg_msg_timer_callback;

extern PF_ROM_CODE_PATCH_FUNC rcp_usb_req_rx_dma_func;

#ifdef _SUPPORT_WL_BT_SCOREBOARD_
extern PF_ROM_CODE_PATCH_FUNC rcp_scoreboard_intrhandler_func;
#endif

#ifdef _ENABLE_VENDOR_GPIO_INTERRUPT_
extern PF_ROM_CODE_PATCH_FUNC rcp_vendor_gpio_intrhandler_func;
#endif

#ifdef _SUPPORT_NEW_USB_LPM_L1_DATA_PATH_150629_
extern PF_ROM_CODE_PATCH_FUNC rcp_hci_dma_usb_check_usb_lpm_l1_then_queue_notification;
extern PF_ROM_CODE_PATCH_FUNC rcp_hci_dma_usb_check_and_allow_enter_usb_lpm_l1;
extern PF_ROM_CODE_PATCH_FUNC rcp_hci_dma_usb_check_and_dequeue_notification;
#endif


/* bt_3dd.c */
extern PF_ROM_CODE_PATCH_FUNC rcp_generate_3dd_gpio_sync_toggle_response_func;
extern PF_ROM_CODE_PATCH_VOID rcp_bt_3dd_driver_handle_ext_clock_capture_by_edge_trigger;
extern PF_ROM_CODE_PATCH_VOID rcp_bt_3dd_driver_fn_set_shutter_delay;
extern PF_ROM_CODE_PATCH_VOID rcp_bt_3dd_driver_fn_set_host_shutter_delay;


#endif /* end of #ifdef _ROM_CODE_PATCHED_ */

extern UINT8 is_wifi_alive;

/* ====================== Module Includes Section  ======================== */
#ifdef SOFT_TRANSPORT_ENABLE
#include "hci_soft_transport.h"
#endif

#include "logger.h"
#include "bz_config.h"
#include "bz_nvconfig.h"
#include "bz_debug.h"

#ifdef SCO_OVER_HCI
#include "bz_pf_isoch.h"
#endif //#ifdef SCO_OVER_HCI

#endif /* _PLATFORM_H_ */

