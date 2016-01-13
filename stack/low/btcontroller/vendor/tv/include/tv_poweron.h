/***************************************************************************
 Copyright (C) RealTek Ltd.
 This module is a confidential and proprietary property of RealTek and
 a possession or use of this module requires written permission of RealTek.
 ***************************************************************************/
#ifndef INSIDE_TV_TOP_HEADER_
#error "Do not include this header directly. Use tv.h instead."
#endif
#ifndef TV_POWERON_H_
#define TV_POWERON_H_

#include "le_ll.h"
#include "otp.h"
#include "platform.h"
#include "tv_types.h"

typedef struct HCI_VENDOR_TV_POWER_ON_OFF_CMD_PARAM_
{
    UCHAR power_on_off;
} HCI_VENDOR_TV_POWER_ON_OFF_CMD_PARAM;

typedef struct HCI_VENDOR_TV_SET_LE_SCAN_PARAM_CMD_PARAM_
{
    UINT8 interval[2];
    UINT8 window[2];
} HCI_VENDOR_TV_SET_LE_SCAN_PARAM_CMD_PARAM;

UCHAR hci_vendor_handle_tv_power_on_off_cmd(HCI_CMD_PKT *hci_cmd, BOOLEAN *has_send_cmd_complt_evt);
UCHAR hci_vendor_handle_tv_set_le_scan_param_cmd(HCI_CMD_PKT *hci_cmd, BOOLEAN *has_send_cmd_complt_evt);

/**
 * @struct struct TV_POWERON_ADV_DATA_
 * @brief Power on parameters.
 */
typedef struct TV_POWERON_ADV_DATA_
{
    /** Remote controller type code in big-endian byte order. */
    UINT8 rc_uid[2];
    /** TV address. */
    UINT8 tv_addr[6];
    /** Key index number for filtering duplicates. */
    UINT8 key_idx : 7;
    /** Key type (0: short press; 1: long press). */
    UINT8 key_type : 1;
    /**
     * @brief Protocol type code.
     *
     * Use \a type0 if bit[7:5] = 0; otherwise, use \a type.
     */
    union {
        struct
        {
            UINT8 code_bit_len : 5; /**< [4:0] Number of valid bits of code. */
            UINT8 prefix : 3; /**< [7:5] type0 prefix. */
        } type0;
        UINT8 type;
    } proto;
    /**
     * @brief Start address of IR info data.
     *
     * The first element of IR info data must be code (variable length
     * according to #proto).
     */
    UINT8 ir_info[0];
} TV_POWERON_ADV_DATA;

#define TV_POWERON_ADV_DATA_GET_RC_UID(o) (betoh16((o)->rc_uid))

#define TV_IR_PROTO_NEC     0x20
#define TV_IR_PROTO_TC9012  0x21
#define TV_IR_PROTO_RCA     0x22
#define TV_IR_PROTO_RC5     0x23
#define TV_IR_PROTO_LC7461  0x24
#define TV_IR_PROTO_M50560  0x25

/**
 * @struct TV_PROTO0_IR_INFO_
 * @brief Type0 TV protocol data.
 */
typedef struct _PACKED_ TV_PROTO0_IR_INFO_
{
    /** [0] If logic 1 pulse starts from high. */
    UINT8 logic1_high_first : 1;
    /** [1] If logic 0 pulse starts from high. */
    UINT8 logic0_high_first : 1;
    /** [2] If there is stop bit.  */
    UINT8 has_stop_bit : 1;
    /** [3] If stop bit high length uses high length of logic 1 or logic 0.*/
    UINT8 stop_bit_val : 1;
    /** [4] Reserved. */
    UINT8 reserved : 1;
    /** [5] Power off type (0: IR; 1: vendor event)
     * @see HCI_VENDOR_TV_POWERON_KEY_SUBEVENT
     */
    UINT8 poweroff_type : 1;
    /** [7:6] Number of shift bits for IR length parameters. */
    UINT8 shift_scalar : 2;
    /** [23:8] Lead code high length. */
    UINT16 lead_high_len;
    /** [39:24] Lead code low length. */
    UINT16 lead_low_len;
    /** [51:40] Logic 1 high length. */
    UINT16 logic1_high_len : 12;
    /** [63:52] Logic 1 low length. */
    UINT16 logic1_low_len : 12;
    /** [75:64] Logic 1 low length. */
    UINT16 logic0_high_len : 12;
    /** [87:76] Logic 1 low length. */
    UINT16 logic0_low_len : 12;
} TV_PROTO0_IR_INFO;

typedef struct TV_POWERON_KEY_INFO_
{
    /** Power key type. (0: short press; 1: long press) */
    UINT8 key_type;
} TV_POWERON_KEY_INFO;

/**
 * @struct TV_IR_GENERATOR_
 * @brief TV IR information to generate IR code.
 */
typedef struct TV_POWERON_IR_INFO_
{
    UINT8 *code;
    UINT8 code_bit_len;
    BOOLEAN logic1_high_first;
    BOOLEAN logic0_high_first;
    BOOLEAN has_stop_bit;
    UINT8 stop_bit_val;
    UINT32 lead_high_len;
    UINT32 lead_low_len;
    UINT32 logic1_high_len;
    UINT32 logic1_low_len;
    UINT32 logic0_high_len;
    UINT32 logic0_low_len;
    BOOLEAN has_gap; /**< Whether to generate gap. */
    UINT8 gap_start_bit; /**< The starting bit of code to generate gap. */
    BOOLEAN has_gap_stop_bit; /**< Whether to generate stop bit for gap. */
    UINT16 gap_len; /**< Gap length in usec. */
} TV_POWERON_IR_INFO;

typedef struct TV_POWERON_INFO_
{
    UINT8 type;
    TV_POWERON_KEY_INFO key_info;
    TV_POWERON_IR_INFO ir_info;
    UINT8 key_idx;
    UINT8 in_ir_gen_process;
} TV_POWERON_INFO;

typedef struct TV_POWERON_SCANNER_UNIT_
{
    /** Enable the scanner. */
    UINT8 enable : 1;
    /** Local address type. (0: public; 1: random) */
    UINT8 local_addr_type : 1;
    /** Enable scanning filter policy. */
    UINT8 filter_policy : 1;
    /** Enable duplicate filter.
     *
     * Need generate LE advertising reports if enabled.
     */
    UINT8 filter_duplicate : 1;
    /** Scan type. (0: passive scan; 1: active scan) */
    UINT8 active_scan : 1;
    /** Scanning channel map. */
    UINT8 ch_map : 3;
    /** Scan interval. */
    UINT16 interval;
    /** Scan window. */
    UINT16 window;
} TV_POWERON_SCANNER_UNIT;

typedef struct TV_POWERON_MANAGER_
{
    TV_POWERON_INFO info;
    TV_POWERON_SCANNER_UNIT scan;
} TV_POWERON_MANAGER;

extern TV_POWERON_MANAGER tv_poweron;

/**
 * @struct BT_EFUSE_TV_POWERON_
 * @brief TV power on EFuse settings.
 *
 * Refer to EFUSE[0x1CD-0x1B0] and otp_str_data.EFuse_TV_Poweron_Group.
 */
typedef struct _PACKED_ BT_EFUSE_TV_POWERON_ {
    /** EFUSE[0x1B1-0x1B0] Lead code high length in us. */
    UINT16 IR_lead_high_len;
    /** EFUSE[0x1B3-0x1B2] Lead code low length in us. */
    UINT16 IR_lead_low_len;
    /** EFUSE[0x1B5][3:0]-EFUSE[0x1B4] Logical 1 high length in us. */
    UINT16 IR_logic1_high_len : 12;
    /** EFUSE[0x1B6]-EFUSE[0x1B5][7:4] Logical 1 low length in us. */
    UINT16 IR_logic1_low_len : 12;
    /** EFUSE[0x1B8][3:0]-EFUSE[0x1B7] Logical 0 high length in us. */
    UINT16 IR_logic0_high_len : 12;
    /** EFUSE[0x1B9]-EFUSE[0x1B8][7:4] Logical 0 low length in us. */
    UINT16 IR_logic0_low_len : 12;

    /** EFUSE[0x1BA] Number of valid bits in IR code. */
    UINT8 IR_num_bits;

    /** EFUSE[0x1BB][0] Enable power on over bluetooth. */
    UINT8 power_on_tv_en : 1;
    /** EFUSE[0x1BB][1] TV power pin default state. (0: low; 1: high) */
    UINT8 TV_default_idle_state : 1;
    /** EFUSE[0x1BB][2] Logical 1 starting level. */
    UINT8 IR_logic1_start_level : 1;
    /** EFUSE[0x1BB][3] Logical 0 starting level. */
    UINT8 IR_logic0_start_level : 1;
    /** EFUSE[0x1BB][4] Enable stop bit in IR code. */
    UINT8 IR_has_stop_bit : 1;
    /** EFUSE[0x1BB][5] Stop bit length indication.
     *
     * 0: using logic0 high length.\n
     * 1: using logic1 high length.\n
     */
    UINT8 IR_stop_bit_mode : 1;
    /** EFUSE[0x1BB][6] Enable remote controller uid checking. */
    UINT8 has_rcuid_check : 1;
    /** EFUSE[0x1BB][7] Reserved. */
    UINT8 reserved0 : 1;
    /** EFUSE[0x1BD-0x1BC]*/
    UINT16 rc_uid;

    /** EFUSE[0x1BE][2:0] Host power status mode.
     *
     * 0: Disable\n
     * 1: Use HOST_WAKE_BT\n
     * 2: Use host heartbeat command/event\n
     * 3-7: Reserved\n
     */
    UINT8 host_power_stat_mode : 3;
    /** EFUSE[0x1BE][3] HOST_WAKE_BT polarity.
     *
     * 0: high level for power on\n
     * 1: low level for power on\n
     */
    UINT8 host_poweron_polarity : 1;
    /** EFUSE[0x1BE][4] Enable heartbeat event sent to host. */
    UINT8 heartbeat_event_enable : 1;
    /** EFUSE[0x1BE][7:5] Reserved. */
    UINT8 reserved1 : 3;
    /** EFUSE[0x1BF] Heartbeat command period in unit of 100ms. */
    UINT8 heartbeat_period;
    /** EFUSE[0x1C1-0x1C0] Background LE scan window. */
    UINT16 le_scan_window;
    /** EFUSE[0x1C3-0x1C2] Background LE scan interval. */
    UINT16 le_scan_interval;

    /** EFUSE[0x1C4][0] Poweron pin polarity. (1: High active, 0: low active) */
    UINT8 poweron_pin_polarity : 1;
    /** EFUSE[0x1C4][2:1] Poweron pin type.
     *
     * 0: BT GPIO.
     * 1: BT_WAKE_HOST.
     */
    UINT8 poweron_pin_type : 2;
    /** EFUSE[0x1C4][7:3] BT GPIO pin number used for poweron pin. */
    UINT8 poweron_pin_num : 5;
} BT_EFUSE_TV_POWERON;

#define efuse_tv_poweron ((BT_EFUSE_TV_POWERON *) otp_str_data.EFuse_TV_Poweron_Group)

#define TV_POWER_STATUS_UNDEFINED           0
#define TV_POWER_STATUS_HOST_WAKE_BT_PIN    1
#define TV_POWER_STATUS_HEARTBEAT           2

#define TV_POWERON_TYPE_IR          0
#define TV_POWERON_TYPE_KEY_EVENT   1

#define TV_POWERON_PIN_BT_GPIO          0
#define TV_POWERON_PIN_BT_WAKE_HOST     1

/* otp.EFuse_TV_Poweron_Group configuration */
#define TV_POWERON_PIN_POLARITY     1
#define TV_POWERON_PIN_TYPE         TV_POWERON_PIN_BT_GPIO
#if defined(_RTL8723D_SPECIFIC_)
#define TV_POWERON_PIN_NUM          15
#elif defined(_RTL8822B_SPECIFIC_)
#define TV_POWERON_PIN_NUM          16
#else
#define TV_POWERON_PIN_NUM          11
#endif
#define TV_POWERON_PIN_CONFIG       (TV_POWERON_PIN_POLARITY \
                                    | (TV_POWERON_PIN_BT_GPIO << 1) \
                                    | (TV_POWERON_PIN_NUM << 3))

#ifdef CONFIG_TV_POWERON_GPIO
#define TV_POWERON_GPIO_TYPE_LOW_LEVEL      0
#define TV_POWERON_GPIO_TYPE_HIGH_LEVEL     1
#define TV_POWERON_GPIO_TYPE_FALLING_EDGE   2
#define TV_POWERON_GPIO_TYPE_RISING_EDGE    3
#endif

extern void (*tv_poweron_pin_set)(UINT8 val);

void tv_poweron_init();

extern UINT8 g_host_wakeup;

void tv_poweron_enable_le_scan(TV_POWERON_SCANNER_UNIT *scan, BOOLEAN in_lps);
void tv_poweron_set_le_scan(UINT8 scan_en);
void tv_poweron_disable_event_mask(void);

/**
 * @brief Fill IR information according to \p proto.
 *
 * This function only sets related values. It's better to initialize \p ir_info
 * with zeros at first.
 *
 * @param ir_info  IR information structure.
 * @param proto  Protocol type.
 * @return \c TRUE if successful, or \c FALSE on unknown protocol.
 */
extern BOOLEAN (*tv_poweron_set_ir_info)(TV_POWERON_IR_INFO *ir_info, UINT8 proto);

void ll_vendor_do_poweron_action(UINT8 key_idx, TV_POWERON_INFO *poweron_info);
void ll_vendor_adv_handle_rtk_cmd_tv_poweron(AD_RTK_CMD *rtk_data);
BOOLEAN ll_vendor_adv_parse_tv_poweron(AD_RTK_CMD *rtk_data, TV_POWERON_INFO *info);

#endif /* TV_POWERON_H_ */
