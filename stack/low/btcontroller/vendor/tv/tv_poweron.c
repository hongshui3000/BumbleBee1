/***************************************************************************
 Copyright (C) RealTek Ltd.
 This module is a confidential and proprietary property of RealTek and
 a possession or use of this module requires written permission of RealTek.
 ***************************************************************************/
/********************************* Logger *************************/
enum { __FILE_NUM__= 142 };
/********************************* Logger *************************/
#include "tv.h"

#include "bt_3dd.h"
#include "bt_fw_hci_internal.h"
#include "hci_vendor_defines.h"
#include "le_ll.h"
#include "le_ll_driver.h"
#include "le_hci_4_0.h"
#include "lmp_vendor_defines.h"
#include "mem.h"
#ifdef MWS_ENABLE
#include "mws_imp.h"
#endif
#include "new_io.h"
#include "gpio.h"

TV_POWERON_MANAGER tv_poweron;

void tv_poweron_init()
{
    tv_poweron.scan.enable = FALSE;
    tv_poweron.scan.local_addr_type = LL_ADDR_TYPE_PUBLIC;
    tv_poweron.scan.filter_policy = 0;
    tv_poweron.scan.filter_duplicate = 0;
    tv_poweron.scan.active_scan = 0;
    tv_poweron.scan.ch_map = LL_DEFAULT_SCAN_CH_MAP;
    tv_poweron.scan.interval = efuse_tv_poweron->le_scan_interval;
    tv_poweron.scan.window = efuse_tv_poweron->le_scan_window;

    if (efuse_tv_poweron->power_on_tv_en)
    {
        if (efuse_tv_poweron->poweron_pin_type == TV_POWERON_PIN_BT_GPIO)
        {
            UINT32 mask = GEN_BIT_MASK(efuse_tv_poweron->poweron_pin_num);
            gpio_ctrl_set_bton(mask);
            gpio_ctrl_set_in_out(mask, 0x1FFFFF);

#ifdef FIX_TV_POWERON_PIN_BT_GPIO11_BLOCKING
            /* Avoid hardware limitation that BT_GPIO[11] cannot be controlled
             * if LTECOEX_CTRL_REG[3]=1. We force LTECOEX_CTRL_REG[3]=0 which
             * might cause problems to mws.
             */
            if (efuse_tv_poweron->poweron_pin_num == 11)
            {
                UINT32 old_reg = mws_read_register_new(LTECOEX_CTRL_REG);
                if (old_reg & BIT3)
                {
                    mws_write_register(LTECOEX_CTRL_REG, old_reg & ~BIT3, 0xf);
                }
            }
#endif
        }

        tv_poweron_pin_set(0);
        g_host_wakeup = 0;
        tv_poweron_disable_event_mask();
        tv_poweron_set_le_scan(TRUE);
    }
}

UCHAR hci_vendor_handle_tv_power_on_off_cmd(HCI_CMD_PKT *hci_cmd,
        BOOLEAN *has_send_cmd_complt_evt)
{
    *has_send_cmd_complt_evt = TRUE;
    if (hci_cmd->param_total_length != sizeof (HCI_VENDOR_TV_POWER_ON_OFF_CMD_PARAM))
    {
        return INVALID_HCI_COMMAND_PARAMETERS_ERROR;
    }
    HCI_VENDOR_TV_POWER_ON_OFF_CMD_PARAM *param =
            (HCI_VENDOR_TV_POWER_ON_OFF_CMD_PARAM *) hci_cmd->cmd_parameter;
    /* Early send command complete event */
    hci_vendor_cmd_generate_event(hci_cmd->cmd_opcode, HCI_COMMAND_SUCCEEDED, NULL);
    *has_send_cmd_complt_evt = FALSE;

    g_host_wakeup = !!(param->power_on_off);
    RT_BT_LOG(BLUE, LOG_TV_POWERON_MSG, 1, g_host_wakeup);
    if (g_host_wakeup == 0)
    {
        UINT16 watch_dog_reset_time_ms = 0;
        if (auto_detach_terminate_all_remote_links() > 0)
        {
            watch_dog_reset_time_ms = 1500;
        }
        else
        {
            watch_dog_reset_time_ms = 100;
        }
        lc_start_write_scan_mode(0);

#ifdef _SUPPORT_AUTO_DETACH_LINK_
        auto_detach_enable_link_timer(
                        AUTO_DETACH_TV_OFF_CMD, watch_dog_reset_time_ms);
#else
        WDG_TIMER_TIMEOUT_SOON;
        while (1); /* infinite loop here to wait watch dog timeout */
#endif

    }
    return HCI_COMMAND_SUCCEEDED;
}

UCHAR hci_vendor_handle_tv_set_le_scan_param_cmd(HCI_CMD_PKT *hci_cmd,
        BOOLEAN *has_send_cmd_complt_evt)
{
    *has_send_cmd_complt_evt = TRUE;
    if (hci_cmd->param_total_length != sizeof (HCI_VENDOR_TV_SET_LE_SCAN_PARAM_CMD_PARAM))
    {
        return INVALID_HCI_COMMAND_PARAMETERS_ERROR;
    }
    HCI_VENDOR_TV_SET_LE_SCAN_PARAM_CMD_PARAM *param =
            (HCI_VENDOR_TV_SET_LE_SCAN_PARAM_CMD_PARAM *) hci_cmd->cmd_parameter;
    UINT16 interval = letoh16(param->interval);
    UINT16 window = letoh16(param->window);
    if (window > interval || window < LL_SCAN_WINDOW_MIN
            || interval > LL_SCAN_INTERVAL_MAX)
    {
        return INVALID_HCI_COMMAND_PARAMETERS_ERROR;
    }
    tv_poweron.scan.interval = interval;
    tv_poweron.scan.window = window;
    tv_poweron_set_le_scan(TRUE);
    return HCI_COMMAND_SUCCEEDED;
}

void tv_poweron_pin_set_imp(UINT8 val)
{
    val = (!!val == efuse_tv_poweron->poweron_pin_polarity);
    switch (efuse_tv_poweron->poweron_pin_type)
    {
    case TV_POWERON_PIN_BT_GPIO:
        gpio_ctrl_write_gpio(efuse_tv_poweron->poweron_pin_num, val);
        break;
    case TV_POWERON_PIN_BT_WAKE_HOST:
        {
            BTON_POW_CTRL_REG_S_TYPE pow_ctrl;
            pow_ctrl.d32 = VENDOR_READ(BTON_POW_CTRL_REG);
            pow_ctrl.b.resume_to_sie_en = val;
            bton_pow_ctrl_avoid_int_sts_w1c(&pow_ctrl);
            VENDOR_WRITE(BTON_POW_CTRL_REG, pow_ctrl.d32);
        }
        break;
    }
}
void (*tv_poweron_pin_set)(UINT8) = tv_poweron_pin_set_imp;

/**
 * @brief Emit rectangular pulse.
 *
 * This function set GPIO level to high for \p high_len us and to low for
 * \p low_len us in order to generate rectangular pulse. This functions would
 * block for total (\p high_len + \p low_len) us.
 * @param high_len  High length in us. If 0, it doesn't trigger high level.
 * If negative, it doesn't block.
 * @param low_len  Low length in us. If 0, it doesn't trigger low. If negative,
 * it doesn't block.
 * @param start_level  Start level. 1 for high; 0 for low.
 */
void tv_poweron_pin_emit_pulse(int high_len, int low_len, UINT8 start_level)
{
    if (start_level == 0)
    {
        int t = high_len;
        high_len = low_len;
        low_len = t;
    }

    if (high_len != 0)
    {
        tv_poweron_pin_set(start_level);
        if (high_len > 0)
        {
            pf_delay_us(high_len);
        }
    }
    if (low_len != 0)
    {
        tv_poweron_pin_set((!start_level));
        if (low_len > 0)
        {
            pf_delay_us(low_len);
        }
    }
}

void tv_poweron_pin_emit_bit(UINT8 bit, TV_POWERON_IR_INFO *info)
{
    if (bit)
    {
        tv_poweron_pin_emit_pulse(info->logic1_high_len, info->logic1_low_len,
                info->logic1_high_first);
    }
    else
    {
        tv_poweron_pin_emit_pulse(info->logic0_high_len, info->logic0_low_len,
                info->logic0_high_first);
    }
}

void tv_poweron_pin_emit_stopbit(TV_POWERON_IR_INFO *info)
{
    if (info->stop_bit_val == 1)
    {
        tv_poweron_pin_emit_pulse(info->logic1_high_len, -1, 1);
    }
    else
    {
        tv_poweron_pin_emit_pulse(info->logic0_high_len, -1, 1);
    }
}

void tv_poweron_pin_emit_byte(UINT8 data, UINT8 num_bits, TV_POWERON_IR_INFO *info)
{
    UINT8 i;
    for (i = 0;i < num_bits;++i)
    {
        tv_poweron_pin_emit_bit(data & 0x1, info);
        data >>= 1;
    }
}

void tv_poweron_pin_emit_gap(TV_POWERON_IR_INFO *info)
{
    if (info->has_gap_stop_bit)
    {
        tv_poweron_pin_emit_pulse(info->logic1_high_len, info->gap_len, 1);
    }
    else
    {
        tv_poweron_pin_emit_pulse(0, info->gap_len, 1);
    }
}

void tv_poweron_pin_emit_code(TV_POWERON_IR_INFO *info)
{
    UINT8 * const code = info->code;
    UINT8 code_bit_len = info->code_bit_len;
    UINT8 code_remainder_bit_i = (code_bit_len >> 3);
    UINT8 code_remainder_bit_len = (code_bit_len & 0x7);
    UINT8 code_i = 0;

    if (info->has_gap)
    {
        UINT8 gap_start_bit = info->gap_start_bit;
        UINT8 gap_byte_i = (gap_start_bit >> 3);
        UINT8 gap_byte_start_bit = (gap_start_bit & 0x7);
        for ( ; code_i < gap_byte_i; ++code_i)
        {
            tv_poweron_pin_emit_byte(code[code_i], 8, info);
            TV_LOG(WHITE, LOG_LEVEL_HIGH, LOG_TV_POWERON_EMIT_BYTE, 2,
                    code[code_i], 8);
        }
        if (gap_byte_start_bit > 0)
        {
            tv_poweron_pin_emit_byte(code[code_i], gap_byte_start_bit, info);
            tv_poweron_pin_emit_gap(info);
            tv_poweron_pin_emit_byte(code[code_i] >> gap_byte_start_bit,
                    8 - gap_byte_start_bit, info);
            TV_LOG(WHITE, LOG_LEVEL_HIGH, LOG_TV_POWERON_EMIT_BYTE2, 3,
                    code[code_i], gap_byte_start_bit, 8 - gap_byte_start_bit);
            ++code_i;
        }
        else
        {
            tv_poweron_pin_emit_gap(info);
        }
    }
    for ( ; code_i < code_remainder_bit_i; ++code_i)
    {
        tv_poweron_pin_emit_byte(code[code_i], 8, info);
        TV_LOG(WHITE, LOG_LEVEL_HIGH, LOG_TV_POWERON_EMIT_BYTE, 2,
                code[code_i], 8);
    }
    if (code_remainder_bit_len > 0)
    {
        tv_poweron_pin_emit_byte(code[code_i], code_remainder_bit_len, info);
        TV_LOG(WHITE, LOG_LEVEL_HIGH, LOG_TV_POWERON_EMIT_BYTE, 2,
                code[code_i], code_remainder_bit_len);
    }
}

#ifdef CONFIG_TV_POWERON_GPIO
typedef union TV_POWERON_GPIO_WAKEUP_PARAM_
{
    struct _PACKED_
    {
        UINT16 type : 2;
        UINT16 count : 9;
        UINT16 reserved : 5;
        INT8 len1;
        INT8 len2;
    };
    void *raw;
} TV_POWERON_GPIO_WAKEUP_PARAM;

TIMER_ID tv_poweron_gpio_wakeup_timer = OS_INVALID_HANDLE;
void tv_poweron_gpio_wakeup_timer_hdlr(TIMER_ID timer, void *raw)
{
    TV_POWERON_GPIO_WAKEUP_PARAM param = { .raw = raw };

    tv_poweron_pin_emit_pulse(param.len1, param.len2, 1);

    OS_DELETE_TIMER(&tv_poweron_gpio_wakeup_timer);
    switch (param.type)
    {
    case TV_POWERON_GPIO_TYPE_FALLING_EDGE:
    case TV_POWERON_GPIO_TYPE_RISING_EDGE:
        --param.count;
        if (param.count > 0)
        {
            INT8 t = param.len1;
            param.len1 = param.len2;
            param.len2 = t;

            if (OS_CREATE_TIMER(ONESHOT_TIMER, &tv_poweron_gpio_wakeup_timer,
                    tv_poweron_gpio_wakeup_timer_hdlr, param.raw, 0) == BT_ERROR_OK)
            {
                OS_START_TIMER(tv_poweron_gpio_wakeup_timer, 10);
            }
        }
        break;
    }
}
void tv_poweron_gpio_wakeup(void *no_arg)
{
    TV_POWERON_GPIO_WAKEUP_PARAM param;
    unsigned long timeout;

    if (tv_poweron_gpio_wakeup_timer != OS_INVALID_HANDLE)
    {
        /* If this timer is already used by other request to generate signal,
         * we just ignore the current request.
         */
        return;
    }

    param.type = efuse_tv_poweron->poweron_gpio_type;
    switch (efuse_tv_poweron->poweron_gpio_type)
    {
    case TV_POWERON_GPIO_TYPE_LOW_LEVEL:
        tv_poweron_pin_emit_pulse(0, -1, 1);
        param.len1 = -1;
        param.len2 = 0;
        timeout = efuse_tv_poweron->gpio_level_time * 10;
        break;
    case TV_POWERON_GPIO_TYPE_HIGH_LEVEL:
        tv_poweron_pin_emit_pulse(-1, 0, 1);
        param.len1 = 0;
        param.len2 = -1;
        timeout = efuse_tv_poweron->gpio_level_time * 10;
        break;
    case TV_POWERON_GPIO_TYPE_FALLING_EDGE:
        tv_poweron_pin_emit_pulse(0, -1, 1);
        param.len1 = -1;
        param.len2 = 0;
        if (efuse_tv_poweron->gpio_level_time > 0)
        {
            param.count = 2 * efuse_tv_poweron->gpio_level_time - 1;
        }
        else
        {
            param.count = 1;
        }
        timeout = 10;
        break;
    case TV_POWERON_GPIO_TYPE_RISING_EDGE:
        tv_poweron_pin_emit_pulse(-1, 0, 1);
        param.len1 = 0;
        param.len2 = -1;
        if (efuse_tv_poweron->gpio_level_time > 0)
        {
            param.count = 2 * efuse_tv_poweron->gpio_level_time - 1;
        }
        else
        {
            param.count = 1;
        }
        timeout = 10;
        break;
    }
    if (OS_CREATE_TIMER(ONESHOT_TIMER, &tv_poweron_gpio_wakeup_timer,
            tv_poweron_gpio_wakeup_timer_hdlr, param.raw, 0) == BT_ERROR_OK)
    {
        OS_START_TIMER(tv_poweron_gpio_wakeup_timer, timeout);
    }
}
#endif /* CONFIG_TV_POWERON_GPIO */

void tv_poweron_ir_wakeup(TV_POWERON_INFO *pwr_info)
{
    /* Lead code */
    tv_poweron_pin_emit_pulse(pwr_info->ir_info.lead_high_len,
            pwr_info->ir_info.lead_low_len, 1);

    tv_poweron_pin_emit_code(&pwr_info->ir_info);

    if (pwr_info->ir_info.has_stop_bit)
    {
        tv_poweron_pin_emit_stopbit(&pwr_info->ir_info);
    }
    pwr_info->in_ir_gen_process = FALSE;
}

void tv_poweron_generate_key_event(UINT8 index, UINT8 type)
{
    HCI_VENDOR_TV_POWERON_KEY_EVT_PARAM evt_param;
    evt_param.subevt_code = HCI_VENDOR_TV_POWERON_KEY_SUBEVENT;
    evt_param.index = index;
    evt_param.type = type;
    hci_generate_event(HCI_VENDOR_SPECIFIC_EVENT, &evt_param,
            sizeof (evt_param));
}

BOOLEAN ll_vendor_adv_parse_tv_poweron(AD_RTK_CMD *rtk_data,
        TV_POWERON_INFO *poweron_info)
{
    TV_POWERON_ADV_DATA * const tpd = &rtk_data->data.tv_power;
    TV_POWERON_IR_INFO * const info = &poweron_info->ir_info;
    memset(info, 0, sizeof (*info));
    poweron_info->ir_info.code = tpd->ir_info;
    if (tpd->proto.type0.prefix == 0x0)
    {
        UINT8 code_bit_len;
        UINT8 code_len;
        code_bit_len = tpd->proto.type0.code_bit_len;
        if (code_bit_len == 0)
        {
            code_bit_len = 32;
        }
        code_len = BITLEN_ALIGN_BYTE(code_bit_len);
        TV_LOG(WHITE, LOG_LEVEL_HIGH, LOG_TV_POWERON_IR_CODE_LEN, 2,
                code_bit_len, code_len);

        if (rtk_data->manu_head.head.len
                + sizeof (rtk_data->manu_head.head.len)
                - sizeof (rtk_data->manu_head)
                - sizeof (rtk_data->cmd)
                - sizeof (*tpd) - code_len >= sizeof (TV_PROTO0_IR_INFO))
        {
            TV_PROTO0_IR_INFO *ir_info;
            ir_info = (TV_PROTO0_IR_INFO *) (tpd->ir_info + code_len);
            UINT8 poweroff_type = ir_info->poweroff_type;
            poweron_info->type = poweroff_type;
            if (poweroff_type == TV_POWERON_TYPE_KEY_EVENT)
            {
                poweron_info->key_info.key_type = tpd->key_type;
            }

            info->code_bit_len = code_bit_len;
            info->logic1_high_first = ir_info->logic1_high_first;
            info->logic0_high_first = ir_info->logic0_high_first;
            info->has_stop_bit = ir_info->has_stop_bit;
            info->stop_bit_val = ir_info->stop_bit_val;
            info->lead_high_len = ir_info->lead_high_len;
            info->lead_low_len = ir_info->lead_low_len;
            info->logic1_high_len = ir_info->logic1_high_len;
            info->logic1_low_len = ir_info->logic1_low_len;
            info->logic0_high_len = ir_info->logic0_high_len;
            info->logic0_low_len = ir_info->logic0_low_len;

            UINT8 shift = ir_info->shift_scalar;
            if (shift > 0)
            {
                info->lead_high_len <<= shift;
                info->lead_low_len <<= shift;
                info->logic1_high_len <<= shift;
                info->logic1_low_len <<= shift;
                info->logic0_high_len <<= shift;
                info->logic0_low_len <<= shift;
            }

            TV_LOG(WHITE, LOG_LEVEL_HIGH, LOG_TV_POWERON_IR_INFO_EX, 2,
                    poweroff_type, shift);
        }
        else
        {
            info->code_bit_len = efuse_tv_poweron->IR_num_bits;
            info->logic1_high_first = efuse_tv_poweron->IR_logic1_start_level;
            info->logic0_high_first = efuse_tv_poweron->IR_logic0_start_level;
            info->has_stop_bit = efuse_tv_poweron->IR_has_stop_bit;
            info->stop_bit_val = efuse_tv_poweron->IR_stop_bit_mode;
            info->lead_high_len = efuse_tv_poweron->IR_lead_high_len;
            info->lead_low_len = efuse_tv_poweron->IR_lead_low_len;
            info->logic1_high_len = efuse_tv_poweron->IR_logic1_high_len;
            info->logic1_low_len = efuse_tv_poweron->IR_logic1_low_len;
            info->logic0_high_len = efuse_tv_poweron->IR_logic0_high_len;
            info->logic0_low_len = efuse_tv_poweron->IR_logic0_low_len;
        }
        {
            UINT8 i = 0;
            for (i = 0; i < code_len; ++i)
            {
                TV_LOG(WHITE, LOG_LEVEL_HIGH, LOG_TV_POWERON_IR_CODE_BYTE, 2,
                        i, info->code[i]);
            }
            TV_LOG(WHITE, LOG_LEVEL_HIGH, LOG_TV_POWERON_IR_INFO, 10,
                    info->logic1_high_first,
                    info->logic0_high_first,
                    info->has_stop_bit,
                    info->stop_bit_val,
                    info->lead_high_len,
                    info->lead_low_len,
                    info->logic1_high_len,
                    info->logic1_low_len,
                    info->logic0_high_len,
                    info->logic0_low_len);
        }
    }
    else
    {
        poweron_info->type = TV_POWERON_TYPE_IR;
        UINT8 proto = tpd->proto.type;
        if (!tv_poweron_set_ir_info(info, proto))
        {
            RT_BT_LOG(RED, LOG_TV_POWERON_UNKNOWN_IR_PROTO, 1, proto);
            return FALSE;
        }
    }
    return TRUE;
}

BOOLEAN tv_poweron_set_ir_info_imp(TV_POWERON_IR_INFO *info, UINT8 proto)
{
    switch (proto)
    {
    case TV_IR_PROTO_NEC:
        info->code_bit_len = 32;
        info->logic1_high_first = TRUE;
        info->logic0_high_first = TRUE;
        info->has_stop_bit = TRUE;
        info->stop_bit_val = 1;
        info->lead_high_len = 9000;
        info->lead_low_len = 4500;
        info->logic1_high_len = 560;
        info->logic1_low_len = 1690;
        info->logic0_high_len = 560;
        info->logic0_low_len = 560;
        break;
    case TV_IR_PROTO_TC9012:
        info->code_bit_len = 32;
        info->logic1_high_first = TRUE;
        info->logic0_high_first = TRUE;
        info->has_stop_bit = TRUE;
        info->stop_bit_val = 1;
        info->lead_high_len = 4500;
        info->lead_low_len = 4500;
        info->logic1_high_len = 560;
        info->logic1_low_len = 1690;
        info->logic0_high_len = 560;
        info->logic0_low_len = 560;
        break;
    case TV_IR_PROTO_RCA:
        info->code_bit_len = 24;
        info->logic1_high_first = TRUE;
        info->logic0_high_first = TRUE;
        info->has_stop_bit = TRUE;
        info->stop_bit_val = 1;
        info->lead_high_len = 4000;
        info->lead_low_len = 4000;
        info->logic1_high_len = 500;
        info->logic1_low_len = 2000;
        info->logic0_high_len = 500;
        info->logic0_low_len = 1000;
        break;
    case TV_IR_PROTO_RC5:
        info->code_bit_len = 14;
        info->logic1_high_first = FALSE;
        info->logic0_high_first = TRUE;
        info->has_stop_bit = FALSE;
        info->stop_bit_val = 1;
        info->lead_high_len =    0;
        info->lead_low_len = 0;
        info->logic1_high_len = 889;
        info->logic1_low_len = 889;
        info->logic0_high_len = 889;
        info->logic0_low_len = 889;
        break;
    case TV_IR_PROTO_LC7461:
        info->code_bit_len = 42;
        info->logic1_high_first = TRUE;
        info->logic0_high_first = TRUE;
        info->has_stop_bit = TRUE;
        info->stop_bit_val = 1;
        info->lead_high_len = 9000;
        info->lead_low_len = 4500;
        info->logic1_high_len = 560;
        info->logic1_low_len = 560;
        info->logic0_high_len = 560;
        info->logic0_low_len = 1690;
        break;
    case TV_IR_PROTO_M50560:
        info->code_bit_len = 16;
        info->logic1_high_first = TRUE;
        info->logic0_high_first = TRUE;
        info->has_stop_bit = TRUE;
        info->stop_bit_val = 1;
        info->lead_high_len = 8000;
        info->lead_low_len = 4000;
        info->logic1_high_len = 500;
        info->logic1_low_len = 1500;
        info->logic0_high_len = 500;
        info->logic0_low_len = 500;
        info->has_gap = TRUE;
        info->gap_start_bit = 8;
        info->has_gap_stop_bit = TRUE;
        info->gap_len = 4000;
        break;
    default:
        return FALSE;
    }
    return TRUE;
}
BOOLEAN (*tv_poweron_set_ir_info)(TV_POWERON_IR_INFO *, UINT8) = tv_poweron_set_ir_info_imp;

void ll_vendor_do_poweron_action(UINT8 key_idx, TV_POWERON_INFO *poweron_info)
{
    switch (poweron_info->type)
    {
    case TV_POWERON_TYPE_KEY_EVENT:
        if (g_host_state == 0)
        {
            tv_poweron_generate_key_event(key_idx,
                    poweron_info->key_info.key_type);
            break;
        }
        /* no break: if host is off, send IR instead. */
    case TV_POWERON_TYPE_IR:
        {
            if (!poweron_info->in_ir_gen_process)
            {
                poweron_info->in_ir_gen_process = TRUE;
                OS_SIGNAL signal = {
                        .type = LMP_GENERIC_SIGNAL,
                        .param = tv_poweron_ir_wakeup,
                        .ext_param = poweron_info
                };
                OS_send_signal_to_task(lmp_task_handle, signal);
            }
        }
        break;
    }
    poweron_info->key_idx = key_idx;
}

void ll_vendor_adv_handle_rtk_cmd_tv_poweron(AD_RTK_CMD *rtk_data)
{
    TV_POWERON_ADV_DATA * const tpd = &rtk_data->data.tv_power;
    if (memcmp(otp_str_data.bt_bd_addr, tpd->tv_addr, LMP_BD_ADDR_SIZE) != 0)
    {
        return;
    }
    TV_LOG(WHITE, LOG_LEVEL_HIGH, LOG_TV_POWERON_RC_UID_LOCAL, 2,
            efuse_tv_poweron->has_rcuid_check,
            efuse_tv_poweron->rc_uid);
    if (efuse_tv_poweron->has_rcuid_check)
    {
        UINT16 rc_uid = TV_POWERON_ADV_DATA_GET_RC_UID(tpd);
        TV_LOG(WHITE, LOG_LEVEL_HIGH, LOG_TV_POWERON_RC_UID_REMOTE, 1, rc_uid);
        if (rc_uid != efuse_tv_poweron->rc_uid)
        {
            return;
        }
    }
    TV_LOG(WHITE, LOG_LEVEL_HIGH, LOG_TV_POWERON_KEY_IDX, 2, tpd->key_idx,
            tv_poweron.info.key_idx);
    if (tpd->key_idx == tv_poweron.info.key_idx)
    {
        return;
    }
    if (!ll_vendor_adv_parse_tv_poweron(rtk_data, &tv_poweron.info))
    {
        return;
    }

    ll_vendor_do_poweron_action(tpd->key_idx, &tv_poweron.info);
}

//UINT8 g_tv_le_always_scan;
//LL_SCANNER_UNIT tv_le_scanner;
UINT8 g_host_wakeup;

void tv_poweron_enable_le_scan(TV_POWERON_SCANNER_UNIT *scan, BOOLEAN in_lps)
{
    TV_POWERON_SCANNER_UNIT saved;

    /* Step 1. store original scanner parameters. */
    saved.local_addr_type = ll_manager.scan_unit.local_addr_type;
    saved.filter_policy = ll_manager.scan_unit.filter_policy;
    saved.filter_duplicate = ll_manager.scan_unit.filter_duplicate;
    saved.active_scan = ll_manager.scan_unit.active_scan;
    saved.ch_map = ll_manager.scan_unit.ch_map;
    saved.interval = ll_manager.scan_unit.interval;
    saved.window = ll_manager.scan_unit.window;

    /* Step 2. change scanner parameters for TV. These parameters
       are used in Step. 3. */
    ll_manager.scan_unit.local_addr_type = scan->local_addr_type;
    ll_manager.scan_unit.filter_policy = scan->filter_policy;
    ll_manager.scan_unit.filter_duplicate = scan->filter_duplicate;
    ll_manager.scan_unit.active_scan = scan->active_scan;
    ll_manager.scan_unit.ch_map = scan->ch_map;
    ll_manager.scan_unit.interval = scan->interval;
    ll_manager.scan_unit.window = scan->window;

    /* Step 3. enable scanning for TV. */
#ifdef _CCH_RETENTION_FLOW_FOR_DLPS_LE_
    ll_driver_enable_scanning(in_lps);
#else
    ll_driver_enable_scanning();
#endif

    /* Step 4. Set tv parameter scanning = TRUE */
    scan->enable = TRUE;

    /* Step 5. restore scanner parameters. */
    ll_manager.scan_unit.local_addr_type = saved.local_addr_type;
    ll_manager.scan_unit.filter_policy = saved.filter_policy;
    ll_manager.scan_unit.filter_duplicate = saved.filter_duplicate;
    ll_manager.scan_unit.active_scan = saved.active_scan;
    ll_manager.scan_unit.ch_map = saved.ch_map;
    ll_manager.scan_unit.interval = saved.interval;
    ll_manager.scan_unit.window = saved.window;
}

void tv_poweron_set_le_scan(UINT8 scan_en)
{
    DEF_CRITICAL_SECTION_STORAGE;
    MINT_OS_ENTER_CRITICAL();

    ll_driver_disable_scanning();
    tv_poweron.scan.enable = FALSE;

    if (scan_en)
    {
        tv_poweron_enable_le_scan(&tv_poweron.scan, FALSE);
    }
    MINT_OS_EXIT_CRITICAL();
}

void tv_poweron_disable_event_mask(void)
{
    /* Set Event Mask */
    HCI_CMD_PKT hci_cmd_ptr;
    hci_cmd_ptr.cmd_opcode = HCI_SET_EVENT_MASK_OPCODE;
    hci_cmd_ptr.param_total_length = 0x08;
    hci_cmd_ptr.cmd_parameter[0] = 0x00;
    hci_cmd_ptr.cmd_parameter[1] = 0x00;
    hci_cmd_ptr.cmd_parameter[2] = 0x00;
    hci_cmd_ptr.cmd_parameter[3] = 0x00;
    hci_cmd_ptr.cmd_parameter[4] = 0x00;
    hci_cmd_ptr.cmd_parameter[5] = 0x00;
    hci_cmd_ptr.cmd_parameter[6] = 0x00;
    hci_cmd_ptr.cmd_parameter[7] = 0x00;
    hci_handle_set_event_mask_command(&hci_cmd_ptr);

    /* Command complete cannot be masked and Command status cannot be masked */
    //lmp_self_device_data.event_mask[1] &= (~0x60);

    /* Num_complete_pkts cannot be masked */
    lmp_self_device_data.event_mask[2] &= (~0x04);
}


