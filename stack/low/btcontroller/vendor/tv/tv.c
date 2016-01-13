/***************************************************************************
 Copyright (C) RealTek Ltd.
 This module is a confidential and proprietary property of RealTek and
 a possession or use of this module requires written permission of RealTek.
 ***************************************************************************/
/********************************* Logger *************************/
enum { __FILE_NUM__= 141 };
/********************************* Logger *************************/
#include "tv.h"
#include "bt_3dd.h"
#include "mem.h"

#ifdef _ROM_CODE_PATCHED_
BOOLEAN (*rcp_tv_dbg_timer_timeout_hdlr)(TimerHandle_t) = NULL;
#endif

AD_RTK_CMD *ll_vendor_adv_parse_rtk_cmds(LE_HW_ADVERTISING_CH_RX_PKT_S *pkt)
{
    AD_HEADER *ad_header = ad_header_find(pkt->AdvInd.u1AdvData,
            pkt->Header.Length - sizeof (pkt->AdvInd.AdvA),
            AD_TYPE_MANUFACTURER_DATA);
    if (ad_header == NULL)
    {
        return NULL;
    }
    TV_LOG(WHITE, LOG_LEVEL_HIGH, LOG_TV_POWERON_AD_DATA_TYPE, 1, ad_header->type);

    UINT16 company_id = AD_MANUFACTURER_DATA_GET_COMPANY_ID(
            (AD_MANUFACTURER_DATA_HEADER *) ad_header);
    TV_LOG(WHITE, LOG_LEVEL_HIGH, LOG_TV_POWERON_MANU_COMP_ID, 1, company_id);
    if (company_id != RTK_COMPANY_ID)
    {
        return NULL;
    }
    return (AD_RTK_CMD *) ad_header;
}

void ll_vendor_adv_handle_rtk_cmds(LE_HW_ADVERTISING_CH_RX_PKT_S *pkt,
        LE_HW_RX_PKT_TAIL_S *hw_status, BOOLEAN *send_adv_rpt)
{
    UINT8 pdu_type = pkt->Header.PDU_Type;

    AD_RTK_CMD *rtk_data = ll_vendor_adv_parse_rtk_cmds(pkt);
    if (rtk_data == NULL)
    {
        return;
    }
    UINT16 cmd = AD_RTK_DATA_GET_CMD(rtk_data);
    TV_LOG(WHITE, LOG_LEVEL_HIGH, LOG_TV_POWERON_RTK_CMD, 1, cmd);
    switch (cmd)
    {
#ifdef CONFIG_TV_POWERON
    case AD_RTK_CMD_TV_POWERON:
        if (efuse_tv_poweron->power_on_tv_en
                && pdu_type == LL_ADV_PDU_TYPE_ADV_NONCONN_IND)
        {
            ll_vendor_adv_handle_rtk_cmd_tv_poweron(rtk_data);
            *send_adv_rpt = FALSE;
        }
        break;
#endif
    default:
        RT_BT_LOG(RED, LOG_TV_POWERON_UNKNOWN_RTK_CMD, 1, cmd);
    }
}


TimerHandle_t tv_dbg_timer = NULL;
void tv_dbg_timer_timeout_hdlr(TimerHandle_t timer)
{
#ifdef _ROM_CODE_PATCHED_
    if (rcp_tv_dbg_timer_timeout_hdlr != NULL)
    {
        if (rcp_tv_dbg_timer_timeout_hdlr(timer))
        {
            return;
        }
    }
#endif

#ifdef CONFIG_TV_POWERON
    TV_LOG(GRAY, LOG_LEVEL_HIGH, DD_MSG_CMD11, 9, tv_poweron.scan.enable,
            tv_poweron.scan.interval, tv_poweron.scan.window,
            tv_poweron.scan.filter_duplicate, lmp_self_device_data.scan_enable,
            BB_read_baseband_register(PAGE_SCAN_WINDOW_REGISTER)<<1,
            BB_read_baseband_register(PAGE_SCAN_INTERVAL_REGISTER)<<1,
            BB_read_baseband_register(INQ_SCAN_WINDOW_REGISTER)<<1,
            BB_read_baseband_register(INQ_SCAN_INTERVAL_REGISTER)<<1);
#endif
}

void tv_app_init_imp(void)
{
#ifdef _SUPPORT_GPIO_WAKEUP_TV_CHECK_NEGATIVE_POLARITY2
    {
        UINT16 temp = RD_16BIT_SYSON_IO(0x64);
        temp |= BIT13;
        WR_16BIT_SYSON_IO(0x64, temp);
    }

#endif

#ifdef _SUPPORT_GPIO_WAKEUP_TV_CHECK
    GENERAL_CONTROL_S_TYPE bt_general_ctrl;
    bt_general_ctrl.d16 = otp_str_data.general_control;

    RT_BT_LOG(WHITE, YL_DBG_HEX_4, 4,
    RD_16BIT_SYSON_IO(0x64),
    bt_general_ctrl.b.gpio_wake_up_fun_en,
    bt_general_ctrl.b.gpio_wake_up_type,
    bt_general_ctrl.b.gpio_wake_up_polarity);
#endif

    if (tv_dbg_timer == NULL
            && OS_CREATE_TIMER(PERIODIC_TIMER, &tv_dbg_timer,
                    tv_dbg_timer_timeout_hdlr, NULL, 0) == BT_ERROR_OK)
    {
        OS_START_TIMER(tv_dbg_timer, 10000);
    }
#ifdef CONFIG_TV_POWERON
    tv_poweron_init();
#endif
}
void (*tv_app_init)(void) = tv_app_init_imp;
