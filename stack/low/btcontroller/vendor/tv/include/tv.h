/***************************************************************************
 Copyright (C) RealTek Ltd.
 This module is a confidential and proprietary property of RealTek and
 a possession or use of this module requires written permission of RealTek.
 ***************************************************************************/

#ifndef TV_H_
#define TV_H_

#define INSIDE_TV_TOP_HEADER_
#ifdef CONFIG_TV_POWERON
#include "tv_poweron.h"
#endif
#undef INSIDE_TV_TOP_HEADER_
#include "le_hci_4_0.h"

/** RTK command in advertising data. */
struct AD_RTK_CMD_
{
    AD_MANUFACTURER_DATA_HEADER manu_head;
    UINT8 cmd[2];
    union
    {
#ifdef CONFIG_TV_POWERON
        TV_POWERON_ADV_DATA tv_power;
#endif
        /**
         * @brief Raw data.
         *
         * Meet maximal size. No other data member can exceed it.
         */
        UINT8 raw[25];
    } data;
};

#define AD_RTK_DATA_GET_CMD(o) (letoh16((o)->cmd))

#define AD_RTK_CMD_TV_POWERON 0x0001
#define AD_RTK_CMD_TV_AUTO_PAIRING 0x0002

AD_RTK_CMD *ll_vendor_adv_parse_rtk_cmds(LE_HW_ADVERTISING_CH_RX_PKT_S *pkt);
/**
 * @brief Parse and handle the RTK command contained in the advertising packet.
 * @param pkt  Advertising packet.
 * @param hw_status  Hardware status.
 * @param[out] send_adv_rpt  Store the value whether an advertising report has
 * to be generated.
 */
void ll_vendor_adv_handle_rtk_cmds(LE_HW_ADVERTISING_CH_RX_PKT_S *pkt, LE_HW_RX_PKT_TAIL_S *hw_status, BOOLEAN *send_adv_rpt);

extern void (*tv_app_init)(void);

#endif /* TV_H_ */

