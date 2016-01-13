
/***************************************************************************
 Copyright (C) RealTek Ltd.
 This module is a confidential and proprietary property of RealTek and
 a possession or use of this module requires written permission of RealTek.
 ***************************************************************************/

#ifndef _MWS_ISR_H_
#define _MWS_ISR_H_

SECTION_ISR void mws_int_handler(void);
SECTION_ISR void mws_handle_lte_rx_to_interrupt(UINT32 interrupt_status);
SECTION_ISR void mws_handle_lte_tx_to_interrupt(UINT32 interrupt_status);
SECTION_ISR void mws_handle_lte_int_pattern2_interrupt(UINT32 interrupt_status);
SECTION_ISR void mws_handle_lte_int_pattern1_interrupt(UINT32 interrupt_status);
SECTION_ISR void mws_handle_lte_framesync_interrupt(UINT32 interrupt_status);
SECTION_ISR void mws_handle_lte_tx_to_idle_rx_interrupt(UINT32 interrupt_status);
SECTION_ISR void mws_handle_lte_tx_on_interrupt(UINT32 interrupt_status);
SECTION_ISR void mws_handle_nrt_bt_tx_done_interrupt(UINT32 interrupt_status);
SECTION_ISR void mws_handle_nrt_wlan_tx_done_interrupt(UINT32 interrupt_status);
SECTION_ISR void mws_handle_nrt_rx_interrupt(void);
SECTION_ISR void mws_handle_rt_rx_interrupt(void);
SECTION_ISR void mws_handle_uart_nrt_rx_timeout_interrupt(void);
SECTION_ISR void mws_handle_uart_rt_rx_timeout_interrupt(void);
SECTION_ISR void mws_handle_uart_lsr_interrupt(void);
SECTION_ISR void mws_handle_nrt_rx_full_interrupt(UINT32 interrupt_status);
SECTION_ISR void mws_handle_rt_rx_full_interrupt(UINT32 interrupt_status);
#endif
