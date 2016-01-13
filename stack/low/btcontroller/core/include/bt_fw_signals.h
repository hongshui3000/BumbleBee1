/***************************************************************************
 Copyright (C) Mindtree Ltd.
 This module is a confidential and proprietary property of MindTree and
 a possession or use of this module requires written permission of MindTree.
 ***************************************************************************/

#ifndef __BT_FW_SIGNALS_H__
#define __BT_FW_SIGNALS_H__

/* External signals range is 0 - BT_FW_EXT_RESERVED_SIGNALS */

#define SIGNAL_BEGIN                            0
#define SIGNAL_END                              1

/* isr_extended_task_handle */
#define ISR_EXTENDED_SIGNAL                     (100)
#define ISR_EXT_UART_RX                         (ISR_EXTENDED_SIGNAL + 0)
#define ISR_EXT_INQ_RES_LOG                     (ISR_EXTENDED_SIGNAL + 1)
#define ISR_EXT_RESET_MAILBOX_TASK              (ISR_EXTENDED_SIGNAL + 2)
#define ISR_EXT_TIMER1_CALLBACK_BG              (ISR_EXTENDED_SIGNAL + 3)
#define ISR_EXT_SLOT_OFST_LOG                   (ISR_EXTENDED_SIGNAL + 4)
#define ISR_EXT_ENTER_LPS_MODE                  (ISR_EXTENDED_SIGNAL + 5)
#define ISR_EXT_BOTTOM_HALF_CALLBACK_PROCESS    (ISR_EXTENDED_SIGNAL + 6)
#ifdef _DAPE_FRAME_SYNC_PROCESS_TO_BG
#define ISR_EXT_FRAME_SYNC_INTR                 (ISR_EXTENDED_SIGNAL + 7)
#define LAST_ISR_EXTENDED_SIGNAL                (ISR_EXTENDED_SIGNAL + 7)
#else
#define LAST_ISR_EXTENDED_SIGNAL                (ISR_EXTENDED_SIGNAL + 6)
#endif

/* timer_task_handle */
#define TIMER_TICK_SIGNAL                       (200)
#define LAST_TIMER_TICK_SIGNAL                  (TIMER_TICK_SIGNAL + 0)

/* hci_command_task_handle */
#define HCI_CMD_RECD_SIGNAL                     (300)
#define LAST_HCI_CMD_RECD_SIGNAL                (HCI_CMD_RECD_SIGNAL + 0)

/* hci_td_task_handle -  hci_td_process_signal */
#define FIRST_HCI_TD_TASK_SIGNAL                (400)
#define HCI_TD_HCI_EVENT_TO_HOST_SIGNAL         (FIRST_HCI_TD_TASK_SIGNAL +0)
#define HCI_TD_ACL_DATA_TO_HOST_SIGNAL          (FIRST_HCI_TD_TASK_SIGNAL +1)
#define HCI_TD_SYNCHRONOUS_DATA_TO_HOST_SIGNAL  (FIRST_HCI_TD_TASK_SIGNAL +2)
#define HCI_TD_HANDLE_TX_COMPLETED_SIGNAL       (FIRST_HCI_TD_TASK_SIGNAL +3)
#define HCI_TD_FREE_ACL_BUFFER_SIGNAL           (FIRST_HCI_TD_TASK_SIGNAL +4)
#define PLATFORM_TASK_SIGNAL                    (FIRST_HCI_TD_TASK_SIGNAL +5)
#ifndef LE_MODE_EN
#define LAST_HCI_TD_TASK_SIGNAL                 (FIRST_HCI_TD_TASK_SIGNAL +5)
#else
#define HCI_TD_LE_ACL_DATA_TO_HOST_SIGNAL       (FIRST_HCI_TD_TASK_SIGNAL +6)
#define LAST_HCI_TD_TASK_SIGNAL                 (FIRST_HCI_TD_TASK_SIGNAL +6)
#endif


/* hci_event_task_handle */
#define FIRST_HCI_EVENT_TASK_SIGNAL              (500)
#define HCI_DISCONNECT_COMPLETE_SIGNAL           (FIRST_HCI_EVENT_TASK_SIGNAL+0)
#define HCI_UNPARK_NEW_CONN_TIMEOUT              (FIRST_HCI_EVENT_TASK_SIGNAL+1)
#define HCI_HANDLE_DATA_TX_SIGNAL                (FIRST_HCI_EVENT_TASK_SIGNAL+2)
#define LC_DELIVER_HOST_SCO_PKT                  (FIRST_HCI_EVENT_TASK_SIGNAL+3)
#define HCI_GEN_QOS_COMPLETE_EVENT_SIGNAL        (FIRST_HCI_EVENT_TASK_SIGNAL+4)
#define HCI_GEN_FLOW_SPEC_COMPLETE_EVENT_SIGNAL  (FIRST_HCI_EVENT_TASK_SIGNAL+5)
#define HCI_GEN_CONN_PKT_TYPE_CHGD_EVENT_SIGNAL  (FIRST_HCI_EVENT_TASK_SIGNAL+6)
#define HCI_GEN_ACTIVE_MODE_CHANGED_EVENT_SIGNAL (FIRST_HCI_EVENT_TASK_SIGNAL+7)
#define HCI_HOST_EVENT_MASKED_SIGNAL             (FIRST_HCI_EVENT_TASK_SIGNAL+8)
#define HCI_SEND_SYNCHRONOUS_PACKET_SIGNAL       (FIRST_HCI_EVENT_TASK_SIGNAL+9)
#define LAST_HCI_EVENT_TASK_SIGNAL               (FIRST_HCI_EVENT_TASK_SIGNAL+9)

/* lmp_task_handle */
#define FIRST_LMP_TASK_SIGNAL                   (600)
#define LMP_HANDLE_INQUIRY_TIMEOUT_SIGNAL       (FIRST_LMP_TASK_SIGNAL +0)
#define LMP_HANDLE_PAGE_TIMEOUT_SIGNAL          (FIRST_LMP_TASK_SIGNAL +1)
#define LMP_SWITCH_ROLE_TIMER_SIGNAL            (FIRST_LMP_TASK_SIGNAL +2)
#define LMP_DELETE_TIMER_HANDLE_SIGNAL          (FIRST_LMP_TASK_SIGNAL +3)
#define LMP_SUPERVISION_TIMEOUT_SIGNAL          (FIRST_LMP_TASK_SIGNAL +4)
#define LMP_NUM_PKTS_TIMER_SIGNAL               (FIRST_LMP_TASK_SIGNAL +5)
#define LMP_SLAVE_UNPARK_REQ_RECD_SIGNAL        (FIRST_LMP_TASK_SIGNAL +6)
#define LMP_COMPLETE_HOLD_MODE_SIGNAL           (FIRST_LMP_TASK_SIGNAL +7)
#define LMP_HOLD_INSTANT_SIGNAL                 (FIRST_LMP_TASK_SIGNAL +8)
#define LMP_LLC_CON_SETUP_COMPLETE              (FIRST_LMP_TASK_SIGNAL +9)
#define LMP_PTT_SIGNAL                          (FIRST_LMP_TASK_SIGNAL +10)
#define LMP_ABNORMAL_EXIT_PARK_SIGNAL           (FIRST_LMP_TASK_SIGNAL +11)
#define LMP_LAST_CON_DISCONNECTED_SIGNAL        (FIRST_LMP_TASK_SIGNAL +12)
#define LMP_UPDATE_MAP_SIGNAL                   (FIRST_LMP_TASK_SIGNAL +13)
#define LMP_CALCULATE_DHKEY_SIGNAL              (FIRST_LMP_TASK_SIGNAL +14)
#define LMP_INITIATE_SSR_REQ                    (FIRST_LMP_TASK_SIGNAL +15)
#define LMP_START_TIMER_SIGNAL                  (FIRST_LMP_TASK_SIGNAL +16)
#define LMP_STOP_TIMER_SIGNAL                   (FIRST_LMP_TASK_SIGNAL +17)
#define LMP_PARK_NCTO_SIGNAL                    (FIRST_LMP_TASK_SIGNAL +18)
#define LMP_ENTER_SNIFF_MODE                    (FIRST_LMP_TASK_SIGNAL +19)
#define LMP_EXIT_SNIFF_MODE                     (FIRST_LMP_TASK_SIGNAL +20)
#ifdef _USE_SNIFF_NO_ACL_COUNT_REGISTER
#define LMP_SNIFF_NO_ACL_RESTORE_SIGNAL         (FIRST_LMP_TASK_SIGNAL +21)
#endif
#define LMP_CALCULATE_DHKEY_PARTIAL_SIGNAL      (FIRST_LMP_TASK_SIGNAL +22)
#define LL_GEN_HCI_EVENT_SIGNAL                 (FIRST_LMP_TASK_SIGNAL +23)
#define LL_CONTROL_SIGNAL                       (FIRST_LMP_TASK_SIGNAL +24)
#define LMP_CHK_AUTH_STAGE2_SIGNAL              (FIRST_LMP_TASK_SIGNAL +25)
#define LAST_LMP_TASK_SIGNAL                    (FIRST_LMP_TASK_SIGNAL +25)

/* lc_rx_task_handle */
#define FIRST_LC_RX_TASK_SIGNAL                 (700)
#define LC_HLC_SIGNAL                           (FIRST_LC_RX_TASK_SIGNAL +0)
#define LC_HANDLE_PAGE_RESP_TIMEOUT_SIGNAL      (FIRST_LC_RX_TASK_SIGNAL +1)
#define LC_HANDLE_RECD_PACKET_SIGNAL            (FIRST_LC_RX_TASK_SIGNAL +2)
#define LC_HANDLE_TIMEOUT_SIGNAL                (FIRST_LC_RX_TASK_SIGNAL +3)
#define LC_HANDLE_ESCO_ACK_SIGNAL               (FIRST_LC_RX_TASK_SIGNAL +4)
#define LC_HANDLE_ESCO_PACKET                   (FIRST_LC_RX_TASK_SIGNAL +5)
#define LC_INVOKE_ESCO_SCHEDULER_SIGNAL         (FIRST_LC_RX_TASK_SIGNAL +6)
#define LC_INVOKE_LOAD_AHEAD_SCO_FIFO_SIGNAL    (FIRST_LC_RX_TASK_SIGNAL +7)
#define LC_HANDLE_RX_SCO_DATA_SIGNAL            (FIRST_LC_RX_TASK_SIGNAL +8)
#define LC_HANDLE_RX_SCO_ERR_DATA_SIGNAL        (FIRST_LC_RX_TASK_SIGNAL +9)
#define SOH_PLAY_PCM_BUFFER                     (FIRST_LC_RX_TASK_SIGNAL +10)
#define ECNR_SIGNAL                             (FIRST_LC_RX_TASK_SIGNAL +11)
#define LC_ENTER_SSR                            (FIRST_LC_RX_TASK_SIGNAL +12)
#define LC_EXIT_SSR                             (FIRST_LC_RX_TASK_SIGNAL +13)
#define LC_LMP_ENTER_SSR                        (FIRST_LC_RX_TASK_SIGNAL +14)
#define LC_ENTER_SM_MODE                        (FIRST_LC_RX_TASK_SIGNAL +15)
#define LC_ENTER_DSM_MODE                       (FIRST_LC_RX_TASK_SIGNAL +16)
#define LC_SWITCH_ROLE_WAIT_TO_SIGNAL           (FIRST_LC_RX_TASK_SIGNAL +17)
#define LC_HANDLE_CODEC_SCO_DATA_SIGNEL         (FIRST_LC_RX_TASK_SIGNAL +18)
#define LC_HANDLE_CODEC_ESCO_DATA_SIGNEL        (FIRST_LC_RX_TASK_SIGNAL +19)
#ifndef LE_MODE_EN
#ifndef _CCH_LPS_
#define LAST_LC_RX_TASK_SIGNAL                  (FIRST_LC_RX_TASK_SIGNAL +19)
#else
#define LC_ENTER_LPS_MODE                       (FIRST_LC_RX_TASK_SIGNAL +20)
#define LC_ENTER_LPS_STATE                      (FIRST_LC_RX_TASK_SIGNAL +21)
#define LAST_LC_RX_TASK_SIGNAL                  (FIRST_LC_RX_TASK_SIGNAL +21)
#endif
#else
#define LL_HANDLE_RECD_PACKET_SIGNAL            (FIRST_LC_RX_TASK_SIGNAL +20)
#ifndef _CCH_LPS_
#define LAST_LC_RX_TASK_SIGNAL                  (FIRST_LC_RX_TASK_SIGNAL +20)
#else
#define LC_ENTER_LPS_MODE                       (FIRST_LC_RX_TASK_SIGNAL +21)
#define LC_ENTER_LPS_STATE                      (FIRST_LC_RX_TASK_SIGNAL +22)
#define LAST_LC_RX_TASK_SIGNAL                  (FIRST_LC_RX_TASK_SIGNAL +22)
#endif
#endif

/* lc_tx_task_handle */
#define FIRST_LC_TX_TASK_SIGNAL                 (800)
#define LC_HANDLE_HOST_DATA_PKT                 (FIRST_LC_TX_TASK_SIGNAL +0)
#define LC_HANDLE_ACK_RECD_SIGNAL               (FIRST_LC_TX_TASK_SIGNAL +1)
#define LC_HANDLE_MAX_RETRY_EXP_SIGNAL          (FIRST_LC_TX_TASK_SIGNAL +2)
#define LC_HANDLE_HOST_SYNCHRONOUS_PKT          (FIRST_LC_TX_TASK_SIGNAL +3)
#define LC_HANDLE_LOOP_BACK_ACK_RECD_SIGNAL     (FIRST_LC_TX_TASK_SIGNAL +4)
#define LC_INVOKE_GENERATE_SCO_DATA_SIGNAL      (FIRST_LC_TX_TASK_SIGNAL +5)
#define LC_RESUME_DATA_TRANSFER_SIGNAL          (FIRST_LC_TX_TASK_SIGNAL +6)
#define LC_INVOKE_DELAYED_SCO_SCHEDULER_ENTRY_FREE (FIRST_LC_TX_TASK_SIGNAL +7)
#define LC_HANDLE_RESCHEDULE                    (FIRST_LC_TX_TASK_SIGNAL +8)
#define LC_HANDLE_MARK_ACTIVE                   (FIRST_LC_TX_TASK_SIGNAL +9)
#define LC_QUEUE_PDU_SIGNAL                     (FIRST_LC_TX_TASK_SIGNAL +10)
#define LC_DYNAMIC_PROGRAM_TPOLL                (FIRST_LC_TX_TASK_SIGNAL +11)
#define LC_SWITCH_ROLE_COMPLETE_SIGNAL          (FIRST_LC_TX_TASK_SIGNAL +12)
#define LC_HANDLE_OPTIMIZE_DATA_TX_DURING_SNIFF (FIRST_LC_TX_TASK_SIGNAL +13)
#define LC_PDU_SENT_SIGNAL                      (FIRST_LC_TX_TASK_SIGNAL +14)
#ifndef LE_MODE_EN
#define LAST_LC_TX_TASK_SIGNAL                  (FIRST_LC_TX_TASK_SIGNAL +14)
#else
#define LL_HANDLE_HOST_DATA_PKT                 (FIRST_LC_TX_TASK_SIGNAL +15)
#define LAST_LC_TX_TASK_SIGNAL                  (FIRST_LC_TX_TASK_SIGNAL +15)
#endif

/* logger_task_handle */
#define FIRST_LOGGER_TASK_SIGNAL                (900)
#define LOGGER_LOG_DATA_SIGNAL                  (FIRST_LOGGER_TASK_SIGNAL +0)
#define UART_DBG_DATA_SIGNAL                    (FIRST_LOGGER_TASK_SIGNAL +1)

#ifndef _CCH_LPS_
#define LAST_LOGGER_TASK_SIGNAL                 (FIRST_LOGGER_TASK_SIGNAL +1)
#else
#define LOGGER_ENTER_LPS_MODE             	    (FIRST_LOGGER_TASK_SIGNAL +2)
#define LAST_LOGGER_TASK_SIGNAL                 (FIRST_LOGGER_TASK_SIGNAL +2)
#endif

/* hci_ch_as_task */
#define FIRST_CH_AS_TASK_SIGNAL                 (1000)
#define LC_PKT_RECD_SIGNAL                      (FIRST_CH_AS_TASK_SIGNAL + 1)
#define LC_NEW_PKT_RECD_SIGNAL                  (FIRST_CH_AS_TASK_SIGNAL + 2)
#define CH_AS_TASK_ENTER_LPS_MODE               (FIRST_CH_AS_TASK_SIGNAL + 3)
#define CH_AS_TASK_ISSC_SEC                     (FIRST_CH_AS_TASK_SIGNAL + 4)
#define CH_AS_TASK_LE_RSSI                      (FIRST_CH_AS_TASK_SIGNAL + 5)
#define LAST_CH_AS_TASK_SIGNAL                  (FIRST_CH_AS_TASK_SIGNAL + 5)



/*
 * This macro is used by OS module as a max bound on the list of signals
 * it can receive from external tasks
 */
#define BT_FW_EXT_RESERVED_SIGNALS              (1100)


/*HCI module signals */

#define HCI_TD_SYNC_LOST_SIGNAL                (BT_FW_EXT_RESERVED_SIGNALS + 1)

#define HCI_CMD_BUFF_FULL_SIGNAL               (BT_FW_EXT_RESERVED_SIGNALS + 2)

#define HCI_CMD_STATUS_EVENT_SIGNAL            (BT_FW_EXT_RESERVED_SIGNALS + 3)

#define HCI_CMD_COMPLETE_EVENT_SIGNAL          (BT_FW_EXT_RESERVED_SIGNALS + 4)

#define HCI_GENERATE_HCI_EVENTS_SIGNAL         (BT_FW_EXT_RESERVED_SIGNALS + 5)

#define HCI_DELIVER_HCI_EVENT_SIGNAL           (BT_FW_EXT_RESERVED_SIGNALS + 6)

#define HCI_DELIVER_ACL_DATA_SIGNAL            (BT_FW_EXT_RESERVED_SIGNALS + 7)

#define HCI_DELIVER_SYNCHRONOUS_DATA_SIGNAL    (BT_FW_EXT_RESERVED_SIGNALS + 8)

#define HCI_DELIVER_LE_ACL_DATA_SIGNAL         (BT_FW_EXT_RESERVED_SIGNALS + 9)

#define BT_FW_HCI_SIGNALS                      (BT_FW_EXT_RESERVED_SIGNALS + 50)

/*LMP module signals */

#define LMP_HARDWARE_LVL_CONN_RECD_SIGNAL       (BT_FW_HCI_SIGNALS + 0)

#define LMP_FHS_PKT_RECD_SIGNAL                 (BT_FW_HCI_SIGNALS + 1)

#define LMP_PDU_RECD_SIGNAL                     (BT_FW_HCI_SIGNALS + 2)

#define LMP_AFH_PROC_COMPLETION_SIGNAL          (BT_FW_HCI_SIGNALS + 4)

#define LMP_SWITCH_ROLE_COMPLETE_SIGNAL         (BT_FW_HCI_SIGNALS + 9)

#define LMP_HANDLE_CQDD_SIGNAL                  (BT_FW_HCI_SIGNALS + 10)

#define LMP_PDU_ACK_RECD_SIGNAL                 (BT_FW_HCI_SIGNALS + 11)

#define LMP_HANDLE_PAGESCAN_NCTO_SIGNAL         (BT_FW_HCI_SIGNALS + 12)

/**
 * @brief LMP generic signal.
 *
 * This signal's handler uses OS_SIGNAL#param and OS_SIGNAL#ext_param.\n
 * OS_SIGNAL#param is the callback function with OS_SIGNAL#ext_param as the
 * parameter. The callback function's signature refers to
 * LMP_GENERIC_SIGNAL_HDLR.
 */
#define LMP_GENERIC_SIGNAL                      (BT_FW_HCI_SIGNALS + 20)

#define BT_FW_LMP_SIGNALS                       (BT_FW_HCI_SIGNALS + 50)

#define LC_HANDLE_TX_FIFO_STATUS_SIGNAL         (BT_FW_LMP_SIGNALS + 2)

#define LC_HANDLE_RX_FIFO_STATUS_SIGNAL         (BT_FW_LMP_SIGNALS + 3)

#define LC_HANDLE_BB_RES_FREE_SIGNAL            (BT_FW_LMP_SIGNALS + 5)

#define LC_HANDLE_START_SCAN_MODE_SIGNAL        (BT_FW_LMP_SIGNALS + 6)

#define BT_FW_LC_SIGNALS                        (BT_FW_LMP_SIGNALS + 50)

#ifdef ESCO_DISC_DEBUG
#define LMP_END_OF_ESCO_WINDOW_SIGNAL           (BT_FW_LMP_SIGNALS + 49)
#endif /* ESCO_DISC_DEBUG */
#define PKT_TO_SIGNAL                           (BT_FW_LC_SIGNALS + 10)

#define HCI_TD_ACL_TO_HOST_TEST_SIGNAL      (BT_FW_LC_SIGNALS + 11)

/** The callback function type used by LMP generic signal. */
typedef void (*LMP_GENERIC_SIGNAL_HDLR)(void *);

#endif /* __BT_FW_SIGNALS_H__ */

