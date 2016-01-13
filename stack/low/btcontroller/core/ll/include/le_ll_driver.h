/***************************************************************************
 Copyright (C) Realtek
 This module is a confidential and proprietary property of Realtek and
 a possession or use of this module requires written permission of Realtek.
 ***************************************************************************/

/**
 * \file
 *  le_ll_driver.h (Low Energy Link Layer Hardware Controller Driver)
 *
 * \author
 */

/** \addtogroup Low Energy Link Layer Driver Module
 *  @{ */
#ifndef _LE_LL_DRIVER_H_
#define _LE_LL_DRIVER_H_
#include "le_hw_reg.h"
#include "array_queue.h"


#define LL_DRIVER_GEN_16BIT_RANDOM(output) \
{ \
    WR_LE_REG(LE_REG_RANDOM_NUM_CTRL, 0x05); \
    output = RD_LE_REG(LE_REG_RANDOM_NUM_L); \
}

#define LL_DRIVER_GEN_32BIT_RANDOM(output) \
{ \
    UINT32 value; \
    WR_LE_REG(LE_REG_RANDOM_NUM_CTRL, 0x05); \
    value = RD_LE_REG(LE_REG_RANDOM_NUM_L) | \
            (RD_LE_REG(LE_REG_RANDOM_NUM_H) << 16); \
    output = value; \
}

#define LL_DRIVER_TRANSLATE_625US_UNIT(duration_us) \
   (((duration_us) + LL_RF_TIMER_SLOT_UNIT - 1) / LL_RF_TIMER_SLOT_UNIT)

#define MAX_FW_BT_RX_INT_FIFO_UNITS    8 /* need 2^N */

/*===========================================================*/
/*  The Data Structure of HW Receiver Packet in ACL RX FIFO  */
/*===========================================================*/
typedef struct LE_HW_RX_PKT_TAIL_S_ {
    union {
        struct {
            UINT16 crc_err:1;       /* bit[0], the crc is error */
            UINT16 mic_err:1;       /* ========== for data channel ==========
                                       bit[1], the mic is error on data channel */
                                    /* ========== for adv. channel ==========
                                       bit[1], scan_rcv_nreso_inita (see LE_CAM.xls) */
            UINT16 duplicate:1;     /* bit[2], this pkt is duplcated */
            UINT16 empty_pdu:1;     /* ========== for data channel ==========
                                       bit[3], this pkt is an empty data pdu */
                                    /* ========== for adv. channel ==========
                                       bit[3], scan_rcv_all_ind (see LE_CAM.xls) */
            UINT16 adv_filtered:1;  /* bit[4], this pkt can be filtered by advertising policy */
            UINT16 len_err:1;       /* bit[5], the length is error */
            UINT16 adv_type_err:1;  /* bit[6], the advertising type is error */
            UINT16 decrypted:1;     /* bit[7], the rx pkt is decrypted */
#ifdef _NEW_BLE_HW_SPEC_FROM_150320_
            UINT16 rx_channel:6;    /* ========== for data channel ==========
                                       bit[13:8], this packet is received in which rx channel */
                                    /* ========== for adv. channel ==========
                                       bit[13], 1: privacy rule passed,
                                       bit[12:8], matched resolving list index */
            UINT16 multi_match:1;   /* bit[14], 1: match multiple entries in resolving list */
            UINT16 adv_channel:1;   /* bit[15], this packet is received in adv channel */
#else
            UINT16 rsvd:8;          /* bit[15:8], reserved */
#endif
        };
        UINT16 rx_status;           /* the status of rx packet */
    };
    UINT16 rssi;                    /* the rssi value */
} LE_HW_RX_PKT_TAIL_S, *PLE_HW_RX_PKT_TAIL_S;

typedef struct FW_BT_RX_INT_UNIT_S_ {
    union {
        struct {
            UINT16 type:1;
            UINT16 tx_cleanup_rdy:1;
            UINT16 must_chk_rxlen:1;
            UINT16 conn_entry:5;
            UINT16 counter:8;
        };
        UINT16 state;
    };

    union {
        UINT16 reg0_value;
        UINT16 rx_payload_reg_value;
        UINT16 ce_word_cnt_reg_value;
    };
    union {
        UINT16 reg1_value;
        UINT16 rx_status_reg_value;
        UINT16 ce_status_reg_value;
        LE_REG_S_STATUS_CE_END_EVENT ce_end_status;
    };

#ifdef _NEW_BLE_HW_SPEC_FROM_150320_
    union {
        UINT16 reg2_value;
        UINT16 anch_diff_rpt0_value;
        LE_REG_S_ANCH_DIFF_RPT0 anch_diff_rpt0;
    };
    union {
        UINT16 reg3_value;
        UINT16 anch_diff_rpt1_value;
        LE_REG_S_ANCH_DIFF_RPT1 anch_diff_rpt1;
    };
#endif

#ifdef _DAPE_TEST_NEW_HW_READ_RSSI_STACK
    UINT16 rssi; //only valid for legacy (from chinwen)
#endif
} FW_BT_RX_INT_UNIT_S, *PFW_BT_RX_INT_UNIT_S;

typedef struct FW_BT_RX_INT_FIFO_S_ {
    FW_BT_RX_INT_UNIT_S unit[MAX_FW_BT_RX_INT_FIFO_UNITS];
    UINT8 wptr;
    UINT8 rptr;
    UINT8 count;
    UINT8 found_legacy;
} FW_BT_RX_INT_FIFO_S, *PFW_BT_RX_INT_FIFO_S;

extern FW_BT_RX_INT_FIFO_S fw_bt_rx_int_fifo;

#ifdef LE_ISSUE_INSTR_IN_ORDER_ON_COMPLT
typedef struct LE_INSTR_ENTITY_CONN_
{
    INT8 conn_entry;
} LE_INSTR_ENTITY_CONN;

typedef struct LE_INSTR_ENTITY_UPDATE_
{
    INT8 conn_entry;
    UINT8 gen_pdu : 1;
    UINT8 reserved : 7;
    UINT16 instant;
} LE_INSTR_ENTITY_UPDATE;

/**
 * @brief LE instruction entity.
 *
 * An LE instruction entity type must have \c conn_entry as the first data
 * member.
 */
typedef union LE_INSTR_ENTITY_
{
    LE_INSTR_ENTITY_CONN conn;
    LE_INSTR_ENTITY_UPDATE update;
} LE_INSTR_ENTITY;

void le_instr_queue_init();
LE_INSTR_ENTITY ll_driver_issue_next_kill_connection();
#endif /* LE_ISSUE_INSTR_IN_ORDER_ON_COMPLT */

UINT8 ll_driver_set_le_instruction(UINT8 insctruction_code);
UINT8 ll_driver_advertising_schedule_bzdma(UINT8 le_pdu_type);
UINT8 ll_driver_data_channel_schedule_bzdma(UINT8 conn_entry,UINT8 is_terminate);
extern void (*ll_driver_channel_map_update)(UINT8 conn_entry_id,UINT32 instant);
extern void (*ll_driver_connection_update)(UINT8 conn_entry_id,UINT32 instant);
void ll_driver_create_connection(UINT8 conn_entry);
void ll_driver_create_connection_cancel(void);
extern void (*ll_driver_disable_advertising)(void);
void ll_driver_disable_scanning(void);
extern void (*ll_driver_disconnect_connection)(UINT8 conn_entry_id,UINT8 reason);
#ifdef _CCH_RETENTION_FLOW_FOR_DLPS_LE_
void ll_driver_enable_advertising(UCHAR in_lps);
void ll_driver_enable_scanning(UCHAR in_lps);
#else
void ll_driver_enable_advertising(void);
void ll_driver_enable_scanning(void);
#endif
extern void (*ll_driver_kill_connection)(UINT8 conn_entry_id);
UINT32 ll_driver_read_cam(UINT16 cam_addr);
UINT8 ll_driver_write_cam(UINT16 cam_addr,UINT32 wdata);

void ll_driver_reset_white_list(void);
UINT8 ll_dev_addr_list_center_add_list_entry(UINT8 list_type, UINT8 addr_type, UINT8 *addr);
#ifdef _BT4_2_PRIVACY_FEATURE_SUPPORT_
UINT8 ll_driver_add_dev_to_list(UINT8 list_type, UINT8 addr_type, UINT8 *addr, UINT8 *local_irk, UINT8 *peer_irk);
BOOLEAN ll_driver_remove_dev_from_list(UINT8 list_type,UINT8 addr_type,UINT8 * addr);
#else
UINT8 ll_driver_add_dev_to_list(UINT8 list_type,UINT8 addr_type,UINT8 * addr);
void ll_driver_remove_dev_from_list(UINT8 list_type,UINT8 addr_type,UINT8 * addr);
#endif
void ll_driver_reset_dev_list(UINT8 list_type);
UINT8 ll_driver_search_dev_from_list(UINT8 list_type,UINT8 addr_type,UINT8 * addr);
void ll_driver_clear_all_duplicated_flags_in_list(UINT8 list_type);

void ll_driver_get_session_key(UINT16 * key,UINT16 * text,UINT16 * session_key);
void ll_driver_install_encryption_info(UINT8 master_role,UINT8 conn_entry,
                                       UINT16 * iv,UINT16 * sess_key);
void ll_driver_set_random_address(void);
void ll_driver_fill_conn_win_size_offset(UINT8 conn_win_size_offset);

void ll_driver_block_legacy_for_le(UINT8 block_en);
void ll_driver_block_legacy_slot_for_le(UINT8 slot_number);

UINT32 ll_driver_read_slave_clock(void);
UINT32 ll_get_slave_clock_diff(UINT32 exp_clk, UINT32 cur_clk);

void ll_driver_set_le_flow_stop(UINT8 on);
void ll_driver_set_le_link_tx_power(UINT8 conn_entry,UINT8 tx_gain);
#ifndef _GET_LE_SLV_ADDR_TYPE_FROM_HW
#ifdef _DAPE_GET_LE_SLV_ADDR_TYPE_FROM_FW
UINT8 ll_driver_search_dev_type_from_list(UINT8 list_type, UINT8 *addr_type, UINT8 *addr);
#endif
#endif
#ifdef _BT4_2_PRIVACY_FEATURE_SUPPORT_
void ll_driver_set_resolvable_address_regenerate(void);
void ll_driver_set_address_resolution_enable(BOOLEAN isEnable);
void ll_driver_dump_hw_resolving_list(void);
void ll_driver_store_tx_resolvable_private_address(void);
UINT8 ll_driver_store_rx_resolvable_private_address(UINT8 *pHdr, UINT8 offset);
#endif

#ifdef _SUPPORT_LBT_FUNC_FOR_RF_ADAPTIVITY_
void ll_driver_enabe_ce_lbt(UINT8 conn_entry, UINT8 tx_power_index);
#endif

#ifdef _NEW_BZDMA_FROM_V8_
UINT8 ll_driver_read_bzdma_rptr_from_cam(UINT8 conn_entry);
void ll_driver_write_bzdma_rptr_to_cam(UINT8 conn_entry, UINT8 new_rptr);
#ifdef _BT4_2_DATA_EXTENSION_FEATURE_SUPPORT_
void ll_driver_enable_data_extension(UINT8 conn_entry, UINT8 enable);
#endif /* end of #ifdef _BT4_2_DATA_EXTENSION_FEATURE_SUPPORT_ */
#if defined(_NEW_BLE_HW_SPEC_FROM_150320_) && defined(_LE_WAKE_FROM_SLAVE_LATENCY_)
void ll_driver_wake_from_slave_latency(void);
#endif /* end of #ifdef _NEW_BLE_HW_SPEC_FROM_150320_ */
#endif /* end of #ifdef _NEW_BZDMA_FROM_V8_ */
#endif /* end of #ifndef _LE_LL_DRIVER_H_ */
