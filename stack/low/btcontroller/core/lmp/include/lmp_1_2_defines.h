/***************************************************************************
 Copyright (C) Mindtree Ltd.
 This module is a confidential and proprietary property of MindTree and
 a possession or use of this module requires written permission of MindTree.
 ***************************************************************************/

/**
 * \file
 *  Structures and Macro definitions specific to BT1.2 LMP layer. It also
 *  contains the function interface.
 */

/** \addtogroup lmp_external
 *  @{ */
#ifndef _LMP_1_2_DEFINES_H_
#define _LMP_1_2_DEFINES_H_
#include "lmp_spec_defines.h"
#include "bt_fw_types.h"
#include "lmp_defines.h"
#ifdef VER_3_0
#include "lmp_3_0.h"
#endif
#ifdef _SUPPORT_VER_4_1_
#include "lmp_4_1.h"
#endif
#ifdef VER_3_0
#define LMP_HANDLE_3_0_INCOMING_PDU \
                 lmp_handle_3_0_incoming_pdu
#else
#define LMP_HANDLE_3_0_INCOMING_PDU(lmp_pdu_ptr,ce_index) \
                 BT_FW_ERROR
#endif
#ifdef _SUPPORT_VER_4_1_
#define LMP_HANDLE_4_1_INCOMING_PDU \
                 lmp_handle_4_1_incoming_pdu
#else
#define LMP_HANDLE_4_1_INCOMING_PDU(lmp_pdu_ptr,ce_index) \
                 BT_FW_ERROR
#endif

#define LMP_HANDLE_2_1_INCOMING_PDU \
                 lmp_handle_2_1_incoming_pdu

#define LMP_HANDLE_1_2_INCOMING_PDU \
                 lmp_handle_1_2_incoming_pdu

#define LMP_HANDLE_1_2_PDU_ACK_RECD \
        lmp_handle_1_2_pdu_ack_recd

#ifdef COMPILE_AFH_HOP_KERNEL
#define LMP_HANDLE_AFH_PROC_COMPLETION \
                lmp_handle_afh_proc_completion

#define LMP_DECIDE_FOR_LMP_AFH_PDU(ce_index) \
                        lmp_decide_for_lmp_afh_pdu(ce_index)


#else
#define LMP_DECIDE_FOR_LMP_AFH_PDU(ce_index)
#define LMP_HANDLE_AFH_PROC_COMPLETION(x)
#endif

#ifdef COMPILE_ESCO
#define LMP_HANDLE_ESCO_DISCONNECT \
            lmp_handle_esco_disconnect
#define LMP_DISCONNECT_ESCO_LINKS  \
            lmp_disconnect_esco_links  
#else
#define LMP_HANDLE_ESCO_DISCONNECT(a,b,c)
#define LMP_DISCONNECT_ESCO_LINKS(a,b)
#endif /* COMPILE_ESCO */                 
        
#define LMP_MIN_AFH_SLOTS                                   96
#define LMP_MIN_DISTANCE_BETWEEN_SET_AFH_IN_SECONDS         1

#define LMP_MIN_NUM_OF_AFH_CHANNEL_ALLOWED                  20

#define INVALID_MAP_INDEX                                   0xFF
#define INVALID_AFH_MODE                                    0xFF
#define LMP_MAX_NUM_OF_MAP_TABLES                           10
#define AFH_ENABLE                                          1
#define AFH_DISABLE                                         0

#define LMP_CH_REPORTING_MIN_INTERVAL                       (1600 * 10)  /* 10 secs */
#define LMP_CH_REPORTING_MAX_INTERVAL                       (1600 * 10)  /* 10 secs */
/* Resolution of min and max timers is 625us */

/* New error codes as per V1.2 */
#define LMP_CHANNEL_CLASSIFICATION_NOT_SUPPORTED_ERROR      0x2E
#define LMP_RESERVED_SLOT_VIOLATION_ERROR                   0x34

/* Local assessment cycle time period */
#define AFH_LA_CYCLE_PERIOD                                 3000   /* 3 secs */

/* Resolution of this timer is seconds. */

#ifdef COMPILE_CHANNEL_ASSESSMENT
/* 
 * Period before which host classification update should be used. Spec says
 * 10 seconds - we are using a value smaller than that.
 */
#define AFH_HOST_UPDATE_TIMEOUT_PERIOD                      8000
#endif /* COMPILE_CHANNEL_ASSESSMENT */

/* esco related defines */
#define MAX_ESCO_DATA_Q_LENGTH                    (BT_FW_TOTAL_SYNCHRONOUS_PKTS_FROM_HOST)
#define LMP_MAX_ESCO_CONN_HANDLES                           LMP_MAX_CONN_HANDLES
#define LMP_MAX_ESCO_NUM_OF_AM_ADDR                         7     
#define LMP_MAX_ESCO_CONN_ENTITIES                          3//6
/* Max number of eSCO connections */
#define LMP_MAX_ESCO_CONNECTIONS                            3//6 

#define LMP_ESCO_MANDATORY_FEATURE                          0x80 
#define LMP_ESCO_EV4_PACKET_FEATURE                          0x01
#define LMP_ESCO_EV5_PACKET_FEATURE                          0x02

#define EDR_ACL_2MBPS_FEATURE                                0x02
#define EDR_ACL_3MBPS_FEATURE                                0x04

#define EDR_ACL_3_SLOT_FEATURE                               0x80

#define EDR_ACL_5_SLOT_FEATURE                               0x01
#define EDR_ESCO_2MBPS_FEATURE                               0x20
#define EDR_ESCO_3MBPS_FEATURE                               0x40
#define EDR_ESCO_3_SLOT_FEATURE                              0x80

/* Packet types defined for eSCO connection at the LMP level */
#define LMP_EV3                                                 0X07
#define LMP_EV4                                                 0x0C
#define LMP_EV5                                                 0X0D
#define LMP_DEFAULT_ESCO_PKT                                    0x00

#define LMP_2_EV3                                              0x26
#define LMP_3_EV3                                              0x37
#define LMP_2_EV5                                              0x2c
#define LMP_3_EV5                                              0x3d


#define ALL_LMP_ESCO_PKT_TYPES               (LMP_EV3 | LMP_EV4 | LMP_EV5)
/* Max and Min lengths of eSCO packet types */
#define ESCO_PKT_MIN_LEN                                        0
#define EV3_PKT_MAX_LEN                                         30
#define EV4_PKT_MAX_LEN                                         120
#define EV5_PKT_MAX_LEN                                         180

/* Defines for the negoation_state during esco connection establishment */
#define LMP_ESCO_INIT_NEG_STATE                                 0
#define LMP_ESCO_DEFAULT_NEG_STATE                              1
#define LMP_ESCO_RES_SLOT_VIOLATION_NEG_STATE                   2
#define LMP_ESCO_MAX_LAT_VIOLATION_NEG_STATE                    3  
#define LMP_ESCO_PARAM_NOT_SUPPORTED_NEG_STATE                  4 


#define BANDWIDTH_6400                                          0x1F40
/* #define SCO_LINK                                                0 */
/* #define HCI_ESCO_LINK                                           1 */

#define PTT_ENABLE                                            0x01
#define PTT_DISABLE                                           0x00


/* Maximum number of times the esco parameters will be negotiated */
#define LMP_MAX_NUM_OF_ESCO_NEGOTIATIONS                            2

/* Maximum number of Retransmissions allowed */
#define LMP_MAX_NUM_OF_RETRANSMISSIONS                              5

/* Default value of Tesco in number of slots */
#define LMP_DEFAULT_TESCO_VALUE                                     24

/* Maximum allowed value of Tesco */
#define LMP_MAX_TESCO_VALUE                                             254

/* Maximum allowed value of Desco */
#define LMP_MAX_DESCO_VALUE                                             254

/* Defines for Timing_control_flag in the esco link req pdu */
#define ESCO_TC_FLAG_INIT1                                                                   0x00    
#define ESCO_TC_FLAG_INIT2                                                                   0x02

/* Maximum number of Esco links supported over Codec */
#define MAX_NUMBER_OF_CODEC_LINKS                                  1

/** Defines for ESCO Negotiation */
#define RETX_EFFORT_CAUSE      0x0001   
#define MASTER_PKT_CAUSE      0x0002      
#define SLAVE_PKT_CAUSE         0x0004             

#ifdef COMPILE_AFH_HOP_KERNEL

typedef struct
{
    UCHAR afh_map[LMP_AFH_MAP_SIZE];
    UCHAR use_count;
    UCHAR use_sts;
}LMP_AFH_MAP_TABLE;

#endif

#ifdef COMPILE_ESCO


typedef struct
{
    UINT16 status;
    UINT16 ce_index;
}LMP_ESCO_AM_ADDR_TO_CE_INDEX_TABLE;

typedef struct
{
    UINT16 status;
    UINT16 ce_index; 
}LMP_ESCO_CONN_HANDLE_TO_CE_INDEX_TABLE;

typedef enum
{
    NO_CONNECTION, 
    ADDING_ESCO_LINK,
    WAITING_FOR_ACCEPT_SYNC_CONN,
    CHANGING_ESCO_PARAMS,
    ESCO_DISCONNECTING,
    ESCO_CONNECTED
}ESCO_CE_CONN_STATUS;

typedef enum
{
    ACTIVE,
    INACTIVE
}ESCO_WINDOW_STATUS;


typedef struct
{
    UCHAR read_index ; /* The index till which packet has been written 
                        * to Scheduler */
    UCHAR write_index ;/* The index to which packets have been queued */
    UCHAR ack_index;   /* The index to which ACK has been received */
    UCHAR q_length ;   /* The no. of packets waiting for ack queue */
    UCHAR pending_length; /* The no of data packets waiting to be loaded to 
                             the scheduler */
    UCHAR wait_index; /* The index to which packets have been waited for free */
    UCHAR wait_cnt;   /* The no. of packets waiting for free */  

    UINT16 fragmented_data_ptr; /* The pointer to the fragment at read_index 
                                 * that has been loaded to the FIFO 
                                 */
    UINT16 ack_fragmented_data_ptr; /* The pointer to the fragment at ack_index
                                     * till which ack has been received 
                                     */ 
    HCI_SYNC_DATA_PKT *esco_data_pkt_Q[MAX_ESCO_DATA_Q_LENGTH] ;
}LMP_ESCO_DATA_Q ;


typedef struct lmp_esco_connection_entity_struct
{
    ESCO_CE_CONN_STATUS status; /* the connection status of eSCO entity*/
    UCHAR entity_status; /* the entity status (UNASSIGNED or ASSIGNED) */ 
    UCHAR bb_esco_connection; /* bb connection is binded to eSCO connection ? */
    UINT16 ce_index ; /* the index of binded connection entity */

    /* [LMP relative] */
    UCHAR esco_handle; /* the eSCO handle (assigned by the master) */
    UCHAR lt_addr; /* the eSCO LT_ADDR (assigned by the master) */    
    UCHAR timing_control_flags; /* the timing control flags */
    UCHAR desco; /* DeSCO (unit:625us) */
    UCHAR tesco; /* TeSCO (unit:625us) */
    UCHAR wesco; /* WeSCO (unit:625us) */
    UCHAR m_to_s_packet_type; /* the eSCO packet type M->S */
    UCHAR s_to_m_packet_type; /* the eSCO packet type S->M */
    UINT16 m_to_s_packet_length; /* the eSCO packet size M->S */
    UINT16 s_to_m_packet_length; /* the eSCO packet size S->M */
    UCHAR air_mode; /* the air mode */      
    UCHAR negotiation_flag; /* negotiation flag */
    UCHAR negotiation_count; /* the counter of negotiation */
    UCHAR esco_window; /* the window length of eSCO 
                          (reserved slots (m2s,s2m) + WeSCO) (unit:625us)*/

    UCHAR use_codec; /* use external codec or not */
    UCHAR num_of_completed_esco_packets; /* number of complete eSCO tx pkts */

    /* [HCI relative] */
    UINT16 conn_handle; /* the connection handle of HCI SCO pkt */
    UINT32 tx_bandwidth; /* the transmissit bandwidth in octects per sec */
    UINT32 rx_bandwidth; /* the receive bandwidth in octects per sec */
    UINT16 max_latency; /* the maximum latency */
    UINT16 voice_setting; /* the voice settings of HCI command */
    UCHAR retransmission_effort; /* the retransmission effort */ 
    UCHAR erroneous_data_reporting; /**<enabled/disabled erroneous data
                                      reporting for this connection */
    UINT16 packet_type; /* the preferred packet type from hci host */
                                    
	UINT16 no_data_count; /* the counter when the connection entity miss data */
    UINT32 next_instant; /* the instant value of next anchor point of eSCO (unit:625us) */

    LMP_ESCO_DATA_Q  esco_data_q; /* the data queue of eSCO */
    ESCO_WINDOW_STATUS esco_window_status; /* the status of eSCO window */
    ESCO_WINDOW_STATUS esco_continous_status; /* the status of eSCO continous */

    UINT32 host_pkt_empty_cnts; /* the counter when the host pkt is empty */
}LMP_ESCO_CONNECTION_ENTITY;

typedef struct lmp_esco_negotiation_history
{
    UCHAR possible_cause_of_rejection;
    UCHAR cause_to_constraint;
    UINT16 possible_disallowed_master_pkt;
    UINT16 possible_disallowed_slave_pkt;
    UINT16 max_latency;
    UINT16 retx_window;
}LMP_ESCO_NEGOTIATION_HISTORY;

#endif /* COMPILE_ESCO */

UCHAR lmp_handle_1_2_incoming_pdu(LMP_PDU_PKT *lmp_pdu_ptr, UINT16 ce_index);

void lmp_handle_1_2_pdu_ack_recd(LMP_PDU_PKT *lmp_pdu_pkt, UCHAR piconet_id,
                                                            UINT16 ce_index);
void lmp_send_lmp_accepted_ext(UINT16 ce_index,UCHAR opcode,UCHAR ext_opcode,
                                                                UCHAR trans_id);
void lmp_send_lmp_not_accepted_ext(UINT16 ce_index,UCHAR opcode,
                                UCHAR ext_opcode,UCHAR trans_id,UCHAR reason);

UINT32 lmp_get_slots_to_delay_afh(UINT16 ce_index);
void lmp_handle_set_afh_pdu(LMP_PDU_PKT *lmp_pdu_ptr, UINT16 ce_index);
UCHAR lmp_handle_channel_classification_req_pdu(LMP_PDU_PKT *lmp_pdu_ptr, 
                                                    UINT16 ce_index);
UCHAR lmp_handle_channel_classification_pdu(LMP_PDU_PKT *lmp_pdu_ptr, 
                                                    UINT16 ce_index);
void lmp_start_afh_map_updation(UCHAR *map,UINT16 ce_index, UCHAR afh_mode);
void lmp_delete_afh_timer(UINT16 ce_index);
void lmp_update_map(UCHAR afh_mode, UINT16 ce_index, UCHAR force_flag);
void lmp_decide_for_lmp_afh_pdu(UINT16 ce_index);
void lmp_handle_afh_instant_timer(TimerHandle_t timer_handle);
void lmp_handle_ack_received_for_set_afh_pdu(UINT16 ce_index);

UCHAR lmp_calculate_afh_instant(UINT16 ce_index);
void  lmp_send_set_afh_pdu(UINT16 ce_index, UINT8 afh_mode);

void  lmp_send_ch_cl_req_pdu(UCHAR afh_mode, UINT16 ce_index);
void lmp_handle_ack_rcvd_for_accepted_ext(LMP_PDU_PKT *lmp_pdu_ptr,
        UINT16 ce_index);
UCHAR lmp_handle_not_accepted_ext_pdu(LMP_PDU_PKT *lmp_pdu_ptr,
        UINT16 ce_index);

UCHAR lmp_handle_accepted_ext_pdu(LMP_PDU_PKT *lmp_pdu_ptr,UINT16 ce_index);

#ifdef COMPILE_ESCO
UCHAR lmp_handle_esco_link_req_pdu(LMP_PDU_PKT *lmp_pdu_ptr, UINT16 ce_index);

UCHAR lmp_handle_esco_link_req_not_accepted(LMP_PDU_PKT *lmp_pdu_ptr,
                                                     UINT16 ce_index);
UCHAR lmp_handle_remove_esco_link_req_pdu(LMP_PDU_PKT *lmp_pdu_ptr,
                                                     UINT16 ce_index);
UCHAR lmp_handle_remove_esco_link_req_accepted(LMP_PDU_PKT *lmp_pdu_ptr,
                                                          UINT16 ce_index);
/** 
 * Allocates an Active Member Address (AM_ADDR, also called as  Logical
 * Transport Address - LT_ADDR).
 * 
 * \param p_am_addr Returned Active Member Address.
 * \param logical_piconet_id Logical Piconet ID of the link.
 * 
 * \return API_SUCCESS, if the operation is successful. API_FAILURE,
 *         otherwise.
 */
#define lmp_get_esco_am_addr(p_am_addr, logical_piconet_id) \
    lmp_get_am_addr_ppi((p_am_addr), (logical_piconet_id))
#define lmp_get_esco_am_addr_ppi(p_am_addr, phy_piconet_id) \
    lmp_get_am_addr_ppi((p_am_addr), (phy_piconet_id))

/** 
 * Frees the given Active Member Address (AM_ADDR, also called as Logical
 * Transport Address - LT_ADDR).
 * 
 * \param am_addr Active Member Address to be freed.
 * \param logical_piconet_id Logical Piconet ID of the AM_ADDR to be freed.
 * 
 * \return API_SUCCESS, if the operation is successful. API_FAILURE,
 *         otherwise.
 */
#define lmp_put_esco_am_addr(am_addr, phy_piconet_id)   \
    lmp_put_am_addr_ppi((am_addr), (phy_piconet_id))
#define lmp_put_esco_am_addr_ppi(am_addr, phy_piconet_id)   \
    lmp_put_am_addr_ppi((am_addr), (phy_piconet_id))

API_RESULT lmp_allocate_esco_conn_handle(UINT16* conn_handle);

API_RESULT lmp_get_esco_ce_index_from_am_addr(UCHAR am_addr, UINT16 piconet_id,
                                                            UINT16* ce_index);

API_RESULT lmp_get_esco_ce_index_from_conn_handle(UINT16 conn_handle,
        UINT16* esco_ce_index);
API_RESULT lmp_get_esco_ce_index_from_ce_index(const UINT16 ce_index,
        const ESCO_CE_CONN_STATUS status, UINT16* esco_ce_index);

UCHAR lmp_esco_release_conn_handle(UINT16 conn_handle);

UINT16 lmp_allocate_esco_entity_from_ce_database(void);

UCHAR lmp_release_esco_entity_to_ce_database(UINT16 ce_index);

UCHAR lmp_init_esco_connection_entity(UINT16 ce_index, UCHAR esco_handle);

API_RESULT lmp_allocate_esco_am_addr(UCHAR *am_addr, UINT16 *ce_index, 
                                                    UCHAR phy_piconet_id);

void lmp_get_esco_params_from_ce(UCHAR *parameter_list,UINT16 esco_ce_index);
void lmp_sync_conn_accept_timeout_handler(TimerHandle_t timer_handle);
void lmp_validate_esco_link_req_pdu(LMP_PDU_PKT *lmp_pdu_ptr,
                                          UINT16 esco_ce_index,
                                          UINT16 ce_index,
                                          UCHAR *response_pdu,
                                          UCHAR *reason);
UCHAR lmp_create_esco_connection(UINT16 ce_index);
UCHAR lmp_kill_esco_connection(UINT16 acl_ce_index);
UCHAR lmp_handle_esco_disconnect(UINT16 conn_handle,UCHAR reason,
                                UCHAR *sent_cmd_status);
void lmp_validate_new_esco_link_req(UINT16 esco_ce_index,UINT16 ce_index,
                                     UCHAR *esco_link_req_response,
                                     UCHAR *reason);

API_RESULT lmp_generate_esco_params(UINT16 ce_index,UINT16 esco_ce_index,
                                      UINT32 tx_bandwidth,UINT32 rx_bandwidth,
                                      UINT16 max_latency,
                                      UCHAR retx_effort);
API_RESULT lmp_check_for_reserved_slot_violation(UCHAR desco,UCHAR tesco,
                                            UCHAR esco_window,UINT16  ce_index,
                                            UINT16  esco_ce_index);
UINT16 max_allowed_length(UINT16 pkt_type);
UINT16 esco_pktsize(UINT16 pkt_type);
UINT16 hci_to_lmp_packet_type(UINT16 packet_type);
UINT16 lmp_to_hci_packet_type(UINT16 packet_type);
void lmp_release_esco_resources(UINT16 ce_index,UINT16 esco_ce_index);
UCHAR lmp_generate_init_esco_params(UINT16 esco_ce_index);
void lmp_handle_esco_conn_setup_failed(UINT16 esco_ce_index,
                                              UINT16 ce_index, 
                                              UCHAR status);

void lmp_disconnect_esco_links(UINT16 ce_index, UCHAR reason);
#ifndef _CCH_SLOT_OFFSET_
UCHAR select_desco_val(UINT16 esco_ce_index,UCHAR ce_index,UCHAR *desco);
#endif
API_RESULT lmp_get_esco_ce_index_from_esco_handle(UCHAR esco_handle, 
                                                  UINT16* ce_index);
void lmp_reset_temp_esco_connection_entity(void);
void lmp_reset_esco_global_variables(void);
void update_esco_connection_entity(UINT16 ce_index,UINT16 esco_ce_index);
void reset_esco_ce_buffer(UINT16 esco_ce_index);

#ifndef COMPILE_SINGLE_SLOT_PACKETS_ONLY
void change_max_slot_with_esco(UINT16 acl_ce_index);
#endif

API_RESULT lmp_check_sync_conn_bandwidth(UCHAR consider_wesco);
API_RESULT lmp_check_res_slot_overlap(UCHAR start_of_window,UCHAR end_of_window,
                                  UINT16 acl_ce_index);
UINT16 lmp_get_allowed_synchronous_pkt_type(UINT16 acl_ce_index, 
                                            UINT16 local_host_packet_type);
API_RESULT check_esco_window_overlap(UCHAR start_of_window,UCHAR end_of_window,
                                     UINT16 ce_index,UINT16 esco_ce_index,
                                     UCHAR retx_overlap,UCHAR tesco,
                                     UCHAR new_gcd_of_tesco);
void lmp_update_max_tesco(void);
void lmp_update_gcd_of_tesco(void);
UINT32 lmp_find_gcd(UINT32 number1, UINT32 number2);
API_RESULT lmp_find_packet_length(UINT32 tx_bandwidth, UINT32 rx_bandwidth,
        UCHAR* tesco, UINT16* tx_pkt_length, UINT16* rx_pkt_length);
UINT16 lmp_find_new_max_latency(UINT32 rx_bandwidth,UINT32 tx_bandwidth,
                                UINT16 max_pkt_length,UCHAR max_window_size,
                                UCHAR retx_effort);
UINT16 lmp_find_packet_slots(UINT16 pkt_type);
void lmp_reset_negotiation_history(void);
void lmp_store_negotiation_history(LMP_ESCO_CONNECTION_ENTITY* self_parameters,
        LMP_PDU_PKT* recieved_pdu);
void lmp_generate_constraints_from_history(UINT16* max_latency,
        UINT16* disallowed_packet_type, UCHAR* retx_effort);
#endif /* COMPILE_ESCO */                                          
UCHAR lmp_calculate_max_slot_for_esco (UINT16 ace_ce_index);

#ifdef COMPILE_FEATURE_REQ_EXT
void lmp_fill_features_page(UCHAR page, UCHAR *buf);

void lmp_send_features_req_or_res_ext_pdu(UINT16 ce_index, UCHAR page,
        UCHAR opcode, LMP_TRAN_ID tid);
void lmp_handle_features_req_ext_pdu(LMP_PDU_PKT *lmp_pdu_ptr,UINT16 ce_index);
void lmp_handle_features_res_ext_pdu(LMP_PDU_PKT *lmp_pdu_ptr,UINT16 ce_index);
#endif /* COMPILE_FEATURE_REQ_EXT */                   

void lmp_handle_ptt_req_accepted(LMP_PDU_PKT* lmp_pdu_ptr, UINT16 ce_index);
void lmp_handle_ptt_req_pdu(LMP_PDU_PKT * lmp_pdu_ptr, UINT16 ce_index);
void lmp_handle_ptt_req_actd_ack_recd(UINT16 ce_index);
UCHAR lmp_generate_lmp_ptt_req_pdu(UINT16 ce_index, UCHAR pkt_type_table);

#endif

/** @} end: lmp_external */
