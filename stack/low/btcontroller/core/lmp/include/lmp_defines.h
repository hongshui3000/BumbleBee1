/***************************************************************************
 Copyright (C) Mindtree Ltd.
 This module is a confidential and proprietary property of MindTree and
 a possession or use of this module requires written permission of MindTree.
 ***************************************************************************/

/**
 * \file
 *  Structure and macro definitions of LMP layer.
 */

/** \addtogroup lmp_external
 *  @{ */
#ifndef __LMP_DEFINES_H__
#define __LMP_DEFINES_H__

#include "lmp_external_defines.h"
#include "bt_fw_hci_external_defines.h"
#include "bz_auth.h"
#include "lmp_edtm.h"

/*
 * Bit field definition for afh_bb_ch_assessment_flag.
 * */

#define BIT_MASK_DEVICE_BB_CH_ASSESSMENT        0x00000001
#define BIT_MASK_BB_CH_ASSESSMENT_MODE          0x00000002
#define BIT_MASK_BB_CH_ASSESSMENT_ROLESWITCH    0x00000004
#define BIT_MASK_BB_CH_ASSESSMENT_PARK          0x00000008
#define BIT_MASK_BB_CH_ASSESSMENT_PAGE          0x00000010
#define BIT_MASK_BB_CH_ASSESSMENT_SCAN          0x00000020

#ifdef _CCH_SLOT_OFFSET_
#define GLOBAL_SLOT_INTERVAL	36
#define GLOBAL_SLOT_USE_NUM	6
extern UINT8 g_global_slot_min_allow_sniff_int;
#define GLOBAL_SLOT_MIN_ALLOW_SNIFF_INT	     g_global_slot_min_allow_sniff_int
#endif

/** FHS Packet structure
    (add dummy to let the length of packet structure be 4N bytes)  */
typedef struct
{
//{{{by liuyong 20090916 for 4 Bytes access for ACL_FIFO_4B_ACCESS
	UINT32 clk;

	UCHAR fhs_pkt[LMP_FHS_PKT_LEN];		//should be aligned to 4 byte alignment address
//}}}by liuyong 20090916 for 4 Bytes access for ACL_FIFO_4B_ACCESS

    UCHAR am_addr;
#ifdef COMPILE_RSSI_REPORTING
    INT8 rssi;
#endif

    /* EIR Data recv. structure */
    UCHAR  recv_length;
    UINT32 recv_data[MAX_EIR_DATA_LEN>>2];

} LMP_FHS_PKT_RECD;

/** Role switch states */
typedef enum
{
    LMP_MSS_INIT = 0,
    LMP_MSS_M_TO_S_WAITING_FOR_FHS = 1,
    LMP_MSS_M_TO_S_FHS_RECD = 2,
    LMP_MSS_S_TO_M_SENT_FHS_PKT = 4,
    LMP_MSS_S_TO_M_SENT_FHS_RECD_ID = 8
} ROLE_SWITCH;

/** Self Device States */
typedef enum
{
    LMP_IDLE = 0,
    LMP_PAGE,

#ifdef COMPILE_PERIODIC_INQUIRY
    LMP_PERIODIC_INQUIRY,
#endif

    LMP_INQUIRY
} LMP_SELF_DEVICE_STATES;

/** Event filters structure */
typedef struct
{
    UINT32 class_of_device;
    UINT32 class_of_device_mask;
    UCHAR bd_addr[LMP_BD_ADDR_SIZE];
    UINT16 dummy_16_1;

    UCHAR auto_accept_flag;
    UCHAR filter_type;
    UCHAR filter_condition_type;
    UCHAR dummy_8_1;
} LMP_EVENT_FILTER;

#ifdef _DAPE_TEST_AUTO_CONN
/** connection white list structure */
typedef struct
{
    UCHAR bd_addr[LMP_BD_ADDR_SIZE];
    UINT8 enabled;
	UINT8 reserved;
} LEGACY_WHITE_LIST;
#endif

/** Feature bits structure */

typedef struct
{
#define feat_page0      features[0]
#define feat_page1      features[1]
#define feat_page2      features[2]
    /**
     * Contains both the normal and extented features of the local device.
     * Index 0 contains the normal features and the rest contains the extended
     * features.
     */
    union {
        UCHAR features[LMP_MAX_FEAT_REQ_PAGES][BTC_FEATURES_LEN];
        UINT32 features_dword[LMP_MAX_FEAT_REQ_PAGES][BTC_FEATURES_LEN/4];
    };

#ifdef COMPILE_FEATURE_REQ_EXT
    UCHAR max_supp_pages;
#endif
} LMP_FEATURE_DATA;


/** Role switch parameters */
typedef struct
{
    UINT16 old_piconet_id;
    UINT16 new_piconet_id;

    UINT16 old_lower_lut_address;
    UINT16 old_upper_lut_address;

    UINT16 new_lower_lut_address;
    UINT16 new_upper_lut_address;

    UINT16 ce_index;

    UCHAR old_am_addr;
    UCHAR new_am_addr;

    UCHAR old_lut_index;
    UCHAR new_lut_index;

    UCHAR reason;

#ifdef COMPILE_AFH_HOP_KERNEL
    UCHAR old_afh_mode;
#endif
} LMP_ROLE_SWITCH_PARAMS;


/** Host information data */
typedef struct
{
    UINT16 host_acl_data_pkt_len;
    UINT16 host_sco_data_pkt_len;
    UINT16 host_total_num_acl_data_pkts;
    UINT16 host_total_num_sco_data_pkts;
} LMP_HOST_INFO_DATA;

/** Inquiry Result Table */
typedef struct
{
    UINT32 class_of_device;
    UINT16 clock_offset;
    UCHAR page_scan_repetition_mode;
    UCHAR page_scan_period_mode;
    UCHAR page_scan_mode;
    UCHAR bd_addr[LMP_BD_ADDR_SIZE];
#ifdef COMPILE_RSSI_REPORTING
    INT8 rssi;
#endif /* COMPILE_RSSI_REPORTING */
} LMP_INQUIRY_RESULT_DATA;

/** Connection handle states */
typedef enum
{
    IDLE=0,
    ACTIVE_BROADCAST,
    PICONET_BROADCAST,
    DEDICATED
} LMP_CE_CONN_HANDLE_STATUS;

/** AFH states */
typedef enum
{
    LMP_AFH_DISABLED = 0,
    LMP_AFH_ENABLING,
    LMP_AFH_ENABLED,
    LMP_AFH_UPDATING,
    LMP_AFH_DISABLING
} LMP_CE_CONN_AFH_STATES;

/** Connection entity states */
typedef enum
{
    LMP_STANDBY=0,                      /* 0 */
    LMP_PAGING,                         /* 1 */
    LMP_PAGE_SCAN,                      /* 2 */
    LMP_BB_HL_CONNECTED,                /* 3 */
    LMP_CONNECTED,                      /* 4 */
    LMP_DURING_CONN_REMOTE_NAME_REQ,    /* 5 */
    LMP_WAITING_FOR_CONN_ACCEPT,        /* 6 */
    LMP_WAITING_FOR_CONN_REQ_RESP,      /* 7 */
    LMP_WAITING_FOR_SETUP_COMPLETE,     /* 8 */
    LMP_STOPPING_ENC_FOR_SWITCH,        /* 9 */
#ifdef COMPILE_ROLE_SWITCH
    LMP_CONNECTION_ROLE_SWITCH,         /* 10 */
#endif
    LMP_DISCONNECTING,                  /* 11 */
    LMP_DISCONNECTED,                   /* 12 */
#ifdef ENABLE_SCO
    LMP_ADDING_SCO_LINK,                /* 13 */
#endif
#ifdef COMPILE_ROLE_SWITCH
    LMP_ROLE_SWITCHING,                 /* 14 */
#endif
#ifdef COMPILE_HOLD_MODE
    LMP_HOLD_MODE,                      /* 15 */
    LMP_HOLD_MODE_NEG,                  /* 16 */
    LMP_ENTERING_HOLD_MODE,             /* 17 */
#endif
#ifdef COMPILE_SNIFF_MODE
    LMP_SNIFF_MODE,                     /* 18 */
    LMP_SNIFF_MODE_NEG,                 /* 19 */
#endif
#ifdef COMPILE_PARK_MODE
    LMP_PARK_MODE,                      /* 20 */
    LMP_PARK_MODE_REQ,                  /* 21 */
    LMP_PARK_TRANSITION,                /* 22 */
#endif
#ifdef COMPILE_ESCO
    LMP_ADDING_ESCO_LINK,               /* 23 */
    LMP_CHANGING_ESCO_PARAMS,           /* 24 */
    LMP_ESCO_DISCONNECTING,             /* 25 */
    LMP_ACL_ESCO_DISCONNECTING,         /* 26 */
#endif
#ifdef COMPILE_SNIFF_MODE
    LMP_ADDING_SCO_LINK_DURING_SNIFF,   /* 27 */
    LMP_SNIFF_MODE_NEG_TERMINATION,     /* 28 */
#endif
#ifdef _SUPPORT_CSB_RECEIVER_
    LMP_TRUNCATED_PAGING,               /* 29 */
    LMP_TRUNCATED_PAGE_COPMLETE,        /* 30 */

#endif
    LMP_NO_STATE_CHANGE=100
} LMP_CE_CONN_STATES;

/** LMP Transaction ID */
typedef enum
{
    MASTER_TID=0,
    SLAVE_TID,
    REMOTE_DEV_TID,
    SELF_DEV_TID
} LMP_TRAN_ID;

/** LMP Setup Complete States */
typedef enum
{
    BEGIN_SETUP = 0X00,
    SENT_SETUP_COMPLETE,
    RECD_SETUP_COMPLETE,
    CONN_COMPLETE_EVENT,
    RCVD_SENT_SETUP_COMPLETE
} LMP_SETUP_COMPLETE_STATES;

/** Low power mode values (Used in MODE_CHANGE_EVENT) */
typedef enum
{
    LP_ACTIVE_MODE = 0x00,
    LP_HOLD_MODE = 0x01,
    LP_SNIFF_MODE = 0x02,
    LP_PARK_MODE = 0x03
} LMP_LOW_POWER_MODES;

/** Structure to store connection-type specific parameters */
typedef struct
{
    /** Connection Handle identifying the connection */
    UINT16 connection_handle;
    /** Packet Type to be Used for this connection */
    UINT16 packet_type;

    /** Nature of the Link - ACL or SCO */
    UCHAR link_type;
} LMP_CONNECTION_TYPE;


#define LC_MAX_NUM_OF_PKTS_SUPPORTED            16

/** Structure to keep track of packets types allowed for this remote device */
typedef struct
{
    //UINT16 pkt_type_lut[LC_MAX_NUM_OF_PKTS_SUPPORTED];
    //UINT16 length[LC_MAX_NUM_OF_PKTS_SUPPORTED];
    UINT16 status;
} PKT_ALLOWED;

/** Disconnect states */
typedef enum
{
    IDLE_STATE=0,
    SIX_TPOLL_STATE,
    THREE_TPOLL_STATE,
    SUPERVISION_STATE,
    REMOTE_DETACH_STATE,
    UNPARK_DISCONNECT,
    UNSNIFF_DISCONNECT,
    HOLD_DISCONNECT
} DISCONNECT_STATE;

/** PTT states */
typedef enum
{
    LMP_PTT_IDLE,
    LMP_PTT_ENABLING,
    LMP_PTT_ENABLED,
    LMP_PTT_DISABLING
} PTT_STATUS;

/** Unpark states */
typedef enum
{
    LMP_UNPARK_IDLE = 0,
    LMP_UNPARK_AUTO_CONTROLLER_INITIATED = 1,
    LMP_UNPARK_HOST_INITIATED = 2,
    LMP_UNPARK_READY = 4,
    LMP_UNPARK_SENT = 8
} UNPARK_STATUS;

typedef enum
{
    HCI_UNPARK_IDLE = 0,
    HCI_UNPARK_AUTO_CONTROLLER_INITIATED = 1,
    HCI_UNPARK_HOST_INITIATED = 2,
    HCI_UNPARK_REMOTE_INITIATED = 4
} HCI_UNPARK_STATUS;

/** LMP Sniff Subrating states */
typedef enum
{
    LMP_SSR_IDLE,       /* No activity                  */
    LMP_SSR_TRIED,      /* SSR tried on entering sniff  */
    LMP_SSR_TRANSITION, /* Waiting for SSR Instant      */
    LMP_SSR_ACTIVE      /* Negotiated. active           */
} LMP_SSR_STATUS;

/** LC Sniff Subrating states */
typedef enum
{
    LC_SSR_IDLE,       /* SSR not active (i.e. sniff/idle)      */
    LC_SSR_SUBRATE     /* In SSR mode (after timeout)           */
} LC_SSR_STATUS;

/** EIR Data Structure */
typedef struct
{
    UINT16 packet_type;
    UCHAR  length;
    UCHAR  fec_req;

    /* by austin 20100614 for enter sram  */
    UCHAR *data;
} EIR_DATA;

/** SSR Data Structure */
typedef struct
{
	/* New SSR Instant (lmp in transition) */
	UINT32 ssr_instant;
	/* Accessed/Changed from interrupt */
	UINT32 prev_pkt_time;

    /* Host given parameters */
    UINT16 max_latency;
    UINT16 min_remote_timeout;
    UINT16 min_local_timeout;

    /* Remote device given parameters */
    UINT16 rem_max_ssr;
    UINT16 rem_min_timeout;

    /* Negotiated parameters */
    UINT16 tsniff;

    /* SSR Connection state */
    UINT8 lmp_ssr_state;
    UINT8 lc_ssr_state;

    UCHAR neg_in_progress;

    UCHAR  enter_exit_ssr; /* 0->Nothing 1->Enter 2->Exit */
} SSR_DATA;

/* SSR State Definition (add by austin) */
enum SSR_STATE_ {
    SSR_STATE_NONE,
    SSR_STATE_ENTER,
    SSR_STATE_EXIT
};

#ifdef VER_3_0
typedef struct
{
    UCHAR epc_step;
    /* EPC State */
    UCHAR epc_req_sent;
} EPC_DATA;
#endif

/** Connection entity structure */
typedef struct
{
	//**************************4Bytes alignment member**********************
	//UINT32, int32, pointer, enum, ...
	//***********************************************************************

    UINT32 class_of_device;     /**< Class of Device of the remote device */

    /* QOS parameters. */
    UINT32 token_rate;
    UINT32 token_bucket_rate;
    UINT32 peak_bandwidth;
    UINT32 latency;
    UINT32 delay_variation;

#ifdef COMPILE_ROLE_SWITCH
    UINT32 switch_instant;
#endif//COMPILE_ROLE_SWITCH

	UINT32 aclq_start_flush_time;

	TimerHandle_t conn_accept_timer_handle;
	TimerHandle_t detach_connection_timer_handle;
	TimerHandle_t supervision_timeout_handle;
	TimerHandle_t lmp_response_timer_handle;

	/* Indicates the type of PDU response device is waiting for */
	UINT32 lmp_expected_pdu_opcode;

#define feat_page0     features[0]

	UINT32 sent_pdu;

#ifdef COMPILE_HOLD_MODE
	UINT32 hold_instant;
#endif

#ifdef COMPILE_SNIFF_MODE
#ifdef POWER_SAVE_FEATURE
	UINT32 sniff_last_instant;          /**< Saves the clock at which the last
										sniff start interrupt occured */
#endif
	UINT32 next_instant_in_nat_clk; /* Unit is clk0. */
	UINT32 next_next_instant_in_nat_clk; /* Unit is clk0. */
	TimerHandle_t sniff_sw_timer_handle;
#endif

#ifdef COMPILE_PARK_MODE
	TimerHandle_t park_mode_timer_handle;
#endif

#ifdef COMPILE_AFH_HOP_KERNEL
	UINT32 last_set_afh_sent_clk;
#ifdef COMPILE_PARK_MODE
	LMP_PDU_PKT *park_lmp_pdu_ptr;
#endif
	LMP_PDU_PKT *mss_pdu_ptr;
	TimerHandle_t afh_instant_timer_handle;
	UINT32 afh_instant;
#endif /* COMPILE_AFH_HOP_KERNEL */

	BZ_AUTH_LINK_PARAMS_PTR auth;
	LMP_EDTM_DATA_PTR edtm;

	//**************************2Bytes alignment member**********************
	//UINT16, short
	//***********************************************************************

	/* Tpoll value for QoS */
	UINT16 qos_tpoll;
	UINT16 qos_setup_tpoll;
	UINT16 flow_tx_tpoll;
	UINT16 flow_rx_tpoll;

#ifdef COMPILE_DYNAMIC_POLLING
	UINT16 current_tpoll;
	UINT16 no_inactive_tpolls;
#endif

	UINT16 hc_num_of_completed_packets;
	UINT16 flush_timeout;
    UINT16 failed_contact_counter;

#ifdef COMPILE_CQDDR
    UINT16 preferred_cqddr_pkt_type;
#endif

    /*
     * Link supervision timeout
     */
    UINT16 link_supervision_timeout;
    UINT16 stored_link_supervision_timeout;
    UINT16 clock_offset;
    UINT16 slot_offset;
    UINT16 rem_manuf_name;
    UINT16 lmp_subversion;

	/* Link Policy settings */
	UINT16 link_policy_settings;

	/* Tpoll slot value */
	UINT16 Tpoll;

	UINT16 rssi_samples_count;
	UINT16 rssi;
    UINT16 rssi_value_accumulate;

#ifdef COMPILE_HOLD_MODE
	/* Hold Mode Parameters */
	UINT16 hold_mode_max_interval;
	UINT16 hold_mode_min_interval;
	UINT16 hold_mode_interval;
#endif

#ifdef COMPILE_SNIFF_MODE
	UINT16 sniff_max_interval;
	UINT16 sniff_min_interval;
	UINT16 sniff_interval;
	UINT16 sniff_slot_offset;
	UINT16 sniff_attempt;
	UINT16 sniff_attempt_for_afh_count;
	UINT16 sniff_timeout;
#endif

#ifdef COMPILE_PARK_MODE
	/* Park Mode Parameters */
	UINT16 beacon_max_interval;
	UINT16 beacon_min_interval;
	UINT16 Dbeacon;
	UINT16 Tbeacon;
	UINT16 auto_unpark_cnt;
#endif

#ifdef COMPILE_HOLD_MODE
	UINT16 hold_neg_max_interval;
#endif

	UINT16 out_standing_data_to_host;

#ifdef COMPILE_AFH_HOP_KERNEL
	UINT16 afh_min_interval;
	UINT16 afh_max_interval;
#endif /* COMPILE_AFH_HOP_KERNEL */

	UINT16 hci_cmd_bits;

#ifdef _RATE_ADAPTION_SAMPLE_
    /* used for adaption reference (austin) */
    UINT16 send_2m_edr_tx_count;
    UINT16 send_2m_edr_pkt_count;
    UINT16 send_2m_edr_ack_count;
    UINT16 send_3m_edr_tx_count;
    UINT16 send_3m_edr_pkt_count;
    UINT16 send_3m_edr_ack_count;
#endif

    //**************************1Bytes alignment member**********************
    //UINT8, UCHAR, CHAR
    //***********************************************************************
    /* Flag to indicate the state of connection between the self and remote
    device */
    UINT8 ce_status;
    UINT8 temp_ce_status;
    UINT8 afh_ce_status;

    UINT8 detach_timer_state;
    UINT8 low_power_disconnect_state;
    UINT8 ptt_status;

#ifdef COMPILE_PARK_MODE
    UINT8 unpark_req_flag;
    UINT8 hci_unpark_req_flag;
#endif /* COMPILE_PARK_MODE */

	/**
     * Authentication role of the local device during the encryption
     * pause/stop and resume/restart procedure. It is valid only if \a
     * is_enc_paused is set.
     *
     * \sa is_enc_paused.
     */
    UINT8 auth_role;

#ifdef _SSP_DHKEY_CALCULATE_NEW_FLOW_
    UINT8 dhkey_calculating;
#endif

    /**
     * Encryption procedure used during the pause/stop and resume/restart
     * encryption. It is valid only if \a is_enc_paused is set.
     *
     * \sa is_enc_paused.
     */
    UINT8 enc_proc;

#ifdef _RESET_HW_LINK_INFO_TO_INIT_STATE_
    UINT8 enc_opcode;
#endif

	/* Quality of service */
	UCHAR QoS_flags;
	UCHAR service_type;

	/* Tpoll value for QoS */
	UCHAR qos_tn_details;

#ifdef COMPILE_ROLE_SWITCH
	UCHAR mss_completion_status;
#endif /* COMPILE_ROLE_SWITCH */

#ifdef ENABLE_SCO
	UCHAR no_of_sco_connections;
	UCHAR no_of_esco_connections;
    union {
        struct {
            UCHAR is_sco_channel:1;      /* this channel is an sco channel ? */
            UCHAR is_esco_channel:1;     /* this channel is an esco channel ? */
            UCHAR sco_ce_idx:2;          /* sco channel index */
            UCHAR esco_ce_idx:3;         /* esco channel index */
            UCHAR rsvd:1;                /* reserved */
        };
        UCHAR sco_status_byte;
    };

    union {
        struct {
            UCHAR is_pcm_input:1;        /* tx input PCM is format ? */
            UCHAR pcm_conv_type:2;       /* linear pcm convertision type */
            UCHAR trx_codec_conv_type:3; /* trx codec convertision type */
            UCHAR txfifo_in_8bit_code:1; /* the input of txfifo is 8 bit code */
            UCHAR rxfifo_in_8bit_code:1; /* the input of rxfifo is 8 bit code */
        };
        UCHAR codec_status_byte;
    };
#endif


	/**
	* Denotes whether the encryption is currently paused or not. It is reset
	* after the encryption is resumed. This is set when the local device has
	* paused/stopped the encryption or the remote device has paused the
	* encryption. Note the difference between the local device and remote
	* device initiated procedures.
	*/
	BOOLEAN is_enc_paused;

	UCHAR is_last_sent_zero_l_l2cap;
	UCHAR aclq_resch_flag; /* 0:Normal 1:ACL Reschdule 2:zero length reschedule*/
	UCHAR flush_running;
	UCHAR enhanced_flush;

#ifdef COMPILE_CQDDR
	UCHAR sent_autorate_pdu;
	UCHAR received_auto_rate_pdu;
#endif

	UCHAR phy_piconet_id; /* the piconet id (0~3) */
	UCHAR optional_page_scheme;
	UCHAR optional_page_setting;

    /* Keeps the entity status */
    UCHAR entity_status;

    /* Role: Master/Slave being played by the remote device */
    UCHAR remote_dev_role;

    /* The remote BD_ADDR this connection is opened to */
    UCHAR bd_addr[LMP_BD_ADDR_SIZE];

    /* AM_ADDR assigned to the slave */
    UCHAR am_addr;

    /* Number of broadcast transmissions...*/
    UCHAR num_of_BC;

    UCHAR supervision_timeout_flag;

    CHAR transmit_power_level;
    UCHAR link_quality;

#ifdef POWER_CONTROL
#ifdef OPTIONAL_PAGING
    UCHAR rssi_meas_flag;
#endif /* OPTIONAL_PAGING */
    /* This flag keeps track of whether max or min power is received. */
    UCHAR power_ctrl_resp;
    /*
     * This value is incremented when we send max power req/min power req
     * to keep track how many times we issued the power control PDU's
     *  Note: This value is not used anywhere currently.
     */
    UCHAR power_ctl_count;
#endif
    /* Page Scan repetition mode */
    UCHAR page_scan_repetition_mode;
    /* Page Scan repetition mode */
    UCHAR page_scan_period_mode;
    /* Page Scan mode */
    UCHAR page_scan_mode;

    UCHAR allow_role_switch;

    /*
     * This flag indicates the role switch is accepted by the
     * remote device
     */
    UCHAR role_switch_accepted_flag;

    /**
     * Contains both the normal and extented features of the remote device.
     * Index 0 contains the normal features and the rest contains the extended
     * features.
     */
    UCHAR features[LMP_MAX_FEAT_REQ_PAGES][BTC_FEATURES_LEN];
    UCHAR requested_ext_page;

#ifdef _SUPPORT_EXT_FEATURES_PAGE_2_
    UCHAR host_requested_ext_page; /* host request extend page */
    UCHAR requested_ext_page_bm; /* only record page 0 ~ page 7 */
#endif

    UCHAR disconnect_reason;

#ifdef _CCH_PAGE_CON_
       /* This Connection Reason is for Remote Name Req(1) or Feature req(2) or Connection(0) */
    UCHAR connect_reason;
#endif

    /* TODO : use dynamic memory management to reduce memory size (austin) */
    UCHAR device_name[LMP_MAX_NAME_LENGTH];

    /*
     * Contains the name offset value sent in name
     * request command.
     */
    UCHAR name_req_name_offset;
#ifdef _REMOTE_NAME_RES_WRONG
    UCHAR name_length_offset_zero;
#endif
    /* LMP version info */
    UCHAR lmp_version;

    /* Max Slot */
    UCHAR tx_max_slot;
    UCHAR rx_max_slot;
    UCHAR old_tx_max_slot;
    UCHAR old_rx_max_slot;
    UCHAR temp_tx_max_slot;
    UCHAR temp_rx_max_slot;

    UCHAR remote_max_drift;
    /* Setup complete state. */
    UCHAR setup_complete_status;
    UCHAR pdu_response_timer_running;

#if defined(COMPILE_SNIFF_MODE) || defined(COMPILE_HOLD_MODE)
    UCHAR cont_poll_count;
#endif

#ifdef COMPILE_SNIFF_MODE
    /* Sniff Mode Parameters */
    UCHAR in_sniff_mode;                /**< Indicates whether the link is in
                                             sniff mode or not */
    UINT16 temp_sniff_interval;
    UCHAR temp_sniff_nego_in_progress;
    UCHAR dummy_sniff_8bit;

#ifdef _CCH_SNIFF_NEG_TIMEOUT_
    UCHAR sniff_neg_count;
#endif

#endif

#ifdef COMPILE_PARK_MODE
    UCHAR supto_auto_repark;
#endif

#ifdef COMPILE_HOLD_MODE
    UCHAR hold_max_interval_negotiated;
    UCHAR hold_min_interval_negotiated;
    UCHAR hold_mode_accepted_flag;
    UCHAR hold_mode_interval_negotiated;
#endif

#ifdef COMPILE_SNIFF_MODE
    UCHAR sniff_max_interval_negotiated;
    UCHAR sniff_min_interval_negotiated;
#endif

#ifdef COMPILE_PARK_MODE
    UCHAR Nbeacon;
    UCHAR Delta_beacon;
    UCHAR pm_addr;
    UCHAR ar_addr;
    UCHAR NB_sleep;
    UCHAR DB_sleep;
    UCHAR D_access;
    UCHAR T_access;
    UCHAR N_acc_slots;
    UCHAR N_poll;
    UCHAR M_access;
    UCHAR access_scheme;
    UCHAR timing_control_flag;
    UCHAR park_mode_negotiated;
    UCHAR park_using_bd_addr;  // used only in vendor command
    UCHAR bc_scan_window;
#endif  /* COMPILE_PARK_MODE */

#ifdef POWER_CONTROL
    UCHAR dec_pow_pdu_drop_flag;
    UCHAR inc_pow_pdu_drop_flag;
    UCHAR power_ctrl_pdu_sent_flag;
#endif

#ifdef COMPILE_AFH_HOP_KERNEL
    UCHAR afh_ch_cl_reporting_mode;
    UCHAR need_to_send_ch_cls_req;
    UCHAR need_to_send_ch_cls;

    UCHAR waiting_for_set_afh_pdu_ack;

#ifdef _CCH_IOT_CSR_RS_
    UCHAR waiting_for_rs_several_times;
#endif


#ifdef COMPILE_PARK_MODE
    UCHAR park_pending_for_afh;
    UCHAR afh_disabled_for_park;
    UCHAR rem_park_pending_for_afh;
#endif
    UCHAR mss_cmd_pending;
    UCHAR status_event_mss_cmd_pending;
    UCHAR mss_pdu_pending;
    UCHAR afh_mode;
    UCHAR old_afh_map[LMP_AFH_MAP_SIZE];
    UCHAR afh_map[LMP_AFH_MAP_SIZE];
    UCHAR last_acked_afh_map[LMP_AFH_MAP_SIZE];
    UCHAR last_recd_ch_cl_map[LMP_AFH_MAP_SIZE];
    UCHAR new_ch_cl_map;
#endif /* COMPILE_AFH_HOP_KERNEL */

    UCHAR hci_cmd_sts_evt_gen_for_mss_or_park;

    UCHAR host_con_req_rx_flag;
    UCHAR paging_completed_flag;
    UCHAR flush_continue_pkts;

#ifndef COMPILE_SINGLE_SLOT_PACKETS_ONLY
    UCHAR last_max_slot_req_sent;
    UCHAR last_max_slot_sent;
    UCHAR last_accepted_max_slot;
    UCHAR last_recd_max_slot;
#endif

#ifdef VER_3_0
    EPC_DATA epc_data;
#endif
#ifndef _ENABLE_COMPILE_NESTED_PAUSE_RESUME_
    UCHAR pause_data_transfer;
#endif
    /* record lmp pdu for debug - austin */
    UCHAR sent_lmp_pdu_opcode;
    UCHAR sent_lmp_pdu_ext_opcode;

    //**************************left struct member***************************
    //......
    //***********************************************************************
    /* Packet types allowed for this device
    this will be decided on the behalf of remote features
    packets allowed by host and number of sco connections */
    PKT_ALLOWED pkts_allowed;

    /* Connection type specific info */
    LMP_CONNECTION_TYPE connection_type;

    LMP_PDU_PKT lmp_pdu_pkt;

#ifdef TEST_MODE
    TEST_MODE_INFO test_mode_info;
#endif

    SSR_DATA ssr_data;

#ifdef _YL_LPS
    UCHAR sniff_xtol;
    UCHAR sniff_xtol_prev;
#endif
    UINT8  got_remote_feature;
    UINT8  got_remote_ext_feature;
    UINT8  got_host_conn_req;
#ifdef _DAPE_TEST_FIX_HW_SCO_NAK_SNIFF_BUG
    UCHAR should_ack;
#endif
#ifdef SECURE_CONN_PING_EN
    UINT16 max_auth_interval;
    TimerHandle_t en_ping_req_timer;
    TimerHandle_t send_max_auth_timeout_timer;
#endif

#ifdef _CCH_SC_ECDH_P256_START_ENC
    UCHAR send_start_enc;
#endif
#ifdef _CCH_SC_ECDH_P256_STOP_ENC
    UCHAR send_stop_enc;
#endif

#ifdef _CCH_SC_TEST_20130129_MRV_01
    UCHAR done_h3;
#endif
#ifdef _SECURE_CONN_TEST_MODE
    UCHAR disable_dm1;
#endif
#ifdef _SECURE_CONN_REFRESH_KEY_WHEN_CONTINUOUS_MIC
    UCHAR mic_err_cnt;
#endif

#ifdef _CCH_RETENTION_FLOW_FOR_DLPS_
    UCHAR dlps_last_seqn;
#endif

} LMP_CONNECTION_ENTITY;


/** Self device specific parameters(Same across piconets). */
typedef struct
{
	//**************************4Bytes     member***************************
	//......
	//***********************************************************************
    UINT32 class_of_device;
	LMP_SELF_DEVICE_STATES device_status;

	UINT32 next_sniff_instant;
	UINT32 next_next_sniff_instant;

	/* This can be Idle, inq or paging. */
	UINT32 lc_cur_dev_state;

	//**************************2Bytes     member***************************
	//......
	//***********************************************************************
    UINT16 conn_accept_timeout;
    UINT16 page_timeout;
    UINT16 page_scan_interval;
    UINT16 page_scan_window;
    UINT16 inquiry_scan_interval;
    UINT16 inquiry_scan_window;
#ifdef _LPS_FOR_8821_
    UINT16 scan_interval_min;
#endif
    UINT16 voice_setting;
    UINT16 sco_pkt_type;
	UINT16 default_link_policy_settings;
	UINT16 cl_rep_min_interval;
	UINT16 cl_rep_max_interval;
	UINT16 min_RSSI_val;
	UINT16 max_RSSI_val;

#ifdef BROADCAST_DATA
	UINT16 asb_num_of_completed_packets;
	UINT16 psb_num_of_completed_packets;
	UINT16 bc_conn_handle;
	UINT16 park_bc_conn_handle;
#endif

	//**************************1Bytes     member***************************
	//......
	//***********************************************************************
    UCHAR local_name[LMP_MAX_NAME_LENGTH];
    UCHAR event_mask[LMP_EVENT_MASK_SIZE];
#if defined(VER_CSA4) || defined(_SUPPORT_SECURE_CONNECTION_)
    UCHAR event_mask_p2[LMP_EVENT_MASK_SIZE];
#endif
    UCHAR lmp_air_mode;
    UCHAR input_data_format;
    UCHAR iac_lap[LMP_MAX_IAC_LAPS][3];
#ifdef _DAPE_UNIFY_VERSION_FEATURE_UINT
    UCHAR lmp_version;
    UCHAR hci_version;
#endif
    /* security keys */
    UCHAR security_key[16];

    /* answer after realtek bt driver association mechanism */
    UCHAR rtk_assoc_data[12];

    /* diac_lap flag is not required */
    UCHAR diac_lap;
    UCHAR local_name_len;

    UCHAR num_hci_command_packets;
    UCHAR num_broadcast_retran;
    UCHAR sco_flow_control_enable;
    /* Not required since we only support one LAP */
    UCHAR num_supported_iac;
    UCHAR flow_control_hc_to_host;
    UCHAR scan_enable;
    UCHAR page_scan_mode;
    UCHAR page_scan_period_mode;
    UCHAR page_scan_repetition_mode;
    UCHAR number_of_acl_conn;

#ifdef ENABLE_SCO
    UCHAR total_no_of_sco_conn;
    UCHAR adding_new_sco_conn;
#endif

#ifdef COMPILE_ESCO
    UCHAR number_of_esco_connections;
#ifdef _DAPE_TEST_SEND_MAX_SLOT_BEFORE_ESCO_CREATED
    UCHAR adding_new_esco_conn;
#endif
#endif

    UCHAR number_of_hlc;
    UCHAR opt_page_scan_flag;

#ifdef COMPILE_PARK_MODE
    UCHAR number_of_parked_dev;
#endif

#ifdef COMPILE_DYNAMIC_POLLING
    UCHAR enable_dynamic_polling;
    UCHAR tpoll_inactive_transition;
    UCHAR tpoll_step;
    UCHAR min_tpoll;
#endif

 #ifdef TEST_MODE
    UCHAR test_mode;
    UCHAR stored_test_mode_state;
    UCHAR host_enable_test_mode;
#endif

    UCHAR inquiry_scan_type;
    UCHAR inquiry_mode;
    UCHAR page_scan_type;
    UCHAR interlaced_inq_scan;
    UCHAR interlaced_page_scan;

    UCHAR afh_channel_assessment_mode;
    UCHAR lc_dsniffs;
    UCHAR unpark_op_code;
    UCHAR default_erroneous_data_reporting;
    UCHAR inq_id_tx_pwr;
    UCHAR num_of_sniff_connections;

#ifdef COMPILE_SNIFF_MODE
#ifdef DETECT_SNIFF_OVERLAP
    UCHAR no_of_sniff_connections;
#endif
#endif

#ifdef COMPILE_HOLD_MODE
    UCHAR hold_mode_activity;
    UCHAR number_of_connections_in_hold_mode;
#endif

    UCHAR lc_no_of_connections[LMP_MAX_PICONETS_SUPPORTED];

#ifdef _USE_SNIFF_NO_ACL_COUNT_REGISTER
#ifdef COMPILE_SNIFF_MODE
    UCHAR no_acl_reduced_flag;
    UCHAR no_acl_ce_index;
#endif
#endif

#ifdef _CCH_SLOT_OFFSET_
    UCHAR global_slot_use_ce_index[GLOBAL_SLOT_USE_NUM];
    UCHAR global_slot_use_acl_ce_index[GLOBAL_SLOT_USE_NUM];
    UCHAR global_slot_use_interval[GLOBAL_SLOT_USE_NUM];
    UCHAR global_slot_use_slot_offset[GLOBAL_SLOT_USE_NUM];
    UCHAR global_slot_use_remote_slot[GLOBAL_SLOT_USE_NUM];
    UCHAR global_slot_use_slot_num[GLOBAL_SLOT_USE_NUM];
    UINT16 global_slot_interval[GLOBAL_SLOT_USE_NUM];
#endif

	//**************************left struct member***************************
	//......
	//***********************************************************************
	LMP_EVENT_FILTER event_filter[LMP_MAX_EVENT_FILTERS];
	EIR_DATA eir_data;


#ifdef _CCH_LPS_

#if 0 // (cch) 20130911 No use anymore
    UINT16 lps_scan_window;
    UINT8  lps_i_scan_window_en;
    UINT8  lps_p_scan_window_en;
    UINT16 lps_i_scan_window;
    UINT16 lps_scan_interval;
#endif
    // (only can use when link interval <= scan things interval )
    // Not each wakeup window need to do scan things
    UINT8  lps_sniff_scan_en;
    UINT16 lps_sniff_scan_count;
#endif
} LMP_SELF_DEVICE_DATA;



#if defined(ENABLE_SCO)|| defined(COMPILE_ESCO)
#define NOT_APPLICABLE                          0
#define NOT_USED                                0
#define OVER_NULL /*OVER_HCI*/                  1
#define OVER_HCI                                1
#define OVER_CODEC                              2

#define SYNC_FIFO1                              1
#define SYNC_FIFO2                              2
#define SYNC_FIFO3                              3

#define PCM_CODEC                               1
#define UDA_CODEC                               2
#define SCO_HCI                                 3
#define ESCO_HCI                                4
#endif /* ENABLE_SCO || COMPILE_ESCO */

#if defined(SCO_OVER_HCI)|| defined(COMPILE_ESCO)
/** Synchronous data packet structure */
/*
 * Synchronous data Header bit positions
 * |1|2|3|4|5|6|7|8|9|10|11|12|13|14|15|16|17|18|19|20|21|22|23|24|
 * |<----- conn_handle ------>|<--->|<--->|<---- data length ---->|
 *              data_status-----|     |
 *                  reserved----------|
 */
typedef struct
{
	UINT32  dummy : 8;
	UINT32  connection_handle : 12;
	UINT32  data_status_flag : 2;
	UINT32  reserved : 2 ;
	UINT32  packet_length : 8;
    UCHAR  hci_sync_data_packet[HCI_SYNCHRONOUS_DATA_PAYLOAD_SIZE];
} HCI_SYNC_DATA_PKT;
#endif  /* SCO_OVER_HCI || COMPILE_ESCO */

#ifdef ENABLE_SCO
/**
 * SCO state machine states.
 * \warning The state value must be powers of 2 (No state value should have
 *          multiple bits set in them).
 */
typedef enum
{
    SCO_FREE                    = BIT0,
    SCO_CONNECTING              = BIT1,
    SCO_CONNECTED               = BIT2,
    SCO_DISCONNECTING           = BIT3,
    SCO_WAITING_FOR_CONN_ACCEPT = BIT4,
    SCO_CHG_PARAMS              = BIT5,
    SCO_ALLOCATED               = BIT6
} LMP_SCO_STATE;

/**
 * Transaction Status.
 */
typedef enum
{
    TRS_INVALID,            /**< Invalid transaction */
    TRS_SELF_INITIATED,     /**< Self device initiated transaction */
    TRS_REMOTE_INITIATED    /**< Remote device initiated transaction */
} LMP_TRANSACTION_STATUS;

/**
 * SCO connection database. It contains an entry for each SCO connection
 * currently established.
 */
typedef struct
{
    UINT32 early_bb_clock;          /* early bluetooth clock */

    UINT16 sco_conn_handle;         /**< Connection handle for this SCO
                                         connection and the scope is between
                                         the host and the host controller. */
    UINT16 conn_entity_index;       /**< ACL ce_index corresponding to this
                                         SCO connection */

    LMP_SCO_STATE sco_conn_status;  /**< Status of the connection */
    LMP_TRANSACTION_STATUS tr_status; /**< Transaction status */
    UCHAR pkt_type;                 /**< Packet type selected for this
                                         connection. It is LMP level packet
                                         type (0=HV1, 1=HV2, or 2=HV3) */
    UCHAR sco_number;               /**< 0 - SCO1, 1- SCO2, 2 - SCO3 */
    UCHAR Tsco;                     /**< The SCO interval */
    UCHAR Dsco;                     /**< The SCO offset */

    UCHAR sco_handle;               /**< The SCO handle and the scope is
                                         between the host controller and the
                                         remote device */
    UCHAR time_control_flags;
    UINT16 host_allowed_packets;
    UCHAR air_mode;                 /**< Air mode */
#ifdef SCO_OVER_HCI
    UCHAR bb_pkt_type;              /**< Baseband packet type (equivalent to
                                         the LMP level \a pkt_type) */
    UCHAR pkt_length;               /**< Length of the packet type selected.
                                         Though it can be derived from the
                                         packet type, it is included here to
                                         avoid repeated computation in various
                                         places. */
    UCHAR erroneous_data_reporting; /**<enabled/disabled erroneous data
                                      reporting for this connection */
#endif /* SCO_OVER_HCI */

	UINT16 max_latency;             /**< Maximum latency allowed by the host */
    UINT16 voice_settings;
    UCHAR fifo_num;                 /**< FIFO associated with this link */
    UCHAR codec_type;               /**< PCM/UDA/NOT_APPLICABLE */
    UCHAR gen_conn_complete;        /**< Flag to indicate whether to generate
                                         Conn_Comp_Evt or Sync_Conn_Comp_Evt */
#ifdef SCO_OVER_HCI
    UCHAR sch_valid_flag;           /**< Flag to indicate whether this entry is
                                         valid for SCO scheduler or not */
    UCHAR codec_state;              /**< Over ESCO_HCI/SCO_HCI/OVER_CODEC */
#endif /* SCO_OVER_HCI */

#ifdef _CCH_SCO_OUTPUT_ZERO_WHEN_NO_RX_
    UINT16 sco_no_rx_count;
    UCHAR sco_no_rx_force_output_zero_en;
#endif

} LMP_SCO_CONNECTION_DATA;

#define HCI_1_1_SCO_PKT_TYPE_MASK       0x00E0
#define HCI_1_2_SCO_PKT_TYPE_MASK       0x0007
#define LMP_SCO_PKT_TYPE_MASK           0x0003
#define HCI_ESCO_EDR_PKT_TYPE_UMASK     0x03C0

/* Conversion of SCO packet types from one form to another */
/**
 * Converts LMP PDU SCO packet type to HCI 1.2 SCO packet type.
 *
 * \param pkt_type SCO packet type in LMP PDU Parameter format (valid values
 *                 are 0, 1, 2).
 *
 * \return SCO packet type in HCI 1.2 [Setup_Sync] format.
 */
#define SCO_PKT_TYPE_LMP_TO_1_2_HCI(pkt_type)                   \
    ((UINT16)((1<<(pkt_type)) & HCI_1_2_SCO_PKT_TYPE_MASK))

/**
 * Converts LMP PDU SCO packet type to HCI 1.1 SCO packet type.
 *
 * \param pkt_type SCO packet type in LMP PDU Parameter format (valid values
 *                 are 0, 1, 2).
 *
 * \return SCO packet type in HCI 1.1 [ADD_SCO] format.
 */
#define SCO_PKT_TYPE_LMP_TO_1_1_HCI(pkt_type)                   \
    ((UINT16)((0x20<<(pkt_type)) & HCI_1_1_SCO_PKT_TYPE_MASK))

/**
 * Converts HCI 1.2 SCO packet type to LMP PDU SCO packet type.
 *
 * \param pkt_type SCO packet type in HCI 1.2 [Setup_Sync] format.
 *
 * \return LMP SCO packet type equivalent to \a pkt_type.
 *
 * \warning Only one of the SCO packet type should be enabled in \a
 *          pkt_type parameter.
 */
#define SCO_PKT_TYPE_1_2_HCI_TO_LMP(pkt_type)               \
    ((UCHAR)(((pkt_type)>>1) & LMP_SCO_PKT_TYPE_MASK))

/**
 * Converts HCI 1.1 SCO packet type to LMP PDU SCO packet type.
 *
 * \param pkt_type SCO packet type in HCI 1.1 [ADD_SCO] format.
 *
 * \return LMP SCO packet type equivalent to \a pkt_type.
 *
 * \warning Only one of the SCO packet type should be enabled in \a
 *          pkt_type parameter.
 */
#define SCO_PKT_TYPE_1_1_HCI_TO_LMP(pkt_type)               \
    ((UCHAR)(((pkt_type)>>6) & LMP_SCO_PKT_TYPE_MASK))

/**
 * Converts HCI 1.1 SCO packet type to HCI 1.2 SCO packet type.
 *
 * \param pkt_type SCO packet type in HCI 1.1 [ADD_SCO] format.
 *
 * \return SCO packet type in HVI 1.2 format.
 */
#define SCO_PKT_TYPE_1_1_HCI_TO_1_2_HCI(pkt_type)           \
    ((UINT16)(((pkt_type)>>5) & HCI_1_2_SCO_PKT_TYPE_MASK))

/**
 * Converts HCI 1.2 SCO packet type to HCI 1.1 SCO packet type.
 *
 * \param pkt_type SCO packet type in HCI 1.2 [ADD_SCO] format.
 *
 * \return SCO packet type in HVI 1.1 format.
 */
#define SCO_PKT_TYPE_1_2_HCI_TO_1_1_HCI(pkt_type)           \
    ((UINT16)(((pkt_type)<<5) & HCI_1_1_SCO_PKT_TYPE_MASK))
#endif /* ENABLE_SCO */

/** AM_Address to Connection entity index table */
typedef struct
{
    UINT8 status;
    UINT8 ce_index;
} LMP_AM_ADDR_TO_CE_INDEX_TABLE;

/** PM_Addres to Connection entity index table */
typedef struct
{
    UINT8 status;
    UINT8 ce_index;
} LMP_PM_ADDR_TO_CE_INDEX_TABLE;

/** AR_Addres to Connection entity index table */
typedef struct
{
    UINT8 status;
    UINT8 ce_index;
} LMP_AR_ADDR_TO_CE_INDEX_TABLE;

/** Connection Handle to Connection entity index table */
typedef struct
{
    UINT8 status;
    UINT8 ce_index;
} LMP_CONN_HANDLE_TO_CE_INDEX_TABLE;

/**  BD_Address to Connection entity index table */
//typedef struct bd_tbl_to_ce_index_map
//{
//    UINT16  ce_index;
//    UCHAR   bd_addr[LMP_BD_ADDR_SIZE];
//    struct bd_tbl_to_ce_index_map *next;
//} LMP_BD_TBL_TO_CE_INDEX_MAP;

/** Device Information data structures */
typedef struct cache_table
{
    UINT16 clock_offset;
    UCHAR bd_addr[LMP_BD_ADDR_SIZE];
    UCHAR state;
    UCHAR page_mode;
    UCHAR page_mode_settings;
    UCHAR next_index;
    UCHAR prev_index;
    UCHAR cur_index;
} CACHE_TABLE;

/** Page mode device information */
typedef struct
{
    UCHAR bd_addr[LMP_BD_ADDR_SIZE];
    UCHAR page_mode;
    UCHAR page_mode_settings;

    UCHAR dummy_8; /* For padding. */

    UINT16 clock_offset;

    UCHAR dummy_16; /* For padding. */
} DEVICE_INFO;

#endif /* __LMP_DEFINES_H__ */

/** @} end: lmp_external */

