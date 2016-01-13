#ifndef _MAILBOX_H_
#define _MAILBOX_H_

#include "platform.h"
#include "DataType.h"
#include "mint_os.h"

#ifdef _ENABLE_MAILBOX_

extern UINT32 mailbox_reg_base_addr[8];
extern UINT32 mailbox_reg_offset;
extern UINT32 mailbox_reg_value;
extern UINT16 mailbox_reg_w_value;
extern UINT32 *mailbox_sram_addr;
extern UINT8 mailbox_rom_code_page_mode;
extern UINT16 mailbox_rom_code_len;
extern UINT16 mailbox_page_count;
extern UINT8 g_wifi_ch;
extern UINT8 mailbox_out_data_busy;
extern UINT32 mailbox_count;
extern OS_HANDLE mailbox_handle;
extern UINT8 bBtPsdMode;	// 20120912 morgan add for bt psd mode setting : default is all on + host_ch_class
extern UINT8 bHIDenable;	//20120913 morgan add for HID enable
extern UINT8 bHIDCount; //20120913 morgan add for HID enable set am-addr
extern UINT8 bBtLNAConstraint; //20120921 morgan add for BT LNA constraint
extern UINT8 a2dp_rate;//20120921 morgan modify FTP estimation depends on a2dp_rate
extern UINT8 bEnableIgnoreWlanActive;
#endif

#ifdef _IQK_HANDSHAKE_FLOW_
#define NO_IQK_HS_FLOW_NO_CAL	0		//no IQK handshake flow, if bt at s0 and FT is valid , use FT 
#define NO_IQK_HS_FLOW_CAL		1		//no IQK handshake flow , bt cal, ¦]¬°WiFi disable
#define IQK_HS_FLOW_FAIL		2		//IQK handshake flow fail , use default value,  because rejected by WiFi
#define IQK_HS_FLOW_SUCCESS		3		//IQK handshake flow ok
#define IQK_HS_FLOW_DEFAULT		0xFF	//default value
#endif

#ifdef MWS_POLL_TIMEOUT
#define MWS_NUM_POLL_MAX1 200

#define MWS_POLL_TIMEOUT_LOG1() RT_BT_LOG(RED, LOG_MWS_POLL_TIMEOUT1, 0, 0)

#define MWS_WAIT_FOR_COEX_CTRL_IDLE() \
				do {\
					int m = 1;\
					while (mws_read_register_new(LTE_COEX_STATUS_REG)&0x3)\
					{\
						if (m > MWS_NUM_POLL_MAX1)\
						{\
							MWS_POLL_TIMEOUT_LOG1();\
							break;\
						}\
						++m;\
					}\
				} while (0)
#else
#define MWS_WAIT_FOR_COEX_CTRL_IDLE() do {} while (mws_read_register_new(LTE_COEX_STATUS_REG)&0x3)
#endif

//add by NeilChen
#define MAILBOX_READ                1
#define MAILBOX_WRITE               2
#define MAILBOX_AFH                 3
#define DW_READ_WRITE_REG           1
#define W_READ_WRITE_REG            2

#define MAILBOX_CTL_REG             0x15C
#define MAILBOX_IN_DATA0_REG        0x168
#define MAILBOX_IN_DATA1_REG        0x16C
#define MAILBOX_OUT_DATA0_REG       0x160
#define MAILBOX_OUT_DATA1_REG       0x164

#define ROM_CODE_PAGE_MIN_ADDRESS   0x8010A000

enum MAILBOX_CMD_TYPE {
    BASEBAND_REGISTER_CMD                       = 0x00,
    HCI_DMA_REGISTER_CMD                        = 0x01,
    TIMMER_REGISTER_CMD                         = 0x02,
    VENDOR_BZDMA_EFUSE_REGISTER_CMD             = 0x03,
    GPIO_REGISTER_CMD                           = 0x04,
    UART_REGISTER_CMD                           = 0x05,
    HCI_UART_REGISTER_CMD                       = 0x06,
    WRITE_DMEM_EFUSE_CMD                        = 0x07,
    MODEM_REGISTER_CMD                          = 0x08,
    RF_REGISTER_CMD                             = 0x09,
    WRITE_DATA_ADDRESS_CMD                      = 0x0A,
    MAILBOX_ID_0x0B                             = 0x0B,
    MAILBOX_ID_0x0C                             = 0x0C,
    MAILBOX_ID_0x0D                             = 0x0D,
    MAILBOX_ID_0x0E                             = 0x0E,
    MAILBOX_ID_0x0F                             = 0x0F,
    MAILBOX_ID_0x10                             = 0x10,
    WIFI_CHANNEL_AND_BANDWIDTH_CMD              = 0x11,
    MAILBOX_ID_0x12                             = 0x12,
    MAILBOX_ID_0x13                             = 0x13,
    MAILBOX_ID_0x14                             = 0x14,
    WIFI_CL_EN                                  = 0x15,
    MAILBOX_ID_0x16                             = 0x16,
    WIFI_FORCE_TX_POWER_CMD                     = 0x17,
    MAILBOX_ID_0x18                             = 0x18,
    MAILBOX_ID_0x19                             = 0x19,
    MAILBOX_ID_0x1A                             = 0x1A,
    BT_ENABLE_IGNORE_WLAN_ACT_CMD               = 0x1B,
    MAILBOX_ID_0x1C                             = 0x1C,
    ENABLE_ISOLATION_MEASURE                    = 0x1D,
    MAILBOX_ID_0x1E                             = 0x1E,
    MAILBOX_ID_0x1F                             = 0x1F,
    WIFI_RESET_BT_CMD                           = 0x20,
    MAILBOX_ID_0x21                             = 0x21,
    MAILBOX_ID_0x22                             = 0x22,
    BT_REPORT_CONN_SCO_INQ_INFO                 = 0x23,
    MAILBOX_ID_0x24                             = 0x24,
    MODEM_REGISTER_CMD_PI                       = 0x25,
    MAILBOX_ID_0x26                             = 0x26,
    BT_AUTO_REPORT_STATUS_INFO 	                = 0x27,
    MAILBOX_ID_0x28		                        = 0x28,
    MAILBOX_ID_0x29	                            = 0x29,
    MAILBOX_ID_0x2A			                    = 0x2A,
    MB_MP_CMD_GROUP                             = 0x30,
    MAILBOX_ID_0x31					            = 0x31,
	MAILBOX_ID_0x32		                        = 0x32,
	BT_LOOPBACK					                = 0x33,
	BT_IQK_MODE									= 0x34,
	MAILBOX_ID_0x35								= 0x35,
	MAILBOX_ID_0x36		    					= 0x36,
	MAILBOX_ID_0x37     						= 0x37,
	BT_PAGE_SCAN_INTERVAL						= 0x38,
	BT_A2DP_EMPTY								= 0x39,
	MWS_CONTROL_INFO							= 0x3A,
	I2C_ENABLE		                            = 0x3F,
};
#define RTK_VENDOR_DW_READ(x, y)         RD_32BIT_IO(x, y)
#define RTK_VENDOR_DW_WRITE(x, y, z)     WR_32BIT_IO(x, y, z)
#define RTK_VENDOR_W_READ(x, y)          RD_16BIT_IO(x, y)
#define RTK_VENDOR_W_WRITE(x, y, z)      WR_16BIT_IO(x, y, z)

typedef struct MAILBOX_BUF_S{
    UINT32 buf_1;
    UINT32 buf_2;
    UINT32 reserved[3];
}MAILBOX_BUF_S_TYPE;


typedef union MAILBOX_CTL_INFO{
    UINT32 d32;
    struct
    {
        UINT32 out_ready          :1;   /* bit[0], Out mailbox data ready flag.
                                           CPU can only update OutData when this
                                           bit is 0. After updating OutData,
                                           CPU should set OutReady to 1.
                                           CPU writes 0 has no effect. (R/W1O)*/
        UINT32 out_empty_int_en   :1;   /* bit[1], After OutData has been read
                                           by remote side (OutReady == 0),
                                           CPU will receive interrupt if
                                           this bit is set to 1. (R/W) */
        UINT32 reserved_2_15      :14;  /* bit[15:2], reserved */
        UINT32 in_ready           :1;   /* bit[16], In mailbox data ready
                                           status. After CPU read InData, it
                                           should write 1 to clear this bit.
                                           (R/W1C) */
        UINT32 in_ready_int_en    :1;   /* bit[17], Enable interrupt of In
                                           mailbox data ready. (R/W) */
        UINT32 reserved_18_31     :14;  /* bit[31:18], reserved  */

    }b;
}MAILBOX_CTL_INFO_TYPE;

typedef union PKT_HAEDER{
    UINT8 d8;
    struct
    {
        UINT8 cmd_type           :6;
        UINT8 is_16b             :1;
        UINT8 is_write           :1;
    }b;
}PKT_HEADER_TYPE;


typedef union WIFI_INFO_DW1{
    UINT32 d32;
    struct
    {
        UINT32 cmd_id           :8;
        UINT32 len                 :8;
        UINT32 wifi_connect   :8;
        UINT32 wifi_ch           :8;
    }b;
}WIFI_INFO_DW1_TYPE;

typedef union WIFI_INFO_DW2{
    UINT32 d32;
    struct
    {
        UINT32 wifi_bw                :8;
        UINT32 ch_mask_num      :8;
        UINT32 reserved_16_31   :16;
    }b;
}WIFI_INFO_DW2_TYPE;


#define MP_MBOX_ERROR_CODE_OK                       0
#define MP_MBOX_ERROR_CODE_VERSION_MISMATCH         1
#define MP_MBOX_ERROR_CODE_UNKNOWN_OPCODE           2
#define MP_MBOX_ERROR_CODE_WRONG_PARAMETERS         3
#define MP_MBOX_ERROR_CODE_COMMAND_DISALLOW         4
#define PATCH_MP_FW_VERSION                         0x00
#define MP_OP_VER                                   0x01

typedef struct MAILBOX_WIFI_MP_MISC_PARAM_S_ {
    UINT16 reg_addr_w;
    UINT16 reg_addr_r;
    UINT32 reg_value_w;
    UINT8 reg_type_w;
    UINT8 reg_type_r;
    UINT8 reg_w_state;
} MAILBOX_WIFI_MP_MISC_PARAM_S;

typedef union MAILBOX_WIFI_MP_CMD_S_ {
    struct {
        UINT8 id;                   /* octet 0 */
        UINT8 seq;                  /* octet 1 */
        UINT8 sub_id;               /* octet 2 */
        union {
            UINT8 rd_data[5];       /* octet 3 ~ 7 */
            struct {
                UINT8 status;       /* octet 3 */
                UINT8 wr_data[4];   /* octet 4 ~ 7 */
            };
        };
    };
    UINT32 DWord[2];                /* octet 0 ~ 7 */
} MAILBOX_WIFI_MP_CMD_S;

enum MAILBOX_CMD30_SUB_IDX {
    /* Request Group */
    MCMD30_MP_BT_OP_GET_BT_CAPABILITY_REQ = 0,
    MCMD30_MP_BT_OP_RESET_REQ = 1,
    MCMD30_MP_BT_OP_TEST_CTRL_REQ = 2,
    MCMD30_MP_BT_OP_SET_BT_MODE_REQ = 3,
    MCMD30_MP_BT_OP_SET_CHNL_REQ = 4,
    MCMD30_MP_BT_OP_SET_PKT_TYPE_LEN_REQ = 5,
    MCMD30_MP_BT_OP_SET_PKT_CNT_L_PK_TYPE_REQ = 6,
    MCMD30_MP_BT_OP_SET_PKT_CNT_H_PKT_INTV_REQ = 7,
    MCMD30_MP_BT_OP_SET_PKT_HEADER_REQ = 8,
    MCMD30_MP_BT_OP_SET_WHITENCOEFF_REQ = 9,
    MCMD30_MP_BT_OP_SET_BD_ADDR_L_REQ = 10,
    MCMD30_MP_BT_OP_SET_BD_ADDR_H_REQ = 11,
    MCMD30_MP_BT_OP_WR_REG_ADDR_REQ = 12,
    MCMD30_MP_BT_OP_WR_REG_VALUE_REQ = 13,
    MCMD30_MP_BT_OP_GET_BT_STATUS_REQ = 14,
    MCMD30_MP_BT_OP_GET_BD_ADDR_L_REQ = 15,
    MCMD30_MP_BT_OP_GET_BD_ADDR_H_REQ = 16,
    MCMD30_MP_BT_OP_RD_REG_REQ = 17,
    MCMD30_MP_BT_OP_SET_TARGET_BD_ADDR_L_REQ = 18,
    MCMD30_MP_BT_OP_SET_TARGET_BD_ADDR_H_REQ = 19,
    MCMD30_MP_BT_OP_SET_TX_POWER_CALIBRATION_REQ = 20,
    MCMD30_MP_BT_OP_GET_RX_PKT_CNT_L_REQ = 21,
    MCMD30_MP_BT_OP_GET_RX_PKT_CNT_H_REQ = 22,
    MCMD30_MP_BT_OP_GET_RX_ERROR_BITS_L_REQ = 23,
    MCMD30_MP_BT_OP_GET_RX_ERROR_BITS_H_REQ = 24,
    MCMD30_MP_BT_OP_GET_RSSI_REQ = 25,
    MCMD30_MP_BT_OP_GET_CFO_HDR_QUALITY_L_REQ = 26,
    MCMD30_MP_BT_OP_GET_CFO_HDR_QUALITY_H_REQ = 27,
    MCMD30_MP_BT_OP_GET_TARGET_BD_ADDR_L_REQ = 28,
    MCMD30_MP_BT_OP_GET_TARGET_BD_ADDR_H_REQ = 29,
    MCMD30_MP_BT_OP_GET_AFH_MAP_L = 30,
    MCMD30_MP_BT_OP_GET_AFH_MAP_M = 31,
    MCMD30_MP_BT_OP_GET_AFH_MAP_H = 32,
    MCMD30_MP_BT_OP_GET_AFH_STATUS = 33,
    MCMD30_MP_BT_OP_SET_TRACKING_INTERVAL = 34,
    MCMD30_MP_BT_OP_SETTHERMAL_METER = 35,
    MCMD30_MP_BT_OP_SET_CFO_TRACKING = 36,
    MCMD30_MP_BT_OP_SET_FW_POLICY_REQ = 37,
    MCMD30_MP_BT_OP_SET_ANT_DETECTION = 38,
    MCMD30_MP_BT_OP_GET_IQK_FLOW = 39,
	MCMD30_MP_BT_OP_GET_IQK_RESULT = 40,
    MCMD30_MP_REQ_LAST = MCMD30_MP_BT_OP_GET_IQK_RESULT,
};

extern MAILBOX_WIFI_MP_MISC_PARAM_S patch_mp_misc_param;

void mailbox_init();
UINT8 pf_os_trigger_mailbox_task(UINT16 type, UINT32 data_1, UINT32 data_2);
UCHAR mailbox_task(OS_SIGNAL *signal_ptr);
UINT32 mailbox_read_write_register(UINT8 type, UINT8 is_write, UINT32 reg_address);
UINT32 mailbox_read_write_rf_modem_register(UINT8 addr, UINT8 type, UINT32 value, UINT8 is_write);
#ifdef _NEW_MODEM_PI_ACCESS_
UINT32 mailbox_read_write_modem_register_pi(UINT8 modem_page, UINT8 addr,
                                            UINT32 value, UINT8 is_write);
#endif
void mailbox_handle_command(UINT32* in_data);
void mailbox_set_afh_map(UINT8 wifi_ch, UINT8 *p_map);
void mailbox_interrupt_handler(void);
void mailbox_task_check(void);
void mailbox_reset_task(void);
void mailbox_bt_report_info(UINT8); //20120809 morgan add

#endif//_OTP_H_

