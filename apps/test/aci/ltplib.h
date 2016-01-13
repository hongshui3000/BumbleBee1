#ifndef     _LTPLIB_H_
#define     _LTPLIB_H_

#include "rtl_types.h"




/****************************************************************************/
/* target specific application context to be handed over to the application */
/* this constand is used by the BTLTPTgtxxx function calls of this library  */
#define LTP_TGT_APPHANDLE   PVOID

#define F_LTPLIB_ASYNC_ASSEMBLY_SUPPORT 0  /*LTP library includes asynchronous LTP_RESET_REQ detection*/

/* definition of valid LTP response causes                                  */
#define LTP_CAUSE_SUCCESS                                                 0x00
#define LTP_CAUSE_ACCEPT                                                  0x01
#define LTP_CAUSE_REJECT                                                  0x02
#define LTP_CAUSE_RESOURCE_ERROR                                          0x03
#define LTP_CAUSE_INVALID_PARAMETER                                       0x04
#define LTP_CAUSE_INVALID_STATE                                           0x05
#define LTP_CAUSE_CONNECTION_DISCONNECT                                   0x06
#define LTP_CAUSE_CONNECTION_DISCONNECTED                                 LTP_CAUSE_CONNECTION_DISCONNECT
#define LTP_CAUSE_CONNECTION_LOST                                         0x07
#define LTP_CAUSE_AUTHENTICATION_FAILED                                   0x08
#define LTP_CAUSE_INIT_TIMEOUT                                            0x09
#define LTP_CAUSE_INIT_OUT_OF_SYNC                                        0x0A
#define LTP_CAUSE_INIT_HARDWARE_fAILURE                                   0x0B
#define LTP_CAUSE_CONNECTION_PAUSED                                       0x30
#define LTP_CAUSE_FLOWCONTROL_VIOLATION                                   0x31
#define LTP_CAUSE_UNSPECIFIED                                             0xFD
#define LTP_CAUSE_NOT_SUPPORTED                                           0xFE

/* definition of locally generated internal Event types                     */
#define LTP_INTERNAL_EVENT_COMMUNICATION_OUT_OF_SYNC                      0x40
#define LTP_INTERNAL_EVENT_MALFORMED_MSG_RECEIVED                         0x41
#define LTP_INTERNAL_EVENT_INVALID_DATA_RECEIVED                          0x42



/****************************************************************************/
/****************************************************************************/
/****************************************************************************/
/****************************************************************************/
/**                                                                        **/
/** 3) Internal functionality used by the LTP-Lib                          **/
/**                                                                        **/
/****************************************************************************/
/****************************************************************************/
/****************************************************************************/
/****************************************************************************/

/* macro definition for read/write uint16_t from/to uint8_t memory                 */
#define NETSHORT2CHAR(p,w)                 \
    *((p)+1) = (uint8_t)((w) & 0xff);         \
    *(p)     = /*lint -e(572,778)*/ (uint8_t)(((w)>>8) & 0xff)

#define NETCHAR2SHORT(p) ((*((p)+1)) & 0xff) + ((*(p)) << 8)

#define NETCHAR2LONG(p) ((uint32_t)(*((p)+3)) & 0xff) + ((uint32_t)(*((p)+2)) << 8) \
    + ((uint32_t)(*((p)+1)) << 16)  + ((uint32_t)(*((p)+0)) << 24)

#define NET24BIT2CHAR(p,w)                  \
    *((p)+2) = (uint8_t)((w) & 0xff);          \
    *((p)+1) = /*lint -e(572,778)*/ (uint8_t)(((w)>>8) & 0xff);     \
    *((p)+0) = /*lint -e(572,778)*/ (uint8_t)(((w)>>16) & 0xff);    \
     
#define NETLONG2CHAR(p,w)                   \
    *((p)+3) = (uint8_t)((w) & 0xff);          \
    *((p)+2) = /*lint -e(572,778)*/ (uint8_t)(((w)>>8) & 0xff);     \
    *((p)+1) = /*lint -e(572,778)*/ (uint8_t)(((w)>>16) & 0xff);    \
    *((p)+0) = /*lint -e(572,778)*/ (uint8_t)(((w)>>24) & 0xff)


/* macro definition to identity internal event location                     */
#define LTP_GENERATE_EVENT_ID  ((0x00)<<24 | (((LTP_SOURCE_FILE_ID) & 0xFF)<<16) | ((__LINE__) & 0xFFFF))

/* message definitions                                                      */
#define LTP_DATA_MIN_HEADER_LENGTH                                           4
#define LTP_OPT_MASK_HEADER_CRC8                                          0x80

#define LTP_OPCODE_RESERVED                                               0x00

/* definitions of message properties                                        */
#define LTP_MDC_MSG         0x01
#define LTP_MDH_MSG         0x02
#define LTP_VAR_LEN_MSG     0x04
#define LTP_CNF_MSG         0x08

/*--------------------------------------------------------------------------*/
/* CONNECT_MDL_INFO                                                         */
#define LTP_CONNECT_MDL_INFO                                              0x04
#define LTP_CONNECT_MDL_INFO_LENGTH                                          9
#define LTP_CONNECT_MDL_INFO_FLAGS                               (LTP_MDC_MSG)

#define LTP_CONNECT_MDL_INFO_OPT_MASK_LINK_TYPE                           0x01
#define LTP_CONNECT_MDL_INFO_OPT_MASK_MAX_TPDU_US_CREDITS                 0x02
#define LTP_CONNECT_MDL_INFO_OPT_MASK_MAX_TPDU_DS_CREDITS                 0x04

/*--------------------------------------------------------------------------*/
/* CREATE_MDL_CNF                                                           */
#define LTP_CREATE_MDL_CNF                                                0x05
#define LTP_CREATE_MDL_CNF_LENGTH                                            6
#define LTP_CREATE_MDL_CNF_FLAGS                     (LTP_CNF_MSG|LTP_MDH_MSG)

#define LTP_CREATE_MDL_CNF_OPT_MASK_MAX_TPDU_US_CREDITS                   0x01
#define LTP_CREATE_MDL_CNF_OPT_MASK_LINK_TYPE                             0x02

/*--------------------------------------------------------------------------*/
/* DELETE_MDL_INFO                                                          */
#define LTP_DELETE_MDL_INFO                                               0x06
#define LTP_DELETE_MDL_INFO_LENGTH                                           5
#define LTP_DELETE_MDL_INFO_FLAGS                                (LTP_MDC_MSG)

/*--------------------------------------------------------------------------*/
/* DISCONNECT_MDL_RSP                                                       */
#define LTP_DISCONNECT_MDL_RSP                                            0x07
#define LTP_DISCONNECT_MDL_RSP_LENGTH                                        6
#define LTP_DISCONNECT_MDL_RSP_FLAGS                             (LTP_MDC_MSG)

/*--------------------------------------------------------------------------*/
/* DISCONNECT_MDL_CNF                                                       */
#define LTP_DISCONNECT_MDL_CNF                                            0x08
#define LTP_DISCONNECT_MDL_CNF_LENGTH                                        5
#define LTP_DISCONNECT_MDL_CNF_FLAGS                 (LTP_CNF_MSG|LTP_MDH_MSG)

#define LTP_DISCONNECT_MDL_CNF_OPT_MASK_LOC_MDL_ID                        0x01


/*--------------------------------------------------------------------------*/
/* EXIT_RSP                                                                 */
#define LTP_EXIT_RSP                                                      0x09
#define LTP_EXIT_RSP_LENGTH                                                  5
#define LTP_EXIT_RSP_FLAGS                                       (LTP_MDC_MSG)

/*--------------------------------------------------------------------------*/
/* ACT_INFO                                                                 */
#define LTP_ACT_INFO                                                      0x0A
#define LTP_ACT_INFO_LENGTH                                                 12
#define LTP_ACT_INFO_FLAGS                       (LTP_VAR_LEN_MSG|LTP_MDC_MSG)

#define LTP_ACT_INFO_OPT_MASK_MAX_RX_MESSAGE_LENGTH                       0x03
#define LTP_ACT_INFO_OPT_MASK_MAX_TX_MESSAGE_LENGTH                       0x0C

/*--------------------------------------------------------------------------*/
/* LTP_ACL_STATUS_INFO                                                      */
#define LTP_ACL_STATUS_INFO                                               0x0B
#define LTP_ACL_STATUS_INFO_LENGTH                                          11
#define LTP_ACL_STATUS_INFO_FLAGS                                (LTP_MDC_MSG)

#define LTP_ACL_STATUS_INFO_OPT_MASK_BD_TYPE                              0x01
#define LTP_ACL_STATUS_INFO_OPT_MASK_KEY_TYPE                             0x02
#define LTP_ACL_STATUS_INFO_OPT_MASK_KEY_SIZE                             0x04
#define LTP_ACL_STATUS_INFO_OPT_MASK_SNIFF_INTERVAL                       0x18

/*--------------------------------------------------------------------------*/
/* RESET_RSP                                                                */
#define LTP_RESET_RSP                                                     0x0C
#define LTP_RESET_RSP_LENGTH                                                 5
#define LTP_RESET_RSP_FLAGS                                      (LTP_MDC_MSG)

/*--------------------------------------------------------------------------*/
/* LTP_INTERNAL_EVENT_INFO                                                  */
#define LTP_INTERNAL_EVENT_INFO                                           0x0D
#define LTP_INTERNAL_EVENT_INFO_LENGTH                                      10
#define LTP_INTERNAL_EVENT_INFO_FLAGS                            (LTP_MDC_MSG)

/*--------------------------------------------------------------------------*/
/* LTP_PASSKEY_REQUEST_CNF                                                  */
#define LTP_PASSKEY_REQUEST_CNF                                           0x0E
#define LTP_PASSKEY_REQUEST_CNF_LENGTH                                      11
#define LTP_PASSKEY_REQUEST_CNF_FLAGS                (LTP_CNF_MSG|LTP_MDH_MSG)

/*--------------------------------------------------------------------------*/
/* LTP_REMOTE_OOB_REQUEST_CNF                                               */
#define LTP_REMOTE_OOB_REQUEST_CNF                                        0x0F
#define LTP_REMOTE_OOB_REQUEST_CNF_LENGTH                                   27
#define LTP_REMOTE_OOB_REQUEST_CNF_FLAGS             (LTP_CNF_MSG|LTP_MDH_MSG)


/*--------------------------------------------------------------------------*/
/* LTP_MCL_STATUS_INFO                                                      */
#define LTP_MCL_STATUS_INFO                                               0x10
#define LTP_MCL_STATUS_INFO_LENGTH                                          12
#define LTP_MCL_STATUS_INFO_FLAGS                                (LTP_MDC_MSG)

/*--------------------------------------------------------------------------*/
/* LTP_PAIRABLE_MODE_SET_RSP                                                */
#define LTP_PAIRABLE_MODE_SET_RSP                                         0x11
#define LTP_PAIRABLE_MODE_SET_RSP_LENGTH                                     5
#define LTP_PAIRABLE_MODE_SET_RSP_FLAGS                          (LTP_MDC_MSG)

/*--------------------------------------------------------------------------*/
/* LTP_PASSKEY_REQ_REPLY_RSP                                                */
#define LTP_PASSKEY_REQ_REPLY_RSP                                         0x12
#define LTP_PASSKEY_REQ_REPLY_RSP_LENGTH                                     5
#define LTP_PASSKEY_REQ_REPLY_RSP_FLAGS                          (LTP_MDC_MSG)

/*--------------------------------------------------------------------------*/
/* LTP_PASSKEY_NOTIFICATION_INFO                                            */
#define LTP_PASSKEY_NOTIFICATION_INFO                                     0x13
#define LTP_PASSKEY_NOTIFICATION_INFO_LENGTH                                14
#define LTP_PASSKEY_NOTIFICATION_INFO_FLAGS                      (LTP_MDC_MSG)

/*--------------------------------------------------------------------------*/
/* CONNECT_GATT_MDL_RSP                                                     */
#define LTP_CONNECT_GATT_MDL_RSP                                          0x14
#define LTP_CONNECT_GATT_MDL_RSP_LENGTH                                     13
#define LTP_CONNECT_GATT_MDL_RSP_FLAGS                           (LTP_MDC_MSG)

/*--------------------------------------------------------------------------*/
/* GATT_SERVICE_REGISTER_RSP */
#define LTP_GATT_SERVICE_REGISTER_RSP                                     0x15
#define LTP_GATT_SERVICE_REGISTER_RSP_LENGTH                                 8
#define LTP_GATT_SERVICE_REGISTER_RSP_FLAGS                      (LTP_MDC_MSG)

/*--------------------------------------------------------------------------*/
/* GATT_ATTRIBUTE_UPDATE_RSP */
#define LTP_GATT_ATTRIBUTE_UPDATE_RSP                                     0x16
#define LTP_GATT_ATTRIBUTE_UPDATE_RSP_LENGTH                                14
#define LTP_GATT_ATTRIBUTE_UPDATE_RSP_FLAGS      (LTP_VAR_LEN_MSG|LTP_MDC_MSG)

/*--------------------------------------------------------------------------*/
/* GATT_ATTRIBUTE_UPDATE_STATUS_CNF */
#define LTP_GATT_ATTRIBUTE_UPDATE_STATUS_CNF                              0x17
#define LTP_GATT_ATTRIBUTE_UPDATE_STATUS_CNF_LENGTH                         12
#define LTP_GATT_ATTRIBUTE_UPDATE_STATUS_CNF_FLAGS   (LTP_CNF_MSG|LTP_MDH_MSG)

/*--------------------------------------------------------------------------*/
/* GATT_ATTRIBUTE_READ_CNF */
#define LTP_GATT_ATTRIBUTE_READ_CNF                                       0x18
#define LTP_GATT_ATTRIBUTE_READ_CNF_LENGTH                                  11
#define LTP_GATT_ATTRIBUTE_READ_CNF_FLAGS        (LTP_VAR_LEN_MSG|LTP_CNF_MSG|LTP_MDH_MSG)

/*--------------------------------------------------------------------------*/
/* GATT_ATTRIBUTE_WRITE_CNF */
#define LTP_GATT_ATTRIBUTE_WRITE_CNF                                      0x19
#define LTP_GATT_ATTRIBUTE_WRITE_CNF_LENGTH                                 11
#define LTP_GATT_ATTRIBUTE_WRITE_CNF_FLAGS           (LTP_CNF_MSG|LTP_MDH_MSG)

/*--------------------------------------------------------------------------*/
/* GATT_CCCD_INFO */
#define LTP_GATT_CCCD_INFO                                                0x1A
#define LTP_GATT_CCCD_INFO_LENGTH                                            6
#define LTP_GATT_CCCD_INFO_FLAGS                 (LTP_VAR_LEN_MSG|LTP_MDC_MSG)

/*--------------------------------------------------------------------------*/
/* GATT_DISCOVERY_RSP */
#define LTP_GATT_DISCOVERY_RSP                                            0x1B
#define LTP_GATT_DISCOVERY_RSP_LENGTH                                        9
#define LTP_GATT_DISCOVERY_RSP_FLAGS                             (LTP_MDC_MSG)

/*--------------------------------------------------------------------------*/
/* GATT_DISCOVERY_CNF */
#define LTP_GATT_DISCOVERY_CNF                                            0x1C
#define LTP_GATT_DISCOVERY_CNF_LENGTH                                       11
#define LTP_GATT_DISCOVERY_CNF_FLAGS                 (LTP_CNF_MSG|LTP_MDH_MSG)

/*--------------------------------------------------------------------------*/
/* GATT_ATTRIBUTE_READ_RSP */
#define LTP_GATT_ATTRIBUTE_READ_RSP                                       0x1D
#define LTP_GATT_ATTRIBUTE_READ_RSP_LENGTH                                  14
#define LTP_GATT_ATTRIBUTE_READ_RSP_FLAGS        (LTP_VAR_LEN_MSG|LTP_MDC_MSG)

/*--------------------------------------------------------------------------*/
/* GATT_ATTRIBUTE_WRITE_RSP */
#define LTP_GATT_ATTRIBUTE_WRITE_RSP                                      0x1E
#define LTP_GATT_ATTRIBUTE_WRITE_RSP_LENGTH                                  9
#define LTP_GATT_ATTRIBUTE_WRITE_RSP_FLAGS                       (LTP_MDC_MSG)

/*--------------------------------------------------------------------------*/
/* GATT_ATTRIBUTE_CNF */
#define LTP_GATT_ATTRIBUTE_CNF                                            0x1F
#define LTP_GATT_ATTRIBUTE_CNF_LENGTH                                        6
#define LTP_GATT_ATTRIBUTE_CNF_FLAGS                 (LTP_CNF_MSG|LTP_MDH_MSG)

/*--------------------------------------------------------------------------*/
/* GATT_ATTRIBUTE_NOTIFICATION_INFO */
#define LTP_GATT_ATTRIBUTE_NOTIFICATION_INFO                              0x20
#define LTP_GATT_ATTRIBUTE_NOTIFICATION_INFO_LENGTH                          7
#define LTP_GATT_ATTRIBUTE_NOTIFICATION_INFO_FLAGS           (LTP_VAR_LEN_MSG|LTP_MDC_MSG)

/*--------------------------------------------------------------------------*/
/* LE_ADVERTISE_RSP */
#define LTP_LE_ADVERTISE_RSP                                              0x21
#define LTP_LE_ADVERTISE_RSP_LENGTH                                          6
#define LTP_LE_ADVERTISE_RSP_FLAGS                               (LTP_MDC_MSG)

/*--------------------------------------------------------------------------*/
/* LE_ADVERTISE_PARAMETER_SET_RSP */
#define LTP_LE_ADVERTISE_PARAMETER_SET_RSP                                0x22
#define LTP_LE_ADVERTISE_PARAMETER_SET_RSP_LENGTH                            5
#define LTP_LE_ADVERTISE_PARAMETER_SET_RSP_FLAGS                 (LTP_MDC_MSG)

/*--------------------------------------------------------------------------*/
/* LE_ADVERTISE_DATA_SET_RSP */
#define LTP_LE_ADVERTISE_DATA_SET_RSP                                     0x23
#define LTP_LE_ADVERTISE_DATA_SET_RSP_LENGTH                                 6
#define LTP_LE_ADVERTISE_DATA_SET_RSP_FLAGS                      (LTP_MDC_MSG)

/*--------------------------------------------------------------------------*/
/* LE_SCAN_RSP */
#define LTP_LE_SCAN_RSP                                                   0x24
#define LTP_LE_SCAN_RSP_LENGTH                                               5
#define LTP_LE_SCAN_RSP_FLAGS                                    (LTP_MDC_MSG)

/*--------------------------------------------------------------------------*/
/* LE_SCAN_INFO */
#define LTP_LE_SCAN_INFO                                                  0x25
#define LTP_LE_SCAN_INFO_LENGTH                                             13
#define LTP_LE_SCAN_INFO_FLAGS                   (LTP_VAR_LEN_MSG|LTP_MDC_MSG)

/*--------------------------------------------------------------------------*/
/* LE_MODIFY_WHITELIST_RSP */
#define LTP_LE_MODIFY_WHITELIST_RSP                                       0x26
#define LTP_LE_MODIFY_WHITELIST_RSP_LENGTH                                   6
#define LTP_LE_MODIFY_WHITELIST_RSP_FLAGS                        (LTP_MDC_MSG)

/*--------------------------------------------------------------------------*/
/* LE_CONNECTION_UPDATE_RSP */
#define LTP_LE_CONNECTION_UPDATE_RSP                                      0x27
#define LTP_LE_CONNECTION_UPDATE_RSP_LENGTH                                  6
#define LTP_LE_CONNECTION_UPDATE_RSP_FLAGS                       (LTP_MDC_MSG)

/*--------------------------------------------------------------------------*/
/* LE_CONNECTION_UPDATE_CNF */
#define LTP_LE_CONNECTION_UPDATE_CNF                                      0x28
#define LTP_LE_CONNECTION_UPDATE_CNF_LENGTH                                  6
#define LTP_LE_CONNECTION_UPDATE_CNF_FLAGS           (LTP_CNF_MSG|LTP_MDH_MSG)

/*--------------------------------------------------------------------------*/
/* LE_CONNECTION_PARAMETER_INFO */
#define LTP_LE_CONNECTION_PARAMETER_INFO                                  0x29
#define LTP_LE_CONNECTION_PARAMETER_INFO_LENGTH                             11
#define LTP_LE_CONNECTION_PARAMETER_INFO_FLAGS                   (LTP_MDC_MSG)

/*--------------------------------------------------------------------------*/
/* GATT_SERVER_STORE_CNF */
#define LTP_GATT_SERVER_STORE_CNF                                         0x2A
#define LTP_GATT_SERVER_STORE_CNF_LENGTH                                    15
#define LTP_GATT_SERVER_STORE_CNF_FLAGS          (LTP_VAR_LEN_MSG|LTP_CNF_MSG|LTP_MDH_MSG)

/*--------------------------------------------------------------------------*/
/* AUTH_RESULT_EXT_CNF */
#define LTP_AUTH_RESULT_EXT_CNF                                           0x2B
#define LTP_AUTH_RESULT_EXT_CNF_LENGTH                                      12
#define LTP_AUTH_RESULT_EXT_CNF_FLAGS                (LTP_CNF_MSG|LTP_MDH_MSG)

/*--------------------------------------------------------------------------*/
/* AUTH_RESULT_REQUEST_EXT_CNF */
#define LTP_AUTH_RESULT_REQUEST_EXT_CNF                                   0x2C
#define LTP_AUTH_RESULT_REQUEST_EXT_CNF_LENGTH                              13
#define LTP_AUTH_RESULT_REQUEST_EXT_CNF_FLAGS    (LTP_VAR_LEN_MSG|LTP_CNF_MSG|LTP_MDH_MSG)

#define LTP_AUTH_RESULT_REQUEST_EXT_CNF_OPT_MASK_RESTART_HANDLE           0x03

/*--------------------------------------------------------------------------*/
/* GATT_SECURITY_RSP */
#define LTP_GATT_SECURITY_RSP                                             0x2D
#define LTP_GATT_SECURITY_RSP_LENGTH                                         8
#define LTP_GATT_SECURITY_RSP_FLAGS                              (LTP_MDC_MSG)

/*--------------------------------------------------------------------------*/
/* GATT_MTU_INFO */
#define LTP_GATT_MTU_INFO                                                 0x2E
#define LTP_GATT_MTU_INFO_LENGTH                                             7
#define LTP_GATT_MTU_INFO_FLAGS                                  (LTP_MDC_MSG)

/*--------------------------------------------------------------------------*/
/* GATT_ATTRIBUTE_WRITE_COMMAND_INFO */
#define LTP_GATT_ATTRIBUTE_WRITE_COMMAND_INFO                             0x2F
#define LTP_GATT_ATTRIBUTE_WRITE_COMMAND_INFO_LENGTH                        12
#define LTP_GATT_ATTRIBUTE_WRITE_COMMAND_INFO_FLAGS          (LTP_VAR_LEN_MSG|LTP_MDC_MSG)

/*--------------------------------------------------------------------------*/
/* 0x7D reserved (LTP_EXTEND_COMMAND)*/


/*--------------------------------------------------------------------------*/
/* 0x84 reserved (LTP_CONNECT_MDL_INFO)*/


/* CREATE_MDL_IND                                                           */
#define LTP_CREATE_MDL_IND                                                0x85
#define LTP_CREATE_MDL_IND_LENGTH                                           11
#define LTP_CREATE_MDL_IND_FLAGS                                 (LTP_MDC_MSG)

#define LTP_CREATE_MDL_IND_OPT_MASK_BD_TYPE                               0x01
#define LTP_CREATE_MDL_IND_OPT_MASK_LINK_TYPE                             0x02
#define LTP_CREATE_MDL_IND_OPT_MASK_LOC_MDEP_ID                           0x04
#define LTP_CREATE_MDL_IND_OPT_MASK_REM_MDEP_ID                           0x08

/*--------------------------------------------------------------------------*/
/* 0x86 reserved (DeleteMDLInfo)                                            */

/*--------------------------------------------------------------------------*/
/* DISCONNECT_MDL_REQ                                                       */
#define LTP_DISCONNECT_MDL_REQ                                            0x87
#define LTP_DISCONNECT_MDL_REQ_LENGTH                                        6
#define LTP_DISCONNECT_MDL_REQ_FLAGS                             (LTP_MDH_MSG)

/*--------------------------------------------------------------------------*/
/* DISCONNECT_MDL_IND                                                       */
#define LTP_DISCONNECT_MDL_IND                                            0x88
#define LTP_DISCONNECT_MDL_IND_LENGTH                                        6
#define LTP_DISCONNECT_MDL_IND_FLAGS                             (LTP_MDC_MSG)

/*--------------------------------------------------------------------------*/
/* EXIT_REQ                                                                 */
#define LTP_EXIT_REQ                                                      0x89
#define LTP_EXIT_REQ_LENGTH                                                  5
#define LTP_EXIT_REQ_FLAGS                                       (LTP_MDH_MSG)

#define LTP_EXIT_REQ_OPT_MASK_STATUS                                      0x01

/*--------------------------------------------------------------------------*/
/* 0x8A - 0x8B reserved (ActInfo, ACLStatusInfo)                  */

/*--------------------------------------------------------------------------*/
/* RESET_REQ                                                                */
#define LTP_RESET_REQ                                                     0x8C
#define LTP_RESET_REQ_LENGTH                                                 4
#define LTP_RESET_REQ_FLAGS                                      (LTP_MDH_MSG)
#define LTP_RESET_REQ_HEADER_CRC                                          0x04

/*--------------------------------------------------------------------------*/
/* 0x8D reserved (LTP_INTERNAL_EVENT_INFO)                  */

/*--------------------------------------------------------------------------*/
/* LTP_PASSKEY_REQUEST_IND                                                  */
#define LTP_PASSKEY_REQUEST_IND                                           0x8E
#define LTP_PASSKEY_REQUEST_IND_LENGTH                                      10
#define LTP_PASSKEY_REQUEST_IND_FLAGS                            (LTP_MDC_MSG)

/*--------------------------------------------------------------------------*/
/* LTP_REMOTE_OOB_REQUEST_IND                                               */
#define LTP_REMOTE_OOB_REQUEST_IND                                        0x8F
#define LTP_REMOTE_OOB_REQUEST_IND_LENGTH                                   10
#define LTP_REMOTE_OOB_REQUEST_IND_FLAGS                         (LTP_MDC_MSG)

/*--------------------------------------------------------------------------*/
/* 0x90 reserved (MCLStatusInfo)                                            */

/*--------------------------------------------------------------------------*/
/* LTP_PAIRABLE_MODE_SET_REQ                                                */
#define LTP_PAIRABLE_MODE_SET_REQ                                         0x91
#define LTP_PAIRABLE_MODE_SET_REQ_LENGTH                                     8
#define LTP_PAIRABLE_MODE_SET_REQ_FLAGS                          (LTP_MDH_MSG)
#define LTP_PAIRABLE_MODE_SET_REQ_OPT_MASK_BLUETOOTH_MODE                 0x01

/*--------------------------------------------------------------------------*/
/* LTP_PASSKEY_REQ_REPLY_REQ                                                */
#define LTP_PASSKEY_REQ_REPLY_REQ                                         0x92
#define LTP_PASSKEY_REQ_REPLY_REQ_LENGTH                                    15
#define LTP_PASSKEY_REQ_REPLY_REQ_FLAGS                          (LTP_MDH_MSG)


/*--------------------------------------------------------------------------*/
/* 0x93 reserved (KeypressNotificationInfo)                                 */


/*--------------------------------------------------------------------------*/
/* CONNECT_GATT_MDL_REQ */
#define LTP_CONNECT_GATT_MDL_REQ                                          0x94
#define LTP_CONNECT_GATT_MDL_REQ_LENGTH                                     29
#define LTP_CONNECT_GATT_MDL_REQ_FLAGS                           (LTP_MDH_MSG)

/*--------------------------------------------------------------------------*/
/* GATT_SERVICE_REGISTER_REQ */
#define LTP_GATT_SERVICE_REGISTER_REQ                                     0x95
#define LTP_GATT_SERVICE_REGISTER_REQ_LENGTH                                10
#define LTP_GATT_SERVICE_REGISTER_REQ_FLAGS                      (LTP_MDH_MSG)
#define LTP_GATT_SERVICE_REGISTER_REQ_OPT_MASK_SERVICE_ID                 0x01

/*--------------------------------------------------------------------------*/
/* GATT_ATTRIBUTE_UPDATE_REQ */
#define LTP_GATT_ATTRIBUTE_UPDATE_REQ                                     0x96
#define LTP_GATT_ATTRIBUTE_UPDATE_REQ_LENGTH                                11
#define LTP_GATT_ATTRIBUTE_UPDATE_REQ_FLAGS      (LTP_VAR_LEN_MSG|LTP_MDH_MSG)

/*--------------------------------------------------------------------------*/
/* GATT_ATTRIBUTE_UPDATE_STATUS_IND */
#define LTP_GATT_ATTRIBUTE_UPDATE_STATUS_IND                              0x97
#define LTP_GATT_ATTRIBUTE_UPDATE_STATUS_IND_LENGTH                         21
#define LTP_GATT_ATTRIBUTE_UPDATE_STATUS_IND_FLAGS               (LTP_MDC_MSG)

/*--------------------------------------------------------------------------*/
/* GATT_ATTRIBUTE_READ_IND */
#define LTP_GATT_ATTRIBUTE_READ_IND                                       0x98
#define LTP_GATT_ATTRIBUTE_READ_IND_LENGTH                                  10
#define LTP_GATT_ATTRIBUTE_READ_IND_FLAGS                        (LTP_MDC_MSG)

/*--------------------------------------------------------------------------*/
/* GATT_ATTRIBUTE_WRITE_IND */
#define LTP_GATT_ATTRIBUTE_WRITE_IND                                      0x99
#define LTP_GATT_ATTRIBUTE_WRITE_IND_LENGTH                                 12
#define LTP_GATT_ATTRIBUTE_WRITE_IND_FLAGS       (LTP_VAR_LEN_MSG|LTP_MDC_MSG)

/*--------------------------------------------------------------------------*/
/* 0x9A reserved (LTP_GATT_CCCD_INFO) */

/*--------------------------------------------------------------------------*/
/* GATT_DISCOVERY_REQ */
#define LTP_GATT_DISCOVERY_REQ                                            0x9B
#define LTP_GATT_DISCOVERY_REQ_LENGTH                                       12
#define LTP_GATT_DISCOVERY_REQ_FLAGS             (LTP_VAR_LEN_MSG|LTP_MDH_MSG)

/*--------------------------------------------------------------------------*/
/* GATT_DISCOVERY_IND */
#define LTP_GATT_DISCOVERY_IND                                            0x9C
#define LTP_GATT_DISCOVERY_IND_LENGTH                                       10
#define LTP_GATT_DISCOVERY_IND_FLAGS             (LTP_VAR_LEN_MSG|LTP_MDC_MSG)

/*--------------------------------------------------------------------------*/
/* GATT_ATTRIBUTE_READ_REQ */
#define LTP_GATT_ATTRIBUTE_READ_REQ                                       0x9D
#define LTP_GATT_ATTRIBUTE_READ_REQ_LENGTH                                  14
#define LTP_GATT_ATTRIBUTE_READ_REQ_FLAGS        (LTP_VAR_LEN_MSG|LTP_MDH_MSG)

/*--------------------------------------------------------------------------*/
/* GATT_ATTRIBUTE_WRITE_REQ */
#define LTP_GATT_ATTRIBUTE_WRITE_REQ                                      0x9E
#define LTP_GATT_ATTRIBUTE_WRITE_REQ_LENGTH                                 8
#define LTP_GATT_ATTRIBUTE_WRITE_REQ_FLAGS       (LTP_VAR_LEN_MSG|LTP_MDH_MSG)

/*--------------------------------------------------------------------------*/
/* GATT_ATTRIBUTE_IND */
#define LTP_GATT_ATTRIBUTE_IND                                            0x9F
#define LTP_GATT_ATTRIBUTE_IND_LENGTH                                        7
#define LTP_GATT_ATTRIBUTE_IND_FLAGS             (LTP_VAR_LEN_MSG|LTP_MDC_MSG)

/*--------------------------------------------------------------------------*/
/* 0xA0 reserved (LTP_GATT_ATTRIBUTE_NOTIFICATION_INFO) */


/*--------------------------------------------------------------------------*/
/* LE_ADVERTISE_REQ */
#define LTP_LE_ADVERTISE_REQ                                              0xA1
#define LTP_LE_ADVERTISE_REQ_LENGTH                                         5
#define LTP_LE_ADVERTISE_REQ_FLAGS                               (LTP_MDH_MSG)

/*--------------------------------------------------------------------------*/
/* LE_ADVERTISE_PARAMETER_SET_REQ */
#define LTP_LE_ADVERTISE_PARAMETER_SET_REQ                                0xA2
#define LTP_LE_ADVERTISE_PARAMETER_SET_REQ_LENGTH                           19
#define LTP_LE_ADVERTISE_PARAMETER_SET_REQ_FLAGS                 (LTP_MDH_MSG)

/*--------------------------------------------------------------------------*/
/* LE_ADVERTISE_DATA_SET_REQ */
#define LTP_LE_ADVERTISE_DATA_SET_REQ                                     0xA3
#define LTP_LE_ADVERTISE_DATA_SET_REQ_LENGTH                                 5
#define LTP_LE_ADVERTISE_DATA_SET_REQ_FLAGS      (LTP_VAR_LEN_MSG|LTP_MDH_MSG)

/*--------------------------------------------------------------------------*/
/* LE_SCAN_REQ */
#define LTP_LE_SCAN_REQ                                                   0xA4
#define LTP_LE_SCAN_REQ_LENGTH                                              12
#define LTP_LE_SCAN_REQ_FLAGS                                    (LTP_MDH_MSG)

/*--------------------------------------------------------------------------*/
/* 0xA5 reserved (LTP_LE_SCAN_INFO) */

/*--------------------------------------------------------------------------*/
/* LE_MODIFY_WHITELIST_REQ */
#define LTP_LE_MODIFY_WHITELIST_REQ                                       0xA6
#define LTP_LE_MODIFY_WHITELIST_REQ_LENGTH                                  12
#define LTP_LE_MODIFY_WHITELIST_REQ_FLAGS                        (LTP_MDH_MSG)

/*--------------------------------------------------------------------------*/
/* LE_CONNECTION_UPDATE_REQ */
#define LTP_LE_CONNECTION_UPDATE_REQ                                      0xA7
#define LTP_LE_CONNECTION_UPDATE_REQ_LENGTH                                 13
#define LTP_LE_CONNECTION_UPDATE_REQ_FLAGS                       (LTP_MDH_MSG)

/*--------------------------------------------------------------------------*/
/* LE_CONNECTION_UPDATE_IND */
#define LTP_LE_CONNECTION_UPDATE_IND                                      0xA8
#define LTP_LE_CONNECTION_UPDATE_IND_LENGTH                                 13
#define LTP_LE_CONNECTION_UPDATE_IND_FLAGS                       (LTP_MDC_MSG)

/*--------------------------------------------------------------------------*/
/* 0xA9 reserved (LTP_LE_CONNECTION_UPDATE_INFO) */

/*--------------------------------------------------------------------------*/
/* GATT_SERVER_STORE_IND */
#define LTP_GATT_SERVER_STORE_IND                                         0xAA
#define LTP_GATT_SERVER_STORE_IND_LENGTH                                    14
#define LTP_GATT_SERVER_STORE_IND_FLAGS          (LTP_VAR_LEN_MSG|LTP_MDC_MSG)

/*--------------------------------------------------------------------------*/
/* AUTH_RESULT_EXT_IND */
#define LTP_AUTH_RESULT_EXT_IND                                           0xAB
#define LTP_AUTH_RESULT_EXT_IND_LENGTH                                      13
#define LTP_AUTH_RESULT_EXT_IND_FLAGS            (LTP_VAR_LEN_MSG|LTP_MDC_MSG)

/*--------------------------------------------------------------------------*/
/* AUTH_RESULT_REQUEST_EXT_IND */
#define LTP_AUTH_RESULT_REQUEST_EXT_IND                                   0xAC
#define LTP_AUTH_RESULT_REQUEST_EXT_IND_LENGTH                              12
#define LTP_AUTH_RESULT_REQUEST_EXT_IND_FLAGS                    (LTP_MDC_MSG)

#define LTP_AUTH_RESULT_REQUEST_EXT_IND_OPT_MASK_RESTART_HANDLE           0x03

/*--------------------------------------------------------------------------*/
/* GATT_SECURITY_REQ */
#define LTP_GATT_SECURITY_REQ                                             0xAD
#define LTP_GATT_SECURITY_REQ_LENGTH                                         8
#define LTP_GATT_SECURITY_REQ_FLAGS                              (LTP_MDH_MSG)

/*--------------------------------------------------------------------------*/
/* 0xAE reserved (LTP_GATT_MTU_INFO) */

/*--------------------------------------------------------------------------*/
/* 0xAF reserved (LTP_GATT_ATTRIBUTE_WRITE_COMMAND_INFO) */


/*--------------------------------------------------------------------------*/
/* LTP_EXTEND_COMMAND */
#define LTP_EXTEND_COMMAND                                               0xFA
#define LTP_EXTEND_COMMAND_LENGTH                                           5
#define LTP_EXTEND_COMMAND_FLAGS                            (LTP_VAR_LEN_MSG|LTP_MDH_MSG)

/*--------------------------------------------------------------------------*/
/* LTP_SUB_SET_RANDOM_ADDRESS_RSP */
#define LTP_SUB_SET_RANDOM_ADDRESS_RSP                                   0x01
#define LTP_SUB_SET_RANDOM_ADDRESS_RSP_LENGTH                               8
#define LTP_SUB_SET_RANDOM_ADDRESS_RSP_FLAGS                    (LTP_MDC_MSG)

/*--------------------------------------------------------------------------*/
/* LTP_DEVICE_CONFIG_DEVICE_NAME_SET_RSP */
#define LTP_SUB_DEVICE_CONFIG_DEVICE_NAME_SET_RSP                        0x02
#define LTP_SUB_DEVICE_CONFIG_DEVICE_NAME_SET_RSP_LENGTH                    6
#define LTP_SUB_DEVICE_CONFIG_DEVICE_NAME_SET_RSP_FLAGS         (LTP_MDC_MSG)

/*--------------------------------------------------------------------------*/
/* LTP_DEVICE_CONFIG_SECURITY_SET_RSP*/
#define LTP_SUB_DEVICE_CONFIG_SECURITY_SET_RSP                           0x03
#define LTP_SUB_DEVICE_CONFIG_SECURITY_SET_RSP_LENGTH                       6
#define LTP_SUB_DEVICE_CONFIG_SECURITY_SET_RSP_FLAGS            (LTP_MDC_MSG)

/*--------------------------------------------------------------------------*/
/* LTP_DEVICE_CONFIG_STORE_SET_RSP */
#define LTP_SUB_DEVICE_CONFIG_STORE_SET_RSP                              0x04
#define LTP_SUB_DEVICE_CONFIG_STORE_SET_RSP_LENGTH                          6
#define LTP_SUB_DEVICE_CONFIG_STORE_SET_RSP_FLAGS                (LTP_MDC_MSG)

/*--------------------------------------------------------------------------*/
/* LTP_DEVICE_CONFIG_APPEARANCE_SET_RSP */
#define LTP_SUB_DEVICE_CONFIG_APPEARANCE_SET_RSP                         0x05
#define LTP_SUB_DEVICE_CONFIG_APPEARANCE_SET_RSP_LENGTH                     6
#define LTP_SUB_DEVICE_CONFIG_APPEARANCE_SET_RSP_FLAGS           (LTP_MDC_MSG)

/*--------------------------------------------------------------------------*/
/* LTP_DEVICE_CONFIG_PER_PREF_CONN_PARAM_SET_RSP */
#define LTP_SUB_DEVICE_CONFIG_PER_PREF_CONN_PARAM_SET_RSP                0x06
#define LTP_SUB_DEVICE_CONFIG_PER_PREF_CONN_PARAM_SET_RSP_LENGTH            6
#define LTP_SUB_DEVICE_CONFIG_PER_PREF_CONN_PARAM_SET_RSP_FLAGS (LTP_MDC_MSG)

/*--------------------------------------------------------------------------*/
/* LTP_SET_LE_TX_POWER_RSP */
#define LTP_SUB_SET_LE_TX_POWER_RSP                                      0x07
#define LTP_SUB_SET_LE_TX_POWER_RSP_LENGTH                                  9
#define LTP_SUB_SET_LE_TX_POWER_RSP_FLAGS                       (LTP_MDC_MSG)

/*--------------------------------------------------------------------------*/
/* LTP_SET_DATA_LENGTH_RSP */
#define LTP_SUB_SET_DATA_LENGTH_RSP                                      0x08
#define LTP_SUB_SET_DATA_LENGTH_RSP_LENGTH                                  7
#define LTP_SUB_SET_DATA_LENGTH_RSP_FLAGS                       (LTP_MDC_MSG)

/*--------------------------------------------------------------------------*/
/* LTP_DATA_LENGTH_CHANGE_INFO */
#define LTP_SUB_DATA_LENGTH_CHANGE_INFO                                  0x09
#define LTP_SUB_DATA_LENGTH_CHANGE_INFO_LENGTH                             14
#define LTP_SUB_DATA_LENGTH_CHANGE_INFO_FLAGS                   (LTP_MDC_MSG)

/*--------------------------------------------------------------------------*/
/* LTP_DOWNLOAD_SERVICE_DATABASE_RSP */
#define LTP_SUB_DOWNLOAD_SERVICE_DATABASE_RSP                            0x0A
#define LTP_SUB_DOWNLOAD_SERVICE_DATABASE_RSP_LENGTH                        6
#define LTP_SUB_DOWNLOAD_SERVICE_DATABASE_RSP_FLAGS             (LTP_MDC_MSG)

/*--------------------------------------------------------------------------*/
/* LTP_SUB_CLEAR_SERVICE_DATABASE_RSP */
#define LTP_SUB_CLEAR_SERVICE_DATABASE_RSP                               0x0B
#define LTP_SUB_CLEAR_SERVICE_DATABASE_RSP_LENGTH                           6
#define LTP_SUB_CLEAR_SERVICE_DATABASE_RSP_FLAGS                (LTP_MDC_MSG)

/*--------------------------------------------------------------------------*/
/* LTP_SUB_SET_TRACE_LEVEL_RSP */
#define LTP_SUB_SET_TRACE_LEVEL_RSP                                      0x0C
#define LTP_SUB_SET_TRACE_LEVEL_RSP_LENGTH                                  6
#define LTP_SUB_SET_TRACE_LEVEL_RSP_FLAGS                        (LTP_MDC_MSG)

/*--------------------------------------------------------------------------*/
/* LTP_SUB_GATT_ATTRIBUTE_PREPARE_WRITE_CNF */
#define LTP_SUB_GATT_ATTRIBUTE_PREPARE_WRITE_CNF                         0x0D
#define LTP_SUB_GATT_ATTRIBUTE_PREPARE_WRITE_CNF_LENGTH                    12
#define LTP_SUB_GATT_ATTRIBUTE_PREPARE_WRITE_CNF_FLAGS          (LTP_VAR_LEN_MSG|LTP_CNF_MSG|LTP_MDH_MSG)

/*--------------------------------------------------------------------------*/
/* LTP_SUB_GATT_ATTRIBUTE_PREPARE_WRITE_RSP */
#define LTP_SUB_GATT_ATTRIBUTE_PREPARE_WRITE_RSP                         0x0E
#define LTP_SUB_GATT_ATTRIBUTE_PREPARE_WRITE_RSP_LENGTH                    11
#define LTP_SUB_GATT_ATTRIBUTE_PREPARE_WRITE_RSP_FLAGS          (LTP_VAR_LEN_MSG|LTP_MDC_MSG)

/*--------------------------------------------------------------------------*/
/* LTP_SUB_GATT_ATTRIBUTE_EXECUTE_WRITE_CNF */
#define LTP_SUB_GATT_ATTRIBUTE_EXECUTE_WRITE_CNF                         0x0F
#define LTP_SUB_GATT_ATTRIBUTE_EXECUTE_WRITE_CNF_LENGTH                    11
#define LTP_SUB_GATT_ATTRIBUTE_EXECUTE_WRITE_CNF_FLAGS          (LTP_CNF_MSG|LTP_MDH_MSG)

/*--------------------------------------------------------------------------*/
/* LTP_SUB_GATT_ATTRIBUTE_EXECUTE_WRITE_RSP */
#define LTP_SUB_GATT_ATTRIBUTE_EXECUTE_WRITE_RSP                         0x10
#define LTP_SUB_GATT_ATTRIBUTE_EXECUTE_WRITE_RSP_LENGTH                     9
#define LTP_SUB_GATT_ATTRIBUTE_EXECUTE_WRITE_RSP_FLAGS          (LTP_MDC_MSG)


/*--------------------------------------------------------------------------*/
/* LTP_SET_RANDOM_ADDRESS_REQ */
#define LTP_SUB_SET_RANDOM_ADDRESS_REQ                                   0x81
#define LTP_SUB_SET_RANDOM_ADDRESS_LENGTH                                  11
#define LTP_SUB_SET_RANDOM_ADDRESS_FLAGS                        (LTP_MDH_MSG)

/*--------------------------------------------------------------------------*/
/* LTP_DEVICE_CONFIG_DEVICE_NAME_SET_REQ */
#define LTP_SUB_DEVICE_CONFIG_DEVICE_NAME_SET_REQ                        0x82
#define LTP_SUB_DEVICE_CONFIG_DEVICE_NAME_SET_REQ_LENGTH                    1
#define LTP_SUB_DEVICE_CONFIG_DEVICE_NAME_SET_REQ_FLAGS     (LTP_VAR_LEN_MSG|LTP_MDH_MSG)

/*--------------------------------------------------------------------------*/
/* LTP_DEVICE_CONFIG_SECURITY_SET_REQ */
#define LTP_SUB_DEVICE_CONFIG_SECURITY_SET_REQ                           0x83
#define LTP_SUB_DEVICE_CONFIG_SECURITY_SET_REQ_LENGTH                       9
#define LTP_SUB_DEVICE_CONFIG_SECURITY_SET_REQ_FLAGS            (LTP_MDH_MSG)

/*--------------------------------------------------------------------------*/
/* LTP_DEVICE_CONFIG_STORE_SET_REQ */
#define LTP_SUB_DEVICE_CONFIG_STORE_SET_REQ                              0x84
#define LTP_SUB_DEVICE_CONFIG_STORE_SET_REQ_LENGTH                          7
#define LTP_SUB_DEVICE_CONFIG_STORE_SET_REQ_FLAGS               (LTP_MDH_MSG)

/*--------------------------------------------------------------------------*/
/* LTP_DEVICE_CONFIG_APPEARANCE_SET_REQ */
#define LTP_SUB_DEVICE_CONFIG_APPEARANCE_SET_REQ                         0x85
#define LTP_SUB_DEVICE_CONFIG_APPEARANCE_SET_REQ_LENGTH                     7
#define LTP_SUB_DEVICE_CONFIG_APPEARANCE_SET_REQ_FLAGS          (LTP_MDH_MSG)

/*--------------------------------------------------------------------------*/
/* LTP_DEVICE_CONFIG_PER_PREF_CONN_PARAM_SET_REQ */
#define LTP_SUB_DEVICE_CONFIG_PER_PREF_CONN_PARAM_SET_REQ                0x86
#define LTP_SUB_DEVICE_CONFIG_PER_PREF_CONN_PARAM_SET_REQ_LENGTH           13
#define LTP_SUB_DEVICE_CONFIG_PER_PREF_CONN_PARAM_SET_REQ_FLAGS (LTP_MDH_MSG)

/*--------------------------------------------------------------------------*/
/* LTP_SET_LE_TX_POWER_REQ */
#define LTP_SUB_SET_LE_TX_POWER_REQ                                      0x87
#define LTP_SUB_SET_LE_TX_POWER_REQ_LENGTH                                  6
#define LTP_SUB_SET_LE_TX_POWER_REQ_FLAGS                       (LTP_MDH_MSG)

/*--------------------------------------------------------------------------*/
/* LTP_SET_DATA_LENGTH_REQ */
#define LTP_SUB_SET_DATA_LENGTH_REQ                                      0x88
#define LTP_SUB_SET_DATA_LENGTH_REQ_LENGTH                                 10
#define LTP_SUB_SET_DATA_LENGTH_REQ_FLAGS                       (LTP_MDH_MSG)
/*--------------------------------------------------------------------------*/
/* 0x89 reserved (LTP_SUB_DATA_LENGTH_CHANGE_INFO) */

/*--------------------------------------------------------------------------*/
/* LTP_DOWNLOAD_SERVICE_DATABASE_REQ */
#define LTP_SUB_DOWNLOAD_SERVICE_DATABASE_REQ                            0x8A
#define LTP_SUB_DOWNLOAD_SERVICE_DATABASE_REQ_LENGTH                       10
#define LTP_SUB_DOWNLOAD_SERVICE_DATABASE_REQ_FLAGS         (LTP_VAR_LEN_MSG|LTP_MDH_MSG)

/*--------------------------------------------------------------------------*/
/* LTP_SUB_CLEAR_SERVICE_DATABASE_REQ */
#define LTP_SUB_CLEAR_SERVICE_DATABASE_REQ                               0x8B
#define LTP_SUB_CLEAR_SERVICE_DATABASE_REQ_LENGTH                           5
#define LTP_SUB_CLEAR_SERVICE_DATABASE_REQ_FLAGS                (LTP_MDH_MSG)

/*--------------------------------------------------------------------------*/
/* LTP_SUB_SET_TRACE_LEVEL_REQ */
#define LTP_SUB_SET_TRACE_LEVEL_REQ                                      0x8C
#define LTP_SUB_SET_TRACE_LEVEL_REQ_LENGTH                                 10
#define LTP_SUB_SET_TRACE_LEVEL_REQ_FLAGS                       (LTP_MDH_MSG)

/*--------------------------------------------------------------------------*/
/* LTP_SUB_GATT_ATTRIBUTE_PREPARE_WRITE_IND */
#define LTP_SUB_GATT_ATTRIBUTE_PREPARE_WRITE_IND                         0x8D
#define LTP_SUB_GATT_ATTRIBUTE_PREPARE_WRITE_IND_LENGTH                    13
#define LTP_SUB_GATT_ATTRIBUTE_PREPARE_WRITE_IND_FLAGS       (LTP_VAR_LEN_MSG|LTP_MDC_MSG)

/*--------------------------------------------------------------------------*/
/* LTP_SUB_GATT_ATTRIBUTE_PREPARE_WRITE_REQ */
#define LTP_SUB_GATT_ATTRIBUTE_PREPARE_WRITE_REQ                         0x8E
#define LTP_SUB_GATT_ATTRIBUTE_PREPARE_WRITE_REQ_LENGTH                    10
#define LTP_SUB_GATT_ATTRIBUTE_PREPARE_WRITE_REQ_FLAGS       (LTP_VAR_LEN_MSG|LTP_MDH_MSG)

/*--------------------------------------------------------------------------*/
/* LTP_SUB_GATT_ATTRIBUTE_EXECUTE_WRITE_IND */
#define LTP_SUB_GATT_ATTRIBUTE_EXECUTE_WRITE_IND                         0x8F
#define LTP_SUB_GATT_ATTRIBUTE_EXECUTE_WRITE_IND_LENGTH                    7
#define LTP_SUB_GATT_ATTRIBUTE_EXECUTE_WRITE_IND_FLAGS          (LTP_MDC_MSG)

/*--------------------------------------------------------------------------*/
/* LTP_SUB_GATT_ATTRIBUTE_EXECUTE_WRITE_REQ */
#define LTP_SUB_GATT_ATTRIBUTE_EXECUTE_WRITE_REQ                         0x90
#define LTP_SUB_GATT_ATTRIBUTE_EXECUTE_WRITE_REQ_LENGTH                     7
#define LTP_SUB_GATT_ATTRIBUTE_EXECUTE_WRITE_REQ_FLAGS          (LTP_MDH_MSG)

#ifdef __cplusplus
extern "C" {
#endif

/* needed message structures                                                */

/* data structure to describe message properties                            */
typedef struct
{
    uint8_t opcode;
    uint8_t length;
    uint8_t properties;
} LTPCmdInfo;
typedef LTPCmdInfo * PLTPCmdInfo;

/* internal Status definition for LTP-Lib                                   */
typedef enum
{
    LTPLibStatusIdle,             /* ready to be called with 'LTPLibxxx' call*/
    LTPLibStatusResync,           /* Lib is ready for re-sync                */
    LTPLibStatusBusy,             /* busy with 'LTPLibxxx' call (re-entrant) */
    LTPLibStatusOffSync           /* Lib detected sync loss in receive data  */
} TLTPStatus;

/* internal data container definition for LTP-Lib                           */
typedef struct
{
    uint16_t   Offset;                 /* offset to data in data buffer           */
    uint16_t   Length;                 /* length of data                          */
    uint8_t * BufferAddress;          /* buffer address                          */
} LTP_DATA_CB_T;
typedef LTP_DATA_CB_T  * LTP_DATA_CB_P;

/* internal data container definition for queue elements                    */
struct ltpQueueElement                                /* dummy definition      */
{
    struct ltpQueueElement  *Next;                      /* point to next element */
    uint8_t                    data[2];                    /* user data             */
};
typedef struct ltpQueueElement LTP_ELEMENT_T;
typedef LTP_ELEMENT_T *LTP_ELEMENT_P;

/* internal data container definition for queue elements                    */
typedef struct
{
    LTP_ELEMENT_P First;                                 /* first element         */
    LTP_ELEMENT_P Last;                                  /* last element          */
    uint16_t          ElementCount;                          /* element count         */
} LTP_QUEUE_T, *LTP_QUEUE_P;

void  ltpQueueIn(LTP_QUEUE_P QueuePtr, void *pQueueElement);
void *ltpQueueOut(LTP_QUEUE_P QueuePtr);

/* internal queue-element for LTP-Lib for data container                    */
typedef struct
{
    /* This MUST be the FIRST structure element ! */
    LTP_ELEMENT_T  QueueElement;
    /* the 'real' data...                         */
    LTP_DATA_CB_T  DataCB;
} TLTPElement;
typedef TLTPElement * PLTPElement;

#if F_LTPLIB_ASYNC_ASSEMBLY_SUPPORT
/* internal Async assemply Status definition for LTP-Lib                    */
typedef enum
{
    LTPLibAsyncStatusIgnore,           /* waiting for precede idle time      */
    LTPLibAsyncStatusWaitCMD,          /* wait for command Byte              */
    LTPLibAsyncStatusWaitCopMsk,       /* wait for CopMsk Byte               */
    LTPLibAsyncStatusWaitLen1,         /* wait for first length Byte         */
    LTPLibAsyncStatusWaitLen1HeaderCRC,/* wait for first length Byte and CRC */
    LTPLibAsyncStatusWaitLen2,         /* wait for second length Byte        */
    LTPLibAsyncStatusWaitLen2HeaderCRC,/* wait for second length Byte and CRC*/
    LTPLibAsyncStatusWaitHeaderCRC,    /* wait for header CRC byte           */
    LTPLibAsyncStatusWaitTimeout       /* wait for consecutive idle time     */
} TLTPAsyncStatus;

/* ID to identify addressed timer for timer/timeout functions               */
typedef enum
{
    LTPLibTimerID_AsyncTimeout
} TLTPTimerID;
typedef TLTPTimerID  * PLTPTimerID;
#endif /* F_LTPLIB_ASYNC_ASSEMBLY_SUPPORT */

/* internal context definition for LTP-Lib. When ever a LTP-Lib function is */
/*  called, a pointer to this context must be provided                      */
typedef struct
{
    LTP_TGT_APPHANDLE   AppHandle;
    TLTPStatus          Status;
    uint16_t                ReceiveOffset;
    uint16_t                ReceiveMaxLength;
    uint16_t                SendOffset;
    uint16_t                LTPMsgStart;
    PLTPElement         pActiveElement;
    uint8_t *              pLTPMsg;
    uint16_t                LTPMsgPos;
    uint16_t                LTPMsgLength;
#if F_LTPLIB_ASYNC_ASSEMBLY_SUPPORT
    TLTPAsyncStatus     AsyncState;
#endif
    LTP_QUEUE_T         UsedElementQueue;
    uint16_t                LTPDataCollected;
} TLTPLib;
typedef TLTPLib * PLTPLib;

/****************************************************************************/
/****************************************************************************/
/****************************************************************************/
/****************************************************************************/
/**                                                                        **/
/** 4) Target specifics that must be implemented by the user of the LTP-Lib**/
/**                                                                        **/
/****************************************************************************/
/****************************************************************************/
/****************************************************************************/
/****************************************************************************/

/****************************************************************************/
/* uint8_t * BTLTPTgtSendBufferAlloc                                           */
/* (                                                                        */
/*    LTP_TGT_APPHANDLE AppHandle : Handle to identify application context  */
/*    uint16_t              len       : size of buffer to be allocated (bytes)  */
/* )                                                                        */
/* return:------------------------------------------------------------------*/
/* pointer to allocated memory in case of success                           */
/* NULL pointer in case of an error                                         */
/*                                                                          */
/* Description:-------------------------------------------------------------*/
/* This target specific function is used to allocate buffers for LTP        */
/* messages that are send to an application with the BT_LTP_Sendxxx         */
/* functions of this library.                                               */
/*                                                                          */
/* TODO: please implement this function for your target                     */
/*                                                                          */
/****************************************************************************/
uint8_t * BTLTPTgtSendBufferAlloc(LTP_TGT_APPHANDLE AppHandle, uint16_t len);

/****************************************************************************/
/* uint8_t * BTLTPTgtAssemblyBfferAlloc                                        */
/* (                                                                        */
/*    LTP_TGT_APPHANDLE AppHandle : Handle to identify application context  */
/* )                                                                        */
/* return:------------------------------------------------------------------*/
/* pointer to allocated memory in case of success                           */
/* NULL pointer in case of no buffer available (this is no error condition) */
/*                                                                          */
/* Description:-------------------------------------------------------------*/
/* This target specific function is used to allocate buffers for LTP-       */
/* message assembly that is processed by functions of this library.         */
/*                                                                          */
/* TODO: please implement this function for your target                     */
/*                                                                          */
/****************************************************************************/
uint8_t * BTLTPTgtAssemblyBufferAlloc(LTP_TGT_APPHANDLE AppHandle);
void BTLTPBufferCallback(THandle Handle);
/****************************************************************************/
/* void BTLTPTgtReceiveBufferRelease                                        */
/* (                                                                        */
/*    LTP_TGT_APPHANDLE AppHandle : Handle to identify application context  */
/*    uint8_t *            pBuffer   : pointer to receive buffer to be released*/
/* )                                                                        */
/* return:------------------------------------------------------------------*/
/* non                                                                      */
/*                                                                          */
/* Description:-------------------------------------------------------------*/
/* This target specific function is used to released buffers for LTP        */
/* messages that are received and consumed by the 'LTPLibHandleReceiveData' */
/* function of this library.                                                */
/*                                                                          */
/* TODO: please implement this function for your target                     */
/*                                                                          */
/****************************************************************************/
void BTLTPTgtReceiveBufferRelease(LTP_TGT_APPHANDLE AppHandle, uint8_t * pBuffer);

/****************************************************************************/
/* BOOL BTLTPTgtSendLTPMessage                                              */
/* (                                                                        */
/*    LTP_TGT_APPHANDLE AppHandle : Handle to identify application context  */
/*    uint8_t *            pMsg      : pointer to of LTP msg buffer to be send */
/*    uint16_t              offset                                              */
/*    uint16_t              dataLen                                             */
/* )                                                                        */
/* return:------------------------------------------------------------------*/
/* TRUE in case the message could be send successfully,                     */
/* FALSE in case the message could not be send but was dumped               */
/*                                                                          */
/* Description:-------------------------------------------------------------*/
/* This function is used to send an LTP message to an application with the  */
/* BT_LTP_Sendxxx functions of this library                                 */
/*                                                                          */
/* TODO: please implement this function for your target                     */
/*                                                                          */
/****************************************************************************/
BOOL BTLTPTgtSendLTPMessage(LTP_TGT_APPHANDLE AppHandle, uint8_t * pMsg, uint16_t offset, uint16_t dataLen);

/****************************************************************************/
/* BOOL BTLTPTgtHandleLTPMessage                                            */
/* (                                                                        */
/*    LTP_TGT_APPHANDLE AppHandle : Handle to identify application context  */
/*    uint8_t *            pMsgBuffer: pointer to message buffer               */
/*    uint8_t              cmd       : identifier for LTP-command to be handled*/
/*    uint8_t              copmsk    : copmsk of LPT-command to be handled     */
/*    uint8_t *            pOpt      : pointer to optional parameters of LTP-  */
/*                                  command to be handled, or NULL in case  */
/*                                  of no optional parameters included      */
/*    uint16_t              lenPara   : length of mandatory parameters of LTP-  */
/*                                  command to be handled                   */
/*    uint8_t *            pPara     : pointer to mandatory parameters of LTP- */
/*                                  command to be handled, or NULL in case  */
/*                                  of no mandatory parameters included     */
/* )                                                                        */
/* return:------------------------------------------------------------------*/
/* FALSE in case the message buffer shall not be re-used for LTP-msg        */
/* assembly by the LTP-Lib, otherwise TRUE                                  */
/*                                                                          */
/* Description:-------------------------------------------------------------*/
/* This function is called by the LTP-Lib if a complete LTP message is      */
/* assembled and is ready to be consumed by the application                 */
/*                                                                          */
/* TODO: please implement this function for your target                     */
/*                                                                          */
/****************************************************************************/
BOOL BTLTPTgtHandleLTPMessage(LTP_TGT_APPHANDLE AppHandle, uint8_t * pMsgBuffer, uint8_t cmd,
                              uint8_t copmsk, uint8_t * pOpt, uint16_t lenPara, uint8_t * pPara);

/****************************************************************************/
/* PLTPElement BTLTPTgtQueueElementAlloc                                    */
/* (                                                                        */
/*    LTP_TGT_APPHANDLE AppHandle : Handle to identify application context  */
/* )                                                                        */
/* return:------------------------------------------------------------------*/
/* pointer to allocated queue element in case of success                    */
/* NULL pointer in case of no queue element available (this is no error     */
/* condition)                                                               */
/*                                                                          */
/* Description:-------------------------------------------------------------*/
/* This target specific function is used to allocate queue elements for LTP-*/
/* message assembly that is processed by functions of this library.         */
/*                                                                          */
/* TODO: please implement this function for your target                     */
/*                                                                          */
/****************************************************************************/
PLTPElement BTLTPTgtQueueElementAlloc(LTP_TGT_APPHANDLE AppHandle);

/****************************************************************************/
/* void BTLTPTgtQueueElementRelease                                         */
/* (                                                                        */
/*    LTP_TGT_APPHANDLE AppHandle : Handle to identify application context  */
/* )                                                                        */
/* return:------------------------------------------------------------------*/
/* non                                                                      */
/*                                                                          */
/* Description:-------------------------------------------------------------*/
/* This target specific function is used to release queue elements for LTP- */
/* message assembly that is processed by functions of this library.         */
/*                                                                          */
/* TODO: please implement this function for your target                     */
/*                                                                          */
/****************************************************************************/
void BTLTPTgtQueueElementRelease(LTP_TGT_APPHANDLE AppHandle, PLTPElement pLTPElement);

#if F_LTPLIB_ASYNC_ASSEMBLY_SUPPORT
/****************************************************************************/
/* void BTLTPTgtTriggerTimer                                                */
/* (                                                                        */
/*    LTP_TGT_APPHANDLE AppHandle : Handle to identify application context  */
/*    TLTPTimerID       timerID   : ID to identify addressed timer          */
/*    uint16_t              timeout_ms: new timeout for addressed timer in ms   */
/* )                                                                        */
/****************************************************************************/
void BTLTPTgtTriggerTimer(LTP_TGT_APPHANDLE AppHandle, TLTPTimerID timerID, uint16_t timeout_ms);

/****************************************************************************/
/* void BTLTPTgtHandleAsyncLTP_RESET_REQ                                    */
/* (                                                                        */
/*    LTP_TGT_APPHANDLE AppHandle : Handle to identify application context  */
/* )                                                                        */
/****************************************************************************/
void BTLTPTgtHandleAsyncLTP_RESET_REQ(LTP_TGT_APPHANDLE AppHandle);
#endif /* F_LTPLIB_ASYNC_ASSEMBLY_SUPPORT */

/****************************************************************************/
/****************************************************************************/
/****************************************************************************/
/****************************************************************************/
/**                                                                        **/
/** 5) Utility fuctions that have to be integrated for using the LTP-Lib   **/
/**                                                                        **/
/****************************************************************************/
/****************************************************************************/
/****************************************************************************/
/****************************************************************************/

/****************************************************************************/
/* BOOL LTPLibInitialize                                                    */
/* (                                                                        */
/*    PLTPLib      pLTPLib        : pointer to LTP context to be initialized*/
/*    LTP_TGT_APPHANDLE AppHandle : Handle to identify application context  */
/*    uint16_t              receiveOffset : offset for assembled LTP-messages   */
/*    uint16_t              receiveMaxLen : max len for assembled LTP-messages  */
/*    uint16_t              sendOffset    : offset for LTP-messages to be send  */
/* )                                                                        */
/* return:------------------------------------------------------------------*/
/*  FALSE if an internal error occured and the lib is not functional,       */
/*  TRUE otherwise (in case of success)                                     */
/*                                                                          */
/* Description:-------------------------------------------------------------*/
/* This function has to be called to initialize the LTP-Lib at system       */
/* startup and/or reset                                                     */
/*                                                                          */
/* TODO: please integrate this function into the target implementation      */
/*                                                                          */
/****************************************************************************/
BOOL LTPLibInitialize(PLTPLib pLTPLib, LTP_TGT_APPHANDLE AppHandle, uint16_t receiveOffset,
                      uint16_t receiveMaxLen, uint16_t sendOffset);

/****************************************************************************/
/* BOOL LTPLibShutdown                                                      */
/* (                                                                        */
/*    PLTPLib      pLTPLib        : pointer to LTP context to be initialized*/
/* )                                                                        */
/* return:------------------------------------------------------------------*/
/*  FALSE if an internal error occured and the lib is not functional,       */
/*  TRUE otherwise (in case of success)                                     */
/*                                                                          */
/* Description:-------------------------------------------------------------*/
/* This function may be called to free all resources used by LTP-Lib        */
/*                                                                          */
/* TODO: please integrate this function into the target implementation      */
/*                                                                          */
/****************************************************************************/
BOOL LTPLibShutdown(PLTPLib pLTPLib);

void LTPLibTriggerLTPProccess(PLTPLib pLTPLib);

#if F_LTPLIB_ASYNC_ASSEMBLY_SUPPORT
/****************************************************************************/
/* void LTPLibHandleTimeout                                                 */
/* (                                                                        */
/*    PLTPLib     pLTPLib  : pointer to LTP context to be used              */
/*    TLTPTimerID timerID  : timer that timed out                           */
/* )                                                                        */
/****************************************************************************/
void LTPLibHandleTimeout(PLTPLib pLTPLib, TLTPTimerID timerID);
#endif

/****************************************************************************/
/* BOOL LTPLibHandleReceiveData                                             */
/* (                                                                        */
/*    PLTPLib pLTPLib      : pointer to LTP context to be used              */
/*    uint8_t *  pRxBuffer    : pointer to buffer that contains new data       */
/*    uint16_t    rxLength     : length of new data received                    */
/*    uint16_t    rxOffset     : offset of new data in buffer                   */
/* )                                                                        */
/* return:------------------------------------------------------------------*/
/*  FALSE data could NOT be handled properly,                               */
/*  TRUE otherwise                                                          */
/*                                                                          */
/* Description:-------------------------------------------------------------*/
/* This function has to be called to introduce new rx data received into    */
/* the LTP-Lib statemachine when ever new data is received                  */
/*                                                                          */
/* TODO: please integrate this function into the target implementation      */
/*                                                                          */
/****************************************************************************/
BOOL LTPLibHandleReceiveData(PLTPLib pLTPLib, uint8_t * pRxBuffer, uint16_t rxLength, uint16_t rxOffset);

/****************************************************************************/
/****************************************************************************/
/****************************************************************************/
/****************************************************************************/
/**                                                                        **/
/** 6) Utility functions that can be used by the user of the LTP-Lib       **/
/**                                                                        **/
/****************************************************************************/
/****************************************************************************/
/****************************************************************************/
/****************************************************************************/



/****************************************************************************/
/* uint16_t LTPLibInsertHeader                                                  */
/* (                                                                        */
/*    PLTPLib pLTPLib      : pointer to LTP context to be used              */
/*    uint8_t *  pBuffer      : pointer to data buffer containing payload      */
/*    LPWORD  offset       : pointer to the offset of the payload           */
/*                           after call: offset of the message in buffer    */
/*    uint16_t    dataLen      : payload length in buffer                       */
/*    uint8_t    cmd          : ltp command                                    */
/*    uint8_t    copmsk       : bitmask defining content of optional parameter */
/*    uint8_t *  pOpt         : pointer to optional parameter structure        */
/*    LPWORD  posParam     : after call: offset of the first parameter      */
/* )                                                                        */
/* return:------------------------------------------------------------------*/
/* length of the ltp message in pBuffer beginning from *offset              */
/* if length is 0, the header could not be inserted                         */
/*                                                                          */
/* Description:-------------------------------------------------------------*/
/* This function can be used to insert a LTP message header with optional   */
/* parameters infront of a given message buffer with payload                */
/*                                                                          */
/****************************************************************************/
uint16_t LTPLibInsertHeader(PLTPLib pLTPLib, uint8_t * pBuffer, LPWORD offset, uint16_t dataLen, uint8_t cmd, uint8_t copmsk, uint8_t * pOpt, LPWORD posParam);
uint16_t LTPLibInsertExtendHeader(PLTPLib pLTPLib, uint8_t * pBuffer, LPWORD offset, uint16_t dataLen, uint8_t subCmd, uint8_t copmsk, uint8_t * pOpt, LPWORD posParam);
/****************************************************************************/
/* BOOL LTPLibSendDeviceConfigDeviceSetRsp                                   */
/* (                                                                        */
/*    PLTPLib pLTPLib      : pointer to LTP context to be used              */
/*    uint8_t    copmsk       : bitmask defining content of optional parameter */
/*    uint8_t *  pOpt         : pointer to optional parameter structure        */
/*    uint8_t    cause        : result cause for this message                  */
/* )                                                                        */
/* return:------------------------------------------------------------------*/
/* TRUE in case the message was send send successfully                      */
/* FALSE otherwise                                                          */
/*                                                                          */
/* Description:-------------------------------------------------------------*/
/* This function can be used to send the LTP message in the function name   */
/*                                                                          */
/* Porting to BlueAPI+:-----------------------------------------------------*/
/* The corresponding function in the Stollmann system internal software API */
/* BlueAPI+ for using LTP functionality without serial link interface is    */
/* blueAPI_SendDeviceConfigSetReq(...)                                      */
/*                                                                          */
/****************************************************************************/
BOOL LTPLibSendDeviceConfigDeviceNameSetRsp(PLTPLib pLTPLib, uint8_t copmsk, uint8_t * pOpt, uint8_t cause);
BOOL LTPLibSendDeviceConfigAppearanceSetRsp(PLTPLib pLTPLib, uint8_t copmsk, uint8_t * pOpt, uint8_t cause);
BOOL LTPLibSendDeviceConfigStoreSetRsp(PLTPLib pLTPLib, uint8_t copmsk, uint8_t * pOpt, uint8_t cause);
BOOL LTPLibSendDeviceConfigPerPrefConnParamSetRsp(PLTPLib pLTPLib, uint8_t copmsk, uint8_t * pOpt, uint8_t cause);

/****************************************************************************/
/* BOOL LTPLibSendDeviceConfigSecuritySetRsp                                */
/* (                                                                        */
/*    PLTPLib pLTPLib      : pointer to LTP context to be used              */
/*    uint8_t    copmsk       : bitmask defining content of optional parameter */
/*    uint8_t *  pOpt         : pointer to optional parameter structure        */
/*    uint8_t    cause        : result cause for this message                  */
/* )                                                                        */
/* return:------------------------------------------------------------------*/
/* TRUE in case the message was send send successfully                      */
/* FALSE otherwise                                                          */
/*                                                                          */
/* Description:-------------------------------------------------------------*/
/* This function can be used to send the LTP message in the function name   */
/*                                                                          */
/* Porting to BlueAPI+:-----------------------------------------------------*/
/* The corresponding function in the Stollmann system internal software API */
/* BlueAPI+ for using LTP functionality without serial link interface is    */
/* blueAPI_SendDeviceConfigSetReq(...)                                      */
/*                                                                          */
/****************************************************************************/
BOOL LTPLibSendDeviceConfigSecuritySetRsp(PLTPLib pLTPLib, uint8_t copmsk, uint8_t * pOpt, uint8_t cause);


/****************************************************************************/
/* BOOL LTPLibSendPairableModeSetRsp                                        */
/* (                                                                        */
/*    PLTPLib pLTPLib      : pointer to LTP context to be used              */
/*    uint8_t    copmsk       : bitmask defining content of optional parameter */
/*    uint8_t *  pOpt         : pointer to optional parameter structure        */
/*    uint8_t    cause        : result code for this message                   */
/* )                                                                        */
/* return:------------------------------------------------------------------*/
/* TRUE in case the message was send send successfully                      */
/* FALSE otherwise                                                          */
/*                                                                          */
/* Description:-------------------------------------------------------------*/
/* This function can be used to send the LTP message in the function name   */
/*                                                                          */
/****************************************************************************/
BOOL LTPLibSendPairableModeSetRsp(PLTPLib pLTPLib, uint8_t copmsk, uint8_t * pOpt, uint8_t cause);

/****************************************************************************/
/* BOOL LTPLibSendPasskeyReqReplyRsp                                        */
/* (                                                                        */
/*    PLTPLib pLTPLib      : pointer to LTP context to be used              */
/*    uint8_t    copmsk       : bitmask defining content of optional parameter */
/*    uint8_t *  pOpt         : pointer to optional parameter structure        */
/*    uint8_t    cause        : result code for this message                   */
/* )                                                                        */
/* return:------------------------------------------------------------------*/
/* TRUE in case the message was send send successfully                      */
/* FALSE otherwise                                                          */
/*                                                                          */
/* Description:-------------------------------------------------------------*/
/* This function can be used to send the LTP message in the function name   */
/*                                                                          */
/****************************************************************************/
BOOL LTPLibSendPasskeyReqReplyRsp(PLTPLib pLTPLib, uint8_t copmsk, uint8_t * pOpt, uint8_t cause);

/****************************************************************************/
/* BOOL LTPLibSendActInfo                                                   */
/* (                                                                        */
/*    PLTPLib pLTPLib      : pointer to LTP context to be used              */
/*    uint8_t    copmsk       : bitmask defining content of optional parameter */
/*    uint8_t *  pOpt         : pointer to optional parameter structure        */
/*    uint8_t    cause        : result code for this message                   */
/*    uint8_t    version      : version of supported LTP protocol              */
/*    uint8_t *  local_BD     : Bluetooth device address of local device       */
/*    uint8_t *  FW_VersionString : human readable version string for FW vers. */
/* )                                                                        */
/* return:------------------------------------------------------------------*/
/* TRUE in case the message was send successfully                           */
/* FALSE otherwise                                                          */
/*                                                                          */
/* Description:-------------------------------------------------------------*/
/* This function can be used to send the LTP message in the function name   */
/*                                                                          */
/****************************************************************************/
BOOL LTPLibSendActInfo(PLTPLib pLTPLib, uint8_t copmsk, uint8_t * pOpt, uint8_t cause, uint8_t version, uint8_t * local_BD, uint8_t * FW_VersionString);

/****************************************************************************/
/* BOOL LTPLibSendPasskeyNotificationInfo                                   */
/* (                                                                        */
/*    PLTPLib pLTPLib      : pointer to LTP context to be used              */
/*    uint8_t    copmsk       : bitmask defining content of optional parameter */
/*    uint8_t *  pOpt         : pointer to optional parameter structure        */
/*    uint8_t *  rem_BD       : Bluetooth device address of remote device      */
/*    uint32_t   displayValue : value to be displayed                          */
/* )                                                                        */
/* return:------------------------------------------------------------------*/
/* TRUE in case the message was send successfully                           */
/* FALSE otherwise                                                          */
/*                                                                          */
/* Description:-------------------------------------------------------------*/
/* This function can be used to send the LTP message in the function name   */
/*                                                                          */
/****************************************************************************/
BOOL LTPLibSendPasskeyNotificationInfo(PLTPLib pLTPLib, uint8_t copmsk, uint8_t * pOpt, uint8_t * rem_BD, uint32_t displayValue);

/****************************************************************************/
/* BOOL LTPLibSendMCLStatusInfo                                             */
/* (                                                                        */
/*    PLTPLib pLTPLib      : pointer to LTP context to be used              */
/*    uint8_t    copmsk       : bitmask defining content of optional parameter */
/*    uint8_t *  pOpt         : pointer to optional parameter structure        */
/*    uint8_t *  rem_BD       : Bluetooth device address of remote device      */
/*    uint8_t    loc_MCL_ID   : Identifier to referenced MCL                   */
/*    uint8_t    loc_MCL_Status : status of referenced MCL                     */
/* )                                                                        */
/* return:------------------------------------------------------------------*/
/* TRUE in case the message was send successfully                           */
/* FALSE otherwise                                                          */
/*                                                                          */
/* Description:-------------------------------------------------------------*/
/* This function can be used to send the LTP message in the function name   */
/*                                                                          */
/****************************************************************************/
BOOL LTPLibSendMCLStatusInfo(PLTPLib pLTPLib, uint8_t copmsk, uint8_t * pOpt, uint8_t * rem_BD, uint8_t loc_MCL_ID, uint8_t loc_MCL_Status);

/****************************************************************************/
/* BOOL LTPLibSendACLStatusInfo                                             */
/* (                                                                        */
/*    PLTPLib pLTPLib      : pointer to LTP context to be used              */
/*    uint8_t    copmsk       : bitmask defining content of optional parameter */
/*    uint8_t *  pOpt         : pointer to optional parameter structure        */
/*    uint8_t *  rem_BD       : Bluetooth device address of remote device      */
/*    uint8_t    loc_MCL_Status : status of referenced MCL                     */
/* )                                                                        */
/* return:------------------------------------------------------------------*/
/* TRUE in case the message was send successfully                           */
/* FALSE otherwise                                                          */
/*                                                                          */
/* Description:-------------------------------------------------------------*/
/* This function can be used to send the LTP message in the function name   */
/*                                                                          */
/****************************************************************************/
BOOL LTPLibSendACLStatusInfo(PLTPLib pLTPLib, uint8_t copmsk, uint8_t * pOpt, uint8_t * rem_BD,
                             uint8_t loc_MCL_Status);


/****************************************************************************/
/* BOOL LTPLibSendPasskeyRequestInd                                         */
/* (                                                                        */
/*    PLTPLib pLTPLib      : pointer to LTP-Context to be used              */
/*    uint8_t    copmsk       : bitmask defining content of optional parameter */
/*    uint8_t *  pOpt         : pointer to potional parameter structure        */
/*    uint8_t *  rem_BD       : Bluetooth device address of remote device      */
/* )                                                                        */
/* return:------------------------------------------------------------------*/
/* TRUE in case the was send send successfully                              */
/* FALSE otherwhise                                                         */
/*                                                                          */
/* Description:-------------------------------------------------------------*/
/* This function can be used to send the LTP message in the function name   */
/*                                                                          */
/****************************************************************************/
BOOL LTPLibSendPasskeyRequestInd(PLTPLib pLTPLib, uint8_t copmsk, uint8_t * pOpt, uint8_t * rem_BD);

/****************************************************************************/
/* BOOL LTPLibSendRemoteOOBRequestInd                                       */
/* (                                                                        */
/*    PLTPLib pLTPLib      : pointer to LTP-Context to be used              */
/*    uint8_t    copmsk       : bitmask defining content of optional parameter */
/*    uint8_t *  pOpt         : pointer to potional parameter structure        */
/*    uint8_t *  rem_BD       : Bluetooth device address of remote device      */
/* )                                                                        */
/* return:------------------------------------------------------------------*/
/* TRUE in case the was send send successfully                              */
/* FALSE otherwhise                                                         */
/*                                                                          */
/* Description:-------------------------------------------------------------*/
/* This function can be used to send the LTP message in the function name   */
/*                                                                          */
/****************************************************************************/
BOOL LTPLibSendRemoteOOBRequestInd(PLTPLib pLTPLib, uint8_t copmsk, uint8_t * pOpt, uint8_t * rem_BD);


/****************************************************************************/
/* BOOL LTPLibSendResultInd                                                 */
/* (                                                                        */
/*    PLTPLib pLTPLib      : pointer to LTP-Context to be used              */
/*    uint8_t    copmsk       : bitmask defining content of optional parameter */
/*    uint8_t *  pOpt         : pointer to potional parameter structure        */
/*    uint8_t    cause        : result of authentication                       */
/*    uint8_t *  rem_BD       : Bluetooth device address of remote device      */
/*    uint8_t    rem_BD_Type  : Type of Bluetooth address                      */
/*    uint8_t    keyType      : type of the generated linkkey                  */
/*    uint32_t   appData      : Application specific data                      */
/*    uint8_t *  linkKey      : Bluetooth Linkkey                              */
/*    uint16_t    linkKeyLength: Length of Linkkey data                         */
/* )                                                                        */
/* return:------------------------------------------------------------------*/
/* TRUE in case the was send send successfully                              */
/* FALSE otherwhise                                                         */
/*                                                                          */
/* Description:-------------------------------------------------------------*/
/* This function can be used to send the LTP message in the function name   */
/*                                                                          */
/****************************************************************************/
BOOL LTPLibSendAuthResultExtInd(PLTPLib pLTPLib, uint8_t copmsk, uint8_t * pOpt, uint8_t cause,
                                uint8_t * rem_BD, uint8_t rem_BD_Type, uint8_t keyType,
                                uint8_t * linkKey, uint16_t linkKeyLength);

/****************************************************************************/
/* BOOL LTPLibSendAuthResultRequestExtInd                                   */
/* (                                                                        */
/*    PLTPLib pLTPLib      : pointer to LTP-Context to be used              */
/*    uint8_t    copmsk       : bitmask defining content of optional parameter */
/*    uint8_t *  pOpt         : pointer to potional parameter structure        */
/*    uint8_t *  rem_BD       : Bluetooth device address of remote device      */
/*    uint8_t    rem_BD_Type  : Type of Bluetooth address                      */
/*    uint8_t    keyType      : type of the generated linkkey                  */
/* )                                                                        */
/* return:------------------------------------------------------------------*/
/* TRUE in case the was send send successfully                              */
/* FALSE otherwhise                                                         */
/*                                                                          */
/* Description:-------------------------------------------------------------*/
/* This function can be used to send the LTP message in the function name   */
/*                                                                          */
/****************************************************************************/
BOOL LTPLibSendAuthResultRequestExtInd(PLTPLib pLTPLib, uint8_t copmsk, uint8_t * pOpt, uint8_t * rem_BD, uint8_t rem_BD_Type, uint8_t keyType);


/****************************************************************************/
/* BOOL LTPLibSendInternalEventInfo                                         */
/* (                                                                        */
/*    PLTPLib pLTPLib      : pointer to LTP-Context to be used              */
/*    uint8_t    copmsk       : bitmask defining content of optional parameter */
/*    uint8_t *  pOpt         : pointer to potional parameter structure        */
/*    uint8_t    cause        : result code for this message                   */
/*    uint8_t    eventType    : indicates the result of an transaction         */
/*    uint32_t   eventInfo    : additional information for this event          */
/* )                                                                        */
/* return:------------------------------------------------------------------*/
/* TRUE in case the was send send successfully                              */
/* FALSE otherwhise                                                         */
/*                                                                          */
/* Description:-------------------------------------------------------------*/
/* This function can be used to send the LTP message in the function name   */
/*                                                                          */
/****************************************************************************/
BOOL LTPLibSendInternalEventInfo(PLTPLib pLTPLib, uint8_t copmsk, uint8_t * pOpt,
                                 uint8_t cause, uint8_t eventType, uint32_t eventInfo);

/****************************************************************************/
/* BOOL LTPLibSendCreateMDLInd                                              */
/* (                                                                        */
/*    PLTPLib pLTPLib      : pointer to LTP context to be used              */
/*    uint8_t    copmsk       : bitmask defining content of optional parameter */
/*    uint8_t *  pOpt         : pointer to optional parameter structure        */
/*    uint8_t *  rem_BD       : Bluetooth device address of remote device      */
/*    uint8_t    loc_MDL_ID   : reference to local MDL identifier              */
/* )                                                                        */
/* return:------------------------------------------------------------------*/
/* TRUE in case the message was send send successfully                      */
/* FALSE otherwise                                                          */
/*                                                                          */
/* Description:-------------------------------------------------------------*/
/* This function can be used to send the LTP message in the function name   */
/*                                                                          */
/****************************************************************************/
BOOL LTPLibSendCreateMDLInd(PLTPLib pLTPLib, uint8_t copmsk, uint8_t * pOpt, uint8_t * rem_BD, uint8_t loc_MDL_ID);


/****************************************************************************/
/* BOOL LTPLibSendConnectMDLInfo                                            */
/* (                                                                        */
/*    PLTPLib pLTPLib      : pointer to LTP context to be used              */
/*    uint8_t    copmsk       : bitmask defining content of optional parameter */
/*    uint8_t *  pOpt         : pointer to optional parameter structure        */
/*    uint8_t    loc_MDL_ID   : reference to local MDL identifier              */
/*    uint16_t    maxLTPSize   : max LTP message size for this MDL              */
/*    uint16_t    maxAPDUSize  : max APDU size for this MDL                     */
/* )                                                                        */
/* return:------------------------------------------------------------------*/
/* TRUE in case the message was send send successfully                      */
/* FALSE otherwise                                                          */
/*                                                                          */
/* Description:-------------------------------------------------------------*/
/* This function can be used to send the LTP message in the function name   */
/*                                                                          */
/****************************************************************************/
BOOL LTPLibSendConnectMDLInfo(PLTPLib pLTPLib, uint8_t copmsk, uint8_t * pOpt, uint8_t loc_MDL_ID,
                              uint16_t maxLTPSize, uint16_t maxAPDUSize);

/****************************************************************************/
/* BOOL LTPLibSendDisconnectMDLInd                                          */
/* (                                                                        */
/*    PLTPLib pLTPLib      : pointer to LTP context to be used              */
/*    uint8_t    copmsk       : bitmask defining content of optional parameter */
/*    uint8_t *  pOpt         : pointer to optional parameter structure        */
/*    uint8_t    cause        : result code for this message                   */
/*    uint8_t    loc_MDL_ID   : reference to local MDL identifier              */
/* )                                                                        */
/* return:------------------------------------------------------------------*/
/* TRUE in case the message was send send successfully                      */
/* FALSE otherwise                                                          */
/*                                                                          */
/* Description:-------------------------------------------------------------*/
/* This function can be used to send the LTP message in the function name   */
/*                                                                          */
/****************************************************************************/
BOOL LTPLibSendDisconnectMDLInd(PLTPLib pLTPLib, uint8_t copmsk, uint8_t * pOpt, uint8_t cause, uint8_t loc_MDL_ID);

/****************************************************************************/
/* BOOL LTPLibSendDeleteMDLInfo                                             */
/* (                                                                        */
/*    PLTPLib pLTPLib      : pointer to LTP context to be used              */
/*    uint8_t    copmsk       : bitmask defining content of optional parameter */
/*    uint8_t *  pOpt         : pointer to optional parameter structure        */
/*    uint8_t    loc_MDL_ID   : reference to local MDL identifier              */
/* )                                                                        */
/* return:------------------------------------------------------------------*/
/* TRUE in case the message was send send successfully                      */
/* FALSE otherwise                                                          */
/*                                                                          */
/* Description:-------------------------------------------------------------*/
/* This function can be used to send the LTP message in the function name   */
/*                                                                          */
/****************************************************************************/
BOOL LTPLibSendDeleteMDLInfo(PLTPLib pLTPLib, uint8_t copmsk, uint8_t * pOpt, uint8_t loc_MDL_ID);

/****************************************************************************/
/* BOOL LTPLibSendDisconnectMDLRsp                                          */
/* (                                                                        */
/*    PLTPLib pLTPLib      : pointer to LTP context to be used              */
/*    uint8_t    copmsk       : bitmask defining content of optional parameter */
/*    uint8_t *  pOpt         : pointer to optional parameter structure        */
/*    uint8_t    cause        : result code for this message                   */
/*    uint8_t    loc_MDL_ID   : reference to local MDL identifier              */
/* )                                                                        */
/* return:------------------------------------------------------------------*/
/* TRUE in case the message was send send successfully                      */
/* FALSE otherwise                                                          */
/*                                                                          */
/* Description:-------------------------------------------------------------*/
/* This function can be used to send the LTP message in the function name   */
/*                                                                          */
/****************************************************************************/
BOOL LTPLibSendDisconnectMDLRsp(PLTPLib pLTPLib, uint8_t copmsk, uint8_t * pOpt, uint8_t cause, uint8_t loc_MDL_ID);


/****************************************************************************/
/* BOOL LTPLibSendResetRsp                                                  */
/* (                                                                        */
/*    PLTPLib pLTPLib      : pointer to LTP context to be used              */
/*    uint8_t    copmsk       : bitmask defining content of optional parameter */
/*    uint8_t *  pOpt         : pointer to optional parameter structure        */
/*    uint8_t    cause        : result code for this message                   */
/* )                                                                        */
/* return:------------------------------------------------------------------*/
/* TRUE in case the message was send send successfully                      */
/* FALSE otherwise                                                          */
/*                                                                          */
/* Description:-------------------------------------------------------------*/
/* This function can be used to send the LTP message in the function name   */
/*                                                                          */
/****************************************************************************/
BOOL LTPLibSendResetRsp(PLTPLib pLTPLib, uint8_t copmsk, uint8_t * pOpt, uint8_t cause);


/****************************************************************************/
/* BOOL LTPLibSendGATTServiceRegisterRsp                                    */
/* (                                                                        */
/*    PLTPLib pLTPLib      : pointer to LTP context to be used              */
/*    uint8_t    copmsk       : bitmask defining content of optional parameter */
/*    uint8_t *  pOpt         : pointer to optional parameter structure        */
/*    uint8_t    cause        : result code for this message                   */
/*    uint16_t    subcause     : detailed result information from lower layers  */
/*    uint8_t    serviceHandle: handle used for service related transactions   */
/* )                                                                        */
/* return:------------------------------------------------------------------*/
/* TRUE in case the message was send send successfully                      */
/* FALSE otherwise                                                          */
/*                                                                          */
/* Description:-------------------------------------------------------------*/
/* This function can be used to send the LTP message in the function name   */
/*                                                                          */
/****************************************************************************/
BOOL LTPLibSendGATTServiceRegisterRsp(PLTPLib pLTPLib, uint8_t copmsk, uint8_t * pOpt, uint8_t cause, uint16_t subCause, uint8_t serviceHandle);



/****************************************************************************/
/* BOOL LTPLibSendGATTAttributeUpdateStatusInd                              */
/* (                                                                        */
/*    PLTPLib pLTPLib      : pointer to LTP context to be used              */
/*    uint8_t    copmsk       : bitmask defining content of optional parameter */
/*    uint8_t *  pOpt         : pointer to optional parameter structure        */
/*    uint8_t    cause        : result code for this message                   */
/*    uint16_t    subcause     : detailed result information from lower layers  */
/*    uint8_t    serviceHandle: handle of service that contains the attribute  */
/*    uint8_t    requestHandle: request handle from attribute update request   */
/*    uint16_t    attribIndex  : index of attribute in service descriptor array */
/*    uint8_t *  rem_BD       : Bluetooth device address of remote device      */
/*    uint8_t    rem_BD_Type  : Type of Bluetooth address                      */
/* )                                                                        */
/* return:------------------------------------------------------------------*/
/* TRUE in case the message was send send successfully                      */
/* FALSE otherwise                                                          */
/*                                                                          */
/* Description:-------------------------------------------------------------*/
/* This function can be used to send the LTP message in the function name   */
/*                                                                          */
/****************************************************************************/
BOOL LTPLibSendGATTAttributeUpdateStatusInd(PLTPLib pLTPLib, uint8_t copmsk, uint8_t * pOpt, uint8_t cause, uint16_t subCause, uint8_t serviceHandle, void * requestHandle, uint16_t attribIndex, uint8_t * rem_BD, uint8_t rem_BD_Type);

/****************************************************************************/
/* BOOL LTPLibSendGATTAttributeReadInd                                      */
/* (                                                                        */
/*    PLTPLib pLTPLib      : pointer to LTP context to be used              */
/*    uint8_t    copmsk       : bitmask defining content of optional parameter */
/*    uint8_t *  pOpt         : pointer to optional parameter structure        */
/*    uint8_t    loc_MDL_ID   : reference to local MDL identifier              */
/*    uint8_t    serviceHandle: handle of service that contains the attribute  */
/*    uint16_t    attribIndex  : index of attribute in service descriptor array */
/*    uint16_t    readOffset   : offset in attribute                            */
/* )                                                                        */
/* return:------------------------------------------------------------------*/
/* TRUE in case the message was send send successfully                      */
/* FALSE otherwise                                                          */
/*                                                                          */
/* Description:-------------------------------------------------------------*/
/* This function can be used to send the LTP message in the function name   */
/*                                                                          */
/****************************************************************************/
BOOL LTPLibSendGATTAttributeReadInd(PLTPLib pLTPLib, uint8_t copmsk, uint8_t * pOpt, uint8_t loc_MDL_ID, uint8_t serviceHandle, uint16_t attribIndex, uint16_t readOffset);

/****************************************************************************/
/* BOOL LTPLibSendGATTServerStoreInd                                        */
/* (                                                                        */
/*    PLTPLib pLTPLib      : pointer to LTP context to be used              */
/*    uint8_t    copmsk       : bitmask defining content of optional parameter */
/*    uint8_t *  pOpt         : pointer to optional parameter structure        */
/*    uint8_t    opCode       : requested operation                            */
/*    uint8_t *  rem_BD       : Bluetooth device address of remote device      */
/*    uint8_t    rem_BD_Type  : Type of Bluetooth address                      */
/*    uint16_t    restartHandle: used for multiple data message                 */
/*    uint8_t *  data         : up to 32byte of data                           */
/*    uint16_t    dataLength   : length of data parameter                       */
/* )                                                                        */
/* return:------------------------------------------------------------------*/
/* TRUE in case the message was send send successfully                      */
/* FALSE otherwise                                                          */
/*                                                                          */
/* Description:-------------------------------------------------------------*/
/* This function can be used to send the LTP message in the function name   */
/*                                                                          */
/****************************************************************************/
BOOL LTPLibSendGATTServerStoreInd(PLTPLib pLTPLib, uint8_t copmsk, uint8_t * pOpt, uint8_t opCode, uint8_t * rem_BD, uint8_t rem_BD_Type, uint16_t restartHandle, uint8_t * data, uint16_t dataLength);

/****************************************************************************/
/* BOOL LTPLibSendConnectGATTMDLRsp                                         */
/* (                                                                        */
/*    PLTPLib pLTPLib      : pointer to LTP context to be used              */
/*    uint8_t    copmsk       : bitmask defining content of optional parameter */
/*    uint8_t *  pOpt         : pointer to optional parameter structure        */
/*    uint8_t    cause        : result code for this message                   */
/*    uint8_t *  rem_BD       : Bluetooth device address of remote device      */
/*    uint8_t    rem_BD_Type  : Type of Bluetooth address                      */
/*    uint8_t    loc_MDL_ID   : reference to local MDL identifier              */
/*    uint8_t    loc_MDEP_ID  : local MDEP ID from connect request             */
/* )                                                                        */
/* return:------------------------------------------------------------------*/
/* TRUE in case the message was send send successfully                      */
/* FALSE otherwise                                                          */
/*                                                                          */
/* Description:-------------------------------------------------------------*/
/* This function can be used to send the LTP message in the function name   */
/*                                                                          */
/****************************************************************************/
BOOL LTPLibSendConnectGATTMDLRsp(PLTPLib pLTPLib, uint8_t copmsk, uint8_t * pOpt, uint8_t cause, uint8_t * rem_BD, uint8_t rem_BD_Type, uint8_t loc_MDL_ID, uint8_t loc_MDEP_ID);

/****************************************************************************/
/* BOOL LTPLibSendGATTDiscoveryRsp                                          */
/* (                                                                        */
/*    PLTPLib pLTPLib      : pointer to LTP context to be used              */
/*    uint8_t    copmsk       : bitmask defining content of optional parameter */
/*    uint8_t *  pOpt         : pointer to optional parameter structure        */
/*    uint8_t    cause        : result code for this message                   */
/*    uint16_t    subcause     : detailed result information from lower layers  */
/*    uint8_t    loc_MDL_ID   : reference to local MDL identifier              */
/*    uint8_t    discoveryType: type of discovery                              */
/* )                                                                        */
/* return:------------------------------------------------------------------*/
/* TRUE in case the message was send send successfully                      */
/* FALSE otherwise                                                          */
/*                                                                          */
/* Description:-------------------------------------------------------------*/
/* This function can be used to send the LTP message in the function name   */
/*                                                                          */
/****************************************************************************/
BOOL LTPLibSendGATTDiscoveryRsp(PLTPLib pLTPLib, uint8_t copmsk, uint8_t * pOpt, uint8_t cause, uint16_t subCause, uint8_t loc_MDL_ID, uint8_t discoveryType);

/****************************************************************************/
/* BOOL LTPLibSendGATTAttributeWriteRsp                                     */
/* (                                                                        */
/*    PLTPLib pLTPLib      : pointer to LTP context to be used              */
/*    uint8_t    copmsk       : bitmask defining content of optional parameter */
/*    uint8_t *  pOpt         : pointer to optional parameter structure        */
/*    uint8_t    cause        : result code for this message                   */
/*    uint16_t    subcause     : detailed result information from lower layers  */
/*    uint8_t    loc_MDL_ID   : reference to local MDL identifier              */
/*    uint8_t    writeType    : type of write                                  */
/* )                                                                        */
/* return:------------------------------------------------------------------*/
/* TRUE in case the message was send send successfully                      */
/* FALSE otherwise                                                          */
/*                                                                          */
/* Description:-------------------------------------------------------------*/
/* This function can be used to send the LTP message in the function name   */
/*                                                                          */
/****************************************************************************/
BOOL LTPLibSendGATTAttributeWriteRsp(PLTPLib pLTPLib, uint8_t copmsk, uint8_t * pOpt, uint8_t cause, uint16_t subCause, uint8_t loc_MDL_ID, uint8_t writeType);

/****************************************************************************/
/* BOOL LTPLibSendGATTAttributeNotificationInfo                             */
/* (                                                                        */
/*    PLTPLib pLTPLib      : pointer to LTP context to be used              */
/*    uint8_t    copmsk       : bitmask defining content of optional parameter */
/*    uint8_t *  pOpt         : pointer to optional parameter structure        */
/*    uint8_t    loc_MDL_ID   : reference to local MDL identifier              */
/*    uint16_t    attribHandle : handle of attribute                            */
/*    uint8_t *  data         : attribute data                                 */
/*    uint16_t    dataLength   : length of attribute data                       */
/* )                                                                        */
/* return:------------------------------------------------------------------*/
/* TRUE in case the message was send send successfully                      */
/* FALSE otherwise                                                          */
/*                                                                          */
/* Description:-------------------------------------------------------------*/
/* This function can be used to send the LTP message in the function name   */
/*                                                                          */
/****************************************************************************/
BOOL LTPLibSendGATTAttributeNotificationInfo(PLTPLib pLTPLib, uint8_t copmsk, uint8_t * pOpt, uint8_t loc_MDL_ID, uint16_t attribHandle, uint8_t * data, uint16_t dataLength);

/****************************************************************************/
/* BOOL LTPLibSendGATTSecurityRsp                                           */
/* (                                                                        */
/*    PLTPLib pLTPLib      : pointer to LTP context to be used              */
/*    uint8_t    copmsk       : bitmask defining content of optional parameter */
/*    uint8_t *  pOpt         : pointer to optional parameter structure        */
/*    uint8_t    cause        : result code for this message                   */
/*    uint8_t    loc_MDL_ID   : reference to local MDL identifier              */
/*    uint8_t    keyType      : generated linkkey type                         */
/*    uint8_t    keySize      : size of generated linkkey                      */
/* )                                                                        */
/* return:------------------------------------------------------------------*/
/* TRUE in case the message was send send successfully                      */
/* FALSE otherwise                                                          */
/*                                                                          */
/* Description:-------------------------------------------------------------*/
/* This function can be used to send the LTP message in the function name   */
/*                                                                          */
/****************************************************************************/
BOOL LTPLibSendGATTSecurityRsp(PLTPLib pLTPLib, uint8_t copmsk, uint8_t * pOpt, uint8_t cause, uint8_t loc_MDL_ID, uint8_t keyType, uint8_t keySize);


/****************************************************************************/
/* BOOL LTPLibSendGATTMtuSizeInfo                                           */
/* (                                                                        */
/*    PLTPLib pLTPLib      : pointer to LTP context to be used              */
/*    uint8_t    copmsk       : bitmask defining content of optional parameter */
/*    uint8_t *  pOpt         : pointer to optional parameter structure        */
/*    uint8_t    loc_MDL_ID   : reference to local MDL identifier              */
/*    uint16_t    mtuSize      : MTU size used for this link                    */
/* )                                                                        */
/* return:------------------------------------------------------------------*/
/* TRUE in case the message was send send successfully                      */
/* FALSE otherwise                                                          */
/*                                                                          */
/* Description:-------------------------------------------------------------*/
/* This function can be used to send the LTP message in the function name   */
/*                                                                          */
/****************************************************************************/
BOOL LTPLibSendGATTMtuSizeInfo(PLTPLib pLTPLib, uint8_t copmsk, uint8_t * pOpt, uint8_t loc_MDL_ID, uint16_t mtuSize);

/****************************************************************************/
/* BOOL LTPLibSendLEAdvertiseRsp                                            */
/* (                                                                        */
/*    PLTPLib pLTPLib      : pointer to LTP context to be used              */
/*    uint8_t    copmsk       : bitmask defining content of optional parameter */
/*    uint8_t *  pOpt         : pointer to optional parameter structure        */
/*    uint8_t    cause        : result code for this message                   */
/*    uint8_t    advMode      : advertising mode from Advertise request        */
/* )                                                                        */
/* return:------------------------------------------------------------------*/
/* TRUE in case the message was send send successfully                      */
/* FALSE otherwise                                                          */
/*                                                                          */
/* Description:-------------------------------------------------------------*/
/* This function can be used to send the LTP message in the function name   */
/*                                                                          */
/****************************************************************************/
BOOL LTPLibSendLEAdvertiseRsp(PLTPLib pLTPLib, uint8_t copmsk, uint8_t * pOpt, uint8_t cause, uint8_t advMode);
BOOL LTPLibSendSetRandomAddressRsp(PLTPLib pLTPLib, uint8_t copmsk, uint8_t * pOpt, uint8_t cause, uint16_t subCause);


/****************************************************************************/
/* BOOL LTPLibSendLEAdvertiseParameterSetRsp                                */
/* (                                                                        */
/*    PLTPLib pLTPLib      : pointer to LTP context to be used              */
/*    uint8_t    copmsk       : bitmask defining content of optional parameter */
/*    uint8_t *  pOpt         : pointer to optional parameter structure        */
/*    uint8_t    cause        : result code for this message                   */
/* )                                                                        */
/* return:------------------------------------------------------------------*/
/* TRUE in case the message was send send successfully                      */
/* FALSE otherwise                                                          */
/*                                                                          */
/* Description:-------------------------------------------------------------*/
/* This function can be used to send the LTP message in the function name   */
/*                                                                          */
/****************************************************************************/
BOOL LTPLibSendLEAdvertiseParameterSetRsp(PLTPLib pLTPLib, uint8_t copmsk, uint8_t * pOpt, uint8_t cause);

/****************************************************************************/
/* BOOL LTPLibSendLEAdvertiseDataSetRsp                                     */
/* (                                                                        */
/*    PLTPLib pLTPLib      : pointer to LTP context to be used              */
/*    uint8_t    copmsk       : bitmask defining content of optional parameter */
/*    uint8_t *  pOpt         : pointer to optional parameter structure        */
/*    uint8_t    cause        : result code for this message                   */
/*    uint8_t    dataType     : data type from Advertise Data Set Request      */
/* )                                                                        */
/* return:------------------------------------------------------------------*/
/* TRUE in case the message was send send successfully                      */
/* FALSE otherwise                                                          */
/*                                                                          */
/* Description:-------------------------------------------------------------*/
/* This function can be used to send the LTP message in the function name   */
/*                                                                          */
/****************************************************************************/
BOOL LTPLibSendLEAdvertiseDataSetRsp(PLTPLib pLTPLib, uint8_t copmsk, uint8_t * pOpt, uint8_t cause, uint8_t dataType);

/****************************************************************************/
/* BOOL LTPLibSendLEScanRsp                                                 */
/* (                                                                        */
/*    PLTPLib pLTPLib      : pointer to LTP context to be used              */
/*    uint8_t    copmsk       : bitmask defining content of optional parameter */
/*    uint8_t *  pOpt         : pointer to optional parameter structure        */
/*    uint8_t    cause        : result code for this message                   */
/* )                                                                        */
/* return:------------------------------------------------------------------*/
/* TRUE in case the message was send send successfully                      */
/* FALSE otherwise                                                          */
/*                                                                          */
/* Description:-------------------------------------------------------------*/
/* This function can be used to send the LTP message in the function name   */
/*                                                                          */
/****************************************************************************/
BOOL LTPLibSendLEScanRsp(PLTPLib pLTPLib, uint8_t copmsk, uint8_t * pOpt, uint8_t cause);

/****************************************************************************/
/* BOOL LTPLibSendLEScanInfo                                                */
/* (                                                                        */
/*    PLTPLib pLTPLib      : pointer to LTP context to be used              */
/*    uint8_t    copmsk       : bitmask defining content of optional parameter */
/*    uint8_t *  pOpt         : pointer to optional parameter structure        */
/*    uint8_t *  rem_BD       : Bluetooth device address of remote device      */
/*    uint8_t    rem_BD_Type  : Type of Bluetooth address                      */
/*    uint8_t    advType      : Type of advertising event                      */
/*    uint8_t    rssi         : remote signal strength indication              */
/*    uint8_t    data         : advertising data / scan response data          */
/*    uint16_t    dataLength   : length of data part                            */
/* )                                                                        */
/* return:------------------------------------------------------------------*/
/* TRUE in case the message was send send successfully                      */
/* FALSE otherwise                                                          */
/*                                                                          */
/* Description:-------------------------------------------------------------*/
/* This function can be used to send the LTP message in the function name   */
/*                                                                          */
/****************************************************************************/
BOOL LTPLibSendLEScanInfo(PLTPLib pLTPLib, uint8_t copmsk, uint8_t * pOpt, uint8_t * rem_BD, uint8_t rem_BD_Type, uint8_t advType, uint8_t rssi, uint8_t * data, uint16_t dataLength);

/****************************************************************************/
/* BOOL LTPLibSendLEModifyWhitelistRsp                                      */
/* (                                                                        */
/*    PLTPLib pLTPLib      : pointer to LTP context to be used              */
/*    uint8_t    copmsk       : bitmask defining content of optional parameter */
/*    uint8_t *  pOpt         : pointer to optional parameter structure        */
/*    uint8_t    cause        : result code for this message                   */
/*    uint8_t    whitelistOp  : whitelist operation from request               */
/* )                                                                        */
/* return:------------------------------------------------------------------*/
/* TRUE in case the message was send send successfully                      */
/* FALSE otherwise                                                          */
/*                                                                          */
/* Description:-------------------------------------------------------------*/
/* This function can be used to send the LTP message in the function name   */
/*                                                                          */
/****************************************************************************/
BOOL LTPLibSendLEModifyWhitelistRsp(PLTPLib pLTPLib, uint8_t copmsk, uint8_t * pOpt, uint8_t cause, uint8_t whitelistOp);

/****************************************************************************/
/* BOOL LTPLibSendLEConnectionUpdateRsp                                     */
/* (                                                                        */
/*    PLTPLib pLTPLib      : pointer to LTP context to be used              */
/*    uint8_t    copmsk       : bitmask defining content of optional parameter */
/*    uint8_t *  pOpt         : pointer to optional parameter structure        */
/*    uint8_t    cause        : result code for this message                   */
/*    uint8_t    loc_MDL_ID   : reference to local MDL identifier              */
/* )                                                                        */
/* return:------------------------------------------------------------------*/
/* TRUE in case the message was send send successfully                      */
/* FALSE otherwise                                                          */
/*                                                                          */
/* Description:-------------------------------------------------------------*/
/* This function can be used to send the LTP message in the function name   */
/*                                                                          */
/****************************************************************************/
BOOL LTPLibSendLEConnectionUpdateRsp(PLTPLib pLTPLib, uint8_t copmsk, uint8_t * pOpt, uint8_t cause, uint8_t loc_MDL_ID);

/****************************************************************************/
/* BOOL LTPLibSendLEConnectionUpdateInd                                     */
/* (                                                                        */
/*    PLTPLib pLTPLib      : pointer to LTP context to be used              */
/*    uint8_t    copmsk       : bitmask defining content of optional parameter */
/*    uint8_t *  pOpt         : pointer to optional parameter structure        */
/*    uint8_t    loc_MDL_ID   : reference to local MDL identifier              */
/*    uint16_t    connIntervalMin   : minimum connection interval               */
/*    uint16_t    connIntervalMax   : maximum connection interval               */
/*    uint16_t    connLatency       : connection latency                        */
/*    uint16_t    supervisionTimeout: supervision timeout                       */
/* )                                                                        */
/* return:------------------------------------------------------------------*/
/* TRUE in case the message was send send successfully                      */
/* FALSE otherwise                                                          */
/*                                                                          */
/* Description:-------------------------------------------------------------*/
/* This function can be used to send the LTP message in the function name   */
/*                                                                          */
/****************************************************************************/
BOOL LTPLibSendLEConnectionUpdateInd(PLTPLib pLTPLib, uint8_t copmsk, uint8_t * pOpt, uint8_t loc_MDL_ID, uint16_t connIntervalMin, uint16_t connIntervalMax, uint16_t connLatency, uint16_t supervisionTimeout);

/****************************************************************************/
/* BOOL LTPLibSendLEConnectionParameterInfo                                 */
/* (                                                                        */
/*    PLTPLib pLTPLib      : pointer to LTP context to be used              */
/*    uint8_t    copmsk       : bitmask defining content of optional parameter */
/*    uint8_t *  pOpt         : pointer to optional parameter structure        */
/*    uint8_t    loc_MDL_ID   : reference to local MDL identifier              */
/*    uint16_t    connInterval : current connection interval                    */
/*    uint16_t    connLatency  : current connection latency                     */
/*    uint16_t    supervisionTimeout: current supervision timeout               */
/* )                                                                        */
/* return:------------------------------------------------------------------*/
/* TRUE in case the message was send send successfully                      */
/* FALSE otherwise                                                          */
/*                                                                          */
/* Description:-------------------------------------------------------------*/
/* This function can be used to send the LTP message in the function name   */
/*                                                                          */
/****************************************************************************/
BOOL LTPLibSendLEConnectionParameterInfo(PLTPLib pLTPLib, uint8_t copmsk, uint8_t * pOpt, uint8_t loc_MDL_ID, uint16_t connInterval, uint16_t connLatency, uint16_t supervisionTimeout);
BOOL LTPLibSendSetBleTxPowerRsp(PLTPLib pLTPLib, uint8_t tx_power_index, uint8_t cause, uint16_t subCause);
BOOL LTPLibSendSetDataLengthRsp(PLTPLib pLTPLib, uint8_t loc_MDL_ID, uint8_t cause);
BOOL LTPLibSendDataLengthChangeInfo(PLTPLib pLTPLib, uint8_t loc_MDL_ID, uint16_t MaxTxOctets, uint16_t MaxTxTime, uint16_t MaxRxOctets, uint16_t MaxRxTime);
BOOL LTPLibSendDownloadServiceDatabaseRsp(PLTPLib pLTPLib, uint8_t copmsk, uint8_t * pOpt, uint8_t cause);
BOOL LTPLibSendClearServiceDatabaseRsp(PLTPLib pLTPLib, uint8_t copmsk, uint8_t * pOpt, uint8_t cause);
BOOL LTPLibSendSetTraceLevelRsp(PLTPLib pLTPLib, uint8_t copmsk, uint8_t * pOpt, uint8_t cause);
BOOL LTPLibSendGATTAttributeExecuteWriteRsp(PLTPLib pLTPLib, uint8_t loc_MDL_ID, uint8_t cause, uint16_t subCause);
BOOL LTPLibSendGATTAttributeExecuteWriteInd(PLTPLib pLTPLib, uint8_t copmsk, uint8_t * pOpt, uint8_t loc_MDL_ID, uint8_t flags);
/****************************************************************************/
/* BOOL LTPLibSendPairableModeSetReq                                        */
/* (                                                                        */
/*    PLTPLib pLTPLib      : pointer to LTP context to be used              */
/*    uint8_t    enablePairableMode   : TRUE if pairable mode is enabled       */
/*    uint8_t    BluetoothMode        : supported Bluetooth mode if pairable   */
/*    uint8_t    AuthRequirements     : authentication requirements if pairable*/
/*    uint8_t    IOCapabilities       : capabilities for auhtentication        */
/*    uint8_t    remoteOOBDataPresent : TRUE if remote OOB data present        */
/* )                                                                        */
/* return:------------------------------------------------------------------*/
/* TRUE in case the message was send send successfully                      */
/* FALSE otherwise                                                          */
/*                                                                          */
/* Description:-------------------------------------------------------------*/
/* This function can be used to send the LTP message in the function name   */
/*                                                                          */
/* Porting to BlueAPI+:-----------------------------------------------------*/
/* The corresponding function in the Stollmann system internal software API */
/* BlueAPI+ for using LTP functionality without serial link interface is    */
/* blueAPI_PairableModeSetReq(...)                                          */
/*                                                                          */
/****************************************************************************/
BOOL LTPLibSendPairableModeSetReq(PLTPLib pLTPLib, uint8_t enablePairableMode, uint8_t AuthRequirements, uint8_t IOCapabilities, uint8_t remoteOOBDataPresent);

/****************************************************************************/
/* BOOL LTPLibSendPasskeyReqReplyReq                                        */
/* (                                                                        */
/*    PLTPLib pLTPLib      : pointer to LTP context to be used              */
/*    uint8_t *  rem_BD       : Bluetooth device address of remote device      */
/*    uint32_t   passKey      : result of Keyboard input                       */
/*    uint8_t    cause        : result code for this message                   */
/* )                                                                        */
/* return:------------------------------------------------------------------*/
/* TRUE in case the message was send send successfully                      */
/* FALSE otherwise                                                          */
/*                                                                          */
/* Description:-------------------------------------------------------------*/
/* This function can be used to send the LTP message in the function name   */
/*                                                                          */
/* Porting to BlueAPI+:-----------------------------------------------------*/
/* The corresponding function in the Stollmann system internal software API */
/* BlueAPI+ for using LTP functionality without serial link interface is    */
/* blueAPI_UserPasskeyReqReplyReq(...)                                      */
/*                                                                          */
/****************************************************************************/
BOOL LTPLibSendPasskeyReqReplyReq(PLTPLib pLTPLib, uint8_t * rem_BD, uint32_t passKey, uint8_t cause );


/****************************************************************************/
/* BOOL LTPLibSendPasskeyRequestCnf                                         */
/* (                                                                        */
/*    PLTPLib pLTPLib      : pointer to LTP context to be used              */
/*    uint8_t *  rem_BD       : Bluetooth device address of remote device      */
/*    uint8_t    cause        : result code for this message                   */
/* )                                                                        */
/* return:------------------------------------------------------------------*/
/* TRUE in case the message was send send successfully                      */
/* FALSE otherwise                                                          */
/*                                                                          */
/* Description:-------------------------------------------------------------*/
/* This function can be used to send the LTP message in the function name   */
/*                                                                          */
/* Porting to BlueAPI+:-----------------------------------------------------*/
/* The corresponding function in the Stollmann system internal software API */
/* BlueAPI+ for using LTP functionality without serial link interface is    */
/* blueAPI_UserPasskeyReqConf(...)                                          */
/*                                                                          */
/****************************************************************************/
BOOL LTPLibSendPasskeyRequestCnf(PLTPLib pLTPLib, uint8_t * rem_BD, uint8_t cause);

/****************************************************************************/
/* BOOL LTPLibSendRemoteOOBReqCnf                                           */
/* (                                                                        */
/*    PLTPLib pLTPLib      : pointer to LTP context to be used              */
/*    uint8_t *  rem_BD       : Bluetooth device address of remote device      */
/*    uint8_t *  C            : 16 byte long hash C value                      */
/*    uint8_t *  R            : 16 byte long Randomizer R value                */
/*    uint8_t    cause        : result code for this message                   */
/* )                                                                        */
/* return:------------------------------------------------------------------*/
/* TRUE in case the message was send send successfully                      */
/* FALSE otherwise                                                          */
/*                                                                          */
/* Description:-------------------------------------------------------------*/
/* This function can be used to send the LTP message in the function name   */
/*                                                                          */
/* Porting to BlueAPI+:-----------------------------------------------------*/
/* The corresponding function in the Stollmann system internal software API */
/* BlueAPI+ for using LTP functionality without serial link interface is    */
/* blueAPI_RemoteOOBDataReqConf(...)                                        */
/*                                                                          */
/****************************************************************************/
BOOL LTPLibSendRemoteOOBReqCnf(PLTPLib pLTPLib, uint8_t * rem_BD, uint8_t * C, uint8_t * R, uint8_t cause);

/****************************************************************************/
/* BOOL LTPLibSendAuthResultExtCnf                                          */
/* (                                                                        */
/*    PLTPLib pLTPLib      : pointer to LTP context to be used              */
/*    uint8_t    cause        : result code for this message                   */
/*    uint8_t *  rem_BD       : pointer to remote device address (6 bytes)     */
/*    uint8_t    rem_BD_Type  : Type of Bluetooth address                      */
/*    uint32_t   AppData      : MDH specific data to a given bond table entry  */
/* )                                                                        */
/* return:------------------------------------------------------------------*/
/* TRUE in case the message was send send successfully                      */
/* FALSE otherwise                                                          */
/*                                                                          */
/* Description:-------------------------------------------------------------*/
/* This function can be used to send the LTP message in the function name   */
/*                                                                          */
/* Porting to BlueAPI+:-----------------------------------------------------*/
/* The corresponding function in the Stollmann system internal software API */
/* BlueAPI+ for using LTP functionality without serial link interface is    */
/* blueAPI_AuthResultConf(...)                                              */
/*                                                                          */
/****************************************************************************/
BOOL LTPLibSendAuthResultExtCnf(PLTPLib pLTPLib, uint8_t cause, uint8_t * rem_BD,
                                uint8_t rem_BD_Type, uint32_t AppData);


/****************************************************************************/
/* BOOL LTPLibSendAuthResultRequestExtCnf                                   */
/* (                                                                        */
/*    PLTPLib pLTPLib      : pointer to LTP context to be used              */
/*    uint8_t    cause        : result code for this message                   */
/*    uint8_t *  rem_BD       : pointer to remote device address (6 bytes)     */
/*    uint8_t    rem_BD_Type  : Type of Bluetooth address                      */
/*    uint8_t    keyType      : type of the generated linkkey                  */
/*    uint8_t *  linkKey      : Bluetooth Linkkey                              */
/*    uint16_t    linkKeyLength: Length of Linkkey data                         */
/* )                                                                        */
/* return:------------------------------------------------------------------*/
/* TRUE in case the message was send send successfully                      */
/* FALSE otherwise                                                          */
/*                                                                          */
/* Description:-------------------------------------------------------------*/
/* This function can be used to send the LTP message in the function name   */
/*                                                                          */
/* Porting to BlueAPI+:-----------------------------------------------------*/
/* The corresponding function in the Stollmann system internal software API */
/* BlueAPI+ for using LTP functionality without serial link interface is    */
/* blueAPI_AuthResultRequestConf(...)                                       */
/*                                                                          */
/****************************************************************************/
BOOL LTPLibSendAuthResultRequestExtCnf(PLTPLib pLTPLib, uint8_t cause, uint8_t * rem_BD,
                                       uint8_t rem_BD_Type, uint8_t keyType,
                                       uint8_t * linkKey, uint16_t linkKeyLength);


/****************************************************************************/
/* BOOL LTPLibSendCreateMDLCnf                                              */
/* (                                                                        */
/*    PLTPLib pLTPLib      : pointer to LTP context to be used              */
/*    uint8_t    loc_MDL_ID   : reference to local MDL identifier              */
/*    uint8_t    LinkConfigType   : requested QoS-configuration for this link  */
/*                               Default = LTP_LINK_CONFIG_RELIABLE         */
/*    uint8_t    maxTPDUusCredits : maximum num of usCredits allowed for MDL   */
/*                               Default = 0x00                             */
/*    uint8_t    cause        : result code for this message                   */
/* )                                                                        */
/* return:------------------------------------------------------------------*/
/* TRUE in case the message was send send successfully                      */
/* FALSE otherwise                                                          */
/*                                                                          */
/* Description:-------------------------------------------------------------*/
/* This function can be used to send the LTP message in the function name   */
/*                                                                          */
/* Porting to BlueAPI+:-----------------------------------------------------*/
/* The corresponding function in the Stollmann system internal software API */
/* BlueAPI+ for using LTP functionality without serial link interface is    */
/* blueAPI_CreateMDLConf(...)                                               */
/*                                                                          */
/****************************************************************************/
BOOL LTPLibSendCreateMDLCnf(PLTPLib pLTPLib, uint8_t loc_MDL_ID, uint8_t LinkConfigType, uint8_t maxTPDUusCredits, uint8_t cause);

/****************************************************************************/
/* BOOL LTPLibSendDisconnectMDLReq                                          */
/* (                                                                        */
/*    PLTPLib pLTPLib      : pointer to LTP context to be used              */
/*    uint8_t    loc_MDL_ID   : reference to local MDL identifier              */
/*    uint8_t    cause        : result code for this message                   */
/* )                                                                        */
/* return:------------------------------------------------------------------*/
/* TRUE in case the message was send send successfully                      */
/* FALSE otherwise                                                          */
/*                                                                          */
/* Description:-------------------------------------------------------------*/
/* This function can be used to send the LTP message in the function name   */
/*                                                                          */
/* Porting to BlueAPI+:-----------------------------------------------------*/
/* The corresponding function in the Stollmann system internal software API */
/* BlueAPI+ for using LTP functionality without serial link interface is    */
/* blueAPI_DisconnectMDLReq(...)                                            */
/*                                                                          */
/****************************************************************************/
BOOL LTPLibSendDisconnectMDLReq(PLTPLib pLTPLib, uint8_t loc_MDL_ID, uint8_t cause);

/****************************************************************************/
/* BOOL LTPLibSendDisconnectMDLCnf                                          */
/* (                                                                        */
/*    PLTPLib pLTPLib      : pointer to LTP context to be used              */
/*    uint8_t    loc_MDL_ID   : reference to local MDL identifier              */
/* )                                                                        */
/* return:------------------------------------------------------------------*/
/* TRUE in case the message was send send successfully                      */
/* FALSE otherwise                                                          */
/*                                                                          */
/* Description:-------------------------------------------------------------*/
/* This function can be used to send the LTP message in the function name   */
/*                                                                          */
/* Porting to BlueAPI+:-----------------------------------------------------*/
/* The corresponding function in the Stollmann system internal software API */
/* BlueAPI+ for using LTP functionality without serial link interface is    */
/* blueAPI_DisconnectMDLConf(...)                                           */
/*                                                                          */
/****************************************************************************/
BOOL LTPLibSendDisconnectMDLCnf(PLTPLib pLTPLib, uint8_t loc_MDL_ID);

uint8_t * LTPLibWriteHeader(PLTPLib pLTPLib, LPWORD offset, uint8_t cmd, uint8_t copmsk, uint8_t * pOpt, uint16_t varLen);
BOOL LTPLibSendMessage_BD(PLTPLib pLTPLib, uint8_t copmsk, uint8_t * pOpt, uint8_t cmd, uint8_t * bd);
BOOL LTPLibSendMessage_BYTE(PLTPLib pLTPLib, uint8_t copmsk, uint8_t * pOpt, uint8_t cmd, uint8_t param);
BOOL LTPLibSendMessage_BYTE_BYTE(PLTPLib pLTPLib, uint8_t copmsk, uint8_t * pOpt, uint8_t cmd, uint8_t param1, uint8_t param2);
BOOL LTPLibSendMessage_BD_BYTE(PLTPLib pLTPLib, uint8_t copmsk, uint8_t * pOpt, uint8_t cmd, uint8_t * bd, uint8_t param);
BOOL LTPLibSendMessage_BD_DWORD(PLTPLib pLTPLib, uint8_t copmsk, uint8_t * pOpt, uint8_t cmd, uint8_t * bd, uint32_t param);
BOOL LTPLibSendMessage_BYTE_BD(PLTPLib pLTPLib, uint8_t copmsk, uint8_t * pOpt, uint8_t cmd, uint8_t param, uint8_t * bd);

#ifdef __cplusplus
}
#endif

#endif /* LTPLIB_H */
