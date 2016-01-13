/**
********************************************************************************************************
Copyright (c) 2015, Realtek Semiconductor Corporation. All rights reserved.
*********************************************************************************************************
* @file        bterrcod.h
* @brief      BLUETOOTH global errorcode definitions
* @details   
*
* @author   	gordon
* @date      	2015-07-13
* @version	v0.1
*/

#ifndef __BTERRCOD_H
#define __BTERRCOD_H


/** Error codes delivered by BlueFace+ and BlueFace+MPA:
*    Highbyte defines Layer (BT_ERR_SRC_MASK)
*    Lowbyte defines Error Code within Layer */

/** Error Mask (Highbyte) to indicate Error Code */
#define BLUEFACE_ERR_SRC_MASK             0xFF00

/** ULS stack - BlueFace+ Api  */
#define BLUEFACE_ERR                0x000

/** ULS stack - protocol layer Sources */
#define HCI_ERR                     0x100
#define L2CAP_ERR                   0x200
#define SDP_ERR                     0x300
#define RFCOMM_ERR                  0x400
#define TCSBIN_ERR                  0x500
#define UPFL2C_ERR                  0x600
#define ATT_ERR                     0x700

#define HCRP_ERR                    0x800
#define BNEP_ERR                    0x900
#define AVDTP_ERR                   0xA00
#define SECMAN_ERR                  0xB00
#define GATT_ERR                    0xC00
#define APP_ERR                    0xD00 /**< @brief GATT layer */

/** BlueFace control applications */
#define BLUESECURE_ERR             0x7000  /**< BlueSecure Error codes - above BlueFace   */

/** profile layer - MPA generic  */
#define MPA_ERR                    0x8000  /**< mpa specific errors defined in bluempa.h */

/** profile layer -  */
#define MPA_GAP_ERR                0x8100  /**< Generic Access Profile */
#define MPA_SDAP_ERR               0x8200  /**< Service Discovery Application Profile */
#define MPA_SPP_ERR                0x8300  /**< Serial Port Profile */
#define MPA_HFP_ERR                0x8400  /**< Handsfree Profile */
#define MPA_OBEX_ERR               0x8500  /**< Obex Profile */

/** Layer Specific Error Code definitions*/
/** Blueface Error Codes  */
/** !!! keep value, must be compatible to blueFaceStatus !!! */
#define BF_SUCCESS                              0x00
#define BF_ILL_VERSION                          0x01
#define BF_ILL_APPLICATION                      0x02
#define BF_NO_RESOURCES                         0x04
#define BF_ILL_PARAMETER                        0x05
#define BF_BUSY                                 0x08
#define BF_PSM_NOT_IMPLEMENTED                  0x0B

/** HCI Error Codes  */
#define HCI_SUCCESS                             0x00
#define HCI_ERR_UNKNOWN_COMMAND                 0x01
#define HCI_ERR_NOCONNECTION                    0x02
#define HCI_ERR_HARDWARE_FAIL                   0x03
#define HCI_ERR_PAGE_TIMEOUT                    0x04
#define HCI_ERR_AUTHENTICATION_FAILED           0x05
#define HCI_ERR_KEY_MISSING                     0x06
#define HCI_ERR_MEMORY_FULL                     0x07
#define HCI_ERR_CONNECTION_TIMEOUT              0x08
#define HCI_ERR_MAX_NUM_CONNECTIONS             0x09
#define HCI_ERR_MAX_NUM_SCO_CONNECTIONS         0x0A  /**< 10 */
#define HCI_ERR_ACL_CONN_ALREADY_EXISTS         0x0B  /**< 11 */
#define HCI_ERR_COMMAND_DISALLOWED              0x0C  /**< 12 */
#define HCI_ERR_HOST_REJECTED_0D                0x0D  /**< 13 */
#define HCI_ERR_REJECT_LIMITED_RESOURCES HCI_ERR_HOST_REJECTED_0D
#define HCI_ERR_HOST_REJECTED_0E                0x0E  /**< 14 */
#define HCI_ERR_REJECT_SECURITY_REASONS HCI_ERR_HOST_REJECTED_0E
#define HCI_ERR_HOST_REJECTED_0F                0x0F  /**< 15 */
#define HCI_ERR_REJECT_UNACCEPTABLE BD_ADDR HCI_ERR_HOST_REJECTED_0F
#define HCI_ERR_HOST_TIMEOUT                    0x10  /**< 16 */
#define HCI_ERR_UNSUPPORTED_PARAM_VALUE         0x11  /**< 17 */
#define HCI_ERR_INVALID_HCI_PARAMETER_VALUE     0x12  /**< 18 */
#define HCI_ERR_OTHER_END_TERMINATE_13          0x13  /**< 19 */
#define HCI_ERR_OTHER_END_TERMINATE_14          0x14  /**< 20 */
#define HCI_ERR_OTHER_END_TERMINATE_15          0x15  /**< 21 */
#define HCI_ERR_CONNECTION_TERMINATE_LOCALLY    0x16  /**< 22 */
#define HCI_ERR_REPEATED_ATTEMPTS               0x17  /**< 23 */
#define HCI_ERR_PARING_NOT_ALLOWED              0x18  /**< 24 */
#define HCI_ERR_UNKNOWN_LMP_PDU                 0x19  /**< 25 */
#define HCI_ERR_UNSUPPORTED_REMOTE_FEATURE      0x1A  /**< 26 */
#define HCI_ERR_SCO_OFFSET_REJECTED             0x1B  /**< 27 */
#define HCI_ERR_SCO_INTERVAL_REJECTED           0x1C  /**< 28 */
#define HCI_ERR_SCO_AIR_MODE_REJECTED           0x1D  /**< 29 */
#define HCI_ERR_INVALID_LMP_PARAMETERS          0x1E  /**< 30 */
#define HCI_ERR_UNSPECIFIED_ERROR               0x1F  /**< 31 */
#define HCI_ERR_UNSUPPORTED_LMP_PARAMETER_VAL   0x20  /**< 32 */
#define HCI_ERR_ROLE_CHANGE_NOT_ALLOWED         0x21  /**< 33 */
#define HCI_ERR_LMP_RESPONSE_TIMEOUT            0x22  /**< 34 */
#define HCI_ERR_LMP_ERROR_TRANSACTION_COLLISION 0x23  /**< 35 */
#define HCI_ERR_LMP_PDU_NOT_ALLOWED             0x24  /**< 36 */
/** the next ones are new in 1.1 */
#define HCI_ERR_ENCRYPTION_MODE_NOT_ACCEPTABLE  0x25  /**< 37 */
#define HCI_ERR_UNIT_KEY_USED                   0x26  /**< 38 */
#define HCI_ERR_QOS_NOT_SUPPORTED               0x27  /**< 39 */
#define HCI_ERR_INSTANT_PASSED                  0x28  /**< 40 */
#define HCI_ERR_PAIRING_WITH_UNIT_KEY_NOT_SUPP  0x29  /**< 41 */

#define HCI_ERR_DIFFERENT_TRANSACTION_COLLISION 0x2A  /**< 42 */
#define HCI_ERR_QOS_UNACCEPTABLE_PARAMETER      0x2C  /**< 44 */
#define HCI_ERR_QOS_REJECT                      0x2D  /**< 45 */
#define HCI_ERR_CHANNEL_ASSESSMENT_NOT_SUPPORTED      0x2E  /**< 46 */
#define HCI_ERR_INSUFFICIENT_SECURITY           0x2F  /**< 47 */
#define HCI_ERR_PARAMETER_OUT_OF_MANDATORY      0x30  /**< 48 */
#define HCI_ERR_ROLE_SWITCH_PANDING             0x32  /**< 50 */
#define HCI_ERR_RESERVED_SLOT_VIOLATION         0x34  /**< 52 */
#define HCI_ERR_ROLE_SWITCH_FAILED              0x35  /**< 53 */
#define HCI_ERR_EXTENDED_INQUIRY_RESPONSE_TOO_LARGE   0x36   /**< 54 */
#define HCI_ERR_SIMPLE_PAIRING_NOT_SUPPORTED_BY_HOST  0x37   /**< 55 */
#define HCI_ERR_HOST_BUSY_PAIRING               0x38  /**< 56 */
#define HCI_ERR_CONNECTION_REJECTED_DUE_TO_NOT_SUITABLE_CHANNEL_FOUND 0x39  /**< 57 */
#define HCI_ERR_CONTROLLER_BUSY                 0x3A  /**< 58 */
#define HCI_ERR_UNACCEPTABLE_CONNECTION_INTERVAL          0x3B  /**< 59 */
#define HCI_ERR_DIRECTED_ADVERTISING_TIMEOUT              0x3C  /**< 60 */
#define HCI_ERR_CONNECTION_TERMINATED_DUE_TO_MIC_FAILURE  0x3D  /**< 61 */
#define HCI_ERR_CONNECTION_FAILED_TO_BE_ESTABLISHED       0x3E  /**< 62 */
#define HCI_ERR_MAC_CONNECTION_FAILED                     0x3F  /**< 63 */

/** HCI locally generated Error Codes */
#define HCI_ERR_ILLEGAL_HANDLE                0x64  /* 100 */
#define HCI_ERR_TIMEOUT                       0x65  /* 101 */
#define HCI_ERR_OUTOFSYNC                     0x66  /* 102 */
#define HCI_ERR_NO_DESCRIPTOR                 0x67  /* 103 */
#define HCI_ERR_INFOPAGE_MISSING              0x68  /* 104 */

/**L2CAP ERROR codes */
#define L2CAP_NO_CAUSE                        0x00
#define L2CAP_CONNECTION_ACCEPT               0x00
#define L2CAP_ERR_PENDING                     0x01
#define L2CAP_ERR_REFUS_INV_PSM               0x02
#define L2CAP_ERR_REFUS_SEC_BLOCK             0x03
#define L2CAP_ERR_REFUS_NO_RESOURCE           0x04
#define L2CAP_ERR_ILLEGAL_PARAMETER           0x05

#define L2CAP_ERR_INSUFFICIENT_AUTHENTICATION 0x10
#define L2CAP_ERR_INSUFFICIENT_AUTHORIZATION  0x11
#define L2CAP_ERR_INSUFFICIENT_KEY_SIZE       0x12
#define L2CAP_ERR_INSUFFICIENT_ENCRYPTION     0x13
#define L2CAP_ERR_CREDITS_EXCEED_RANGE        0x14
#define L2CAP_ERR_INVAILD_PDU                 0x15
#define L2CAP_ERR_CREDITS_LACK                0x16
#define L2CAP_ERR_NO_RESOURCE                 0x17
#define L2CAP_ERR_BUSY                        0x18
#define L2CAP_ERR_LE_CHANNEL_NOT_OPEN         0x19
#define L2CAP_ERR_ILLEGAL_STATE               0x20
#define L2CAP_ERR_LINK_NOT_EXIST              0x21
#define L2CAP_ERR_CMD_NOT_UNDERSTOOD          0x22
#define L2CAP_ERR_QUEUE_IS_FULL               0x23
#define L2CAP_ERR_INVAlID_SOURCE_CID          0x24
#define L2CAP_ERR_SOURCE_CID_ALREADY_ALLOCATED  0x25

#define L2CAP_ERR_RXSEQ_INV                   0x30
#define L2CAP_ERR_RXTXSEQ_INV                 0x31
#define L2CAP_ERR_NO_RESPONSE                 0x32
#define L2CAP_ERR_MISSING_IFRAME              0x33

#define L2CAP_ERR_INCOMPATIBLE_FLUSHTO_VALUE  0x40
#define L2CAP_ERR_OUT_OF_RANGE                0xED
#define L2CAP_ERR_TIMEOUT_EXTERNAL            0xEE

/** sdp error codes, p.347 */
#define SDP_ERR_RESERVED                      0x00
#define SDP_ERR_UNSUPPORTED_SDP_VERSION       0x01
#define SDP_INVALID_SERVICE_RECORD_HANDLE     0x02
#define SDP_INVALID_REQUEST_SYNTAX            0x03
#define SDP_INVALID_PDU_SIZE                  0x04
#define SDP_INVALID_CONTINUATION_STATE        0x05
#define SDP_INSUFFICIENT_RESOURCES            0x06

/** Local SDP error codes */
#define SDP_ERR_UNHANDLED_CODE                0x64  /**< 100 Client received unhandled SDP opcode */
#define SDP_ERR_TIMEOUT                       0x65  /**< 101 No answer from server (timeout)      */
#define SDP_ERR_NOTFOUND                      0x66  /**< 102 specified service not found          */
#define SDP_INVALID_RESPONSE_SYNTAX           0x67  /**< 103 Syntax Error in Response             */
#define SDP_NOT_FOUND                         0xC8  /**< 200 not really an error code             */

/** ATT Error Codes (see ATT spec. table 3.3)  */
#define ATT_OK                              0    /**< internal value .. */
#define ATT_ERR_INVALID_HANDLE              0x01 /**< Attribute handle value given was not valid on this attribute server */
#define ATT_ERR_READ_NOT_PERMITTED          0x02 /**< Attribute cannot be read */
#define ATT_ERR_WRITE_NOT_PERMITTED         0x03 /**< Attribute cannot be written */
#define ATT_ERR_INVALID_PDU                 0x04 /**< The attribute PDU was invalid */
#define ATT_ERR_INSUFFICIENT_AUTHENTICATION 0x05 /**< The attribute requires authentication before it can be read or written */
#define ATT_ERR_UNSUPPORTED_REQUEST         0x06 /**< Attribute server doesn't support the request received from the attribute client */
#define ATT_ERR_INVALID_OFFSET              0x07 /**< Offset specified was past the end of the attribute */
#define ATT_ERR_INSUFFICIENT_AUTHORIZATION  0x08 /**< The attribute requires an authorization before it can be read or written */
#define ATT_ERR_PREPARE_QUEUE_FULL          0x09 /**< Too many prepare writes have been queued */
#define ATT_ERR_ATTR_NOT_FOUND              0x0A /**< No attribute found within the given attribute handle range */
#define ATT_ERR_ATTR_NOT_LONG               0x0B /**< Attribute cannot be read or written using the Read Blob Request or Prepare Write Request */
#define ATT_ERR_INSUFFICIENT_KEY_SIZE       0x0C /**< The Encryption Key Size used for encrypting this link is insufficient */
#define ATT_ERR_INVALID_VALUE_SIZE          0x0D /**< The attribute value length is invalid for the operation */
#define ATT_ERR_UNLIKELY                    0x0E /**< The attribute request that was requested has encountered an error that was very unlikely, and therefore could not be completed as requested */
#define ATT_ERR_INSUFFICIENT_ENCRYPTION     0x0F /**< The attribute requires encryption before it can be read or written */
#define ATT_ERR_UNSUPPORTED_GROUP_TYPE      0x10 /**< The attribute type is not a supported grouping attribute as defined by a higher layer specification */
#define ATT_ERR_INSUFFICIENT_RESOURCES      0x11 /**< Insufficient Resources to complete the request */

/** profile dependent application error codes >= 0x80: */
#define ATT_ERR_MIN_APPLIC_CODE          0x80 /**< minimal application error code */
#define ATT_ERR_INVALID_VALUE            0x80 /**< The attribute value is invalid for the operation */

/** manufacturer specific error codes that are "missing" in GATT spec. >= 0xC0:   */
#define ATT_ERR_INVALID_CCC_BITS         0xC0 /**< invalid client char. config. bits */

/** error codes common to various profiles (see "CSS v2.pdf"), >= 0xE0 */
#define ATT_ERR_CCCD_IMPROPERLY_CONFIGURED  0xFD
#define ATT_ERR_PROC_ALREADY_IN_PROGRESS    0xFE
#define ATT_ERR_OUT_OF_RANGE                0xFF

/** GATT error codes  */
#define GATT_SUCCESS                        0x00
#define GATT_ERR_OUT_OF_RESOURCE            (GATT_ERR | 0x01)
#define GATT_ERR_UNSUPPORTED                (GATT_ERR | 0x02)
#define GATT_ERR_ILLEGAL_ROLE               (GATT_ERR | 0x03)
#define GATT_ERR_ILLEGAL_STATE              (GATT_ERR | 0x04)
#define GATT_ERR_ILLEGAL_CODING             (GATT_ERR | 0x05)
#define GATT_ERR_ILLEGAL_HANDLE             (GATT_ERR | 0x06)
#define GATT_ERR_ILLEGAL_PARAMETER          (GATT_ERR | 0x07)
#define GATT_ERR_INTERNAL                   (GATT_ERR | 0x08)
#define GATT_ERR_NOT_ALLOWED                (GATT_ERR | 0x09)
#define GATT_ERR_NOTIF_IND_NOT_CFG          (GATT_ERR | 0x0A)
#define GATT_ERR_NOTIF_IND_CFG              (GATT_ERR | 0x0B)
#define GATT_ERR_NOTIF_IND_CONF_PD          (GATT_ERR | 0x0C)
#define GATT_ERR_TIMEOUT                    (GATT_ERR | 0x0D)
#define GATT_ERR_LINK_DEACTIVATED           (GATT_ERR | 0x0E)
#define GATT_ERR_NOT_AUTHENTICATED          (GATT_ERR | 0x0F)

/** RFCOMM Layer To Layer Error Code */
#define RFCOMM_SUCCESS                        0x00
#define RFCOMM_ERR_NORESOURCES                0x01
#define RFCOMM_ERR_ILL_PARAMETER              0x02

/** RFCOMM Local Error Codes */
#define RFCOMM_ERR_REJECTED                   0x64  /**< 100 Connection setup was rejected by remote side (DM) */
#define RFCOMM_ERR_TIMEOUT                    0x65  /**< 101 Connection timed out  */
#define RFCOMM_ERR_NSC                        0x66  /**< 102 Non Supported Command received */
#define RFCOMM_ERR_ILLPARAMETER               0x67  /**< 103 Illegal parameter */

/** HCRP Layer To Layer Error Code */
#define HCRP_SUCCESS                          0x00
#define HCRP_ERR_NORESOURCES                  0x01

/** BNEP Layer To Layer Error Code */
#define BNEP_SUCCESS                          0x00
#define BNEP_ERR_INVALID_DEST_UUID            0x01
#define BNEP_ERR_INVALID_SOURCE_UUID          0x02
#define BNEP_ERR_INVALID_UUID_SIZE            0x03
#define BNEP_ERR_NOT_ALLOWED                  0x04
#define BNEP_ERR_NORESOURCES                  0x0A    /**< 10 */
#define BNEP_ERR_INVALID_FRAME_SIZE           0x0B    /**< 11 */
#define BNEP_ERR_TIMEOUT                      0x0C    /**< 12 */

/** AVDTP Layer To Layer Error Code (the values 0x01 ... 0x31 are defined by AVDTP)  */
#define AVDTP_SUCCESS                         0x00

#define AVDTP_ERR_BAD_HEADER_FORMAT           0x01

#define AVDTP_ERR_BAD_LENGTH                  0x11
#define AVDTP_ERR_BAD_ACP_SEID                0x12
#define AVDTP_ERR_SEP_IN_USE                  0x13
#define AVDTP_ERR_SEP_NOT_IN_USE              0x14
#define AVDTP_ERR_BAD_SERV_CATEGORY           0x17
#define AVDTP_ERR_BAD_PAYLOAD_FORMAT          0x18
#define AVDTP_ERR_NOT_SUPPORTED_COMMAND       0x19
#define AVDTP_ERR_INVALID_CAPABILITIES        0x1A

#define AVDTP_ERR_BAD_RECOVERY_TYPE           0x22
#define AVDTP_ERR_BAD_MEDIA_TRANSPORT_FORMAT  0x23
#define AVDTP_ERR_BAD_RECOVERY_FORMAT         0x25
#define AVDTP_ERR_BAD_ROHC_FORMAT             0x26
#define AVDTP_ERR_BAD_CP_FORMAT               0x27
#define AVDTP_ERR_BAD_MULTIPLEXING_FORMAT     0x28
#define AVDTP_ERR_UNSUPPORTED_CONFIGURATION   0x29

#define AVDTP_ERR_BAD_STATE                   0x31


#define AVDTP_ERR_NORESOURCES                 0xF1
#define AVDTP_ERR_INVALID_FRAME_SIZE          0xF2
#define AVDTP_ERR_TIMEOUT                     0xF3
#define AVDTP_ERR_UNKNOWN_SIGNAL              0xF4
#define AVDTP_ERR_ILLEGAL_COMMAND             0xF5
#define AVDTP_ERR_ILLEGAL_RESPONSE            0xF6


/** TCS BIN error codes (the values 0x01 ... 0x7F are defined by TCS-BIN */
/**  1: unassigned number */
#define TCS_BIN_CAUSE_UNASSIGNED_NUMBER            0x01
/**  2: no route to transit network */
#define TCS_BIN_CAUSE_NO_ROUTE_TO_TNW              0x02
/**  3: no route to destination */
#define TCS_BIN_CAUSE_NO_ROUTE_TO_DEST             0x03
/**  4: send special information tone */
#define TCS_BIN_CAUSE_SEND_SPECIAL_INFO_TONE       0x04
/**  5: misdialled trunk prefix */
#define TCS_BIN_CAUSE_MISDIALLED_TRUNK_PREFIX      0x05
/**  6: channel unacceptable */
#define TCS_BIN_CAUSE_CHAN_UNACCEPTABLE            0x06
/**  7: call awarded and delivered */
#define TCS_BIN_CAUSE_CALL_AWARDED                 0x07
/**  8: preemption */
#define TCS_BIN_CAUSE_PREEMPTION                   0x08
/**  9: preemption, circuit reserved */
#define TCS_BIN_CAUSE_PREEMPTION_CIRCUIT_RESERVED  0x09
/** 16: normal clearing */
#define TCS_BIN_CAUSE_NORMAL_CLEARING              0x10
/** 17: user busy */
#define TCS_BIN_CAUSE_USER_BUSY                    0x11
/** 18: no user responding */
#define TCS_BIN_CAUSE_NO_USER_RESPONDING           0x12
/** 19: no answer from user (user alerted) */
#define TCS_BIN_CAUSE_NO_ANSWER_FROM_USER          0x13
/** 20: subscriber absent */
#define TCS_BIN_CAUSE_SUBSCRIBER_ABSENT            0x14
/** 21: call rejected */
#define TCS_BIN_CAUSE_CALL_REJECTED                0x15
/** 22: number changed */
#define TCS_BIN_CAUSE_NUMBER_CHANGED               0x16
/** 26: non-selected user clearing */
#define TCS_BIN_CAUSE_CLEAR_NON_SEL_USER           0x1A
/** 27: destination out of order */
#define TCS_BIN_CAUSE_OUT_OF_ORDER                 0x1B
/** 28: invalid number format */
#define TCS_BIN_CAUSE_INVALID_NUMBER               0x1C
/** 29: facility rejected */
#define TCS_BIN_CAUSE_FAC_REJECTED                 0x1D
/** 30: response to status enquiry */
#define TCS_BIN_CAUSE_RESP_STAT_ENQ                0x1E
/** 31: normal, unspecified */
#define TCS_BIN_CAUSE_NORMAL_UNSPEC                0x1F
/** 34: no channel available */
#define TCS_BIN_CAUSE_NO_CH_AVAILABLE              0x22
/** 38: network out of order */
#define TCS_BIN_CAUSE_NET_OUT_OF_ORDER             0x26
/** 39: permanent frame mode connection out-of-service */
#define TCS_BIN_CAUSE_PERM_FR_MODE_OUT_OF_SERVICE  0x27
/** 40: permanent frame mode connection operational */
#define TCS_BIN_CAUSE_PERM_FR_MODE_OPERATIONAL     0x28
/** 41: temporary failure */
#define TCS_BIN_CAUSE_TEMP_FAILURE                 0x29
/** 42: switching equipment congestion */
#define TCS_BIN_CAUSE_SWITCH_CONGESTION            0x2A
/** 43: user info discarded */
#define TCS_BIN_CAUSE_USER_INFO_DISC               0x2B
/** 44: requested channel not available */
#define TCS_BIN_CAUSE_REQ_CH_NOT_AVAIL             0x2C
/** 46: precedence call blocked */
#define TCS_BIN_CAUSE_PRECEDENCE_CALL_BLOCKED      0x2E
/** 47: resource unavailable, unspecified */
#define TCS_BIN_CAUSE_RESOURCE_UNAVAIL             0x2F
/** 49: quality of service unavailable */
#define TCS_BIN_CAUSE_QUALITY_UNAVAIL              0x31
/** 50: requested facility not subscribed */
#define TCS_BIN_CAUSE_REQ_FAC_NOT_SUBSCR           0x32
/** 53: outgoing calls barred within CUG */
#define TCS_BIN_CAUSE_OUT_CALLS_BARRED_CUG         0x35
/** 55: incoming calls barred within CUG */
#define TCS_BIN_CAUSE_IN_CALLS_BARRED_CUG          0x37
/** 57: bearer capability not authorized */
#define TCS_BIN_CAUSE_BC_NOT_AUTHORIZED            0x39
/** 58: bearer capability not available */
#define TCS_BIN_CAUSE_BC_NOT_AVAIL                 0x3A
/** 62: inconsistency outg. access / subscr */
#define TCS_BIN_CAUSE_INCONSISTENCY                0x3E
/** 63: service/option not available */
#define TCS_BIN_CAUSE_SERV_NOT_AVAIL               0x3F
/** 65: bearer capability not implemented */
#define TCS_BIN_CAUSE_BC_NOT_IMPL                  0x41
/** 66: channel type not implemented */
#define TCS_BIN_CAUSE_CH_TYPE_NOT_IMPL             0x42
/** 69: requested facility not implemented */
#define TCS_BIN_CAUSE_REQ_FAC_NOT_IMPL             0x45
/** 70: only restricted dig. BC available */
#define TCS_BIN_CAUSE_ONLY_RESTRICTED              0x46
/** 79: service/option not implemented */
#define TCS_BIN_CAUSE_SERV_NOT_IMPL                0x4F
/** 81: invalid cr value */
#define TCS_BIN_CAUSE_INVALID_CR_VALUE             0x51
/** 82: identified channel does not exist */
#define TCS_BIN_CAUSE_IDENT_CH_NOT_EXIST           0x52
/** 83: call identity does not exist */
#define TCS_BIN_CAUSE_CALL_IDENT_NOT_EXIST         0x53
/** 84: call identy in use */
#define TCS_BIN_CAUSE_CALL_IDENT_IN_USE            0x54
/** 85: no user suspended / deactivated */
#define TCS_BIN_CAUSE_NO_USER_SUSP                 0x55
/** 86: call having requested CAI has been cleared */
#define TCS_BIN_CAUSE_CALL_WITH_CAI_CLEARED        0x56
/** 87: destination address not member of CUG */
#define TCS_BIN_CAUSE_DAD_NOT_MEMB_OF_CUG          0x57
/** 88: incompatible destination */
#define TCS_BIN_CAUSE_INCOMP_DEST                  0x58
/** 90: non-existent closed user group */
#define TCS_BIN_CAUSE_NON_EXISTENT_CUG             0x5A
/** 91: invalid transit network selection */
#define TCS_BIN_CAUSE_INVALID_TNS                  0x5B
/** 95: invalid message, unspecified */
#define TCS_BIN_CAUSE_INVALID_MESSAGE              0x5F
/** 96: mandatory element missing */
#define TCS_BIN_CAUSE_MANDATORY_MISS               0x60
/** 97: message type does not exist */
#define TCS_BIN_CAUSE_MESS_NOT_EXIST               0x61
/** 98: message not compatible with call state or ...*/
#define TCS_BIN_CAUSE_MESS_NOT_COMPATIBLE          0x62
/** 99: information element non-existent/impl. */
#define TCS_BIN_CAUSE_ELE_NOT_EXIST                0x63
/** 100: invalid information element content */
#define TCS_BIN_CAUSE_INVALID_INF_ELE              0x64
/** 101: message not compatible with call state */
#define TCS_BIN_CAUSE_MESS_NOT_COMPATIBLE_1        0x65
/** 102: recovery on timer expiry */
#define TCS_BIN_CAUSE_RECOVERY_TIMER_EX            0x66
/** 103: parameter non-existent or not implemented */
#define TCS_BIN_CAUSE_PARAMETER_NON_EX_OR_NOT_IMPL 0x67
/** 110: message with unrecognized parameter discarded */
#define TCS_BIN_CAUSE_MESS_UNKNOWN_PAR_DISCARDED   0x6E
/** 111: protocol error, unspecified */
#define TCS_BIN_CAUSE_PROTOCOL_ERR                 0x6F
/** 127: interworking, unspecified */
#define TCS_BIN_CAUSE_INTERWORKING                 0x7F

#define TCS_BIN_NO_CAUSE                      0x00
#define TCS_BIN_ERR_NORESOURCES               0x80
#define TCS_BIN_ERR_ILL_PARAMETER             0x81
#define TCS_BIN_ERR_BAD_STATE                 0x82
#define TCS_BIN_ERR_INV_CONNECTION            0x83
#define TCS_BIN_ERR_TIMEOUT                   0x84
#define TCS_BIN_ERR_TOO_MUCH_GW_ADDR          0x85

/** SECMAN error codes */
#define SECMAN_SUCCESS                        0
#define SECMAN_ERR_NOENTRY                    1      /**< tried to remove entry, but entry was not found / identified */
#define SECMAN_ERR_DBFULL                     2      /**< tried to insert table of security settings is full */
#define SECMAN_ERR_INVALID_PARAMETER          3      /**< tried to use invalid parameters in reqmessage */
#define SECMAN_ERR_LINKKEY_NOT_SUFFICIENT     4      /**< linkkey is not sufficient for this connection */
#define SECMAN_ERR_LE_BD_NOT_RESOLVED         5      /**< random resolvable address not found */

/** MPA error codes */
#define MPA_STATUS_SUCCESS                     0
#define MPA_STATUS_NO_RESOURCES                (MPA_ERR | 1)
#define MPA_STATUS_SERVICEID_NOT_ALLOWED       (MPA_ERR | 2)    /**< enable req sent with serviceID unequal 0    */
#define MPA_STATUS_SERVICEID_NOT_FOUND         (MPA_ERR | 3)
#define MPA_STATUS_SERVICE_ALREADY_ENABLED     (MPA_ERR | 4)    /**< try to enable an already enabled profile    */
/** #define MPA_STATUS_SERVICE_ALREADY_DISABLED    (MPA_ERR | 5)  try to disable an already disabled profile  */
#define MPA_STATUS_DEVICE_REMOVED              (MPA_ERR | 6)
#define MPA_STATUS_PAIRING_TIMEOUT             (MPA_ERR | 7)
#define MPA_STATUS_PAIRING_PIN_REJECT          (MPA_ERR | 8)
#define MPA_STATUS_UNKNOWN_PROFILE_ID          (MPA_ERR | 9)
#define MPA_STATUS_PROFILE_ALREADY_CONNECTED   (MPA_ERR | 10)
#define MPA_STATUS_NO_BONDED_DEVICE_FOUND      (MPA_ERR | 11)
#define MPA_STATUS_NO_SERVER_CHANNEL_FOUND     (MPA_ERR | 12)
#define MPA_STATUS_NO_SEND_RIGHTS              (MPA_ERR | 13)
#define MPA_STATUS_APPLICATIONID_NOT_FOUND     (MPA_ERR | 14)
#define MPA_STATUS_APPLICATIONID_NOT_ALLOWED   (MPA_ERR | 15)
#define MPA_STATUS_NOT_IMPLEMENTED             (MPA_ERR | 16)
#define MPA_STATUS_UNSUPPORTED_PARAMETER       (MPA_ERR | 17)
#define MPA_STATUS_UNSUPPORTED_FEATURE         (MPA_ERR | 18)
#define MPA_STATUS_HFP_HAYES_ERROR             (MPA_ERR | 19)
#define MPA_STATUS_HFP_HAYES_TIMEOUT           (MPA_ERR | 20)
#define MPA_STATUS_WRONG_STATE                 (MPA_ERR | 21)
#define MPA_STATUS_WRONG_CHANNELID             (MPA_ERR | 22)
#define MPA_STATUS_WRONG_PROFILE_ID            (MPA_ERR | 23)
#define MPA_STATUS_A2DP_WRONG_NUMBER_OF_CHANNELS (MPA_ERR | 24)
#define MPA_STATUS_A2DP_WRONG_SAMPLING_RATE    (MPA_ERR | 25)
#define MPA_STATUS_ILLEGAL_VERSION             (MPA_ERR | 26)
#define MPA_STATUS_QOS_CANCELED                (MPA_ERR | 27)
#define MPA_STATUS_GENERIC_TIMEOUT             (MPA_ERR | 28)

/** BlueSecure error codes */
#define BTSEC_SUCCESS                          0
#define BTSEC_ERR_UNSPECIFIED                  (BLUESECURE_ERR| 1)
#define BTSEC_ERR_ILLEGAL_COMMAND              (BLUESECURE_ERR| 2)
#define BTSEC_ERR_ACTION_FAILED                (BLUESECURE_ERR| 3)
#define BTSEC_ERR_INVALID_PARAMETER            (BLUESECURE_ERR| 4)
#define BTSEC_ERR_BUSY                         (BLUESECURE_ERR| 5)
#define BTSEC_ERR_RESOURCE_ERROR               (BLUESECURE_ERR| 6)
#define BTSEC_ERR_FEATURE_NOT_IMPLEMENTED      (BLUESECURE_ERR| 7)
#define BTSEC_ERR_READ_ONLY                    (BLUESECURE_ERR| 8)
#define BTSEC_ERR_VOTING_FAILED                (BLUESECURE_ERR| 9)
#define BTSEC_ERR_ITEM_NOT_FOUND               (BLUESECURE_ERR|10)
#define BTSEC_ERR_ITEM_EXISTS                  (BLUESECURE_ERR|11)


/** @defgroup APP_Error APP Error Codes
  * @brief APP Error Codes
  * @{
  */
#define APP_SUCCESS                             0
#define APP_ERR_PENDING                         1      /**< @brief app procedure pending */
#define APP_ERR_HAVE_CONFIRM                    2      /**< @brief confirm has done by app */
#define APP_ERR_ACCEPT                          3      /**< @brief accept some request from lower layer */
#define APP_ERR_REJECT                          4      /**< @brief reject some request from lower layer */

/** @defgroup TAppResult TAppResult
  * @brief  profile procedure application return results
  * @{
  */
typedef enum
{
    AppResult_Success         = (APP_SUCCESS),
    AppResult_Pending         = (APP_ERR | APP_ERR_PENDING),
    //for prepare write & execute write
    AppResult_PreWrQueueFull  = (ATT_ERR | ATT_ERR_PREPARE_QUEUE_FULL),
    AppResult_AplicationErr   = (ATT_ERR | ATT_ERR_MIN_APPLIC_CODE),
    AppResult_InvalidOffset   = (ATT_ERR | ATT_ERR_INVALID_OFFSET),
    AppResult_InvalidValueSize = (ATT_ERR | ATT_ERR_INVALID_VALUE_SIZE),
    AppResult_InvalidParameter = (ATT_ERR | ATT_ERR_INVALID_PDU),
    AppResult_HaveConfirm      = (APP_ERR| APP_ERR_HAVE_CONFIRM),
    AppResult_Accept           = (APP_ERR| APP_ERR_ACCEPT),
    AppResult_Reject           = (APP_ERR| APP_ERR_REJECT),
    AppResult_UnknownError
} TAppResult;
/** @} End of TAppResult */
/** @} End of APP_Error */

/** @} End of Sub_Cause */
#endif /**< __BTERRCOD_H */
