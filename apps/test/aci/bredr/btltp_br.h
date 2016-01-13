#ifndef  _BTLTP_BR_H_
#define  _BTLTP_BR_H_

//#include <string.h>
//#include <blueapi_types.h>
#include <ltplib.h>
#include "btltp.h"
//#include "aci_if.h"
//#include "gatt.h"

/*--------------------------------------------------------------------------*/
/* LTP_ACL_BD_RESOLVED_INFO */
#define LTP_ACL_BD_RESOLVED_INFO                                          0x40
#define LTP_ACL_BD_RESOLVED_INFO_LENGTH                                     18
#define LTP_ACL_BD_RESOLVED_INFO_FLAGS                           (LTP_MDC_MSG)

/*--------------------------------------------------------------------------*/
/* LTP_AUTH_DELETE_RSP                                                      */
#define LTP_AUTH_DELETE_RSP                                               0x41
#define LTP_AUTH_DELETE_RSP_LENGTH                                          11
#define LTP_AUTH_DELETE_RSP_FLAGS                                (LTP_MDC_MSG)

#define LTP_AUTH_DELETE_RSP_OPT_MASK_BD_TYPE                              0x01

/*--------------------------------------------------------------------------*/
/* LTP_AUTH_LIST_RSP                                                        */
#define LTP_AUTH_LIST_RSP                                                 0x42
#define LTP_AUTH_LIST_RSP_LENGTH                                            11
#define LTP_AUTH_LIST_RSP_FLAGS                                  (LTP_MDC_MSG)

#define LTP_AUTH_LIST_RSP_OPT_MASK_BD_TYPE                                0x01

/*--------------------------------------------------------------------------*/
/* LTP_AUTH_LIST_INFO                                                       */
#define LTP_AUTH_LIST_INFO                                                0x43
#define LTP_AUTH_LIST_INFO_LENGTH                                           15
#define LTP_AUTH_LIST_INFO_FLAGS                 (LTP_VAR_LEN_MSG|LTP_MDC_MSG)

#define LTP_AUTH_LIST_INFO_OPT_MASK_BD_TYPE                               0x01

/*--------------------------------------------------------------------------*/
/* CONNECT_MDL_RSP                                                          */
#define LTP_CONNECT_MDL_RSP                                               0x44
#define LTP_CONNECT_MDL_RSP_LENGTH                                          12
#define LTP_CONNECT_MDL_RSP_FLAGS                                (LTP_MDC_MSG)

#define LTP_CONNECT_MDL_RSP_OPT_MASK_LOC_MDL_ID                           0x01
#define LTP_CONNECT_MDL_RSP_OPT_MASK_LOC_MDEP_ID                          0x02

/*--------------------------------------------------------------------------*/
/* AUTH_REQUEST_CNF                                                         */
#define LTP_AUTH_REQUEST_CNF                                              0x45
#define LTP_AUTH_REQUEST_CNF_LENGTH                                         11
#define LTP_AUTH_REQUEST_CNF_FLAGS   (LTP_CNF_MSG|LTP_VAR_LEN_MSG|LTP_MDH_MSG)

/*--------------------------------------------------------------------------*/
/* RELEASE_MDEP_RSP                                                         */
#define LTP_RELEASE_MDEP_RSP                                              0x46
#define LTP_RELEASE_MDEP_RSP_LENGTH                                          5
#define LTP_RELEASE_MDEP_RSP_FLAGS                               (LTP_MDC_MSG)

#define LTP_RELEASE_MDEP_RSP_OPT_MASK_MDEP_HANDLE                         0x01

/*--------------------------------------------------------------------------*/
/* INQUIRY_RSP                                                              */
#define LTP_INQUIRY_RSP                                                   0x47
#define LTP_INQUIRY_RSP_LENGTH                                               5
#define LTP_INQUIRY_RSP_FLAGS                                    (LTP_MDC_MSG)

#define LTP_INQUIRY_RSP_OPT_MASK_CANCEL_INQUIRY                           0x01

/*--------------------------------------------------------------------------*/
/* INQUIRY_DEVICE_INFO                                                      */
#define LTP_INQUIRY_DEVICE_INFO                                           0x48
#define LTP_INQUIRY_DEVICE_INFO_LENGTH                                      10
#define LTP_INQUIRY_DEVICE_INFO_FLAGS            (LTP_VAR_LEN_MSG|LTP_MDC_MSG)

#define LTP_INQUIRY_DEVICE_INFO_OPT_MASK_DEVICE_CLASS                     0x07
#define LTP_INQUIRY_DEVICE_INFO_OPT_MASK_RSSI                             0x08


/*--------------------------------------------------------------------------*/
/* DID_DEVICE_INFO                                                          */
#define LTP_DID_DEVICE_INFO                                               0x49
#define LTP_DID_DEVICE_INFO_LENGTH                                          16
#define LTP_DID_DEVICE_INFO_FLAGS                (LTP_VAR_LEN_MSG|LTP_MDC_MSG)

#define LTP_DID_DEVICE_INFO_OPT_MASK_VENDOR_ID_SOURCE                     0x03

/*--------------------------------------------------------------------------*/
/* LTP_AUTH_RSP                                                             */
#define LTP_AUTH_RSP                                                      0x4A
#define LTP_AUTH_RSP_LENGTH                                                 11
#define LTP_AUTH_RSP_FLAGS                                       (LTP_MDC_MSG)

/*--------------------------------------------------------------------------*/
/* 0x1B AUTH_RESULT_REQUEST_CNF                                             */
#define LTP_AUTH_RESULT_REQUEST_CNF                                       0x4B
#define LTP_AUTH_RESULT_REQUEST_CNF_LENGTH                                  28
#define LTP_AUTH_RESULT_REQUEST_CNF_FLAGS                        (LTP_MDH_MSG)

/*--------------------------------------------------------------------------*/
/* LTP_USER_CONF_REQUEST_CNF                                                */
#define LTP_USER_CONF_REQUEST_CNF                                         0x4C
#define LTP_USER_CONF_REQUEST_CNF_LENGTH                                    11
#define LTP_USER_CONF_REQUEST_CNF_FLAGS              (LTP_CNF_MSG|LTP_MDH_MSG)

/*--------------------------------------------------------------------------*/
/* LTP_AUTH_RESULT_CNF                                                      */
#define LTP_AUTH_RESULT_CNF                                               0x4D
#define LTP_AUTH_RESULT_CNF_LENGTH                                          15
#define LTP_AUTH_RESULT_CNF_FLAGS                    (LTP_CNF_MSG|LTP_MDH_MSG)

/*--------------------------------------------------------------------------*/
/* LTP_CONFIG_TUNNEL_RSP                                                    */
#define LTP_CONFIG_TUNNEL_RSP                                             0x4E
#define LTP_CONFIG_TUNNEL_RSP_LENGTH                                         5
#define LTP_CONFIG_TUNNEL_RSP_FLAGS                              (LTP_MDC_MSG)

/*--------------------------------------------------------------------------*/
/* LTP_CONFIG_TUNNEL_INFO                                                   */
#define LTP_CONFIG_TUNNEL_INFO                                            0x4F
#define LTP_CONFIG_TUNNEL_INFO_LENGTH                                        4
#define LTP_CONFIG_TUNNEL_INFO_FLAGS             (LTP_VAR_LEN_MSG|LTP_MDC_MSG)

/*--------------------------------------------------------------------------*/
/* LTP_RADIO_MODE_SET_RSP                                                   */
#define LTP_RADIO_MODE_SET_RSP                                            0x50
#define LTP_RADIO_MODE_SET_RSP_LENGTH                                        5
#define LTP_RADIO_MODE_SET_RSP_FLAGS                             (LTP_MDC_MSG)

/*--------------------------------------------------------------------------*/
/* LTP_KEYPRESS_NOTIFICATION_RSP                                            */
#define LTP_KEYPRESS_NOTIFICATION_RSP                                     0x51
#define LTP_KEYPRESS_NOTIFICATION_RSP_LENGTH                                 5
#define LTP_KEYPRESS_NOTIFICATION_RSP_FLAGS                      (LTP_MDC_MSG)

/*--------------------------------------------------------------------------*/
/* LTP_KEYPRESS_NOTIFICATION_INFO                                           */
#define LTP_KEYPRESS_NOTIFICATION_INFO                                    0x52
#define LTP_KEYPRESS_NOTIFICATION_INFO_LENGTH                               11
#define LTP_KEYPRESS_NOTIFICATION_INFO_FLAGS                     (LTP_MDC_MSG)

/*--------------------------------------------------------------------------*/
/* LTP_LOCAL_OOB_RSP                                                        */
#define LTP_LOCAL_OOB_RSP                                                 0x53
#define LTP_LOCAL_OOB_RSP_LENGTH                                            37
#define LTP_LOCAL_OOB_RSP_FLAGS                                  (LTP_MDC_MSG)

/*--------------------------------------------------------------------------*/
/* LTP_DEVICE_NAME_RSP                                                      */
#define LTP_DEVICE_NAME_RSP                                               0x54
#define LTP_DEVICE_NAME_RSP_LENGTH                                          11
#define LTP_DEVICE_NAME_RSP_FLAGS                (LTP_VAR_LEN_MSG|LTP_MDC_MSG)

/*--------------------------------------------------------------------------*/
/* SPP_DISCOVERY_RSP                                                        */
#define LTP_SPP_DISCOVERY_RSP                                             0x55
#define LTP_SPP_DISCOVERY_RSP_LENGTH                                         5
#define LTP_SPP_DISCOVERY_RSP_FLAGS                              (LTP_MDC_MSG)

/*--------------------------------------------------------------------------*/
/* SPP_ENDPOINT_INFO                                                        */
#define LTP_SPP_ENDPOINT_INFO                                             0x56
#define LTP_SPP_ENDPOINT_INFO_LENGTH                                        13
#define LTP_SPP_ENDPOINT_INFO_FLAGS              (LTP_VAR_LEN_MSG|LTP_MDC_MSG)

/*--------------------------------------------------------------------------*/
/* REGISTER_HDP_MDEP_RSP                                                    */
#define LTP_REGISTER_SPP_MDEP_RSP                                         0x57
#define LTP_REGISTER_SPP_MDEP_RSP_LENGTH                                     6
#define LTP_REGISTER_SPP_MDEP_RSP_FLAGS                          (LTP_MDC_MSG)

#define LTP_REGISTER_SPP_MDEP_RSP_OPT_MASK_MDEP_ID                        0x01
#define LTP_REGISTER_SPP_MDEP_RSP_OPT_MASK_DATA_TYPE                      0x06

/*--------------------------------------------------------------------------*/
/* AUTHORIZATION_REQ_CNF                                                    */
#define LTP_AUTHORIZATION_REQ_CNF                                         0x58
#define LTP_AUTHORIZATION_REQ_CNF_LENGTH                                    11
#define LTP_AUTHORIZATION_REQ_CNF_FLAGS              (LTP_CNF_MSG|LTP_MDH_MSG)

/*--------------------------------------------------------------------------*/
/* LTP_DEVICE_CONFIG_DEVICE_SET_RSP                                         */
#define LTP_DEVICE_CONFIG_DEVICE_SET_RSP                                  0x59
#define LTP_DEVICE_CONFIG_DEVICE_SET_RSP_LENGTH                              5
#define LTP_DEVICE_CONFIG_DEVICE_SET_RSP_FLAGS                   (LTP_MDC_MSG)

/*--------------------------------------------------------------------------*/
/* LTP_DEVICE_CONFIG_DID_SET_RSP                                            */
#define LTP_DEVICE_CONFIG_DID_SET_RSP                                     0x5A
#define LTP_DEVICE_CONFIG_DID_SET_RSP_LENGTH                                 5
#define LTP_DEVICE_CONFIG_DID_SET_RSP_FLAGS                      (LTP_MDC_MSG)

/*--------------------------------------------------------------------------*/
/* LTP_DEVICE_CONFIG_SPP_SET_RSP                                            */
#define LTP_DEVICE_CONFIG_SPP_SET_RSP                                     0x5B
#define LTP_DEVICE_CONFIG_SPP_SET_RSP_LENGTH                                 5
#define LTP_DEVICE_CONFIG_SPP_SET_RSP_FLAGS                      (LTP_MDC_MSG)

/*--------------------------------------------------------------------------*/
/* LTP_DEVICE_CONFIG_SPP_PAGESCAN_RSP                                       */
#define LTP_DEVICE_CONFIG_PAGESCAN_SET_RSP                                0x5C
#define LTP_DEVICE_CONFIG_PAGESCAN_SET_RSP_LENGTH                            5
#define LTP_DEVICE_CONFIG_PAGESCAN_SET_RSP_FLAGS                 (LTP_MDC_MSG)

/*--------------------------------------------------------------------------*/
/* LTP_DEVICE_CONFIG_LINKPOLICY_SET_RSP                                     */
#define LTP_DEVICE_CONFIG_LINKPOLICY_SET_RSP                              0x5D
#define LTP_DEVICE_CONFIG_LINKPOLICY_SET_RSP_LENGTH                          5
#define LTP_DEVICE_CONFIG_LINKPOLICY_SET_RSP_FLAGS               (LTP_MDC_MSG)

/*--------------------------------------------------------------------------*/
/* LTP_DEVICE_CONFIG_MAXTXPOWER_SET_RSP                                     */
#define LTP_DEVICE_CONFIG_MAXTXPOWER_SET_RSP                              0x5E
#define LTP_DEVICE_CONFIG_MAXTXPOWER_SET_RSP_LENGTH                          5
#define LTP_DEVICE_CONFIG_MAXTXPOWER_SET_RSP_FLAGS               (LTP_MDC_MSG)

#define LTP_DATA_OPT_MASK_LOC_MDL_ID                                      0x01
#define LTP_DATA_OPT_MASK_RETURN_CREDITS                                  0x02

/*--------------------------------------------------------------------------*/
/* DATA_UNSEGMENTED                                                         */
#define LTP_DATA_UNSEGMENTED                                              0x5F
#define LTP_DATA_UNSEGMENTED_LENGTH                                          4
#define LTP_DATA_UNSEGMENTED_FLAGS   (LTP_VAR_LEN_MSG|LTP_MDC_MSG|LTP_MDH_MSG)

#define LTP_DATA_UNSEGMENTED_OPT_MASK_LOC_MDL_ID                          LTP_DATA_OPT_MASK_LOC_MDL_ID
#define LTP_DATA_UNSEGMENTED_OPT_MASK_RETURN_CREDITS                      LTP_DATA_OPT_MASK_RETURN_CREDITS

/*--------------------------------------------------------------------------*/
/* DATA_START_SEGMENT                                                       */
#define LTP_DATA_START_SEGMENT                                            0x60
#define LTP_DATA_START_SEGMENT_LENGTH                                        6
#define LTP_DATA_START_SEGMENT_FLAGS (LTP_VAR_LEN_MSG|LTP_MDC_MSG|LTP_MDH_MSG)

#define LTP_DATA_START_SEGMENT_OPT_MASK_LOC_MDL_ID                        LTP_DATA_OPT_MASK_LOC_MDL_ID
#define LTP_DATA_START_SEGMENT_OPT_MASK_RETURN_CREDITS                    LTP_DATA_OPT_MASK_RETURN_CREDITS

/*--------------------------------------------------------------------------*/
/* DATA_END_SEGMENT                                                         */
#define LTP_DATA_END_SEGMENT                                              0x61
#define LTP_DATA_END_SEGMENT_LENGTH                                          4
#define LTP_DATA_END_SEGMENT_FLAGS  (LTP_VAR_LEN_MSG|LTP_MDC_MSG|LTP_MDH_MSG)

#define LTP_DATA_END_SEGMENT_OPT_MASK_LOC_MDL_ID                          LTP_DATA_OPT_MASK_LOC_MDL_ID
#define LTP_DATA_END_SEGMENT_OPT_MASK_RETURN_CREDITS                      LTP_DATA_OPT_MASK_RETURN_CREDITS

/*--------------------------------------------------------------------------*/
/* DATA_CONTINUE_SEGMENT                                                    */
#define LTP_DATA_CONTINUE_SEGMENT                                         0x62
#define LTP_DATA_CONTINUE_SEGMENT_LENGTH                                     4
#define LTP_DATA_CONTINUE_SEGMENT_FLAGS (LTP_VAR_LEN_MSG|LTP_MDC_MSG|LTP_MDH_MSG)

#define LTP_DATA_CONTINUE_SEGMENT_OPT_MASK_LOC_MDL_ID                     LTP_DATA_OPT_MASK_LOC_MDL_ID
#define LTP_DATA_CONTINUE_SEGMENT_OPT_MASK_RETURN_CREDITS                 LTP_DATA_OPT_MASK_RETURN_CREDITS

/*--------------------------------------------------------------------------*/
/* GATT_SDP_DISCOVERY_RSP */
#define LTP_GATT_SDP_DISCOVERY_RSP                                        0x63
#define LTP_GATT_SDP_DISCOVERY_RSP_LENGTH                                    5
#define LTP_GATT_SDP_DISCOVERY_RSP_FLAGS                         (LTP_MDC_MSG)

/*--------------------------------------------------------------------------*/
/* GATT_SDP_DISCOVERY_INFO */
#define LTP_GATT_SDP_DISCOVERY_INFO                                       0x64
#define LTP_GATT_SDP_DISCOVERY_INFO_LENGTH                                  16
#define LTP_GATT_SDP_DISCOVERY_INFO_FLAGS                        (LTP_MDC_MSG)


/*--------------------------------------------------------------------------*/
/* LTP_ACL_CONFIG_LINKPOLICY_RSP */
#define LTP_ACL_CONFIG_LINKPOLICY_RSP                                     0x65
#define LTP_ACL_CONFIG_LINKPOLICY_RSP_LENGTH                                11
#define LTP_ACL_CONFIG_LINKPOLICY_RSP_FLAGS                      (LTP_MDC_MSG)

/*--------------------------------------------------------------------------*/
/* LTP_ACL_CONFIG_SNIFFMODE_RSP */
#define LTP_ACL_CONFIG_SNIFFMODE_RSP                                      0x66
#define LTP_ACL_CONFIG_SNIFFMODE_RSP_LENGTH                                 11
#define LTP_ACL_CONFIG_SNIFFMODE_RSP_FLAGS                       (LTP_MDC_MSG)

/*--------------------------------------------------------------------------*/
/* LTP_ACL_SNIFF_SUBRATE_INFO */
#define LTP_ACL_SNIFF_SUBRATE_INFO                                        0x67
#define LTP_ACL_SNIFF_SUBRATE_INFO_LENGTH                                   18
#define LTP_ACL_SNIFF_SUBRATE_INFO_FLAGS                         (LTP_MDC_MSG)

/*--------------------------------------------------------------------------*/
/* LTP_ACL_CONFIG_LINKSTATUS_RSP */
#define LTP_ACL_CONFIG_LINKSTATUS_RSP                                     0x68
#define LTP_ACL_CONFIG_LINKSTATUS_RSP_LENGTH                                12
#define LTP_ACL_CONFIG_LINKSTATUS_RSP_FLAGS                      (LTP_MDC_MSG)

/*--------------------------------------------------------------------------*/
/* LTP_ACL_LINKSTATUS_INFO */
#define LTP_ACL_LINKSTATUS_INFO                                           0x69
#define LTP_ACL_LINKSTATUS_INFO_LENGTH                                      17
#define LTP_ACL_LINKSTATUS_INFO_FLAGS                            (LTP_MDC_MSG)

/*--------------------------------------------------------------------------*/
/* LTP_REMOTE_OOB_REQUEST_CNF                                               */
#define LTP_LEGACY_REMOTE_OOB_REQUEST_CNF                                 0x6A
#define LTP_LEGACY_REMOTE_OOB_REQUEST_CNF_LENGTH                            43
#define LTP_LEGACY_REMOTE_OOB_REQUEST_CNF_FLAGS       (LTP_CNF_MSG|LTP_MDH_MSG)

/*--------------------------------------------------------------------------*/
/* 0xC0 reserved (LTP_ACL_BD_RESOLVED_INFO)                                        */

/*--------------------------------------------------------------------------*/
/* LTP_AUTH_DELETE_REQ                                                      */
#define LTP_AUTH_DELETE_REQ                                               0xC1
#define LTP_AUTH_DELETE_REQ_LENGTH                                          10
#define LTP_AUTH_DELETE_REQ_FLAGS                                (LTP_MDH_MSG)

#define LTP_AUTH_DELETE_REQ_OPT_MASK_BD_TYPE                              0x01

/*--------------------------------------------------------------------------*/
/* LTP_AUTH_LIST_REQ                                                        */
#define LTP_AUTH_LIST_REQ                                                 0xC2
#define LTP_AUTH_LIST_REQ_LENGTH                                            10
#define LTP_AUTH_LIST_REQ_FLAGS                                  (LTP_MDH_MSG)

#define LTP_AUTH_LIST_REQ_OPT_MASK_BD_TYPE                                0x01

/*--------------------------------------------------------------------------*/
/* 0xC3 reserved (LTP_AUTH_LIST_INFO)                                       */

/* CONNECT_MDL_REQ                                                          */
#define LTP_CONNECT_MDL_REQ                                               0xC4
#define LTP_CONNECT_MDL_REQ_LENGTH                                          15
#define LTP_CONNECT_MDL_REQ_FLAGS                                (LTP_MDH_MSG)

//#define LTP_CONNECT_MDL_REQ_OPT_MASK_LINK_TYPE                            0x01		//Commented by SerialTester
#define LTP_CONNECT_MDL_REQ_OPT_MASK_LOC_MDEP_ID                          0x01

/*--------------------------------------------------------------------------*/
/* AUTH_REQUEST_IND                                                         */
#define LTP_AUTH_REQUEST_IND                                              0xC5
#define LTP_AUTH_REQUEST_IND_LENGTH                                         10
#define LTP_AUTH_REQUEST_IND_FLAGS                               (LTP_MDC_MSG)

/*--------------------------------------------------------------------------*/
/* RELEASE_MDEP_REQ                                                         */
#define LTP_RELEASE_MDEP_REQ                                              0xC6
#define LTP_RELEASE_MDEP_REQ_LENGTH                                          5
#define LTP_RELEASE_MDEP_REQ_FLAGS                               (LTP_MDH_MSG)

/*--------------------------------------------------------------------------*/
/* INQUIRY_REQ                                                              */
#define LTP_INQUIRY_REQ                                                   0xC7
#define LTP_INQUIRY_REQ_LENGTH                                               4
#define LTP_INQUIRY_REQ_FLAGS                                    (LTP_MDH_MSG)

#define LTP_INQUIRY_REQ_OPT_MASK_LIMITED_INQUIRY                          0x01
#define LTP_INQUIRY_REQ_OPT_MASK_CANCEL_INQUIRY                           0x02

/*--------------------------------------------------------------------------*/
/* 0xC8 reserved (LTP_INQUIRY_DEVICE_INFO)                                  */

/*--------------------------------------------------------------------------*/
/* 0xC9 reserved (LTP_DID_DEVICE_INFO)                                  */

/*--------------------------------------------------------------------------*/
/* LTP_AUTH_REQ                                                             */
#define LTP_AUTH_REQ                                                      0xCA
#define LTP_AUTH_REQ_LENGTH                                                 10
#define LTP_AUTH_REQ_FLAGS                                       (LTP_MDH_MSG)

/*--------------------------------------------------------------------------*/
/* 0x9B AUTH_RESULT_REQUEST_IND                                             */
#define LTP_AUTH_RESULT_REQUEST_IND                                       0xCB
#define LTP_AUTH_RESULT_REQUEST_IND_LENGTH                                  10
#define LTP_AUTH_RESULT_REQUEST_IND_FLAGS                        (LTP_MDC_MSG)

/*--------------------------------------------------------------------------*/
/* LTP_USER_CONF_REQUEST_IND                                                */
#define LTP_USER_CONF_REQUEST_IND                                         0xCC
#define LTP_USER_CONF_REQUEST_IND_LENGTH                                    14
#define LTP_USER_CONF_REQUEST_IND_FLAGS                          (LTP_MDC_MSG)

/*--------------------------------------------------------------------------*/
/* LTP_AUTH_RESULT_IND                                                      */
#define LTP_AUTH_RESULT_IND                                               0xCD
#define LTP_AUTH_RESULT_IND_LENGTH                                          32
#define LTP_AUTH_RESULT_IND_FLAGS                                (LTP_MDC_MSG)

/*--------------------------------------------------------------------------*/
/* LTP_CONFIG_TUNNEL_REQ                                                    */
#define LTP_CONFIG_TUNNEL_REQ                                             0xCE
#define LTP_CONFIG_TUNNEL_REQ_LENGTH                                         4
#define LTP_CONFIG_TUNNEL_REQ_FLAGS              (LTP_VAR_LEN_MSG|LTP_MDH_MSG)

/*--------------------------------------------------------------------------*/
/* 0xCF reserved (LTP_CONFIG_TUNNEL_INFO)                                         */

/*--------------------------------------------------------------------------*/
/* LTP_RADIO_MODE_SET_REQ                                                   */
#define LTP_RADIO_MODE_SET_REQ                                            0xD0
#define LTP_RADIO_MODE_SET_REQ_LENGTH                                        5
#define LTP_RADIO_MODE_SET_REQ_FLAGS                             (LTP_MDH_MSG)

#define LTP_RADIO_MODE_SET_REQ_OPT_MASK_LIMITED_DISCOVERABLE              0x01

/*--------------------------------------------------------------------------*/
/* LTP_KEYPRESS_NOTIFICATION_REQ                                            */
#define LTP_KEYPRESS_NOTIFICATION_REQ                                     0xD1
#define LTP_KEYPRESS_NOTIFICATION_REQ_LENGTH                                11
#define LTP_KEYPRESS_NOTIFICATION_REQ_FLAGS                      (LTP_MDH_MSG)

/*--------------------------------------------------------------------------*/
/* 0xD2 reserved (LTP_KEYPRESS_NOTIFICATION_INFO)                                 */

/*--------------------------------------------------------------------------*/
/* LTP_LOCAL_OOB_REQ                                                        */
#define LTP_LOCAL_OOB_REQ                                                 0xD3
#define LTP_LOCAL_OOB_REQ_LENGTH                                             4
#define LTP_LOCAL_OOB_REQ_FLAGS                                  (LTP_MDH_MSG)

/*--------------------------------------------------------------------------*/
/* LTP_DEVICE_NAME_REQ                                                      */
#define LTP_DEVICE_NAME_REQ                                               0xD4
#define LTP_DEVICE_NAME_REQ_LENGTH                                          10
#define LTP_DEVICE_NAME_REQ_FLAGS                                (LTP_MDH_MSG)

/*--------------------------------------------------------------------------*/
/* SPP_DISCOVERY_REQ                                                        */
#define LTP_SPP_DISCOVERY_REQ                                             0xD5
#define LTP_SPP_DISCOVERY_REQ_LENGTH                                        12
#define LTP_SPP_DISCOVERY_REQ_FLAGS                              (LTP_MDH_MSG)

#define LTP_SPP_DISCOVERY_REQ_OPT_MASK_DID_DISCOVERY                      0x01

/*--------------------------------------------------------------------------*/
/* 0xD6 reserved (LTP_SPP_ENDPOINT_INFO)                                 */

/*--------------------------------------------------------------------------*/
/* REGISTER_SPP_MDEP_REQ                                                    */
#define LTP_REGISTER_SPP_MDEP_REQ                                         0xD7
#define LTP_REGISTER_SPP_MDEP_REQ_LENGTH                                    11
#define LTP_REGISTER_SPP_MDEP_REQ_FLAGS          (LTP_VAR_LEN_MSG|LTP_MDH_MSG)

/*--------------------------------------------------------------------------*/
/* AUTHORIZATION_REQ_IND                                                    */
#define LTP_AUTHORIZATION_REQ_IND                                         0xD8
#define LTP_AUTHORIZATION_REQ_IND_LENGTH                                    10
#define LTP_AUTHORIZATION_REQ_IND_FLAGS                          (LTP_MDC_MSG)

/*--------------------------------------------------------------------------*/
/* LTP_DEVICE_CONFIG_DEVICE_SET_REQ                                         */
#define LTP_DEVICE_CONFIG_DEVICE_SET_REQ                                  0xD9
#define LTP_DEVICE_CONFIG_DEVICE_SET_REQ_LENGTH                              7
#define LTP_DEVICE_CONFIG_DEVICE_SET_REQ_FLAGS   (LTP_VAR_LEN_MSG|LTP_MDH_MSG)

/*--------------------------------------------------------------------------*/
/* LTP_DEVICE_CONFIG_DID_SET_REQ                                            */
#define LTP_DEVICE_CONFIG_DID_SET_REQ                                     0xDA
#define LTP_DEVICE_CONFIG_DID_SET_REQ_LENGTH                                11
#define LTP_DEVICE_CONFIG_DID_SET_REQ_FLAGS                      (LTP_MDH_MSG)

/*--------------------------------------------------------------------------*/
/* LTP_DEVICE_CONFIG_SPP_SET_REQ                                            */
#define LTP_DEVICE_CONFIG_SPP_SET_REQ                                     0xDB
#define LTP_DEVICE_CONFIG_SPP_SET_REQ_LENGTH                                 8
#define LTP_DEVICE_CONFIG_SPP_SET_REQ_FLAGS                      (LTP_MDH_MSG)

/*--------------------------------------------------------------------------*/
/* LTP_DEVICE_CONFIG_SPP_PAGESCAN_REQ                                       */
#define LTP_DEVICE_CONFIG_PAGESCAN_SET_REQ                                0xDC
#define LTP_DEVICE_CONFIG_PAGESCAN_SET_REQ_LENGTH                           12
#define LTP_DEVICE_CONFIG_PAGESCAN_SET_REQ_FLAGS                 (LTP_MDH_MSG)

/*--------------------------------------------------------------------------*/
/* LTP_DEVICE_CONFIG_LINKPOLICY_SET_REQ                                     */
#define LTP_DEVICE_CONFIG_LINKPOLICY_SET_REQ                              0xDD
#define LTP_DEVICE_CONFIG_LINKPOLICY_SET_REQ_LENGTH                          8
#define LTP_DEVICE_CONFIG_LINKPOLICY_SET_REQ_FLAGS               (LTP_MDH_MSG)

/*--------------------------------------------------------------------------*/
/* LTP_DEVICE_CONFIG_MAXTXPOWER_SET_REQ                                     */
#define LTP_DEVICE_CONFIG_MAXTXPOWER_SET_REQ                              0xDE
#define LTP_DEVICE_CONFIG_MAXTXPOWER_SET_REQ_LENGTH                          5
#define LTP_DEVICE_CONFIG_MAXTXPOWER_SET_REQ_FLAGS               (LTP_MDH_MSG)

/*--------------------------------------------------------------------------*/
/* 0xDF-0xE2 reserved (LTP_DATA_UNSEGMENTED)                                 */

/*--------------------------------------------------------------------------*/
/* GATT_SDP_DISCOVERY_REQ */
#define LTP_GATT_SDP_DISCOVERY_REQ                                        0xE3
#define LTP_GATT_SDP_DISCOVERY_REQ_LENGTH                                   13
#define LTP_GATT_SDP_DISCOVERY_REQ_FLAGS                         (LTP_MDH_MSG)

/*--------------------------------------------------------------------------*/
/* 0xE4 reserved (LTP_GATT_SDP_DISCOVERY_INFO)                              */

/*--------------------------------------------------------------------------*/
/* LTP_ACL_CONFIG_LINKPOLICY_REQ */
#define LTP_ACL_CONFIG_LINKPOLICY_REQ                                     0xE5
#define LTP_ACL_CONFIG_LINKPOLICY_REQ_LENGTH                                14
#define LTP_ACL_CONFIG_LINKPOLICY_REQ_FLAGS                      (LTP_MDC_MSG)

/*--------------------------------------------------------------------------*/
/* LTP_ACL_CONFIG_SNIFFMODE_REQ */
#define LTP_ACL_CONFIG_SNIFFMODE_REQ                                      0xE6
#define LTP_ACL_CONFIG_SNIFFMODE_REQ_LENGTH                                 24
#define LTP_ACL_CONFIG_SNIFFMODE_REQ_FLAGS                       (LTP_MDC_MSG)

/*--------------------------------------------------------------------------*/
/* 0xE7 reserved (LTP_ACL_SNIFF_SUBRATE_INFO) */

/*--------------------------------------------------------------------------*/
/* LTP_ACL_CONFIG_LINKSTATUS_REQ */
#define LTP_ACL_CONFIG_LINKSTATUS_REQ                                     0xE8
#define LTP_ACL_CONFIG_LINKSTATUS_REQ_LENGTH                                13
#define LTP_ACL_CONFIG_LINKSTATUS_REQ_FLAGS                      (LTP_MDC_MSG)

/*--------------------------------------------------------------------------*/
/* 0xE9 reserved (LTP_ACL_LINKSTATUS_INFO) */

/*--------------------------------------------------------------------------*/
/* LTP_LEGACY_REMOTE_OOB_REQUEST_IND                                        */
#define LTP_LEGACY_REMOTE_OOB_REQUEST_IND                                 0xEA
#define LTP_LEGACY_REMOTE_OOB_REQUEST_IND_LENGTH                            10
#define LTP_LEGACY_REMOTE_OOB_REQUEST_IND_FLAGS                  (LTP_MDC_MSG)

typedef enum
{
	blueAPI_ApplicationDefined = 0,   /**< application provides services */
	blueAPI_ServiceDIS,                      /**< Device Information service    */
	blueAPI_ServiceGLS,                      /**< Glucose service               */
	blueAPI_ServiceBAS,                      /**< Battery service               */
	blueAPI_ServiceCTS,                      /**< Current Time service          */
	blueAPI_ServiceNDCS,                     /**< Next DST Change service       */
	blueAPI_ServiceRTUS,                     /**< Reference Time Update service */
	blueAPI_ServiceBLS,                      /**< Blood Pressure service        */
	blueAPI_ServiceHRS,                      /**< Heart Rate service            */

	/* the following services are not (yet) available as built-in services:   */
	blueAPI_ServiceCGM,                      /**< Continous Glucose Monitor service */
	blueAPI_ServiceBMS,                      /**< Bond Management service       */
	blueAPI_ServiceQUL,                      /**< Qualification test service    */
	blueAPI_ServiceTST                       /**< Test service                  */
} GATTServiceID;

PVOID blueAPI_GATTServiceGet(GATTServiceID serviceID, LPWORD pNbrOfAttrib);


BOOL LTPLibSendConnectMDLRsp(PLTPLib pLTPLib, uint8_t copmsk, uint8_t * pOpt, uint8_t cause, uint8_t * rem_BD);
BOOL LTPLibSendACLSniffSubrateInfo(PLTPLib pLTPLib, uint8_t copmsk, uint8_t * pOpt,
                                 uint8_t * rem_BD, uint16_t maxTxLatency, uint16_t maxRxLatency,
                                 uint16_t minRemoteTimeout, uint16_t minLocalTimeout
                                 );
BOOL LTPLibSendAuthResultInd(PLTPLib pLTPLib, uint8_t copmsk, uint8_t * pOpt, uint8_t cause, uint8_t * rem_BD, uint8_t * linkKey, uint8_t keyType, uint32_t appData);
void LTPHandle_BREDR_DeviceConfigSetRsp(PBTLtp pBTLtp, PBlueAPI_DeviceConfigSetRsp pCOM_DeviceConfigSetRsp);
void BTLTPHandleBREDRLtpMsg(PBTLtp pBTLtp, uint8_t command, uint8_t copmsk, uint8_t * pOpt, uint16_t lenPara, uint8_t * pPara);

void BTLTPHandleBREDRBLUE_API_MSG(PBTLtp pBTLtp, uint8_t * pBuffer, uint16_t offset);
BOOL LTPLibSendAuthResultRequestInd(PLTPLib pLTPLib, uint8_t copmsk, uint8_t * pOpt, uint8_t * rem_BD);

#endif
