/***************************************************************************
 Copyright (C) Mindtree Ltd.
 This module is a confidential and proprietary property of MindTree and
 a possession or use of this module requires written permission of MindTree.
 ***************************************************************************/
/**
 * \file
 * Contains Spec definitions for HCI Layer
 */
/** \addtogroup hci_external
 *  @{ */
#ifndef __HCI_SPEC_DEFINES_H__
#define __HCI_SPEC_DEFINES_H__


/* HCI Command OpCodes */
#define HCI_NULL_OPCODE                                     0x0000
/* Link Control Commands - OGF : 0x01 */
#define HCI_INQUIRY_OPCODE                                  0x0401
#define HCI_INQUIRY_CANCEL_OPCODE                           0x0402
#define HCI_PERIODIC_INQUIRY_MODE_OPCODE                    0x0403
#define HCI_EXIT_PERIODIC_INQUIRY_MODE_OPCODE               0x0404
#define HCI_CREATE_CONNECTION_OPCODE                        0x0405
#define HCI_DISCONNECT_OPCODE                               0x0406
#define HCI_ADD_SCO_CONNECTION_OPCODE                       0x0407
#define HCI_CREATE_CONNECTION_CANCEL_OPCODE                 0x0408
#define HCI_ACCEPT_CONNECTION_REQUEST_OPCODE                0x0409
#define HCI_REJECT_CONNECTION_REQUEST_OPCODE                0x040A
#define HCI_LINK_KEY_REQUEST_REPLY_OPCODE                   0x040B
#define HCI_LINK_KEY_REQUEST_NEGATIVE_REPLY_OPCODE          0x040C
#define HCI_PIN_CODE_REQUEST_REPLY_OPCODE                   0x040D
#define HCI_PIN_CODE_REQUEST_NEGATIVE_REPLY_OPCODE          0x040E
#define HCI_CHANGE_CONNECTION_PACKET_TYPE_OPCODE            0x040F
#define HCI_AUTHENTICATION_REQUESTED_OPCODE                 0x0411
#define HCI_SET_CONNECTION_ENCRYPTION_OPCODE                0x0413
#define HCI_CHANGE_CONNECTION_LINK_KEY_OPCODE               0x0415
#define HCI_MASTER_LINK_KEY_OPCODE                          0x0417
#define HCI_REMOTE_NAME_REQUEST_OPCODE                      0x0419
#define HCI_REMOTE_NAME_REQUEST_CANCEL_OPCODE               0x041A
#define HCI_READ_REMOTE_SUPPORTED_FEATURES_OPCODE           0x041B
#define HCI_READ_REMOTE_EXTENDED_FEATURES_OPCODE            0x041C
#define HCI_READ_REMOTE_VERSION_INFORMATION_OPCODE          0x041D
#define HCI_READ_CLOCK_OFFSET_OPCODE                        0x041F
#define HCI_READ_LMP_HANDLE_OPCODE                          0x0420
/* Version 1.2 Additional Link Control Commands */
#define HCI_SETUP_SYNCHRONOUS_CONNECTION_OPCODE             0x0428
#define HCI_ACCEPT_SYNCHRONOUS_CONNECTION_REQ_OPCODE        0x0429
#define HCI_REJECT_SYNCHRONOUS_CONNECTION_REQ_OPCODE        0x042A
/* Version 2.1 Additional Link Control Commands */
#define HCI_IO_CAPABILITY_REQUEST_REPLY_OPCODE              0x042B
#define HCI_USER_CONFIRMATION_REQUEST_REPLY_OPCODE          0x042C
#define HCI_USER_CONFIRMATION_REQUEST_NEGATIVE_REPLY_OPCODE 0x042D
#define HCI_USER_PASSKEY_REQUEST_REPLY_OPCODE				0x042E
#define HCI_USER_PASSKEY_REQUEST_NEGATIVE_REPLY_OPCODE      0x042F
#define HCI_REMOTE_OOB_DATA_REQUEST_REPLY_OPCODE            0x0430
#define HCI_REMOTE_OOB_DATA_REQUEST_NEGATIVE_REPLY_OPCODE   0x0433
#define HCI_IO_CAPABILITY_REQUEST_NEGATIVE_REPLY_OPCODE     0x0434

#ifdef VER_3_0
/* Version 3.0 Additional Link Control Commands */
#define HCI_CREATE_PHYSICAL_LINK_OPCODE                     0x0435
#define HCI_ACCEPT_PHYSICAL_LINK_OPCODE                     0x0436
#define HCI_DISCONNECT_PHYSICAL_LINK_OPCODE                 0x0437
#define HCI_CREATE_LOGICAL_LINK_OPCODE                      0x0438
#define HCI_ACCEPT_LOGICAL_LINK_OPCODE                      0x0439
#define HCI_DISCONNECT_LOGICAL_LINK_OPCODE                  0x043A
#define HCI_LOGICAL_LINK_CANCEL_OPCODE                      0x043B
#define HCI_FLOW_SPEC_MODIFY_OPCODE                         0x043C
#endif

#ifdef VER_CSA4
/* Version CSA4 Additional Link Control Commands */
#define HCI_TRUNCATED_PAGE_OPCODE                               0x043F
#define HCI_TRUNCATED_PAGE_CANCEL_OPCODE                        0x0440
#define HCI_SET_CONNECTIONLESS_SLAVE_BROADCAST_OPCODE           0x0441
#define HCI_SET_CONNECTIONLESS_SLAVE_BROADCAST_RECEIVE_OPCODE   0x0442
#define HCI_START_SYNCHRONIZATION_TRAIN_OPCODE                  0x0443
#define HCI_RECEIVE_SYNCHRONIZATION_TRAIN_OPCODE                0x0444
#endif
#ifdef _SUPPORT_SECURE_CONNECTION_
#define HCI_REMOTE_OOB_EXT_DATA_REQUEST_REPLY_OPCODE        0x0445
#endif

/* Link Policy Commands - OGF : 0x02 */
#define HCI_HOLD_MODE_OPCODE                                0x0801
#define HCI_SNIFF_MODE_OPCODE                               0x0803
#define HCI_EXIT_SNIFF_MODE_OPCODE                          0x0804
#define HCI_PARK_MODE_OPCODE                                0x0805
#define HCI_EXIT_PARK_MODE_OPCODE                           0x0806
#define HCI_QOS_SETUP_OPCODE                                0x0807
#define HCI_ROLE_DISCOVERY_OPCODE                           0x0809
#define HCI_SWITCH_ROLE_OPCODE                              0x080B
#define HCI_READ_LINK_POLICY_SETTNGS_OPCODE                 0x080C
#define HCI_WRITE_LINK_POLICY_SETTINGS_OPCODE               0x080D
#define HCI_READ_DEFAULT_LINK_POLICY_SETTNGS_OPCODE         0x080E
#define HCI_WRITE_DEFAULT_LINK_POLICY_SETTINGS_OPCODE       0x080F
#define HCI_FLOW_SPECIFICATION_OPCODE                       0x0810
#define HCI_SNIFF_SUBRATING_OPCODE                          0x0811

/* Host Controller & BaseBand Commands - OGF : 0x03 */
#define HCI_SET_EVENT_MASK_OPCODE                           0x0C01
#define HCI_RESET_OPCODE                                    0x0C03
#define HCI_SET_EVENT_FILTER_OPCODE                         0x0C05
#define HCI_FLUSH_OPCODE                                    0x0C08
#define HCI_READ_PIN_TYPE_OPCODE                            0x0C09
#define HCI_WRITE_PIN_TYPE_OPCODE                           0x0C0A
#define HCI_CREATE_NEW_UNIT_KEY_OPCODE                      0x0C0B
#define HCI_READ_STORED_LINK_KEY_OPCODE                     0x0C0D
#define HCI_WRITE_STORED_LINK_KEY_OPCODE                    0x0C11
#define HCI_DELETE_STORED_LINK_KEY_OPCODE                   0x0C12
#define HCI_CHANGE_LOCAL_NAME_OPCODE                        0x0C13
#define HCI_READ_LOCAL_NAME_OPCODE                          0x0C14
#define HCI_READ_CONNECTION_ACCEPT_TIMEOUT_OPCODE           0x0C15
#define HCI_WRITE_CONNECTION_ACCEPT_TIMEOUT_OPCODE          0x0C16
#define HCI_READ_PAGE_TIMEOUT_OPCODE                        0x0C17
#define HCI_WRITE_PAGE_TIMEOUT_OPCODE                       0x0C18
#define HCI_READ_SCAN_ENABLE_OPCODE                         0x0C19
#define HCI_WRITE_SCAN_ENABLE_OPCODE                        0x0C1A
#define HCI_READ_PAGE_SCAN_ACTIVITY_OPCODE                  0x0C1B
#define HCI_WRITE_PAGE_SCAN_ACTIVITY_OPCODE                 0x0C1C
#define HCI_READ_INQUIRY_SCAN_ACTIVITY_OPCODE               0x0C1D
#define HCI_WRITE_INQUIRY_SCAN_ACTIVITY_OPCODE              0x0C1E
#define HCI_READ_AUTHENTICATION_ENABLE_OPCODE               0x0C1F
#define HCI_WRITE_AUTHENTICATION_ENABLE_OPCODE              0x0C20
#define HCI_READ_ENCRYPTION_MODE_OPCODE                     0x0C21
#define HCI_WRITE_ENCRYPTION_MODE_OPCODE                    0x0C22
#define HCI_READ_CLASS_OF_DEVICE_OPCODE                     0x0C23
#define HCI_WRITE_CLASS_OF_DEVICE_OPCODE                    0x0C24
#define HCI_READ_VOICE_SETTING_OPCODE                       0x0C25
#define HCI_WRITE_VOICE_SETTING_OPCODE                      0x0C26
#define HCI_READ_AUTOMATIC_FLUSH_TIMEOUT_OPCODE             0x0C27
#define HCI_WRITE_AUTOMATIC_FLUSH_TIMEOUT_OPCODE            0x0C28
#define HCI_READ_NUM_BROADCAST_RETRANSMISSIONS_OPCODE       0x0C29
#define HCI_WRITE_NUM_BROADCAST_RETRANSMISSIONS_OPCODE      0x0C2A
#define HCI_READ_HOLD_MODE_ACTIVITY_OPCODE                  0x0C2B
#define HCI_WRITE_HOLD_MODE_ACTIVITY_OPCODE                 0x0C2C
#define HCI_READ_TRANSMIT_POWER_LEVEL_OPCODE                0x0C2D
#define HCI_READ_SCO_FLOW_CONTROL_ENABLE_OPCODE             0x0C2E
#define HCI_WRITE_SCO_FLOW_CONTROL_ENABLE_OPCODE            0x0C2F
#define HCI_SET_CONTROLLER_TO_HOST_FLOW_CONTROL_OPCODE      0x0C31
#define HCI_HOST_BUFFER_SIZE_OPCODE                         0x0C33
#define HCI_HOST_NUMBER_OF_COMPLETED_PACKETS_OPCODE         0x0C35
#define HCI_READ_LINK_SUPERVISION_TIMEOUT_OPCODE            0x0C36
#define HCI_WRITE_LINK_SUPERVISION_TIMEOUT_OPCODE           0x0C37
#define HCI_READ_NUMBER_OF_SUPPORTED_IAC_OPCODE             0x0C38
#define HCI_READ_CURRENT_IAC_LAP_OPCODE                     0x0C39
#define HCI_WRITE_CURRENT_IAC_LAP_OPCODE                    0x0C3A
#define HCI_READ_PAGE_SCAN_PERIOD_MODE_OPCODE               0x0C3B
#define HCI_WRITE_PAGE_SCAN_PERIOD_MODE_OPCODE              0x0C3C
#define HCI_READ_PAGE_SCAN_MODE_OPCODE                      0x0C3D
#define HCI_WRITE_PAGE_SCAN_MODE_OPCODE                     0x0C3E
/* Version 1.2 Additional HC BB Commands */
#define HCI_SET_AFH_HOST_CHANNEL_CLASSIFICATION_OPCODE      0x0C3F
#define HCI_READ_INQUIRY_SCAN_TYPE_OPCODE                   0x0C42
#define HCI_WRITE_INQUIRY_SCAN_TYPE_OPCODE                  0x0C43
#define HCI_READ_INQUIRY_MODE_OPCODE                        0x0C44
#define HCI_WRITE_INQUIRY_MODE_OPCODE                       0x0C45
#define HCI_READ_PAGE_SCAN_TYPE_OPCODE                      0x0C46
#define HCI_WRITE_PAGE_SCAN_TYPE_OPCODE                     0x0C47
#define HCI_READ_AFH_CHANNEL_ASSESMENT_MODE_OPCODE          0x0C48
#define HCI_WRITE_AFH_CHANNEL_ASSESMENT_MODE_OPCODE         0x0C49
/* Version 2.1 Additional HC BB Commands */
#define HCI_READ_EXTENDED_INQUIRY_RESPONSE_OPCODE       	0x0C51
#define HCI_WRITE_EXTENDED_INQUIRY_RESPONSE_OPCODE       	0x0C52
#define HCI_REFRESH_ENCRYPTION_KEY_OPCODE			       	0x0C53
#define HCI_READ_SIMPLE_PAIRING_MODE_OPCODE			       	0x0C55
#define HCI_WRITE_SIMPLE_PAIRING_MODE_OPCODE		       	0x0C56
#define HCI_READ_LOCAL_OOB_DATA_OPCODE				       	0x0C57
#define HCI_READ_INQUIRY_RES_TRANS_POWER_LEVEL_OPCODE       0x0C58
#define HCI_WRITE_INQUIRY_TRANS_POWER_LEVEL_OPCODE			0x0C59
#define HCI_SEND_KEYPRESS_NOTIFICATION_OPCODE				0x0C60
#define HCI_READ_DEFAULT_ERRONEOUS_DATA_REPORTING_OPCODE	0x0C5A
#define HCI_WRITE_DEFAULT_ERRONEOUS_DATA_REPORTING_OPCODE	0x0C5B
#define HCI_ENHANCED_FLUSH_OPCODE				       		0x0C5F

#ifdef VER_3_0
/* Version 3.0 Additional HC BB Commands */
#define HCI_READ_LOGICAL_LINK_ACCEPT_TIMEOUT_OPCODE         0x0C61
#define HCI_WRITE_LOGICAL_LINK_ACCEPT_TIMEOUT_OPCODE        0x0C62
#define HCI_SET_EVENT_MASK_PAGE_2_OPCODE                    0x0C63
#define HCI_READ_LOCAL_DATA_OPCODE                          0x0C64
#define HCI_WRITE_LOCAL_DATA_OPCODE                         0x0C65
#define HCI_READ_FLOW_CONTROL_MODE_OPCODE                   0x0C66
#define HCI_WRITE_FLOW_CONTROL_MODE_OPCODE                  0x0C67
#define HCI_READ_ENHANCED_TRANSMIT_POWER_LEVEL_OPCODE       0x0C68
#define HCI_READ_BEST_EFFORT_FLUSH_TIMEOUT_OPCODE           0x0C69
#define HCI_WRITE_BEST_EFFORT_FLUSH_TIMEOUT_OPCODE          0x0C6A
#define HCI_SHORT_RANGE_MODE_OPCODE                         0x0C6B
#endif

#ifdef LE_MODE_EN
/* Version 4.0 Additional HC BB Commands */
#define HCI_READ_LE_HOST_SUPPORTED_COMMAND_OPCODE           0x0C6C
#define HCI_WRITE_LE_HOST_SUPPORTED_COMMAND_OPCODE          0x0C6D
#endif

#ifdef MWS_ENABLE
#define HCI_SET_MWS_CHANNEL_PATAMETERS_OPCODE               0x0C6E
#define HCI_SET_EXTERNAL_FRAME_CONFIGURATION_OPCODE         0x0C6F
#define HCI_SET_MWS_SIGNALING                               0x0C70
#define HCI_SET_MWS_TRANSPORT_LAYER                         0x0C71
#define HCI_SET_MWS_SCAN_FREQUENCY_TABLE                    0x0C72
#define HCI_SET_MWS_PATTERN_CONFIGURATION                   0x0C73
#endif

#ifdef VER_CSA4
/* Version CSA4 Additional HC BB Commands */
#define HCI_SET_RESERVED_LT_ADDR_OPCODE                     0x0C74
#define HCI_DELETE_RESERVED_LT_ADDR_OPCODE                  0x0C75
#define HCI_SET_CONNECTIONLESS_SLAVE_BROADCAST_DATA_OPCODE  0x0C76
#define HCI_READ_SYNCHRONIZATION_TRAIN_PARAMETERS_OPCODE    0x0C77
#define HCI_WRITE_SYNCHRONIZATION_TRAIN_PARAMETERS_OPCODE   0x0C78
#endif
#ifdef _SUPPORT_SECURE_CONNECTION_
#define HCI_READ_SECURE_CONN_HOST_SUPPORT_OPCODE            0x0C79
#define HCI_WRITE_SECURE_CONN_HOST_SUPPORT_OPCODE           0x0C7A
#define HCI_READ_AUTH_PAYLOAD_TIMEOUT_OPCODE                0x0C7B
#define HCI_WRITE_AUTH_PAYLOAD_TIMEOUT_OPCODE               0x0C7C
#define HCI_READ_LOCAL_OOB_EXT_DATA_OPCODE                  0x0C7D
#endif

/* Informational Parameters - OGF : 0x04 */
#define HCI_READ_LOCAL_VERSION_INFORMATION_OPCODE           0x1001
#define HCI_READ_LOCAL_SUPPORTED_COMMANDS_OPCODE            0x1002
#define HCI_READ_LOCAL_SUPPORTED_FEATURES_OPCODE            0x1003
#define HCI_READ_LOCAL_EXTENDED_FEATURES_OPCODE             0x1004
#define HCI_READ_BUFFER_SIZE_OPCODE                         0x1005
#define HCI_READ_COUNTRY_CODE_OPCODE                        0x1007
#define HCI_READ_BD_ADDR_OPCODE                             0x1009

#ifdef VER_3_0
/* Version 3.0 Additional Informational Commands */
#define HCI_READ_DATA_BLOCK_SIZE_OPCODE                     0x100A
#endif

/* Status Parameters - OGF : 0x05 */
#define HCI_READ_FAILED_CONTACT_COUNTER_OPCODE              0x1401
#define HCI_RESET_FAILED_CONTACT_COUNTER_OPCODE             0x1402
#define HCI_GET_LINK_QUALITY_OPCODE                         0x1403
#define HCI_READ_RSSI                                       0x1405
#define HCI_READ_AFH_CHANNEL_MAP_OPCODE                     0x1406
#define HCI_READ_CLOCK_OPCODE                               0x1407

#ifdef VER_3_0
/* Version 3.0 Additional Status Commands */
#define HCI_READ_ENCRYPTION_KEY_SIZE_OPCODE                 0x1408
#define HCI_READ_LOCAL_AMP_INFO_OPCODE                      0x1409
#define HCI_READ_LOCAL_AMP_ASSOC_OPCODE                     0x140A
#define HCI_WRITE_LOCAL_AMP_ASSOC_OPCODE                    0x140B
#endif

#ifdef VER_CSA4
/* Version CSA4 Additional Status Commands */
#define HCI_SET_TRIGGERED_CLOCK_CAPTURE_OPCODE              0x140D
#endif

/* Testing Commands - OGF : 0x06 */
#define HCI_READ_LOOPBACK_MODE_OPCODE                       0x1801
#define HCI_WRITE_LOOPBACK_MODE_OPCODE                      0x1802
#define HCI_ENABLE_DEVICE_UNDER_TEST_MODE_OPCODE            0x1803
/* Version 2.1 Additional Testing Commands */
#define HCI_WRITE_SIMPLE_PAIRING_DEBUG_MODE_OPCODE          0x1804

#ifdef VER_3_0
/* Version 3.0 Additional Testing Commands */
#define HCI_ENABLE_AMP_RECEIVER_REPORTS_OPCODE              0x1807
#define HCI_AMP_TEST_END_OPCODE                             0x1808
#define HCI_AMP_TEST_OPCODE                                 0x1809
#endif

#ifdef TEST_MODE
/* Proprietory HCI Commands to Invoke test Mode */
#define HCI_ACTIVATE_TEST_MODE_OPCODE                       0x1805
#define HCI_TEST_CONTROL_OPCODE                             0x1806
#endif /* TEST_MODE */
#ifdef _SUPPORT_SECURE_CONNECTION_
#define HCI_WRITE_SECURE_CONN_TEST_MODE_OPCODE              0x180A
#endif

/* LE Controller Commands - OGF : 0x08 - added by austin */
#define LE_HCI_OGF                                              0x08

#define HCI_LE_SET_EVENT_MASK_OCF                               0x0001
#define HCI_LE_READ_BUFFER_SIZE_OCF                             0x0002
#define HCI_LE_READ_LOCAL_SUPPORTED_FEATURES_OCF                0x0003
#define HCI_LE_SET_RANDOM_ADDRESS_OCF                           0x0005
#define HCI_LE_SET_ADVERTISING_PARAMETERS_OCF                   0x0006
#define HCI_LE_READ_ADVERTISING_CHANNEL_TX_POWER_OCF            0x0007
#define HCI_LE_SET_ADVERTISING_DATA_OCF                         0x0008
#define HCI_LE_SET_SCAN_RESPONSE_DATA_OCF                       0x0009
#define HCI_LE_SET_ADVERTISING_ENABLE_OCF                       0x000A
#define HCI_LE_SET_SCAN_PARAMETERS_OCF                          0x000B
#define HCI_LE_SET_SCAN_ENABLE_OCF                              0x000C
#define HCI_LE_CREATE_CONNECTION_OCF                            0x000D
#define HCI_LE_CREATE_CONNECTION_CANCEL_OCF                     0x000E
#define HCI_LE_READ_WHITE_LIST_SIZE_OCF                         0x000F
#define HCI_LE_CLEAR_WHITE_LIST_OCF                             0x0010
#define HCI_LE_ADD_DEVICE_TO_WHITE_LIST_OCF                     0x0011
#define HCI_LE_REMOVE_DEVICE_FROM_WHITE_LIST_OCF                0x0012
#define HCI_LE_CONNECTION_UPDATE_OCF                            0x0013
#define HCI_LE_SET_HOST_CHANNEL_CLASSIFICATION_OCF              0x0014
#define HCI_LE_READ_CHANNEL_MAP_OCF                             0x0015
#define HCI_LE_READ_REMOTE_USED_FEATURES_OCF                    0x0016
#define HCI_LE_ENCRYPT_OCF                                      0x0017
#define HCI_LE_RAND_OCF                                         0x0018
#define HCI_LE_START_ENCRYPTION_OCF                             0x0019
#define HCI_LE_LONG_TERM_KEY_REQUEST_REPLY_OCF                  0x001A
#define HCI_LE_LONG_TERM_KEY_REQUESTED_NEGATIVE_REPLY_OCF       0x001B
#define HCI_LE_READ_SUPPORTED_STATES_OCF                        0x001C
#define HCI_LE_RECEIVER_TEST_OCF                                0x001D
#define HCI_LE_TRANSMITTER_TEST_OCF                             0x001E
#define HCI_LE_TEST_END_OCF                                     0x001F
#define HCI_LE_SET_DATA_LENGTH_OCF                              0x0022
#define HCI_LE_READ_DEFAULT_DATA_LENGTH_OCF                     0x0023
#define HCI_LE_WRITE_DEFAULT_DATA_LENGTH_OCF                    0x0024
#define HCI_LE_READ_LOCAL_P256_PUBLIC_KEY_OCF                   0x0025
#define HCI_LE_GENERATE_DHKEY_OCF                               0x0026
#define HCI_LE_ADD_DEVICE_TO_RESOLVING_LIST_OCF                 0x0027
#define HCI_LE_REMOVE_DEVICE_FROM_RESOLVING_LIST_OCF            0x0028
#define HCI_LE_CLEAR_RESOLVING_LIST_OCF                         0x0029
#define HCI_LE_READ_RESOLVING_LIST_SIZE_OCF                     0x002A
#define HCI_LE_READ_PEER_RESOLVABLE_ADDRESS_OCF                 0x002B
#define HCI_LE_READ_LOCAL_RESOLVABLE_ADDRESS_OCF                0x002C
#define HCI_LE_SET_ADDRESS_RESOLUTION_ENABLE_OCF                0x002D
#define HCI_LE_SET_RESOLVABLE_PRIVATE_ADDRESS_TIMEOUT_OCF       0x002E
#define HCI_LE_READ_MAXIMUM_DATA_LENGTH_OCF                     0x002F
#define HCI_LE_READ_PHY_OCF                                     0x0030
#define HCI_LE_SET_DEFAULT_PHY_OCF                              0x0031
#define HCI_LE_SET_PHY_OCF                                      0x0032
#define HCI_LE_ENHANCED_RECEIVER_TEST_OCF                       0x0033
#define HCI_LE_ENHANCED_TANSMITTER_TEST_OCF                     0x0034
#define HCI_LE_MAX_OCF                                          HCI_LE_ENHANCED_TANSMITTER_TEST_OCF

/* bit[15:10] OGF and bit[9:0] OCF */
#define HCI_LE_SET_EVENT_MASK_OPCODE                            0x2001
#define HCI_LE_READ_BUFFER_SIZE_OPCODE                          0x2002
#define HCI_LE_READ_LOCAL_SUPPORTED_FEATURES_OPCODE             0x2003
#define HCI_LE_SET_RANDOM_ADDRESS_OPCODE                        0x2005
#define HCI_LE_SET_ADVERTISING_PARAMETERS_OPCODE                0x2006
#define HCI_LE_READ_ADVERTISING_CHANNEL_TX_POWER_OPCODE         0x2007
#define HCI_LE_SET_ADVERTISING_DATA_OPCODE                      0x2008
#define HCI_LE_SET_SCAN_RESPONSE_DATA_OPCODE                    0x2009
#define HCI_LE_SET_ADVERTISING_ENABLE_OPCODE                    0x200A
#define HCI_LE_SET_SCAN_PARAMETERS_OPCODE                       0x200B
#define HCI_LE_SET_SCAN_ENABLE_OPCODE                           0x200C
#define HCI_LE_CREATE_CONNECTION_OPCODE                         0x200D
#define HCI_LE_CREATE_CONNECTION_CANCEL_OPCODE                  0x200E
#define HCI_LE_READ_WHITE_LIST_SIZE_OPCODE                      0x200F
#define HCI_LE_CLEAR_WHITE_LIST_OPCODE                          0x2010
#define HCI_LE_ADD_DEVICE_TO_WHITE_LIST_OPCODE                  0x2011
#define HCI_LE_REMOVE_DEVICE_FROM_WHITE_LIST_OPCODE             0x2012
#define HCI_LE_CONNECTION_UPDATE_OPCODE                         0x2013
#define HCI_LE_SET_HOST_CHANNEL_CLASSIFICATION_OPCODE           0x2014
#define HCI_LE_READ_CHANNEL_MAP_OPCODE                          0x2015
#define HCI_LE_READ_REMOTE_USED_FEATURES_OPCODE                 0x2016
#define HCI_LE_ENCRYPT_OPCODE                                   0x2017
#define HCI_LE_RAND_OPCODE                                      0x2018
#define HCI_LE_START_ENCRYPTION_OPCODE                          0x2019
#define HCI_LE_LONG_TERM_KEY_REQUEST_REPLY_OPCODE               0x201A
#define HCI_LE_LONG_TERM_KEY_REQUESTED_NEGATIVE_REPLY_OPCODE    0x201B
#define HCI_LE_READ_SUPPORTED_STATES_OPCODE                     0x201C
#define HCI_LE_RECEIVER_TEST_OPCODE                             0x201D
#define HCI_LE_TRANSMITTER_TEST_OPCODE                          0x201E
#define HCI_LE_TEST_END_OPCODE                                  0x201F
#define HCI_LE_REMOTE_CONNECTION_PARAMETER_REQ_REPLY_OPCODE     0x2020
#define HCI_LE_REMOTE_CONNECTION_PARAMETER_REQ_NEG_REPLY_OPCODE 0x2021
#define HCI_LE_SET_DATA_LENGTH_OPCODE                           0x2022
#define HCI_LE_READ_DEFAULT_DATA_LENGTH_OPCODE                  0x2023
#define HCI_LE_WRITE_DEFAULT_DATA_LENGTH_OPCODE                 0x2024
#define HCI_LE_READ_LOCAL_P256_PUBLIC_KEY_OPCODE                0x2025
#define HCI_LE_GENERATE_DHKEY_OPCODE                            0x2026
#define HCI_LE_ADD_DEVICE_TO_RESOLVING_LIST_OPCODE              0x2027
#define HCI_LE_REMOVE_DEVICE_FROM_RESOLVING_LIST_OPCODE         0x2028
#define HCI_LE_CLEAR_RESOLVING_LIST_OPCODE                      0x2029
#define HCI_LE_READ_RESOLVING_LIST_SIZE_OPCODE                  0x202A
#define HCI_LE_READ_PEER_RESOLVABLE_ADDRESS_OPCODE              0x202B
#define HCI_LE_READ_LOCAL_RESOLVABLE_ADDRESS_OPCODE             0x202C
#define HCI_LE_SET_ADDRESS_RESOLUTION_ENABLE_OPCODE             0x202D
#define HCI_LE_SET_RESOLVABLE_PRIVATE_ADDRESS_TIMEOUT_OPCODE    0x202E
#define HCI_LE_READ_MAXIMUM_DATA_LENGTH_OPCODE                  0x202F
#define HCI_LE_READ_PHY_OPCODE                                  0x2030
#define HCI_LE_SET_DEFAULT_PHY_OPCODE                           0x2031
#define HCI_LE_SET_PHY_OPCODE                                   0x2032
#define HCI_LE_ENHANCED_RECEIVER_TEST_OPCODE                    0x2033
#define HCI_LE_ENHANCED_TRANSMITTER_TEST_OPCODE                 0x2034

/*===================== HCI Event Codes =================== */
#define HCI_INQUIRY_COMPLETE_EVENT                          0x01
#define HCI_INQUIRY_RESULT_EVENT                            0x02
#define HCI_CONNECTION_COMPLETE_EVENT                       0x03
#define HCI_CONNECTION_REQUEST_EVENT                        0x04
#define HCI_DISCONNECTION_COMPLETE_EVENT                    0x05
#define HCI_AUTHENTICATION_COMPLETE_EVENT                   0x06
#define HCI_REMOTE_NAME_REQUEST_COMPLETE_EVENT              0x07
#define HCI_ENCRYPTION_CHANGE_EVENT                         0x08
#define HCI_CHANGE_CONNECTION_LINK_KEY_COMPLETE_EVENT       0x09
#define HCI_MASTER_LINK_KEY_COMPLETE_EVENT                  0x0A
#define HCI_READ_REMOTE_SUPPORTED_FEATURES_COMPLETE_EVENT   0x0B
#define HCI_READ_REMOTE_VERSION_INFORMATION_COMPLETE_EVENT  0x0C
#define HCI_QOS_SETUP_COMPLETE_EVENT                        0x0D
#define HCI_COMMAND_COMPLETE_EVENT                          0x0E
#define HCI_COMMAND_STATUS_EVENT                            0x0F
#define HCI_HARDWARE_ERROR_EVENT                            0x10
#define HCI_FLUSH_OCCURRED_EVENT                            0x11
#define HCI_ROLE_CHANGE_EVENT                               0x12
#define HCI_NUMBER_OF_COMPLETED_PACKETS_EVENT               0x13
#define HCI_MODE_CHANGE_EVENT                               0x14
#define HCI_RETURN_LINK_KEYS_EVENT                          0x15
#define HCI_PIN_CODE_REQUEST_EVENT                          0x16
#define HCI_LINK_KEY_REQUEST_EVENT                          0x17
#define HCI_LINK_KEY_NOTIFICATION_EVENT                     0x18
#define HCI_LOOPBACK_COMMAND_EVENT                          0x19
#define HCI_DATA_BUFFER_OVERFLOW_EVENT                      0x1A
#define HCI_MAX_SLOTS_CHANGE_EVENT                          0x1B
#define HCI_READ_CLOCK_OFFSET_COMPLETE_EVENT                0x1C
#define HCI_CONNECTION_PACKET_TYPE_CHANGED_EVENT            0x1D
#define HCI_QOS_VIOLATION_EVENT                             0x1E
#define HCI_PAGE_SCAN_MODE_CHANGE_EVENT                     0x1F
#define HCI_PAGE_SCAN_REPETITION_MODE_CHANGE_EVENT          0x20
#define HCI_FLOW_SPECIFICATION_COMPLETE_EVENT               0x21
#define HCI_INQUIRY_RESULT_WITH_RSSI_EVENT                  0x22
#define HCI_READ_REMOTE_EXT_FEATURES_COMPLETE_EVENT         0x23
#define HCI_SYNCHRONOUS_CONNECTION_COMPLETE_EVENT           0x2C
#define HCI_SYNCHRONOUS_CONNECTION_CHANGED_EVENT            0x2D
#define HCI_CONNECTION_COMPLETE_EVENT_WITH_RS               0x70
/* Version 2.1 Additional HCI Events */
#define HCI_SNIFF_SUBRATING_EVENT   						    0x2E
#define HCI_EXTENDED_INQUIRY_RESULT_EVENT		 				0x2F
#define HCI_ENCRYPTION_KEY_REFRESH_COMPLETE_EVENT  				0x30
#define HCI_IO_CAPABILITY_REQUEST_EVENT    						0x31
#define HCI_IO_CAPABILITY_RESPONSE_EVENT    					0x32
#define HCI_USER_CONFIRMATION_REQUEST_EVENT    					0x33
#define HCI_USER_PASSKEY_REQUEST_EVENT    						0x34
#define HCI_REMOTE_OOB_DATA_REQUEST_EVENT    					0x35
#define HCI_SIMPLE_PAIRING_COMPLETE_EVENT    					0x36
#define HCI_LSTO_CHANGE_EVENT                                   0x38
#define HCI_ENHANCED_FLUSH_COMPLETE_EVENT                       0x39
#define HCI_USER_PASSKEY_NOTIFICATION_EVENT    				    0x3B
#define HCI_KEYPRESS_NOTIFICATION_EVENT    				  	    0x3C
#define HCI_REMOTE_HOST_SUPPORTED_FEATURES_NOTIF_EVENT          0x3D

/* Version 4.0 LE Mega Event - added by austin */
#define HCI_LE_MEGA_EVENT                                       0x3E
/* Version 4.0 LE Subevent - added by austin */
#define HCI_LE_CONNECTION_COMPLETE_SUBEVENT                     0x01
#define HCI_LE_ADVERTISING_REPORT_SUBEVENT                      0x02
#define HCI_LE_CONNECTION_UPDATE_COMPLETE_SUBEVENT              0x03
#define HCI_LE_READ_REMOTE_USED_FEATURES_COMPLETE_SUBEVENT      0x04
#define HCI_LE_LONG_TERM_KEY_REQUEST_SUBEVENT                   0x05
#define HCI_LE_DATA_LENGTH_CHANGE_SUBEVENT                      0x07
#define HCI_LE_READ_LOCAL_P256_PUBLIC_KEY_COMPLETE_SUBEVENT     0x08
#define HCI_LE_GENERATE_DHKEY_COMPLETE_SUBEVENT                 0x09
#define HCI_LE_ENHANCED_CONNECTION_COMPLETE_SUBEVENT            0x0A
#define HCI_LE_DIRECT_ADVERTISING_REPORT_SUBEVENT               0x0B
#define HCI_LE_PHY_UPDATE_COMPLETE_SUBEVENT                     0x0C

#ifdef VER_3_0
/* Version 3.0 Additional HCI Events */
#define HCI_PHYSICAL_LINK_COMPLETE_EVENT                        0x40
#define HCI_CHANNEL_SELECTED_EVENT                              0x41
#define HCI_DISCONNECTION_PHYSICAL_LINK_COMPLETE_EVENT          0x42
#define HCI_PHYSICAL_LINK_LOSS_EARLY_WARNING_EVENT              0x43
#define HCI_PHYSICAL_LINK_RECOVERY_EVENT                        0x44
#define HCI_LOGICAL_LINK_COMPLETE_EVENT                         0x45
#define HCI_DISCONNECTION_LOGICAL_LINK_COMPLETE_EVENT           0x46
#define HCI_FLOW_SPEC_MODIFY_COMPLETE_EVENT                     0x47
#define HCI_NUMBER_OF_COMPLETED_DATA_BLOCKS_EVENT               0x48
#define HCI_SHORT_RANGE_MODE_CHANGE_COMPLETE_EVENT              0x4C
#define HCI_AMP_STATUS_CHANGE_EVENT                             0x4D
#define HCI_AMP_START_TEST_EVENT                                0x49
#define HCI_AMP_TEST_END_EVENT                                  0x4A
#define HCI_AMP_RECEIVER_REPORT_EVENT                           0x4B
#endif

#ifdef VER_CSA4
/* Version CSA4 Additional HCI Events */
#define HCI_TRIGGERED_CLOCK_CAPTURE_EVENT                           0x4E
#define HCI_SYNCHRONIZATION_TRAIN_COMPLETE_EVENT                    0x4F
#define HCI_SYNCHRONIZATION_TRAIN_RECEIVED_EVENT                    0x50
#define HCI_CONNECTIONLESS_SLAVE_BROADCAST_RECEIVED_EVENT           0x51
#define HCI_CONNECTIONLESS_SLAVE_BROADCAST_TIMEOUT_EVENT            0x52
#define HCI_TRUNCATED_PAGE_COMPLETE_EVENT                           0x53
#define HCI_SLAVE_PAGE_RESPONSE_TIMEOUT_EVENT                       0x54
#define HCI_CONNECTIONLESS_SLAVE_BROADCAST_CHANNEL_MAP_CHANGE_EVENT 0x55
#define HCI_INQUIRY_RESPONSE_NOTIFICATION_EVENT                     0x56
#endif
#ifdef _SUPPORT_SECURE_CONNECTION_
#define HCI_AUTH_PAYLOAD_TIMEOUT_EXPIRED_EVENT                      0x57
#endif

/*========================= HCI Event Mask Codes =======================*/
#define HCI_INQUIRY_COMPLETE_EVENT_MASK                           0x00000001
#define HCI_INQUIRY_RESULT_EVENT_MASK                             0x00000002
#define HCI_CONNECTION_COMPLETE_EVENT_MASK                        0x00000004
#define HCI_CONNECTION_REQUEST_EVENT_MASK                         0x00000008
#define HCI_DISCONNECTION_COMPLETE_EVENT_MASK                     0x00000010
#define HCI_AUTHENTICATION_COMPLETE_EVENT_MASK                    0x00000020
#define HCI_REMOTE_NAME_REQUEST_COMPLETE_EVENT_MASK               0x00000040
#define HCI_ENCRYPTION_CHANGE_EVENT_MASK                          0x00000080
#define HCI_CHANGE_CONNECTION_LINK_KEY_COMPLETE_EVENT_MASK        0x00000100
#define HCI_MASTER_LINK_KEY_COMPLETE_EVENT_MASK                   0x00000200
#define HCI_READ_REMOTE_SUPPORTED_FEATURES_COMPLETE_EVENT_MASK    0x00000400
#define HCI_READ_REMOTE_VERSION_INFORMATION_COMPLETE_EVENT_MASK   0x00000800
#define HCI_QOS_SETUP_COMPLETE_EVENT_MASK                         0x00001000
#define HCI_COMMAND_COMPLETE_EVENT_MASK                           0x00002000
#define HCI_COMMAND_STATUS_EVENT_MASK                             0x00004000
#define HCI_HARDWARE_ERROR_EVENT_MASK                             0x00008000
#define HCI_FLUSH_OCCURRED_EVENT_MASK                             0x00010000
#define HCI_ROLE_CHANGE_EVENT_MASK                                0x00020000
#define HCI_NUMBER_OF_COMPLETED_PACKETS_EVENT_MASK                0x00040000
#define HCI_MODE_CHANGE_EVENT_MASK                                0x00080000
#define HCI_RETURN_LINK_KEYS_EVENT_MASK                           0x00100000
#define HCI_PIN_CODE_REQUEST_EVENT_MASK                           0x00200000
#define HCI_LINK_KEY_REQUEST_EVENT_MASK                           0x00400000
#define HCI_LINK_KEY_NOTIFICATION_EVENT_MASK                      0x00800000
#define HCI_LOOPBACK_COMMAND_EVENT_MASK                           0x01000000
#define HCI_DATA_BUFFER_OVERFLOW_EVENT_MASK                       0x02000000
#define HCI_MAX_SLOTS_CHANGE_EVENT_MASK                           0x04000000
#define HCI_READ_CLOCK_OFFSET_COMPLETE_EVENT_MASK                 0x08000000
#define HCI_CONNECTION_PACKET_TYPE_CHANGED_EVENT_MASK             0x10000000
#define HCI_QOS_VIOLATION_EVENT_MASK                              0x20000000
#define HCI_PAGE_SCAN_MODE_CHANGE_EVENT_MASK                      0x40000000
#define HCI_PAGE_SCAN_REPETITION_MODE_CHANGE_EVENT_MASK           0x80000000
/* Version 2.1 Additional HCI Event Masks */
/* Might not be used */
#define SNIFF_SUBRATING_EVENT_MASK  							0x0000200000000000
#define EXTENDED_INQUIRY_RESULT_EVENT_MASK 						0x0000400000000000
#define ENCRYPTION_KEY_REFRESH_COMPLETE_EVENT_MASK 				0x0000800000000000
#define IO_CAPABILITY_REQUEST_EVENT_MASK 						0x0001000000000000
#define IO_CAPABILITY_REQUEST_REPLY_EVENT_MASK 					0x0002000000000000
#define USER_CONFIRMATION_REQUEST_EVENT_MASK 					0x0004000000000000
#define USER_PASSKEY_REQUEST_EVENT_MASK 						0x0008000000000000
#define REMOTE_OOB_DATA_REQUEST_EVENT_MASK 						0x0010000000000000
#define SIMPLE_PAIRING_COMPLETE_EVENT_MASK 						0x0020000000000000
#define LINK_SUPERVISION_TIMEOUT_CHANGED_EVENT_MASK 			0x0080000000000000
#define ENHANCED_FLUSH_COMPLETE_EVENT_MASK 						0x0100000000000000
#define USER_PASSKEY_NOTIFICATION_EVENT_MASK 					0x0400000000000000
#define KEYPRESS_NOTIFICATION_EVENT_MASK 						0x0800000000000000
#define REMOTE_HOST_SUPPORTED_FEATURES_NOTIFICATION_EVENT_MASK  0x1000000000000000

/* Version 4.0 LE Event Masks - added by austin */
#define LE_CONNECTION_COMPLATE_EVENT_MASK                       0x0000000000000001
#define LE_ADVERTISING_REPORT_EVENT_MASK                        0x0000000000000002
#define LE_CONNECTION_UPDATE_COMPLETE_EVENT_MASK                0x0000000000000004
#define LE_READ_REMOTE_USED_FEATURES_COMPLETE_EVENT_MASK        0x0000000000000008
#define LE_LONG_TERM_KEY_REQUEST_EVENT_MASK                     0x0000000000000010
#define LE_DEFAULT_EVENT_MASK                                   0x000000000000001F
#define LE_DATA_LENGTH_CHANGE_EVENT_MASK                        0x0000000000000040
#define LE_ENHANCED_CONNECTION_COMPLETE_EVENT_MASK              0x0000000000000200
#define LE_DIRECT_ADVERTISING_REPORT_EVENT_MASK                 0x0000000000000400
#define LE_PHY_UPDATE_COMPLETE_EVENT_MASK                       0x0000000000000800


#define LE_CONNECTION_COMPLATE_EVENT_LSB_MASK                   0x01
#define LE_ADVERTISING_REPORT_EVENT_LSB_MASK                    0x02
#define LE_CONNECTION_UPDATE_COMPLETE_EVENT_LSB_MASK            0x04
#define LE_READ_REMOTE_USED_FEATURES_COMPLETE_EVENT_LSB_MASK    0x08
#define LE_LONG_TERM_KEY_REQUEST_EVENT_LSB_MASK                 0x10
#define LE_DEFAULT_EVENT_LSB_MASK                               0x1F

/*============================= Error Codes =============================*/
#define HCI_COMMAND_SUCCEEDED                                 0x00
#define UNKNOWN_HCI_COMMAND_ERROR                             0x01
#define NO_CONNECTION_ERROR                                   0x02
#define HARDWARE_FAILURE_ERROR                                0x03
#define PAGE_TIMEOUT_ERROR                                    0x04
#define AUTHENTICATION_FAILURE_ERROR                          0x05
#define KEY_MISSING_ERROR                                     0x06
#define MEMORY_FULL_ERROR                                     0x07
#define CONNECTION_TIMEOUT_ERROR                              0x08
#define MAX_NUMBER_OF_CONNECTIONS_ERROR                       0x09
#define MAX_SCO_CONNECTIONS_REACHED_ERROR                     0x0A
#define ACL_CONNECTION_EXISTS_ERROR                           0x0B
#define COMMAND_DISALLOWED_ERROR                              0x0C
#define HOST_REJECTED_LIMITED_RESOURCES_ERROR                 0x0D
#define HOST_REJECTED_SECURITY_REASONS_ERROR                  0x0E
#define HOST_REJECTED_PERSONAL_DEVICE_ERROR                   0x0F
#define CONNECTION_ACCEPT_TIMEOUT_EXCEEDED_ERROR              0x10
#define UNSUPPORTED_FEATURE_OR_PARAMETER_ERROR                0x11
#define INVALID_HCI_COMMAND_PARAMETERS_ERROR                  0x12
#define CONNECTION_TERMINATED_USER_ERROR                      0x13
#define CONNECTION_TERMINATED_LOW_RESOURCES_ERROR             0x14
#define CONNECTION_TERMINATED_POWER_OFF_ERROR                 0x15
#define CONNECTION_TERMINATED_LOCAL_HOST_ERROR                0x16
#define REPEATED_ATTEMPTS_ERROR                               0x17
#define PAIRING_NOT_ALLOWED_ERROR                             0x18
#define UNKNOWN_LMP_PDU_ERROR                                 0x19
#define UNSUPPORTED_REMOTE_FEATURE_ERROR                      0x1A
#define SCO_OFFSET_REJECTED_ERROR                             0x1B
#define SCO_INTERVAL_REJECTED_ERROR                           0x1C
#define SCO_AIR_MODE_REJECTED_ERROR                           0x1D
#define INVALID_LMP_PARAMETERS_ERROR                          0x1E
#define UNSPECIFIED_ERROR                                     0x1F
#define UNSUPPORTED_PARAMETER_VALUE_ERROR                     0x20
#define SWITCH_NOT_ALLOWED_ERROR                              0x21
#define LMP_RESPONSE_TIMEOUT_ERROR                            0x22
#define LL_RESPONSE_TIMEOUT_ERROR                             0x22
#define LMP_ERROR_TRANSACTION_COLLISION_ERROR                 0x23
#define PDU_NOT_ALLOWED_ERROR                                 0x24
#define ENCRYPTION_MODE_NOT_ACCEPTABLE_ERROR                  0x25
#define UNIT_KEY_USED_ERROR                                   0x26
#define QOS_NOT_SUPPORTED_ERROR                               0x27
#define INSTANT_PASSED_ERROR                                  0x28
#define PAIRING_WITH_UNIT_KEY_NOT_SUPPPORTED_ERROR            0x29
/* For 1.2 */
#define DIFFERENT_TRANSACTION_COLLISION                       0x2A
#define QOS_UNACCEPTABLE_PARAMETER                            0x2C
#define QOS_REJECTED_ERROR                                    0x2D
#define CHANNEL_CLASSIFICATION_NOT_SUPPORTED                  0x2E
#define INSUFFICIENT_SECURITY                                 0x2F
#define PARAMETER_OUT_OF_MANDATORY_RANGE                      0x30
#define ROLE_SWITCH_PENDING                                   0x32
#define RESERVED_SLOT_VIOLATION                               0x34
#define ROLE_SWITCH_FAILED                                    0x35
/* Version 2.1 Additional Error Codes */
#define EXTENDED_INQUIRY_RESPONSE_TOO_LARGE  		      0x36
#define SECURE_SIMPLE_PAIRING_NOT_SUPPORTED_BY_HOST 	      0x37
#define HOST_BUSY_PAIRING  				      0x38

/* Version 3.0 Additional Error Codes */
#define CONNECTION_REJECTED_DUE_TO_NO_SUITABLE_CHANNEL_FOUND  0x39
#define CONTROLLER_BUSY                                       0x3A

/* Version 4.0 Additional Error Codes - added by austin */
#define UNACCEPTABLE_CONNECTION_INTERVAL                      0x3B
#define DIRECTED_ADVERTISING_TIMEOUT                          0x3C
#define CONNECTION_TERMINATED_DUE_TO_MIC_FAILURE              0x3D
#define CONNECTION_FAILED_TO_BE_ESTABLISHED                   0x3E
#define MAC_CONNECTION_FAILED                                 0x3F

/* Version 4.1 Additional Error Codes */
#define COARSE_CLK_ADJ_REJ_BUT_WILL_TRY_TO_ADJUST_USING_DRAG  0x40

#ifdef _RTK8723_UART_INIT_
#define HARDWARE_FAILURE_ERROR_RTK_H4                                0xA0
#endif

#ifndef BT_HCI_4_0
#ifndef VER_3_0
#define LAST_ENTRY_BLUETOOTH_ERROR_CODE                       HOST_BUSY_PAIRING
#else
#define LAST_ENTRY_BLUETOOTH_ERROR_CODE                       CONTROLLER_BUSY
#endif
#else
#define LAST_ENTRY_BLUETOOTH_ERROR_CODE                       MAC_CONNECTION_FAILED
#endif

/* ============== Event Lengths ==================== */
#define HCI_INQUIRY_COMPLETE_EVENT_LEN                        0x01
#define HCI_HARDWARE_ERROR_EVENT_LEN                          0x01
#define HCI_FLUSH_OCCURRED_EVENT_LEN                          0x02
#define HCI_COMMAND_STATUS_EVENT_LEN                          0x04
#define HCI_SNIFF_SUBRATING_EVENT_LEN                         0x0B

#define HCI_SNIFF_SUBRATING_EVENT_LEN                           0x0B
#define HCI_EXTENDED_INQUIRY_RESULT_EVENT_LEN		 			0xFF
#define HCI_ENCRYPTION_KEY_REFRESH_COMPLETE_EVENT_LEN           0x03
#define HCI_IO_CAPABILITY_REQUEST_EVENT_LEN    					0x06
#define HCI_IO_CAPABILITY_RESPONSE_EVENT_LEN    				0x09
#define HCI_USER_CONFIRMATION_REQUEST_EVENT_LEN    				0x0A
#define HCI_USER_PASSKEY_REQUEST_EVENT_LEN    					0x06
#define HCI_REMOTE_OOB_DATA_REQUEST_EVENT_LEN    				0x06
#define HCI_SIMPLE_PAIRING_COMPLETE_EVENT_LEN    				0x07
#define HCI_LSTO_CHANGE_EVENT_LEN                               0x04
#define HCI_ENHANCED_FLUSH_COMPLETE_EVENT_LEN                   0x02
#define HCI_USER_PASSKEY_NOTIFICATION_EVENT_LEN    				0x0A
#define HCI_KEYPRESS_NOTIFICATION_EVENT_LEN    				  	0x07
#define HCI_REMOTE_HOST_SUPPORTED_FEATURES_NOTIF_EVENT_LEN      0x0E

/* Hold mode activity defines. */
#define HCI_HOLD_MODE_ACTIVITY_DEFAULT                      0x00
#define HCI_HOLD_MODE_ACTIVITY_SUSPEND_PAGE_SCAN            0x01
#define HCI_HOLD_MODE_ACTIVITY_SUSPEND_INQ_SCAN             0x02
#define HCI_HOLD_MODE_ACTIVITY_SUSPEND_PER_INQ              0x04

/**
 * This Macro is to extract group for which the comand belongs from the
 * command opcode.
 */
#define HCI_GET_GROUP_OPCODE(cmd_opcode)   (cmd_opcode>>10)
 /* CMD PARAM Maximum and Minimum Length */
#define HCI_MAX_CMD_PARAM_LEN                255
#define HCI_MIN_CMD_PARAM_LEN                  0

/* MAX and MIN EVENT PARAM LEN */
#define HCI_MAX_EVENT_PARAM_LEN              255
#define HCI_MIN_EVENT_PARAM_LEN               02

#define HCI_NEW_DEV_RESP                    0x00
#define HCI_NEW_DEV_COD                     0x01
#define HCI_NEW_DEV_BD_ADDR                 0x02
#define HCI_ALLOW_ALL_CONNECTIONS           0x00
#define HCI_ALLOW_COD_CONNECTIONS           0x01
#define HCI_ALL_BD_ADDR_CONNECTIONS         0x02
/* No operation opcode */
#define HCI_NOP                             0x00

/* Inquiry parameters */
#define HCI_MIN_INQUIRY_LENGTH              0x01
#define HCI_MAX_INQUIRY_LENGTH              0x30

/* QoS parameters */
#define QOS_NO_TRAFFIC                      0x00
#define QOS_BEST_EFFORT                     0x01
#define QOS_GUARANTEED                      0x02
#define QOS_MAX_SERVICE_TYPE                0x02
#define HCI_DEFAULT_QoS_ACCESS_LATENCY    0xffffffff
#define HCI_DEFAULT_QoS_TOKEN_RATE        0x00000000
#define HCI_QoS_TOKEN_RATE_WILD_CARD      0xffffffff

#define LE_CONN_MAX_TX_SIZE_MIN     0x001B
#define LE_CONN_MAX_TX_SIZE_MAX     0x00FB
#define LE_CONN_MAX_TX_TIME_MIN     0x0148
#define LE_CONN_MAX_TX_TIME_MAX     0x0848
#define LE_CONN_MAX_RX_SIZE_MIN     LE_CONN_MAX_TX_SIZE_MIN
#define LE_CONN_MAX_RX_SIZE_MAX     LE_CONN_MAX_TX_SIZE_MAX
#define LE_CONN_MAX_RX_TIME_MIN     LE_CONN_MAX_TX_TIME_MIN
#define LE_CONN_MAX_RX_TIME_MAX     LE_CONN_MAX_TX_TIME_MAX

/* ============== Packet Types ===================== */
#define DM1                                      0x0008U
#define DH1                                      0x0010U
#define DM3                                      0x0400U
#define DH3                                      0x0800U
#define DM5                                      0x4000U
#define DH5                                      0x8000U
#define DM1_DH1                                  0x0018U
#define DM3_DH3                                  0x0C00U
#define DM5_DH5                                  0xC000U
#define HV1                                      0x0020U
#define HV2                                      0x0040U
#define HV3                                      0x0080U
#define  EDR_2DH1                                0x0002U
#define  EDR_2DH3                                0x0100U
#define  EDR_2DH5                                0x1000U
#define  EDR_3DH1                                0x0004U
#define  EDR_3DH3                                0x0200U
#define  EDR_3DH5                                0x2000U

/* ALL_HCI_1_1_SCO_PKT_TYPES = (HV1 | HV2 | HV3) */
#define ALL_HCI_1_1_SCO_PKT_TYPES    0x00E0
/* ALL_ACL_PKT_TYPES = DM1 | DH1 | DM3 | DH3 | DM5 | DH5 */
#define ALL_ACL_PKT_TYPES                        0xFF1EU

/* ==============1.2 Sycnchronous connection packet tyeps====== */
#define HCI_HV1                                   0x0001
#define HCI_HV2                                   0x0002
#define HCI_HV3                                   0x0004
#define HCI_EV3                                   0x0008
#define HCI_EV4                                   0x0010
#define HCI_EV5                                   0x0020
#define HCI_2_EV3                                 0x0040
#define HCI_3_EV3                                 0x0080
#define HCI_2_EV5                                 0x0100
#define HCI_3_EV5                                 0x0200
#define MAX_NO_OF_BYTES_IN_EV3                        30
#define MAX_NO_OF_BYTES_IN_EV4                       120
#define MAX_NO_OF_BYTES_IN_EV5                       180
#define MAX_NO_OF_BYTES_IN_2_EV3                      60
#define MAX_NO_OF_BYTES_IN_3_EV3                      90
#define MAX_NO_OF_BYTES_IN_2_EV5                     360
#define MAX_NO_OF_BYTES_IN_3_EV5                     540
#define ALL_HCI_1_2_SCO_PKT_TYPES    (HCI_HV1 | HCI_HV2 | HCI_HV3)
#define ALL_HCI_1_2_ESCO_PKT_TYPES   (HCI_EV3 | HCI_EV4 | HCI_EV5)
#define ALL_HCI_SCO_PKT_TYPES        (ALL_HCI_1_2_SCO_PKT_TYPES)

#define ALL_HCI_2_0_ESCO_PKT_TYPES   (HCI_2_EV3 | HCI_3_EV3 | HCI_2_EV5 | \
                                      HCI_3_EV5)
#define ALL_HCI_ESCO_PKT_TYPES  (ALL_HCI_1_2_ESCO_PKT_TYPES | \
                                 ALL_HCI_2_0_ESCO_PKT_TYPES)

#define FEATURE_BYTE_1_AIR_MODES_MASK               0xC0
#define FEATURE_BYTE_2_AIR_MODES_MASK               0x09
/* Minimum value of Max latency in milliseconds */
#define MAX_LATENCY_MIN_VAL                            4
/* Minimum value of Max latency in slots */
#define MAX_LATENCY_MIN_VAL_IN_SLOTS                   6
#define INVALID_TEST_SCENARIO_BEGIN                   10
#define INVALID_TEST_SCENARIO_END                    254
#define WHITENING_VALUE                             0x55
/*
 * This value has to be programmed when a change of hops is made
 * to 23 hop mode.NOTE: It is DECIMAL 52, not hex 52 :-)
 */
#define HOP_OFFSET_23                                 52
/* HCI inquiry result formats */
#define HCI_STD_INQUIRY_RESULT                      0x00
#define HCI_INQUIRY_RESULT_WITH_RSSI                0x01
#ifdef TEST_MODE
/* Test modes */
#define HCI_NO_LOOPBACK_MODE                        0x00
#define HCI_LOCAL_LOOPBACK_MODE                     0x01
#define HCI_REMOTE_LOOPBACK_MODE                    0x02
#define HCI_DEVICE_UNDER_TEST_MODE                  0x03
#define HCI_DUT_LOOPBACK_MODE                       0x04

/*  Added by Wallice Su for RTK LoopBack Mode*/
#define HCI_RTK_LOCAL_LOOPBACK_MODE                 0xFF
/*  End Added by Wallice Su for RTK LoopBack Mode*/

/* Test States */
#define TEST_INITIALIZED                            0x01
#define TEST_ACTIVATED                              0x02
#define TEST_STARTED                                0x03
#define PACKET_TYPE_DESCRIPTION_BR_ACL_SCO          0x00

#ifdef COMPILE_ESCO
#define PACKET_TYPE_DESCRIPTION_BR_eSCO             0x01
#endif

#define PACKET_TYPE_DESCRIPTION_EDR_ACL             0x02
#ifdef COMPILE_ESCO
#define PACKET_TYPE_DESCRIPTION_EDR_eSCO            0x03
#endif

#define	MAX_REMOTE_TEST_DATA_LEN                     1064

/* Test scenarios */
#define TEST_MODE_PAUSE_TEST_MODE								0x00
#define TEST_MODE_TRANSMITTER_TEST_ZERO_PATTERN					0x01
#define TEST_MODE_TRANSMITTER_TEST_ONE_PATTERN                  0x02
#define TEST_MODE_TRANSMITTER_TEST_1010_PATTERN                 0x03
#define TEST_MODE_PSEUDORANDOM_BIT_SEQUENCE						0x04
#define TEST_MODE_CLOSED_LOOPBACK_ACL_PKTS                      0x05
#define TEST_MODE_CLOSED_LOOPBACK_SCO_PKTS                      0x06
#define TEST_MODE_ACL_PKTS_WITHOUT_WHITENING                    0x07
#define TEST_MODE_SCO_PKTS_WITHOUT_WHITENING                    0x08
#define TEST_MODE_TRANSMITTER_TEST_11110000_PATTERN             0x09
#define TEST_MODE_EXIT_TEST_MODE                                255
/* Default state of the test scenario. */
#define TEST_MODE_DEFAULT_STATE                                 0x12
#define HOP_BITS_ISOLATE(value)	                      (value &= 0xff9f)
#define		TM_MAX_PACKET_TYPES                       15
#define		TC_MAX_NUMBER_OF_BYTES_IN_AUX_PACKET      29
#define		TC_MAX_NUMBER_OF_BYTES_IN_HV3_PACKET      30
#define		TC_MAX_NUMBER_OF_BYTES_IN_HV2_PACKET      20
#define		TC_MAX_NUMBER_OF_BYTES_IN_HV1_PACKET      10

/* These defines are used for parameter checking */
#define MAX_DM1_PKT_SIZE                              17
#define MAX_DM3_PKT_SIZE                             121
#define MAX_DM5_PKT_SIZE                             224
#define MAX_DH1_PKT_SIZE                              27
#define MAX_DH3_PKT_SIZE                             183
#define MAX_DH5_PKT_SIZE                             339
#define MAX_HV1_PKT_SIZE                              10
#define MAX_HV2_PKT_SIZE                              20
#define MAX_HV3_PKT_SIZE                              30
#define MAX_2DH1_PKT_SIZE                             54
#define MAX_2DH3_PKT_SIZE                            367
#define MAX_2DH5_PKT_SIZE                            679
#define MAX_3DH1_PKT_SIZE                             83
#define MAX_3DH3_PKT_SIZE                            552
#define MAX_3DH5_PKT_SIZE                           1021

/* These defines are used to check incoming HCI comand */
#define		TEST_MODE_SINGLE_FREQUENCY_MODE         0x00
#define		TEST_MODE_HOP_79                        0x01 /* USA, Europe */
/*  Set of baseband register values corresponding to the hops */
#define		TEST_MODE_BASEBAND_HOP_79               0x20
#define		TEST_MODE_BASEBAND_HOP_23               0x00
#define		TEST_MODE_BASEBAND_HOP_1                0x60
#define		TEST_MODE_BASEBAND_HOP_5                0x40
#define		MAX_FREQUENCY_VALUE	                      93
#define		MAX_POWER_CONTROL_VALUE                    1
/* Values used for packet type, TestControlMaster command */
#define	TC_POLL_PACKET_TYPE                         0x01
#define TC_NULL_PACKET_TYPE                         0x02
#define TC_DM1_PACKET_TYPE                          0x03
#define TC_HV1_PACKET_TYPE                          0x04
#define TC_HV2_PACKET_TYPE                          0x05
#define TC_HV3_PACKET_TYPE                          0x06
#define TC_DV_PACKET_TYPE                           0x07
#define TC_DH1_PACKET_TYPE                          0x08
#define TC_DM3_PACKET_TYPE                          0x09
#define TC_DM5_PACKET_TYPE                          0x0a
#define TC_DH5_PACKET_TYPE                          0x0b
#define TC_AUX_PACKET_TYPE                          0x0c
/*  Hopping modes used in Lmp_Test_Control PDU */
#define HOPPING_MODE_RX_TX_SINGLE_FREQUENCY         0x00
#define HOPPING_MODE_EUROPE_USA                     0x01
#define HOPPING_MODE_FRANCE                         0x03
#define HOPPING_MODE_REDUCED                        0x05
#endif /* TEST_MODE */
#endif  /* __HCI_SPEC_DEFINES_H__ */
/** @} end: hci_external */
