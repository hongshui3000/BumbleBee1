enum { __FILE_NUM__ = 0 };

#include <ltplib.h>
#include <string.h>
#include "aci_low_power.h"
#include "trace.h"
#if BREDR_SUPPORT
#include "btltp_br.h"
#endif

#define LTP_SOURCE_FILE_ID 0x83

static uint16_t LTPLibGetOptLength(uint8_t copmsk);
static BOOL LTPLibHandleUnkownCommand(PLTPLib pLTPLib, uint8_t cmd);

const LTPCmdInfo  LTPCmdTable[] =
{
    /* name,                                min length,                                   flags */
    /* MDC -> MDH */
    { LTP_CONNECT_MDL_INFO,                 LTP_CONNECT_MDL_INFO_LENGTH,                  LTP_CONNECT_MDL_INFO_FLAGS                  },
    { LTP_CREATE_MDL_CNF,                   LTP_CREATE_MDL_CNF_LENGTH,                    LTP_CREATE_MDL_CNF_FLAGS                    },
    { LTP_DELETE_MDL_INFO,                  LTP_DELETE_MDL_INFO_LENGTH,                   LTP_DELETE_MDL_INFO_FLAGS                   },
    { LTP_DISCONNECT_MDL_RSP,               LTP_DISCONNECT_MDL_RSP_LENGTH,                LTP_DISCONNECT_MDL_RSP_FLAGS                },
    { LTP_DISCONNECT_MDL_CNF,               LTP_DISCONNECT_MDL_CNF_LENGTH,                LTP_DISCONNECT_MDL_CNF_FLAGS                },
    { LTP_EXIT_RSP,                         LTP_EXIT_RSP_LENGTH,                          LTP_EXIT_RSP_FLAGS                          },
    { LTP_ACT_INFO,                         LTP_ACT_INFO_LENGTH,                          LTP_ACT_INFO_FLAGS                          },
    { LTP_ACL_STATUS_INFO,                  LTP_ACL_STATUS_INFO_LENGTH,                   LTP_ACL_STATUS_INFO_FLAGS                   },
    { LTP_RESET_RSP,                        LTP_RESET_RSP_LENGTH,                         LTP_RESET_RSP_FLAGS                         },
    { LTP_INTERNAL_EVENT_INFO,              LTP_INTERNAL_EVENT_INFO_LENGTH,               LTP_INTERNAL_EVENT_INFO_FLAGS               },
    { LTP_PASSKEY_REQUEST_CNF,              LTP_PASSKEY_REQUEST_CNF_LENGTH,               LTP_PASSKEY_REQUEST_CNF_FLAGS               },
    { LTP_REMOTE_OOB_REQUEST_CNF,           LTP_REMOTE_OOB_REQUEST_CNF_LENGTH,            LTP_REMOTE_OOB_REQUEST_CNF_FLAGS            },
    { LTP_MCL_STATUS_INFO, /*delete*/       LTP_MCL_STATUS_INFO_LENGTH,                   LTP_MCL_STATUS_INFO_FLAGS                   },
    { LTP_PAIRABLE_MODE_SET_RSP,            LTP_PAIRABLE_MODE_SET_RSP_LENGTH,             LTP_PAIRABLE_MODE_SET_RSP_FLAGS             },
    { LTP_PASSKEY_REQ_REPLY_RSP,            LTP_PASSKEY_REQ_REPLY_RSP_LENGTH,             LTP_PASSKEY_REQ_REPLY_RSP_FLAGS             },
    { LTP_PASSKEY_NOTIFICATION_INFO,        LTP_PASSKEY_NOTIFICATION_INFO_LENGTH,         LTP_PASSKEY_NOTIFICATION_INFO_FLAGS         },
    { LTP_CONNECT_GATT_MDL_RSP,             LTP_CONNECT_GATT_MDL_RSP_LENGTH,              LTP_CONNECT_GATT_MDL_RSP_FLAGS              },
    { LTP_GATT_SERVICE_REGISTER_RSP,        LTP_GATT_SERVICE_REGISTER_RSP_LENGTH,         LTP_GATT_SERVICE_REGISTER_RSP_FLAGS         },
    { LTP_GATT_ATTRIBUTE_UPDATE_RSP,        LTP_GATT_ATTRIBUTE_UPDATE_RSP_LENGTH,         LTP_GATT_ATTRIBUTE_UPDATE_RSP_FLAGS         },
    { LTP_GATT_ATTRIBUTE_UPDATE_STATUS_CNF, LTP_GATT_ATTRIBUTE_UPDATE_STATUS_CNF_LENGTH,  LTP_GATT_ATTRIBUTE_UPDATE_STATUS_CNF_FLAGS  },
    { LTP_GATT_ATTRIBUTE_READ_CNF,          LTP_GATT_ATTRIBUTE_READ_CNF_LENGTH,           LTP_GATT_ATTRIBUTE_READ_CNF_FLAGS           },
    { LTP_GATT_ATTRIBUTE_WRITE_CNF,         LTP_GATT_ATTRIBUTE_WRITE_CNF_LENGTH,          LTP_GATT_ATTRIBUTE_WRITE_CNF_FLAGS          },
    { LTP_GATT_CCCD_INFO,                   LTP_GATT_CCCD_INFO_LENGTH,                    LTP_GATT_CCCD_INFO_FLAGS                    },
    { LTP_GATT_DISCOVERY_RSP,               LTP_GATT_DISCOVERY_RSP_LENGTH,                LTP_GATT_DISCOVERY_RSP_FLAGS                },
    { LTP_GATT_DISCOVERY_CNF,               LTP_GATT_DISCOVERY_CNF_LENGTH,                LTP_GATT_DISCOVERY_CNF_FLAGS                },
    { LTP_GATT_ATTRIBUTE_READ_RSP,          LTP_GATT_ATTRIBUTE_READ_RSP_LENGTH,           LTP_GATT_ATTRIBUTE_READ_RSP_FLAGS           },
    { LTP_GATT_ATTRIBUTE_WRITE_RSP,         LTP_GATT_ATTRIBUTE_WRITE_RSP_LENGTH,          LTP_GATT_ATTRIBUTE_WRITE_RSP_FLAGS          },
    { LTP_GATT_ATTRIBUTE_CNF,               LTP_GATT_ATTRIBUTE_CNF_LENGTH,                LTP_GATT_ATTRIBUTE_CNF_FLAGS                },
    { LTP_GATT_ATTRIBUTE_NOTIFICATION_INFO, LTP_GATT_ATTRIBUTE_NOTIFICATION_INFO_LENGTH,  LTP_GATT_ATTRIBUTE_NOTIFICATION_INFO_FLAGS  },
    { LTP_LE_ADVERTISE_RSP,                 LTP_LE_ADVERTISE_RSP_LENGTH,                  LTP_LE_ADVERTISE_RSP_FLAGS                  },
    { LTP_LE_ADVERTISE_PARAMETER_SET_RSP,   LTP_LE_ADVERTISE_PARAMETER_SET_RSP_LENGTH,    LTP_LE_ADVERTISE_PARAMETER_SET_RSP_FLAGS    },
    { LTP_LE_ADVERTISE_DATA_SET_RSP,        LTP_LE_ADVERTISE_DATA_SET_RSP_LENGTH,         LTP_LE_ADVERTISE_DATA_SET_RSP_FLAGS         },
    { LTP_LE_SCAN_RSP,                      LTP_LE_SCAN_RSP_LENGTH,                       LTP_LE_SCAN_RSP_FLAGS                       },
    { LTP_LE_SCAN_INFO,                     LTP_LE_SCAN_INFO_LENGTH,                      LTP_LE_SCAN_INFO_FLAGS                      },
    { LTP_LE_MODIFY_WHITELIST_RSP,          LTP_LE_MODIFY_WHITELIST_RSP_LENGTH,           LTP_LE_MODIFY_WHITELIST_RSP_FLAGS           },
    { LTP_LE_CONNECTION_UPDATE_RSP,         LTP_LE_CONNECTION_UPDATE_RSP_LENGTH,          LTP_LE_CONNECTION_UPDATE_RSP_FLAGS          },
    { LTP_LE_CONNECTION_UPDATE_CNF,         LTP_LE_CONNECTION_UPDATE_CNF_LENGTH,          LTP_LE_CONNECTION_UPDATE_CNF_FLAGS          },
    { LTP_LE_CONNECTION_PARAMETER_INFO,     LTP_LE_CONNECTION_PARAMETER_INFO_LENGTH,      LTP_LE_CONNECTION_PARAMETER_INFO_FLAGS      },
    { LTP_GATT_SERVER_STORE_CNF,            LTP_GATT_SERVER_STORE_CNF_LENGTH,             LTP_GATT_SERVER_STORE_CNF_FLAGS             },
    { LTP_AUTH_RESULT_EXT_CNF,              LTP_AUTH_RESULT_EXT_CNF_LENGTH,               LTP_AUTH_RESULT_EXT_CNF_FLAGS               },
    { LTP_AUTH_RESULT_REQUEST_EXT_CNF,      LTP_AUTH_RESULT_REQUEST_EXT_CNF_LENGTH,       LTP_AUTH_RESULT_REQUEST_EXT_CNF_FLAGS       },
    { LTP_GATT_SECURITY_RSP,                LTP_GATT_SECURITY_RSP_LENGTH,                 LTP_GATT_SECURITY_RSP_FLAGS                 },
    { LTP_GATT_MTU_INFO,                    LTP_GATT_MTU_INFO_LENGTH,                     LTP_GATT_MTU_INFO_FLAGS                     },
    { LTP_GATT_ATTRIBUTE_WRITE_COMMAND_INFO, LTP_GATT_ATTRIBUTE_WRITE_COMMAND_INFO_LENGTH, LTP_GATT_ATTRIBUTE_WRITE_COMMAND_INFO_FLAGS },

#if BREDR_SUPPORT
    { LTP_ACL_BD_RESOLVED_INFO,             LTP_ACL_BD_RESOLVED_INFO_LENGTH,              LTP_ACL_BD_RESOLVED_INFO_FLAGS              },
    { LTP_AUTH_DELETE_RSP,                  LTP_AUTH_DELETE_RSP_LENGTH,                   LTP_AUTH_DELETE_RSP_FLAGS                   },
    { LTP_AUTH_LIST_RSP,                    LTP_AUTH_LIST_RSP_LENGTH,                     LTP_AUTH_LIST_RSP_FLAGS                     },
    { LTP_AUTH_LIST_INFO,                   LTP_AUTH_LIST_INFO_LENGTH,                    LTP_AUTH_LIST_INFO_FLAGS                    },

    { LTP_CONNECT_MDL_RSP,                  LTP_CONNECT_MDL_RSP_LENGTH,                   LTP_CONNECT_MDL_RSP_FLAGS                   },
    { LTP_AUTH_REQUEST_CNF,                 LTP_AUTH_REQUEST_CNF_LENGTH,                  LTP_AUTH_REQUEST_CNF_FLAGS                  },
    { LTP_RELEASE_MDEP_RSP,                 LTP_RELEASE_MDEP_RSP_LENGTH,                  LTP_RELEASE_MDEP_RSP_FLAGS                  },
    { LTP_INQUIRY_RSP,                      LTP_INQUIRY_RSP_LENGTH,                       LTP_INQUIRY_RSP_FLAGS                       },
    { LTP_INQUIRY_DEVICE_INFO,              LTP_INQUIRY_DEVICE_INFO_LENGTH,               LTP_INQUIRY_DEVICE_INFO_FLAGS               },
    { LTP_DID_DEVICE_INFO,                  LTP_DID_DEVICE_INFO_LENGTH,                   LTP_DID_DEVICE_INFO_FLAGS                   },
    { LTP_AUTH_RSP,                         LTP_AUTH_RSP_LENGTH,                          LTP_AUTH_RSP_FLAGS                          },
    { LTP_AUTH_RESULT_REQUEST_CNF,          LTP_AUTH_RESULT_REQUEST_CNF_LENGTH,           LTP_AUTH_RESULT_REQUEST_CNF_FLAGS           },
    { LTP_USER_CONF_REQUEST_CNF,            LTP_USER_CONF_REQUEST_CNF_LENGTH,             LTP_USER_CONF_REQUEST_CNF_FLAGS             },
    { LTP_AUTH_RESULT_CNF,                  LTP_AUTH_RESULT_CNF_LENGTH,                   LTP_AUTH_RESULT_CNF_FLAGS                   },
    { LTP_CONFIG_TUNNEL_RSP,                LTP_CONFIG_TUNNEL_RSP_LENGTH,                 LTP_CONFIG_TUNNEL_RSP_FLAGS                 },
    { LTP_CONFIG_TUNNEL_INFO,               LTP_CONFIG_TUNNEL_INFO_LENGTH,                LTP_CONFIG_TUNNEL_INFO_FLAGS                },
    { LTP_RADIO_MODE_SET_RSP,               LTP_RADIO_MODE_SET_RSP_LENGTH,                LTP_RADIO_MODE_SET_RSP_FLAGS                },
    { LTP_KEYPRESS_NOTIFICATION_RSP,        LTP_KEYPRESS_NOTIFICATION_RSP_LENGTH,         LTP_KEYPRESS_NOTIFICATION_RSP_FLAGS         },
    { LTP_KEYPRESS_NOTIFICATION_INFO,       LTP_KEYPRESS_NOTIFICATION_INFO_LENGTH,        LTP_KEYPRESS_NOTIFICATION_INFO_FLAGS        },
    { LTP_LOCAL_OOB_RSP,                    LTP_LOCAL_OOB_RSP_LENGTH,                     LTP_LOCAL_OOB_RSP_FLAGS                     },
    { LTP_DEVICE_NAME_RSP,                  LTP_DEVICE_NAME_RSP_LENGTH,                   LTP_DEVICE_NAME_RSP_FLAGS                   },
    { LTP_SPP_DISCOVERY_RSP,                LTP_SPP_DISCOVERY_RSP_LENGTH,                 LTP_SPP_DISCOVERY_RSP_FLAGS                 },
    { LTP_SPP_ENDPOINT_INFO,                LTP_SPP_ENDPOINT_INFO_LENGTH,                 LTP_SPP_ENDPOINT_INFO_FLAGS                 },
    { LTP_REGISTER_SPP_MDEP_RSP,            LTP_REGISTER_SPP_MDEP_RSP_LENGTH,             LTP_REGISTER_SPP_MDEP_RSP_FLAGS             },
    { LTP_AUTHORIZATION_REQ_CNF,            LTP_AUTHORIZATION_REQ_CNF_LENGTH,             LTP_AUTHORIZATION_REQ_CNF_FLAGS             },
    { LTP_DEVICE_CONFIG_DEVICE_SET_RSP,     LTP_DEVICE_CONFIG_DEVICE_SET_RSP_LENGTH,      LTP_DEVICE_CONFIG_DEVICE_SET_RSP_FLAGS      },
    { LTP_DEVICE_CONFIG_DID_SET_RSP,        LTP_DEVICE_CONFIG_DID_SET_RSP_LENGTH,         LTP_DEVICE_CONFIG_DID_SET_RSP_FLAGS         },
    { LTP_DEVICE_CONFIG_SPP_SET_RSP,        LTP_DEVICE_CONFIG_SPP_SET_RSP_LENGTH,         LTP_DEVICE_CONFIG_SPP_SET_RSP_FLAGS         },
    { LTP_DEVICE_CONFIG_PAGESCAN_SET_RSP,   LTP_DEVICE_CONFIG_PAGESCAN_SET_RSP_LENGTH,    LTP_DEVICE_CONFIG_PAGESCAN_SET_RSP_FLAGS    },
    { LTP_DEVICE_CONFIG_LINKPOLICY_SET_RSP, LTP_DEVICE_CONFIG_LINKPOLICY_SET_RSP_LENGTH,  LTP_DEVICE_CONFIG_LINKPOLICY_SET_RSP_FLAGS  },
    { LTP_DEVICE_CONFIG_MAXTXPOWER_SET_RSP, LTP_DEVICE_CONFIG_MAXTXPOWER_SET_RSP_LENGTH,  LTP_DEVICE_CONFIG_MAXTXPOWER_SET_RSP_FLAGS  },

/* DATA */
    { LTP_DATA_UNSEGMENTED,                 LTP_DATA_UNSEGMENTED_LENGTH,                  LTP_DATA_UNSEGMENTED_FLAGS                  },
    { LTP_DATA_START_SEGMENT,               LTP_DATA_START_SEGMENT_LENGTH,                LTP_DATA_START_SEGMENT_FLAGS                },
    { LTP_DATA_END_SEGMENT,                 LTP_DATA_END_SEGMENT_LENGTH,                  LTP_DATA_END_SEGMENT_FLAGS                  },
    { LTP_DATA_CONTINUE_SEGMENT,            LTP_DATA_CONTINUE_SEGMENT_LENGTH,             LTP_DATA_CONTINUE_SEGMENT_FLAGS             },
    { LTP_GATT_SDP_DISCOVERY_RSP,           LTP_GATT_SDP_DISCOVERY_RSP_LENGTH,            LTP_GATT_SDP_DISCOVERY_RSP_FLAGS            },
    { LTP_GATT_SDP_DISCOVERY_INFO,          LTP_GATT_SDP_DISCOVERY_INFO_LENGTH,           LTP_GATT_SDP_DISCOVERY_INFO_FLAGS           },
    { LTP_ACL_CONFIG_LINKPOLICY_RSP,        LTP_ACL_CONFIG_LINKPOLICY_RSP_LENGTH,         LTP_ACL_CONFIG_LINKPOLICY_RSP_FLAGS         },
    { LTP_ACL_CONFIG_SNIFFMODE_RSP,         LTP_ACL_CONFIG_SNIFFMODE_RSP_LENGTH,          LTP_ACL_CONFIG_SNIFFMODE_RSP_FLAGS          },
    { LTP_ACL_SNIFF_SUBRATE_INFO,           LTP_ACL_SNIFF_SUBRATE_INFO_LENGTH,            LTP_ACL_SNIFF_SUBRATE_INFO_FLAGS            },
    { LTP_ACL_CONFIG_LINKSTATUS_RSP,        LTP_ACL_CONFIG_LINKSTATUS_RSP_LENGTH,         LTP_ACL_CONFIG_LINKSTATUS_RSP_FLAGS         },
    { LTP_ACL_LINKSTATUS_INFO,              LTP_ACL_LINKSTATUS_INFO_LENGTH,               LTP_ACL_LINKSTATUS_INFO_FLAGS               },
    { LTP_LEGACY_REMOTE_OOB_REQUEST_CNF,    LTP_LEGACY_REMOTE_OOB_REQUEST_CNF_LENGTH,     LTP_LEGACY_REMOTE_OOB_REQUEST_CNF_FLAGS            },
    //{ LTP_DEVICE_CONFIG_HDP_SET_RSP,        LTP_DEVICE_CONFIG_HDP_SET_RSP_LENGTH,         LTP_DEVICE_CONFIG_HDP_SET_RSP_FLAGS         },
    //{ LTP_DEVICE_CONFIG_SECURITY_SET_RSP,   LTP_DEVICE_CONFIG_SECURITY_SET_RSP_LENGTH,    LTP_DEVICE_CONFIG_SECURITY_SET_RSP_FLAGS    },
    //{ LTP_RECONNECT_MDL_RSP,                LTP_RECONNECT_MDL_RSP_LENGTH,                 LTP_RECONNECT_MDL_RSP_FLAGS                 },
    //{ LTP_RECONNECT_MDL_CNF,                LTP_RECONNECT_MDL_CNF_LENGTH,                 LTP_RECONNECT_MDL_CNF_FLAGS                 },
    //{ LTP_EXIT_INFO,                        LTP_EXIT_INFO_LENGTH,                         LTP_EXIT_INFO_FLAGS                         },
    //{ LTP_HDP_DISCOVERY_RSP,                LTP_HDP_DISCOVERY_RSP_LENGTH,                 LTP_HDP_DISCOVERY_RSP_FLAGS                 },
    //{ LTP_HDP_SERVICE_INFO,                 LTP_HDP_SERVICE_INFO_LENGTH,                  LTP_HDP_SERVICE_INFO_FLAGS                  },
    //{ LTP_REGISTER_HDP_MDEP_RSP,            LTP_REGISTER_HDP_MDEP_RSP_LENGTH,             LTP_REGISTER_HDP_MDEP_RSP_FLAGS             },
    //{ LTP_HDP_ENDPOINT_INFO,                LTP_HDP_ENDPOINT_INFO_LENGTH,                 LTP_HDP_ENDPOINT_INFO_FLAGS                 },
    //{ LTP_GATT_SERVICE_RELEASE_RSP,         LTP_GATT_SERVICE_RELEASE_RSP_LENGTH,          LTP_GATT_SERVICE_RELEASE_RSP_FLAGS          },
    //{ LTP_LE_PRIVACY_MODE_RSP,              LTP_LE_PRIVACY_MODE_RSP_LENGTH,               LTP_LE_PRIVACY_MODE_RSP_FLAGS               },

#endif

    
    /* MDH -> MDC */
    { LTP_CREATE_MDL_IND,                   LTP_CREATE_MDL_IND_LENGTH,                    LTP_CREATE_MDL_IND_FLAGS                    },
    { LTP_DISCONNECT_MDL_REQ,               LTP_DISCONNECT_MDL_REQ_LENGTH,                LTP_DISCONNECT_MDL_REQ_FLAGS                },
    { LTP_DISCONNECT_MDL_IND,               LTP_DISCONNECT_MDL_IND_LENGTH,                LTP_DISCONNECT_MDL_IND_FLAGS                },
    { LTP_EXIT_REQ,                         LTP_EXIT_REQ_LENGTH,                          LTP_EXIT_REQ_FLAGS                          },
    { LTP_RESET_REQ,                        LTP_RESET_REQ_LENGTH,                         LTP_RESET_REQ_FLAGS                         },
    { LTP_PASSKEY_REQUEST_IND,              LTP_PASSKEY_REQUEST_IND_LENGTH,               LTP_PASSKEY_REQUEST_IND_FLAGS               },
    { LTP_REMOTE_OOB_REQUEST_IND,           LTP_REMOTE_OOB_REQUEST_IND_LENGTH,            LTP_REMOTE_OOB_REQUEST_IND_FLAGS            },
    { LTP_PAIRABLE_MODE_SET_REQ,            LTP_PAIRABLE_MODE_SET_REQ_LENGTH,             LTP_PAIRABLE_MODE_SET_REQ_FLAGS             },
    { LTP_PASSKEY_REQ_REPLY_REQ,            LTP_PASSKEY_REQ_REPLY_REQ_LENGTH,             LTP_PASSKEY_REQ_REPLY_REQ_FLAGS             },
    { LTP_CONNECT_GATT_MDL_REQ,             LTP_CONNECT_GATT_MDL_REQ_LENGTH,              LTP_CONNECT_GATT_MDL_REQ_FLAGS              },
    { LTP_GATT_SERVICE_REGISTER_REQ,        LTP_GATT_SERVICE_REGISTER_REQ_LENGTH,         LTP_GATT_SERVICE_REGISTER_REQ_FLAGS         },
    { LTP_GATT_ATTRIBUTE_UPDATE_REQ,        LTP_GATT_ATTRIBUTE_UPDATE_REQ_LENGTH,         LTP_GATT_ATTRIBUTE_UPDATE_REQ_FLAGS         },
    { LTP_GATT_ATTRIBUTE_UPDATE_STATUS_IND, LTP_GATT_ATTRIBUTE_UPDATE_STATUS_IND_LENGTH,  LTP_GATT_ATTRIBUTE_UPDATE_STATUS_IND_FLAGS  },
    { LTP_GATT_ATTRIBUTE_READ_IND,          LTP_GATT_ATTRIBUTE_READ_IND_LENGTH,           LTP_GATT_ATTRIBUTE_READ_IND_FLAGS           },
    { LTP_GATT_ATTRIBUTE_WRITE_IND,         LTP_GATT_ATTRIBUTE_WRITE_IND_LENGTH,          LTP_GATT_ATTRIBUTE_WRITE_IND_FLAGS          },
    { LTP_GATT_DISCOVERY_REQ,               LTP_GATT_DISCOVERY_REQ_LENGTH,                LTP_GATT_DISCOVERY_REQ_FLAGS                },
    { LTP_GATT_DISCOVERY_IND,               LTP_GATT_DISCOVERY_IND_LENGTH,                LTP_GATT_DISCOVERY_IND_FLAGS                },
    { LTP_GATT_ATTRIBUTE_READ_REQ,          LTP_GATT_ATTRIBUTE_READ_REQ_LENGTH,           LTP_GATT_ATTRIBUTE_READ_REQ_FLAGS           },
    { LTP_GATT_ATTRIBUTE_WRITE_REQ,         LTP_GATT_ATTRIBUTE_WRITE_REQ_LENGTH,          LTP_GATT_ATTRIBUTE_WRITE_REQ_FLAGS          },
    { LTP_GATT_ATTRIBUTE_IND,               LTP_GATT_ATTRIBUTE_IND_LENGTH,                LTP_GATT_ATTRIBUTE_IND_FLAGS                },
    { LTP_LE_ADVERTISE_REQ,                 LTP_LE_ADVERTISE_REQ_LENGTH,                  LTP_LE_ADVERTISE_REQ_FLAGS                  },
    { LTP_LE_ADVERTISE_PARAMETER_SET_REQ,   LTP_LE_ADVERTISE_PARAMETER_SET_REQ_LENGTH,    LTP_LE_ADVERTISE_PARAMETER_SET_REQ_FLAGS    },
    { LTP_LE_ADVERTISE_DATA_SET_REQ,        LTP_LE_ADVERTISE_DATA_SET_REQ_LENGTH,         LTP_LE_ADVERTISE_DATA_SET_REQ_FLAGS         },
    { LTP_LE_SCAN_REQ,                      LTP_LE_SCAN_REQ_LENGTH,                       LTP_LE_SCAN_REQ_FLAGS                       },
    { LTP_LE_MODIFY_WHITELIST_REQ,          LTP_LE_MODIFY_WHITELIST_REQ_LENGTH,           LTP_LE_MODIFY_WHITELIST_REQ_FLAGS           },
    { LTP_LE_CONNECTION_UPDATE_REQ,         LTP_LE_CONNECTION_UPDATE_REQ_LENGTH,          LTP_LE_CONNECTION_UPDATE_REQ_FLAGS          },
    { LTP_LE_CONNECTION_UPDATE_IND,         LTP_LE_CONNECTION_UPDATE_IND_LENGTH,          LTP_LE_CONNECTION_UPDATE_IND_FLAGS          },
    { LTP_GATT_SERVER_STORE_IND,            LTP_GATT_SERVER_STORE_IND_LENGTH,             LTP_GATT_SERVER_STORE_IND_FLAGS             },
    { LTP_AUTH_RESULT_EXT_IND,              LTP_AUTH_RESULT_EXT_IND_LENGTH,               LTP_AUTH_RESULT_EXT_IND_FLAGS               },
    { LTP_AUTH_RESULT_REQUEST_EXT_IND,      LTP_AUTH_RESULT_REQUEST_EXT_IND_LENGTH,       LTP_AUTH_RESULT_REQUEST_EXT_IND_FLAGS       },
    { LTP_GATT_SECURITY_REQ,                LTP_GATT_SECURITY_REQ_LENGTH,                 LTP_GATT_SECURITY_REQ_FLAGS                 },
//bredr
#if BREDR_SUPPORT
    { LTP_AUTH_DELETE_REQ,                  LTP_AUTH_DELETE_REQ_LENGTH,                   LTP_AUTH_DELETE_REQ_FLAGS                   },
    { LTP_AUTH_LIST_REQ,                    LTP_AUTH_LIST_REQ_LENGTH,                     LTP_AUTH_LIST_REQ_FLAGS                     },

    { LTP_CONNECT_MDL_REQ,                  LTP_CONNECT_MDL_REQ_LENGTH,                   LTP_CONNECT_MDL_REQ_FLAGS                   },
    { LTP_AUTH_REQUEST_IND,                 LTP_AUTH_REQUEST_IND_LENGTH,                  LTP_AUTH_REQUEST_IND_FLAGS                  },
    { LTP_RELEASE_MDEP_REQ,                 LTP_RELEASE_MDEP_REQ_LENGTH,                  LTP_RELEASE_MDEP_REQ_FLAGS                  },
    { LTP_INQUIRY_REQ,                      LTP_INQUIRY_REQ_LENGTH,                       LTP_INQUIRY_REQ_FLAGS                       },
    { LTP_AUTH_REQ,                         LTP_AUTH_REQ_LENGTH,                          LTP_AUTH_REQ_FLAGS                          },
    { LTP_AUTH_RESULT_REQUEST_IND,          LTP_AUTH_RESULT_REQUEST_IND_LENGTH,           LTP_AUTH_RESULT_REQUEST_IND_FLAGS           },
    { LTP_USER_CONF_REQUEST_IND,            LTP_USER_CONF_REQUEST_IND_LENGTH,             LTP_USER_CONF_REQUEST_IND_FLAGS             },
    { LTP_AUTH_RESULT_IND,                  LTP_AUTH_RESULT_IND_LENGTH,                   LTP_AUTH_RESULT_IND_FLAGS                   },
    { LTP_CONFIG_TUNNEL_REQ,                LTP_CONFIG_TUNNEL_REQ_LENGTH,                 LTP_CONFIG_TUNNEL_REQ_FLAGS                 },
    { LTP_RADIO_MODE_SET_REQ,               LTP_RADIO_MODE_SET_REQ_LENGTH,                LTP_RADIO_MODE_SET_REQ_FLAGS                },
    { LTP_KEYPRESS_NOTIFICATION_REQ,        LTP_KEYPRESS_NOTIFICATION_REQ_LENGTH,         LTP_KEYPRESS_NOTIFICATION_REQ_FLAGS         },
    { LTP_LOCAL_OOB_REQ,                    LTP_LOCAL_OOB_REQ_LENGTH,                     LTP_LOCAL_OOB_REQ_FLAGS                     },
    { LTP_DEVICE_NAME_REQ,                  LTP_DEVICE_NAME_REQ_LENGTH,                   LTP_DEVICE_NAME_REQ_FLAGS                   },
    { LTP_SPP_DISCOVERY_REQ,                LTP_SPP_DISCOVERY_REQ_LENGTH,                 LTP_SPP_DISCOVERY_REQ_FLAGS                 },
    { LTP_REGISTER_SPP_MDEP_REQ,            LTP_REGISTER_SPP_MDEP_REQ_LENGTH,             LTP_REGISTER_SPP_MDEP_REQ_FLAGS             },
    { LTP_AUTHORIZATION_REQ_IND,            LTP_AUTHORIZATION_REQ_IND_LENGTH,             LTP_AUTHORIZATION_REQ_IND_FLAGS             },
    { LTP_DEVICE_CONFIG_DEVICE_SET_REQ,     LTP_DEVICE_CONFIG_DEVICE_SET_REQ_LENGTH,      LTP_DEVICE_CONFIG_DEVICE_SET_REQ_FLAGS      },
    { LTP_DEVICE_CONFIG_DID_SET_REQ,        LTP_DEVICE_CONFIG_DID_SET_REQ_LENGTH,         LTP_DEVICE_CONFIG_DID_SET_REQ_FLAGS         },
    { LTP_DEVICE_CONFIG_SPP_SET_REQ,        LTP_DEVICE_CONFIG_SPP_SET_REQ_LENGTH,         LTP_DEVICE_CONFIG_SPP_SET_REQ_FLAGS         },
    { LTP_DEVICE_CONFIG_PAGESCAN_SET_REQ,   LTP_DEVICE_CONFIG_PAGESCAN_SET_REQ_LENGTH,    LTP_DEVICE_CONFIG_PAGESCAN_SET_REQ_FLAGS    },
    { LTP_DEVICE_CONFIG_LINKPOLICY_SET_REQ, LTP_DEVICE_CONFIG_LINKPOLICY_SET_REQ_LENGTH,  LTP_DEVICE_CONFIG_LINKPOLICY_SET_REQ_FLAGS  },
    { LTP_DEVICE_CONFIG_MAXTXPOWER_SET_REQ, LTP_DEVICE_CONFIG_MAXTXPOWER_SET_REQ_LENGTH,  LTP_DEVICE_CONFIG_MAXTXPOWER_SET_REQ_FLAGS  },        
    { LTP_GATT_SDP_DISCOVERY_REQ,           LTP_GATT_SDP_DISCOVERY_REQ_LENGTH,            LTP_GATT_SDP_DISCOVERY_REQ_FLAGS            },
    { LTP_ACL_CONFIG_LINKPOLICY_REQ,        LTP_ACL_CONFIG_LINKPOLICY_REQ_LENGTH,         LTP_ACL_CONFIG_LINKPOLICY_REQ_FLAGS         },
    { LTP_ACL_CONFIG_SNIFFMODE_REQ,         LTP_ACL_CONFIG_SNIFFMODE_REQ_LENGTH,          LTP_ACL_CONFIG_SNIFFMODE_REQ_FLAGS          },
    { LTP_ACL_CONFIG_LINKSTATUS_REQ,        LTP_ACL_CONFIG_LINKSTATUS_REQ_LENGTH,         LTP_ACL_CONFIG_LINKSTATUS_REQ_FLAGS         },
    { LTP_LEGACY_REMOTE_OOB_REQUEST_IND,    LTP_LEGACY_REMOTE_OOB_REQUEST_IND_LENGTH,     LTP_LEGACY_REMOTE_OOB_REQUEST_IND_FLAGS            },

    //{ LTP_RECONNECT_MDL_REQ,                LTP_RECONNECT_MDL_REQ_LENGTH,                 LTP_RECONNECT_MDL_REQ_FLAGS                 },
    //{ LTP_RECONNECT_MDL_IND,                LTP_RECONNECT_MDL_IND_LENGTH,                 LTP_RECONNECT_MDL_IND_FLAGS                 },
    //{ LTP_REGISTER_HDP_MDEP_REQ,            LTP_REGISTER_HDP_MDEP_REQ_LENGTH,             LTP_REGISTER_HDP_MDEP_REQ_FLAGS             },
    //{ LTP_HDP_DISCOVERY_REQ,                LTP_HDP_DISCOVERY_REQ_LENGTH,                 LTP_HDP_DISCOVERY_REQ_FLAGS                 },
    //{ LTP_DEVICE_CONFIG_SECURITY_SET_REQ,   LTP_DEVICE_CONFIG_SECURITY_SET_REQ_LENGTH,    LTP_DEVICE_CONFIG_SECURITY_SET_REQ_FLAGS    },
    //{ LTP_DEVICE_CONFIG_HDP_SET_REQ,        LTP_DEVICE_CONFIG_HDP_SET_REQ_LENGTH,         LTP_DEVICE_CONFIG_HDP_SET_REQ_FLAGS         },
    //{ LTP_REGISTER_HID_MDEP_REQ,            LTP_REGISTER_HID_MDEP_REQ_LENGTH,             LTP_REGISTER_HID_MDEP_REQ_FLAGS             },
    //{ LTP_REGISTER_HID_MDEP_RSP,            LTP_REGISTER_HID_MDEP_RSP_LENGTH,             LTP_REGISTER_HID_MDEP_RSP_FLAGS             },  /* MDC -> MDH */
    //{ LTP_HID_MESSAGE_REQ,                  LTP_HID_MESSAGE_REQ_LENGTH,                   LTP_HID_MESSAGE_REQ_FLAGS                   },
    //{ LTP_HID_MESSAGE_RSP,                  LTP_HID_MESSAGE_RSP_LENGTH,                   LTP_HID_MESSAGE_RSP_FLAGS                   },  /* MDC -> MDH */
    //{ LTP_HID_MESSAGE_CONF,                 LTP_HID_MESSAGE_CONF_LENGTH,                  LTP_HID_MESSAGE_CONF_FLAGS                  },
    //{ LTP_HID_MESSAGE_IND,                  LTP_HID_MESSAGE_IND_LENGTH,                   LTP_HID_MESSAGE_IND_FLAGS                   },  /* MDC -> MDH */
    //{ LTP_REGISTER_OBEX_MDEP_REQ,           LTP_REGISTER_OBEX_MDEP_REQ_LENGTH,            LTP_REGISTER_OBEX_MDEP_REQ_FLAGS            },
    //{ LTP_REGISTER_OBEX_MDEP_RSP,           LTP_REGISTER_OBEX_MDEP_RSP_LENGTH,            LTP_REGISTER_OBEX_MDEP_RSP_FLAGS            },  /* MDC -> MDH */
    //{ LTP_OBEX_DISCOVERY_REQ,               LTP_OBEX_DISCOVERY_REQ_LENGTH,                LTP_OBEX_DISCOVERY_REQ_FLAGS                          },
    //{ LTP_OBEX_DISCOVERY_RSP,               LTP_OBEX_DISCOVERY_RSP_LENGTH,                LTP_OBEX_DISCOVERY_RSP_FLAGS                          },  /* MDC -> MDH */
    //{ LTP_OBEX_ENDPOINT_CONF,               LTP_OBEX_ENDPOINT_CONF_LENGTH,                LTP_OBEX_ENDPOINT_CONF_FLAGS                          },
    //{ LTP_OBEX_ENDPOINT_IND,                LTP_OBEX_ENDPOINT_IND_LENGTH,                 LTP_OBEX_ENDPOINT_IND_FLAGS                           },  /* MDC -> MDH */
    //{ LTP_GATT_SERVICE_RELEASE_REQ,         LTP_GATT_SERVICE_RELEASE_REQ_LENGTH,          LTP_GATT_SERVICE_RELEASE_REQ_FLAGS          },
    //{ LTP_LE_PRIVACY_MODE_REQ,              LTP_LE_PRIVACY_MODE_REQ_LENGTH,               LTP_LE_PRIVACY_MODE_REQ_FLAGS               },

#endif
    
    { LTP_EXTEND_COMMAND,                   LTP_EXTEND_COMMAND_LENGTH,                    LTP_EXTEND_COMMAND_FLAGS                 },
    //{ LTP_OPCODE_RESERVED,                  0,                                            0                                           },
};
const uint8_t CmdTableSize = (uint8_t)(sizeof(LTPCmdTable) / sizeof(LTPCmdInfo));

const LTPCmdInfo  LTPSubCmdTable[] =
{
    { LTP_SUB_SET_RANDOM_ADDRESS_RSP,            LTP_SUB_SET_RANDOM_ADDRESS_RSP_LENGTH,            LTP_SUB_SET_RANDOM_ADDRESS_RSP_FLAGS            },
    { LTP_SUB_DEVICE_CONFIG_DEVICE_NAME_SET_RSP, LTP_SUB_DEVICE_CONFIG_DEVICE_NAME_SET_RSP_LENGTH, LTP_SUB_DEVICE_CONFIG_DEVICE_NAME_SET_RSP_FLAGS },
    { LTP_SUB_DEVICE_CONFIG_SECURITY_SET_RSP,    LTP_SUB_DEVICE_CONFIG_SECURITY_SET_RSP_LENGTH,    LTP_SUB_DEVICE_CONFIG_SECURITY_SET_RSP_FLAGS    },
    { LTP_SUB_DEVICE_CONFIG_STORE_SET_RSP,       LTP_SUB_DEVICE_CONFIG_STORE_SET_RSP_LENGTH,       LTP_SUB_DEVICE_CONFIG_STORE_SET_RSP_FLAGS       },
    { LTP_SUB_DEVICE_CONFIG_APPEARANCE_SET_RSP,  LTP_SUB_DEVICE_CONFIG_APPEARANCE_SET_RSP_LENGTH,  LTP_SUB_DEVICE_CONFIG_APPEARANCE_SET_RSP_FLAGS  },
    { LTP_SUB_DEVICE_CONFIG_PER_PREF_CONN_PARAM_SET_RSP,   LTP_SUB_DEVICE_CONFIG_PER_PREF_CONN_PARAM_SET_RSP_LENGTH,    LTP_SUB_DEVICE_CONFIG_PER_PREF_CONN_PARAM_SET_RSP_FLAGS    },
    { LTP_SUB_SET_LE_TX_POWER_RSP,               LTP_SUB_SET_LE_TX_POWER_RSP_LENGTH,               LTP_SUB_SET_LE_TX_POWER_RSP_FLAGS               },
    { LTP_SUB_SET_DATA_LENGTH_RSP,               LTP_SUB_SET_DATA_LENGTH_RSP_LENGTH,               LTP_SUB_SET_DATA_LENGTH_RSP_FLAGS               },
    { LTP_SUB_DATA_LENGTH_CHANGE_INFO,           LTP_SUB_DATA_LENGTH_CHANGE_INFO_LENGTH,           LTP_SUB_DATA_LENGTH_CHANGE_INFO_FLAGS           },
    { LTP_SUB_DOWNLOAD_SERVICE_DATABASE_RSP,     LTP_SUB_DOWNLOAD_SERVICE_DATABASE_RSP_LENGTH,     LTP_SUB_DOWNLOAD_SERVICE_DATABASE_RSP_FLAGS     },
    { LTP_SUB_CLEAR_SERVICE_DATABASE_RSP,        LTP_SUB_CLEAR_SERVICE_DATABASE_RSP_LENGTH,        LTP_SUB_CLEAR_SERVICE_DATABASE_RSP_FLAGS        },
    { LTP_SUB_SET_TRACE_LEVEL_RSP,		         LTP_SUB_SET_TRACE_LEVEL_RSP_LENGTH, 	           LTP_SUB_SET_TRACE_LEVEL_RSP_FLAGS 	           },
    { LTP_SUB_GATT_ATTRIBUTE_PREPARE_WRITE_CNF,	 LTP_SUB_GATT_ATTRIBUTE_PREPARE_WRITE_CNF_LENGTH,  LTP_SUB_GATT_ATTRIBUTE_PREPARE_WRITE_CNF_FLAGS  },
    { LTP_SUB_GATT_ATTRIBUTE_PREPARE_WRITE_RSP,	 LTP_SUB_GATT_ATTRIBUTE_PREPARE_WRITE_RSP_LENGTH,  LTP_SUB_GATT_ATTRIBUTE_PREPARE_WRITE_RSP_FLAGS  },
    { LTP_SUB_GATT_ATTRIBUTE_EXECUTE_WRITE_CNF,	 LTP_SUB_GATT_ATTRIBUTE_EXECUTE_WRITE_CNF_LENGTH,  LTP_SUB_GATT_ATTRIBUTE_EXECUTE_WRITE_CNF_FLAGS  },
    { LTP_SUB_GATT_ATTRIBUTE_EXECUTE_WRITE_RSP,	 LTP_SUB_GATT_ATTRIBUTE_EXECUTE_WRITE_RSP_LENGTH,  LTP_SUB_GATT_ATTRIBUTE_EXECUTE_WRITE_RSP_FLAGS  },
        
    { LTP_SUB_SET_RANDOM_ADDRESS_REQ,            LTP_SUB_SET_RANDOM_ADDRESS_LENGTH,                 LTP_SUB_SET_RANDOM_ADDRESS_FLAGS               },
    { LTP_SUB_DEVICE_CONFIG_DEVICE_NAME_SET_REQ, LTP_SUB_DEVICE_CONFIG_DEVICE_NAME_SET_REQ_LENGTH, LTP_SUB_DEVICE_CONFIG_DEVICE_NAME_SET_REQ_FLAGS },
    { LTP_SUB_DEVICE_CONFIG_SECURITY_SET_REQ,    LTP_SUB_DEVICE_CONFIG_SECURITY_SET_REQ_LENGTH,    LTP_SUB_DEVICE_CONFIG_SECURITY_SET_REQ_FLAGS    },
    { LTP_SUB_DEVICE_CONFIG_STORE_SET_REQ,       LTP_SUB_DEVICE_CONFIG_STORE_SET_REQ_LENGTH,       LTP_SUB_DEVICE_CONFIG_STORE_SET_REQ_FLAGS       },
    { LTP_SUB_DEVICE_CONFIG_APPEARANCE_SET_REQ,  LTP_SUB_DEVICE_CONFIG_APPEARANCE_SET_REQ_LENGTH,  LTP_SUB_DEVICE_CONFIG_APPEARANCE_SET_REQ_FLAGS  },
    { LTP_SUB_DEVICE_CONFIG_PER_PREF_CONN_PARAM_SET_REQ,   LTP_SUB_DEVICE_CONFIG_PER_PREF_CONN_PARAM_SET_REQ_LENGTH,    LTP_SUB_DEVICE_CONFIG_PER_PREF_CONN_PARAM_SET_REQ_FLAGS    },
    { LTP_SUB_SET_LE_TX_POWER_REQ,               LTP_SUB_SET_LE_TX_POWER_REQ_LENGTH,               LTP_SUB_SET_LE_TX_POWER_REQ_FLAGS               },
    { LTP_SUB_SET_DATA_LENGTH_REQ,               LTP_SUB_SET_DATA_LENGTH_REQ_LENGTH,               LTP_SUB_SET_DATA_LENGTH_REQ_FLAGS               },
    { LTP_SUB_DOWNLOAD_SERVICE_DATABASE_REQ,     LTP_SUB_DOWNLOAD_SERVICE_DATABASE_REQ_LENGTH,     LTP_SUB_DOWNLOAD_SERVICE_DATABASE_REQ_FLAGS     },
    { LTP_SUB_CLEAR_SERVICE_DATABASE_REQ,        LTP_SUB_CLEAR_SERVICE_DATABASE_REQ_LENGTH,        LTP_SUB_CLEAR_SERVICE_DATABASE_REQ_FLAGS        },
	{ LTP_SUB_SET_TRACE_LEVEL_REQ,		         LTP_SUB_SET_TRACE_LEVEL_REQ_LENGTH, 	           LTP_SUB_SET_TRACE_LEVEL_REQ_FLAGS 	           },
    { LTP_SUB_GATT_ATTRIBUTE_PREPARE_WRITE_IND,	 LTP_SUB_GATT_ATTRIBUTE_PREPARE_WRITE_IND_LENGTH,  LTP_SUB_GATT_ATTRIBUTE_PREPARE_WRITE_IND_FLAGS  },
    { LTP_SUB_GATT_ATTRIBUTE_PREPARE_WRITE_REQ,	 LTP_SUB_GATT_ATTRIBUTE_PREPARE_WRITE_REQ_LENGTH,  LTP_SUB_GATT_ATTRIBUTE_PREPARE_WRITE_REQ_FLAGS  },
    { LTP_SUB_GATT_ATTRIBUTE_EXECUTE_WRITE_IND,	 LTP_SUB_GATT_ATTRIBUTE_EXECUTE_WRITE_IND_LENGTH,  LTP_SUB_GATT_ATTRIBUTE_EXECUTE_WRITE_IND_FLAGS  },
    { LTP_SUB_GATT_ATTRIBUTE_EXECUTE_WRITE_REQ,	 LTP_SUB_GATT_ATTRIBUTE_EXECUTE_WRITE_REQ_LENGTH,  LTP_SUB_GATT_ATTRIBUTE_EXECUTE_WRITE_REQ_FLAGS  },
	{ LTP_OPCODE_RESERVED,                  0,                                            0                                           },
};
const uint8_t ExtendCmdTableSize = (uint8_t)(sizeof(LTPSubCmdTable) / sizeof(LTPCmdInfo));

/*--------------------------------------------------------------------------*/
/* FCS lookup table.                                                        */
/* generator polynomial: x**8 + x**2 + x + 1                                */
/* -------------------------------------------------------------------------*/
static const uint8_t crc8EtsTable[256] =
{
    0x00, 0x91, 0xE3, 0x72, 0x07, 0x96, 0xE4, 0x75,
    0x0E, 0x9F, 0xED, 0x7C, 0x09, 0x98, 0xEA, 0x7B,
    0x1C, 0x8D, 0xFF, 0x6E, 0x1B, 0x8A, 0xF8, 0x69,
    0x12, 0x83, 0xF1, 0x60, 0x15, 0x84, 0xF6, 0x67,
    0x38, 0xA9, 0xDB, 0x4A, 0x3F, 0xAE, 0xDC, 0x4D,
    0x36, 0xA7, 0xD5, 0x44, 0x31, 0xA0, 0xD2, 0x43,
    0x24, 0xB5, 0xC7, 0x56, 0x23, 0xB2, 0xC0, 0x51,
    0x2A, 0xBB, 0xC9, 0x58, 0x2D, 0xBC, 0xCE, 0x5F,
    0x70, 0xE1, 0x93, 0x02, 0x77, 0xE6, 0x94, 0x05,
    0x7E, 0xEF, 0x9D, 0x0C, 0x79, 0xE8, 0x9A, 0x0B,
    0x6C, 0xFD, 0x8F, 0x1E, 0x6B, 0xFA, 0x88, 0x19,
    0x62, 0xF3, 0x81, 0x10, 0x65, 0xF4, 0x86, 0x17,
    0x48, 0xD9, 0xAB, 0x3A, 0x4F, 0xDE, 0xAC, 0x3D,
    0x46, 0xD7, 0xA5, 0x34, 0x41, 0xD0, 0xA2, 0x33,
    0x54, 0xC5, 0xB7, 0x26, 0x53, 0xC2, 0xB0, 0x21,
    0x5A, 0xCB, 0xB9, 0x28, 0x5D, 0xCC, 0xBE, 0x2F,
    0xE0, 0x71, 0x03, 0x92, 0xE7, 0x76, 0x04, 0x95,
    0xEE, 0x7F, 0x0D, 0x9C, 0xE9, 0x78, 0x0A, 0x9B,
    0xFC, 0x6D, 0x1F, 0x8E, 0xFB, 0x6A, 0x18, 0x89,
    0xF2, 0x63, 0x11, 0x80, 0xF5, 0x64, 0x16, 0x87,
    0xD8, 0x49, 0x3B, 0xAA, 0xDF, 0x4E, 0x3C, 0xAD,
    0xD6, 0x47, 0x35, 0xA4, 0xD1, 0x40, 0x32, 0xA3,
    0xC4, 0x55, 0x27, 0xB6, 0xC3, 0x52, 0x20, 0xB1,
    0xCA, 0x5B, 0x29, 0xB8, 0xCD, 0x5C, 0x2E, 0xBF,
    0x90, 0x01, 0x73, 0xE2, 0x97, 0x06, 0x74, 0xE5,
    0x9E, 0x0F, 0x7D, 0xEC, 0x99, 0x08, 0x7A, 0xEB,
    0x8C, 0x1D, 0x6F, 0xFE, 0x8B, 0x1A, 0x68, 0xF9,
    0x82, 0x13, 0x61, 0xF0, 0x85, 0x14, 0x66, 0xF7,
    0xA8, 0x39, 0x4B, 0xDA, 0xAF, 0x3E, 0x4C, 0xDD,
    0xA6, 0x37, 0x45, 0xD4, 0xA1, 0x30, 0x42, 0xD3,
    0xB4, 0x25, 0x57, 0xC6, 0xB3, 0x22, 0x50, 0xC1,
    0xBA, 0x2B, 0x59, 0xC8, 0xBD, 0x2C, 0x5E, 0xCF
};

static uint8_t BTLTPTgtDoCRC8(LTP_TGT_APPHANDLE AppHandle, uint8_t * pStart, uint16_t length)
{
    uint8_t fcs = 0xff;

    while (length--)
    {
        fcs = crc8EtsTable[fcs ^ *pStart++];
    }
    return 0xff - fcs;
}

void ltpQueueIn(LTP_QUEUE_P QueuePtr, void *pQueueElement)
{
    LTP_ELEMENT_P QueueElementPtr = (LTP_ELEMENT_P)pQueueElement;
    LTP_ELEMENT_P LastPtr;

    if ((LastPtr = QueuePtr->Last) == (LTP_ELEMENT_P)0)    /* if queue is empty,  */
        QueuePtr->First = QueueElementPtr;    /* q->first = q->last = new entry */
    else                                    /* if it is not empty, new entry  */
        LastPtr->Next  = QueueElementPtr;     /* is next from last entry        */
    QueuePtr->Last = QueueElementPtr;
    QueueElementPtr->Next = (LTP_ELEMENT_P)0;
    QueuePtr->ElementCount++;               /* increment element count        */
}

void *ltpQueueOut(LTP_QUEUE_P QueuePtr)
{
    LTP_ELEMENT_P FirstPtr;

    if ((FirstPtr = QueuePtr->First) != (LTP_ELEMENT_P)0)
    {
        /* if queue not empty and    */
        /* it is the last entry      */
        if ((QueuePtr->First = FirstPtr->Next) == (LTP_ELEMENT_P)0)
            QueuePtr->Last = (LTP_ELEMENT_P)0;           /* set queue empty           */
        QueuePtr->ElementCount--;                  /* decrement element count   */
    }
    return (FirstPtr);
}

static PLTPCmdInfo BTLTPGetMsgProperty(uint8_t opcode)
{
    uint8_t minTabPos = 0;
    uint8_t maxTabPos = CmdTableSize;
    uint8_t tabPos    = 0;
    uint8_t oldTabPos;

    while (TRUE)
    {
        oldTabPos = tabPos;
        tabPos    = minTabPos + ((maxTabPos - minTabPos) >> 1); /* /2 */

        if (LTPCmdTable[tabPos].opcode == opcode)
        {
            return (PLTPCmdInfo) & (LTPCmdTable[tabPos]);
        }
        else if (LTPCmdTable[tabPos].opcode > opcode)
        {
            maxTabPos = tabPos;
        }
        else if (LTPCmdTable[tabPos].opcode < opcode)
        {
            minTabPos = tabPos;
        }

        if (oldTabPos == tabPos)
        {
            return NULL;
        }
    }
}

static PLTPCmdInfo BTLTPGetSubMsgProperty(uint8_t opcode)
{
    uint8_t minTabPos = 0;
    uint8_t maxTabPos = ExtendCmdTableSize;
    uint8_t tabPos    = 0;
    uint8_t oldTabPos;

    while (TRUE)
    {
        oldTabPos = tabPos;
        tabPos    = minTabPos + ((maxTabPos - minTabPos) >> 1); /* /2 */

        if (LTPSubCmdTable[tabPos].opcode == opcode)
        {
            return (PLTPCmdInfo) & (LTPSubCmdTable[tabPos]);
        }
        else if (LTPSubCmdTable[tabPos].opcode > opcode)
        {
            maxTabPos = tabPos;
        }
        else if (LTPSubCmdTable[tabPos].opcode < opcode)
        {
            minTabPos = tabPos;
        }

        if (oldTabPos == tabPos)
        {
            return NULL;
        }
    }
}

static BOOL BTLTPTransferLTPElementData(PLTPLib pLTPLib, uint16_t copyLength)
{
    BOOL        retVal = FALSE;     /* ==> all requested data copied */
    PLTPElement pActiveElement = pLTPLib->pActiveElement;

    if (pActiveElement == NULL)
    {
        return TRUE;    /* ==> new queue element required                       */
    }

    if (copyLength > pActiveElement->DataCB.Length)
    {
        copyLength = pActiveElement->DataCB.Length;
        retVal = TRUE;    /* ==> not enough data in element!!!!!!                     */
    }

    /* copy element data to msg buffer                                        */
    memcpy(&pLTPLib->pLTPMsg[pLTPLib->LTPMsgPos],
           &pActiveElement->DataCB.BufferAddress[pActiveElement->DataCB.Offset],
           copyLength
          );

    /* updata data structures for element and msg buffer                      */
    pLTPLib->LTPDataCollected += copyLength;
    pLTPLib->LTPMsgPos        += copyLength;

    pActiveElement->DataCB.Offset += copyLength;
    pActiveElement->DataCB.Length -= copyLength;

    /* check if element is consumed completely                                */
    if (!pActiveElement->DataCB.Length)
    {
        BTLTPTgtReceiveBufferRelease(pLTPLib->AppHandle, pActiveElement->DataCB.BufferAddress);

        BTLTPTgtQueueElementRelease(pLTPLib->AppHandle, pActiveElement);

        pLTPLib->pActiveElement = NULL;
    }

    return retVal;
}


static void BTLTPInitLTPAssembly(PLTPLib pLTPLib, BOOL reUseBuffer)
{
    if (reUseBuffer)
    {
        pLTPLib->LTPMsgStart      = pLTPLib->ReceiveOffset;
        pLTPLib->LTPMsgPos        = pLTPLib->ReceiveOffset;
        pLTPLib->LTPDataCollected = 0;
        pLTPLib->LTPMsgLength     = 0; /* not known */
    }
    else
    {
        pLTPLib->pLTPMsg = NULL;
    }
}


static BOOL LTPLibTrigger(PLTPLib pLTPLib)
{
    uint8_t        cmd;
    uint8_t        subCmd;
    uint8_t        copmsk;
    uint16_t        lenPara;
    uint16_t        optLen;
    uint8_t *      pOpt;
    uint8_t *      pPara;
    uint16_t        copyLength;
    BOOL        reuseBuffer = FALSE;
    PLTPCmdInfo pCmdInfo;

    /* if we have no assembly buffer => try to get one */
    if (!pLTPLib->pLTPMsg)
    {
        pLTPLib->pLTPMsg = BTLTPTgtAssemblyBufferAlloc(pLTPLib->AppHandle);

        if (pLTPLib->pLTPMsg)
        {
            BTLTPInitLTPAssembly(pLTPLib, TRUE);
        }
        else
        {
            return FALSE; /* no re-trigger */
        }
    }

    /* if we have no LTP data element to work with => try to get one */
    if (!pLTPLib->pActiveElement)
    {
        pLTPLib->pActiveElement = (PLTPElement)ltpQueueOut(&pLTPLib->UsedElementQueue);

        if (!pLTPLib->pActiveElement)
        {
            return FALSE; /* no re-trigger */
        }
    }

    /* if we don't know how long the LTP msg is we try to assemble */
    /* ==> try to determine LTP msg length */
    if (!pLTPLib->LTPMsgLength)
    {
        if (pLTPLib->LTPDataCollected < LTP_DATA_MIN_HEADER_LENGTH)
        {
            copyLength = LTP_DATA_MIN_HEADER_LENGTH - pLTPLib->LTPDataCollected;
            if (BTLTPTransferLTPElementData(pLTPLib, copyLength))
            {
                return TRUE; /* not enough data => re-trigger*/
            }
        }

        /* basic mgs header complete, check for optional CRC */
        copmsk  = pLTPLib->pLTPMsg[pLTPLib->LTPMsgStart + 1];

        /* if there is a CRC => use it for msg integrety check */
        if (copmsk & LTP_OPT_MASK_HEADER_CRC8)
        {
            optLen = LTPLibGetOptLength(copmsk);

            if (pLTPLib->LTPDataCollected < (LTP_DATA_MIN_HEADER_LENGTH + optLen))
            {
                copyLength = (LTP_DATA_MIN_HEADER_LENGTH + optLen) - pLTPLib->LTPDataCollected;
                if (BTLTPTransferLTPElementData(pLTPLib, copyLength))
                {
                    return TRUE; /* not enough data => re-trigger */
                }
            }

            /* we have the CRC => check msg header                                */
            if (BTLTPTgtDoCRC8(pLTPLib, &pLTPLib->pLTPMsg[pLTPLib->LTPMsgStart], LTP_DATA_MIN_HEADER_LENGTH)
                    != pLTPLib->pLTPMsg[pLTPLib->LTPMsgStart + LTP_DATA_MIN_HEADER_LENGTH + optLen - 1])
            {
                LTPLibSendInternalEventInfo(pLTPLib, 0, NULL, LTP_CAUSE_CONNECTION_LOST, LTP_INTERNAL_EVENT_COMMUNICATION_OUT_OF_SYNC, LTP_GENERATE_EVENT_ID);
                BTLTPInitLTPAssembly(pLTPLib, FALSE);
                return TRUE;
            }
        }

        pLTPLib->LTPMsgLength = NETCHAR2SHORT(&pLTPLib->pLTPMsg[pLTPLib->LTPMsgStart + 2]);
        DBG_BUFFER(MODULE_LTP, LEVEL_TRACE, "LTPLibTrigger:pLTPLib->LTPMsgLength = %d", 1, pLTPLib->LTPMsgLength);

        if (pLTPLib->LTPMsgLength > pLTPLib->ReceiveMaxLength)
        {
            LTPLibSendInternalEventInfo(pLTPLib, 0, NULL, LTP_CAUSE_INVALID_PARAMETER, LTP_INTERNAL_EVENT_COMMUNICATION_OUT_OF_SYNC, LTP_GENERATE_EVENT_ID);
            BTLTPInitLTPAssembly(pLTPLib, FALSE);
            return TRUE;
        }
    }

    /* added by LV for robustness - module might send invalid data on reset...*/
    if (!pLTPLib->LTPMsgLength)
    {
        BTLTPInitLTPAssembly(pLTPLib, FALSE);
        return TRUE;
    }

    /* wait for message completed */
    if (pLTPLib->LTPMsgLength > pLTPLib->LTPDataCollected)
    {
        /* try to complete LTP message */
        if (BTLTPTransferLTPElementData(pLTPLib, pLTPLib->LTPMsgLength - pLTPLib->LTPDataCollected))
        {
            return TRUE; /* not enough data => re-trigger */
        }
    }
    /*------------------------------------------------------------------------*/
    /* message is completed => process it                                     */
    /*------------------------------------------------------------------------*/

    /* prepare optional parameter handling */
    copmsk  = pLTPLib->pLTPMsg[pLTPLib->LTPMsgStart + 1];

    if (copmsk)
    {
        /* if there are optiona parameters ==> set access parameter */
        pOpt   = &pLTPLib->pLTPMsg[pLTPLib->LTPMsgStart + 4];
        optLen = LTPLibGetOptLength(copmsk);
    }
    else
    {
        pOpt   = NULL;
        optLen = 0;
    }

    /* prepare mandatory parameter handling */
    lenPara = pLTPLib->LTPMsgLength - 4 - optLen;

    if (lenPara)
    {
        /* if there are mandatory parameter ==> this is the pointer to them */
        pPara = &pLTPLib->pLTPMsg[pLTPLib->LTPMsgStart + 4 + optLen];
    }
    else
    {
        pPara = NULL;
    }

    /* we have a LTP message completely re assembled => handle it             */
    cmd     = pLTPLib->pLTPMsg[pLTPLib->LTPMsgStart];

    if (cmd < 0xFB)
    {
        if (cmd == LTP_EXTEND_COMMAND)
        {
            if (pPara == NULL)
            {
                DBG_BUFFER(MODULE_LTP, LEVEL_ERROR, "LTPLibTrigger: cmd = 0xFA pPara == NULL", 0);
                BTLTPInitLTPAssembly(pLTPLib, FALSE);
                return TRUE; /* msg done => re-trigger                             */
            }
            subCmd = pPara[0];
            pCmdInfo = BTLTPGetSubMsgProperty(subCmd);
        }
        else
        {
            pCmdInfo = BTLTPGetMsgProperty(cmd);
        }

        if (pCmdInfo)
        {
            if (pCmdInfo->properties & LTP_VAR_LEN_MSG)
            {
                /* valiable length msg => check min length                              */
                if (pLTPLib->LTPMsgLength < (pCmdInfo->length + optLen))
                {
                    /* if not confirmations with cause "notSuppoerted"                   */
                    if (((pCmdInfo->properties & LTP_CNF_MSG) != LTP_CNF_MSG)                 ||
                            (pLTPLib->LTPMsgLength             != LTP_DATA_MIN_HEADER_LENGTH + 1) ||
                            (pOpt                              != NULL)                        ||
                            (pPara[0]                          != LTP_CAUSE_NOT_SUPPORTED)
                       )
                    {
                        LTPLibSendInternalEventInfo(pLTPLib, 0, NULL, LTP_CAUSE_INVALID_PARAMETER, LTP_INTERNAL_EVENT_MALFORMED_MSG_RECEIVED, LTP_GENERATE_EVENT_ID);
                        LTPLibHandleUnkownCommand(pLTPLib, cmd);
                        BTLTPInitLTPAssembly(pLTPLib, FALSE);
                        return TRUE; /* msg done => re-trigger                             */
                    }
                }
            }
            else
            {
                /* fixed length msg => check length                                     */
                if (pLTPLib->LTPMsgLength != (pCmdInfo->length + optLen))
                {
                    /* if not confirmations with cause "notSuppoerted"                   */
                    if (((pCmdInfo->properties & LTP_CNF_MSG) != LTP_CNF_MSG)                 ||
                            (pLTPLib->LTPMsgLength             != LTP_DATA_MIN_HEADER_LENGTH + 1) ||
                            (pOpt                              != NULL)                        ||
                            (pPara[0]                          != LTP_CAUSE_NOT_SUPPORTED)
                       )
                    {
                        LTPLibSendInternalEventInfo(pLTPLib, 0, NULL, LTP_CAUSE_INVALID_PARAMETER, LTP_INTERNAL_EVENT_MALFORMED_MSG_RECEIVED, LTP_GENERATE_EVENT_ID);
                        LTPLibHandleUnkownCommand(pLTPLib, cmd);
                        BTLTPInitLTPAssembly(pLTPLib, FALSE);
                        return TRUE; /* msg done => re-trigger                             */
                    }
                }
            }
        }
    }

    /* jsut for testing aci */
    if (cmd == 0xFC)
    {
        //reuseBuffer = LTPLL_HandleLowPowerCmd(pLTPLib->pLTPMsg, cmd, copmsk, pOpt, lenPara, pPara);
    }
    else
    {
        reuseBuffer = BTLTPTgtHandleLTPMessage(pLTPLib->AppHandle, pLTPLib->pLTPMsg, cmd, copmsk, pOpt, lenPara, pPara);
    }

    BTLTPInitLTPAssembly(pLTPLib, reuseBuffer);

    return TRUE; /* msg done => re-trigger                                    */
}

BOOL LTPLibInitialize(PLTPLib pLTPLib, LTP_TGT_APPHANDLE AppHandle, uint16_t ReceiveOffset, uint16_t ReceiveMaxLen, uint16_t SendOffset)
{
    /* first of all, clear context data                                       */
    memset(pLTPLib, 0, sizeof(TLTPLib));

    /* initialize context                                                     */
    pLTPLib->AppHandle        = AppHandle;
    pLTPLib->ReceiveOffset    = ReceiveOffset;
    pLTPLib->ReceiveMaxLength = ReceiveMaxLen;
    pLTPLib->SendOffset       = SendOffset;

    /* initialize message assembly                                            */
    BTLTPInitLTPAssembly(pLTPLib, TRUE);

#if F_LTPLIB_ASYNC_ASSEMBLY_SUPPORT
    /* initialize asyncronus message asse                                     */
    pLTPLib->AsyncState = LTPLibAsyncStatusIgnore;

    BTLTPTgtTriggerTimer(pLTPLib->AppHandle, LTPLibTimerID_AsyncTimeout, 1000 /* 1s */);
#endif

    /* ready to rock...                                                       */
    pLTPLib->Status = LTPLibStatusIdle;

    return TRUE; /* OK */
}


BOOL LTPLibShutdown(PLTPLib pLTPLib)
{
    if (pLTPLib == NULL)
    {
        return FALSE;
    }

    if (pLTPLib->pLTPMsg != NULL)
    {
        pLTPLib->pLTPMsg = NULL;
    }

    if (pLTPLib->pActiveElement != NULL)
    {
        BTLTPTgtQueueElementRelease(pLTPLib->AppHandle, pLTPLib->pActiveElement);
        pLTPLib->pActiveElement = NULL;
    }

    return TRUE;
}


#if 0
static void LTPLibTriggerLTPProccess(PLTPLib pLTPLib)
#else
void LTPLibTriggerLTPProccess(PLTPLib pLTPLib)
#endif	
{
    if (pLTPLib->Status >= LTPLibStatusBusy) /* off-sync or re-entrant          */
    {
        return;
    }
    else                                /* now we are busy                    */
    {
        pLTPLib->Status = LTPLibStatusBusy;
    }

    while (LTPLibTrigger(pLTPLib));

    if (pLTPLib->Status == LTPLibStatusBusy) /* might be off-sync               */
    {
        pLTPLib->Status = LTPLibStatusIdle;
    }
}

#if F_LTPLIB_ASYNC_ASSEMBLY_SUPPORT
void LTPLibHandleTimeout(PLTPLib pLTPLib, TLTPTimerID timerID)
{
    switch (timerID)
    {
    case LTPLibTimerID_AsyncTimeout: /*-------------------------------------*/
        switch (pLTPLib->AsyncState)
        {
        case LTPLibAsyncStatusIgnore: /*....................................*/
            if (pLTPLib->LTPDataCollected)
            {
                pLTPLib->AsyncState = LTPLibAsyncStatusWaitCMD;
            }
            /* next data will re-trigger timer                                */
            break;
        case LTPLibAsyncStatusWaitTimeout: /*...............................*/
            /* let App handle message                                         */
            BTLTPTgtHandleAsyncLTP_RESET_REQ(pLTPLib->AppHandle);
            /* reset assembly, but keep in mind that we just had 1s idle...   */
            pLTPLib->AsyncState     = LTPLibAsyncStatusIgnore;
            /* next data will re-trigger timer                                */
            break;
        default: /*.........................................................*/
            break; /* rest does not matter                                    */
        }
        break;

    default: /*-------------------------------------------------------------*/
        break; /* shall never happen!!!!!                                    */
    }
} /* end of LTPLibHandleTimeout */
#endif /* F_LTPLIB_ASYNC_ASSEMBLY_SUPPORT */


BOOL LTPLibHandleReceiveData(PLTPLib pLTPLib, uint8_t * pRxBuffer, uint16_t rxLength, uint16_t rxOffset)
{
    PLTPElement pLTPElement;

    /* try to get new storage element                                         */
    pLTPElement = BTLTPTgtQueueElementAlloc(pLTPLib->AppHandle);
    if (pLTPElement)
    {
        /* store information                                                   */
        pLTPElement->DataCB.BufferAddress = pRxBuffer;
        pLTPElement->DataCB.Length        = rxLength;
        pLTPElement->DataCB.Offset        = rxOffset;

        ltpQueueIn(&pLTPLib->UsedElementQueue, pLTPElement);
        //DBG_BUFFER(MODULE_LTP, LEVEL_TRACE, "LTPLibHandleReceiveData: Alloc new element: len = %d ", 1, rxLength);
    }
    else
    {
        DBG_BUFFER(MODULE_LTP, LEVEL_ERROR, "LTPLibHandleReceiveData: Alloc new element failed!!!", 0);
    }

    /* handle of-sync command detection                                       */
#if F_LTPLIB_ASYNC_ASSEMBLY_SUPPORT
    switch (pLTPLib->AsyncState)
    {
    case LTPLibAsyncStatusIgnore: /*----------------------------------------*/
        break;
    case LTPLibAsyncStatusWaitCMD: /*---------------------------------------*/
        if (pRxBuffer[rxOffset] == LTP_RESET_REQ)
        {
            pLTPLib->AsyncState = LTPLibAsyncStatusWaitCopMsk;
            rxLength--;
            rxOffset++;
        }
        else
        {
            pLTPLib->AsyncState = LTPLibAsyncStatusIgnore;
            rxLength            = 0;
            break;
        }
    /* no break */
    case LTPLibAsyncStatusWaitCopMsk: /*------------------------------------*/
        if (rxLength)
        {
            if (pRxBuffer[rxOffset] == 0x00) /* LTP_RESET_REQ CopMsk = 0x00        */
            {
                pLTPLib->AsyncState = LTPLibAsyncStatusWaitLen1;
                rxLength--;
                rxOffset++;
            }
            else if (pRxBuffer[rxOffset] == LTP_OPT_MASK_HEADER_CRC8)
            {
                pLTPLib->AsyncState = LTPLibAsyncStatusWaitLen1HeaderCRC;
                rxLength--;
                rxOffset++;
            }
            else
            {
                pLTPLib->AsyncState = LTPLibAsyncStatusIgnore;
                rxLength            = 0;
                break;
            }
        }
    /* no break */
    case LTPLibAsyncStatusWaitLen1: /*--------------------------------------*/
    case LTPLibAsyncStatusWaitLen1HeaderCRC: /*-----------------------------*/
        if (rxLength)
        {
            if (pRxBuffer[rxOffset] == 0x00)   /* LTP_RESET_REQ len = 0x00 0x0X */
            {
                if (pLTPLib->AsyncState == LTPLibAsyncStatusWaitLen1)
                {
                    pLTPLib->AsyncState = LTPLibAsyncStatusWaitLen2;
                }
                else   /* pLTPLib->AsyncState==LTPLibAsyncStatusWaitLen1HeaderCRC */
                {
                    pLTPLib->AsyncState = LTPLibAsyncStatusWaitLen2HeaderCRC;
                }
                rxLength--;
                rxOffset++;
            }
            else
            {
                pLTPLib->AsyncState = LTPLibAsyncStatusIgnore;
                rxLength            = 0;
                break;
            }
        }
    /* no break */
    case LTPLibAsyncStatusWaitLen2: /*--------------------------------------*/
    case LTPLibAsyncStatusWaitLen2HeaderCRC: /*-----------------------------*/
        if (rxLength)
        {
            if (pLTPLib->AsyncState == LTPLibAsyncStatusWaitLen2)
            {
                if ((pRxBuffer[rxOffset] == LTP_RESET_REQ_LENGTH) && (rxLength == 1))
                {
                    pLTPLib->AsyncState = LTPLibAsyncStatusWaitTimeout;
                    BTLTPTgtTriggerTimer(pLTPLib->AppHandle, LTPLibTimerID_AsyncTimeout, 1000 /* 1s */);
                    break;
                }
                else
                {
                    pLTPLib->AsyncState = LTPLibAsyncStatusIgnore;
                    rxLength            = 0;
                }
            }
            else if (pLTPLib->AsyncState == LTPLibAsyncStatusWaitLen2HeaderCRC)
            {
                if (pRxBuffer[rxOffset] == LTP_RESET_REQ_LENGTH + 1)
                {
                    pLTPLib->AsyncState = LTPLibAsyncStatusWaitHeaderCRC;
                    rxLength--;
                    rxOffset++;
                }
                else
                {
                    pLTPLib->AsyncState = LTPLibAsyncStatusIgnore;
                    rxLength            = 0;
                }
            }
            else
            {
                pLTPLib->AsyncState = LTPLibAsyncStatusIgnore;
                rxLength            = 0;
            }
        }
    /* no break */
    case LTPLibAsyncStatusWaitHeaderCRC: /*---------------------------------*/
        if (rxLength)
        {
            if ((pRxBuffer[rxOffset] == LTP_RESET_REQ_HEADER_CRC) && (rxLength == 1))
            {
                pLTPLib->AsyncState = LTPLibAsyncStatusWaitTimeout;
                BTLTPTgtTriggerTimer(pLTPLib->AppHandle, LTPLibTimerID_AsyncTimeout, 1000 /* 1s */);
            }
            else
            {
                pLTPLib->AsyncState = LTPLibAsyncStatusIgnore;
                rxLength            = 0;
            }
        }
        break;
    case LTPLibAsyncStatusWaitTimeout: /*-----------------------------------*/
        BTLTPTgtTriggerTimer(pLTPLib->AppHandle, LTPLibTimerID_AsyncTimeout, 1000 /* 1s */);
        pLTPLib->AsyncState = LTPLibAsyncStatusIgnore;
        break;
    default: /*-------------------------------------------------------------*/
        break; /* shall never happen!!!!!                                     */
    }

    if (pLTPLib->AsyncState == LTPLibAsyncStatusIgnore)
    {
        BTLTPTgtTriggerTimer(pLTPLib->AppHandle, LTPLibTimerID_AsyncTimeout, 1000 /* 1s */);
    }
#endif /* F_LTPLIB_ASYNC_ASSEMBLY_SUPPORT */

    if (!pLTPElement)
    {
        return FALSE; /* data could not be handled, please re-try later...      */
    }

    /* trigger LTP statemachine                                               */
    LTPLibTriggerLTPProccess(pLTPLib);

    return TRUE; /* OK */
}


static uint16_t LTPLibGetOptLength(uint8_t copmsk)
{
    uint16_t mask   = 0x0001;
    uint16_t optLen = 0;

    do
    {
        if (mask & ((uint16_t)copmsk))
        {
            optLen++;
        }
        mask = mask << 1;

    }
    while (mask & 0x00FF);

    return optLen;
}

uint16_t LTPLibInsertHeader(PLTPLib pLTPLib, uint8_t * pBuffer, LPWORD offset, uint16_t dataLen, uint8_t cmd, uint8_t copmsk, uint8_t * pOpt, LPWORD posParam)
{
    PLTPCmdInfo pCmdInfo  = BTLTPGetMsgProperty(cmd);
    uint16_t        optLen    = LTPLibGetOptLength(copmsk);
    uint8_t        DoCRC8    = ((copmsk & LTP_OPT_MASK_HEADER_CRC8) >> 7);
    uint16_t        pos       = *offset;

    /* unknown command, not a command with variable data length or */
    /* offset is too small to insert LTP header + parameters */
    if ((pCmdInfo == NULL) ||
            !(pCmdInfo->properties & LTP_VAR_LEN_MSG) ||
            (pos < (pCmdInfo->length + optLen))
       )
    {
        return 0;
    }

    /* set offset to first byte of the message */
    pos -= (pCmdInfo->length + optLen);
    *offset = pos;

    pBuffer[pos++] = pCmdInfo->opcode;
    pBuffer[pos++] = copmsk;
    NETSHORT2CHAR(&pBuffer[pos], pCmdInfo->length + optLen + dataLen);
    pos += 2;

    if (copmsk)
    {
        memcpy(&pBuffer[pos], pOpt, optLen - DoCRC8);
        pos += optLen - DoCRC8;
    }

    if (DoCRC8)
    {
        pBuffer[pos++] = BTLTPTgtDoCRC8(pLTPLib, &pBuffer[*offset], LTP_DATA_MIN_HEADER_LENGTH);
    }

    /* set offset to first parameter */
    *posParam = pos;

    /* return message length */
    return pCmdInfo->length + optLen + dataLen;
}

uint16_t LTPLibInsertExtendHeader(PLTPLib pLTPLib, uint8_t * pBuffer, LPWORD offset, uint16_t dataLen, uint8_t subCmd, uint8_t copmsk, uint8_t * pOpt, LPWORD posParam)
{
    PLTPCmdInfo pCmdInfo  = BTLTPGetSubMsgProperty(subCmd);
    uint16_t        optLen    = LTPLibGetOptLength(copmsk);
    uint8_t        DoCRC8    = ((copmsk & LTP_OPT_MASK_HEADER_CRC8) >> 7);
    uint16_t        pos       = *offset;

    /* unknown command, not a command with variable data length or */
    /* offset is too small to insert LTP header + parameters */
    if ((pCmdInfo == NULL) ||
            !(pCmdInfo->properties & LTP_VAR_LEN_MSG) ||
            (pos < (pCmdInfo->length + optLen))
       )
    {
        return 0;
    }

    /* set offset to first byte of the message */
    pos -= (pCmdInfo->length + optLen);
    *offset = pos;

    pBuffer[pos++] = LTP_EXTEND_COMMAND;
    pBuffer[pos++] = copmsk;
    NETSHORT2CHAR(&pBuffer[pos], pCmdInfo->length + optLen + dataLen);
    pos += 2;

    if (copmsk)
    {
        memcpy(&pBuffer[pos], pOpt, optLen - DoCRC8);
        pos += optLen - DoCRC8;
    }

    if (DoCRC8)
    {
        pBuffer[pos++] = BTLTPTgtDoCRC8(pLTPLib, &pBuffer[*offset], LTP_DATA_MIN_HEADER_LENGTH);
    }
    pBuffer[pos++] = pCmdInfo->opcode;
    /* set offset to first parameter */
    *posParam = pos;

    /* return message length */
    return pCmdInfo->length + optLen + dataLen;
}


uint8_t * LTPLibWriteExtendHeader(PLTPLib pLTPLib, LPWORD offset, uint8_t subCmd, uint8_t copmsk, uint8_t * pOpt, uint16_t varLen)
{
    PLTPCmdInfo cmdInfo = BTLTPGetSubMsgProperty(subCmd);
    uint16_t        optLen  = LTPLibGetOptLength(copmsk);
    uint8_t        DoCRC8  = ((copmsk & LTP_OPT_MASK_HEADER_CRC8) >> 7);
    uint16_t        pos     = *offset;
    uint8_t *      pBuffer;

    if (cmdInfo == NULL)
    {
        return NULL;
    }

    if (!(cmdInfo->properties & LTP_VAR_LEN_MSG))
    {
        varLen = 0;
    }

    pBuffer = BTLTPTgtSendBufferAlloc(pLTPLib->AppHandle, *offset + cmdInfo->length + optLen + varLen);

    //DBG_BUFFER(MODULE_LTP, LEVEL_TRACE, "*offset = 0x%x, cmdInfo->length = 0x%x, optLen = 0x%x, varLen = 0x%x", 4,\
    //*offset, cmdInfo->length, optLen, varLen);

    if (pBuffer == NULL)
    {
        return NULL;
    }

    pBuffer[pos++] = LTP_EXTEND_COMMAND;
    pBuffer[pos++] = copmsk;
    NETSHORT2CHAR(&pBuffer[pos], cmdInfo->length + optLen + varLen);
    pos += 2;

    if (copmsk)
    {
        memcpy(&pBuffer[pos], pOpt, optLen - DoCRC8);
        pos += optLen - DoCRC8;
    }

    if (DoCRC8)
    {
        pBuffer[pos++] = BTLTPTgtDoCRC8(pLTPLib, &pBuffer[*offset], LTP_DATA_MIN_HEADER_LENGTH);
    }
    pBuffer[pos++] = cmdInfo->opcode;

    *offset = pos;
    return pBuffer;
} /* end of LTPLibWriteHeader */

uint8_t * LTPLibWriteHeader(PLTPLib pLTPLib, LPWORD offset, uint8_t cmd, uint8_t copmsk, uint8_t * pOpt, uint16_t varLen)
{
    PLTPCmdInfo cmdInfo = BTLTPGetMsgProperty(cmd);
    uint16_t        optLen  = LTPLibGetOptLength(copmsk);
    uint8_t        DoCRC8  = ((copmsk & LTP_OPT_MASK_HEADER_CRC8) >> 7);
    uint16_t        pos     = *offset;
    uint8_t *      pBuffer;

    if (cmdInfo == NULL)
    {
        return NULL;
    }

    if (!(cmdInfo->properties & LTP_VAR_LEN_MSG))
    {
        varLen = 0;
    }

    pBuffer = BTLTPTgtSendBufferAlloc(pLTPLib->AppHandle, *offset + cmdInfo->length + optLen + varLen);

    DBG_BUFFER(MODULE_LTP, LEVEL_TRACE, "*offset = 0x%x, cmdInfo->length = 0x%x, optLen = 0x%x, varLen = 0x%x", 4, \
               *offset, cmdInfo->length, optLen, varLen);

    if (pBuffer == NULL)
    {
        return NULL;
    }

    pBuffer[pos++] = cmd;
    pBuffer[pos++] = copmsk;
    NETSHORT2CHAR(&pBuffer[pos], cmdInfo->length + optLen + varLen);
    pos += 2;

    if (copmsk)
    {
        memcpy(&pBuffer[pos], pOpt, optLen - DoCRC8);
        pos += optLen - DoCRC8;
    }

    if (DoCRC8)
    {
        pBuffer[pos++] = BTLTPTgtDoCRC8(pLTPLib, &pBuffer[*offset], LTP_DATA_MIN_HEADER_LENGTH);
    }

    *offset = pos;
    return pBuffer;
} /* end of LTPLibWriteHeader */


BOOL LTPLibSendMessage_BYTE(PLTPLib pLTPLib, uint8_t copmsk, uint8_t * pOpt, uint8_t cmd, uint8_t param)
{
    uint16_t   pos     = pLTPLib->SendOffset;
    uint8_t * pBuffer = LTPLibWriteHeader(pLTPLib, &pos, cmd, copmsk, pOpt, 0);

    if (!pBuffer)
    {
        return FALSE;
    }

    pBuffer[pos++] = param;

    return (BTLTPTgtSendLTPMessage(pLTPLib->AppHandle, pBuffer, pLTPLib->SendOffset, pos - pLTPLib->SendOffset));
}


BOOL LTPLibSendMessage_BYTE_BYTE(PLTPLib pLTPLib, uint8_t copmsk, uint8_t * pOpt, uint8_t cmd, uint8_t param1, uint8_t param2)
{
    uint16_t   pos     = pLTPLib->SendOffset;
    uint8_t * pBuffer = LTPLibWriteHeader(pLTPLib, &pos, cmd, copmsk, pOpt, 0);

    if (!pBuffer)
    {
        return FALSE;
    }

    pBuffer[pos++] = param1;
    pBuffer[pos++] = param2;

    return (BTLTPTgtSendLTPMessage(pLTPLib->AppHandle, pBuffer, pLTPLib->SendOffset, pos - pLTPLib->SendOffset));
}

static BOOL LTPLibSendSubMessage_BYTE(PLTPLib pLTPLib, uint8_t copmsk, uint8_t * pOpt, uint8_t cmd, uint8_t param1)
{
    uint16_t   pos     = pLTPLib->SendOffset;
    uint8_t * pBuffer = LTPLibWriteExtendHeader(pLTPLib, &pos, cmd, copmsk, pOpt, 0);

    if (!pBuffer)
    {
        return FALSE;
    }

    pBuffer[pos++] = param1;

    return (BTLTPTgtSendLTPMessage(pLTPLib->AppHandle, pBuffer, pLTPLib->SendOffset, pos - pLTPLib->SendOffset));
}

BOOL LTPLibSendMessage_BD(PLTPLib pLTPLib, uint8_t copmsk, uint8_t * pOpt, uint8_t cmd, uint8_t * bd)
{
    uint16_t   pos     = pLTPLib->SendOffset;
    uint8_t * pBuffer = LTPLibWriteHeader(pLTPLib, &pos, cmd, copmsk, pOpt, 0);

    if (!pBuffer)
    {
        return TRUE;
    }

    memcpy(&pBuffer[pos], bd, 6);
    pos += 6;

    return BTLTPTgtSendLTPMessage(pLTPLib->AppHandle, pBuffer, pLTPLib->SendOffset, pos - pLTPLib->SendOffset);
}


BOOL LTPLibSendMessage_BD_BYTE(PLTPLib pLTPLib, uint8_t copmsk, uint8_t * pOpt, uint8_t cmd, uint8_t * bd, uint8_t param)
{
    uint16_t   pos     = pLTPLib->SendOffset;
    uint8_t * pBuffer = LTPLibWriteHeader(pLTPLib, &pos, cmd, copmsk, pOpt, 0);

    if (!pBuffer)
    {
        return TRUE;
    }

    memcpy(&pBuffer[pos], bd, 6);
    pos += 6;
    pBuffer[pos++] = param;

    return BTLTPTgtSendLTPMessage(pLTPLib->AppHandle, pBuffer, pLTPLib->SendOffset, pos - pLTPLib->SendOffset);
}


BOOL LTPLibSendMessage_BD_DWORD(PLTPLib pLTPLib, uint8_t copmsk, uint8_t * pOpt, uint8_t cmd, uint8_t * bd, uint32_t param)
{
    uint16_t   pos     = pLTPLib->SendOffset;
    uint8_t * pBuffer = LTPLibWriteHeader(pLTPLib, &pos, cmd, copmsk, pOpt, 0);

    if (!pBuffer)
    {
        return FALSE;
    }

    memcpy(&pBuffer[pos], bd, 6);
    pos += 6;
    NETLONG2CHAR(&pBuffer[pos], param);
    pos += 4;

    return (BTLTPTgtSendLTPMessage(pLTPLib->AppHandle, pBuffer, pLTPLib->SendOffset, pos - pLTPLib->SendOffset));
}

BOOL LTPLibSendMessage_BYTE_BD(PLTPLib pLTPLib, uint8_t copmsk, uint8_t * pOpt, uint8_t cmd, uint8_t param, uint8_t * bd)
{
    uint16_t   pos     = pLTPLib->SendOffset;
    uint8_t * pBuffer = LTPLibWriteHeader(pLTPLib, &pos, cmd, copmsk, pOpt, 0);

    if(!pBuffer)
    {
        return TRUE;
    }

    pBuffer[pos++] = param;
    memcpy(&pBuffer[pos], bd, 6);
    pos += 6;

    return BTLTPTgtSendLTPMessage(pLTPLib->AppHandle, pBuffer, pLTPLib->SendOffset, pos - pLTPLib->SendOffset);
} /* end of LTPLibSendMessage_BYTE_BD */

static BOOL LTPLibHandleUnkownCommand(PLTPLib pLTPLib, uint8_t cmd)
{
    uint16_t   pos     = pLTPLib->SendOffset;
    uint8_t * pBuffer;

    if (cmd & 0x80) /* we have to generate a response                           */
    {
        pBuffer = BTLTPTgtSendBufferAlloc(pLTPLib->AppHandle, 5 + pLTPLib->SendOffset);

        if (pBuffer)
        {
            pBuffer[pos++] = cmd & 0x7F;              /* response opcode            */
            pBuffer[pos++] = 0x00;                    /* copmsk                     */
            NETSHORT2CHAR(&pBuffer[pos], 5);
            pos += 2; /* msg length                 */
            pBuffer[pos++] = LTP_CAUSE_NOT_SUPPORTED; /* cause                      */

            return (BTLTPTgtSendLTPMessage(pLTPLib->AppHandle, pBuffer, pLTPLib->SendOffset, pos - pLTPLib->SendOffset));
        }
        else
        {
            /* todo: anything useful left to do? */
            return FALSE;
        }
    }
    else /* just ignore and dump                                              */
    {
        return TRUE;  /* OK */
    }
}


BOOL LTPLibSendDeviceConfigDeviceNameSetRsp(PLTPLib pLTPLib, uint8_t copmsk, uint8_t * pOpt, uint8_t cause)
{
    return LTPLibSendSubMessage_BYTE(pLTPLib, copmsk, pOpt, LTP_SUB_DEVICE_CONFIG_DEVICE_NAME_SET_RSP, cause);
}

BOOL LTPLibSendDeviceConfigStoreSetRsp(PLTPLib pLTPLib, uint8_t copmsk, uint8_t * pOpt, uint8_t cause)
{
    return LTPLibSendSubMessage_BYTE(pLTPLib, copmsk, pOpt, LTP_SUB_DEVICE_CONFIG_STORE_SET_RSP, cause);
}

BOOL LTPLibSendDeviceConfigSecuritySetRsp(PLTPLib pLTPLib, uint8_t copmsk, uint8_t * pOpt, uint8_t cause)
{
    return LTPLibSendSubMessage_BYTE(pLTPLib, copmsk, pOpt, LTP_SUB_DEVICE_CONFIG_SECURITY_SET_RSP, cause);
}

BOOL LTPLibSendDeviceConfigAppearanceSetRsp(PLTPLib pLTPLib, uint8_t copmsk, uint8_t * pOpt, uint8_t cause)
{
    return LTPLibSendSubMessage_BYTE(pLTPLib, copmsk, pOpt, LTP_SUB_DEVICE_CONFIG_APPEARANCE_SET_RSP, cause);
}

BOOL LTPLibSendDeviceConfigPerPrefConnParamSetRsp(PLTPLib pLTPLib, uint8_t copmsk, uint8_t * pOpt, uint8_t cause)
{
    return LTPLibSendSubMessage_BYTE(pLTPLib, copmsk, pOpt, LTP_SUB_DEVICE_CONFIG_PER_PREF_CONN_PARAM_SET_RSP, cause);
}

BOOL LTPLibSendPairableModeSetRsp(PLTPLib pLTPLib, uint8_t copmsk, uint8_t * pOpt, uint8_t cause)
{
    return LTPLibSendMessage_BYTE(pLTPLib, copmsk, pOpt, LTP_PAIRABLE_MODE_SET_RSP, cause);
}


BOOL LTPLibSendPasskeyReqReplyRsp(PLTPLib pLTPLib, uint8_t copmsk, uint8_t * pOpt, uint8_t cause)
{
    return LTPLibSendMessage_BYTE(pLTPLib, copmsk, pOpt, LTP_PASSKEY_REQ_REPLY_RSP, cause);
}


BOOL LTPLibSendPasskeyNotificationInfo(PLTPLib pLTPLib, uint8_t copmsk, uint8_t * pOpt, uint8_t * rem_BD, uint32_t displayValue)
{
    return LTPLibSendMessage_BD_DWORD(pLTPLib, copmsk, pOpt, LTP_PASSKEY_NOTIFICATION_INFO, rem_BD, displayValue);
}


BOOL LTPLibSendActInfo(PLTPLib pLTPLib, uint8_t copmsk, uint8_t * pOpt, uint8_t cause, uint8_t version, uint8_t * local_BD, uint8_t * FW_VersionString)
{
    uint16_t   strLen  = (uint16_t) strlen((char*)FW_VersionString) + 1;
    uint16_t   pos     = pLTPLib->SendOffset;
    uint8_t * pBuffer = LTPLibWriteHeader(pLTPLib, &pos, LTP_ACT_INFO, copmsk, pOpt, strLen);

    if (!pBuffer)
    {
        return FALSE;
    }

    pBuffer[pos++] = cause;
    pBuffer[pos++] = version;
    memcpy(&pBuffer[pos], local_BD, 6);
    pos += 6;
    memcpy(&pBuffer[pos], FW_VersionString, strLen);
    pos += strLen;

    return (BTLTPTgtSendLTPMessage(pLTPLib->AppHandle, pBuffer, pLTPLib->SendOffset, pos - pLTPLib->SendOffset));
}


BOOL LTPLibSendMCLStatusInfo(PLTPLib pLTPLib, uint8_t copmsk, uint8_t * pOpt, uint8_t * rem_BD, uint8_t loc_MCL_ID, uint8_t loc_MCL_Status)
{
    uint16_t   pos     = pLTPLib->SendOffset;
    uint8_t * pBuffer = LTPLibWriteHeader(pLTPLib, &pos, LTP_MCL_STATUS_INFO, copmsk, pOpt, 0);

    if (!pBuffer)
    {
        return FALSE;
    }

    memcpy(&pBuffer[pos], rem_BD, 6);
    pos += 6;

    pBuffer[pos++] = loc_MCL_ID;
    pBuffer[pos++] = loc_MCL_Status;

    return (BTLTPTgtSendLTPMessage(pLTPLib->AppHandle, pBuffer, pLTPLib->SendOffset, pos - pLTPLib->SendOffset));
}


BOOL LTPLibSendACLStatusInfo(PLTPLib pLTPLib, uint8_t copmsk, uint8_t * pOpt, uint8_t * rem_BD, uint8_t loc_ACL_Status)
{
    return LTPLibSendMessage_BD_BYTE(pLTPLib, copmsk, pOpt, LTP_ACL_STATUS_INFO, rem_BD, loc_ACL_Status);
}

BOOL LTPLibSendPasskeyRequestInd(PLTPLib pLTPLib, uint8_t copmsk, uint8_t * pOpt, uint8_t * rem_BD)
{
    return LTPLibSendMessage_BD(pLTPLib, copmsk, pOpt, LTP_PASSKEY_REQUEST_IND, rem_BD);
}


BOOL LTPLibSendRemoteOOBRequestInd(PLTPLib pLTPLib, uint8_t copmsk, uint8_t * pOpt, uint8_t * rem_BD)
{
    return LTPLibSendMessage_BD(pLTPLib, copmsk, pOpt, LTP_REMOTE_OOB_REQUEST_IND, rem_BD);
}


BOOL LTPLibSendAuthResultExtInd(PLTPLib pLTPLib, uint8_t copmsk, uint8_t * pOpt, uint8_t cause, uint8_t * rem_BD, uint8_t rem_BD_Type, uint8_t keyType, uint8_t * linkKey, uint16_t linkKeyLength)
{
    uint16_t   pos     = pLTPLib->SendOffset;
    uint8_t * pBuffer = LTPLibWriteHeader(pLTPLib, &pos, LTP_AUTH_RESULT_EXT_IND, copmsk, pOpt, linkKeyLength);

    if (!pBuffer)
    {
        return TRUE;
    }

    pBuffer[pos++] = cause;
    memcpy(&pBuffer[pos], rem_BD, 6);
    pos += 6;
    pBuffer[pos++] = rem_BD_Type;
    pBuffer[pos++] = keyType;
    memcpy(&pBuffer[pos], linkKey, linkKeyLength);
    pos += linkKeyLength;

    return BTLTPTgtSendLTPMessage(pLTPLib->AppHandle, pBuffer, pLTPLib->SendOffset, pos - pLTPLib->SendOffset);
}


BOOL LTPLibSendAuthResultRequestExtInd(PLTPLib pLTPLib, uint8_t copmsk, uint8_t * pOpt, uint8_t * rem_BD, uint8_t rem_BD_Type, uint8_t keyType)
{
    uint16_t   pos     = pLTPLib->SendOffset;
    uint8_t * pBuffer = LTPLibWriteHeader(pLTPLib, &pos, LTP_AUTH_RESULT_REQUEST_EXT_IND, copmsk, pOpt, 0);

    if (!pBuffer)
    {
        return TRUE;
    }

    memcpy(&pBuffer[pos], rem_BD, 6);
    pos += 6;
    pBuffer[pos++] = rem_BD_Type;
    pBuffer[pos++] = keyType;

    return BTLTPTgtSendLTPMessage(pLTPLib->AppHandle, pBuffer, pLTPLib->SendOffset, pos - pLTPLib->SendOffset);
}


BOOL LTPLibSendInternalEventInfo(PLTPLib pLTPLib, uint8_t copmsk, uint8_t * pOpt, uint8_t cause, uint8_t eventType, uint32_t eventInfo)
{
    uint16_t   pos     = pLTPLib->SendOffset;
    uint8_t * pBuffer = LTPLibWriteHeader(pLTPLib, &pos, LTP_INTERNAL_EVENT_INFO, copmsk, pOpt, 0);

    if (!pBuffer)
    {
        return TRUE;
    }

    pBuffer[pos++] = (uint8_t)cause;
    pBuffer[pos++] = (uint8_t)eventType;
    NETLONG2CHAR(&pBuffer[pos], eventInfo);
    pos += 4;

    return BTLTPTgtSendLTPMessage(pLTPLib->AppHandle, pBuffer, pLTPLib->SendOffset, pos - pLTPLib->SendOffset);
}


BOOL LTPLibSendCreateMDLInd(PLTPLib pLTPLib, uint8_t copmsk, uint8_t * pOpt, uint8_t * rem_BD, uint8_t loc_MDL_ID)
{
    return LTPLibSendMessage_BD_BYTE(pLTPLib, copmsk, pOpt, LTP_CREATE_MDL_IND, rem_BD, loc_MDL_ID);
}


BOOL LTPLibSendConnectMDLInfo(PLTPLib pLTPLib, uint8_t copmsk, uint8_t * pOpt, uint8_t loc_MDL_ID, uint16_t maxLTPSize, uint16_t maxAPDUSize)
{
    uint16_t   pos     = pLTPLib->SendOffset;
    uint8_t * pBuffer = LTPLibWriteHeader(pLTPLib, &pos, LTP_CONNECT_MDL_INFO, copmsk, pOpt, 0);

    if (!pBuffer)
    {
        return FALSE;
    }

    pBuffer[pos++] = loc_MDL_ID;
    NETSHORT2CHAR(&pBuffer[pos], maxLTPSize);
    pos += 2;
    NETSHORT2CHAR(&pBuffer[pos], maxAPDUSize);
    pos += 2;

    return (BTLTPTgtSendLTPMessage(pLTPLib->AppHandle, pBuffer, pLTPLib->SendOffset, pos - pLTPLib->SendOffset));
}

BOOL LTPLibSendDisconnectMDLInd(PLTPLib pLTPLib, uint8_t copmsk, uint8_t * pOpt, uint8_t cause, uint8_t loc_MDL_ID)
{
    return LTPLibSendMessage_BYTE_BYTE(pLTPLib, copmsk, pOpt, LTP_DISCONNECT_MDL_IND, cause, loc_MDL_ID);
}


BOOL LTPLibSendDeleteMDLInfo(PLTPLib pLTPLib, uint8_t copmsk, uint8_t * pOpt, uint8_t loc_MDL_ID)
{
    return LTPLibSendMessage_BYTE(pLTPLib, copmsk, pOpt, LTP_DELETE_MDL_INFO, loc_MDL_ID);
}


BOOL LTPLibSendDisconnectMDLRsp(PLTPLib pLTPLib, uint8_t copmsk, uint8_t * pOpt, uint8_t cause, uint8_t loc_MDL_ID)
{
    return LTPLibSendMessage_BYTE_BYTE(pLTPLib, copmsk, pOpt, LTP_DISCONNECT_MDL_RSP, cause, loc_MDL_ID);
}


BOOL LTPLibSendResetRsp(PLTPLib pLTPLib, uint8_t copmsk, uint8_t * pOpt, uint8_t cause)
{
    return LTPLibSendMessage_BYTE(pLTPLib, copmsk, pOpt, LTP_RESET_RSP, cause);
}


BOOL LTPLibSendGATTServiceRegisterRsp(PLTPLib pLTPLib, uint8_t copmsk, uint8_t * pOpt,
                                      uint8_t cause, uint16_t subCause, uint8_t serviceHandle)
{
    uint16_t   pos     = pLTPLib->SendOffset;
    uint8_t * pBuffer = LTPLibWriteHeader(pLTPLib, &pos, LTP_GATT_SERVICE_REGISTER_RSP, copmsk, pOpt, 0);

    if (!pBuffer)
    {
        return FALSE;
    }

    pBuffer[pos++] = cause;
    NETSHORT2CHAR(&pBuffer[pos], subCause);
    pos += 2;
    pBuffer[pos++] = serviceHandle;

    return (BTLTPTgtSendLTPMessage(pLTPLib->AppHandle, pBuffer, pLTPLib->SendOffset, pos - pLTPLib->SendOffset));
}

BOOL LTPLibSendGATTAttributeUpdateStatusInd(PLTPLib pLTPLib, uint8_t copmsk, uint8_t * pOpt,
        uint8_t cause, uint16_t subCause, uint8_t serviceHandle,
        void* requestHandle, uint16_t attribIndex,
        uint8_t * rem_BD, uint8_t rem_BD_Type)
{
    uint16_t   pos     = pLTPLib->SendOffset;
    uint8_t * pBuffer = LTPLibWriteHeader(pLTPLib, &pos, LTP_GATT_ATTRIBUTE_UPDATE_STATUS_IND, copmsk, pOpt, 0);

    if (!pBuffer)
    {
        return FALSE;
    }

    pBuffer[pos++] = cause;
    NETSHORT2CHAR(&pBuffer[pos], subCause);
    pos += 2;
    pBuffer[pos++] = serviceHandle;
    NETLONG2CHAR(&pBuffer[pos], (uint32_t)requestHandle);
    pos += 4;
    NETSHORT2CHAR(&pBuffer[pos], attribIndex);
    pos += 2;
    memcpy(&pBuffer[pos], rem_BD, 6);
    pos += 6;
    pBuffer[pos++] = rem_BD_Type;

    return (BTLTPTgtSendLTPMessage(pLTPLib->AppHandle, pBuffer, pLTPLib->SendOffset, pos - pLTPLib->SendOffset));
}

BOOL LTPLibSendGATTAttributeReadInd(PLTPLib pLTPLib, uint8_t copmsk, uint8_t * pOpt,
                                    uint8_t loc_MDL_ID, uint8_t serviceHandle,
                                    uint16_t attribIndex, uint16_t readOffset)
{
    uint16_t   pos     = pLTPLib->SendOffset;
    uint8_t * pBuffer = LTPLibWriteHeader(pLTPLib, &pos, LTP_GATT_ATTRIBUTE_READ_IND, copmsk, pOpt, 0);

    if (!pBuffer)
    {
        return FALSE;
    }

    pBuffer[pos++] = loc_MDL_ID;
    pBuffer[pos++] = serviceHandle;
    NETSHORT2CHAR(&pBuffer[pos], attribIndex);
    pos += 2;
    NETSHORT2CHAR(&pBuffer[pos], readOffset);
    pos += 2;

    return (BTLTPTgtSendLTPMessage(pLTPLib->AppHandle, pBuffer, pLTPLib->SendOffset, pos - pLTPLib->SendOffset));
}


BOOL LTPLibSendGATTServerStoreInd(PLTPLib pLTPLib, uint8_t copmsk, uint8_t * pOpt,
                                  uint8_t opCode, uint8_t * rem_BD, uint8_t rem_BD_Type,
                                  uint16_t restartHandle, uint8_t * data, uint16_t dataLength)
{
    uint16_t   pos     = pLTPLib->SendOffset;
    uint8_t * pBuffer = LTPLibWriteHeader(pLTPLib, &pos, LTP_GATT_SERVER_STORE_IND, copmsk, pOpt, dataLength);

    if (!pBuffer)
    {
        return FALSE;
    }

    pBuffer[pos++] = opCode;
    memcpy(&pBuffer[pos], rem_BD, 6);
    pos += 6;
    pBuffer[pos++] = rem_BD_Type;
    NETSHORT2CHAR(&pBuffer[pos], restartHandle);
    pos += 2;
    memcpy(&pBuffer[pos], data, dataLength);
    pos += dataLength;

    return (BTLTPTgtSendLTPMessage(pLTPLib->AppHandle, pBuffer, pLTPLib->SendOffset, pos - pLTPLib->SendOffset));
}

BOOL LTPLibSendConnectGATTMDLRsp(PLTPLib pLTPLib, uint8_t copmsk, uint8_t * pOpt,
                                 uint8_t cause, uint8_t * rem_BD, uint8_t rem_BD_Type,
                                 uint8_t loc_MDL_ID, uint8_t loc_MDEP_ID)
{
    uint16_t   pos     = pLTPLib->SendOffset;
    uint8_t * pBuffer = LTPLibWriteHeader(pLTPLib, &pos, LTP_CONNECT_GATT_MDL_RSP, copmsk, pOpt, 0);

    if (!pBuffer)
    {
        return FALSE;
    }

    pBuffer[pos++] = cause;
    memcpy(&pBuffer[pos], rem_BD, 6);
    pos += 6;
    pBuffer[pos++] = rem_BD_Type;
    pBuffer[pos++] = loc_MDL_ID;

    return (BTLTPTgtSendLTPMessage(pLTPLib->AppHandle, pBuffer, pLTPLib->SendOffset, pos - pLTPLib->SendOffset));
}

BOOL LTPLibSendGATTDiscoveryRsp(PLTPLib pLTPLib, uint8_t copmsk, uint8_t * pOpt,
                                uint8_t cause, uint16_t subCause, uint8_t loc_MDL_ID,
                                uint8_t discoveryType)
{
    uint16_t   pos     = pLTPLib->SendOffset;
    uint8_t * pBuffer = LTPLibWriteHeader(pLTPLib, &pos, LTP_GATT_DISCOVERY_RSP, copmsk, pOpt, 0);

    if (!pBuffer)
    {
        return FALSE;
    }

    pBuffer[pos++] = cause;
    NETSHORT2CHAR(&pBuffer[pos], subCause);
    pos += 2;
    pBuffer[pos++] = loc_MDL_ID;
    pBuffer[pos++] = discoveryType;

    return (BTLTPTgtSendLTPMessage(pLTPLib->AppHandle, pBuffer, pLTPLib->SendOffset, pos - pLTPLib->SendOffset));
}

BOOL LTPLibSendGATTAttributeWriteRsp(PLTPLib pLTPLib, uint8_t copmsk, uint8_t * pOpt,
                                     uint8_t cause, uint16_t subCause, uint8_t loc_MDL_ID,
                                     uint8_t writeType)
{
    uint16_t   pos     = pLTPLib->SendOffset;
    uint8_t * pBuffer = LTPLibWriteHeader(pLTPLib, &pos, LTP_GATT_ATTRIBUTE_WRITE_RSP, copmsk, pOpt, 0);

    if (!pBuffer)
    {
        return FALSE;
    }

    pBuffer[pos++] = cause;
    NETSHORT2CHAR(&pBuffer[pos], subCause);
    pos += 2;
    pBuffer[pos++] = loc_MDL_ID;
    pBuffer[pos++] = writeType;

    return (BTLTPTgtSendLTPMessage(pLTPLib->AppHandle, pBuffer, pLTPLib->SendOffset, pos - pLTPLib->SendOffset));
}


BOOL LTPLibSendGATTAttributeNotificationInfo(PLTPLib pLTPLib, uint8_t copmsk, uint8_t * pOpt,
        uint8_t loc_MDL_ID, uint16_t attribHandle,
        uint8_t * data, uint16_t dataLength)
{
    uint16_t   pos     = pLTPLib->SendOffset;
    uint8_t * pBuffer = LTPLibWriteHeader(pLTPLib, &pos, LTP_GATT_ATTRIBUTE_NOTIFICATION_INFO, copmsk, pOpt, dataLength);

    if (!pBuffer)
    {
        return FALSE;
    }

    pBuffer[pos++] = loc_MDL_ID;
    NETSHORT2CHAR(&pBuffer[pos], attribHandle);
    pos += 2;
    memcpy(&pBuffer[pos], data, dataLength);
    pos += dataLength;

    return (BTLTPTgtSendLTPMessage(pLTPLib->AppHandle, pBuffer, pLTPLib->SendOffset, pos - pLTPLib->SendOffset));
}


BOOL LTPLibSendGATTSecurityRsp(PLTPLib pLTPLib, uint8_t copmsk, uint8_t * pOpt,
                               uint8_t cause, uint8_t loc_MDL_ID, uint8_t keyType,
                               uint8_t keySize)
{
    uint16_t   pos     = pLTPLib->SendOffset;
    uint8_t * pBuffer = LTPLibWriteHeader(pLTPLib, &pos, LTP_GATT_SECURITY_RSP, copmsk, pOpt, 0);

    if (!pBuffer)
    {
        return FALSE;
    }

    pBuffer[pos++] = cause;
    pBuffer[pos++] = loc_MDL_ID;
    pBuffer[pos++] = keyType;
    pBuffer[pos++] = keySize;

    return (BTLTPTgtSendLTPMessage(pLTPLib->AppHandle, pBuffer, pLTPLib->SendOffset, pos - pLTPLib->SendOffset));
}

BOOL LTPLibSendGATTMtuSizeInfo(PLTPLib pLTPLib, uint8_t copmsk, uint8_t * pOpt,
                               uint8_t loc_MDL_ID, uint16_t mtuSize)
{
    uint16_t   pos     = pLTPLib->SendOffset;
    uint8_t * pBuffer = LTPLibWriteHeader(pLTPLib, &pos, LTP_GATT_MTU_INFO, copmsk, pOpt, 0);

    if (!pBuffer)
    {
        return FALSE;
    }

    pBuffer[pos++] = loc_MDL_ID;
    NETSHORT2CHAR(&pBuffer[pos], mtuSize);
    pos += 2;

    return (BTLTPTgtSendLTPMessage(pLTPLib->AppHandle, pBuffer, pLTPLib->SendOffset, pos - pLTPLib->SendOffset));
}

BOOL LTPLibSendLEAdvertiseRsp(PLTPLib pLTPLib, uint8_t copmsk, uint8_t * pOpt,
                              uint8_t cause, uint8_t advMode)
{
    return LTPLibSendMessage_BYTE_BYTE(pLTPLib, copmsk, pOpt, LTP_LE_ADVERTISE_RSP, cause, advMode);
}

BOOL LTPLibSendSetRandomAddressRsp(PLTPLib pLTPLib, uint8_t copmsk, uint8_t * pOpt,
                                   uint8_t cause, uint16_t subCause)
{
    uint16_t   pos     = pLTPLib->SendOffset;
    uint8_t * pBuffer = LTPLibWriteExtendHeader(pLTPLib, &pos, LTP_SUB_SET_RANDOM_ADDRESS_RSP, LTP_OPT_MASK_HEADER_CRC8, NULL, 0);

    if (!pBuffer)
    {
        return FALSE;
    }

    pBuffer[pos++] = cause;
    NETSHORT2CHAR(&pBuffer[pos], subCause);
    pos += 2;

    return (BTLTPTgtSendLTPMessage(pLTPLib->AppHandle, pBuffer, pLTPLib->SendOffset, pos - pLTPLib->SendOffset));
}

BOOL LTPLibSendLEAdvertiseParameterSetRsp(PLTPLib pLTPLib, uint8_t copmsk, uint8_t * pOpt,
        uint8_t cause)
{
    return LTPLibSendMessage_BYTE(pLTPLib, copmsk, pOpt, LTP_LE_ADVERTISE_PARAMETER_SET_RSP, cause);
}

BOOL LTPLibSendLEAdvertiseDataSetRsp(PLTPLib pLTPLib, uint8_t copmsk, uint8_t * pOpt,
                                     uint8_t cause, uint8_t dataType)
{
    return LTPLibSendMessage_BYTE_BYTE(pLTPLib, copmsk, pOpt, LTP_LE_ADVERTISE_DATA_SET_RSP, cause, dataType);
}

BOOL LTPLibSendLEScanRsp(PLTPLib pLTPLib, uint8_t copmsk, uint8_t * pOpt,
                         uint8_t cause)
{
    return LTPLibSendMessage_BYTE(pLTPLib, copmsk, pOpt, LTP_LE_SCAN_RSP, cause);
}

BOOL LTPLibSendLEScanInfo(PLTPLib pLTPLib, uint8_t copmsk, uint8_t * pOpt,
                          uint8_t * rem_BD, uint8_t rem_BD_Type, uint8_t advType,
                          uint8_t rssi, uint8_t * data, uint16_t dataLength)
{
    uint16_t   pos     = pLTPLib->SendOffset;
    uint8_t * pBuffer = LTPLibWriteHeader(pLTPLib, &pos, LTP_LE_SCAN_INFO, copmsk, pOpt, dataLength);

    if (!pBuffer)
    {
        return FALSE;
    }

    memcpy(&pBuffer[pos], rem_BD, 6);
    pos += 6;
    pBuffer[pos++] = rem_BD_Type;
    pBuffer[pos++] = advType;
    pBuffer[pos++] = rssi;
    memcpy(&pBuffer[pos], data, dataLength);
    pos += dataLength;

    return (BTLTPTgtSendLTPMessage(pLTPLib->AppHandle, pBuffer, pLTPLib->SendOffset, pos - pLTPLib->SendOffset));
}

BOOL LTPLibSendLEModifyWhitelistRsp(PLTPLib pLTPLib, uint8_t copmsk, uint8_t * pOpt,
                                    uint8_t cause, uint8_t whitelistOp)
{
    return LTPLibSendMessage_BYTE_BYTE(pLTPLib, copmsk, pOpt, LTP_LE_MODIFY_WHITELIST_RSP, cause, whitelistOp);
}

BOOL LTPLibSendLEConnectionUpdateRsp(PLTPLib pLTPLib, uint8_t copmsk, uint8_t * pOpt,
                                     uint8_t cause, uint8_t loc_MDL_ID)
{
    return LTPLibSendMessage_BYTE_BYTE(pLTPLib, copmsk, pOpt, LTP_LE_CONNECTION_UPDATE_RSP, cause, loc_MDL_ID);
}

BOOL LTPLibSendLEConnectionUpdateInd(PLTPLib pLTPLib, uint8_t copmsk, uint8_t * pOpt,
                                     uint8_t loc_MDL_ID, uint16_t connIntervalMin,
                                     uint16_t connIntervalMax, uint16_t connLatency,
                                     uint16_t supervisionTimeout)
{
    uint16_t   pos     = pLTPLib->SendOffset;
    uint8_t * pBuffer = LTPLibWriteHeader(pLTPLib, &pos, LTP_LE_CONNECTION_UPDATE_IND, copmsk, pOpt, 0);

    if (!pBuffer)
    {
        return FALSE;
    }

    pBuffer[pos++] = loc_MDL_ID;
    NETSHORT2CHAR(&pBuffer[pos], connIntervalMin);
    pos += 2;
    NETSHORT2CHAR(&pBuffer[pos], connIntervalMax);
    pos += 2;
    NETSHORT2CHAR(&pBuffer[pos], connLatency);
    pos += 2;
    NETSHORT2CHAR(&pBuffer[pos], supervisionTimeout);
    pos += 2;

    return (BTLTPTgtSendLTPMessage(pLTPLib->AppHandle, pBuffer, pLTPLib->SendOffset, pos - pLTPLib->SendOffset));
}

BOOL LTPLibSendLEConnectionParameterInfo(PLTPLib pLTPLib, uint8_t copmsk, uint8_t * pOpt,
        uint8_t loc_MDL_ID, uint16_t connInterval,
        uint16_t connLatency, uint16_t supervisionTimeout)
{
    uint16_t   pos     = pLTPLib->SendOffset;
    uint8_t * pBuffer = LTPLibWriteHeader(pLTPLib, &pos, LTP_LE_CONNECTION_PARAMETER_INFO, copmsk, pOpt, 0);

    if (!pBuffer)
    {
        return FALSE;
    }

    pBuffer[pos++] = loc_MDL_ID;
    NETSHORT2CHAR(&pBuffer[pos], connInterval);
    pos += 2;
    NETSHORT2CHAR(&pBuffer[pos], connLatency);
    pos += 2;
    NETSHORT2CHAR(&pBuffer[pos], supervisionTimeout);
    pos += 2;

    return (BTLTPTgtSendLTPMessage(pLTPLib->AppHandle, pBuffer, pLTPLib->SendOffset, pos - pLTPLib->SendOffset));
}

BOOL LTPLibSendSetBleTxPowerRsp(PLTPLib pLTPLib, uint8_t tx_power_index,
                                uint8_t cause, uint16_t subCause)
{
    uint16_t   pos     = pLTPLib->SendOffset;
    uint8_t * pBuffer = LTPLibWriteExtendHeader(pLTPLib, &pos, LTP_SUB_SET_LE_TX_POWER_RSP, LTP_OPT_MASK_HEADER_CRC8, NULL, 0);

    if (!pBuffer)
    {
        return FALSE;
    }

    pBuffer[pos++] = tx_power_index;
    pBuffer[pos++] = cause;
    NETSHORT2CHAR(&pBuffer[pos], subCause);
    pos += 2;

    return (BTLTPTgtSendLTPMessage(pLTPLib->AppHandle, pBuffer, pLTPLib->SendOffset, pos - pLTPLib->SendOffset));
}

BOOL LTPLibSendSetDataLengthRsp(PLTPLib pLTPLib, uint8_t loc_MDL_ID,
                                uint8_t cause)
{
    uint16_t   pos     = pLTPLib->SendOffset;
    uint8_t * pBuffer = LTPLibWriteExtendHeader(pLTPLib, &pos, LTP_SUB_SET_DATA_LENGTH_RSP, LTP_OPT_MASK_HEADER_CRC8, NULL, 0);

    if (!pBuffer)
    {
        return FALSE;
    }

    pBuffer[pos++] = loc_MDL_ID;
    pBuffer[pos++] = cause;

    return (BTLTPTgtSendLTPMessage(pLTPLib->AppHandle, pBuffer, pLTPLib->SendOffset, pos - pLTPLib->SendOffset));
}

BOOL LTPLibSendDataLengthChangeInfo(PLTPLib pLTPLib, uint8_t loc_MDL_ID,
                                    uint16_t MaxTxOctets, uint16_t MaxTxTime,
                                    uint16_t MaxRxOctets, uint16_t MaxRxTime)
{
    uint16_t   pos     = pLTPLib->SendOffset;
    uint8_t * pBuffer = LTPLibWriteExtendHeader(pLTPLib, &pos, LTP_SUB_DATA_LENGTH_CHANGE_INFO, LTP_OPT_MASK_HEADER_CRC8, NULL, 0);

    if (!pBuffer)
    {
        return FALSE;
    }

    pBuffer[pos++] = loc_MDL_ID;
    NETSHORT2CHAR(&pBuffer[pos], MaxTxOctets);
    pos += 2;
    NETSHORT2CHAR(&pBuffer[pos], MaxTxTime);
    pos += 2;
    NETSHORT2CHAR(&pBuffer[pos], MaxRxOctets);
    pos += 2;
    NETSHORT2CHAR(&pBuffer[pos], MaxRxTime);
    pos += 2;

    return (BTLTPTgtSendLTPMessage(pLTPLib->AppHandle, pBuffer, pLTPLib->SendOffset, pos - pLTPLib->SendOffset));
}

BOOL LTPLibSendGATTAttributeExecuteWriteRsp(PLTPLib pLTPLib, uint8_t loc_MDL_ID, uint8_t cause, uint16_t subCause)
{
    uint16_t   pos     = pLTPLib->SendOffset;
    uint8_t * pBuffer = LTPLibWriteExtendHeader(pLTPLib, &pos, LTP_SUB_GATT_ATTRIBUTE_EXECUTE_WRITE_RSP, LTP_OPT_MASK_HEADER_CRC8, NULL, 0);

    if (!pBuffer)
    {
        return FALSE;
    }

    pBuffer[pos++] = loc_MDL_ID;
    pBuffer[pos++] = cause;
    NETSHORT2CHAR(&pBuffer[pos], subCause);
    pos += 2;

    return (BTLTPTgtSendLTPMessage(pLTPLib->AppHandle, pBuffer, pLTPLib->SendOffset, pos - pLTPLib->SendOffset));
}

BOOL LTPLibSendGATTAttributeExecuteWriteInd(PLTPLib pLTPLib, uint8_t copmsk, uint8_t * pOpt,
        uint8_t loc_MDL_ID, uint8_t flags)
{
    uint16_t   pos     = pLTPLib->SendOffset;
    uint8_t * pBuffer = LTPLibWriteExtendHeader(pLTPLib, &pos, LTP_SUB_GATT_ATTRIBUTE_EXECUTE_WRITE_IND, copmsk, pOpt, 0);

    if (!pBuffer)
    {
        return FALSE;
    }

    pBuffer[pos++] = loc_MDL_ID;
    pBuffer[pos++] = flags;

    return (BTLTPTgtSendLTPMessage(pLTPLib->AppHandle, pBuffer, pLTPLib->SendOffset, pos - pLTPLib->SendOffset));
}

BOOL LTPLibSendDownloadServiceDatabaseRsp(PLTPLib pLTPLib, uint8_t copmsk, uint8_t * pOpt,
        uint8_t cause)
{
    return LTPLibSendSubMessage_BYTE(pLTPLib, copmsk, pOpt, LTP_SUB_DOWNLOAD_SERVICE_DATABASE_RSP, cause);
}


BOOL LTPLibSendClearServiceDatabaseRsp(PLTPLib pLTPLib, uint8_t copmsk, uint8_t * pOpt,
                                       uint8_t cause)
{
    return LTPLibSendSubMessage_BYTE(pLTPLib, copmsk, pOpt, LTP_SUB_CLEAR_SERVICE_DATABASE_RSP, cause);
}

BOOL LTPLibSendSetTraceLevelRsp(PLTPLib pLTPLib, uint8_t copmsk, uint8_t * pOpt,
                                       uint8_t cause)
{
    return LTPLibSendSubMessage_BYTE(pLTPLib, copmsk, pOpt, LTP_SUB_SET_TRACE_LEVEL_RSP, cause);
}
