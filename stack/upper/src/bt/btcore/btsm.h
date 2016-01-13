/**
********************************************************************************************************
Copyright (c) 2015, Realtek Semiconductor Corporation. All rights reserved.
*********************************************************************************************************
* @file       btsm.h
* @brief     bt security manager
* @details   
*
* @author   	gordon
* @date      	2015-07-10
* @version	v0.1
*/

#if !defined(__BTSM_H)
#define      __BTSM_H

#include <flags.h>
#include <os_message.h>
#include <trace_binary.h>
#undef  TRACE_MODULE_ID     



/** reject authentication on slave if master has oob, but slave has not */
#define X_BT_LE_STRICT_OOB_HANDLING     1
//#define SM_MAX_CONNECTION_COUNT        (BT_MAX_ACL_CONNECTION_COUNT)

/** timerIDs */
#define SM_TIMER_AUTH_RETRY            1
#define SM_TIMER_SMP_TIMEOUT           2
/** timeout after 30s */
#define SMP_TIMEOUT                   30000

#if (F_BT_LOW_ENERGY)
/** SMP OP Codes */
#define LE_SMP_PAIRING_REQUEST                      0x01
#define LE_SMP_PAIRING_RESPONSE                     0x02
#define LE_SMP_PAIRING_CONFIRM                      0x03
#define LE_SMP_PAIRING_RANDOM                       0x04
#define LE_SMP_PAIRING_FAILED                       0x05
#define LE_SMP_ENCRYPTION_INFORMATION               0x06
#define LE_SMP_MASTER_IDENTIFICATION                0x07
#define LE_SMP_IDENTITY_INFORMATION                 0x08
#define LE_SMP_IDENTITY_ADDRESS_INFORMATION         0x09
#define LE_SMP_SIGNING_INFORMATION                  0x0A
#define LE_SMP_SECURITY_REQUEST                     0x0B

/** SMP message lengths */
#define LE_SMP_128BIT_VALUE_LENGTH                  (1 + 16)
#define LE_SMP_PAIRING_REQUEST_LENGTH               (1 + 6)
#define LE_SMP_PAIRING_RESPONSE_LENGTH              (1 + 6)
#define LE_SMP_PAIRING_CONFIRM_LENGTH               LE_SMP_128BIT_VALUE_LENGTH
#define LE_SMP_PAIRING_RANDOM_LENGTH                LE_SMP_128BIT_VALUE_LENGTH
#define LE_SMP_PAIRING_FAILED_LENGTH                (1 + 1)
#define LE_SMP_ENCRYPTION_INFORMATION_LENGTH        LE_SMP_128BIT_VALUE_LENGTH
#define LE_SMP_MASTER_IDENTIFICATION_LENGTH         (1 + 2 + 8)
#define LE_SMP_IDENTITY_INFORMATION_LENGTH          LE_SMP_128BIT_VALUE_LENGTH
#define LE_SMP_IDENTITY_ADDRESS_INFORMATION_LENGTH  (1 + 1 + 6)
#define LE_SMP_SIGNING_INFORMATION_LENGTH           LE_SMP_128BIT_VALUE_LENGTH
#define LE_SMP_SECURITY_REQUEST_LENGTH              (1 + 1)

/** LE_SMP_PAIRING_FAILED causes */
#define LE_SMP_ERROR_SUCCESS                        0x00
#define LE_SMP_ERROR_PASSKEY_ENTRY_FAILED           0x01
#define LE_SMP_ERROR_OOB_NOT_AVAIABLE               0x02
#define LE_SMP_ERROR_AUTHENTICATION_REQUIREMENTS    0x03
#define LE_SMP_ERROR_CONFIRM_VALUE_FAILED           0x04
#define LE_SMP_ERROR_PAIRING_NOT_SUPPORTED          0x05
#define LE_SMP_ERROR_ENCRYPTION_KEY_SIZE            0x06
#define LE_SMP_ERROR_COMMAND_NOT_SUPPORTED          0x07
#define LE_SMP_ERROR_UNSPECIFIED_REASON             0x08
#define LE_SMP_ERROR_REPEATED_ATTEMPTS              0x09
#define LE_SMP_ERROR_INVALID_PARAMETERS             0x0A

/** PairingReq / PairingRsp oobFlag */
#define LE_SMP_OOB_DATA_NOT_PRESENT                 0x00
#define LE_SMP_OOB_DATA_PRESENT                     0x01

/** PairingReq / PairingRsp / SecurityReq authrequirements */
#define LE_SMP_AUTHREQ_BONDING_MASK                 0x03
#define LE_SMP_AUTHREQ_NO_BONDING                   0x00
#define LE_SMP_AUTHREQ_BONDING                      0x01
#define LE_SMP_AUTHREQ_MITM                         0x04

/** PairingReq / PairingRsp ioCap */
#define LE_SMP_IOCAP_DISPLAY_ONLY                   0x00
#define LE_SMP_IOCAP_DISPLAY_YES_NO                 0x01
#define LE_SMP_IOCAP_KEYBOARD_ONLY                  0x02
#define LE_SMP_IOCAP_NOIO                           0x03
#define LE_SMP_IOCAP_KEYBOARD_DISPLAY               0x04

/** PairingReq / PairingRsp initKeyDist/respKeyDist */
#define LE_SMP_KEYDIST_ENCKEY                       0x01
#define LE_SMP_KEYDIST_IDKEY                        0x02
#define LE_SMP_KEYDIST_SIGNKEY                      0x04

/** smpState values */
#define LE_SMPSTATE_PAIRING_REQ_SENT      0x00000001  /**< LE_SMP_PAIRING_REQUEST              */
#define LE_SMPSTATE_PAIRING_REQ_RECVD     0x00000002
#define LE_SMPSTATE_PAIRING_RSP_SENT      0x00000004  /**< LE_SMP_PAIRING_RESPONSE             */
#define LE_SMPSTATE_PAIRING_RSP_RECVD     0x00000008
#define LE_SMPSTATE_PAIRING_CONF_SENT     0x00000010  /**< LE_SMP_PAIRING_CONFIRM              */
#define LE_SMPSTATE_PAIRING_CONF_RECVD    0x00000020
#define LE_SMPSTATE_PAIRING_RAND_SENT     0x00000040  /**< LE_SMP_PAIRING_RANDOM               */
#define LE_SMPSTATE_PAIRING_RAND_RECVD    0x00000080
#define LE_SMPSTATE_PAIRING_FAILED_SENT   0x00000100  /**< LE_SMP_PAIRING_FAILED               */
#define LE_SMPSTATE_PAIRING_FAILED_RECVD  0x00000200
#define LE_SMPSTATE_ENCRYPT_INFO_SENT     0x00000400  /**< LE_SMP_ENCRYPTION_INFORMATION       */
#define LE_SMPSTATE_ENCRYPT_INFO_RECVD    0x00000800
#define LE_SMPSTATE_MASTER_ID_SENT        0x00001000  /**< LE_SMP_MASTER_IDENTIFICATION        */
#define LE_SMPSTATE_MASTER_ID_RECVD       0x00002000
#define LE_SMPSTATE_IDENT_INFO_SENT       0x00004000  /**< LE_SMP_IDENTITY_INFORMATION         */
#define LE_SMPSTATE_IDENT_INFO_RECVD      0x00008000
#define LE_SMPSTATE_IDENT_ADDR_SENT       0x00010000  /**< LE_SMP_IDENTITY_ADDRESS_INFORMATION */
#define LE_SMPSTATE_IDENT_ADDR_RECVD      0x00020000
#define LE_SMPSTATE_SIGN_INFO_SENT        0x00040000  /**< LE_SMP_SIGNING_INFORMATION          */
#define LE_SMPSTATE_SIGN_INFO_RECVD       0x00080000
#define LE_SMPSTATE_SECURITY_REQ_SENT     0x00100000  /**< LE_SMP_SECURITY_REQUEST             */
#define LE_SMPSTATE_SECURITY_REQ_RECVD    0x00200000

#define LE_SMPSTATE_DEVICE_DATA_STORE     0x08000000  /**< waiting for DEVICE_DATA_SET_RESP    */
#define LE_SMPSTATE_LOCAL_CONF_VALID      0x10000000  /**< local confirm value calculated      */
#define LE_SMPSTATE_STK_VALID             0x20000000  /**< STK generated                       */
#define LE_SMPSTATE_STK_REQUESTED         0x40000000  /**< STK requested via LE_LTK_REQ        */
#define LE_SMPSTATE_TIMEOUT_RUNNING       0x80000000  /**< SMP timeout (30s) running           */

/** sspMech values */
#define LE_SMP_MECH_JUST_WORKS                      0x00
#define LE_SMP_MECH_DISPLAY                         0x01
#define LE_SMP_MECH_KEYBOARD                        0x02
#define LE_SMP_MECH_OUT_OF_BAND                     0x03

typedef struct _TLE_SMP_DATA_PHASE2
{
	uint8_t          local_TK[16];           /**< local TK value */
	uint8_t          local_RAND[16];         /**< local MRAND/SRAND */
	uint8_t          local_CONF[16];         /**< local MCONF/SCONF */
	uint8_t          remote_RAND[16];        /**< remote MRAND/SRAND */
	uint8_t          remote_CONF[16];        /**< remote MCONF/SCONF */
} TLE_SMP_DATA_PHASE2,  * LPLE_SMP_DATA_PHASE2;

typedef struct _TLE_SMP_DATA_PHASE3
{
	TDEVICE_DATA_ELEMENT_SECMAN_LTK   lLTK;
	TDEVICE_DATA_ELEMENT_SECMAN_LTK   rLTK;
#if (F_BT_LE_PRIVACY_RESOLVING)
	TDEVICE_DATA_ELEMENT_SECMAN_IRK   rIRK;
#endif
#if (F_BT_LE_DATA_SIGNING)
	TDEVICE_DATA_ELEMENT_SECMAN_CSRK  lCSRK;
	TDEVICE_DATA_ELEMENT_SECMAN_CSRK  rCSRK;
#endif
	uint8_t                              localKeysStored;  /**< LE_SMP_KEYDIST_* */
	uint8_t                              remoteKeysStored; /**< LE_SMP_KEYDIST_* */
} TLE_SMP_DATA_PHASE3,  * LPLE_SMP_DATA_PHASE3;

typedef struct _TLE_SMP_DATA
{
	uint32_t         smpState;               /**< LE_SMPSTATE_*             */

	union _TLE_SMP_DATA_UNION
	{
	TLE_SMP_DATA_PHASE2 ph2;            /**< phase2: TK/RAND/CONF values */
	uint8_t                stk[16];        /**< STK generated in phase 2 */
	TLE_SMP_DATA_PHASE3 ph3;            /**< phase3: LTK/IRK/CSRK key distribution */
	} p;

	uint8_t          remote_initKeyDist;     /**< LE_SMP_KEYDIST_* */
	uint8_t          remote_respKeyDist;     /**< LE_SMP_KEYDIST_* */
	uint8_t          sspMech;                /**< LE_SMP_MECH_* SSP mechanism */

	uint8_t          pairingReq[LE_SMP_PAIRING_REQUEST_LENGTH];          /**< Data from PairingRequest  */
	uint8_t          pairingRsp[LE_SMP_PAIRING_RESPONSE_LENGTH];         /**< Data from PairingResponse */
} TLE_SMP_DATA,  * LPLE_SMP_DATA;

#define LE_CRYPT_HANDLE_LINK      0x0100      /**< low byte of handle is link index */
#define LE_CRYPT_HANDLE_CACHE     0x0200      /**< low byte of handle is cache index */
#define LE_CRYPT_HANDLE_PRIVACY   0x0400      /**< low byte is cryptState for local IRK/BD generation */
#define LE_CRYPT_HANDLE_MASK      0x00FF      /**< mask of handle */
#define LE_CRYPT_HANDLE_IDLE      0xFFFF      /**< no crypt transaction pending, invalid handle */

/** helper for handle in LE_Encrypt / LE_Rand HCI Commands */
#define LE_LINK2HANDLE(pBtSM, pLink)    (LE_CRYPT_HANDLE_LINK | (pLink->index))
#define LE_CACHE2HANDLE(pBtSM, pEntry)  (LE_CRYPT_HANDLE_CACHE | (pEntry - pBtSM->LE_resolveCache))

/** cryptState values */
#define LE_CRYPT_IDLE             0x00 /**< idle */
#define LE_CRYPT_GEN_RAND_LO      0x01 /**< generate MRAND lo */
#define LE_CRYPT_GEN_RAND_HI      0x02 /**< generate MRAND hi */
#define LE_CRYPT_GEN_CONF_1       0x03 /**< generate MCONF step 1 */
#define LE_CRYPT_GEN_CONF_2       0x04 /**< generate MCONF step 2 */
#define LE_CRYPT_CHK_CONF_1       0x05 /**< check SCONF step 1 */
#define LE_CRYPT_CHK_CONF_2       0x06 /**< check SCONF step 2 */
#define LE_CRYPT_GEN_STK          0x07 /**< generate STK */
#define LE_CRYPT_GEN_PASSKEY      0x08 /**< generate 6digit passkey */

#define LE_CRYPT_GEN_LTK_LO       0x11 /**< generate LTK_local lo */
#define LE_CRYPT_GEN_LTK_HI       0x12 /**< generate LTK_local hi */
#define LE_CRYPT_GEN_MA_ID_LO     0x13 /**< generate Master Identification lo */
#define LE_CRYPT_GEN_MA_ID_HI     0x14 /**< generate Master Identification hi */

#define LE_CRYPT_GEN_IRK_LO       0x21 /**< generate IRK_local lo */
#define LE_CRYPT_GEN_IRK_HI       0x22 /**< generate IRK_local hi */
#define LE_CRYPT_GEN_IA_RAND      0x23 /**< generate random address (random part) */
#define LE_CRYPT_GEN_IA_HASH      0x24 /**< generate random address (hash part) */
#define LE_CRYPT_RESOLVE_IRK      0x25 /**< resolve random address with IRK */

#define LE_CRYPT_GEN_CSRK_LO      0x31 /**< generate CSRK_local lo */
#define LE_CRYPT_GEN_CSRK_HI      0x32 /**< generate CSRK_local hi */

#define LE_CRYPT_GEN_STATIC_BD    0x41 /**< generate static private address */
#define LE_CRYPT_GEN_NONRESOLV    0x42 /**< generate non-resolvable address */

#endif /**< (F_BT_LOW_ENERGY) */

/** link state settings */
#define SEC_LINK_IDLE                     0   /**< link is idle, no ongoing action */
#define SEC_LINK_AUTHOR_REQUESTED         1   /**< Authorisation Request send to UL, waiting for _CONF */
#define SEC_LINK_AUTHEN_REQUESTED         2   /**< AUTH_REQ send to HCI, waiting for AUTH_CONF */
#define SEC_LINK_AUTHEN_MITM_REQUESTED    3   /**< AUTH_REQ send to HCI, force MITM, waiting for AUTH_CONF */
#define SEC_LINK_ENCRYPT_REQUESTED        4   /**< ENCRYPT_REQ, wait for confirmation from HCI layer  */
#define SEC_LINK_EXTERNAL_AUTH_REQ        5   /**< AUTH_REQ forwarded from BTMAN to HCI, waiting for AUTH_CONF */
#define SEC_LINK_AUTHEN_RETRY             6   /**< received AUTH_CONF(fail) with stored key, retry after timer */
#define SEC_LINK_SECURITY_REQ             7   /**< got SMP Security Request from slave */
#define SEC_LINK_SECURITY_MITM_REQ        8   /**< got SMP Security Request with MITM requirement from slave */
#define SEC_LINK_PAIRING_REQ              9   /**< got SMP Pairing Request from master */
#define SEC_LINK_REMOTE_SSP              10   /**< remote side triggered Secure Simple Pairing */

/** link mode bit settings */
#define LINKMODE_AUTHEN              0x0001   /**< link is authenticated */
#define LINKMODE_MITM                0x0002   /**< link is authenticated with MITM protection */
#define LINKMODE_ENCRYPTED           0x0004   /**< link is currently encrypted */
#define LINKMODE_AUTHOR              0x0008   /**< link is authorized */
#define LINKMODE_SSP_KNOWN           0x0010   /**< we know regarding SSP capability of the link */
#define LINKMODE_SSP_CAPA            0x0020   /**< link is capable of SSP authentication */
#define LINKMODE_IOCAPS_KNOWN        0x0040   /**< ioCaps of the peer are known */
#define LINKMODE_STORED_KEY          0x0080   /**< link is authenticated using stored linkkey */
#define LINKMODE_AUTHEN_REQUIRED     0x0100   /**< link shall be authenticated */
#define LINKMODE_MITM_REQUIRED       0x0200   /**< link shall be authenticated with MITM protection */
#define LINKMODE_ENCR_REQUIRED       0x0400   /**< link shall be encrypted */
#define LINKMODE_AUTHOR_REQUIRED     0x0800   /**< link shall be authorized */
#define LINKMODE_FORCE_PAIRING       0x1000   /**< do not use stored linkkey, force new pairing */
#define LINKMODE_POLICY_BASED        0x2000   /**< requirements based on policy -> send RESP */

#define SM_LINK_FREE                 0x00
#define SM_LINK_USED                 0x01

#define SECSTAT_IDLE                 0x00
#define SECSTAT_AUTH_REQ             0x01     /**< HCI_AUTH_REQ to HCI */
#define SECSTAT_AUTH_COMPLETE        0x02     /**< HCI_AUTH_CONF from HCI */
#define SECSTAT_SSP_START            0x03     /**< SSP started (numeric comparison, passkey entry or OOB data) */
#define SECSTAT_SSP_COMPLETE         0x04     /**< HCI_SIMPLE_PARING_COMPLETE from HCI */
#define SECSTAT_LINK_KEY_REQ         0x05     /**< HCI_LINK_KEY_REQ from HCI */
#define SECSTAT_LINK_KEY_REQ_RESP    0x06     /**< HCI_COMMAND_COMPLETE(HCI_LINK_KEY_REQ_(NEG_)REPLY) from HCI */
#define SECSTAT_PIN_CODE_REQ         0x07     /**< HCI_PIN_CODE_REQ from HCI */
#define SECSTAT_NEW_LINK_KEY         0x08     /**< HCI_LINK_KEY_NOTIFICATION from HCI */
#define SECSTAT_LINK_DISCONNECTED    0x09     /**< Link got disconnected */
#define SECSTAT_ABORT                0x0A     /**< security reject (incoming BT2.1 without auth/encr reject, MITM not possible) */
#if (F_BT_LOW_ENERGY)
#define SECSTAT_LE_AUTH_REQ          0x10     /**< SECMAN_AUTH_REQ */
#define SECSTAT_SMP_SECURITY_REQ     0x11     /**< LE Master: SMP SecurityRequest received */
#define SECSTAT_SMP_PAIRING_REQ      0x12     /**< LE Slave: SMP PairingRequest received */
#define SECSTAT_LTK_REQ              0x13     /**< LE Slave: LongTermKey Request received */
#define SECSTAT_LE_ENCRYPTION_CHANGE 0x14     /**< LE: Encryption changed event */
#define SECSTAT_LE_KEYDIST_COMPLETE  0x15     /**< LE: key distribution completed */
#endif /**< (F_BT_LOW_ENERGY) */

/** Descriptor of link on security manager level */
typedef struct _TSECLINK 
{
	uint8_t           used;                /**< SM_LINK_XXX */
    uint8_t           index;
	TBdAddr        bd;                  /**< address of connected remote device */
	uint8_t           bdType;              /**< BLUEFACE_BDTYPE_* */
	uint8_t           state;               /**< SEC_LINK_XXX */
	uint16_t           mode;                /**< LINKMODE_XXX */
#if F_BT_BREDR_SUPPORT
    uint8_t           extAuthReqId;        /**< id of ext. AuthReq */
    
	uint16_t           auth_ref;            /**< during AUTHOR/AUTHEN: channel reference */
	uint16_t           auth_cid;            /**< during AUTHOR/AUTHEN: channel id */
	uint16_t           auth_uuid;           /**< during AUTHOR/AUTHEN: uuid */
	uint8_t           auth_source;       /**< during AUTHOR/AUTHEN: moduleId of the request */
	uint8_t           auth_outg;           /**< during AUTHOR/AUTHEN: outgoing channel establishment */

	uint8_t           remote_authRequirements;   /**< from HCI_SSP_IO_CAPABILITY_REPLY (DEVAUTH_SETTING_XXX) */
	uint8_t           remote_ioCapability;       /**< from HCI_SSP_IO_CAPABILITY_REPLY (BLUEFACE_IOCAPA_XXX) */
#endif
	uint8_t           secStat;             /**< tracks the link state for sending SECURITY_STATUS updates */

	uint8_t           keyType;              /**< BLUEFACE_KEYTYPE_* */
	uint8_t           keySize;
    void *         timerHandle;
#if (F_BT_LOW_ENERGY)
	/** from TLEConnectionComplete */
	uint16_t           handle;
	uint8_t           role;                 /**< LE_ROLE_* */
	LPLE_SMP_DATA  pSMPData;
	uint16_t           cryptState;           /**< LE_CRYPT_*  */
	uint8_t           minKeySize;
    uint8_t           ownAddressType;
#if (F_BT_LE_PRIVACY_RESOLVING)
	TBdAddr        unresolved_bd;        /**< unresolved address of remote device */
	uint8_t           unresolved_bdType;    /**< BLUEFACE_BDTYPE_* */
#endif /**< (F_BT_LE_PRIVACY_RESOLVING) */
#endif /**< (F_BT_LOW_ENERGY) */    
} TSECLINK,  * LPTSECLINK;


#if (F_BT_LOW_ENERGY) && (F_BT_LE_PRIVACY_RESOLVING)

/** bond size for positive entries + some negative entries */

#define LE_RESOLVE_CACHE_SIZE                   (8)


#define LE_RESOLVE_ENTRY_STATE_FREE             0x00  /**< entry not used */
#define LE_RESOLVE_ENTRY_STATE_RESOLVING_MASK   0x10
#define LE_RESOLVE_ENTRY_STATE_STORE_GET        0x11  /**< fetching IRK/BD from store */
#define LE_RESOLVE_ENTRY_STATE_RESOLVING        0x12  /**< resolving IRK */
#define LE_RESOLVE_ENTRY_STATE_QUEUE            0x13  /**< currently resolving, queue message */
#define LE_RESOLVE_ENTRY_STATE_RESOLVED_MASK    0x20
#define LE_RESOLVE_ENTRY_STATE_POSITIVE         0x23  /**< resolved_bd is valid */
#define LE_RESOLVE_ENTRY_STATE_NEGATIVE         0x24  /**< random_bd is not known */
#define LE_RESOLVE_ENTRY_ERROR_INVALID_TYPE     0x81
#define LE_RESOLVE_ENTRY_ERROR_NO_RESSOURCE     0x82

/** LE random resolvable address resolution cache entry */
typedef struct _TRESOLVEENTRY
{
	uint8_t          state;          /**< LE_CACHE_STATE_* */
	uint8_t          resolved_bdType;
	uint8_t          resolved_bd[BD_ADDR_SIZE];
	uint8_t          random_bd[BD_ADDR_SIZE];

	uint16_t          restartHandle;  /**< request handle for next IRK */
	uint32_t         timestamp;      /**< last use (os ticks) */
} TRESOLVEENTRY, * PRESOLVEENTRY;
#endif /**< (F_BT_LOW_ENERGY) && (F_BT_LE_PRIVACY_RESOLVING) */

#define LE_PRIVACY_ENABLE       0x01    /**< privacy mode enabled, use random address */
#define LE_PRIVACY_VALID_IRK    0x02    /**< local IRK is valid */
#define LE_PRIVACY_VALID_BD     0x04    /**< random resolvable BD is valid */
#define LE_PRIVACY_FORCE_NEW_BD 0x08    /**< create new random resolvable BD */

typedef struct tagBtSM
{
	uint8_t               deferredQueueID;

#if (F_BT_LOW_ENERGY)
	uint8_t               cryptQueueID;        /**< crypto transaction queue      */
	uint16_t               cryptHandle;         /**< handle of current transaction */
	uint8_t               localBD[6];          /**< needed for PairingExchange    */
    uint8_t               localBdType;
    uint8_t               randomBD[6]; 
#endif

	MESSAGE_T          message;
   
	TSEC_CONF_DEVICE_SECURITY devSecurity;                      /**< device level security settings from application layer */
#if F_BT_BREDR_SUPPORT
    TSEC_CONF_SECURITY        secEntries[BT_SECMAN_POLICY_COUNT]; /**< service related security requirements */
#endif
#if F_BT_LE_BT41_SUPPORT
    TSEC_CONF_LE_SECURITY     secLeEntries[BT_LE_SECMAN_POLICY_COUNT];
#endif
    TSECLINK*   plinksDon;   /* link state descriptors */
    TSECLINK*   plinksDoff;   /* link state descriptors */

#if (F_BT_LOW_ENERGY)
	uint8_t           LE_maxKeySize;
#if (F_BT_LE_PRIVACY_MODE)
	uint8_t           LE_privacyMode;          /**< LE_PRIVACY_* */
	uint8_t           LE_localRandomBD[BD_ADDR_SIZE];
	uint8_t           LE_localIRK[16];
#endif /**< (F_BT_LE_PRIVACY_MODE) */
#if (F_BT_LE_PRIVACY_RESOLVING)
	TRESOLVEENTRY  LE_resolveCache[LE_RESOLVE_CACHE_SIZE];
#endif /**< (F_BT_LE_PRIVACY_RESOLVING) */
	uint32_t          LE_fixedDisplayValue;
#endif /**< (F_BT_LOW_ENERGY) */

} TBtSM, *PBtSM;

#endif /**< !defined(__BTSM_H) */
