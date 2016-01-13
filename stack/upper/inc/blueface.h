/**
********************************************************************************************************
Copyright (c) 2015, Realtek Semiconductor Corporation. All rights reserved.
*********************************************************************************************************
* @file       blueface.h
* @brief     
* @details   
*
* @author   	gordon
* @date      	2015-07-13
* @version	v0.1
*/

#if ! defined (__BLUEFACE_H)
#define __BLUEFACE_H
#include <flags.h>
#include <bttypes.h>
#include <bterrcod.h>         /**< form this file we get all error code definitions */

#if defined(_M_IX86) || defined (__BORLANDC__)    /**< ( _MSC_VER >= 700 ) || Borland*/
/** 4103 "'%Fs' : used #pragma pack to change alignment " */
#pragma warning(disable : 4103)
#define BYTE_ALIGNMENT  1
#endif

#if defined (BYTE_ALIGNMENT)
#pragma pack(1)
#endif

#if defined(PRAGMA_PACK)
#pragma pack(1)
#endif

#ifndef PACKED
#define PACKED(a) a
#endif

#define BLUEFACE_MAJOR_VERSION   1
#define BLUEFACE_MINOR_VERSION   9
#define BLUEFACE_VERSION         ((BLUEFACE_MAJOR_VERSION << 8) | BLUEFACE_MINOR_VERSION)

/** GSM 07.10 MSC command status bits */
#define BLUEFACE_DV_BIT     0x80
#define BLUEFACE_IC_BIT     0x40
/** 0x20 / 0x10 are reserved */
#define BLUEFACE_RTR_BIT    0x08
#define BLUEFACE_RTC_BIT    0x04
#define BLUEFACE_FLOW_BIT   0x02     /**< Flow Bit Position in RFCOMM MSC Command */
/** 0x01 is the extension bit */

/** PSM value settings                */
#define BLUEFACE_PSM_SDP                        0x01
#define BLUEFACE_PSM_RFCOMM                     0x03
#define BLUEFACE_PSM_TCSBIN                     0x05
#define BLUEFACE_PSM_TCSBIN_CORDLESS            0x07
#define BLUEFACE_PSM_BNEP                       0x0F
#define BLUEFACE_PSM_HID_CONTROL                0x11
#define BLUEFACE_PSM_HID_INTERRUPT              0x13
#define BLUEFACE_PSM_AVCTP                      0x17
#define BLUEFACE_PSM_AVDTP                      0x19
#define BLUEFACE_PSM_UDI_CPLANE                 0x1D
#define BLUEFACE_PSM_ATT                        0x1F
#define BLUEFACE_PSM_UPFL2C                   0x1231  /**< UnPlugFest Test */

#define BLUEFACE_PSM_SCO                      0x1234  /**< this is not really a PSM value */
/** removed #define BLUEFACE_PSM_OBEX          0x1236     this is not really a PSM value */
#define BLUEFACE_PSM_L2CAP                    0x1238  /**< this is not really a PSM value */
#define BLUEFACE_PSM_HCRP_CLIENT              0x123A  /**< this is not really a PSM value */
#define BLUEFACE_PSM_GATT                     0x123C  /**< this is not really a PSM value */

/** external defined vendor PSM */
#define BLUEFACE_PSM_NOKIA_FBUS               0x5401  /**< FBUS over Bluetooth L2CAP */

/** CON_RESP reply codes */
#define BLUEFACE_CON_ACCEPT              0       /**< accept connection */
#define BLUEFACE_CON_REJECT              1       /**< reject connection */
#define BLUEFACE_CON_IGNORE              2       /**< ignore connection */
#define BLUEFACE_CON_ALERT               3       /**< alert  connection */

/** SCO/ESCO packet mask codes */
#define BLUEFACE_HCI_PACKET_TYPE_HV1          0x0001
#define BLUEFACE_HCI_PACKET_TYPE_HV2          0x0002
#define BLUEFACE_HCI_PACKET_TYPE_HV3          0x0004
#define BLUEFACE_HCI_PACKET_TYPE_EV3          0x0008
#define BLUEFACE_HCI_PACKET_TYPE_EV4          0x0010
#define BLUEFACE_HCI_PACKET_TYPE_EV5          0x0020
#define BLUEFACE_HCI_PACKET_TYPE_NOT_2EV3     0x0040
#define BLUEFACE_HCI_PACKET_TYPE_NOT_3EV3     0x0080
#define BLUEFACE_HCI_PACKET_TYPE_NOT_2EV5     0x0100
#define BLUEFACE_HCI_PACKET_TYPE_NOT_3EV5     0x0200
#define BLUEFACE_HCI_PACKET_TYPE_ALL_SCO      0xfc3f

#ifdef __cplusplus
extern "C" {
#endif

/** Version History on the level of BLUEFACE_MAJOR / BLUEFACE_MINOR*/
/**     1.03   unions added*/
/**     1.04   sco support and first tcs elements */

/** some common naming conventions used in BLUEFACE*/
/**     - a HANDLE is allocated by BLUEFACE and is used to identify a resouce by BLUEFACE*/
/**     - a CONTEXT is allocated by an APPLICATION and is used to identify a*/
/**       resource on APPLICATION level.*/
/**     - a RESOURCE is either an application id or a connection id */

#define     BAPPHANDLE        PVOID
#define     BAPPCONTEXT       PVOID
#define     BLINKHANDLE       PVOID
#define     BLINKCONTEXT      PVOID
#define     BREFERENCE        PVOID
#define     BDEVPLUGINHANDLE  PVOID

typedef enum                    /**< !! ATTENTION do NOT change Sequence !! */
{
    blueFaceNoError,
    blueFaceIllVersion,
    blueFaceIllApp,
    blueFaceIllMessage,
    blueFaceNoResources,
    blueFaceIllParameter,
    blueFaceDriverRemoved,
    blueFaceQueueFull,
    blueFaceBusy,
    blueFaceTransportError,
    blueFaceLicenseViolation,
    blueFacePsmNotImplemented
#if (VARIABLE_ENUM_SIZE==TRUE)
    ,blueFaceForceSize = 0xFFFFFFFF       /**< force a 32 bit enum */
#endif /**< FORCE_32BIT_ENUM */
} blueFaceStatus;


/** device class and piconet operation mode definitions for BT_HCI_CLASS_REQ                                    */
/** HCI major device classes */
#define HCI_MAJOR_DEVICE_CLASS_MISCELLANEOUS          ((uint32_t)0x000000)
#define HCI_MAJOR_DEVICE_CLASS_COMPUTER               ((uint32_t)0x000100)
#define HCI_MAJOR_DEVICE_CLASS_PHONE                  ((uint32_t)0x000200)
#define HCI_MAJOR_DEVICE_CLASS_LAN_ACCESS_POINT       ((uint32_t)0x000300)
#define HCI_MAJOR_DEVICE_CLASS_AUDIO                  ((uint32_t)0x000400)
#define HCI_MAJOR_DEVICE_CLASS_PERIPHERAL             ((uint32_t)0x000500)
#define HCI_MAJOR_DEVICE_CLASS_IMAGING                ((uint32_t)0x000600)
#define HCI_MAJOR_DEVICE_CLASS_HEALTH                 ((uint32_t)0x000900)
#define HCI_MAJOR_DEVICE_CLASS_UNCLASSIFIED           ((uint32_t)0x001F00)

/** minor device class global                     */
#define HCI_MINOR_DEVICE_CLASS_UNCLASSIFIED           ((uint32_t)0x000000)

/** major class= COMPUTER --> minor class         */
#define HCI_MINOR_DEVICE_CLASS_DESKTOP_WORKSTATION    ((uint32_t)0x000004)
#define HCI_MINOR_DEVICE_CLASS_SERVER_CLASS_COMPUTER  ((uint32_t)0x000008)
#define HCI_MINOR_DEVICE_CLASS_LAPTOP                 ((uint32_t)0x00000C)
#define HCI_MINOR_DEVICE_CLASS_HANDHELD               ((uint32_t)0x000010)
#define HCI_MINOR_DEVICE_CLASS_PALM_SIZED             ((uint32_t)0x000014)
#define HCI_MINOR_DEVICE_CLASS_WEARABLE_COMPUTER      ((uint32_t)0x000018)

/** major class= PHONE --> minor class            */
#define HCI_MINOR_DEVICE_CLASS_CELLULAR               ((uint32_t)0x000004)
#define HCI_MINOR_DEVICE_CLASS_CORDLESS               ((uint32_t)0x000008)
#define HCI_MINOR_DEVICE_CLASS_SMART_PHONE            ((uint32_t)0x00000C)
#define HCI_MINOR_DEVICE_CLASS_WIRED_MODEM            ((uint32_t)0x000010)
#define HCI_MINOR_DEVICE_CLASS_VOICE_GATEWAY          ((uint32_t)0x000010) /**< 10 is ok!! */
#define HCI_MINOR_DEVICE_CLASS_ISDN_GATEWAY           ((uint32_t)0x000014)

/** major class= LAN_ACCESS_POINT --> minor class */
#define HCI_MINOR_DEVICE_CLASS_FULLY_AVAILABLE        ((uint32_t)0x000000)
#define HCI_MINOR_DEVICE_CLASS_1_17_PERCENT_UTILIZED  ((uint32_t)0x000020)
#define HCI_MINOR_DEVICE_CLASS_17_33_PERCENT_UTILIZED ((uint32_t)0x000040)
#define HCI_MINOR_DEVICE_CLASS_33_50_PERCENT_UTILIZED ((uint32_t)0x000060)
#define HCI_MINOR_DEVICE_CLASS_50_67_PERCENT_UTILIZED ((uint32_t)0x000080)
#define HCI_MINOR_DEVICE_CLASS_67_83_PERCENT_UTILIZED ((uint32_t)0x0000A0)
#define HCI_MINOR_DEVICE_CLASS_83_99_PERCENT_UTILIZED ((uint32_t)0x0000C0)
#define HCI_MINOR_DEVICE_CLASS_NO_SERVICE_AVAILABLE   ((uint32_t)0x0000E0)

/** major class= AUDIOVIDEO --> minor class */
#define HCI_MINOR_DEVICE_CLASS_HEADSET                ((uint32_t)0x000004)
#define HCI_MINOR_DEVICE_CLASS_HANDSFREE              ((uint32_t)0x000008)
#define HCI_MINOR_DEVICE_CLASS_MICROPHONE             ((uint32_t)0x000010)
#define HCI_MINOR_DEVICE_CLASS_LOUDSPEAKER            ((uint32_t)0x000014)
#define HCI_MINOR_DEVICE_CLASS_HEADPHONES             ((uint32_t)0x000018)
#define HCI_MINOR_DEVICE_CLASS_PORTABLEAUDIOLE        ((uint32_t)0x00001c)
#define HCI_MINOR_DEVICE_CLASS_CARAUDIO               ((uint32_t)0x000020)
#define HCI_MINOR_DEVICE_CLASS_SETTOPBOX              ((uint32_t)0x000024)
#define HCI_MINOR_DEVICE_CLASS_HIFIAUDIO              ((uint32_t)0x000028)
#define HCI_MINOR_DEVICE_CLASS_VCR                    ((uint32_t)0x00002c)
#define HCI_MINOR_DEVICE_CLASS_VIDEOCAMERA            ((uint32_t)0x000030)
#define HCI_MINOR_DEVICE_CLASS_CAMCORDER              ((uint32_t)0x000034)
#define HCI_MINOR_DEVICE_CLASS_VIDEOMONITOR           ((uint32_t)0x000038)
#define HCI_MINOR_DEVICE_CLASS_VIDEODISPLAYSPEAKER    ((uint32_t)0x00003c)
#define HCI_MINOR_DEVICE_CLASS_VIDEOCONFERENCING      ((uint32_t)0x000040)
#define HCI_MINOR_DEVICE_CLASS_GAMINGTOY              ((uint32_t)0x000048)

/** major class= PERIPHERAL --> minor class */
#define HCI_MINOR_DEVICE_CLASS_KEYBOARD               ((uint32_t)0x000040)
#define HCI_MINOR_DEVICE_CLASS_POINTING_DEVICE        ((uint32_t)0x000080)
#define HCI_MINOR_DEVICE_CLASS_COMBO_KEY_POINTING     ((uint32_t)0x0000C0)
#define HCI_MINOR_DEVICE_CLASS_PERIMULT_UNCATEGORIZED ((uint32_t)0x000000)   /**< multifunction device definitions */
#define HCI_MINOR_DEVICE_CLASS_PERIMULT_JOYSTICK      ((uint32_t)0x000004)
#define HCI_MINOR_DEVICE_CLASS_PERIMULT_GAMEPAD       ((uint32_t)0x000008)
#define HCI_MINOR_DEVICE_CLASS_PERIMULT_REMOTECONTROL ((uint32_t)0x00000C)
#define HCI_MINOR_DEVICE_CLASS_PERIMULT_SENSING       ((uint32_t)0x000010)
#define HCI_MINOR_DEVICE_CLASS_PERIMULT_DIGITIZER     ((uint32_t)0x000014)
#define HCI_MINOR_DEVICE_CLASS_PERIMULT_CARDREADER    ((uint32_t)0x000018)


/** major class= IMAGING */
#define HCI_MINOR_DEVICE_IMAGE_DISPLAY                ((uint32_t)0x000010)    /**< may be combined use as mask */
#define HCI_MINOR_DEVICE_IMAGE_CAMERA                 ((uint32_t)0x000020)    /**< may be combined use as mask */
#define HCI_MINOR_DEVICE_IMAGE_SCANNER                ((uint32_t)0x000040)    /**< may be combined use as mask */
#define HCI_MINOR_DEVICE_IMAGE_PRINTER                ((uint32_t)0x000080)    /**< may be combined use as mask */

/** major class= HEALTH --> minor class */
#define HCI_MINOR_DEVICE_HEALTH_BLOODPRESSURE         ((uint32_t)0x0004)
#define HCI_MINOR_DEVICE_HEALTH_THERMOMETER           ((uint32_t)0x0008)
#define HCI_MINOR_DEVICE_HEALTH_WEIGHINGSCALE         ((uint32_t)0x000C)
#define HCI_MINOR_DEVICE_HEALTH_GLUCOSEMETER          ((uint32_t)0x0010)
#define HCI_MINOR_DEVICE_HEALTH_PULSEOXIMETER         ((uint32_t)0x0014)
#define HCI_MINOR_DEVICE_HEALTH_HEARTPULSERATE        ((uint32_t)0x0018)
#define HCI_MINOR_DEVICE_HEALTH_DATADISPLAY           ((uint32_t)0x001C)
#define HCI_MINOR_DEVICE_HEALTH_STEPCOUNTER           ((uint32_t)0x0020)

/** HCI service classes     */
#define HCI_SERVICE_CLASS_LIMITED_DISCOVERABLE_MODE   ((uint32_t)0x002000)
#define HCI_SERVICE_CLASS_RESERVED1                   ((uint32_t)0x004000)
#define HCI_SERVICE_CLASS_RESERVED2                   ((uint32_t)0x008000)
#define HCI_SERVICE_CLASS_POSITIONING                 ((uint32_t)0x010000)
#define HCI_SERVICE_CLASS_NETWORKING                  ((uint32_t)0x020000)
#define HCI_SERVICE_CLASS_RENDERING                   ((uint32_t)0x040000)
#define HCI_SERVICE_CLASS_CAPTURING                   ((uint32_t)0x080000)
#define HCI_SERVICE_CLASS_OBJECT_TRANSFER             ((uint32_t)0x100000)
#define HCI_SERVICE_CLASS_AUDIO                       ((uint32_t)0x200000)
#define HCI_SERVICE_CLASS_TELEPHONY                   ((uint32_t)0x400000)
#define HCI_SERVICE_CLASS_INFORMATION                 ((uint32_t)0x800000)

/** HCI COD masks */
#define HCI_SERVICE_CLASS_DEVICE_MASK                 ((uint32_t)0x00001FFF)
#define HCI_SERVICE_CLASS_SERVICE_MASK                ((uint32_t)0x00FFE000)

/** Piconet operation mode */
#define HCI_PICONET_SLAVE_REQUIRED            4
#define HCI_PICONET_SLAVE_PREFERRED           3
#define HCI_PICONET_MASTER_REQUIRED           2
#define HCI_PICONET_MASTER_PREFERRED          1
#define HCI_PICONET_MASTER_DONTCARE           0


/** RPN Parameters and structers, cf. TS 07.10 */
/** Octet 2 */
#define BLUEFACE_RPN_B_2K       0x00
#define BLUEFACE_RPN_B_4K       0x01
#define BLUEFACE_RPN_B_7K       0x02
#define BLUEFACE_RPN_B_9K       0x03
#define BLUEFACE_RPN_B_19K      0x04
#define BLUEFACE_RPN_B_38K      0x05
#define BLUEFACE_RPN_B_57K      0x06
#define BLUEFACE_RPN_B_115K     0x07
#define BLUEFACE_RPN_B_230K     0x08
#define BLUEFACE_RPN_B_DEFAULT  0x03

/** Octet 3 */
/** bit masks */
#define BLUEFACE_RPN_D1_BIT     0x001
#define BLUEFACE_RPN_D2_BIT     0x002
#define BLUEFACE_RPN_D_BITS     0x003
#define BLUEFACE_RPN_S_BIT      0x004
#define BLUEFACE_RPN_P_BIT      0x008
#define BLUEFACE_RPN_PT1_BIT    0x010
#define BLUEFACE_RPN_PT2_BIT    0x020
#define BLUEFACE_RPN_PT_BITS    0x030

/** parameter values */
#define BLUEFACE_RPN_D_5BITS    0x00
#define BLUEFACE_RPN_D_6BITS    0x02
#define BLUEFACE_RPN_D_7BITS    0x01
#define BLUEFACE_RPN_D_8BITS    0x03
#define BLUEFACE_RPN_D_DEFAULT  0x03

#define BLUEFACE_RPN_S_1STOP    0x00
#define BLUEFACE_RPN_S_15STOP   0x04

#define BLUEFACE_RPN_P_NONE     0x00
#define BLUEFACE_RPN_P_PARITY   0x08

#define BLUEFACE_RPN_PT_ODD     0x000
#define BLUEFACE_RPN_PT_EVEN    0x020
#define BLUEFACE_RPN_PT_MARK    0x010
#define BLUEFACE_RPN_PT_SPACE   0x030

#define BLUEFACE_RPN_OC3_DEFAULT (BLUEFACE_RPN_D_8BITS | BLUEFACE_RPN_S_1STOP | BLUEFACE_RPN_P_NONE)

/** Octet 4 */
#define BLUEFACE_RPN_FLC_XONIN  0x001
#define BLUEFACE_RPN_FLC_XONOUT 0x002
#define BLUEFACE_RPN_FLC_RTRIN  0x004
#define BLUEFACE_RPN_FLC_RTROUT 0x008
#define BLUEFACE_RPN_FLC_RTCIN  0x010
#define BLUEFACE_RPN_FLC_RTCOUT 0x020

/** Octet 5 */
#define BLUEFACE_RPN_XON_DEFAULT  0x11

/** Octet 6 */
#define BLUEFACE_RPN_XOFF_DEFAULT 0x13

/** Octet 7 */
#define BLUEFACE_RPN_PM1_BITRATE     0x00001
#define BLUEFACE_RPN_PM1_DATABITS    0x00002
#define BLUEFACE_RPN_PM1_STOPBITS    0x00004
#define BLUEFACE_RPN_PM1_PARITY      0x00008
#define BLUEFACE_RPN_PM1_PARITYTYPE  0x00010
#define BLUEFACE_RPN_PM1_XONCHAR     0x00020
#define BLUEFACE_RPN_PM1_XOFFCHAR    0x00040

/** Octet 8 */
#define BLUEFACE_RPN_PM2_XONIN       0x00001
#define BLUEFACE_RPN_PM2_XONOUT      0x00002
#define BLUEFACE_RPN_PM2_RTRIN       0x00004
#define BLUEFACE_RPN_PM2_RTROUT      0x00008
#define BLUEFACE_RPN_PM2_RTCIN       0x00010
#define BLUEFACE_RPN_PM2_RTCOUT      0x00020

typedef PACKED(struct) _TRPN{
    uint8_t    dlci;
    uint8_t    baudrate;
    uint8_t    oc3;
    uint8_t    flctl;
    uint8_t    xon;
    uint8_t    xoff;
    uint8_t    pm1;
    uint8_t    pm2;
} TRPN, * PRPN;
typedef TRPN * LPRPN;

/** The General- and Device-Specific Inquiry Access Codes used on Inquiry Command and Write IAC */
/**   GAIC - General/Unlimited Inquiry Access Code (default)*/
/**   LIAC - Limited Dedicated Inquiry Access Code */
/**   DIAC - Dedicated Inquiry Access Code indexes are in range from 2 to 63  */
#define IAC_GIAC_IDX              ((uint8_t)0)
#define IAC_LIAC_IDX              ((uint8_t)1)
#define IAC_DIAC_FIRST_IDX        ((uint8_t)2)
#define IAC_DIAC_LAST_IDX         ((uint8_t)63)


/** typedef enum tagblueFaceEvent */
typedef enum tagblueFaceEvent
{
	BT_CON_REQ = 0x0004,

    /** data traffic */
    BT_DATA_REQ = 0x0020,

    /** BTSEC / Device data storage/retrieval */
    BT_DEVICE_DATA_GET_IND,
    BT_DEVICE_DATA_GET_RESP,
    BT_DEVICE_DATA_SET_IND,
    BT_DEVICE_DATA_SET_RESP

#if (VARIABLE_ENUM_SIZE==TRUE)
    ,BT_FORCE_SIZE = 0xFFFFFFFF       /**< force a 32 bit enum */
#endif /**< FORCE_32BIT_ENUM */
 } TblueFaceEvent;

/** BD types for BR/EDR and LE peers (values MUST match        */
/** TBlueAPI_RemoteBDType definitions in blueapi_types.h !!!!) */
#define BLUEFACE_BDTYPE_BR_EDR            0
#define BLUEFACE_BDTYPE_LE_PUBLIC         2
#define BLUEFACE_BDTYPE_LE_RANDOM         3
#define BLUEFACE_BDTYPE_LE_MASK           2
#define BLUEFACE_BDTYPE_ANY               4
#define BLUEFACE_BDTYPE_LE_RESOLVED_MASK  8

#define BLUEFACE_CON_TYPE_BR_EDR    1
#define BLUEFACE_CON_TYPE_LE        2
#define AUTHEN_SETTING_NOSECURITY                   0
#define AUTHEN_SETTING_UNAUTHENTICATED_ENCRYTION    1
#define AUTHEN_SETTING_AUTHENTICATED_ENCRYTION      2
#define AUTHEN_SETTING_UNAUTHENTICATED_DATA_SIGNING 3
#define AUTHEN_SETTING_AUTHENTICATED_DATA_SIGNING   4
#define AUTHEN_SETTING_AUTHORIZATION                5

/** LE_RANDOM BD types MSB (bd[5]) values */
#define BLUEFACE_RANDBD_NONRESOLVABLE     0x00
#define BLUEFACE_RANDBD_STATIC            0xC0
#define BLUEFACE_RANDBD_RESOLVABLE        0x40
#define BLUEFACE_RANDBD_MASK              0xC0

/** L2CAP Configuration parameter */
/** L2CAP */
typedef PACKED(struct) _TFlowSpec
{
    uint8_t              flags;
    uint8_t              serviceType;
    uint32_t             tokenRate;
    uint32_t             tokenBucket;
    uint32_t             peakBandwidth;
    uint32_t             latency;
    uint32_t             delayVariation;
} TFlowSpec;
typedef TFlowSpec  * LPFlowSpec;


                                                  /**< (1 << mode) */
#define BLUEFACE_L2CAP_MODE_BASIC                    (1 << 0x00)    /**< Basic L2CAP mode */
#if 0    /**< not implemented */
#define BLUEFACE_L2CAP_MODE_RETRANSMISSION           (1 << 0x01)    /* Retransmission mode */
#define BLUEFACE_L2CAP_MODE_FLOWCONTROL              (1 << 0x02)    /* Flow control mode */
#endif
#define BLUEFACE_L2CAP_MODE_ENHANCED_RETRANSMISSION  (1 << 0x03)    /**< Enhanced Retransmission mode */
#define BLUEFACE_L2CAP_MODE_STREAMING                (1 << 0x04)    /**< Streaming mode */

typedef PACKED(struct) _TFlowControl
{
    uint8_t              txWindowSize;
    uint8_t              maxTransmit;
    uint16_t              retransmissionTimeout;
    uint16_t              monitorTimeout;
    uint16_t              maxPDUSize;
} TFlowControl;
typedef TFlowControl  * LPFlowControl;

typedef PACKED(struct) _TCONF_L2CAP
{
    uint16_t              mode;
    uint16_t              mtuSize;       /**< incoming MTU - not used in CON_REQ (frameSize) */
    uint16_t              flushTO;       /**< Flush timeout setting */
    TFlowSpec         flow;          /**< QOS Parameters */
    TFlowControl      flc;           /**< Flow control */
    uint8_t              fcs;           /**< frame check sequence: 0 - no FCS */
                                     /**<                       1 - 16-bit FCS */
    uint8_t              maxDsBuffer;   /**< max. downstream buffer */
} TCONF_L2CAP;
typedef TCONF_L2CAP  * LPCONF_L2CAP;

/** CON_REQ */
typedef PACKED(struct) _TCON_REQ_RFC
{
    uint8_t              serverChannel; /**< RFCOMM serverchannel */
    uint8_t              mscState;      /**< Initial MSC status */
    uint8_t              creditBased;   /**< Flow Control Type requested */
    uint8_t              encrypted;     /**< Connection should be encrypted */
    uint8_t              maxCredits;    /**< max no of credits in upstream direction */
} TCON_REQ_RFC;

typedef struct _TCON_REQ_SCO
{
    uint16_t            packetType;    /**< Packet types to use for voice connection (HV1/HV2/HV3) */
    uint8_t            encrypted;    /* Connection should be encrypted */

    /** extensions regarding eSCO control: all parameters from HCI layer */
    uint32_t           transmitBandwidth;
    uint32_t           receiveBandwidth;
    uint16_t            maxLatency;
    uint16_t            content;
    uint8_t            retry;
} TCON_REQ_SCO, *PCON_REQ_SCO;

typedef PACKED(struct) _TCON_REQ_L2CAP
{
    uint16_t              psm;
    TCONF_L2CAP       conf;
    uint8_t              encrypted;     /**< Connection should be encrypted */
} TCON_REQ_L2CAP;

typedef struct _TCON_REQ_GATT
{
    uint8_t              bdType;                /**< BLUEFACE_BDTYPE_ */
    uint8_t              localBdType;
    uint16_t              mtuSize;               /**XXXXMJMJ */

    uint16_t              scanInterval;
    uint16_t              scanWindow;
    uint16_t              connIntervalMin;
    uint16_t              connIntervalMax;
    uint16_t              connLatency;
    uint16_t              supervisionTimeout;
    uint16_t              CE_Length;
} TCON_REQ_GATT, *PCON_REQ_GATT;

typedef union _TCON_REQ_P
{
     TCON_REQ_RFC    rfc;
     TCON_REQ_L2CAP  l2cap;
     TCON_REQ_GATT   gatt;
     TCON_REQ_SCO    sco;
} TCON_REQ_P;

typedef struct _TCON_REQ
{
    BLINKCONTEXT      bLinkContext;  /**< Context to use for this connection */
    TBdAddr           bd;            /**< Target Bluetooth Address */
    uint16_t              frameSize;     /**< frameSize to use on this connection */
    uint16_t              psm;           /**< Target protocol */
    uint16_t              uuid;          /**< UUID */
    TCON_REQ_P        p;             /**< protocol specific data */
} TCON_REQ;

/** CON_IND */
typedef PACKED(struct) _TCON_IND_RFC
{
    uint8_t              serverChannel; /**< serverChannel requested */
    uint8_t              creditBased;   /**< Flow Control Tape requested */
    uint8_t              outgoing;      /**< Triggered by outgoing user channel (if serverchannel==0) */
} TCON_IND_RFC;

typedef PACKED(struct) _TCON_IND_L2CAP
{
    uint16_t              lcid;
    uint16_t              psm;
    uint8_t              id;
    uint8_t              outg;
} TCON_IND_L2CAP;

typedef PACKED(struct) _TCON_IND_GATT
{
    uint8_t              bdType;                /**< BLUEFACE_BDTYPE_* */
    uint16_t              mtuSize;               /** XXXXMJMJ */
} TCON_IND_GATT;

typedef PACKED(union) _TCON_IND_P
{
     TCON_IND_RFC rfc;
     TCON_IND_L2CAP l2cap;
     TCON_IND_GATT  gatt;
} TCON_IND_P;

/** CON_RESP */
typedef PACKED(struct) _TCON_RESP_RFC
{
    uint8_t              mscState;      /**< Initial MSC status to remote */
    uint8_t              encrypted;     /**< Connection should be encrypted */
    uint8_t              maxCredits;    /**< max no of credits in upstream direction */
    uint32_t             dummy;         /**< @@@ make sure that framesize field is at offset longer than obex variant */
    uint16_t              frameSize;     /**< frameSize to use on the link (field might be missing, 0 for default==con_ind value) */
} TCON_RESP_RFC;

typedef PACKED(struct) _TCON_RESP_L2CAP
{
    uint16_t               response;
    uint16_t               status;
    TCONF_L2CAP        conf;
    uint8_t               encrypted;           /**< Connection should be encrypted */
} TCON_RESP_L2CAP;

typedef struct _TCON_RESP_SCO
{
    uint8_t  encrypted;           /* Connection should be encrypted */
    uint32_t txBandwidth;
    uint32_t rxBandwidth;
    uint16_t  maxLatency;
    uint16_t  voiceSetting;
    uint8_t  retransEffort;
    uint16_t  packetType;
} TCON_RESP_SCO, *PCON_RESP_SCO;

typedef PACKED(struct) _TCON_RESP
{
	BLINKCONTEXT      bLinkContext;  /**< Context to use for this connection */
	BLINKHANDLE       bLinkHandle;   /**< Link handle for this connection */
	uint8_t              accept;        /**< 0=reject, 1=accept this connection, 2=accept (lo prio) */
	PACKED(union) 
	{
		TCON_RESP_RFC     rfc;
		TCON_RESP_L2CAP   l2cap;
        TCON_RESP_SCO     sco;
	} p;
} TCON_RESP;

/** CON_ACT_IND */
typedef PACKED(struct) _TCON_ACT_IND_RFC
{
    uint8_t              creditBased;               /**< initial msc state on this connection */
    uint16_t              unconfirmedRequests;
    uint16_t              unrespondedIndications;
} TCON_ACT_IND_RFC;

typedef PACKED(struct) _TCON_ACT_IND_L2CAP
{
    uint16_t              lcid;
    uint16_t              result;
    uint16_t              status;
    uint16_t              unconfirmedRequests;
    TCONF_L2CAP       conf;
    uint16_t              mtuSize;        /**< incoming MTU size */
    uint16_t              txWindowSize;   /**< max Rx window size */
    uint16_t              maxPDUSize;     /**< incoming PDU size */
} TCON_ACT_IND_L2CAP;

typedef PACKED(struct) _TCON_ACT_IND_GATT 
{
    TBdAddr           bd;
    uint8_t              bdType;
    uint16_t              credits;  /**< applicable for WriteCommand and Notification only */
    uint16_t              connInterval;
    uint16_t              connLatency;
    uint16_t              supervisionTimeout;
} TCON_ACT_IND_GATT;

/**  data traffic  */
typedef PACKED(struct) _TDATA_REQ
{
    BLINKHANDLE     bLinkHandle;
    uint16_t            dsPoolId;       /**< id of ds pool to use... */
    uint16_t            gap;            /**< alignment gap */
    uint8_t            buf[1];         /**< variable size */
}TDATA_REQ;

/**  sdp messages */
typedef PACKED(struct) _TSDP_SEARCH_CONF
{
    BLINKCONTEXT      bLinkContext;
    uint16_t              totalHandles;   /**< total no of handles returned */
    uint8_t              gap;            /**< size of gap (alignment) */
    uint32_t             handles[1];     /**< variable size */
} TSDP_SEARCH_CONF;

typedef PACKED(struct) _TSDP_ATTRIBUTE_CONF
{
    BLINKCONTEXT      bLinkContext;
    uint16_t              totalLen;       /**< total size of avList returned */
    uint8_t              gap;            /**< size of gap (alignment) */
    uint8_t              avList[1];      /**< variable size attribute value pairs */
} TSDP_ATTRIBUTE_CONF;

typedef PACKED(struct) _TSDP_REGISTER_REQ
{
    uint16_t              totalLen;       /**< length of entry */
    BREFERENCE        bReference;       /**< transaction reference */
    uint8_t              serviceRecord[1]; /**< service record  */
} TSDP_REGISTER_REQ;



/**  GATT defines and messages  */
#define  UUID_TYPE_16    16     /**< 16 bit UUID  */
#define  UUID_TYPE_128  128     /**< 128 bit UUID */

/** GATT server specific messages */
typedef PACKED(struct) _TGATT_ATTRIB_UPDATE_LIST_ELEMENT
{
	TBdAddr bd;
	uint8_t    bdType;
} TGATT_ATTRIB_UPDATE_LIST_ELEMENT, * PGATT_ATTRIB_UPDATE_LIST_ELEMENT;

#if (F_BT_LOW_ENERGY)  /* [ */
/** opcodes for peripheral advertise / scan response control */
#define  GATT_LE_ADVERTISE_OPCODE_AD_PARAMETER  1
#define  GATT_LE_ADVERTISE_OPCODE_AD_DATA       2
#define  GATT_LE_ADVERTISE_OPCODE_SC_RSP        3
#define  GATT_LE_ADVERTISE_OPCODE_AD_ENABLE     4
#define  GATT_LE_ADVERTISE_OPCODE_AD_DISABLE    5
#define  GATT_LE_ADVERTISE_OPCODE_AD_DIRECTED   6

typedef PACKED(struct) _TGATT_LE_ADVERTISE_DATA
{
	uint8_t      length;
	uint8_t      data[31];         /**< advertising / scan response data template */
} TGATT_LE_ADVERTISE_DATA;

typedef PACKED(struct) _TGATT_LE_ADVERTISE_PARAMETER
{
	uint8_t      advType;
	uint8_t      advChannelMap;
	uint8_t      filterPolicy;
	uint16_t      minAdvInterval;
	uint16_t      maxAdvInterval;
    TBdAddr   bd;
    uint8_t      bdType;
	uint8_t      local_bdType;
} TGATT_LE_ADVERTISE_PARAMETER;

typedef PACKED(struct) _TGATT_LE_ADVERTISE_DIRECTED
{
	TBdAddr   bd;
	uint8_t      bdType;
    uint8_t      local_bdType;
} TGATT_LE_ADVERTISE_DIRECTED;

#endif /**< ] (F_BT_LOW_ENERGY) */

#if (F_BT_LOW_ENERGY) /**< [ */
/**LE specific messages: ---*/
/** from blueface.h: */
typedef TGATT_LE_ADVERTISE_DATA       TGATTLEAdvertiseData,       * PGATTLEAdvertiseData;
typedef TGATT_LE_ADVERTISE_PARAMETER  TGATTLEAdvertiseParameter,  * PGATTLEAdvertiseParameter;
typedef TGATT_LE_ADVERTISE_DIRECTED   TGATTLEAdvertiseDirected,   * PGATTLEAdvertiseDirected;

#endif /**< ] (F_BT_LOW_ENERGY) */

/** discovery types */
#define  GATT_TYPE_UNDEFINED            0  /**< for internal use only ..    */
#define  GATT_TYPE_DISCOVERY_PSRV_ALL   1  /**< all primary services        */
#define  GATT_TYPE_DISCOVERY_PSRV_UUID  2  /**< specific primary services by UUID */
#define  GATT_TYPE_DISCOVERY_CHAR_ALL   3  /**< all characteristics of a service  */
#define  GATT_TYPE_DISCOVERY_CHAR_DESCR 4  /**< characteristic descriptors, internally */
                                           /**< mapped to GATT_TYPE_DISCOVERY_GET_UUID */
#define  GATT_TYPE_DISCOVERY_RELATION   5  /**< relationship                */
#if 0
#define  GATT_TYPE_DISCOVERY_CHAR_UUID  6  /**< all characteristics by UUID: not needed, */
                                           /**< intended functionality is provided thru  */
                                           /**< read with type=GATT_READ_TYPE_TYPE,    */
                                           /**< see Errata 3817 ...                    */
#endif

/** for internal use only */
#define  GATT_TYPE_DISCOVERY_GET_UUID   0x4000  /**< UUIDs of handle range         */
#define  GATT_TYPE_DISCOVERY_CMP_UUID   0x4001  /**< compare UUIDs in handle range */
#define  GATT_TYPE_CURR_PROC_READ       0x4002  /**< local client read operation   */
#define  GATT_TYPE_CURR_PROC_WRITE      0x4003  /**< local client write operation  */
#define  GATT_TYPE_CURR_PROC_PREPARE_WRITE  0x4004  /**< local client prepare write op. */
#define  GATT_TYPE_CURR_PROC_EXECUTE_WRITE  0x4005  /**< local client execute write op. */
#define  GATT_TYPE_CURR_TRANS_READ  /**< remote client read transaction  */ \
                                        GATT_TYPE_CURR_PROC_READ
#define  GATT_TYPE_CURR_TRANS_WRITE /**< remote client write transaction */ \
                                        GATT_TYPE_CURR_PROC_WRITE
#define  GATT_TYPE_CURR_TRANS_PREPARE_WRITE /**< remote client prepare write op. */ \
		                                    GATT_TYPE_CURR_PROC_PREPARE_WRITE
#define  GATT_TYPE_CURR_TRANS_EXECUTE_WRITE /**< remote client execute write op. */ \
                                        GATT_TYPE_CURR_PROC_EXECUTE_WRITE



/** TGATT_DISCOVERY_IND list elements, some discovery type results */
/** share the same generic list element format:                    */
typedef PACKED(struct) _TGATT_GENERIC_ELEMENT_16
{
	uint16_t  attHandle;
	uint16_t  endGroupHandle;
	uint16_t  uuid16;                /**< 16 bit UUID */
} TGATT_GENERIC_ELEMENT_16, * PGATT_GENERIC_ELEMENT_16;

typedef PACKED(struct) _TGATT_GENERIC_ELEMENT_128
{
	uint16_t  attHandle;
	uint16_t  endGroupHandle;
	uint8_t  uuid128[16];           /**< 128 bit UUID */
} TGATT_GENERIC_ELEMENT_128, * PGATT_GENERIC_ELEMENT_128;

/** type = GATT_TYPE_DISCOVERY_PSRV_ALL */
typedef TGATT_GENERIC_ELEMENT_16 TGATT_PSRV_ALL_ELEMENT_16;
typedef TGATT_PSRV_ALL_ELEMENT_16 * PGATT_PSRV_ALL_ELEMENT_16;

typedef TGATT_GENERIC_ELEMENT_128 TGATT_PSRV_ALL_ELEMENT_128;
typedef TGATT_PSRV_ALL_ELEMENT_128 * PGATT_PSRV_ALL_ELEMENT_128;


/** type = GATT_TYPE_DISCOVERY_PSRV_UUID */
typedef PACKED(struct) _TGATT_PSRV_UUID_ELEMENT
{
	uint16_t  attHandle;
	uint16_t  endGroupHandle;
} TGATT_PSRV_UUID_ELEMENT, * PGATT_PSRV_UUID_ELEMENT;


/** type = GATT_TYPE_DISCOVERY_RELATION */
typedef PACKED(struct) _TGATT_RELATION_ELEMENT_16
{
	uint16_t  declHandle;
	uint16_t  attHandle;
	uint16_t  endGroupHandle;
	uint16_t  uuid16;                /**< 16 bit UUID */
} TGATT_RELATION_ELEMENT_16, * PGATT_RELATION_ELEMENT_16;

typedef PACKED(struct) _TGATT_RELATION_ELEMENT_128
{
  uint16_t  declHandle;
  uint16_t  attHandle;
  uint16_t  endGroupHandle;
  uint8_t  uuid128[16];           /**< 128 bit UUID */
} TGATT_RELATION_ELEMENT_128, * PGATT_RELATION_ELEMENT_128;


/** type = GATT_TYPE_DISCOVERY_CHAR_ALL */
typedef PACKED(struct) _TGATT_CHAR_ALL_ELEMENT_16
{
	uint16_t  declHandle;
	uint16_t  properties;            /**< not uint8_t: BlueAPI struct is NOT PACKED(struct) .. */
	uint16_t  valueHandle;
	uint16_t  uuid16;                /**< 16 bit UUID */
} TGATT_CHAR_ALL_ELEMENT_16, * PGATT_CHAR_ALL_ELEMENT_16;

typedef PACKED(struct) _TGATT_CHAR_ALL_ELEMENT_128
{
	uint16_t  declHandle;
	uint16_t  properties;            /**< not uint8_t: BlueAPI struct is NOT PACKED(struct) .. */
	uint16_t  valueHandle;
	uint8_t  uuid128[16];           /**< 128 bit UUID */
} TGATT_CHAR_ALL_ELEMENT_128, * PGATT_CHAR_ALL_ELEMENT_128;


/** type = GATT_TYPE_DISCOVERY_CHAR_DESCR */
typedef PACKED(struct) _TGATT_CHAR_DESCR_ELEMENT_16
{
	uint16_t  handle;
	uint16_t  uuid16;                /**< 16 bit UUID */
} TGATT_CHAR_DESCR_ELEMENT_16, * PGATT_CHAR_DESCR_ELEMENT_16;

typedef PACKED(struct) _TGATT_CHAR_DESCR_ELEMENT_128
{
	uint16_t  handle;
	uint8_t  uuid128[16];           /**< 128 bit UUID */
} TGATT_CHAR_DESCR_ELEMENT_128, * PGATT_CHAR_DESCR_ELEMENT_128;



/** read attribute value(s) */
/** read types */
#define  GATT_READ_TYPE_BASIC      1      /**< ATT "Read Request"          */
#define  GATT_READ_TYPE_BLOB       2      /**< ATT "Read Blob Request"     */
#define  GATT_READ_TYPE_MULTIPLE   3      /**< ATT "Read Multiple Request" */
#define  GATT_READ_TYPE_TYPE       4      /**< ATT "Read By Type Request"  */

/** write attribute value(s) */
/** write types (see GATT_CURR_PROC_xxx definitions too ..) */
#define  GATT_WRITE_TYPE_REQ          1      /**< ATT "Write Request"          */
#define  GATT_WRITE_TYPE_CMD          2      /**< ATT "Write Command"          */
#define  GATT_WRITE_TYPE_PREP         3      /**< ATT "Prepare Write Request"  */
#define  GATT_WRITE_TYPE_EXEC         4      /**< ATT "Execute Write Request"  */
/** not yet implemented: */
#define  GATT_WRITE_TYPE_CMD_SIGNED   5      /**< ATT "Signed Write Command"   */

#if (F_BT_LOW_ENERGY) /**< [ */
typedef PACKED(struct) _TGATT_LE_CONNECTION_UPDATE_PARAM
{
	uint16_t    connIntervalMin;
	uint16_t    connIntervalMax;
	uint16_t    connLatency;
	uint16_t    supervisionTimeout;
	uint16_t    minimumCELength;
	uint16_t    maximumCELength;
} TGATT_LE_CONNECTION_UPDATE_PARAM;

#define GATT_LE_CONNECTION_UPDATE_TYPE_REQUEST  1
#define GATT_LE_CONNECTION_UPDATE_TYPE_EVENT    2

#define GATT_LE_MODIFY_WHITELIST_OPCODE_CLEAR  0
#define GATT_LE_MODIFY_WHITELIST_OPCODE_ADD    1
#define GATT_LE_MODIFY_WHITELIST_OPCODE_REMOVE 2
#endif /**< ] (F_BT_LOW_ENERGY) */


/** constant definition for requestType */
#define BLUEFACE_LINKKEY           1
#define BLUEFACE_LOCALPIN          2
#define BLUEFACE_REMOTEPIN         3
#define BLUEFACE_LINKKEY_MITM      4

/** constant definition for keyType in THCI_NEWKEY_IND */
#define BLUEFACE_KEYTYPE_COMBINATION      0x00  /**< bt2.0 key */
#define BLUEFACE_KEYTYPE_LOCAL_UNIT       0x01
#define BLUEFACE_KEYTYPE_REMOTE_UNIT      0x02
#define BLUEFACE_KEYTYPE_UNAUTHENTICATED  0x04  /**< bt2.1 ssp key */
#define BLUEFACE_KEYTYPE_AUTHENTICATED    0x05  /**< bt2.1 ssp mitm key */
#define BLUEFACE_KEYTYPE_CHANGED          0x06  /**< bt2.1 new key but same keytype */
#define BLUEFACE_KEYTYPE_DELETE           0xFF  /**< delete link key */

typedef PACKED(struct) _THCI_SNIFF_REQ
{
    BLINKHANDLE       bLinkHandle;
    uint16_t              maxSniff;             /**< use zero to exit sniff mode ! */
    uint16_t              minSniff;
    uint16_t              attempt;
    uint16_t              timeOut;
    TBdAddr           bd;
} THCI_SNIFF_REQ;

/**  global stack configuration and           */
/**  status exchange                        */
typedef PACKED(struct) _THCI_MODECHANGE_IND
{
    BLINKCONTEXT      bLinkContext;
    TBdAddr           bd;
    uint16_t              status;               /**< full blueface status code */
    uint8_t              mode;                 /**< current link mode */
    uint16_t              interval;             /**< link mode additional parameter */
} THCI_MODECHANGE_IND;
typedef THCI_MODECHANGE_IND  * LPHCI_MODECHANGE_IND;

#define BLUEFACE_EXTENDED_INQUIRY_RESPONSE_SIZE 240
typedef PACKED(struct) _THCI_CONF_INQUIRY_RESULT
{
    TBdAddr      bd;
    TDevClass    classOfDevice;
    char         rssi;
    uint8_t         extendedResponse[1]; /**< alsways send length byte */
                                      /**< if EIR is available send up to 240 bytes */
} THCI_CONF_INQUIRY_RESULT,  * LPTHCI_CONF_INQUIRY_RESULT;

#define CLEAR_ALL_CONNECTIONS      0 /**< no active connections       */
#define NEW_ACL_CONNECTION         1 /**< new ACL connection          */
#define NEW_SCO_CONNECTION         2 /**< new SCO connection          */
#define CLEAR_ACL_CONNECTION       3 /**< ACL connection disconnected */
#define CLEAR_SCO_CONNECTION       4 /**< SCO connection disconnected */
#define ROLE_CHANGE_TO_MASTER      5 /**< indicates role of host changed to master for reported bd */
#define ROLE_CHANGE_TO_SLAVE       6 /**< indicates role of host changed to slave for reported bd  */
#define ROLE_CHANGE_TO_MASTER_FAIL 7 /**< error on role changing to master for reported bd      */
#define ROLE_CHANGE_TO_SLAVE_FAIL  8 /**< error on role changing to slave for reported bd       */
#define NEW_LE_CONNECTION         10 /**< new BLE connection */
#define CLEAR_LE_CONNECTION       11 /**< BLE connection disconnected */
#define ACL_CONNECTION_NONSSP     12 /**< indicate that peer has no SSP support */
#define ACL_CONNECTION_SSP        13 /**< indicate that peer has SSP support */

typedef struct _THCI_CONF_NO_CONNECTIONS 
{
    uint16_t         noAclConnections;
    uint16_t         noScoConnections;
    uint16_t         noLeConnections;
    uint8_t         indId;
    uint8_t         bdType;
    TBdAddr      bd;                  /**< bluetooth device address of the remote device */
} THCI_CONF_NO_CONNECTIONS,  * LPTHCI_CONF_NO_CONNECTIONS;

typedef PACKED(struct) _THCI_CONF_EXTENDED_INQUIRY_RESPONSE 
{
	uint8_t  fec;   /**< 0 - fec not required, 1 fec required, 2-0xFF reserved */
	uint8_t  extendedResult[1]; /**< All octets zero -> Default */
} THCI_CONF_EXTENDED_INQUIRY_RESPONSE,  * LPHCI_CONF_EXTENDED_INQUIRY_RESPONSE;

typedef PACKED(struct) _THCI_CONF_OOB_DATA 
{
	uint8_t    Status;
	uint8_t    C[16];
	uint8_t    R[16];
	TBdAddr Bd;    /**< remote device address */
} THCI_CONF_OOB_DATA,  * LPHCI_CONF_OOB_DATA;


/** application level authorization of link setup */
#define CONF_SECURITY_STATUS_AUTH_STARTED 0
#define CONF_SECURITY_STATUS_AUTH_SUCCESS 1
#define CONF_SECURITY_STATUS_AUTH_FAILED  2
#define CONF_SECURITY_STATUS_ENCRYPTED     3
#define CONF_SECURITY_STATUS_NOT_ENCRYPTED 4

/** Authentication mode settings */
#define AUTHEN_SETTING_NONE                0       /**< no security requirements for the adressed link */
#define AUTHEN_SETTING_AUTHENTICATE        1       /**< link establishment requires link key, new bonding not allowed */

/** define service / link related security requirements */
typedef struct _TSEC_CONF_SECURITY 
{
	uint16_t psm;               /**< L2CAP PSM. =0: not used */
	uint16_t serverChannel;     /**< RFCOMM channel. =0: not used */
	uint8_t outgoing;          /**< define requirement for outgoing connection (true) or for incoming connection (false) */
	uint8_t active;            /**< TRUE: entry is active. FALSE: delete corresponding entry */
	uint16_t uuid;              /**< UUID used in corresponding AUTHORIZATION_IND */
	uint8_t authenSetting;     /**< one of AUTHEN_SETTING_XXX coding*/
	                        /**< if set and authentication not already done, then on*/
	                        /**< - outgoing:       start authentication before sending conReq to remote*/
	                        /**< - incoming bt2.0: start authentication before sending conResp to remote*/
	                        /**< - incoming bt2.1: reject connection (bt2.1 specification requirement)*/
	uint8_t authorize;         /**< link establishment requires explicit user level authorization (done before authentication) */
	uint8_t encryption;        /**< TRUE: activate encryption on this link, only possible if authen_setting != none*/
	                        /**< if set and encryption not already present, then on*/
	                        /**< - outgoing:       enable encryption before sending conReq to remote*/
	                        /**< - incoming bt2.0: enable encryption before sending conResp to remote*/
	                        /**< - incoming bt2.1: reject connection (bt2.1 specification requirement) */
	uint8_t mitm;              /**< TRUE: mitm required, only possible if authen_setting != none, FALSE: no MITM required */
} TSEC_CONF_SECURITY,  * LPTSEC_CONF_SECURITY;

//#if F_BT_LE_BT41_SUPPORT
typedef PACKED(struct) _TSEC_CONF_LE_SECURITY
{
    uint8_t psm;               /* L2CAP PSM. =0: not used */
    uint8_t active;            /* TRUE: entry is active. FALSE: delete corresponding entry */
    uint8_t secMode;          /* TRUE: mitm required, only possible if authen_setting != none, FALSE: no MITM required */
    uint8_t keySize;
} TSEC_CONF_LE_SECURITY,  * LPTSEC_CONF_LE_SECURITY;
//#endif
/** how to define security conditions:*/
/**   outgoing connection, verify on RFCOMM level:*/
/**       outgoing=1,serverchannel==any value<>0, psm=0, uuid=matching to con_req*/
/**   outgoing connection, verify on L2CAP level:*/
/**       outgoing=1,serverchannel==0, psm==any value <>0, uuid=matching to con_req*/
/**   incoming connection, verify on RFCOMM level*/
/**       outgoing=0,serverchannel==serverchannel to be controlled, psm==0, uuid=d.c. (will be used in authorization only)*/
/**   incoming connection, verify on L2CAP level*/
/**      outgoing=0,serverchannel==0, psm==value to be controlled, uuid=d.c. (will be used in authorization only) */

#define BTMODE_SETTING_NO_BT21                                  0x00 /**< No BT2.1 operation */
#define BTMODE_SETTING_BT21                                     0x01 /**< BT2.1 operation, use authsettings */
#define BTMODE_SETTING_BT21_DEBUG                               0x02 /**< BT2.1 debug (traceble) operation, use authsettings */


/** mode settings as defined for SSP */
#define DEVAUTH_SETTING_NO_MITM_NO_BOND_NUMERIC_COMPARE         0x00
#define DEVAUTH_SETTING_MITM_NO_BOND_USE_IO_CAPABILITIES        0x01
#define DEVAUTH_SETTING_NO_MITM_DEDICATED_BOND_NUMERIC_COMPARE  0x02
#define DEVAUTH_SETTING_MITM_DEDICATED_BOND_USE_IO_CAPABILITIES 0x03
#define DEVAUTH_SETTING_NO_MITM_GENERAL_BOND_NUMERIC_COMPARE    0x04
#define DEVAUTH_SETTING_MITM_GENERAL_BOND_USE_IO_CAPABILITIES   0x05

#define DEVAUTH_SETTING_MITM_MASK                               0x01
#define DEVAUTH_SETTING_BOND_MASK                               0x06

#define DEVAUTH_SETTING_NO_MITM                                 0x00
#define DEVAUTH_SETTING_MITM                                    0x01

#define DEVAUTH_SETTING_NO_BOND                                 0x00
#define DEVAUTH_SETTING_DEDICATED_BOND                          0x02
#define DEVAUTH_SETTING_GENERAL_BOND                            0x04

#define BLUEFACE_IOCAPA_DISPLAYONLY         0
#define BLUEFACE_IOCAPA_DISPLAYYESNO        1
#define BLUEFACE_IOCAPA_KEYBOARDONLY        2
#define BLUEFACE_IOCAPA_NOIO                3
#define BLUEFACE_IOCAPA_KEYBOARDDISPLAY     4  /**< BLE only, for BR mapped to DISPLAYONLY/KEYBOARDONLY */

/** define device related security settings */
typedef PACKED(struct) _TSEC_CONF_DEVICE_SECURITY
{
	uint8_t pairable_mode;         /**< 0: non pairable, 1 : pairable */
	uint8_t bt_mode;               /**< one of BTMODE_SETTING_XXX values */
	uint8_t authenSettings;        /**< one of DEVAUTH_SETTING_XXX coding */
	uint8_t io_capabilities;       /**< one of BLUEFACE_IOCAPA_XXX values */
	uint8_t oob_present;           /**< 0: no oop available, 1: oop available */
} TSEC_CONF_DEVICE_SECURITY,  * LPTEC_CONF_DEVICE_SECURITY;

typedef PACKED(struct) _TSEC_CONF_AUTHORIZE
{
   TBdAddr bd;             /* Bluetooth Address of device to authorize */
   uint8_t active;            /* in _IND: true: active authorize request, false: request terminated (due to link terminarion etc.=) */
                           /* in _REQ: true: authorize connection, false: reject connection attempt */
} TSEC_CONF_AUTHORIZE,  * LPTSEC_CONF_AUTHORIZE;

typedef PACKED(struct) _TSEC_CONF_LE_SSP_PARAMETER
{
	uint32_t   fixedDisplayValue;  /**< use 0xFFFFFFFF for "normal" display behavior */
} TSEC_CONF_LE_SSP_PARAMETER,  * LPSEC_CONF_LE_SSP_PARAMETER;

/** STACK CONFIGURATION REQ/IND ID Settings*/
/**    values 1 - 3 and 5 are reserved for compability reasons */
#define BLUEFACE_CONF_NO_CONNECTIONS                 4   /**< to Appli:      No of ACL connections      */

#define BLUEFACE_CONF_SECURITY                      32    /**< REQ/CONF:      change service/link security settings */
#define BLUEFACE_CONF_AUTHORIZE                     33 
#define BLUEFACE_CONF_DEVICE_SECURITY               34    /**< REQ/CONF:      define device related security requirements */

#define BLUEFACE_CONF_LE_SSP_PARAMETER              42    /**< from Appli:    LE: set a fixed display value for SSP auth */
#define BLUEFACE_CONF_LE_SECURITY                   43   

/**  BTSEC / device data   */
/** data types */
#define DEVICE_DATA_TYPE_MASK               0xF0
#define DEVICE_DATA_TYPE_GATT               0x00
#define DEVICE_DATA_TYPE_SECMAN             0x10

#define DEVICE_DATA_TYPE_GATT_CCC_BITS       0x01   /**< GATT client charac. config. bits */
#define DEVICE_DATA_TYPE_GATT_CCC_REV_BITS   0x02   /**< GATT client BDs */
#define DEVICE_DATA_TYPE_GATT_SRV_CHG_HANDLE 0x03   /**< GATT service changed charac. */

#define DEVICE_DATA_TYPE_SECMAN_LTK_LOCAL   0x11   /**< used by peer to connect */
#define DEVICE_DATA_TYPE_SECMAN_LTK_REMOTE  0x12   /**< used by us to connect   */
#define DEVICE_DATA_TYPE_SECMAN_IRK_LOCAL   0x13   /**< used by peer to resolve our bd (unique) */
#define DEVICE_DATA_TYPE_SECMAN_IRK_REMOTE  0x14   /**< used by us to resolve peer bd                  */
#define DEVICE_DATA_TYPE_SECMAN_CSRK_LOCAL  0x15   /**< used by us to sign, by peer to check */
#define DEVICE_DATA_TYPE_SECMAN_CSRK_REMOTE 0x16   /**< used by peer to sign, us to check    */

#define DEVICE_DATA_TYPE_BREDR_LINKKEY      0x21   /**< BR/EDR linkkey */

/** data type specific list elements */
/** handle/CCC bits */
typedef PACKED(struct) _DEVICE_DATA_ELEMENT_GATT_CCC_BITS
{
	uint16_t  attHandle;
	uint16_t  cccBits;
} TDEVICE_DATA_ELEMENT_GATT_CCC_BITS, * PDEVICE_DATA_ELEMENT_GATT_CCC_BITS;

/** reverse search for BDs for given handle/CCC bits */
typedef PACKED(struct) _DEVICE_DATA_ELEMENT_GATT_CCC_BITS_REV
{
	TBdAddr bd;
	uint8_t    bdType;
} TDEVICE_DATA_ELEMENT_GATT_CCC_BITS_REV, * PDEVICE_DATA_ELEMENT_GATT_CCC_BITS_REV;

/** handle of GATT/ServiceChanged characteristic / indication flag */
typedef PACKED(struct) _DEVICE_DATA_ELEMENT_GATT_SRV_CHG_HANDLE
{
	uint16_t  attHandle;
	uint8_t  indicate;
} TDEVICE_DATA_ELEMENT_GATT_SRV_CHG_HANDLE, * PDEVICE_DATA_ELEMENT_GATT_SRV_CHG_HANDLE;

typedef PACKED(struct) _DEVICE_DATA_ELEMENT_BREDR_LINKKEY
{
	uint8_t  key[16];
	uint8_t  keyType;
} TDEVICE_DATA_ELEMENT_BREDR_LINKKEY, * PDEVICE_DATA_ELEMENT_BREDR_LINKKEY;

typedef PACKED(struct) _DEVICE_DATA_ELEMENT_SECMAN_LTK
{
	uint8_t  key[16];
	uint8_t  rand[8];
	uint16_t  ediv;
	uint8_t  keySize;
	uint8_t  mitmFlag;
} TDEVICE_DATA_ELEMENT_SECMAN_LTK, * PDEVICE_DATA_ELEMENT_SECMAN_LTK;

typedef PACKED(struct) _DEVICE_DATA_ELEMENT_SECMAN_IRK
{
	uint8_t  key[16];
	uint8_t  bd[BD_ADDR_SIZE];
	uint8_t  mitmFlag;
} TDEVICE_DATA_ELEMENT_SECMAN_IRK, * PDEVICE_DATA_ELEMENT_SECMAN_IRK;

typedef PACKED(struct) _DEVICE_DATA_ELEMENT_SECMAN_CSRK
{
	uint8_t  key[16];
	uint32_t signCount;
	uint8_t  mitmFlag;
} TDEVICE_DATA_ELEMENT_SECMAN_CSRK, * PDEVICE_DATA_ELEMENT_SECMAN_CSRK;

#define DEVICE_DATA_ELEMENT_GATT_CCC_BITS_MAX       8
#define DEVICE_DATA_ELEMENT_GATT_CCC_BITS_REV_MAX   4

typedef PACKED(union) _DEVICE_DATA_ELEMENT
{
	TDEVICE_DATA_ELEMENT_GATT_CCC_BITS        cccBit[DEVICE_DATA_ELEMENT_GATT_CCC_BITS_MAX];
	TDEVICE_DATA_ELEMENT_GATT_CCC_BITS_REV    cccBitRev[DEVICE_DATA_ELEMENT_GATT_CCC_BITS_REV_MAX];
	TDEVICE_DATA_ELEMENT_GATT_SRV_CHG_HANDLE  srvChg;
	TDEVICE_DATA_ELEMENT_BREDR_LINKKEY        brKey;
	TDEVICE_DATA_ELEMENT_SECMAN_LTK           leLTK;
	TDEVICE_DATA_ELEMENT_SECMAN_IRK           leIRK;
	TDEVICE_DATA_ELEMENT_SECMAN_CSRK          leCSRK;
	uint8_t                                      data[32];
} TDEVICE_DATA_ELEMENT, * PDEVICE_DATA_ELEMENT;

typedef struct _DEVICE_DATA
{
    TBdAddr     bd;
    uint8_t     bdType;                   /**< BLUEFACE_BDTYPE_* */
    uint8_t     dataType;                 /**< DEVICE_DATA_TYPE_* */
    uint16_t    restartHandle;
    uint16_t    elementCount;
    TDEVICE_DATA_ELEMENT p;
} TDEVICE_DATA;
typedef TDEVICE_DATA * PDEVICE_DATA;

typedef struct _DEVICE_DATA_IND
{
    PVOID        handle;      /**< to be returned in RESP */
    TDEVICE_DATA  deviceData;
} TDEVICE_DATA_IND;
typedef TDEVICE_DATA_IND * PDEVICE_DATA_IND;

typedef PACKED(struct) _DEVICE_DATA_RESP
{
	PVOID        handle;
	uint16_t          status;
	TDEVICE_DATA  deviceData;
} TDEVICE_DATA_RESP;
typedef TDEVICE_DATA_RESP * PDEVICE_DATA_RESP;

/**  base structure  */
typedef struct _TblueFaceMsg
{
    TblueFaceEvent          command;
    uint16_t                length;      /**< total length including header */
    uint16_t                seqnbr;
    union
    {
        /** active connection setup */
        TCON_REQ            connectRequest;

        /** data traffic */
        TDATA_REQ           dataRequest;

        /** sdp client and server */
        TSDP_SEARCH_CONF    sdpSearchConfirmation;
        TSDP_ATTRIBUTE_CONF sdpAttributeConfirmation;
        TSDP_REGISTER_REQ   sdpRegisterRequest;

        /** hci tunneling */
        THCI_SNIFF_REQ      hciSniffRequest;

        /** BTSEC / device data */
        TDEVICE_DATA_IND    deviceDataIndication;
    } p /**< union */;
} TblueFaceMsg;

typedef TblueFaceMsg     *  PblueFaceMsg;
typedef TblueFaceMsg  * LPblueFaceMsg;
typedef CONST TblueFaceMsg  * LPCblueFaceMsg;

#define blueFaceMsgSize (offsetof(TblueFaceMsg,p.connectRequest))

typedef void (* PblueFaceCallBack)(
                                       BAPPCONTEXT pvContext,
                                       LPblueFaceMsg   pmsg
                                      );


typedef struct _TblueFaceReg
{
    BAPPCONTEXT         appContext;

#if (WIN32)
    char driver[64];

    /** @@@ these are some private and temporary extensions */
    char comPort[32];
    /** @@@ end of some private and temporary extensions */
#endif
} TblueFaceReg;
typedef TblueFaceReg  * LPblueFaceReg;

#ifdef __cplusplus
}
#endif /**< __cplusplus */


#if defined (BYTE_ALIGNMENT)
#pragma  pack()
#undef BYTE_ALIGNMENT
#endif

#if defined(PRAGMA_PACK)
#pragma pack()
#endif

#endif /**< !defined(__BLUEFACE_H) */
