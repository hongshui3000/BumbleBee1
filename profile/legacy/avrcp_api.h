
#ifndef __AVRCP_API_H
#define __AVRCP_API_H

#include "rtl_types.h"
#include "blueapi_types.h"

/**????*/

typedef uint8_t TBdAddr[BLUE_API_BD_SIZE];


#define AVRCP_PANEL 0x09

/** AVRCP cmd ctype (for vendor_cmd_handler)  */
#define AVRCP_CONTROL 	        0x00
#define AVRCP_STATUS	        0x01
#define	AVRCP_SPECIFIC_INQUIRY	0x02
#define	AVRCP_NOTIFY	        0x03
#define	AVRCP_GENERAL_INQUIRY	0x04
/**AVRCP Response (for vendor_cmd_handler)   */
#define AVRCP_NOT_IMPLEMENTED	0x8
#define AVRCP_ACCEPTED		 	0x9
#define AVRCP_REJECTED			0xA
#define AVRCP_IN_TRANSITION		0xB
#define AVRCP_IMPLEMENTED		0xC
#define	AVRCP_CHANGED			0xD
#define	AVRCP_INTERIM			0xF


#define CAPABILITY_ID_COMPANY_ID        0x2
#define CAPABILITY_ID_EVENTS_SUPPORTED  0x3

#define MAX_ELEMENT_ATTR_NUM                7
#define ELEMENT_ATTRIBUTE_TITLE             1
#define ELEMENT_ATTRIBUTE_ARTIST            2
#define ELEMENT_ATTRIBUTE_ALBUM             3
#define ELEMENT_ATTRIBUTE_TRACK             4
#define ELEMENT_ATTRIBUTE_TOTAL_TRACK       5
#define ELEMENT_ATTRIBUTE_GENRE             6
#define ELEMENT_ATTRIBUTE_PLAYING_TIME      7

#define AVRCP_PLAYSTATUS_STOPPED        0x0
#define AVRCP_PLAYSTATUS_PLAYING        0x1
#define AVRCP_PLAYSTATUS_PAUSED         0x2
#define AVRCP_PLAYSTATUS_FWD_SEEK       0x3
#define AVRCP_PLAYSTATUS_REV_SEEK       0x4
#define AVRCP_PLAYSTATUS_ERR            0xff

#define COMPANY_INVALID         0xffffff
#define COMPANY_BT_SIG          0x001958

/** EventID   */
#define EVENT_PLAYBACK_STATUS_CHANGED               0x01 /* (CT) Change in playback status of the current track.*/
#define EVENT_TRACK_CHANGED                         0x02 /* (CT)Change of current track.*/

typedef enum
{
    AVRCP_US_MESSAGE_AUTHORIZATION_IND,
    AVRCP_US_MESSAGE_CONNECT_REQ,
    AVRCP_US_MESSAGE_CONNECTED,
    AVRCP_US_MESSAGE_DISCONNECTED,

    /*volume sync feature as catogory2(amplifier) TG*/
    AVRCP_US_MESSAGE_CMD_VOL_UP,
    AVRCP_US_MESSAGE_CMD_VOL_DOWN,
    AVRCP_US_MESSAGE_CMD_ABS_VOL,
    AVRCP_US_MESSAGE_CMD_REGISTER_VOL_CHANGE,

    /*catogory1(player) CT*/
    AVRCP_US_MESSAGE_RSP_UNIT_INFO,
    AVRCP_US_MESSAGE_RSP_PASSTHROUGH,
    AVRCP_US_MESSAGE_RSP_GET_CAPABILITY,
    AVRCP_US_MESSAGE_RSP_GET_PLAYSTATUS,
    AVRCP_US_MESSAGE_RSP_GET_ELEMENT_ATTR,
    AVRCP_US_MESSAGE_RSP_REGISTER_NOTIFICATION,

    AVRCP_US_MESSAGE_RSP_DUMMY,

    AVRCP_US_MESSAGE_NOTIFICATION_CHANGED,
    AVRCP_US_MESSAGE_ERR
}AvrcpUsMessageType;

typedef enum
{
    AVRCP_KEY_PLAY = 0x44,
    AVRCP_KEY_STOP = 0x45,
    AVRCP_KEY_PAUSE = 0x46,
    AVRCP_KEY_FORWARD = 0x4B,
    AVRCP_KEY_BACKWARD = 0x4C,
    AVRCP_KEY_FAST_FORWARD = 0x49,
    AVRCP_KEY_REWIND = 0x48
}AvrcpKey;


typedef struct
{
    uint32_t        attribute_id;
    uint16_t        character_set_id;
    uint16_t        length;
    uint8_t         *buf;
}TElementAttribute;

typedef enum
{
    AVRCP_RSP_STATE_SUCCESS	            = 0,
    AVRCP_RSP_STATE_FAIL                = 1
//  AVRCP_REMOTE_REJECT?
}AvrcpRspState;
typedef enum
{
    AVRCP_US_ERR_WAIT_RESPONSE_TIMEOUT,
}AvrcpUsMessageErr;
typedef struct
{
    AvrcpRspState   state;
    uint8_t         sub_unit_type;
    uint8_t         sub_unit_id;
    uint32_t        company_id;
}TRspUnitInfo;
typedef struct
{
    AvrcpRspState   state;
    AvrcpKey        key;
    bool            pressed;
}TRspPassthrough;
typedef struct
{
    AvrcpRspState   state;
    uint8_t         capability_id;
    uint8_t         capability_count;
    uint8_t         *buf;
}TRspCapability;
typedef struct
{
    AvrcpRspState       state;
    uint32_t            length_ms;
    uint32_t            position_ms;
    uint8_t             play_status;
}TRspGetPlayStatus;
typedef struct
{
    AvrcpRspState       state;
    uint8_t             number_of_attributes;
    TElementAttribute   attr[MAX_ELEMENT_ATTR_NUM];
}TRspGetElementAttributes;
typedef struct
{
    AvrcpRspState       state;
    uint8_t             event_id;
    union
    {
        uint8_t play_status;
        unsigned long long *p_track_id; //0xFFFFFFFFFFFFFFFF means no track;
    }                   u;
}TRspRegisterNotification;
typedef struct
{
    uint8_t             event_id;
    union
    {
        uint8_t play_status;
        unsigned long long *p_track_id;
    }                   u;
}TNotificationChanged;
typedef enum
{
    AVRCP_SUCCESS,
    AVRCP_INVALID_PARA,
    AVRCP_INVALID_STATE,
    AVRCP_NO_RESOURCE,
    AVRCP_UNEXPECTED
}AvrcpReturnStatus;


typedef void (*TAvrcpAppMessageHandler)(TBdAddr bd, AvrcpUsMessageType message_type, void *message);
typedef void (*TAvrcpVendorCmdHandler)(TBdAddr bd, uint32_t company_id, uint8_t *p_pdu, uint16_t length, uint8_t ctype, uint8_t transact);
typedef void (*TAvrcpVendorRspHandler)(TBdAddr bd, uint32_t company_id, uint8_t *p_pdu, uint16_t length, uint8_t response);

AvrcpReturnStatus avrcp_Init(TAvrcpAppMessageHandler app_message_handler, bool role_amplifier_support, uint32_t company_id,
                             TAvrcpVendorCmdHandler vendor_cmd_handler, TAvrcpVendorRspHandler vendor_rsp_handler); //

AvrcpReturnStatus avrcp_ConnectConfirm(TBdAddr bd, bool accept);
AvrcpReturnStatus avrcp_Connect(TBdAddr bd);
AvrcpReturnStatus avrcp_DisConnect(TBdAddr bd);

AvrcpReturnStatus avrcp_CmdUnitInfo(TBdAddr bd);
AvrcpReturnStatus avrcp_CmdPassThrough(TBdAddr bd, AvrcpKey key, bool pressed);
AvrcpReturnStatus avrcp_CmdGetCapability(TBdAddr bd, uint8_t capability_id);
AvrcpReturnStatus avrcp_CmdGetPlayStatus(TBdAddr bd);
AvrcpReturnStatus avrcp_CmdGetElementAttributes(TBdAddr bd);
AvrcpReturnStatus avrcp_CmdRegisterNotification(TBdAddr bd, uint8_t event_id);

AvrcpReturnStatus avrcp_NotifyVolumeChange(TBdAddr bd, uint8_t volume);

AvrcpReturnStatus avrcp_SendVendorCommand(TBdAddr bd, uint8_t ctype, uint32_t company_id,
                                          uint8_t *p_pdu, uint16_t pdu_length);
AvrcpReturnStatus avrcp_SendVendorResponse(TBdAddr bd, uint8_t transact, uint8_t response, uint32_t company_id,
                                           uint8_t *p_pdu, uint16_t pdu_length);



#endif
