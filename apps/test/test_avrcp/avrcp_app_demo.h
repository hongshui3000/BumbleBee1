

#define FEATURE_VOLUME_SYNC_SUPPORT     1

#define MAX_AVRCP_APP_INST      4

#define ELEMENT_ATTR_LENGTH		30

#define MY_COMPANY_ID  0x123456

#define AVRCP_MAX_QUEUED_CMD    10  //should be larger than local supported CT events number

#define LOCAL_EVENTS_SUPPORTED_MASK ((0x1 << EVENT_PLAYBACK_STATUS_CHANGED) | (0x1 << EVENT_TRACK_CHANGED))

/**After key(multi-purpose) pressed KEY_HOLD_DETERMINE_TIME, 
 * we determine it's a press-hold action   */
#define KEY_HOLD_DETERMINE_TIME     (1000 / portTICK_RATE_MS)

/* command with the pressed value is valid for TWO seconds from
the time when a target sends back a response of the command. The 
controller shall continue sending pressed value with identical 
operation id value in the operation_id field while the command 
is wished to stay valid */
#define KEY_HOLD_RESEND_TIME        (1500 / portTICK_RATE_MS)

typedef struct
{
    char title[ELEMENT_ATTR_LENGTH];
    char artist[ELEMENT_ATTR_LENGTH];
    char album[ELEMENT_ATTR_LENGTH];
    char track[ELEMENT_ATTR_LENGTH];
    char total_track[ELEMENT_ATTR_LENGTH];
    char genre[ELEMENT_ATTR_LENGTH];
    char playing_time[ELEMENT_ATTR_LENGTH];
}TElementAttr;
typedef struct
{
    void            *func; //TCmdActionFunc
    uint8_t         para;
}TAvrcpCmdAction;
typedef struct
{
    TAvrcpCmdAction buf[AVRCP_MAX_QUEUED_CMD + 1]; //queue depth,can hold n-1 items.
    short   write_index;
    short   read_index;
}TMiniQueue;


#include "avrcp_api.h"
typedef enum
{
    avrcpStateIdle = 0,
    avrcpStateDisconnected,
//  avrcpStateDisconnecting,
//  avrcpStateIncomingConnecting,
    avrcpStateOutgoingConnecting,
    avrcpStateConnected //can send unit info cmd(to get subunit type & subunit id)
//  avrcpStateRemoteUnitInfoGot, //can send cmd to subunit(passthrough, ...)
//  avrcpStateRemoteCapabilityGot //can register EVENT_XX_CHANGED
}eAvrcpState;

#define RELEASED    1
#define PRESSED     0

typedef struct
{
    eAvrcpState         state;
    TBdAddr             remote_bd;
    uint8_t             cmd_credits;
    uint8_t             remote_sub_unit_id;
    uint32_t            remote_company_id;
    uint16_t            remote_events_supported; //0x1 << EVENT_ID

#if FEATURE_VOLUME_SYNC_SUPPORT
    bool                vol_change_registered;
#endif

    TMiniQueue          pending_cmd_action_queue;

}TAvrcpLink;

typedef bool (*TCmdActionFunc)(TAvrcpLink *p_link, uint8_t para);


bool avrcp_app_init(void);
