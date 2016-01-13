#ifndef __AVRCP_H
#define __AVRCP_H

#include "rtl_types.h"
#include "mpa.h"


#define MAX_AVRCP_LINK              4
#define AVRCP_MAX_PACKET_LENGTH     (512 + 3)   //spec 512; IOS 515?
#define AVRCP_MAX_CONTINUE_PDU_LENGTH     1000   //tmp

/**private defs   */
#define WAIT_RESPONSE_TIMEOUT       1500

#define OPCODE_UNIT_INFO            0x30
#define OPCODE_SUB_UNIT_INFO        0x31
#define OPCODE_PASS_THROUGH         0x7C
#define OPCODE_VENDOR_DEPENDENT     0x00

#define PASSTHROUGH_STATE_PRESSED   0
#define PASSTHROUGH_STATE_RELEASED  1


#define AVRCP_OP_ID_VOLUP       0x41
#define AVRCP_OP_ID_VOLDOWN     0x42

#define PDU_ID_GET_CAPABILITIES         0x10    //AV/C STATUS
#define PDU_ID_GET_ELEMENT_ATTRIBUTES   0x20    //AV/C STATUS
#define PDU_ID_GET_PLAY_STATUS          0x30    //AV/C STATUS
#define PDU_ID_REGISTER_NOTIFICATION    0x31    //AV/C NOTIFY
#define PDU_ID_REQUEST_CONTINUE_RSP     0x40    //AV/C CONTROL
#define PDU_ID_ABORT_CONTINUE_RSP       0x41    //AV/C CONTROL
#define PDU_ID_SET_ABSOLUTEVOLUME       0x50    //AV/C CONTROL




#define AVRCP_PDU_NON_FRAGMENTED    0x0
#define AVRCP_PDU_START             0x1
#define AVRCP_PDU_CONTINUE          0x2
#define AVRCP_PDU_END               0x3
/** AVRCP ctype*/
#define AVRCP_CONTROL 	        0x00
#define AVRCP_STATUS	        0x01
#define	AVRCP_SPECIFIC_INQUIRY	0x02
#define	AVRCP_NOTIFY	        0x03
#define	AVRCP_GENERAL_INQUIRY	0x04

/**AVRCP Response*/
#define AVRCP_NOT_IMPLEMENTED	0x8
#define AVRCP_ACCEPTED		 	0x9
#define AVRCP_REJECTED			0xA
#define AVRCP_IN_TRANSITION		0xB
#define AVRCP_IMPLEMENTED		0xC
#define	AVRCP_CHANGED			0xD
#define	AVRCP_INTERIM			0xF


/** EventID   */
#define EVENT_PLAYBACK_STATUS_CHANGED               0x01 /* (CT) Change in playback status of the current track.*/
#define EVENT_TRACK_CHANGED                         0x02 /* (CT)Change of current track.*/
#define EVENT_TRACK_REACHED_END                     0x03 /* Reached end of a track.*/
#define EVENT_TRACK_REACHED_START                   0x04 /* Reached start of a track.*/
#define EVENT_PLAYBACK_POS_CHANGED                  0x05 /* (CT?)Change in playback position. Returned after the specified playback notification change notification interval.*/
#define EVENT_BATT_STATUS_CHANGED                   0x06 /* Change in battery status.*/
#define EVENT_SYSTEM_STATUS_CHANGED                 0x07 /* Change in system status.*/
#define EVENT_PLAYER_APPLICATION_SETTING_CHANGED    0x08 /* Change in player application setting.*/
#define EVENT_NOW_PLAYING_CONTENT_CHANGED           0x09 /* The content of the Now Playing list has changed, see Section 6.9.5.*/
#define EVENT_AVAILABLE_PLAYERS_CHANGED             0x0a /* The available players have changed, see Section 6.9.4.*/
#define EVENT_ADDRESSED_PLAYER_CHANGED              0x0b /* The Addressed Player has been changed, see Section 6.9.2.*/
#define EVENT_UIDS_CHANGED                          0x0c /* The UIDs have changed, see Section 6.10.3.3.*/
#define EVENT_VOLUME_CHANGED                        0x0d /* (TG) The volume has been changed locally on the TG, see Section 6.13.3.*/



typedef struct
{
    uint8_t *buf;
    uint16_t write_index;
}TAvrcpContinuation;


typedef struct //to .c file
{
    //acl status?
    TBdAddr             remote_bd;
    uint16_t            cid;
    uint8_t             cmd_credits;
    uint8_t             transact_label;
    TAvrcpContinuation  recombine;
    uint8_t             vol_change_pending_transact;
    void                *wait_response_timer;
} TAvrcpInst;

typedef struct
{
    TAvrcpInst              inst[MAX_AVRCP_LINK];
    bool                    role_amplifier_support;
    bool                    role_target_support;
    uint32_t                local_company_id;
    TAvrcpAppMessageHandler app_message_handler;
    TAvrcpVendorCmdHandler  vendor_cmd_handler;
    TAvrcpVendorRspHandler  vendor_rsp_handler;

    uint16_t                incalling_cid;
    TBdAddr                 outcalling_bd;
} TAvrcpProfile; //sizeof() = 140

/* called by avctp */
void avrcp_HandleConnectReq(PBlueAPI_L2cConInd ind);
void avrcp_HandleConnectCompleteInd(PBlueAPI_L2cConActInd ind);
void avrcp_HandleDataInd(/*TBdAddr bd, */uint16_t cid, uint8_t transact, uint8_t crType, uint8_t *pData, uint16_t length);
void avrcp_HandleDisconnectInd(uint16_t cid);
void avrcp_HandleSecurityRegisterRsp(PBlueAPI_L2cSecurityRegisterRsp rsp);
void avrcp_HandleAuthorizationInd(PBlueAPI_UserAuthorizationReqInd ind);


#endif
