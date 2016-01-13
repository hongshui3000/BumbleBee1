
#include "avrcp_api.h"
#include "my_stdio.h"
#include <FreeRTOS.h>
#include <task.h>
#include <timers.h>


#include "avrcp_app_demo.h"
struct
{
    TAvrcpLink          link[MAX_AVRCP_APP_INST];
    TAvrcpLink          *p_active_link;
    uint8_t             volume;
    uint8_t             play_status;
    uint32_t            song_length;
    uint32_t            song_position;
    TElementAttr        attr;
    unsigned long long  track_id;

//  xTimerHandle        key_hold_determine_timer;
//  eAvrcpKey           key_hold_determine_key;

    xTimerHandle        key_hold_resend_timer;
    uint8_t             key_hold_resend_op_id;

}
App =
{
    .p_active_link = &App.link[0],
    .volume = 0x7f / 2
};



const char* PlayStatusToString(uint8_t play_status)
{
    return play_status == AVRCP_PLAYSTATUS_STOPPED ?   "Stopped"
           : play_status == AVRCP_PLAYSTATUS_PLAYING ?   "Playing"
           : play_status == AVRCP_PLAYSTATUS_PAUSED ?    "Paused"
           : play_status == AVRCP_PLAYSTATUS_FWD_SEEK ?  "Fwd_seek"
           : play_status == AVRCP_PLAYSTATUS_REV_SEEK ?  "Rev_seek"
           : play_status == AVRCP_PLAYSTATUS_ERR ?       "Err"
           : "Unkown";
}
void PrintUTF8(char *str)
{
    while (*str != '\0')
    {
        printf("%c", (*str >= 32 && *str <= 126) ? *str : '?'); // ' ' to '~'
        str++;
    }
}

#define MiniQueueEmpty(p_queue)  ((p_queue)->read_index == (p_queue)->write_index)
#define MiniQueueFull(p_queue)   (((p_queue)->write_index + 1) % (sizeof((p_queue)->buf)/sizeof((p_queue)->buf[0])) == (p_queue)->read_index)
static void MiniQueueReset(TMiniQueue *p_queue)
{
    p_queue->write_index = 0;
    p_queue->read_index = 0;
}
static int MiniQueueIn(TMiniQueue *p_queue, TAvrcpCmdAction *p_act)
{
    if (MiniQueueFull(p_queue))
    {
        printf("MiniQueueFull!\r\n");
        return 0;
    }
    p_queue->buf[p_queue->write_index] = *p_act;
    p_queue->write_index = (p_queue->write_index + 1) % (sizeof(p_queue->buf) / sizeof(p_queue->buf[0]));
    return 1;
}
static int MiniQueueOut(TMiniQueue *p_queue, TAvrcpCmdAction *p_act)
{
    if (MiniQueueEmpty(p_queue))
    {
        return 0;
    }
    *p_act = p_queue->buf[p_queue->read_index];
    p_queue->read_index = (p_queue->read_index + 1) % (sizeof(p_queue->buf) / sizeof(p_queue->buf[0]));
    return 1;
}
static void ClearPendingCmdAction(TAvrcpLink *p_link)
{
    MiniQueueReset(&(p_link->pending_cmd_action_queue));
}
static int FlushPendingCmdAction(TAvrcpLink *p_link)
{
    if (p_link->state >= avrcpStateConnected && p_link->cmd_credits == 1)
    { //if there is pending cmd action, process it
        TAvrcpCmdAction act;
        if (1 == MiniQueueOut(&(p_link->pending_cmd_action_queue), &act))
        {
            TCmdActionFunc func = (TCmdActionFunc)act.func;
            if (true == func(p_link, act.para))
            {
                p_link->cmd_credits = 0;
            }
            return 1;
        }
    }
    return 0;
}
static int PendCmdAction(TAvrcpLink *p_link, TCmdActionFunc func, uint8_t para)
{
    int ret = MiniQueueIn(&(p_link->pending_cmd_action_queue), &(TAvrcpCmdAction){ (void *)func, para });
    FlushPendingCmdAction(p_link);
    return ret;
}


void VolumeUp(void)
{
    if (App.volume == 0x7f)
    {
        return; //do not send EVENT_VOLUME_CHANGED
    }
    App.volume = App.volume <= 0x7f - 8 ? App.volume + 8 : 0x7f;
    if (App.p_active_link->vol_change_registered)
    {
        avrcp_NotifyVolumeChange(App.p_active_link->remote_bd, App.volume);
    }
}
void VolumeDown(void)
{
    if (App.volume == 0)
    {
        return; //do not send EVENT_VOLUME_CHANGED
    }
    App.volume = App.volume >= 8 ? App.volume - 8 : 0;
    if (App.p_active_link->vol_change_registered)
    {
        avrcp_NotifyVolumeChange(App.p_active_link->remote_bd, App.volume);
    }
}


bool CmdAction_UnitInfo(TAvrcpLink *p_link, uint8_t para)
{
    return avrcp_CmdUnitInfo(p_link->remote_bd) == AVRCP_SUCCESS? true : false;
}
bool CmdAction_Passthrough(TAvrcpLink *p_link, uint8_t para)
{
    return avrcp_CmdPassThrough(p_link->remote_bd, (AvrcpKey)(para & 0x7f), (para & 0x80) == 0 ? true : false) == AVRCP_SUCCESS?true : false;
}
bool CmdAction_GetCapability(TAvrcpLink *p_link, uint8_t para)
{
    return avrcp_CmdGetCapability(p_link->remote_bd, para) == AVRCP_SUCCESS?true : false;
}
bool CmdAction_GetPlayStatus(TAvrcpLink *p_link, uint8_t para)
{
    return avrcp_CmdGetPlayStatus(p_link->remote_bd) == AVRCP_SUCCESS?true : false;
}
bool CmdAction_GetElementAttributes(TAvrcpLink *p_link, uint8_t para)
{
    return avrcp_CmdGetElementAttributes(p_link->remote_bd) == AVRCP_SUCCESS?true : false;
}
bool CmdAction_RegisterNotification(TAvrcpLink *p_link, uint8_t event_id)
{
    return avrcp_CmdRegisterNotification(p_link->remote_bd, event_id) == AVRCP_SUCCESS?true : false;
}

/**
 * @brief
 *
 * @date 2015/11/18
 * @param void
 *
 */
static void RegisterNotificationSupported(TAvrcpLink *p_link)
{
    //register events both supported
    uint16_t tmp = p_link->remote_events_supported & LOCAL_EVENTS_SUPPORTED_MASK;
    uint8_t event_id = 0;
    while (tmp != 0)
    {
        if ((tmp & 0x1) != 0)
        {
            PendCmdAction(p_link, CmdAction_RegisterNotification, event_id);
        }
        tmp >>= 1;
        event_id++;
    }
//  //if support track changed
    PendCmdAction(p_link, CmdAction_GetPlayStatus, 0);
    PendCmdAction(p_link, CmdAction_GetElementAttributes, 0);
}

void KeyPress(uint8_t op_id) //auto start/stop resend timer
{
    App.key_hold_resend_op_id = op_id;
    xTimerStart(App.key_hold_resend_timer, 0);
    PendCmdAction(App.p_active_link, CmdAction_Passthrough, (PRESSED << 7) | op_id);
}
void KeyRelease(uint8_t op_id)
{
    xTimerStop(App.key_hold_resend_timer, 0);
    App.key_hold_resend_op_id = 0;
    PendCmdAction(App.p_active_link, CmdAction_Passthrough, (RELEASED << 7) | op_id);
}

void KeyHoldResendTimeout(xTimerHandle timer) //not auto reload: timeout->resend key press->start timer again
{
    KeyPress(App.key_hold_resend_op_id);
    printf("Key hold resend[%x]\r\n", App.key_hold_resend_op_id);
}

void HandleRspUnitInfo(TAvrcpLink *p_link, const TRspUnitInfo *rsp) //app specific
{
    if (rsp->state == AVRCP_RSP_STATE_SUCCESS)
    {
        printf("subUnitType=0x%x, ", rsp->sub_unit_type); //AVRCP_PANEL
        printf("subUnitId=0x%x, ", rsp->sub_unit_id);
        printf("company_id=%x\r\n", rsp->company_id);

        p_link->remote_company_id = rsp->company_id;
        ; //if company_id == mycompany, do some thing
    }

}
void HandleRspPassthrough(TAvrcpLink *p_link, const TRspPassthrough *rsp) //app specific
{
    if (rsp->state != AVRCP_RSP_STATE_SUCCESS)
    {
        printf("rsp_passthrough fail\r\n");
    }
}
void HandleRspGetCapability(TAvrcpLink *p_link, const TRspCapability *rsp) //app specific
{
    if (rsp->state == AVRCP_RSP_STATE_SUCCESS)
    {
        uint8_t *buf = rsp->buf;
        switch (rsp->capability_id)
        {
        case CAPABILITY_ID_COMPANY_ID:
            for (int i = 0; i < rsp->capability_count; i++)
            {
                p_link->remote_company_id = (uint32_t)buf[0] << 16 | (uint32_t)buf[1] << 8 | (uint32_t)buf[2];
                buf += 3;
                printf("capability: company_id = %x\r\n", p_link->remote_company_id);
            }
            break;
        case CAPABILITY_ID_EVENTS_SUPPORTED:
            for (int i = 0; i < rsp->capability_count; i++)
            {
                uint8_t event_id = *buf++;
                if (event_id < 8 * sizeof(p_link->remote_events_supported))
                {
                    p_link->remote_events_supported |= (uint16_t)0x1 << event_id;
                }
            }
            RegisterNotificationSupported(p_link);
        }
    } else
    {
        printf("GetCapabilityRsp err\r\n");
    }
}
void HandleRspGetPlayStatus(TAvrcpLink *p_link, const TRspGetPlayStatus *rsp) //app specific
{
    if (rsp->state == AVRCP_RSP_STATE_SUCCESS)
    {
//      if (p_link == App.p_active_link)
        App.play_status = rsp->play_status;
        App.song_length = rsp->length_ms;
        App.song_position = rsp->position_ms;
        printf("App[%d]:\t", p_link - &App.link[0]);
        /*play status*/
        printf("PlayStatus:%d/%d,%s\r\n", App.song_position / 1000, App.song_length / 1000, PlayStatusToString(App.play_status));
    } else
    {
        printf("GetPlayStatusRsp err\r\n");
    }
}
void HandleRspGetElementAttributes(TAvrcpLink *p_link, const TRspGetElementAttributes *rsp) //app specific
{
    if (rsp->state == AVRCP_RSP_STATE_SUCCESS)
    {
        for (int i = 0; i < rsp->number_of_attributes; i++)
        {
            uint32_t attribute_id       = rsp->attr[i].attribute_id;
            uint16_t character_set_id   = rsp->attr[i].character_set_id;
            UNUSED_PARAMETER(character_set_id);
            uint16_t attribute_length   = rsp->attr[i].length;
            uint8_t *buf                = rsp->attr[i].buf;
            char *p_write;
            switch (attribute_id)
            {
            case ELEMENT_ATTRIBUTE_TITLE:
                p_write = App.attr.title;           break;
            case ELEMENT_ATTRIBUTE_ARTIST:
                p_write = App.attr.artist;          break;
            case ELEMENT_ATTRIBUTE_ALBUM:
                p_write = App.attr.album;           break;
            case ELEMENT_ATTRIBUTE_TRACK:
                p_write = App.attr.track;           break;
            case ELEMENT_ATTRIBUTE_TOTAL_TRACK:
                p_write = App.attr.total_track;     break;
            case ELEMENT_ATTRIBUTE_GENRE:
                p_write = App.attr.genre;           break;
            case ELEMENT_ATTRIBUTE_PLAYING_TIME:
                p_write = App.attr.playing_time;    break;
            default:
                p_write = NULL;
                //              printf("unkown attribute_id:%x\r\n", attribute_id);
                break;
            }
            if (p_write != NULL)
            {
                if (attribute_length < ELEMENT_ATTR_LENGTH)
                {
                    p_write[attribute_length] = '\0';
                    memcpy(p_write, buf, attribute_length);
                } else
                {
                    p_write[ELEMENT_ATTR_LENGTH - 1] = '\0';
                    memcpy(p_write, buf, ELEMENT_ATTR_LENGTH - 1 - 3);
                    p_write[ELEMENT_ATTR_LENGTH - 1 - 3] = '.';
                    p_write[ELEMENT_ATTR_LENGTH - 1 - 3 + 1] = '.';
                    p_write[ELEMENT_ATTR_LENGTH - 1 - 3 + 2] = '.'; //to indicate str is truncated
                }
            }
        }

    } else
    {
        printf("GetElementAttributesRsp err\r\n");
    }
}
void HandleRspRegisterNotification(TAvrcpLink *p_link, const TRspRegisterNotification *rsp) //app specific
{
    if (rsp->state == AVRCP_RSP_STATE_SUCCESS)
    {
        printf("register event(%x) ok\r\n", rsp->event_id);
        switch (rsp->event_id)
        {
        case EVENT_PLAYBACK_STATUS_CHANGED:
            App.play_status = rsp->u.play_status;
            break;
        case EVENT_TRACK_CHANGED:
            App.track_id = *(rsp->u.p_track_id);
            break;
        default:
            break;
        }
    } else
    {
        printf("RegisterNotificationRsp err\r\n");
    }
}
void HandleNofication(TAvrcpLink *p_link, const TNotificationChanged *rsp) //app specific
{
    PendCmdAction(p_link, CmdAction_RegisterNotification, rsp->event_id); //re-register
    printf("App[%d]:\t", p_link - &App.link[0]);
    switch (rsp->event_id)
    {
    case EVENT_PLAYBACK_STATUS_CHANGED:
        App.play_status = rsp->u.play_status;

        if (rsp->u.play_status == AVRCP_PLAYSTATUS_PLAYING)
        {
            PendCmdAction(p_link, CmdAction_GetPlayStatus, 0); //update status/playpos/length
        }
        printf("PlayStatus->%s\r\n", PlayStatusToString(App.play_status));
        break;
    case EVENT_TRACK_CHANGED:
        App.track_id = *(rsp->u.p_track_id);
        PendCmdAction(p_link, CmdAction_GetPlayStatus, 0);
        PendCmdAction(p_link, CmdAction_GetElementAttributes, 0);
        printf("Track->%x%x\r\n", *((int *)&(App.track_id) + 1), (int)App.track_id);
        break;
    default:
        break;
    }
}
TAvrcpLink* AllocateNewLinkInst(TBdAddr bd)
{
    for (int i = 0; i < MAX_AVRCP_APP_INST; i++) //first find for old link
    {
        TAvrcpLink *p_link = &App.link[i];
        if (p_link->state != avrcpStateConnected && memcmp(p_link->remote_bd, bd, BLUE_API_BD_SIZE) == 0)
        {
            return p_link;
        }
    }
    for (int i = 0; i < MAX_AVRCP_APP_INST; i++) //no old device, allocate new avrcp inst
    {
        TBdAddr bd_zero = { 0 };
        TAvrcpLink *p_link = &App.link[i];
        if (p_link->state != avrcpStateConnected && memcmp(p_link->remote_bd, bd_zero, BLUE_API_BD_SIZE) == 0) //new inst
        {
            return p_link;
        }
    }
    for (int i = 0; i < MAX_AVRCP_APP_INST; i++) //no new avrcp inst, overwrite an old(bd != 0) inst
    {
        TAvrcpLink *p_link = &App.link[i];
        if (p_link->state != avrcpStateConnected)
        {
            return p_link;
        }
    }
    return NULL;
}
TAvrcpLink* GetAvrcpLinkByBd(TBdAddr bd)
{
    for (int i = 0; i < MAX_AVRCP_APP_INST; i++)
    {
        TAvrcpLink *p_link = &App.link[i];
        if (/*p_link->state == avrcpStateConnected && */memcmp(p_link->remote_bd, bd, BLUE_API_BD_SIZE) == 0)
        {
            return p_link;
        }
    }
    return NULL;
}
void HandleMessageConnectReq(TBdAddr bd)
{
    TAvrcpLink *p_link = AllocateNewLinkInst(bd); //just peek if there is empty link
    if (p_link != NULL)
    {
        avrcp_ConnectConfirm(bd, true);
    } else
    {
        avrcp_ConnectConfirm(bd, false);
    }
}
void HandleMessageConnected(TBdAddr bd)
{
    TAvrcpLink *p_link = AllocateNewLinkInst(bd);
    if (memcmp(bd, p_link->remote_bd, BLUE_API_BD_SIZE) != 0) //new device
    {
        memset(p_link, 0, sizeof(*p_link));
        memcpy(p_link->remote_bd, bd, BLUE_API_BD_SIZE);
        p_link->cmd_credits = 1;
        p_link->state = avrcpStateConnected;

        PendCmdAction(p_link, CmdAction_UnitInfo, 0); //must be the first cmd
        PendCmdAction(p_link, CmdAction_GetCapability, CAPABILITY_ID_COMPANY_ID);
        PendCmdAction(p_link, CmdAction_GetCapability, CAPABILITY_ID_EVENTS_SUPPORTED);
        printf("avrcp new device[%d]: Connected:[%x %x %x]\r\n", p_link - &App.link[0], bd[2], bd[1], bd[0]);
    } else //old device
    {
        p_link->cmd_credits = 1;
        p_link->state = avrcpStateConnected;
        FlushPendingCmdAction(p_link); //flush pending passthrough act
        RegisterNotificationSupported(p_link);
        printf("avrcp old device[%d]: Connected:[%x %x %x]\r\n", p_link - &App.link[0], bd[2], bd[1], bd[0]);
    }
}
void HandleMessageDisconnected(TBdAddr bd)
{
    TAvrcpLink *p_link = GetAvrcpLinkByBd(bd);
    if (p_link != NULL) //may be connected or outgoing connecting
    {
        if (p_link->state == avrcpStateOutgoingConnecting)
        {
            printf("reconnect failed\r\n");
        }
        p_link->state = avrcpStateDisconnected;
        p_link->vol_change_registered = false;
        ClearPendingCmdAction(p_link); //abort pending act
//      xTimerStop(pAvrcpProfile->key_hold_resend_timer, 0);
        printf("avrcp[%d]: Disconnected: [%x %x %x]\r\n", p_link - &App.link[0],  bd[2], bd[1], bd[0]);
    }
}

void HandleCmdRegisterVolChange(TAvrcpLink *p_link, uint8_t *ret_vollume)
{
    p_link->vol_change_registered = true;
    *ret_vollume = App.volume;
}
void MyVendorCmdHandler(TBdAddr bd, uint32_t company_id, uint8_t *p_pdu, uint16_t length, uint8_t ctype, uint8_t transact)
{
    TAvrcpLink *p_link = GetAvrcpLinkByBd(bd);
    if (p_link == NULL)
    {
        return;
    }
    printf("App[%d]:\t", p_link - &App.link[0]);
    printf("recv MyVendorCmd:Vendor=%x,pdu length=%d\r\n", company_id, length);
    if (0)
    {
        ; //handle cmd
    } else
    {
        avrcp_SendVendorResponse(bd, transact, AVRCP_NOT_IMPLEMENTED, MY_COMPANY_ID, p_pdu, length);
    }
}
void MyVendorRspHandler(TBdAddr bd, uint32_t company_id, uint8_t *p_pdu, uint16_t length, uint8_t response)
{
    TAvrcpLink *p_link = GetAvrcpLinkByBd(bd);
    if (p_link == NULL)
    {
        return;
    }
    printf("App[%d]:\t", p_link - &App.link[0]);
    printf("recv MyVendorRsp:Vendor=%x,pdu length=%d\r\n", company_id, length);
}
void AvrcpMessageHandler(TBdAddr bd, AvrcpUsMessageType message_type, void *message)
{
    if (/*message_type >= AVRCP_US_MESSAGE_CONNECT_REQ &&*/message_type <= AVRCP_US_MESSAGE_DISCONNECTED)
    {
        switch (message_type)
        {
        case AVRCP_US_MESSAGE_CONNECT_REQ:
            HandleMessageConnectReq(bd);
            break;
        case AVRCP_US_MESSAGE_CONNECTED:
            HandleMessageConnected(bd);
            break;
        case AVRCP_US_MESSAGE_DISCONNECTED:
            HandleMessageDisconnected(bd);
            break;
        }
        return;
    } else if (message_type >= AVRCP_US_MESSAGE_RSP_UNIT_INFO && message_type < AVRCP_US_MESSAGE_RSP_DUMMY)
    {
        TAvrcpLink *p_link = GetAvrcpLinkByBd(bd);
        if (p_link == NULL)
        {
            return;
        }

        p_link->cmd_credits = 1;

        switch (message_type)
        {
        case AVRCP_US_MESSAGE_RSP_UNIT_INFO:
            HandleRspUnitInfo(p_link, (TRspUnitInfo *)message); //
            break;
        case AVRCP_US_MESSAGE_RSP_PASSTHROUGH:
            HandleRspPassthrough(p_link, (TRspPassthrough *)message); //
            break;
        case AVRCP_US_MESSAGE_RSP_GET_CAPABILITY:
            HandleRspGetCapability(p_link, (TRspCapability *)message); //
            break;
        case AVRCP_US_MESSAGE_RSP_GET_PLAYSTATUS:
            HandleRspGetPlayStatus(p_link, (TRspGetPlayStatus *)message); //
            break;
        case AVRCP_US_MESSAGE_RSP_GET_ELEMENT_ATTR:
            HandleRspGetElementAttributes(p_link, (TRspGetElementAttributes *)message);
            break;
        case AVRCP_US_MESSAGE_RSP_REGISTER_NOTIFICATION:
            HandleRspRegisterNotification(p_link, (TRspRegisterNotification *)message);
            break;
        }

        FlushPendingCmdAction(p_link); //if there is cmdCredits, process one pending cmd action
        return;
    }
#if FEATURE_VOLUME_SYNC_SUPPORT
else if (message_type >= AVRCP_US_MESSAGE_CMD_VOL_UP && message_type <= AVRCP_US_MESSAGE_CMD_REGISTER_VOL_CHANGE)
    {
        TAvrcpLink *p_link = GetAvrcpLinkByBd(bd);
        if (p_link == NULL)
        {
            return;
        }
        switch (message_type)
        {
        case AVRCP_US_MESSAGE_CMD_VOL_UP:
            VolumeUp();
            break;
        case AVRCP_US_MESSAGE_CMD_VOL_DOWN:
            VolumeDown();
            break;
        case AVRCP_US_MESSAGE_CMD_ABS_VOL:
            App.volume = *(uint8_t *)message;
            printf("link[%d] cmd:\t", p_link - &App.link[0]);
            printf("VOLUME-> %d/127\r\n", App.volume);
            break;
        case AVRCP_US_MESSAGE_CMD_REGISTER_VOL_CHANGE:
            {
                uint8_t *ret_volume = (uint8_t *)message;
                HandleCmdRegisterVolChange(p_link, ret_volume);
            }
            break;
        }
        return;
    }
#endif
else
    {
        TAvrcpLink *p_link = GetAvrcpLinkByBd(bd);
        if (p_link == NULL)
        {
            return;
        }
        switch (message_type)
        {
        case AVRCP_US_MESSAGE_NOTIFICATION_CHANGED:
            HandleNofication(p_link, (TNotificationChanged *)message);
            break;
        case AVRCP_US_MESSAGE_ERR:
            {
                AvrcpUsMessageErr err = (AvrcpUsMessageErr)(int)message;
                if (err == AVRCP_US_ERR_WAIT_RESPONSE_TIMEOUT)
                {
                    printf("avrcp err wait response timeout\r\n");
                } else
                {
                    printf("avrcp err:%x\r\n", err);
                }
                //  if (p_avrcp->state >= avrcpStateConnected)
                //  {
                //      printf("WaitResponseTimeout\r\n");
                //      avctp_Disconnect();
                //      //p_avrcp->state = avrcpStateDisconnecting;
                //  }
            }
            break;
        default:
            break;
        }
        return;
    }
}

bool avrcp_app_init(void)
{
    UARTInit();
    if (AVRCP_SUCCESS != avrcp_Init(AvrcpMessageHandler, true, MY_COMPANY_ID, MyVendorCmdHandler, MyVendorRspHandler))
    {
        printf("avrcp_Init fail\r\n");
        return false;
    }
    App.p_active_link = &App.link[0];
    App.key_hold_resend_timer = xTimerCreate(NULL, KEY_HOLD_RESEND_TIME, pdFALSE, NULL, KeyHoldResendTimeout); /*no auto reload*/
    if (App.key_hold_resend_timer == NULL)
    {
        printf("xTimerCreate fail\r\n");
        return false;
    }
    //  pAvrcpProfile->key_hold_determine_timer = xTimerCreate(NULL, KEY_HOLD_DETERMINE_TIME, pdFALSE, NULL, KeyHoldDetermineTimeout); /*no auto reload */
    //  if (pAvrcpProfile->key_hold_determine_timer == NULL)
    //  {
    //      printf("xTimerCreate fail\r\n");
    //      return false;
    //  }
    printf("avrcp app ready\r\n");
    return true;
}

void avrcp_app_sim_cmd(char rx)
{

    printf("%c\r\n", rx);

    if (rx == '+')
    {
        VolumeUp();
    } else if (rx == '-')
    {
        VolumeDown();
    }
    else if (rx == 'u') //update play status
    {
        PendCmdAction(App.p_active_link, CmdAction_GetPlayStatus, 0);
        PendCmdAction(App.p_active_link, CmdAction_GetElementAttributes, 0);
    } else if (rx == '>') //switch active inst -> to app layer
    {
        int index = (App.p_active_link - &App.link[0] + 1) % MAX_AVRCP_APP_INST; //0->1->(Max - 1)->0
        App.p_active_link = &App.link[index];
        printf("active inst->%d\r\n", index);
        return;
    } else if (rx == 'i')
    {
        printf("\r\n------info of app:-------\r\n"); //
        printf("remote bd[...%x:%x:%x]\r\n", App.p_active_link->remote_bd[2], App.p_active_link->remote_bd[1], App.p_active_link->remote_bd[0]);
        printf("state=%x\r\n", App.p_active_link->state);
        printf("company_id=%x\r\n", App.p_active_link->remote_company_id);
        /*events*/
        uint16_t tmp = App.p_active_link->remote_events_supported;
        int i = 0;
        printf("remoteEventsSupported:\r\n");
        while (tmp != 0)
        {
            if ((tmp & 0x1) != 0) printf("%x ", i);
            tmp >>= 1;
            i++;
        }
        printf("\r\n");
        /*play status*/
        printf("PlayStatus:%d/%d,%s\r\n", App.song_position / 1000, App.song_length / 1000, PlayStatusToString(App.play_status));
        printf("Track ID :%x%x\r\n", *((int *)&(App.track_id) + 1), (int)App.track_id);
        /*attr*/
        PrintUTF8(App.attr.title); printf(" -- ");
        PrintUTF8(App.attr.track); printf("/");
        PrintUTF8(App.attr.total_track); printf("\r\n");
        PrintUTF8(App.attr.artist); printf(" -- ");
        PrintUTF8(App.attr.album); printf("\r\n");
        PrintUTF8(App.attr.genre); printf("\r\n");
    } else if (rx == 'd')
    {
        avrcp_DisConnect(App.p_active_link->remote_bd);
        printf("disconnecting link[%d]...\r\n", App.p_active_link - &App.link[0]);
    } else
    {

        AvrcpKey key = (AvrcpKey)0;
        switch (rx)
        {
        case 'P':
        case 'p':
            key = AVRCP_KEY_PLAY; break;
        case 'S':
        case 's':
            key = AVRCP_KEY_STOP; break;
        case 'F':
        case 'f':
            key = AVRCP_KEY_FORWARD; break;
        case 'B':
        case 'b':
            key = AVRCP_KEY_BACKWARD; break;
        case 'A':
        case 'a':
            key = AVRCP_KEY_PAUSE; break;
        case 'R':
        case 'r':
            key = AVRCP_KEY_REWIND; break;
        case 'T':
        case 't':
            key = AVRCP_KEY_FAST_FORWARD; break;
        default:
            break;
        }
        if (key != 0)
        {
            { /*if not connected, reconnect*/
                //          if (AppAvrcp.pActiveInst->state == avrcpStateIdle) //no last connect info, for test
                //          {
                //              uint8_t bd[6] = //my phone
                //              { 0x7b, 0xf4, 0x10, 0x14, 0xe3, 0x24 };
                //              //      memcpy(AppAvrcp.pActiveInst->remoteBd, bd, 6);
                //              AppAvrcp.pActiveInst->state = avrcpStateOutgoingConnecting;
                //              avctp_Connect(bd);
                //              printf("reconnect[%d]...\r\n", AppAvrcp.pActiveInst - &AppAvrcp.inst[0]);
                //          }

                if (App.p_active_link->state == avrcpStateIdle)
                {
                    printf("no remote device\r\n");
                    return;
                }

                if (App.p_active_link->state == avrcpStateDisconnected)
                {
                    App.p_active_link->state = avrcpStateOutgoingConnecting;
                    avrcp_Connect(App.p_active_link->remote_bd);
                    printf("reconnect[%d]...\r\n", App.p_active_link - &App.link[0]);
                }
            }

            static bool pressed = false;
            if (rx >= 'a') //lower case
            {
                if (pressed) //release
                {
                    pressed = false;
                    KeyRelease((uint8_t)key & 0x7f);
                } else //press and then auto release on cmdCredit gained
                {
                    KeyPress((uint8_t)key & 0x7f);
                    KeyRelease((uint8_t)key & 0x7f);
                }
            } else //upper case, press
            {
                pressed = true;
                KeyPress((uint8_t)key & 0x7f);
            }
        }
        //    avrcp_DebugCmd(rx);
    }
    return;
}
