
enum
{
    __FILE_NUM__ = 0 };

#include "rtl_types.h"
#include "blueapi.h"
#include "legacy.h"
#include "trace.h"
#include "common_defs.h"
//#include "sppdemo.h"
#include "avrcp_api.h"
#include "avrcp.h"
#include "avctp.h"
#include "mpa.h"
#include "os_timer.h"
#include "os_mem.h"
//#include <FreeRTOS.h>
//#include <task.h>
//#include <timers.h>



static TAvrcpProfile *pAvrcpProfile; // = {.AppAvrcp.pActiveInst = &AppAvrcp.inst[0]};
//static TAvrcpInst *AppAvrcp.pActiveInst = &AppAvrcp.inst[0];


/**
 * @brief 
 * 
 * @date 2015/11/17
 * 
 * @return TAvrcpInst* 
 */
static TAvrcpInst* AllocateAvrcpInst(void)
{
    TBdAddr bd = { 0 };
    for (int i = 0; i < MAX_AVRCP_LINK; i++)
    {
        TAvrcpInst *p_avrcp = &pAvrcpProfile->inst[i];
        if (memcmp(p_avrcp->remote_bd, bd, BLUE_API_BD_SIZE) == 0)
        {
            return p_avrcp;
        }
    }
    return NULL; //active avrcp inst num > MAX_AVRCP_LINK
}
static TAvrcpInst* GetAvrcpInstByBd(TBdAddr bd)
{
    if (bd == NULL)
    {
        return NULL;
    }
    for (int i = 0; i < MAX_AVRCP_LINK; i++)
    {
        TAvrcpInst *p_avrcp = &pAvrcpProfile->inst[i];
        if (memcmp(p_avrcp->remote_bd, bd, BLUE_API_BD_SIZE) == 0)
        {
            return p_avrcp;
        }
    }
    return NULL;
}
static TAvrcpInst* GetAvrcpInstByCid(uint16_t cid)
{
    for (int i = 0; i < MAX_AVRCP_LINK; i++)
    {
        TAvrcpInst *p_avrcp = &pAvrcpProfile->inst[i];
        if (p_avrcp->cid == cid)
        {
            return p_avrcp;
        }
    }
    return NULL;
}

void WaitResponseTimeout(void *p_timer) //don't block
{
    TAvrcpInst *p_avrcp;
    for (int i = 0; i < MAX_AVRCP_LINK; i++)
    {
        if (p_timer == pAvrcpProfile->inst[i].wait_response_timer)
        {
            p_avrcp = &pAvrcpProfile->inst[i];
            break;
        }
    }
    p_avrcp->cmd_credits = 1;
    pAvrcpProfile->app_message_handler(p_avrcp->remote_bd, AVRCP_US_MESSAGE_ERR, (void *)AVRCP_US_ERR_WAIT_RESPONSE_TIMEOUT);

}

/**
 * @brief 
 * 
 * @date 2015/11/17
 * 
 * @param bd
 * 
 * @return TAvrcpInst
 */
//TAvrcpInst* GetConnectingAvrcpInst(TBdAddr bd)
//{
//    for (int i = 0; i < MAX_AVRCP_LINK; i++) //first find for old device
//    {
//        TAvrcpInst *p_avrcp = &pAvrcpProfile->inst[i];
//        if ((p_avrcp->state == avrcpStateIncomingConnecting || p_avrcp->state == avrcpStateOutgoingConnecting)
//            && memcmp(p_avrcp->remote_bd, bd, BLUE_API_BD_SIZE) == 0)
//        {
////          printf("getConnecting old AppAvrcp.inst[%d]\r\n", i);
//            return p_avrcp;
//        }
//    }
//    for (int i = 0; i < MAX_AVRCP_LINK; i++) //no old device, allocate any connecting avrcp inst
//    {
//        TAvrcpInst *p_avrcp = &pAvrcpProfile->inst[i];
//        if (p_avrcp->state == avrcpStateIncomingConnecting || p_avrcp->state == avrcpStateOutgoingConnecting)
//        {
////          printf("getConnecting new AppAvrcp.inst[%d]\r\n", i);
//            return p_avrcp;
//        }
//    }
//    return NULL;
//}
//*GetOutgoingAvrcpInst(void)
//{
//    for (int i = 0; i < MAX_AVRCP_LINK; i++)
//    {
//        TAvrcpInst *p_avrcp = &pAvrcpProfile->inst[i];
//        if (p_avrcp->state == avrcpStateOutgoingConnecting)
//        {
//            return p_avrcp;
//        }
//    }
//    return NULL;
//}
static AvrcpReturnStatus SendGenericCmd(TAvrcpInst *p_avrcp, uint8_t *avrcp_data, short length)
{
    if (p_avrcp->cmd_credits == 1)
    {
        p_avrcp->transact_label = (p_avrcp->transact_label + 1) & 0x0f; /* advance transaction label (4 bit field) */
        int ret = avctpSendData(p_avrcp->cid, avrcp_data, length, p_avrcp->transact_label, AVCTP_MESSAGE_TYPE_COMMAND);
        if (1 == ret)
        {
            p_avrcp->cmd_credits = 0;
            osStartTimer(&p_avrcp->wait_response_timer, 0, 0, 0, WAIT_RESPONSE_TIMEOUT, WaitResponseTimeout);
//          xTimerStart(p_avrcp->wait_response_timer, 0); //start timer
            return AVRCP_SUCCESS;
        } else if (ret == -1)
        {
            return AVRCP_INVALID_PARA;
        } else //ret == 0
        {
            return AVRCP_NO_RESOURCE;
        }
//      lastCmdTick = xTaskGetTickCount();
    } else
    {
        return AVRCP_INVALID_STATE;
    }
}
static AvrcpReturnStatus SendVendorBT_SIGCmd(TAvrcpInst *p_avrcp, uint8_t ctype, uint8_t pdu_id, uint8_t *para, int16_t para_length)
{
    if (p_avrcp->cmd_credits == 1)
    {
        p_avrcp->transact_label = (p_avrcp->transact_label + 1) & 0x0f; /* advance transaction label (4 bit field) */

        uint8_t avrcp_data[10]; //header before pdu para
        uint8_t *p = avrcp_data;
        *p++ = 0xf & ctype;
        *p++ = (AVRCP_PANEL << 3) | (0 & 0x7);
        *p++ = OPCODE_VENDOR_DEPENDENT;           //Operation Code
        *p++ = (uint8_t)(COMPANY_BT_SIG >> 16);
        *p++ = (uint8_t)(COMPANY_BT_SIG >> 8);
        *p++ = (uint8_t)COMPANY_BT_SIG;
        *p++ = pdu_id; //pdu id
        *p++ = AVRCP_PDU_NON_FRAGMENTED & 0x3; //Packet type: Non-Fragmented
        *p++ = (uint8_t)(para_length >> 8);
        *p++ = (uint8_t)para_length;
        int ret = avctp_SendData2Buf(p_avrcp->cid, avrcp_data, sizeof(avrcp_data), para, para_length, p_avrcp->transact_label, AVCTP_MESSAGE_TYPE_COMMAND);
        if (1 == ret)
        {
            p_avrcp->cmd_credits = 0;
            osStartTimer(&p_avrcp->wait_response_timer, 0, 0, 0, WAIT_RESPONSE_TIMEOUT, WaitResponseTimeout);
//          xTimerStart(p_avrcp->wait_response_timer, 0); //start timer
            return AVRCP_SUCCESS;
        } else if (ret == -1)
        {
            return AVRCP_INVALID_PARA;
        } else
        {
            return AVRCP_NO_RESOURCE;
        }
//      lastCmdTick = xTaskGetTickCount();
    } else
    {
        return AVRCP_INVALID_STATE;
    }
}

AvrcpReturnStatus avrcp_SendVendorCommand(TBdAddr bd, uint8_t ctype, uint32_t company_id, uint8_t *p_pdu, uint16_t pdu_length)
{
    TAvrcpInst *p_avrcp = GetAvrcpInstByBd(bd);
    if (p_avrcp != NULL)
    {
        if (p_avrcp->cmd_credits == 1)
        {
            p_avrcp->transact_label = (p_avrcp->transact_label + 1) & 0x0f; /* advance transaction label (4 bit field) */
            uint8_t avrcp_data[] = //[6]header before pdu
            {
                0xf & ctype,
                (AVRCP_PANEL << 3) | (0 & 0x7),
                OPCODE_VENDOR_DEPENDENT,
                (uint8_t)(company_id >> 16),
                (uint8_t)(company_id >> 8),
                (uint8_t)company_id
            };
            if (1 == avctp_SendData2Buf(p_avrcp->cid, avrcp_data, sizeof(avrcp_data), p_pdu, pdu_length, p_avrcp->transact_label, AVCTP_MESSAGE_TYPE_COMMAND))
            {
                p_avrcp->cmd_credits = 0;
                osStartTimer(&p_avrcp->wait_response_timer, 0, 0, 0, WAIT_RESPONSE_TIMEOUT, WaitResponseTimeout);
//              xTimerStart(p_avrcp->wait_response_timer, 0); //start timer
                return AVRCP_SUCCESS;
            } else
            {
                return AVRCP_INVALID_PARA; // to change
            }
        } else
        {
            return AVRCP_INVALID_STATE;
        }
    } else
    {
        return AVRCP_INVALID_PARA;
    }
}
static void SendVendorBT_SIGResponse(TAvrcpInst *p_avrcp, uint8_t transact, uint8_t response, uint8_t pdu_id, uint8_t *para, uint16_t para_length)
{
    uint8_t avrcp_data[] = { //[10]avrcp header before pdu para
        response & 0xf,          //response type
        (AVRCP_PANEL << 3) | (0 & 0x7),
        OPCODE_VENDOR_DEPENDENT,
        (uint8_t)(COMPANY_BT_SIG >> 16),
        (uint8_t)(COMPANY_BT_SIG >> 8),
        (uint8_t)COMPANY_BT_SIG,
        pdu_id, //pdu id
        AVRCP_PDU_NON_FRAGMENTED & 0x3, //Packet type: Non-Fragmented
        (uint8_t)(para_length >> 8),
        (uint8_t)para_length
    };
    avctp_SendData2Buf(p_avrcp->cid, avrcp_data, sizeof(avrcp_data),
                       para, para_length, transact, AVCTP_MESSAGE_TYPE_RESPONSE);
}


AvrcpReturnStatus avrcp_SendVendorResponse(TBdAddr bd, uint8_t transact, uint8_t response, uint32_t company_id, uint8_t *p_pdu, uint16_t pdu_length)
{
    TAvrcpInst *p_avrcp = GetAvrcpInstByBd(bd);
    if (p_avrcp != NULL)
    {
        uint8_t avrcp_data[] = //[6]avrcp header before pdu
        {
            response & 0xf,          //response type
            (AVRCP_PANEL << 3) | (0 & 0x7),
            OPCODE_VENDOR_DEPENDENT,
            (uint8_t)(company_id >> 16),
            (uint8_t)(company_id >> 8),
            (uint8_t)company_id
        };
        if (1 == avctp_SendData2Buf(p_avrcp->cid, avrcp_data, sizeof(avrcp_data),
                                    p_pdu, pdu_length, transact, AVCTP_MESSAGE_TYPE_RESPONSE))
        {
            return AVRCP_SUCCESS;
        } else
        {
            return AVRCP_INVALID_PARA; // to change
        }
    } else
    {
        return AVRCP_INVALID_PARA;
    }
}

AvrcpReturnStatus avrcp_CmdUnitInfo(TBdAddr bd)
{
    TAvrcpInst *p_avrcp = GetAvrcpInstByBd(bd);
    if (p_avrcp != NULL)
    {
        uint8_t avrcp_data[] = {
            AVRCP_STATUS,        //command type
            0xff,                //subunit type << 3 , subunit ID
            OPCODE_UNIT_INFO,           //Operation Code
            0xff, 0xff, 0xff, 0xff, 0xff
        };
        return SendGenericCmd(p_avrcp, avrcp_data, sizeof(avrcp_data));
    } else
    {
        return AVRCP_INVALID_PARA;
    }
}
/**
 * @brief 
 * @note if key is pressed for >2s, key press cmd should be 
 *       resent every <= 2s. (Command with the pressed value is
 *       valid for TWO seconds from the time when a target sends
 *       back a response of the command. The controller shall
 *       continue sending pressed value with identical operation
 *       id value in the operation_id field while the command is
 *       wished to stay valid. 9.4, AV/C Panel Subunit
 *       Specification 1.1)
 * 
 * @date 2015/12/14
 * 
 * @param bd 
 * @param key 
 * @param pressed 
 * 
 * @return bool 
 */
AvrcpReturnStatus avrcp_CmdPassThrough(TBdAddr bd, AvrcpKey key, bool pressed)
{
//  check para;
    TAvrcpInst *p_avrcp = GetAvrcpInstByBd(bd);
    if (p_avrcp != NULL)
    {
        uint8_t avrcp_data[] = {
            AVRCP_CONTROL,          //command type
            (AVRCP_PANEL << 3) | (0 & 0x7), //subunit type << 3 , subunit ID
            OPCODE_PASS_THROUGH,           //Operation Code
            (pressed ? PASSTHROUGH_STATE_PRESSED : PASSTHROUGH_STATE_RELEASED) << 7 | ((uint8_t)key & 0x7f), //state flag << 7, operation ID
            0                       //opration data length
        };
        return SendGenericCmd(p_avrcp, avrcp_data, sizeof(avrcp_data));
    } else
    {
        return AVRCP_INVALID_PARA;
    }
}
AvrcpReturnStatus avrcp_CmdGetCapability(TBdAddr bd, uint8_t capability_id)
{
    ; //check para
    TAvrcpInst *p_avrcp = GetAvrcpInstByBd(bd);
    if (p_avrcp != NULL)
    {
        return SendVendorBT_SIGCmd(p_avrcp, AVRCP_STATUS, PDU_ID_GET_CAPABILITIES, &capability_id, 1); //
    } else
    {
        return AVRCP_INVALID_PARA;
    }
}
AvrcpReturnStatus avrcp_CmdGetPlayStatus(TBdAddr bd)
{
    TAvrcpInst *p_avrcp = GetAvrcpInstByBd(bd);
    if (p_avrcp != NULL)
    {
        return SendVendorBT_SIGCmd(p_avrcp, AVRCP_STATUS, PDU_ID_GET_PLAY_STATUS, NULL, 0);
    } else
    {
        return AVRCP_INVALID_PARA;
    }
}
AvrcpReturnStatus avrcp_CmdGetElementAttributes(TBdAddr bd)
{
    TAvrcpInst *p_avrcp = GetAvrcpInstByBd(bd);
    if (p_avrcp != NULL)
    {
        uint8_t pdu_para[9] = { 0 };
        //;[0~7] Identifier = PLAYING (0x0)
        //;pdu_para[8] = 0; //num= 0 : all attr
//      short para_length;
//      if (para == ELEMENT_ATTRIBUTE_ALL)
//      {
//          pdu_para[8] = 0; //num= 0 : all attr
//          para_length = 9;
//      } else
//      {
//          pdu_para[8] = 1;
//          NETLONG2CHAR(pdu_para + 9, (uint32_t)(para)); //1-title(Mandatory),2-artist,3-album,4-track num,5-total track num,6-genre,7-song length in miliseconds
//          para_length = 13;
//      }
        return SendVendorBT_SIGCmd(p_avrcp, AVRCP_STATUS, PDU_ID_GET_ELEMENT_ATTRIBUTES, pdu_para, sizeof(pdu_para)); //
    } else
    {
        return AVRCP_INVALID_PARA;
    }
}
AvrcpReturnStatus avrcp_CmdRegisterNotification(TBdAddr bd, uint8_t event_id)
{
    TAvrcpInst *p_avrcp = GetAvrcpInstByBd(bd);
    if (p_avrcp != NULL)
    {
        uint8_t pdu_para[] = {
            event_id,
            0, 0, 0, 0 };
        return SendVendorBT_SIGCmd(p_avrcp, AVRCP_STATUS, PDU_ID_REGISTER_NOTIFICATION, pdu_para, sizeof(pdu_para));
    } else
    {
        return AVRCP_INVALID_PARA;
    }
}



//static void KeyHoldDetermineTimeout(xTimerHandle timer) //not auto reload
//{
//    uint8_t op_id = pAvrcpProfile->key_hold_determine_key == AVRCP_KEY_RIGHT_ARROW ? AVRCP_OP_ID_FF : AVRCP_OP_ID_REWIND;
//    pAvrcpProfile->key_hold_determine_key = (eAvrcpKey)0;
//
//    PendPassthroughCmdAction(op_id, true);
//    printf("KeyHoldDetermineTimeout:send key[%x]\r\n", op_id);
//}

//static bool remoteSupportEvent(uint8_t event_id)
//{
//    return (pAvrcpInst->remoteEventsSupported & ((uint16_t)0x1 << event_id)) != 0 ? true : false;
//}
static void Handle_VendorBT_SIGResponse(TAvrcpInst *p_avrcp, uint8_t *p_pdu, uint16_t pdu_length, uint8_t response)
{
    uint8_t *p_data = p_pdu;

    uint8_t pdu_id = *p_data++;
    uint8_t packet_type = *p_data++ & 0x3;
    short para_length = NETCHAR2SHORT(p_data);
    p_data += 2;

    //recombine pdus(<=512 - 10) to large pdu
    if (packet_type == AVRCP_PDU_NON_FRAGMENTED)
    {
        if (p_avrcp->recombine.buf != NULL) //last recombination not completed, abort it
        {
            osMemoryFree(p_avrcp->recombine.buf);
            memset(&p_avrcp->recombine, 0, sizeof(p_avrcp->recombine));
        }
    } else //if START,CONTINUE,END
    {
        if (packet_type == AVRCP_PDU_START)
        {
            { //init recombine struct
                if (p_avrcp->recombine.buf == NULL)
                {
                    p_avrcp->recombine.buf = osMemoryAllocate(RAM_TYPE_DATA_OFF, AVRCP_MAX_CONTINUE_PDU_LENGTH);
                    if (p_avrcp->recombine.buf == NULL)
                    {
                        SendVendorBT_SIGCmd(p_avrcp, AVRCP_CONTROL, PDU_ID_ABORT_CONTINUE_RSP, &pdu_id, 1); //send immediately, not PendCmdAction()
                        DBG_BUFFER(MODULE_PROFILE, LEVEL_ERROR, "avrcp continuation aborted: malloc fail", 0);
                        ; //send err message to app?
                        return;
                    }
                } else //else :last recombination not completed, retain buf
                {
//                  printf("old recombine aborted\r\n");
                }
                p_avrcp->recombine.write_index = 0;
            }
        }
        if (p_avrcp->recombine.buf == NULL /*recv CONTINUE/END without START*/)
        {
            if (packet_type != AVRCP_PDU_END) SendVendorBT_SIGCmd(p_avrcp, AVRCP_CONTROL, PDU_ID_ABORT_CONTINUE_RSP, &pdu_id, 1); //
            DBG_BUFFER(MODULE_PROFILE, LEVEL_ERROR, "avrcp recv CONTINUE/END without START", 0);
            return;
        }
        if (p_avrcp->recombine.write_index + para_length > AVRCP_MAX_CONTINUE_PDU_LENGTH /*would exceed buf[]*/)
        {
            osMemoryFree(p_avrcp->recombine.buf);
            memset(&p_avrcp->recombine, 0, sizeof(p_avrcp->recombine));
            if (packet_type != AVRCP_PDU_END) SendVendorBT_SIGCmd(p_avrcp, AVRCP_CONTROL, PDU_ID_ABORT_CONTINUE_RSP, &pdu_id, 1);
            DBG_BUFFER(MODULE_PROFILE, LEVEL_ERROR, "avrcp continuation aborted: pdu too large", 0);
            return;
        }
        memcpy(p_avrcp->recombine.buf + p_avrcp->recombine.write_index, p_data, para_length);
        p_avrcp->recombine.write_index += para_length;
        if (packet_type != AVRCP_PDU_END)
        {
            SendVendorBT_SIGCmd(p_avrcp, AVRCP_CONTROL, PDU_ID_REQUEST_CONTINUE_RSP, &pdu_id, 1); //send immediately, not PendCmdAction()
            return;
        } else
        {
            p_data = p_avrcp->recombine.buf;
            para_length = p_avrcp->recombine.write_index;
            //fall through
        }
    }

    if (packet_type == AVRCP_PDU_END)
    {
        DBG_BUFFER(MODULE_PROFILE, LEVEL_INFO, "avrcp rx combined pdu: len=%d", 1, p_avrcp->recombine.write_index);
    }
    switch (pdu_id) //parse pdu (maybe large)
    {
    case PDU_ID_GET_CAPABILITIES:
        {
            TRspCapability tmp;
            if (response == AVRCP_IMPLEMENTED)
            {
                tmp.state = AVRCP_RSP_STATE_SUCCESS;
                tmp.capability_id = *p_data++;
                tmp.capability_count = *p_data++;
                tmp.buf = p_data;
            } else
            {
                tmp.state = AVRCP_RSP_STATE_FAIL;
            }
            pAvrcpProfile->app_message_handler(p_avrcp->remote_bd, AVRCP_US_MESSAGE_RSP_GET_CAPABILITY, &tmp);
        }
        break;
    case PDU_ID_GET_ELEMENT_ATTRIBUTES:
        {
            TRspGetElementAttributes tmp;
            if (response == AVRCP_IMPLEMENTED)
            {
                tmp.state = AVRCP_RSP_STATE_SUCCESS;
                tmp.number_of_attributes = *p_data++;
                if (tmp.number_of_attributes > 7) //trunck num to 7
                {
                    tmp.number_of_attributes = 7;
                }
                for (int i = 0; i < tmp.number_of_attributes; i++)
                {
                    tmp.attr[i].attribute_id      = NETCHAR2LONG(p_data); p_data += 4;
                    tmp.attr[i].character_set_id  = NETCHAR2SHORT(p_data); p_data += 2;
                    tmp.attr[i].length            = NETCHAR2SHORT(p_data); p_data += 2;
                    tmp.attr[i].buf               = p_data;
                    p_data += tmp.attr[i].length;
                }
            } else
            {
                tmp.state = AVRCP_RSP_STATE_FAIL;
            }
            pAvrcpProfile->app_message_handler(p_avrcp->remote_bd, AVRCP_US_MESSAGE_RSP_GET_ELEMENT_ATTR, &tmp);
        }
        break;
    case PDU_ID_GET_PLAY_STATUS:
        {
            TRspGetPlayStatus tmp;
            if (response == AVRCP_IMPLEMENTED)
            {
                tmp.state = AVRCP_RSP_STATE_SUCCESS;
                tmp.length_ms = NETCHAR2LONG(p_data); p_data += 4;
                tmp.position_ms = NETCHAR2LONG(p_data); p_data += 4;
                tmp.play_status = *p_data++;
            } else
            {
                tmp.state = AVRCP_RSP_STATE_FAIL;
            }
            pAvrcpProfile->app_message_handler(p_avrcp->remote_bd, AVRCP_US_MESSAGE_RSP_GET_PLAYSTATUS, &tmp);
        }
        break;
    case PDU_ID_REGISTER_NOTIFICATION:
        if (response == AVRCP_INTERIM)
        {
            TRspRegisterNotification tmp;
            tmp.state = AVRCP_RSP_STATE_SUCCESS;
            tmp.event_id = *p_data++;
            switch (tmp.event_id)
            {
            case EVENT_PLAYBACK_STATUS_CHANGED:
                tmp.u.play_status = *p_data++;
                break;
            case EVENT_TRACK_CHANGED:
                ;
                unsigned long long track_id;
                *((uint32_t *)&track_id + 1) = NETCHAR2LONG(p_data); p_data += 4;
                *(uint32_t *)&track_id = NETCHAR2LONG(p_data); p_data += 4;

                tmp.u.p_track_id = &track_id;

                break;
            default:
                break;
            }
            pAvrcpProfile->app_message_handler(p_avrcp->remote_bd, AVRCP_US_MESSAGE_RSP_REGISTER_NOTIFICATION, &tmp);
        } else if (response == AVRCP_CHANGED)
        {
            TNotificationChanged tmp;
            tmp.event_id = *p_data++;
            switch (tmp.event_id)
            {
            case EVENT_PLAYBACK_STATUS_CHANGED:
                tmp.u.play_status = *p_data++;
                break;
            case EVENT_TRACK_CHANGED:
                ;
                unsigned long long track_id;
                *((uint32_t *)&track_id + 1) = NETCHAR2LONG(p_data); p_data += 4;
                *(uint32_t *)&track_id = NETCHAR2LONG(p_data); p_data += 4;

                tmp.u.p_track_id = &track_id;

                break;
            default:
                break;
            }
            pAvrcpProfile->app_message_handler(p_avrcp->remote_bd, AVRCP_US_MESSAGE_NOTIFICATION_CHANGED, &tmp);
        } else
        {
            TRspRegisterNotification tmp;
            tmp.state = AVRCP_RSP_STATE_FAIL;
            pAvrcpProfile->app_message_handler(p_avrcp->remote_bd, AVRCP_US_MESSAGE_RSP_REGISTER_NOTIFICATION, &tmp);
        }
        break;
    case PDU_ID_REQUEST_CONTINUE_RSP: //only when TG response err
        break;
    case PDU_ID_ABORT_CONTINUE_RSP:
        //do nothing
        break;
    default:
        break;
    }
    if (packet_type == AVRCP_PDU_END) //normal clean
    {
        osMemoryFree(p_avrcp->recombine.buf);
        memset(&p_avrcp->recombine, 0, sizeof(p_avrcp->recombine));
    }
    return;
}
static void Handle_VendorDependentResponse(TAvrcpInst *p_avrcp, uint8_t *p_oprand, uint16_t length, uint8_t response)
{
    uint8_t *p_data = p_oprand;

    int company_id = (uint32_t)p_data[0] << 16 | (uint32_t)p_data[1] << 8 | p_data[2];
    p_data += 3;
    length -= 3;
    if (company_id == COMPANY_BT_SIG)
    {
        Handle_VendorBT_SIGResponse(p_avrcp, p_data, length, response);
    } else if (pAvrcpProfile->vendor_rsp_handler != NULL)
    {
        pAvrcpProfile->vendor_rsp_handler(p_avrcp->remote_bd, company_id, p_data, length, response);
    } else
    {
        DBG_BUFFER(MODULE_PROFILE, LEVEL_WARN, "avrcp rsp dropped: company_id=%x", 1, company_id);
    }
}
static void HandleResponse(TAvrcpInst *p_avrcp, uint8_t *p_data, uint16_t length)
{
    uint8_t response = *p_data++ & 0x0f;
    p_data++;
//  subUnitType = *p_data & 0xF8;//unused
//  subUnitID = *p_data & 0x07;//unused
    uint8_t op_code = *p_data++;
    length -= 3;
    switch (op_code)
    {
    case OPCODE_UNIT_INFO:
        {
            TRspUnitInfo tmp;
            tmp.state = AVRCP_RSP_STATE_FAIL;
            if (response == AVRCP_IMPLEMENTED)
            {
                if (p_data[0] == 0x07) //const byte 0x07
                {
                    tmp.state = AVRCP_RSP_STATE_SUCCESS;
                    tmp.sub_unit_type = p_data[1] >> 3;
                    tmp.sub_unit_id = p_data[1] & 0x07;
                    tmp.company_id = (uint32_t)p_data[2] << 16 | (uint32_t)p_data[3] << 8 | p_data[4];
                }
            } // else //does not meet spec
            pAvrcpProfile->app_message_handler(p_avrcp->remote_bd, AVRCP_US_MESSAGE_RSP_UNIT_INFO, &tmp);
        }
        break;
    case OPCODE_PASS_THROUGH:
        {
            TRspPassthrough tmp;
            if (response == AVRCP_ACCEPTED)
            {
                uint8_t op_id           = p_data[0] & 0x7f;
                uint8_t state_flag      = p_data[0] >> 7;

                tmp.state = AVRCP_RSP_STATE_SUCCESS;
                tmp.key = (AvrcpKey)op_id;
                tmp.pressed = state_flag == 0 ? true : false;
            } else
            {
                tmp.state = AVRCP_RSP_STATE_FAIL;
            }
            pAvrcpProfile->app_message_handler(p_avrcp->remote_bd, AVRCP_US_MESSAGE_RSP_PASSTHROUGH, &tmp);
        }
        break;
    case OPCODE_VENDOR_DEPENDENT:
        Handle_VendorDependentResponse(p_avrcp, p_data, length, response);
        break;
//  case OPCODE_SUB_UNIT_INFO: //useless temply
//      {
//          uint8_t *opRand = pL2cDataInd->buf + pL2cDataInd->dataOffset;
//          //printf("OPCODE_UNIT_INFO rsp %x:", response);
//
//          uint8_t subUnitTyte, subUnitId;
//          int company_id = 0;
//          if (response == AVRCP_IMPLEMENTED)
//          {
//              if (opRand[0] == 0x07) //page = 0 | no extension (7)
//              {
//                  int i;
//                  for (i = 1; i < 5 && opRand[i] != 0xff; i++)
//                  {
//                      subUnitTyte = opRand[1] >> 3;
//                      printf("subUnitTyte=0x%x\r\n", subUnitTyte);
//                      subUnitId = opRand[1] & 0x07;
//                      printf("subUnitId= 0~%d\r\n", subUnitId);
//                  }
//              } else
//              {
//                  printf("Note: OPCODE_SUB_UNIT_INFO rsp extended\r\n");
//              }
//          } else
//          {
//              printf("Note: avrcp rsp[%x]\r\n", response);
//          }
//      }
//      break;

    default:
        break;
    }


    return;
}
static void Handle_VendorBT_SIGCommand(TAvrcpInst *p_avrcp, uint8_t *p_pdu, uint16_t length, uint8_t ctype, uint8_t transact)
{
    uint8_t *p_data = p_pdu;

    uint8_t pdu_id = *p_data++;
    uint8_t packet_type = *p_data++ & 0x3;
    short para_length = NETCHAR2SHORT(p_data);
    p_data += 2;
    if (packet_type != AVRCP_PDU_NON_FRAGMENTED)
    {
        DBG_BUFFER(MODULE_PROFILE, LEVEL_ERROR, "avrcp rx cmd pdu dropped: fragmented pdu", 0);
        return;
    }
    switch (ctype)
    {
    case AVRCP_CONTROL:
        switch (pdu_id)
        {
        case PDU_ID_SET_ABSOLUTEVOLUME:
            if (pAvrcpProfile->role_amplifier_support)
            {
                uint8_t volume = 0x7f & *p_data; //1bit RFA
                pAvrcpProfile->app_message_handler(p_avrcp->remote_bd, AVRCP_US_MESSAGE_CMD_ABS_VOL, &volume);

                SendVendorBT_SIGResponse(p_avrcp, transact, AVRCP_ACCEPTED, pdu_id, &volume, 1);
            } else
            {
                SendVendorBT_SIGResponse(p_avrcp, transact, AVRCP_NOT_IMPLEMENTED, pdu_id, p_data, para_length);
            }
            break;
        default:
            SendVendorBT_SIGResponse(p_avrcp, transact, AVRCP_NOT_IMPLEMENTED, pdu_id, p_data, para_length);
            DBG_BUFFER(MODULE_PROFILE, LEVEL_WARN, "avrcp rx unsupported vendor cmd,CONTROL pdu id[%x]", 1, pdu_id);
            break;
        }
        break;
    case AVRCP_STATUS:
        switch (pdu_id)
        {
        case PDU_ID_GET_CAPABILITIES:
            {
                uint8_t capability_id = *p_data;
                switch (capability_id)
                {
                case CAPABILITY_ID_COMPANY_ID:
                    {
                        if (pAvrcpProfile->local_company_id == COMPANY_INVALID) //no specific vendor
                        {
                            uint8_t para[] = {
                                capability_id,
                                1/* company count*/,
                                (uint8_t)(COMPANY_BT_SIG >> 16),
                                (uint8_t)(COMPANY_BT_SIG >> 8),
                                (uint8_t)COMPANY_BT_SIG };
                            SendVendorBT_SIGResponse(p_avrcp, transact, AVRCP_IMPLEMENTED, pdu_id, para, sizeof(para));
                        } else
                        {
                            uint32_t company_id = pAvrcpProfile->local_company_id;
                            uint8_t para[] = {
                                capability_id,
                                2/* company count*/,
                                (uint8_t)(COMPANY_BT_SIG >> 16),
                                (uint8_t)(COMPANY_BT_SIG >> 8),
                                (uint8_t)COMPANY_BT_SIG,
                                (uint8_t)(company_id >> 16),
                                (uint8_t)(company_id >> 8),
                                (uint8_t)company_id };
                            SendVendorBT_SIGResponse(p_avrcp, transact, AVRCP_IMPLEMENTED, pdu_id, para, sizeof(para));
                        }

                    }
                    break;
                case CAPABILITY_ID_EVENTS_SUPPORTED:
                    if (pAvrcpProfile->role_amplifier_support)
                    {
                        uint8_t para[] = {
                            capability_id,
                            1/* capa count*/,
                            EVENT_VOLUME_CHANGED };
                        SendVendorBT_SIGResponse(p_avrcp, transact, AVRCP_IMPLEMENTED, pdu_id, para, sizeof(para));
                    } else
                    {
                        uint8_t para[] = {
                            capability_id,
                            0/* capa count*/ };
                        SendVendorBT_SIGResponse(p_avrcp, transact, AVRCP_IMPLEMENTED, pdu_id, para, sizeof(para));
                    }
                    break; 
                default:
                    SendVendorBT_SIGResponse(p_avrcp, transact, AVRCP_NOT_IMPLEMENTED, pdu_id, p_data, para_length);
                    break;
                }
            }
            break;
        default:
            SendVendorBT_SIGResponse(p_avrcp, transact, AVRCP_NOT_IMPLEMENTED, pdu_id, p_data, para_length);
            DBG_BUFFER(MODULE_PROFILE, LEVEL_WARN, "avrcp rx unsupported vendor cmd,STATUS pdu id[%x]", 1, pdu_id);
            break;
        }
        break;
    case AVRCP_NOTIFY:
        switch (pdu_id)
        {
        case PDU_ID_REGISTER_NOTIFICATION:
            {
                //      handleRegisterNotificationCommand(p_data, response);
                uint8_t event_id = *p_data;
                switch (event_id)
                {
                case EVENT_VOLUME_CHANGED:
                    if (pAvrcpProfile->role_amplifier_support)
                    {
                        p_avrcp->vol_change_pending_transact = transact;
                        uint8_t    ret_volume; //for app to write the app volume
                        pAvrcpProfile->app_message_handler(p_avrcp->remote_bd, AVRCP_US_MESSAGE_CMD_REGISTER_VOL_CHANGE, &ret_volume);
                        uint8_t para[] = {
                            EVENT_VOLUME_CHANGED,
                            ret_volume
                        };
                        SendVendorBT_SIGResponse(p_avrcp, transact, AVRCP_INTERIM, pdu_id, para, sizeof(para));
                    } else
                    {
                        SendVendorBT_SIGResponse(p_avrcp, transact, AVRCP_NOT_IMPLEMENTED, pdu_id, p_data, para_length);
                    }
                    break;
                default:
                    SendVendorBT_SIGResponse(p_avrcp, transact, AVRCP_NOT_IMPLEMENTED, pdu_id, p_data, para_length);
                    break;
                }
            }
            break;
        default:
            SendVendorBT_SIGResponse(p_avrcp, transact, AVRCP_NOT_IMPLEMENTED, pdu_id, p_data, para_length);
            DBG_BUFFER(MODULE_PROFILE, LEVEL_WARN, "avrcp rx unsupported vendor cmd,NOTIFY pdu id[%x]", 1, pdu_id);
            break;
        }
        break;
    default: //won't happen
        break;
    }

    return;
}

static void Handle_VendorDependentCommand(TAvrcpInst *p_avrcp, uint8_t *p_oprand, uint16_t length, uint8_t ctype, uint8_t transact)
{
    uint8_t *p_data = p_oprand;

    int company_id = (uint32_t)p_data[0] << 16 | (uint32_t)p_data[1] << 8 | p_data[2];
    p_data += 3;
    length -= 3;
    if (company_id == COMPANY_BT_SIG)
    {
        Handle_VendorBT_SIGCommand(p_avrcp, p_data, length, ctype, transact);
    } else if (pAvrcpProfile->vendor_cmd_handler != NULL)
    {
        pAvrcpProfile->vendor_cmd_handler(p_avrcp->remote_bd, company_id, p_data, length, ctype, transact);
    } else
    {
        p_data -= 3;
        length += 3;
        *p_data = (*p_data & 0xf0) | (AVRCP_NOT_IMPLEMENTED & 0x0f);
        avctpSendData(p_avrcp->cid, p_data, length, transact, AVCTP_MESSAGE_TYPE_RESPONSE);

        DBG_BUFFER(MODULE_PROFILE, LEVEL_WARN, "avrcp rx vendor cmd dropped: company id=%x", 1, company_id);
    }
}

static void HandleCommad(TAvrcpInst *p_avrcp, uint8_t *p_data, uint16_t length, uint8_t transact)
{

    uint8_t ctype = *p_data++ & 0x0f;
    uint8_t sub_unit_type_and_id = *p_data++;
//  uint8_t subUnitID = *p_data & 0x07;
    uint8_t op_code = *p_data++;
    length -= 3;

    uint8_t response = 0; //0-no rsp; else rsp(response)
    if (sub_unit_type_and_id == 0xff || sub_unit_type_and_id == (AVRCP_PANEL << 3 | 0))
    {
        switch (ctype)
        {
        case AVRCP_CONTROL:
            {
                switch (op_code)
                {
                case OPCODE_PASS_THROUGH:
                    if (pAvrcpProfile->role_amplifier_support)
                    {
                        uint8_t state = *p_data >> 7;
                        uint8_t op_id = *p_data & 0x7f;
                        switch (op_id)
                        {
                        case AVRCP_OP_ID_VOLUP:
                            if (state == 0) //pressed
                            {
                                pAvrcpProfile->app_message_handler(p_avrcp->remote_bd, AVRCP_US_MESSAGE_CMD_VOL_UP, NULL);
                            } //ignore button release
                            response = AVRCP_ACCEPTED;
                            break;
                        case AVRCP_OP_ID_VOLDOWN:
                            if (state == 0) //pressed
                            {
                                pAvrcpProfile->app_message_handler(p_avrcp->remote_bd, AVRCP_US_MESSAGE_CMD_VOL_DOWN, NULL);
                            } //ignore button release
                            response = AVRCP_ACCEPTED;
                            break;
                        default:
                            response = AVRCP_NOT_IMPLEMENTED;
                            break;
                        }
                    } else
                    {
                        response = AVRCP_NOT_IMPLEMENTED;
                    }
                    break;
                case OPCODE_VENDOR_DEPENDENT:
                    Handle_VendorDependentCommand(p_avrcp, p_data, length, AVRCP_CONTROL, transact);
                    break;
                default: //6.3.3)
                    response = AVRCP_NOT_IMPLEMENTED;
                    break;
                }
            }
            break;
        case AVRCP_STATUS:
            {
                switch (op_code)
                {
                case OPCODE_UNIT_INFO:
                    p_data -= 3;
                    length += 3;
                    p_data[0] = (*p_data & 0xf0) | (AVRCP_IMPLEMENTED & 0x0f);
                    p_data[3] = 0x07; //const byte 0x07
                    p_data[4] = AVRCP_PANEL << 3 | 0;
                    {
                        uint32_t company_id = pAvrcpProfile->local_company_id;
                        p_data[5] = (uint8_t)(company_id >> 16);
                        p_data[6] = (uint8_t)(company_id >> 8);
                        p_data[7] = (uint8_t)company_id;
                    }
                    avctpSendData(p_avrcp->cid, p_data, length, transact, AVCTP_MESSAGE_TYPE_RESPONSE);
                    break;
                case OPCODE_SUB_UNIT_INFO:
                    p_data -= 3;
                    length += 3;
                    p_data[0] = (*p_data & 0xf0) | (AVRCP_IMPLEMENTED & 0x0f);
                    p_data[3] = 0x07; //page = 0 | no extension (7)
                    p_data[4] = AVRCP_PANEL << 3 | 0; //sub unit type | max subunit id
                    p_data[5] = 0xff;
                    p_data[6] = 0xff;
                    p_data[7] = 0xff;
                    avctpSendData(p_avrcp->cid, p_data, length, transact, AVCTP_MESSAGE_TYPE_RESPONSE);
                    break;
                case OPCODE_VENDOR_DEPENDENT:
                    Handle_VendorDependentCommand(p_avrcp, p_data, length, AVRCP_STATUS, transact);
                    break;
                default:
                    response = AVRCP_NOT_IMPLEMENTED;
                }
            }
            break;
        case AVRCP_NOTIFY:
            {
                switch (op_code)
                {
                case OPCODE_VENDOR_DEPENDENT:
                    Handle_VendorDependentCommand(p_avrcp, p_data, length, AVRCP_NOTIFY, transact);
                    break;
                default:
                    response = AVRCP_NOT_IMPLEMENTED;
                    break;
                }
            }
            break;
        case AVRCP_SPECIFIC_INQUIRY:
            response = AVRCP_NOT_IMPLEMENTED;
            break;
        case AVRCP_GENERAL_INQUIRY:
            response = AVRCP_NOT_IMPLEMENTED;
            break;
        default: //6.3.1), ignore
            break;
        }
    } else //6.3.2)
    {
        response = AVRCP_NOT_IMPLEMENTED;
    }
    if (response != 0)
    {
        p_data -= 3;
        length += 3;
        *p_data = (*p_data & 0xf0) | (response & 0x0f);
        avctpSendData(p_avrcp->cid, p_data, length, transact, AVCTP_MESSAGE_TYPE_RESPONSE);
    }
    return;
}

static void DisableAvrcp(TAvrcpInst *p_avrcp)
{
    if (p_avrcp != NULL)
    {
        //deinit avrcp
        p_avrcp->cid = 0;
        memset(p_avrcp->remote_bd, 0, BLUE_API_BD_SIZE);
        osStopTimer(&p_avrcp->wait_response_timer);
//      xTimerStop(p_avrcp->wait_response_timer, 0); /* stop timer potentially running */
        if (p_avrcp->recombine.buf != NULL) //recombination not completed, abort it
        {
            osMemoryFree(p_avrcp->recombine.buf);
            p_avrcp->recombine.buf = NULL;
            p_avrcp->recombine.write_index = 0;
        }
    }
}


void avrcp_HandleDataInd(uint16_t cid, uint8_t transact, uint8_t cr_type, uint8_t *p_data, uint16_t length)
{
    TAvrcpInst *p_avrcp = GetAvrcpInstByCid(cid);
    if (p_avrcp != NULL)
    {
        if (cr_type == AVCTP_MESSAGE_TYPE_COMMAND)
        {
            if (pAvrcpProfile->role_target_support)
            {
                HandleCommad(p_avrcp, p_data, length, transact);
            } else
            {
                //ignore
            }
        } else //AVCTP_MESSAGE_TYPE_RESPONSE
        {
            if (p_avrcp->cmd_credits == 0 //ds cmd is blocked (to exclude repeated rsp with the same label, eg.AVRCP_CHANGED)
                && transact == p_avrcp->transact_label) //to exclude repeated rsp not for last sent cmd, eg.AVRCP_CHANGED)
            {
                p_avrcp->cmd_credits = 1;
                osStopTimer(&p_avrcp->wait_response_timer);
//              xTimerStop(p_avrcp->wait_response_timer, 0);
//          printf("\r\n%d\r\n", xTaskGetTickCount() - lastCmdTick);
            }
            //max AV/C frame size = 512bytes
            HandleResponse(p_avrcp, p_data, length);
        }
    }
}


bool avrcp_SdpRegister(bool role_target_supported)
{

    {
        uint16_t length = 0;
        void *pbuffer = NULL;

        length = legacy_SDPRecordLength("< I<U> I<<UI><UI>> I<U> I<III> I<<UI>> IS II>",
                                        SDP_ATTR_SERVICECLASSIDLIST, UUID_AVREMOTECONTROL,
                                        SDP_ATTR_PROTOCOLDESCRIPTORLIST, UUID_L2CAP, PSM_AVCTP, UUID_AVCTP, 0x0104 /* Protocol Version 1.0 */,
                                        SDP_ATTR_BROWSEGROUPLIST, UUID_PUBLIC_BROWSE_GROUP,
                                        SDP_ATTR_LANGUAGEBASEATTRIBUTEIDLIST, SDP_LANGUAGE_ENGLISH, SDP_CHARCODE_UTF8, SDP_BASE_LANG_OFFSET,
                                        SDP_ATTR_BLUETOOTHPROFILEDESCRIPTORLIST, UUID_AVREMOTECONTROL, 0x0106 /* Version Number 1.0 */,
                                        SDP_ATTR_SERVICENAME + SDP_BASE_LANG_OFFSET, "avrcp ct" /* (O) English Servicename */,
                                        SDP_ATTR_SUPPORTEDFEATURES, 0x0001 // 0x0001 /* Category 1 Player / Recorder */
                                       );

        //DBG_BUFFER(MODULE_APP, LEVEL_INFO, "TestProfileInit: SDP buffer length cal is %d", 1, length);

        if (length)
        {
            pbuffer = osMemoryAllocate(RAM_TYPE_DATA_ON, length);
            length = legacy_SDPCreateDes(pbuffer,
                                         "< I<U> I<<UI><UI>> I<U> I<III> I<<UI>> IS II>",
                                         SDP_ATTR_SERVICECLASSIDLIST, UUID_AVREMOTECONTROL,
                                         SDP_ATTR_PROTOCOLDESCRIPTORLIST, UUID_L2CAP, PSM_AVCTP, UUID_AVCTP, 0x0104 /* Protocol Version 1.0 */,
                                         SDP_ATTR_BROWSEGROUPLIST, UUID_PUBLIC_BROWSE_GROUP,
                                         SDP_ATTR_LANGUAGEBASEATTRIBUTEIDLIST, SDP_LANGUAGE_ENGLISH, SDP_CHARCODE_UTF8, SDP_BASE_LANG_OFFSET,
                                         SDP_ATTR_BLUETOOTHPROFILEDESCRIPTORLIST, UUID_AVREMOTECONTROL, 0x0106 /* Version Number 1.0 */,
                                         SDP_ATTR_SERVICENAME + SDP_BASE_LANG_OFFSET, "avrcp ct" /* (O) English Servicename */,
                                         SDP_ATTR_SUPPORTEDFEATURES, 0x0001 // 0x0001 /* Category 1 Player / Recorder */
                                        );

            //DBG_BUFFER(MODULE_APP, LEVEL_INFO, "TestProfileInit: SDP buffer length is %d", 1, length);

            if (length)
            {
                legacy_AddSDPRecord(pbuffer, length);
            } else
            {
                osMemoryFree(pbuffer);
                return false;
            }
        }
    }
    /*avrcp TG*/
    if (role_target_supported)
    {
        uint16_t length = 0;
        void *pbuffer = NULL;

        length = legacy_SDPRecordLength("< I<U> I<<UI><UI>> I<U> I<III> I<<UI>> IS II>",
                                        SDP_ATTR_SERVICECLASSIDLIST, UUID_AVREMOTECONTROLTARGET,
                                        SDP_ATTR_PROTOCOLDESCRIPTORLIST, UUID_L2CAP, PSM_AVCTP, UUID_AVCTP, 0x0104 /* Protocol Version 1.0 */,
                                        SDP_ATTR_BROWSEGROUPLIST, UUID_PUBLIC_BROWSE_GROUP,
                                        SDP_ATTR_LANGUAGEBASEATTRIBUTEIDLIST, SDP_LANGUAGE_ENGLISH, SDP_CHARCODE_UTF8, SDP_BASE_LANG_OFFSET,
                                        SDP_ATTR_BLUETOOTHPROFILEDESCRIPTORLIST, UUID_AVREMOTECONTROL, 0x0106 /* Version Number 1.6 */,
                                        SDP_ATTR_SERVICENAME + SDP_BASE_LANG_OFFSET, "avrcp tg" /* (O) English Servicename */,
                                        SDP_ATTR_SUPPORTEDFEATURES, 0x0002 /* Category 2 amplifier */
                                       );

        //DBG_BUFFER(MODULE_APP, LEVEL_INFO, "TestProfileInit: SDP buffer length cal is %d", 1, length);

        if (length)
        {
            pbuffer = osMemoryAllocate(RAM_TYPE_DATA_ON, length);
            length = legacy_SDPCreateDes(pbuffer,
                                         "< I<U> I<<UI><UI>> I<U> I<III> I<<UI>> IS II>",
                                         SDP_ATTR_SERVICECLASSIDLIST, UUID_AVREMOTECONTROLTARGET,
                                         SDP_ATTR_PROTOCOLDESCRIPTORLIST, UUID_L2CAP, PSM_AVCTP, UUID_AVCTP, 0x0104 /* Protocol Version 1.0 */,
                                         SDP_ATTR_BROWSEGROUPLIST, UUID_PUBLIC_BROWSE_GROUP,
                                         SDP_ATTR_LANGUAGEBASEATTRIBUTEIDLIST, SDP_LANGUAGE_ENGLISH, SDP_CHARCODE_UTF8, SDP_BASE_LANG_OFFSET,
                                         SDP_ATTR_BLUETOOTHPROFILEDESCRIPTORLIST, UUID_AVREMOTECONTROL, 0x0106 /* Version Number 1.6 */,
                                         SDP_ATTR_SERVICENAME + SDP_BASE_LANG_OFFSET, "avrcp tg" /* (O) English Servicename */,
                                         SDP_ATTR_SUPPORTEDFEATURES, 0x0002 /* Category 2 amplifier */
                                        );

            //DBG_BUFFER(MODULE_APP, LEVEL_INFO, "TestProfileInit: SDP buffer length is %d", 1, length);

            if (length)
            {
                legacy_AddSDPRecord(pbuffer, length);
            } else
            {
                osMemoryFree(pbuffer);
                return false;
            }
        }
    }
    return true;
}

///**
// * @brief api to user
// *
// * @date 2015/10/13
// *
// * @param button
// * @param pressed true for pressed, false for released
// */
//bool avrcp_KeyAction(eAvrcpKey key, bool pressed)
//{
//    if (!(key >= AVRCP_KEY_PLAY && key <= AVRCP_KEY_REWIND))
//    {
//        return false;
//    }
//

////
////  if (key == AVRCP_KEY_RIGHT_ARROW || key == AVRCP_KEY_LEFT_ARROW) //op_id is not determined: if hold time > 1s, it's press and hold; else it's short press
////  {
////      /* FORWARD/FAST_FORWARD and BACKWARD/REWIND depend on press hold time */
////      if (pressed)
////      {
////          pAvrcpProfile->key_hold_determine_key = key;
////          xTimerStart(pAvrcpProfile->key_hold_determine_timer, 0);
////      } else //release
////      {
////          if (pAvrcpProfile->key_hold_determine_key != (eAvrcpKey)0) //key released before determine_timer expires, it's short press
////          {
////              xTimerStop(pAvrcpProfile->key_hold_determine_timer, 0);
////              pAvrcpProfile->key_hold_determine_key = (eAvrcpKey)0;
////
////              uint8_t op_id = key == AVRCP_KEY_RIGHT_ARROW ? AVRCP_OP_ID_FORWARD : AVRCP_OP_ID_BACKWARD;
////              PendPassthroughCmdAction(op_id, true);
////              PendPassthroughCmdAction(op_id, false);
////          } else //key released after determine_timer expires, it's press and hold then release
////          {
////              uint8_t op_id = key == AVRCP_KEY_RIGHT_ARROW ? AVRCP_OP_ID_FF : AVRCP_OP_ID_REWIND;
////              PendPassthroughCmdAction(op_id, false);
////          }
////      }
////  }
//    uint8_t op_id;
//    switch (key)
//    {
//    case AVRCP_KEY_PLAY:
//        op_id = AVRCP_OP_ID_PLAY;
//        break;
//    case AVRCP_KEY_STOP:
//        op_id = AVRCP_OP_ID_STOP;
//        break;
//    case AVRCP_KEY_PAUSE:
//        op_id = AVRCP_OP_ID_PAUSE;
//        break;
//    case AVRCP_KEY_FORWARD:
//        op_id = AVRCP_OP_ID_FORWARD;
//        break;
//    case AVRCP_KEY_BACKWARD:
//        op_id = AVRCP_OP_ID_BACKWARD;
//        break;
//    case AVRCP_KEY_REWIND:
//        op_id = AVRCP_OP_ID_REWIND;
//        break;
//    case AVRCP_KEY_FAST_FORWARD:
//        op_id = AVRCP_OP_ID_FF;
//        break;
////  case AVRCP_KEY_PLAY_PAUSE: /*PLAY/PAUSE depends on current play status*/
////      {
////          static uint8_t LastSentPlayPauseOpid = 0;
////          if (pressed)
////          {
////              if (pAvrcpProfile->pActiveInst->play_status != AVRCP_PLAYSTATUS_PLAYING)
////              {
////                  op_id = AVRCP_OP_ID_PLAY;
////
////              } else
////              {
////                  op_id = AVRCP_OP_ID_PAUSE;
////              }
////              LastSentPlayPauseOpid = op_id;
////          } else
////          {
////              if (LastSentPlayPauseOpid != 0)
////              {
////                  op_id = LastSentPlayPauseOpid;
////                  LastSentPlayPauseOpid = 0;
////              } else
////              { //press-release dont match
////                  return false;
////              }
////          }
////      }
////      break;
//    }
//    PendPassthroughCmdAction(op_id, pressed);
//
//    return true;
//}

AvrcpReturnStatus avrcp_NotifyVolumeChange(TBdAddr bd, uint8_t volume)
{
    if (volume > 0x7f)
    {
        return AVRCP_INVALID_PARA;
    }
    TAvrcpInst *p_avrcp = GetAvrcpInstByBd(bd);
    if (p_avrcp != NULL && p_avrcp->vol_change_pending_transact != 0xff)
    {
        uint8_t para[] = {
            EVENT_VOLUME_CHANGED,
            volume
        };
        SendVendorBT_SIGResponse(p_avrcp, p_avrcp->vol_change_pending_transact, AVRCP_CHANGED, PDU_ID_REGISTER_NOTIFICATION, para, sizeof(para));
        p_avrcp->vol_change_pending_transact = 0xff;
        return AVRCP_SUCCESS;
    } else
    {
        return AVRCP_INVALID_STATE;
    }
}

/**
 * @brief 
 * 
 * @date 2015/12/16
 * 
 * @param app_message_handler 
 * @param role_amplifier_support whether support volume sync
 * @param company_id company_id as avrcp Target, may be aquired 
 *                   by remote CT,COMPANY_INVALID means no
 *                   vendor info
 * @param vendor_cmd_handler if not NULL, all vendor specific 
 *                           cmd will be sent to
 *                           vendor_cmd_handler
 * @param vendor_rsp_handler if not NULL, all vendor specific 
 *                           rsp will be sent to
 *                           vendor_cmd_handler
 * @return AvrcpReturnStatus 
 */
AvrcpReturnStatus avrcp_Init(TAvrcpAppMessageHandler app_message_handler, bool role_amplifier_support,
                             uint32_t company_id, TAvrcpVendorCmdHandler vendor_cmd_handler, TAvrcpVendorRspHandler vendor_rsp_handler)
{
    if (app_message_handler == NULL || (company_id & 0xff000000) != 0)
    {
        return AVRCP_INVALID_PARA;
    }
//  if (vendor_cmd_handler != NULL && company_id == COMPANY_INVALID)
//  {
//      return false;
//  }
//  if (vendor_rsp_handler != NULL && company_id == COMPANY_INVALID)
//  {
//      return false;
//  }

    pAvrcpProfile = osMemoryAllocate(RAM_TYPE_DATA_ON, sizeof(*pAvrcpProfile));
    if (pAvrcpProfile == NULL)
    {
        return AVRCP_NO_RESOURCE;
    }
    memset(pAvrcpProfile, 0, sizeof(*pAvrcpProfile));
    for (int i = 0; i < MAX_AVRCP_LINK; i++)
    {
        pAvrcpProfile->inst[i].wait_response_timer = NULL; //osStartTimer will create timers
//      TAvrcpInst *p_avrcp = &pAvrcpProfile->inst[i];
//      p_avrcp->wait_response_timer = xTimerCreate(NULL, WAIT_RESPONSE_TIMEOUT, pdFALSE, NULL, WaitResponseTimeout);
//      if (p_avrcp->wait_response_timer == NULL)
//      {
//          DBG_BUFFER(MODULE_PROFILE, LEVEL_ERROR, "avrcp init xTimerCreate fail", 0);
//          return AVRCP_NO_RESOURCE;
//      }
    }
    pAvrcpProfile->role_amplifier_support   = role_amplifier_support;
    pAvrcpProfile->role_target_support      = role_amplifier_support || (vendor_cmd_handler != NULL);
    pAvrcpProfile->local_company_id         = company_id;
    pAvrcpProfile->app_message_handler      = app_message_handler;
    pAvrcpProfile->vendor_cmd_handler       = vendor_cmd_handler;
    pAvrcpProfile->vendor_rsp_handler       = vendor_rsp_handler;
    /**Protocol init*/
    if (false == avctp_ProtocolInit())
    {
        return AVRCP_NO_RESOURCE;
    }
    /**Sdp register*/
    if (false == avrcp_SdpRegister(pAvrcpProfile->role_target_support/*roles*/))
    {
        return AVRCP_NO_RESOURCE;
    }
    return AVRCP_SUCCESS;
}


AvrcpReturnStatus avrcp_Connect(TBdAddr bd)
{
    TAvrcpInst *p_avrcp = AllocateAvrcpInst(); //do nothing with plink, just check if there is empty link
    if (p_avrcp == NULL)
    {
        return AVRCP_NO_RESOURCE;
    }
    if (avctp_Connect(bd) == true)
    {
        memcpy(pAvrcpProfile->outcalling_bd, bd, BLUE_API_BD_SIZE);
        return AVRCP_SUCCESS;
    } else
    {
        return AVRCP_NO_RESOURCE;
    }
}

void avrcp_HandleConnectReq(PBlueAPI_L2cConInd ind)
{
    TAvrcpInst *p_avrcp = AllocateAvrcpInst(); //do nothing with plink, just check if there is empty link
    if (p_avrcp != NULL && pAvrcpProfile->incalling_cid == 0/*there no other incalling req*/)
    {
        pAvrcpProfile->incalling_cid = ind->cid;
        pAvrcpProfile->app_message_handler(ind->remote_BD, AVRCP_US_MESSAGE_CONNECT_REQ, NULL);
    } else
    {
        mpa_Sendl2cConConf(L2CAP_ERR_REFUS_NO_RESOURCE, ind->cid);
    }
}
AvrcpReturnStatus avrcp_ConnectConfirm(TBdAddr bd, bool accept)
{
    if (pAvrcpProfile->incalling_cid != 0)
    {
        if (accept)
        {
            mpa_Sendl2cConConf(L2CAP_CONNECTION_ACCEPT, pAvrcpProfile->incalling_cid);
        } else
        {
            mpa_Sendl2cConConf(L2CAP_ERR_REFUS_NO_RESOURCE, pAvrcpProfile->incalling_cid);
        }

        pAvrcpProfile->incalling_cid = 0;
        return AVRCP_SUCCESS;
    } else
    {
        return AVRCP_INVALID_STATE;
    }
}
void avrcp_HandleConnectCompleteInd(PBlueAPI_L2cConActInd ind)
{
    TAvrcpInst *p_avrcp = AllocateAvrcpInst(); //real allocate
    if (p_avrcp != NULL)
    {
        memcpy(p_avrcp->remote_bd, ind->remoteBd, BLUE_API_BD_SIZE);
        p_avrcp->cid = ind->cid;
        p_avrcp->cmd_credits = 1;
        p_avrcp->transact_label = 0;
        p_avrcp->vol_change_pending_transact = 0xff;

        pAvrcpProfile->app_message_handler(ind->remoteBd, AVRCP_US_MESSAGE_CONNECTED, NULL);
    } //else should not happen
}

AvrcpReturnStatus avrcp_DisConnect(TBdAddr bd)
{
    TAvrcpInst *p_avrcp = GetAvrcpInstByBd(bd);
    if (p_avrcp != NULL)
    {
        return avctp_Disconnect(p_avrcp->cid) ? AVRCP_SUCCESS : AVRCP_INVALID_PARA;
//      DisableAvrcp(p_avrcp);
    } else
    {
        return AVRCP_INVALID_PARA;
    }
}
void avrcp_HandleDisconnectInd(uint16_t cid)
{
    TAvrcpInst *p_avrcp = GetAvrcpInstByCid(cid); //no bd in PBlueAPI_l2cDisInd?
    if (p_avrcp != NULL) //connect -> disconnect
    {
        pAvrcpProfile->app_message_handler(p_avrcp->remote_bd, AVRCP_US_MESSAGE_DISCONNECTED, NULL);
        DisableAvrcp(p_avrcp);
    } else
    {
        TBdAddr bd = { 0 };
        if (memcmp(pAvrcpProfile->outcalling_bd, bd, BLUE_API_BD_SIZE) != 0) //outgoing connecting->disconnect, don't know bd addr
        {
            pAvrcpProfile->app_message_handler(pAvrcpProfile->outcalling_bd, AVRCP_US_MESSAGE_DISCONNECTED, NULL);
            memset(pAvrcpProfile->outcalling_bd, 0, BLUE_API_BD_SIZE);
        }
    }
}
void avrcp_HandleSecurityRegisterRsp(PBlueAPI_L2cSecurityRegisterRsp rsp){
    ;
}

void avrcp_HandleAuthorizationInd(PBlueAPI_UserAuthorizationReqInd ind){
    pAvrcpProfile->app_message_handler(ind->remote_BD, AVRCP_US_MESSAGE_AUTHORIZATION_IND, NULL);
}
//bool avrcp_SwitchActiveInst(int index)
//{
//    if (index >= 0 && index < MAX_AVRCP_LINK)
//    {
//        pAvrcpProfile->pActiveInst = &pAvrcpProfile->inst[index];
//        if (pAvrcpProfile->pActiveInst->state >= avrcpStateRemoteUnitInfoGot)
//        {
////      //if support track changed
//            PendCmdAction(pAvrcpProfile->pActiveInst, avrcp_CmdGetElementAttributes, ELEMENT_ATTRIBUTE_ALL);
//            PendCmdAction(pAvrcpProfile->pActiveInst, avrcp_CmdGetPlayStatus, 0);
//        } else
//        {
//            //do nothing
//        }
//
//        return true;
//    } else
//    {
//        return false;
//    }
//
//}
//void avrcp_DebugCmd(char rx)
//{
//    if (rx == 'w')
//    {
//        uint8_t bd[6] = { 0x7b, 0xf4, 0x10, 0x14, 0xe3, 0x24 };
//        mpaL2cSDPDiscover(bd); /*test*/
//        return;
