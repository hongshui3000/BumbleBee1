
enum
{
    __FILE_NUM__ = 0x0 };

#include "rtl_types.h"
#include "blueapi.h"
#include "trace.h"
#include "common_defs.h"

#include "os_mem.h"

#include "avctp.h"
#include <sdp_code.h>
#include <bterrcod.h>

#include "avrcp.h"

#define MAX_AVCTP_LINK              MAX_AVRCP_LINK

#define BT_L1_HCI_DS_OFFSET			4
#define AVCTP_WRITE_OFFSET    13
//(BT_L1_HCI_DS_OFFSET + ACL_HDR_LENGTH + L2CAP_HDR_LENGTH)

#define AVCTP_PACKET_TYPE_UNFRAGMENTED  0x0
#define AVCTP_PACKET_TYPE_START         0x1
#define AVCTP_PACKET_TYPE_CONTINUE      0x2
#define AVCTP_PACKET_TYPE_END           0x3

typedef enum _TAvctpState
{
    avctpStateClosed = 0,               /* no L2CAP connection  */
    avctpStateL2CConnected,             /* L2CAP connected               */
    avctpStateL2CDisconnected
} TAvctpState;
typedef struct
{
    uint8_t     *buf;
    uint16_t    write_index;
    uint8_t     number_of_packets;
    uint16_t    profile_id;
}TAvctpRecombine;

typedef struct
{
    TAvctpState     state;
    uint16_t        cid;
    uint16_t        local_us_mtu;
    uint16_t        remote_mtu;
    TAvctpRecombine recombine;
    uint16_t        ds_pool_id;               /**< downstream pool id */
}TAvctpLink;
typedef struct _TAppAvctp
{
    unsigned short  queue_id;              /**< own (input) queue     */
//  uint8_t          txOffset;
//  uint8_t 		writeOffset;
    TAvctpLink      link[MAX_AVCTP_LINK];
} TAppAvctp; //sizeof() = 100

static TAppAvctp *pAvctp = NULL;

static TAvctpLink* AllocateAvctpLink(void)
{
    if (pAvctp == NULL)
    {
        return NULL;
    }
    for (int i = 0; i < MAX_AVCTP_LINK; i++)
    {
        TAvctpLink *p_link = &(pAvctp->link[i]);
        if (p_link->state != avctpStateL2CConnected)
        {
//          printf("allocated pAppAvctp->link[%d]\r\n", i);
            return p_link;
        }
    }
    return NULL;
}
static TAvctpLink* GetActiveAvctpLinkByCid(uint16_t cid)
{
    if (pAvctp == NULL)
    {
        return NULL;
    }
    for (int i = 0; i < MAX_AVCTP_LINK; i++)
    {
        TAvctpLink *p_link = &(pAvctp->link[i]);
        if (p_link->state == avctpStateL2CConnected && p_link->cid == cid)
        {
//          printf("GetActiveAvctpLinkByCid[%d]\r\n", i);
            return p_link;
        }
    }
    return NULL;
}
bool avctp_Connect(TBdAddr remote_bd)
{
    TAvctpLink *p_link = AllocateAvctpLink(); //do nothing with plink, just check if there is empty link
    if (p_link != NULL)
    {
        mpa_Sendl2cConReq(PSM_AVCTP, UUID_AVCTP/*?*/, pAvctp->queue_id, AVCTP_L2CAP_MTU_SIZE, remote_bd);
        return true;
    }
    return false;
}
bool avctp_Disconnect(uint16_t cid)
{
    TAvctpLink *p_link = GetActiveAvctpLinkByCid(cid);
    if (p_link != NULL)
    {
        mpa_Sendl2cDiscReq(cid);
        return true;
    }
    return false;
}

/**
 * @brief 
 * 
 * @date 2015/11/6
 * 
 * @param cid
 * @param avrcp_data
 * @param avrcp_length
 * @param avrcp_data2
 * @param length2
 * @param transact
 * @param crtype
 * @return int
 */
int avctp_SendData2Buf(uint16_t cid, uint8_t *avrcp_data, short avrcp_length, uint8_t *avrcp_data2, short length2, uint8_t transact, uint8_t crtype)
{
    TAvctpLink *p_link = GetActiveAvctpLinkByCid(cid);
    if (p_link == NULL)
    {
        return -1; //invalid cid
    }
    if (3 + avrcp_length + length2 > p_link->remote_mtu) //check length, if  > mtu, fragment it
    {
        DBG_BUFFER(MODULE_PROFILE, LEVEL_ERROR, "avctp_SendData2Buf (%d)larger than remote_mtu(%d)", 2,
                   3 + avrcp_length + length2, p_link->remote_mtu);
        return -1; //invalid length
    }
    uint8_t *my_buf;
    if (blueAPI_CauseSuccess !=
            blueAPI_BufferGet(p_link->ds_pool_id,
                              AVCTP_WRITE_OFFSET + 3 + avrcp_length + length2,
                              24/*dsoffset count?what's this*/,
                              (void **)&my_buf))
    {
        //	bfStatus = blueFaceIllParameter;
        return 0; //no resouce
    }
    uint8_t *p_data = my_buf + AVCTP_WRITE_OFFSET;
    //3byte avctp header;
    *p_data++ = (transact << 4 & 0xf0) | (AVCTP_PACKET_TYPE_UNFRAGMENTED << 2 & 0x0c) | (crtype << 1 & 0x2); /* IPID (b0) is 0 */
    NETSHORT2CHAR(p_data, UUID_AVREMOTECONTROL);
    p_data += 2;
    //avrcp payload
    memcpy(p_data, avrcp_data, avrcp_length);
    p_data += avrcp_length;
    memcpy(p_data, avrcp_data2, length2);

    mpa_Sendl2cDataReq(my_buf, AVCTP_WRITE_OFFSET, p_link->cid, 3 + avrcp_length + length2);
    return 1;

}



static void HandleL2cDataInd(PBlueAPI_L2cDataInd pL2cDataInd)
{
    TAvctpLink *p_link = GetActiveAvctpLinkByCid(pL2cDataInd->cid);
    if (p_link != NULL)
    {
        uint8_t *p_data = pL2cDataInd->buf + pL2cDataInd->dataOffset;
        uint16_t length = pL2cDataInd->length;

        uint8_t crtype = (*p_data & 0x02) >> 1;
        uint8_t packet_type = (*p_data & 0x0c) >> 2;
        uint8_t transact_label = *p_data >> 4;
        p_data++;
        length--;

        //recombine avctp frames(<=L2CAP MTU) to avrcp packets(<=512 bytes)
        if (packet_type == AVCTP_PACKET_TYPE_UNFRAGMENTED)
        {
            if (p_link->recombine.buf != NULL) //last recombination not completed, abort it
            {
                osMemoryFree(p_link->recombine.buf);
                memset(&p_link->recombine, 0, sizeof(p_link->recombine));
            }
            p_link->recombine.profile_id = NETCHAR2SHORT(p_data); //recombine.profileID also stores profileID of UNFRAGMENT packets
            p_data += 2;
            length -= 2;
        } else
        {
            if (packet_type == AVCTP_PACKET_TYPE_START)
            {
                { //init recombine struct
                    if (p_link->recombine.buf == NULL)
                    {
                        p_link->recombine.buf = osMemoryAllocate(RAM_TYPE_DATA_OFF, AVRCP_MAX_PACKET_LENGTH);
                        if (p_link->recombine.buf == NULL)
                        {
                            DBG_BUFFER(MODULE_PROFILE, LEVEL_ERROR, "avctp recombine aborted: malloc fail", 0);
                            avctp_Disconnect(p_link->cid);
                            goto release_buffer_and_return;
                        }
                    } else //else :last recombination not completed, retain buf
                    {
//                      printf("old recombine aborted\r\n");
                    }
                    p_link->recombine.write_index = 0;
                }
                p_link->recombine.number_of_packets = *p_data++;
                length--;
                p_link->recombine.profile_id = NETCHAR2SHORT(p_data);
                p_data += 2;
                length -= 2;
            }
            if (p_link->recombine.buf == NULL)
            {
                DBG_BUFFER(MODULE_PROFILE, LEVEL_ERROR, "avctp recombine recv CONTINUE/END without START", 0);
                goto release_buffer_and_return;
            }
            if (p_link->recombine.number_of_packets == 0 /*recvd packets > numberOfPackets*/
                || (packet_type == AVCTP_PACKET_TYPE_END && p_link->recombine.number_of_packets > 1/*recvd packets < numberOfPackets*/)
                || p_link->recombine.write_index + length > AVRCP_MAX_PACKET_LENGTH/*would exceed buf[]*/)
            {
                osMemoryFree(p_link->recombine.buf);
                memset(&p_link->recombine, 0, sizeof(p_link->recombine));

                DBG_BUFFER(MODULE_PROFILE, LEVEL_ERROR, "avctp unexpected recombine packet", 0);
                avctp_Disconnect(p_link->cid);
                goto release_buffer_and_return;
            }
            memcpy(p_link->recombine.buf + p_link->recombine.write_index, p_data, length);
            p_link->recombine.write_index += length;
            p_link->recombine.number_of_packets--;
            if (packet_type == AVCTP_PACKET_TYPE_END)
            {
                p_data = p_link->recombine.buf;
                length = p_link->recombine.write_index;
            }
        }
        if (packet_type == AVCTP_PACKET_TYPE_UNFRAGMENTED || packet_type == AVCTP_PACKET_TYPE_END)
        {
            //to avrcp
            if (packet_type == AVCTP_PACKET_TYPE_END)
            {
                DBG_BUFFER(MODULE_PROFILE, LEVEL_INFO, "avctp rx combined packet: len=%d", 1, p_link->recombine.write_index);
            }
            if (p_link->recombine.profile_id == UUID_AVREMOTECONTROL)
            {
                avrcp_HandleDataInd(pL2cDataInd->cid, transact_label, crtype, p_data, length);
            } else //profileID != UUID_AVREMOTECONTROL
            {
                ; //send response IPID=1
            }
            if (packet_type == AVCTP_PACKET_TYPE_END) //normal clean
            {
                osMemoryFree(p_link->recombine.buf);
                memset(&p_link->recombine, 0, sizeof(p_link->recombine));
            }
        }
    }


release_buffer_and_return:
    if (/*releaseBuffer && */(pL2cDataInd->flag & DATA_CB_RELEASE))
    {
        blueAPI_BufferRelease(pL2cDataInd->buf);
    }
    return;

}


void avctp_CallBack(void *buf, ProtocolUsMsg l2c_msg)
{
//  if (l2c_msg != L2CAP_DATA_IND) printf("\t\tavCtpCallBack:%x\r\n", l2c_msg);
    switch (l2c_msg)
    {
    case L2CAP_CONNECT_IND:
        {
            PBlueAPI_L2cConInd ind = (PBlueAPI_L2cConInd)buf;
            TAvctpLink *p_link = AllocateAvctpLink(); //do nothing with plink, just check if there is empty link
            if (p_link != NULL)
            {
                avrcp_HandleConnectReq(ind);
            } else
            {
                mpa_Sendl2cConConf(L2CAP_ERR_REFUS_NO_RESOURCE, ind->cid);
            }
        }
        break;
    case L2CAP_CONNECT_RSP:
        {
            PBlueAPI_L2cConRsp ind = (PBlueAPI_L2cConRsp)buf;
//          printf("\t\tL2CAP_CONNECT_RSP: cid[%x],status[%x],bd[%x]\r\n", ind->cid, ind->status, ind->remote_BD[0]);
            if (ind->status != 0)
            {
                avrcp_HandleDisconnectInd(0xffff); //send to app (disconnected)
            }
        }
        break;
    case L2CAP_CONNECT_COMPLETE:
        {
            TAvctpLink *p_link = AllocateAvctpLink();
            if (p_link != NULL)
            {
                PBlueAPI_L2cConActInd ind = (PBlueAPI_L2cConActInd)buf;
                p_link->ds_pool_id = ind->dsPoolID;
                p_link->cid = ind->cid;
                p_link->local_us_mtu = ind->localUsMTU;
                p_link->remote_mtu = ind->remoteMTU;
                DBG_BUFFER(MODULE_PROFILE, LEVEL_INFO,
                           "avctp connect complete:cid=%x,bd=[...%x%x],localUsMTU=%d,remoteMTU=%d", 5,
                           ind->cid, ind->remoteBd[1], ind->remoteBd[0], p_link->local_us_mtu, p_link->remote_mtu);
                p_link->state = avctpStateL2CConnected;
                avrcp_HandleConnectCompleteInd((PBlueAPI_L2cConActInd)buf);
            }
        }
        break;
    case L2CAP_DATA_IND:
        HandleL2cDataInd((PBlueAPI_L2cDataInd)buf);
        break;
    case L2CAP_DISC_IND:
        {
            PBlueAPI_L2cDiscInd ind = (PBlueAPI_L2cDiscInd)buf;
            TAvctpLink *p_link = GetActiveAvctpLinkByCid(ind->cid);
            if (p_link != NULL)
            {
                p_link->state = avctpStateL2CDisconnected;
            }
            mpa_Sendl2cDiscConf(ind->cid);
            avrcp_HandleDisconnectInd(ind->cid);
        }
        break;
    case L2CAP_DISC_RSP:
        {
            PBlueAPI_L2cDiscRsp ind = (PBlueAPI_L2cDiscRsp)buf;
            TAvctpLink *p_link = GetActiveAvctpLinkByCid(ind->cid);
            if (p_link != NULL)
            {
                p_link->state = avctpStateL2CDisconnected;
//              printf("\t\tL2CAP_DISC_RSP: cid[%x],status[%x]\r\n", ind->cid, ind->status);
                avrcp_HandleDisconnectInd(ind->cid);
            }
        }
        break;
    case PROTOCOL_SECURITY_REGISTER_RSP: /**< protocol security register response */
        avrcp_HandleSecurityRegisterRsp((PBlueAPI_L2cSecurityRegisterRsp)buf); 
        break;
    case PROTOCOL_AUTHORIZATION_IND:
        avrcp_HandleAuthorizationInd((PBlueAPI_UserAuthorizationReqInd)buf);
        break;
    default:
        DBG_BUFFER(MODULE_PROFILE, LEVEL_WARN, "avctp rx unkown mpa msg=%x", 1, l2c_msg);
        break;
    }
}

bool avctp_ProtocolInit(void)
{
    pAvctp = osMemoryAllocate(RAM_TYPE_DATA_ON, sizeof(*pAvctp));
    if (pAvctp == NULL)
    {
        return false;
    }
    memset(pAvctp, 0, sizeof(*pAvctp));
    int queue_id = mpa_RegisterProtocol(PSM_AVCTP, avctp_CallBack); //reg protocol
    if (queue_id == -1)
    {
        /*get protocol queue id fail*/
        return false;
    }
    pAvctp->queue_id = queue_id;

    mpa_Sendl2cProtocolRegister(PSM_AVCTP, pAvctp->queue_id, 1);
    return true;
}
