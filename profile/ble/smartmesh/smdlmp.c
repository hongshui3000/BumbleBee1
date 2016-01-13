enum { __FILE_NUM__ = 0 };

/**
*********************************************************************************************************
*               Copyright(c) 2015, Realtek Semiconductor Corporation. All rights reserved.
**********************************************************************************************************
* @file     smdlmp.c
* @brief    smart mesh dimmable light mesh profile source file.
* @details  Interfaces to access smdlmp.
* @author    ranhui_xia
* @date      2015-08-05
* @version   v1.0
*********************************************************************************************************
*/
#include "rtl_types.h"
#include "stdint.h"
#include "stddef.h"
#include "gatt.h"
#include <string.h>
#include "trace.h"
#include "profile_api.h"
#include "mesh_api.h"
#include "smdlmp.h"
//#include "application.h"

static uint8_t MyTid = 0;
static uint8_t MyState = 0;

bool smdlp_SetLevelWithoutAck(uint16_t dest_addr, uint8_t state)
{
    uint8_t * pbuffer;
    TSetLevelWithoutAckMsg *pset_level_without_ack_msg;
    
    pbuffer = MeshDataRamPoolBufferGet();
    if (pbuffer == NULL)
    {
        return FALSE;
    }
    
    pset_level_without_ack_msg = (TSetLevelWithoutAckMsg *) (pbuffer + MESH_APP_PAYLOAD_OFFSET);
    pset_level_without_ack_msg->opcode = DIMMABLE_LIGHT_SET_LEVEL_WITHOUT_ACK;
    pset_level_without_ack_msg->state = state;
    
    return MeshAppSend(dest_addr, DIMMABLE_LIGHT_MSG_PRIORITY, pbuffer, MESH_APP_PAYLOAD_OFFSET, sizeof(TSetLevelWithoutAckMsg));
}

bool smdlp_SetLevelWithAck(uint16_t dest_addr, uint8_t state)
{
    uint8_t * pbuffer;
    TSetLevelWithAckMsg *pset_level_with_ack_msg;

    pbuffer = MeshDataRamPoolBufferGet();
    if (pbuffer == NULL)
    {
        return FALSE;
    }

    pset_level_with_ack_msg = (TSetLevelWithAckMsg *) (pbuffer + MESH_APP_PAYLOAD_OFFSET);
    pset_level_with_ack_msg->opcode = DIMMABLE_LIGHT_SET_LEVEL_WITH_ACK;
    pset_level_with_ack_msg->tid = MyTid;
    pset_level_with_ack_msg->state = state;

    //MyTid += 1;
    return MeshAppSend(dest_addr, DIMMABLE_LIGHT_MSG_PRIORITY, pbuffer, MESH_APP_PAYLOAD_OFFSET, sizeof(TSetLevelWithAckMsg));
}

bool smdlp_GetState(uint16_t dest_addr)
{
    uint8_t * pbuffer;
    TGetStateMsg *pget_state_msg;

    pbuffer = MeshDataRamPoolBufferGet();
    if (pbuffer == NULL)
    {
        return FALSE;
    }

    pget_state_msg = (TGetStateMsg *) (pbuffer + MESH_APP_PAYLOAD_OFFSET);
    pget_state_msg->opcode = DIMMABLE_LIGHT_GET_STATE;
    pget_state_msg->tid = MyTid;

    //MyTid += 1;
    return MeshAppSend(dest_addr, DIMMABLE_LIGHT_MSG_PRIORITY, pbuffer, MESH_APP_PAYLOAD_OFFSET, sizeof(TGetStateMsg));
}

bool smdlp_State(uint16_t dest_addr, uint8_t tid, uint8_t state)
{
    uint8_t * pbuffer;
    TStateMsg *pstate_msg;

    pbuffer = MeshDataRamPoolBufferGet();
    if (pbuffer == NULL)
    {
        return FALSE;
    }

    pstate_msg = (TStateMsg *) (pbuffer + MESH_APP_PAYLOAD_OFFSET);
    pstate_msg->opcode = DIMMABLE_LIGHT_STATE;
    pstate_msg->tid = tid;
    pstate_msg->state = state;

    return MeshAppSend(dest_addr, DIMMABLE_LIGHT_MSG_PRIORITY, pbuffer, MESH_APP_PAYLOAD_OFFSET, sizeof(TStateMsg));
}

void SmdlpApp_LevelNotify(uint8_t state, uint8_t msg_type);
void SmdlpApp_SetLevel(uint8_t state);
void smdlp_ReceiveMsg(uint16_t sr_addr, uint8_t *pdata, uint8_t data_len, uint16_t dst_addr)
{
    uint16_t opcode = LE_EXTRN2WORD(&pdata[0]);
    switch (opcode)
    {
        case DIMMABLE_LIGHT_SET_LEVEL_WITHOUT_ACK:
        {
            if (data_len != sizeof(TSetLevelWithoutAckMsg))
                return;
            
            TSetLevelWithoutAckMsg *pmsg = (TSetLevelWithoutAckMsg *) pdata;
            SmdlpApp_SetLevel(pmsg->state);
            break;
        }
        case DIMMABLE_LIGHT_SET_LEVEL_WITH_ACK:
        {
            if (data_len != sizeof(TSetLevelWithAckMsg))
                return;
            
            TSetLevelWithAckMsg *pmsg = (TSetLevelWithAckMsg *) pdata;
            SmdlpApp_SetLevel(pmsg->state);
            smdlp_State(sr_addr, pmsg->tid, pmsg->state);
            break;
        }
        case DIMMABLE_LIGHT_GET_STATE:
        {
            if (data_len != sizeof(TGetStateMsg))
                return;
            
            TGetStateMsg *pmsg = (TGetStateMsg *) pdata;
            smdlp_State(sr_addr, pmsg->tid, MyState);
            break;
        }
        case DIMMABLE_LIGHT_STATE:
        {
            if (data_len != sizeof(TStateMsg))
                return;
            
            TStateMsg *pmsg = (TStateMsg *) pdata;
            if (MyTid == pmsg->tid)
            {
                MyTid += 1;
                SmdlpApp_LevelNotify(pmsg->state, 0);
            }
            else
            {
                
            }
            break;
        }
        default:
            break; 
    }
}



