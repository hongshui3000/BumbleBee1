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

#include <string.h>
#include "trace.h"

#include "mesh_api.h"
#include "smgap.h"

static uint8_t gapTid = 0;

void smgap_ConfigDefaultTTLGet(uint16_t dest_addr)
{
    uint8_t * pbuffer;
    TConfigDefaultTTLGet *pConfigDefaultTTLGet;

    pbuffer = MeshDataRamPoolBufferGet();
    if (pbuffer == NULL)
    {
        return ;
    }

    pConfigDefaultTTLGet = (TConfigDefaultTTLGet *) (pbuffer + MESH_APP_PAYLOAD_OFFSET);
    pConfigDefaultTTLGet->opcode = MESH_OPCODE_CONFIG_DEFAULT_TTL_GET;
    pConfigDefaultTTLGet->tid = TRANSACTION_REL_BIT_ENABLE|gapTid;
    
    MeshAppSend(dest_addr, MESH_PRIORITY_NORMAL, pbuffer, MESH_APP_PAYLOAD_OFFSET, sizeof(TConfigDefaultTTLGet));
}

void smgap_ConfigDefaultTTLSet(uint16_t dest_addr)
{
    uint8_t * pbuffer;
    TConfigDefaultTTLSet *pConfigDefaultTTLSet;
    
    pbuffer = MeshDataRamPoolBufferGet();
    if (pbuffer == NULL)
    {
        return ;
    }

    pConfigDefaultTTLSet = (TConfigDefaultTTLSet *) (pbuffer + MESH_APP_PAYLOAD_OFFSET);
    pConfigDefaultTTLSet->opcode = MESH_OPCODE_CONFIG_DEFAULT_TTL_SET;
    pConfigDefaultTTLSet->tid = gapTid;
    
    MeshAppSend(dest_addr, MESH_PRIORITY_NORMAL, pbuffer, MESH_APP_PAYLOAD_OFFSET, sizeof(TConfigDefaultTTLSet));
}

void smgap_ConfigDefaultTTLStatus(uint16_t dest_addr, uint8_t defaultTTL)
{
    uint8_t * pbuffer;
    TConfigDefaultTTLStatus *pConfigDefaultTTLStatus;

    pbuffer = MeshDataRamPoolBufferGet();
    if (pbuffer == NULL)
    {
        return ;
    }

    pConfigDefaultTTLStatus = (TConfigDefaultTTLStatus *) (pbuffer + MESH_APP_PAYLOAD_OFFSET);
    pConfigDefaultTTLStatus->opcode = MESH_OPCODE_CONFIG_DEFAULT_TTL_STATUS;
    pConfigDefaultTTLStatus->tid = gapTid;
    pConfigDefaultTTLStatus->defaultTTL = defaultTTL;

    MeshAppSend(dest_addr, MESH_PRIORITY_NORMAL, pbuffer, MESH_APP_PAYLOAD_OFFSET, sizeof(TConfigDefaultTTLStatus));

}

void smgap_ConfigFutureGet(uint16_t dest_addr)
{
    uint8_t * pbuffer;
    TConfigFutureGet *pConfigFutureGet;

    pbuffer = MeshDataRamPoolBufferGet();
    if (pbuffer == NULL)
    {
        return ;
    }

    pConfigFutureGet = (TConfigFutureGet *) (pbuffer + MESH_APP_PAYLOAD_OFFSET);
    pConfigFutureGet->opcode = MESH_OPCODE_CONFIG_FUTURE_GET;
    pConfigFutureGet->tid = gapTid;

    
    MeshAppSend(dest_addr, MESH_PRIORITY_NORMAL, pbuffer, MESH_APP_PAYLOAD_OFFSET, sizeof(TConfigFutureGet));


}

void smgap_ConfigFutureSet(uint16_t dest_addr, uint8_t size, uint16_t interval, uint16_t window)
{
    uint8_t * pbuffer;
    TConfigFutureSet *pConfigFutureSet;

    pbuffer = MeshDataRamPoolBufferGet();
    if (pbuffer == NULL)
    {
        return ;
    }

    pConfigFutureSet = (TConfigFutureSet *) (pbuffer + MESH_APP_PAYLOAD_OFFSET);
    pConfigFutureSet->opcode = MESH_OPCODE_CONFIG_FUTURE_SET;
    pConfigFutureSet->tid = gapTid;
    pConfigFutureSet->size = size;
    pConfigFutureSet->interval = interval;
    pConfigFutureSet->window = window;

    MeshAppSend(dest_addr, MESH_PRIORITY_NORMAL, pbuffer, MESH_APP_PAYLOAD_OFFSET, sizeof(TConfigFutureSet));
}

void smgap_ConfigFutureStatus(uint16_t dest_addr, uint8_t size, uint8_t available,uint16_t interval, uint16_t window)
{
    uint8_t * pbuffer;
    TConfigFutureStatus *pConfigFutureStatus;

    pbuffer = MeshDataRamPoolBufferGet();
    if (pbuffer == NULL)
    {
        return ;
    }

    pConfigFutureStatus = (TConfigFutureStatus *) (pbuffer + MESH_APP_PAYLOAD_OFFSET);
    pConfigFutureStatus->opcode = MESH_OPCODE_CONFIG_FUTURE_STATUS;
    pConfigFutureStatus->tid = gapTid;
    pConfigFutureStatus->size = size;
    pConfigFutureStatus->available = available;
    pConfigFutureStatus->interval = interval;
    pConfigFutureStatus->window = window;

    MeshAppSend(dest_addr, MESH_PRIORITY_NORMAL, pbuffer, MESH_APP_PAYLOAD_OFFSET, sizeof(TConfigFutureStatus));

}

void smgap_ConfigRelayGet(uint16_t dest_addr)
{
    uint8_t * pbuffer;
    TConfigRelayGet *pConfigRelayGet;

    pbuffer = MeshDataRamPoolBufferGet();
    if (pbuffer == NULL)
    {
        return ;
    }

    pConfigRelayGet = (TConfigRelayGet *) (pbuffer + MESH_APP_PAYLOAD_OFFSET);
    pConfigRelayGet->opcode = MESH_OPCODE_CONFIG_RELAY_GET;
    pConfigRelayGet->tid = gapTid;


    MeshAppSend(dest_addr, MESH_PRIORITY_NORMAL, pbuffer, MESH_APP_PAYLOAD_OFFSET, sizeof(TConfigRelayGet));

}

void smgap_ConfigRelaySet(uint16_t dest_addr, uint8_t state)
{
    uint8_t * pbuffer;
    TConfigRelaySet *pConfigRelaySet;

    pbuffer = MeshDataRamPoolBufferGet();
    if (pbuffer == NULL)
    {
        return ;
    }

    pConfigRelaySet = (TConfigRelaySet *) (pbuffer + MESH_APP_PAYLOAD_OFFSET);
    pConfigRelaySet->opcode = MESH_OPCODE_CONFIG_RELAY_SET;
    pConfigRelaySet->tid = gapTid;
    pConfigRelaySet->state = state;
    
    MeshAppSend(dest_addr, MESH_PRIORITY_NORMAL, pbuffer, MESH_APP_PAYLOAD_OFFSET, sizeof(TConfigRelaySet));

}

void smgap_ConfigRelayStatus(uint16_t dest_addr, uint8_t state)
{
    uint8_t * pbuffer;
    TConfigRelayStatus *pConfigRelayStatus;

    pbuffer = MeshDataRamPoolBufferGet();
    if (pbuffer == NULL)
    {
        return ;
    }

    pConfigRelayStatus = (TConfigRelayStatus *) (pbuffer + MESH_APP_PAYLOAD_OFFSET);
    pConfigRelayStatus->opcode = MESH_OPCODE_CONFIG_RELAY_STATUS;
    pConfigRelayStatus->tid = gapTid;
    pConfigRelayStatus->state = state;
    MeshAppSend(dest_addr, MESH_PRIORITY_NORMAL, pbuffer, MESH_APP_PAYLOAD_OFFSET, sizeof(TConfigRelayStatus));

}

void smgap_GenericDeltaSetFinal(uint16_t dest_addr, uint16_t delta)
{
    uint8_t * pbuffer;
    TGenericDeltaSetFinal *pGenericDeltaSetFinal;

    pbuffer = MeshDataRamPoolBufferGet();
    if (pbuffer == NULL)
    {
        return ;
    }

    pGenericDeltaSetFinal = (TGenericDeltaSetFinal *) (pbuffer + MESH_APP_PAYLOAD_OFFSET);
    pGenericDeltaSetFinal->opcode = MESH_OPCODE_GENERIC_DELTA_SET_FINAL;
    pGenericDeltaSetFinal->tid = gapTid;
    pGenericDeltaSetFinal->delta = delta;
    
    MeshAppSend(dest_addr, MESH_PRIORITY_NORMAL, pbuffer, MESH_APP_PAYLOAD_OFFSET, sizeof(TGenericDeltaSetFinal));


}

void smgap_GenericDeltaSetUpdate(uint16_t dest_addr, uint8_t delta)
{
    uint8_t * pbuffer;
    TGenericDeltaSetUpdate *pGenericDeltaSetUpdate;

    pbuffer = MeshDataRamPoolBufferGet();
    if (pbuffer == NULL)
    {
        return ;
    }

    pGenericDeltaSetUpdate = (TGenericDeltaSetUpdate *) (pbuffer + MESH_APP_PAYLOAD_OFFSET);
    pGenericDeltaSetUpdate->opcode = MESH_OPCODE_GENERIC_DELTA_SET_UPDATE;
    pGenericDeltaSetUpdate->tid = gapTid;
    pGenericDeltaSetUpdate->delta = delta;
    
    MeshAppSend(dest_addr, MESH_PRIORITY_NORMAL, pbuffer, MESH_APP_PAYLOAD_OFFSET, sizeof(TGenericDeltaSetUpdate));
}

void smgap_GenericLevelGet(uint16_t dest_addr)
{
    uint8_t * pbuffer;
    TGenericLevelGet *pGenericLevelGet;

    pbuffer = MeshDataRamPoolBufferGet();
    if (pbuffer == NULL)
    {
        return ;
    }

    pGenericLevelGet = (TGenericLevelGet *) (pbuffer + MESH_APP_PAYLOAD_OFFSET);
    pGenericLevelGet->opcode = MESH_OPCODE_GENERIC_LEVEL_GET;
    pGenericLevelGet->tid = gapTid;

    
    MeshAppSend(dest_addr, MESH_PRIORITY_NORMAL, pbuffer, MESH_APP_PAYLOAD_OFFSET, sizeof(TGenericLevelGet));
}

void smgap_GenericLevelSet(uint16_t dest_addr, uint8_t delta)
{
    uint8_t * pbuffer;
    TGenericLevelSet *pGenericLevelSet;

    pbuffer = MeshDataRamPoolBufferGet();
    if (pbuffer == NULL)
    {
        return ;
    }

    pGenericLevelSet = (TGenericLevelSet *) (pbuffer + MESH_APP_PAYLOAD_OFFSET);
    pGenericLevelSet->opcode = MESH_OPCODE_GENERIC_LEVEL_SET_WITH_ACK;
    pGenericLevelSet->tid = gapTid;
    pGenericLevelSet->delta = delta;
    
    MeshAppSend(dest_addr, MESH_PRIORITY_NORMAL, pbuffer, MESH_APP_PAYLOAD_OFFSET, sizeof(TGenericLevelSet));
}



void smgap_GenericLevelStatus(uint16_t dest_addr, uint8_t delta)
{
    uint8_t * pbuffer;
    TGenericLevelStatus *pGenericLevelStatus;

    pbuffer = MeshDataRamPoolBufferGet();
    if (pbuffer == NULL)
    {
        return ;
    }

    pGenericLevelStatus = (TGenericLevelStatus *) (pbuffer + MESH_APP_PAYLOAD_OFFSET);
    pGenericLevelStatus->opcode = MESH_OPCODE_GENERIC_LEVEL_STATUS;
    pGenericLevelStatus->tid = gapTid;
    pGenericLevelStatus->delta = delta;
    
    MeshAppSend(dest_addr, MESH_PRIORITY_NORMAL, pbuffer, MESH_APP_PAYLOAD_OFFSET, sizeof(TGenericLevelStatus));
}

void smgap_GenericStateGet(uint16_t dest_addr)
{
    uint8_t * pbuffer;
    TGenericStateGet *pGenericStateGet;

    pbuffer = MeshDataRamPoolBufferGet();
    if (pbuffer == NULL)
    {
        return ;
    }

    pGenericStateGet = (TGenericStateGet *) (pbuffer + MESH_APP_PAYLOAD_OFFSET);
    pGenericStateGet->opcode = MESH_OPCODE_GENERIC_STATE_GET;
    pGenericStateGet->tid = gapTid;
    
    MeshAppSend(dest_addr, MESH_PRIORITY_NORMAL, pbuffer, MESH_APP_PAYLOAD_OFFSET, sizeof(TGenericStateGet));
}



void smgap_GenericStateSetWithoutAck(uint16_t dest_addr, uint8_t state)
{
    uint8_t * pbuffer;
    TGenericStateSet *pGenericStateSet;

    pbuffer = MeshDataRamPoolBufferGet();
    if (pbuffer == NULL)
    {
        return ;
    }

    pGenericStateSet = (TGenericStateSet *) (pbuffer + MESH_APP_PAYLOAD_OFFSET);
    pGenericStateSet->opcode = MESH_OPCODE_GENERIC_STATE_SET_WITHOUT_ACK;
    pGenericStateSet->tid = gapTid;
    pGenericStateSet->state = state;
    
    MeshAppSend(dest_addr, MESH_PRIORITY_NORMAL, pbuffer, MESH_APP_PAYLOAD_OFFSET, sizeof(TGenericStateSet));
}

void smgap_GenericStateSetWithAck(uint16_t dest_addr, uint8_t state)
{
    uint8_t * pbuffer;
    TGenericStateSet *pGenericStateSet;

    pbuffer = MeshDataRamPoolBufferGet();
    if (pbuffer == NULL)
    {
        return ;
    }

    pGenericStateSet = (TGenericStateSet *) (pbuffer + MESH_APP_PAYLOAD_OFFSET);
    pGenericStateSet->opcode = MESH_OPCODE_GENERIC_STATE_SET_WITH_ACK;
    pGenericStateSet->tid = gapTid;
    pGenericStateSet->state = state;
    
    MeshAppSend(dest_addr, MESH_PRIORITY_NORMAL, pbuffer, MESH_APP_PAYLOAD_OFFSET, sizeof(TGenericStateSet));
}

void smgap_GenericStateStatus(uint16_t dest_addr, uint8_t state)
{
    uint8_t * pbuffer;
    TGenericStateStatus *pGenericStateStatus;

    pbuffer = MeshDataRamPoolBufferGet();
    if (pbuffer == NULL)
    {
        return ;
    }

    pGenericStateStatus = (TGenericStateStatus *) (pbuffer + MESH_APP_PAYLOAD_OFFSET);
    pGenericStateStatus->opcode = MESH_OPCODE_GENERIC_STATE_STATUS;
    pGenericStateStatus->tid = gapTid;
    pGenericStateStatus->state = state;
    
    MeshAppSend(dest_addr, MESH_PRIORITY_NORMAL, pbuffer, MESH_APP_PAYLOAD_OFFSET, sizeof(TGenericStateStatus));
}

void smgap_LightColorTemperatureSet(uint16_t dest_addr, uint16_t colorTemp)
{
    uint8_t * pbuffer;
    TLightColorTemperatureSet *pLightColorTemperatureSet;

    pbuffer = MeshDataRamPoolBufferGet();
    if (pbuffer == NULL)
    {
        return ;
    }

    pLightColorTemperatureSet = (TLightColorTemperatureSet *) (pbuffer + MESH_APP_PAYLOAD_OFFSET);
    pLightColorTemperatureSet->opcode = MESH_OPCODE_LIGHT_COLOR_TEMPERATURE_SET;
    pLightColorTemperatureSet->tid = gapTid;
    pLightColorTemperatureSet->colorTemp = colorTemp;
    
    MeshAppSend(dest_addr, MESH_PRIORITY_NORMAL, pbuffer, MESH_APP_PAYLOAD_OFFSET, sizeof(TLightColorTemperatureSet));
}

void smgap_LightLevelTransitionTimeSet(uint16_t dest_addr, uint8_t level, uint16_t transitionTime)
{
    uint8_t * pbuffer;
    TLightLevelTransitionTimeSet *pLightLevelTransitionTimeSet;

    pbuffer = MeshDataRamPoolBufferGet();
    if (pbuffer == NULL)
    {
        return ;
    }

    pLightLevelTransitionTimeSet = (TLightLevelTransitionTimeSet *) (pbuffer + MESH_APP_PAYLOAD_OFFSET);
    pLightLevelTransitionTimeSet->opcode = MESH_OPCODE_LIGHT_LEVEL_TRANSITIONTIME_SET;
    pLightLevelTransitionTimeSet->tid = gapTid;
    pLightLevelTransitionTimeSet->level = level;
    pLightLevelTransitionTimeSet->transitionTime = transitionTime;
    
    MeshAppSend(dest_addr, MESH_PRIORITY_NORMAL, pbuffer, MESH_APP_PAYLOAD_OFFSET, sizeof(TLightLevelTransitionTimeSet));
}

void smgap_LightLevelSetUpdate(uint16_t dest_addr, uint8_t level)
{
    uint8_t * pbuffer;
    TLightLevelSetUpdate *pLightLevelSetUpdate;

    pbuffer = MeshDataRamPoolBufferGet();
    if (pbuffer == NULL)
    {
        return ;
    }

    pLightLevelSetUpdate = (TLightLevelSetUpdate *) (pbuffer + MESH_APP_PAYLOAD_OFFSET);
    pLightLevelSetUpdate->opcode = MESH_OPCODE_LIGHT_LEVEL_SET_UPDATE;
    pLightLevelSetUpdate->tid = gapTid;
    pLightLevelSetUpdate->level = level;
    
    MeshAppSend(dest_addr, MESH_PRIORITY_NORMAL, pbuffer, MESH_APP_PAYLOAD_OFFSET, sizeof(TLightLevelSetUpdate));
}


void smgap_LightLevelSet(uint16_t dest_addr, uint8_t level)
{
    uint8_t * pbuffer;
    TLightLevelSet *pLightLevelSet;

    pbuffer = MeshDataRamPoolBufferGet();
    if (pbuffer == NULL)
    {
        return ;
    }

    pLightLevelSet = (TLightLevelSet *) (pbuffer + MESH_APP_PAYLOAD_OFFSET);
    pLightLevelSet->opcode = MESH_OPCODE_LIGHT_LEVEL_SET_WITHOUT_ACK;
    pLightLevelSet->tid = gapTid;
    pLightLevelSet->level = level;
    
    MeshAppSend(dest_addr, MESH_PRIORITY_NORMAL, pbuffer, MESH_APP_PAYLOAD_OFFSET, sizeof(TLightLevelSet));
}

void smgap_LightStateGet(uint16_t dest_addr)
{
    uint8_t * pbuffer;
    TLightStateGet *pLightStateGet;

    pbuffer = MeshDataRamPoolBufferGet();
    if (pbuffer == NULL)
    {
        return ;
    }

    pLightStateGet = (TLightStateGet *) (pbuffer + MESH_APP_PAYLOAD_OFFSET);
    pLightStateGet->opcode = MESH_OPCODE_LIGHT_STATE_GET;
    pLightStateGet->tid = gapTid;
   
    MeshAppSend(dest_addr, MESH_PRIORITY_NORMAL, pbuffer, MESH_APP_PAYLOAD_OFFSET, sizeof(TLightStateGet));
}

void smgap_LightStateStatus(uint16_t dest_addr, uint8_t level)
{
    uint8_t * pbuffer;
    TLightStateStatus *pLightStateStatus;

    pbuffer = MeshDataRamPoolBufferGet();
    if (pbuffer == NULL)
    {
        return ;
    }

    pLightStateStatus = (TLightStateStatus *) (pbuffer + MESH_APP_PAYLOAD_OFFSET);
    pLightStateStatus->opcode = MESH_OPCODE_LIGHT_STATE_STATUS;
    pLightStateStatus->tid = gapTid;
    pLightStateStatus->level = level;
    
    MeshAppSend(dest_addr, MESH_PRIORITY_NORMAL, pbuffer, MESH_APP_PAYLOAD_OFFSET, sizeof(TLightStateStatus));
}

void smgap_PowerStateGet(uint16_t dest_addr)
{
    uint8_t * pbuffer;
    TPowerStateGet *pPowerStateGet;

    pbuffer = MeshDataRamPoolBufferGet();
    if (pbuffer == NULL)
    {
        return ;
    }

    pPowerStateGet = (TPowerStateGet *) (pbuffer + MESH_APP_PAYLOAD_OFFSET);
    pPowerStateGet->opcode = MESH_OPCODE_POWER_STATE_GET;
    pPowerStateGet->tid = gapTid;
    
    MeshAppSend(dest_addr, MESH_PRIORITY_NORMAL, pbuffer, MESH_APP_PAYLOAD_OFFSET, sizeof(TPowerStateGet));
}

void smgap_PowerStateSet(uint16_t dest_addr, uint8_t state)
{
    uint8_t * pbuffer;
    TPowerStateSet *pPowerStateSet;

    pbuffer = MeshDataRamPoolBufferGet();
    if (pbuffer == NULL)
    {
        return ;
    }

    pPowerStateSet = (TPowerStateSet *) (pbuffer + MESH_APP_PAYLOAD_OFFSET);
    pPowerStateSet->opcode = MESH_OPCODE_POWER_STATE_SET_WITH_ACK;
    pPowerStateSet->tid = gapTid;
    pPowerStateSet->state = state;
    
    MeshAppSend(dest_addr, MESH_PRIORITY_NORMAL, pbuffer, MESH_APP_PAYLOAD_OFFSET, sizeof(TPowerStateSet));
}



void smgap_PowerStateStatus(uint16_t dest_addr, uint8_t state)
{
    uint8_t * pbuffer;
    TPowerStateStatus *pPowerStateStatus;

    pbuffer = MeshDataRamPoolBufferGet();
    if (pbuffer == NULL)
    {
        return ;
    }

    pPowerStateStatus = (TPowerStateStatus *) (pbuffer + MESH_APP_PAYLOAD_OFFSET);
    pPowerStateStatus->opcode = MESH_OPCODE_POWER_STATE_STATUS;
    pPowerStateStatus->tid = gapTid;
    pPowerStateStatus->state = state;
    
    MeshAppSend(dest_addr, MESH_PRIORITY_NORMAL, pbuffer, MESH_APP_PAYLOAD_OFFSET, sizeof(TPowerStateStatus));
}

void smgap_SceneNumberSelect(uint16_t dest_addr, uint8_t scene)
{
    uint8_t * pbuffer;
    TSceneNumberSelect *pSceneNumberSelect;

    pbuffer = MeshDataRamPoolBufferGet();
    if (pbuffer == NULL)
    {
        return ;
    }

    pSceneNumberSelect = (TSceneNumberSelect *) (pbuffer + MESH_APP_PAYLOAD_OFFSET);
    pSceneNumberSelect->opcode = MESH_OPCODE_SCENE_NUMBER_SELECT;
    pSceneNumberSelect->tid = gapTid;
    pSceneNumberSelect->scene = scene;
    
    MeshAppSend(dest_addr, MESH_PRIORITY_NORMAL, pbuffer, MESH_APP_PAYLOAD_OFFSET, sizeof(TSceneNumberSelect));
}

void smgap_SceneNumberStatus(uint16_t dest_addr, uint8_t last, uint8_t duration)
{
    uint8_t * pbuffer;
    TSceneNumberStatus *pSceneNumberStatus;

    pbuffer = MeshDataRamPoolBufferGet();
    if (pbuffer == NULL)
    {
        return ;
    }

    pSceneNumberStatus = (TSceneNumberStatus *) (pbuffer + MESH_APP_PAYLOAD_OFFSET);
    pSceneNumberStatus->opcode = MESH_OPCODE_SCENE_NUMBER_STATUS;
    pSceneNumberStatus->last = last;
    pSceneNumberStatus->duration = duration;
    
    MeshAppSend(dest_addr, MESH_PRIORITY_NORMAL, pbuffer, MESH_APP_PAYLOAD_OFFSET, sizeof(TSceneNumberStatus));
}

void smgap_SceneNumberStore(uint16_t dest_addr, uint8_t last, uint8_t duration)
{
    /*
    TGenericDeltaSetFinal GenericDeltaSetFinal;
    GenericDeltaSetFinal.opcode = MESH_OPCODE_SCENE_NUMBER_STORE;
    GenericDeltaSetFinal.tid = gapTid;
    GenericDeltaSetFinal.delta = delta;
    
    MeshAppSend(dest_addr, MESH_PRIORITY_NORMAL, (uint8_t *) &GenericDeltaSetFinal, sizeof(GenericDeltaSetFinal));
    */
}

void smgap_SceneNumberSupported_Get(uint16_t dest_addr)
{
    uint8_t * pbuffer;
    TSceneNumberSupportedGet *pSceneNumberSupportedGet;

    pbuffer = MeshDataRamPoolBufferGet();
    if (pbuffer == NULL)
    {
        return ;
    }

    pSceneNumberSupportedGet = (TSceneNumberSupportedGet *) (pbuffer + MESH_APP_PAYLOAD_OFFSET);
    pSceneNumberSupportedGet->opcode = MESH_OPCODE_SCENE_NUMBER_SUPPORTED_GET;
    pSceneNumberSupportedGet->tid = gapTid;

    
    MeshAppSend(dest_addr, MESH_PRIORITY_NORMAL, pbuffer, MESH_APP_PAYLOAD_OFFSET, sizeof(TSceneNumberSupportedGet));

}

void smgap_SceneNumberSupportedStatus(uint16_t dest_addr, uint8_t last, uint8_t duration)
{
    uint8_t * pbuffer;
    TSceneNumberStatus *pSceneNumberStatus;

    pbuffer = MeshDataRamPoolBufferGet();
    if (pbuffer == NULL)
    {
        return ;
    }

    pSceneNumberStatus = (TSceneNumberStatus *) (pbuffer + MESH_APP_PAYLOAD_OFFSET);
    pSceneNumberStatus->opcode = MESH_OPCODE_SCENE_NUMBER_SUPPORTED_STATUS;
    pSceneNumberStatus->tid = gapTid;
    pSceneNumberStatus->last = last;
    pSceneNumberStatus->duration = duration;
    MeshAppSend(dest_addr, MESH_PRIORITY_NORMAL, pbuffer, MESH_APP_PAYLOAD_OFFSET, sizeof(TSceneNumberStatus));
}


void MeshGapReceive(uint16_t src, uint8_t* pbuffer, uint8_t len, uint8_t pri, uint8_t broadcast_flag)
{
    volatile uint8_t Error = 0;
    uint16_t opcode = LE_EXTRN2WORD(&pbuffer[0]);
    
    switch (opcode)
    {
    case MESH_OPCODE_CONFIG_DEFAULT_TTL_GET:
        {
            //len check
            if(len != sizeof(TConfigDefaultTTLGet))
            {
                break;
            }
            //parameter check
         volatile PConfigDefaultTTLGet pConfigDefaultTTLGet = (PConfigDefaultTTLGet)pbuffer;
            
        }
        break;
    
    case MESH_OPCODE_CONFIG_DEFAULT_TTL_SET:
        break;
    case MESH_OPCODE_CONFIG_DEFAULT_TTL_STATUS:
        break;
    case MESH_OPCODE_CONFIG_FUTURE_GET:
        break;
//    case MESH_OPCODE_CONFIG_FUTURE_SET:
//        break;
    case MESH_OPCODE_CONFIG_FUTURE_STATUS:
        break;
    case MESH_OPCODE_CONFIG_RELAY_GET:
        break;
    case MESH_OPCODE_CONFIG_RELAY_SET:
        break;
    case MESH_OPCODE_CONFIG_RELAY_STATUS:
        break;
    case MESH_OPCODE_GENERIC_DELTA_SET_FINAL:
        break;
    case MESH_OPCODE_GENERIC_DELTA_SET_UPDATE:
        break;
    case MESH_OPCODE_GENERIC_LEVEL_GET:
        break;
    case MESH_OPCODE_GENERIC_LEVEL_SET_WITH_ACK:
        break;
    case MESH_OPCODE_GENERIC_LEVEL_SET_WITHOUT_ACK:
        break;
    case MESH_OPCODE_GENERIC_LEVEL_STATUS:
        break;
    case MESH_OPCODE_GENERIC_STATE_GET:
        break;
    case MESH_OPCODE_GENERIC_STATE_SET_WITH_ACK:
        break;
    case MESH_OPCODE_GENERIC_STATE_SET_WITHOUT_ACK:
        break;
    case MESH_OPCODE_GENERIC_STATE_STATUS:
        break;
    case MESH_OPCODE_LIGHT_COLOR_TEMPERATURE_SET:
        break;
    case MESH_OPCODE_LIGHT_LEVEL_TRANSITIONTIME_SET:
        break;
    case MESH_OPCODE_LIGHT_LEVEL_SET_UPDATE:
        break;
    case MESH_OPCODE_LIGHT_LEVEL_SET_WITH_ACK:
        break;
    case MESH_OPCODE_LIGHT_LEVEL_SET_WITHOUT_ACK:
        break;
    case MESH_OPCODE_LIGHT_STATE_GET:
        break;
    case MESH_OPCODE_LIGHT_STATE_STATUS:
        break;
    case MESH_OPCODE_POWER_STATE_GET:
        break;
    case MESH_OPCODE_POWER_STATE_SET_WITH_ACK:
        break;
    case MESH_OPCODE_POWER_STATE_SET_WITHOUT_ACK:
        break;
    case MESH_OPCODE_POWER_STATE_STATUS:
        break;
    case MESH_OPCODE_SCENE_NUMBER_SELECT:
        break;
    case MESH_OPCODE_SCENE_NUMBER_STATUS:
        break;
    case MESH_OPCODE_SCENE_NUMBER_STORE:
        break;
    case MESH_OPCODE_SCENE_NUMBER_SUPPORTED_GET:
        break;
    case MESH_OPCODE_SCENE_NUMBER_SUPPORTED_STATUS:
        break;

    default:
        break;
	}
}



