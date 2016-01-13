/**
*********************************************************************************************************
*               Copyright(c) 2015, Realtek Semiconductor Corporation. All rights reserved.
**********************************************************************************************************
* @file     smdlmp.c
* @brief    smart mesh dimmable light mesh profile source file.
* @details  Interfaces to access smdlmp.
* @author   ranhui_xia
* @date     2015-07-22
* @version  v0.1
*********************************************************************************************************
*/

/* Define to prevent recursive inclusion */
#ifndef _SMGAP_H_
#define _SMGAP_H_

#ifdef  __cplusplus
extern "C" {
#endif      /* __cplusplus */


#define MESH_OPCODE_CONFIG_DEFAULT_TTL_GET	                           0XA023	//RELIABLE MESSAGE
#define MESH_OPCODE_CONFIG_DEFAULT_TTL_SET                             0XA024	//STATUS MESSAGE
#define MESH_OPCODE_CONFIG_DEFAULT_TTL_STATUS	                    0XA025	//STATUS /MESSAGE
#define MESH_OPCODE_CONFIG_FUTURE_GET	                                  0XA026	//RELIABLE MESSAGE
#define MESH_OPCODE_CONFIG_FUTURE_SET	                                  0XA026	//RELIABLE MESSAGE
#define MESH_OPCODE_CONFIG_FUTURE_STATUS	                           0XA027	//STATUS MESSAGE
#define MESH_OPCODE_CONFIG_RELAY_GET	                                  0XA020	//RELIABLE MESSAGE
#define MESH_OPCODE_CONFIG_RELAY_SET	                                         0XA021	//RELIABLE MESSAGE
#define MESH_OPCODE_CONFIG_RELAY_STATUS	                                  0XA022	//STATUS MESSAGE
#define MESH_OPCODE_GENERIC_DELTA_SET_FINAL	                           0XA007	//RELIABLE MESSAGE
#define MESH_OPCODE_GENERIC_DELTA_SET_UPDATE	                    0XA006	//UNRELIABLE MULTIPLE MESSAGE
#define MESH_OPCODE_GENERIC_LEVEL_GET	                                  0XA004	//RELIABLE MESSAGE
#define MESH_OPCODE_GENERIC_LEVEL_SET_WITH_ACK	                    0XA003	//RELIABLE MESSAGE
#define MESH_OPCODE_GENERIC_LEVEL_SET_WITHOUT_ACK	             0X8003	//UNRELIABLE MESSAGE
#define MESH_OPCODE_GENERIC_LEVEL_STATUS	                           0XA005	//STATUS MESSAGE
#define MESH_OPCODE_GENERIC_STATE_GET	                                  0XA001	//RELIABLE MESSAGE
#define MESH_OPCODE_GENERIC_STATE_SET_WITH_ACK	                    0XA000	//RELIABLE MESSAGE
#define MESH_OPCODE_GENERIC_STATE_SET_WITHOUT_ACK	             0X8000	//UNRELIABLE MESSAGE
#define MESH_OPCODE_GENERIC_STATE_STATUS	                           0XA002	//STATUS MESSAGE
#define MESH_OPCODE_LIGHT_COLOR_TEMPERATURE_SET	             0XA034	//UNRELIABLE MESSAGE
#define MESH_OPCODE_LIGHT_LEVEL_TRANSITIONTIME_SET	             0XA036	//RELIABLE MESSAGE
#define MESH_OPCODE_LIGHT_LEVEL_SET_UPDATE	                           0XA035	//UNRELIABLE MULTIPLE MESSAGE
#define MESH_OPCODE_LIGHT_LEVEL_SET_WITH_ACK	                    0XA031	//RELIABLE MESSAGE
#define MESH_OPCODE_LIGHT_LEVEL_SET_WITHOUT_ACK	             0X8031	//UNRELIABLE MESSAGE
#define MESH_OPCODE_LIGHT_STATE_GET	                                         0XA033	//UNRELIABLE MESSAGE
#define MESH_OPCODE_LIGHT_STATE_STATUS	                                  0XA032	//STATUS MESSAGE
#define MESH_OPCODE_POWER_STATE_GET	                                         0XA011	//RELIABLE MESSAGE
#define MESH_OPCODE_POWER_STATE_SET_WITH_ACK	                    0XA010	//RELIABLE MESSAGE
#define MESH_OPCODE_POWER_STATE_SET_WITHOUT_ACK	             0X8010	//UNRELIABLE MESSAGE
#define MESH_OPCODE_POWER_STATE_STATUS	                                  0XA012	//STATUS MESSAGE
#define MESH_OPCODE_SCENE_NUMBER_SELECT	                                  0XA043	//RELIABLE MESSAGE
#define MESH_OPCODE_SCENE_NUMBER_STATUS	                           0XA044	//STATUS MESSAGE
#define MESH_OPCODE_SCENE_NUMBER_STORE	                                  0XA042	//RELIABLE MESSAGE
#define MESH_OPCODE_SCENE_NUMBER_SUPPORTED_GET	             0XA040	//RELIABLE MESSAGE
#define MESH_OPCODE_SCENE_NUMBER_SUPPORTED_STATUS	      0XA041	//STATUS MESSAGE

inline BOOL isTransactionReliable(uint8_t Tid)
{
    return (0x80&Tid?TRUE:FALSE);
}

inline BOOL isTransactionEnd(uint8_t Tid)
{
    return (0x40&Tid?TRUE:FALSE);
}

inline uint8_t getTid(uint8_t Tid)
{
    return (0x3F&Tid);
}

#define MESH_PRIORITY_NORMAL            0
#define MESH_PRIORITY_EMERGENCY     1

#define TRANSACTION_REL_BIT_ENABLE   0x80
#define TRANSACTION_END_BIT_ENABLE  0x40


typedef struct _TConfigDefaultTTLGet
{
    uint16_t opcode;
    uint8_t tid;
}_PACKED_ TConfigDefaultTTLGet;
typedef TConfigDefaultTTLGet * PConfigDefaultTTLGet;


typedef struct _TConfigDefaultTTLSet
{
    uint16_t opcode;
    uint8_t tid;
}_PACKED_ TConfigDefaultTTLSet;
typedef TConfigDefaultTTLSet *PConfigDefaultTTLSet;

typedef struct _TConfigDefaultTTLStatus
{
    uint16_t opcode;
    uint8_t tid;
    uint8_t defaultTTL;
}_PACKED_ TConfigDefaultTTLStatus;
typedef TConfigDefaultTTLStatus *PConfigDefaultTTLStatus;


typedef struct _TConfigFutureGet
{
    uint16_t opcode;
    uint8_t tid;
}_PACKED_ TConfigFutureGet;
typedef TConfigFutureGet *PConfigFutureGet;

typedef struct _TConfigFutureSet
{
    uint16_t opcode;
    uint8_t tid;
    uint8_t size;
    uint16_t interval;    
    uint16_t window;    
}_PACKED_ TConfigFutureSet;
typedef TConfigFutureSet *PConfigFutureSet;

typedef struct _TConfigFutureStatus
{
    uint16_t opcode;
    uint8_t tid;
    uint8_t size;
    uint8_t available;
    uint16_t interval;    
    uint16_t window;    
}_PACKED_ TConfigFutureStatus;
typedef TConfigFutureStatus *PConfigFutureStatus;

typedef struct _TConfigRelayGet
{
    uint16_t opcode;
    uint8_t tid;  
}_PACKED_ TConfigRelayGet;
typedef TConfigRelayGet *PConfigRelayGet;

typedef struct _TConfigRelaySet
{
    uint16_t opcode;
    uint8_t tid;  
    uint8_t state;  
}_PACKED_ TConfigRelaySet;
typedef TConfigRelaySet *PConfigRelaySet;

typedef struct _TConfigRelayStatus
{
    uint16_t opcode;
    uint8_t tid;  
    uint8_t state;  
}_PACKED_ TConfigRelayStatus;
typedef TConfigRelayStatus *PConfigRelayStatus;


typedef struct _TGenericDeltaSetFinal
{
    uint16_t opcode;
    uint8_t tid;  
    uint16_t delta;  
}_PACKED_ TGenericDeltaSetFinal;
typedef TGenericDeltaSetFinal *PGenericDeltaSetFinal;


typedef struct _TGenericDeltaSetUpdate
{
    uint16_t opcode;
    uint8_t tid;  
    uint16_t delta;  
}_PACKED_ TGenericDeltaSetUpdate;
typedef TGenericDeltaSetUpdate *PGenericDeltaSetUpdate;


typedef struct _TGenericLevelGet
{
    uint16_t opcode;
    uint8_t tid;  
}_PACKED_ TGenericLevelGet;
typedef TGenericLevelGet *PGenericLevelGet;

typedef struct _TGenericLevelSet
{
    uint16_t opcode;
    uint8_t tid;  
    uint16_t delta;  
}_PACKED_ TGenericLevelSet;
typedef TGenericLevelSet *PGenericLevelSet;


typedef struct _TGenericLevelStatus
{
    uint16_t opcode;
    uint8_t tid;  
    uint16_t delta;  
}_PACKED_ TGenericLevelStatus;
typedef TGenericLevelStatus *PGenericLevelStatus;

typedef struct _TGenericStateGet
{
    uint16_t opcode;
    uint8_t tid;
}_PACKED_ TGenericStateGet;
typedef TGenericStateGet *PGenericStateGet;

typedef struct _TGenericStateSet
{
    uint16_t opcode;
    uint8_t tid;
    uint8_t state;
}_PACKED_ TGenericStateSet;
typedef TGenericStateSet *PGenericStateSet;

typedef struct _TGenericStateStatus
{
    uint16_t opcode;
    uint8_t tid;
    uint8_t state;
}_PACKED_ TGenericStateStatus;
typedef TGenericStateStatus *PGenericStateStatus;

typedef struct _TLightColorTemperatureSet
{
    uint16_t opcode;
    uint8_t tid;
    uint16_t colorTemp;
}_PACKED_ TLightColorTemperatureSet;
typedef TLightColorTemperatureSet *PLightColorTemperatureSet;

typedef struct _TLightLevelTransitionTimeSet
{
    uint16_t opcode;
    uint8_t tid;
    uint8_t level;
    uint16_t transitionTime;
}_PACKED_ TLightLevelTransitionTimeSet;
typedef TLightLevelTransitionTimeSet *PLightLevelTransitionTimeSet;

typedef struct _TLightLevelSetUpdate
{
    uint16_t opcode;
    uint8_t tid;
    uint8_t level;
}_PACKED_ TLightLevelSetUpdate;
typedef TLightLevelSetUpdate *PLightLevelSetUpdate;

typedef struct _TLightLevelSet
{
    uint16_t opcode;
    uint8_t tid;
    uint8_t level;
}_PACKED_ TLightLevelSet;
typedef TLightLevelSet *PLightLevelSet;

typedef struct _TLightStateGet
{
    uint16_t opcode;
    uint8_t tid;
}_PACKED_ TLightStateGet;
typedef TLightStateGet *PLightStateGet;
	
typedef struct _TLightStateStatus
{
    uint16_t opcode;
    uint8_t tid;
    uint8_t level;
    uint16_t colorTemp;
    uint16_t transitionTime;
}_PACKED_ TLightStateStatus;
typedef TLightStateStatus *PLightStateStatus;

typedef struct _TPowerStateGet
{
    uint16_t opcode;
    uint8_t tid;
}_PACKED_ TPowerStateGet;
typedef TPowerStateGet *PPowerStateGet;

typedef struct _TPowerStateSet
{
    uint16_t opcode;
    uint8_t tid;
    uint8_t state;
}_PACKED_ TPowerStateSet;
typedef TPowerStateSet *PPowerStateSet;

typedef struct _TPowerStateStatus
{
    uint16_t opcode;
    uint8_t tid;
    uint8_t state;
}_PACKED_ TPowerStateStatus;
typedef TPowerStateStatus *PPowerStateStatus;

typedef struct _TSceneNumberSelect
{
    uint16_t opcode;
    uint8_t tid;
    uint8_t scene;
}_PACKED_ TSceneNumberSelect;
typedef TSceneNumberSelect *PSceneNumberSelect;

typedef struct _TSceneNumberStatus
{
    uint16_t opcode;
    uint8_t tid;
    uint8_t last;
    uint8_t duration;
}_PACKED_ TSceneNumberStatus;
typedef TSceneNumberStatus *PSceneNumberStatus;


typedef struct _TSceneNumberSupportedGet
{
    uint16_t opcode;
    uint8_t tid;
}_PACKED_ TSceneNumberSupportedGet;
typedef TSceneNumberSupportedGet *PSceneNumberSupportedGet;

typedef struct _TSceneNumberSupportedStatus
{
    uint16_t opcode;
    uint8_t tid;
    uint8_t number;
}_PACKED_ TSceneNumberSupportedStatus;
typedef TSceneNumberSupportedStatus *PSceneNumberSupportedStatus;


void smgap_ConfigDefaultTTLGet(uint16_t dest_addr);
void smgap_ConfigDefaultTTLSet(uint16_t dest_addr);
void smgap_ConfigDefaultTTLStatus(uint16_t dest_addr, uint8_t defaultTTL);
void smgap_ConfigFutureGet(uint16_t dest_addr);
void smgap_ConfigFutureSet(uint16_t dest_addr, uint8_t size, uint16_t interval, uint16_t window);
void smgap_ConfigFutureStatus(uint16_t dest_addr, uint8_t size, uint8_t available,uint16_t interval, uint16_t window);
void smgap_ConfigRelayGet(uint16_t dest_addr);
void smgap_ConfigRelaySet(uint16_t dest_addr, uint8_t state);
void smgap_ConfigRelayStatus(uint16_t dest_addr, uint8_t state);
void smgap_GenericDeltaSetFinal(uint16_t dest_addr, uint16_t delta);
void smgap_GenericDeltaSetUpdate(uint16_t dest_addr, uint8_t delta);
void smgap_GenericLevelGet(uint16_t dest_addr);
void smgap_GenericLevelSet(uint16_t dest_addr, uint8_t delta);
void smgap_GenericLevelStatus(uint16_t dest_addr, uint8_t delta);
void smgap_GenericStateGet(uint16_t dest_addr);
void smgap_GenericStateSet(uint16_t dest_addr, uint8_t state);
void smgap_GenericStateStatus(uint16_t dest_addr, uint8_t state);
void smgap_LightColorTemperatureSet(uint16_t dest_addr, uint16_t colorTemp);
void smgap_LightLevelTransitionTimeSet(uint16_t dest_addr, uint8_t level, uint16_t transitionTime);
void smgap_LightLevelSetUpdate(uint16_t dest_addr, uint8_t level);
void smgap_LightLevelSet(uint16_t dest_addr, uint8_t level);
void smgap_LightStateGet(uint16_t dest_addr);
void smgap_LightStateStatus(uint16_t dest_addr, uint8_t level);
void smgap_PowerStateGet(uint16_t dest_addr);
void smgap_PowerStateSet(uint16_t dest_addr, uint8_t state);
void smgap_PowerStateStatus(uint16_t dest_addr, uint8_t state);
void smgap_SceneNumberSelect(uint16_t dest_addr, uint8_t scene);
void smgap_SceneNumberStatus(uint16_t dest_addr, uint8_t last, uint8_t duration);
void smgap_SceneNumberStore(uint16_t dest_addr, uint8_t last, uint8_t duration);
void smgap_SceneNumberSupportedGet(uint16_t dest_addr);
void smgap_SceneNumberSupportedStatus(uint16_t dest_addr, uint8_t last, uint8_t duration);


void MeshGapReceive(uint16_t src, uint8_t* pbuffer, uint8_t len, uint8_t pri, uint8_t broadcast_flag);



#ifdef  __cplusplus
}
#endif      /*  __cplusplus */

#endif
