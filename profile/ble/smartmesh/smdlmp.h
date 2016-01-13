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
#ifndef _SMDLMP_H_
#define _SMDLMP_H_

#ifdef  __cplusplus
extern "C" {
#endif      /* __cplusplus */

#define DIMMABLE_LIGHT_MSG_PRIORITY                 0

/** @defgroup Dimmable_Light_Opcode Dimmable Light Opcode
  * @brief opcode
  * @{
  */ 
#define DIMMABLE_LIGHT_SET_LEVEL_WITHOUT_ACK        0x8020
#define DIMMABLE_LIGHT_SET_LEVEL_WITH_ACK           0xA020
#define DIMMABLE_LIGHT_GET_STATE                    0xA021
#define DIMMABLE_LIGHT_STATE                        0xA022
/** @} */

typedef struct
{
    uint16_t opcode;
    uint8_t state;
}_PACKED_ TSetLevelWithoutAckMsg;

typedef struct
{
    uint16_t opcode;
    uint8_t tid;
    uint8_t state;
}_PACKED_ TSetLevelWithAckMsg;

typedef struct
{
    uint16_t opcode;
    uint8_t tid;
}_PACKED_ TGetStateMsg;

typedef struct
{
    uint16_t opcode;
    uint8_t tid;
    uint8_t state;
}_PACKED_ TStateMsg;

bool smdlp_SetLevelWithoutAck(uint16_t dest_addr, uint8_t state);
bool smdlp_SetLevelWithAck(uint16_t dest_addr, uint8_t state);
bool smdlp_GetState(uint16_t dest_addr);
bool smdlp_State(uint16_t dest_addr, uint8_t tid, uint8_t state);
void smdlp_ReceiveMsg(uint16_t sr_addr, uint8_t *pdata, uint8_t data_len, uint16_t dst_addr);

#ifdef  __cplusplus
}
#endif      /*  __cplusplus */

#endif
