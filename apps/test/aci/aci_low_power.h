/**
*********************************************************************************************************
*               Copyright(c) 2014, Realtek Semiconductor Corporation. All rights reserved.
*********************************************************************************************************
* @file      aci_low_power.h
* @brief     low power handle when using ACI.
* @details   none.
* @author    tifnan
* @date      2014-11-19
* @version   v0.1
* *********************************************************************************************************
*/

#ifndef _ACI_LOW_POWER_H_
#define _ACI_LOW_POWER_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "FreeRTOS.h"
#include "timers.h"
#include "btltp.h"

/* power status */
#define LTPLL_STATE_BE_WAKED            0x01    /* waked by the other side */
#define LTPLL_STATE_PRE_WAKE            0x02    /* not really wake, may enter dlps immediately */
#define LTPLL_STATE_OWN_WAKE            0x03    /* waked itself */
#define LTPLL_STATE_W4ACK               0x04    /* wait for ack from the other side */
#define LTPLL_STATE_DLPS                0x05    /* in dlps state */
#define LTPLL_STATE_PRE_SLEEP           0x06    /* bee is waiting for host gpio low */

/* power mode---low power mode or not */
#define LTPLL_LPWR_OFF                  0x00
#define LTPLL_LPWR_ON                   0x01

/* get power status */
#define LTPLL_GetPowerStatus()                              (LtpPowerState)
#define LTPLL_SetPowerStatus(power_state)                   DBG_BUFFER(MODULE_LTP, LEVEL_INFO,  "--->power status:0x%x-->0x%x", 2,\
        LtpPowerState, (power_state));\
LtpPowerState = (power_state)

/* globals */
extern uint8_t LtpPowerState;
extern uint8_t HostPwrMode;
extern uint8_t BeePwrMode;
extern uint8_t MonitorTimeout;
extern xTimerHandle MoniterTimer;

extern BOOL LTPLL_HandleLowPowerCmd(uint8_t* p_msg, uint8_t cmd, uint8_t copmsk,
                                    uint8_t* pOpt, uint16_t lenPara, uint8_t* pPara);
void LTPLL_WakeUpHost(void);
BOOL LTPLL_IsHostSleep(void);

#ifdef __cplusplus
}
#endif

#endif  /* _ACI_LOW_POWER_H_ */

