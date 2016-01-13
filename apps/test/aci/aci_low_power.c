/**
*********************************************************************************************************
*               Copyright(c) 2014, Realtek Semiconductor Corporation. All rights reserved.
*********************************************************************************************************
* @file      aci_low_power.c
* @brief     low power handle when using ACI.
* @details   none.
* @author    Tifnan
* @date      2014-11-19
* @version   v0.1
* *********************************************************************************************************
*/
#if 0
#include "aci_low_power.h"
#include "btltp.h"
#include "dlps_platform.h"
#include "blueapi.h"
#include "rtl876x_gpio.h"
#include "rtl876x_nvic.h"
#include "rtl876x_rcc.h"
#include "rtl876x_io_dlps.h"
#include "rtl876x.h"
#include "freeRTOS.h"
#include "timers.h"
#include "board.h"
#include "trace.h"
#include "aci_low_power_utils.h"

/* default input wakeup pin and output notify pin */
uint32_t AciIn_WakeupPin;
uint32_t AciOut_NotifyPin;

/** @brief the current power state of bee */
uint8_t LtpPowerState = LTPLL_STATE_PRE_WAKE;
/** @brief Host power mode */
uint8_t HostPwrMode = LTPLL_LPWR_OFF;
/** @brief Bee power mode */
uint8_t BeePwrMode = LTPLL_LPWR_OFF;
/** @brief monitor timer flag */
uint8_t MonitorTimeout  = 0;
/** @brief monitor timer*/
xTimerHandle MoniterTimer = NULL;

/* monitor timer timeout handler */
void MonitorTimerHandler(xTimerHandle handle)
{
    DBG_BUFFER(MODULE_LTP, LEVEL_INFO, "<<<<<<<<<<<Moniter timer timeout", 0);

    MonitorTimeout = 1;

    return;
}

/**
 * @brief send the data in queue save when bee is waiting for host ack.
 *
 * @param p_buf pointer to the buffer start address.
 * @param buf_len the length of the buffer.
 * @return none.
 * @retal   void.
*/
static void LtpLowPowerQueueSend(void)
{
    uint16_t i = 0;
    PLTPLL_BufMsg p_buf_msg = NULL;
    TLTPData tx_data = {NULL, 0};
    uint16_t queue_size = LTPLL_GetQueueSize();

    DBG_BUFFER(MODULE_LTP, LEVEL_INFO, "LtpLowPowerQueueSend--Queue size: %d", 1, queue_size);

    /* send all message in tx buffer */
    for (i = 0; i < queue_size; i++)
    {
        p_buf_msg = LTPLL_QueueOut();
        tx_data.pBuffer = p_buf_msg->buf_addr;
        tx_data.Length = p_buf_msg->size;
        if (NULL != p_buf_msg)
        {
            /* take care that exceed the size of QueueHandleTxData!!! */
            if (pdFALSE == xQueueSend(P_BtLtp->p_aci_tcb->QueueHandleTxData, &tx_data, 0))
            {
                DBG_BUFFER(MODULE_LTP, LEVEL_ERROR, "LtpLowPowerQueueSend:xQueueSend fail", 0);
            }

            /* free buffer */
            osMemoryFree(p_buf_msg);
        }
    }
}

/**
 * @brief goio interrupt handle ISR.
 * @param none.
 * @return none.
 * @retval void.
*/
void LTPLL_HostGpioIsr(void)
{
    portBASE_TYPE TaskWoken = pdFALSE;

    GPIO_MaskINTConfig(AciIn_WakeupPin, ENABLE);

    DBG_BUFFER(MODULE_LTP, LEVEL_INFO, "<<<<<<<<<<<ACI GPIO ISR", 0);

    /* host wake up bee */
    if (LTPLL_STATE_PRE_WAKE == LTPLL_GetPowerStatus())
    {
        DBG_BUFFER(MODULE_LTP, LEVEL_INFO, "--->GPIO ISR:host wakeup bee", 0);

        /* sene ack to host */
        GPIO_SetBits(AciOut_NotifyPin);
        LTPLL_SetPowerStatus(LTPLL_STATE_BE_WAKED);

    }
    /* host ack */
    else if (LTPLL_STATE_W4ACK == LTPLL_GetPowerStatus())
    {
        DBG_BUFFER(MODULE_LTP, LEVEL_INFO, "--->GPIO ISR:host ack bee", 0);

        LTPLL_SetPowerStatus(LTPLL_STATE_OWN_WAKE);

        /* if have stack message in queue, handle it */
        LtpLowPowerQueueSend();
    }
    /* wrong interrupt sig */
    else
    {
        DBG_BUFFER(MODULE_LTP, LEVEL_ERROR, "LTPLL_HostGpioIsr:wrong interrupt:0x%x", 1, LTPLL_GetPowerStatus());
    }

    GPIO_ClearINTPendingBit(AciIn_WakeupPin);

    /* if do not use low power mode, do not unmask interrupt */
    if (BeePwrMode == LTPLL_LPWR_OFF)
    {
        GPIO_MaskINTConfig(AciIn_WakeupPin, DISABLE);
    }

    /* reset timer */
    if (MoniterTimer)
    {
        xTimerResetFromISR(MoniterTimer, &TaskWoken);
        MonitorTimeout = 0;
    }

    portYIELD_FROM_ISR(TaskWoken);

    return;
}

BOOL LTPLL_IsHostSleep(void)
{
    return !GPIO_ReadInputDataBit(AciIn_WakeupPin);
}

void LTPLL_WakeUpHost(void)
{
    uint32_t i = 0;
    
    //host LPS on, bee LPS off
    if(BeePwrMode == LTPLL_LPWR_OFF
         && HostPwrMode == LTPLL_LPWR_ON)
    {
        GPIO_ResetBits(AciOut_NotifyPin);
        //delay some time, about 25us@40M
        for(i = 1000; i > 0; i--);
        GPIO_SetBits(AciOut_NotifyPin);
    }
    else
    {
        GPIO_SetBits(AciOut_NotifyPin);
    }
}

/**
 * @brief when bee exit dlps, this function will be called.
 * @param none.
 * @return none.
 * @retval void.
*/
void LTPLL_ExitDlpsRoutin(void)
{
    unsigned long io_level = 0;

    Pad_Config(UART_TX_PIN, PAD_PINMUX_MODE, PAD_IS_PWRON, PAD_PULL_NONE, PAD_OUT_DISABLE, PAD_OUT_LOW);
    Pad_Config(UART_RX_PIN, PAD_PINMUX_MODE, PAD_IS_PWRON, PAD_PULL_UP, PAD_OUT_DISABLE, PAD_OUT_LOW);
    Pad_Config(DLPS_HOST_PIN, PAD_PINMUX_MODE, PAD_IS_PWRON, PAD_PULL_DOWN, PAD_OUT_DISABLE, PAD_OUT_LOW);
    Pad_Config(DLPS_BEE_PIN, PAD_PINMUX_MODE, PAD_IS_PWRON, PAD_PULL_DOWN, PAD_OUT_ENABLE, PAD_OUT_LOW);

    io_level = GPIO_ReadInputDataBit(AciIn_WakeupPin);

    /* host wakeup Bee */
    if (1 == io_level)
    {
        DBG_BUFFER(MODULE_LTP, LEVEL_INFO, "--->host wakeup bee", 0);
    }
    else /* Bee wake up itself */
    {
        //DBG_BUFFER(MODULE_LTP, LEVEL_INFO, "<---bee wakeup itself", 0);
    }

    LTPLL_SetPowerStatus(LTPLL_STATE_PRE_WAKE);

    return;
}

void LTPLL_EnterDlpsRoutin(void)
{
    //DBG_BUFFER(MODULE_LTP, LEVEL_INFO,  "-->Bee send sleep indication to host\n", 0);
    /* pad settings */
    Pad_Config(UART_TX_PIN, PAD_SW_MODE, PAD_IS_PWRON, PAD_PULL_NONE, PAD_OUT_DISABLE, PAD_OUT_HIGH);
    Pad_Config(UART_RX_PIN, PAD_SW_MODE, PAD_IS_PWRON, PAD_PULL_NONE, PAD_OUT_DISABLE, PAD_OUT_HIGH);
    Pad_Config(DLPS_HOST_PIN, PAD_SW_MODE, PAD_IS_PWRON, PAD_PULL_DOWN, PAD_OUT_DISABLE, PAD_OUT_LOW);
    Pad_Config(DLPS_BEE_PIN, PAD_SW_MODE, PAD_IS_PWRON, PAD_PULL_DOWN, PAD_OUT_ENABLE, PAD_OUT_LOW);

    /* unmask gpio interrupt again */
    GPIO_MaskINTConfig(AciIn_WakeupPin, DISABLE);
    LTPLL_SetPowerStatus(LTPLL_STATE_DLPS);

    return;
}

BOOL LTPLL_EnterDlpsCheck(void)
{
    if (LtpDlpsEnterCheck() == FALSE)
    {
        return FALSE;
    }
    /* timeout */
    if (MonitorTimeout)
    {
        if( LTPLL_GetPowerStatus() == LTPLL_STATE_OWN_WAKE
            || LTPLL_GetPowerStatus() == LTPLL_STATE_BE_WAKED)
        {
            /* pull gpio low */
            GPIO_ResetBits(AciOut_NotifyPin);
            LTPLL_SetPowerStatus(LTPLL_STATE_PRE_SLEEP);
        }

        /* wait host gpio low */
        if (!GPIO_ReadInputDataBit(AciIn_WakeupPin))
        {
            //there is data in queue
            if(LTPLL_GetQueueSize())
            {
                /* unmask gpio interrupt again */
                GPIO_MaskINTConfig(AciIn_WakeupPin, DISABLE);
                GPIO_SetBits(AciOut_NotifyPin);
                LTPLL_SetPowerStatus(LTPLL_STATE_W4ACK);
                xTimerReset(MoniterTimer, 5);
                MonitorTimeout = 0;
                return FALSE;
            }
            else
            {
                return TRUE;
            }
        }
        else
        {
            return FALSE;
        }
    }
    else
    {
        return FALSE;
    }
}

/******************************************  aci export function ************************************/

void LTPLL_HandleHwInitReq(uint8_t hostPwrMode, uint8_t beePwrMode)
{
    GPIO_InitTypeDef GPIO_InitStruct;

    /* if host and Bee both do not use low power mode, no need to init GPIO */
    if (hostPwrMode == LTPLL_LPWR_OFF
            && beePwrMode == LTPLL_LPWR_OFF)
    {
        return;
    }

    /* turn on gpio clock */
    RCC_PeriphClockCmd(APBPeriph_GPIO, APBPeriph_GPIO_CLOCK, ENABLE);

    /* pad config */
    Pad_Config(DLPS_HOST_PIN, PAD_PINMUX_MODE, PAD_IS_PWRON, PAD_PULL_DOWN, PAD_OUT_DISABLE, PAD_OUT_LOW);
    Pad_Config(DLPS_BEE_PIN, PAD_PINMUX_MODE, PAD_IS_PWRON, PAD_PULL_DOWN, PAD_OUT_ENABLE, PAD_OUT_LOW);

    /* pinmux confguration */
    Pinmux_Config(DLPS_HOST_PIN, GPIO_FUN);
    Pinmux_Config(DLPS_BEE_PIN, GPIO_FUN);

    AciOut_NotifyPin = GPIO_GetPin(DLPS_BEE_PIN);
    AciIn_WakeupPin  = GPIO_GetPin(DLPS_HOST_PIN);

    /* struct init */
    GPIO_StructInit(&GPIO_InitStruct);

    /* output pin init */
    GPIO_InitStruct.GPIO_Pin    = AciOut_NotifyPin;
    GPIO_InitStruct.GPIO_Mode   = GPIO_Mode_OUT;
    GPIO_InitStruct.GPIO_ITCmd  = DISABLE;
    GPIO_Init(&GPIO_InitStruct);
    GPIO_SetBits(AciOut_NotifyPin);     //pull high

    /* input pin init */
    GPIO_InitStruct.GPIO_Pin          = AciIn_WakeupPin;
    GPIO_InitStruct.GPIO_Mode         = GPIO_Mode_IN;
    GPIO_InitStruct.GPIO_ITCmd        = ENABLE;
    GPIO_InitStruct.GPIO_ITTrigger    = GPIO_INT_Trigger_LEVEL;
    GPIO_InitStruct.GPIO_ITPolarity   = GPIO_INT_POLARITY_ACTIVE_HIGH;
    GPIO_InitStruct.GPIO_ITDebounce   = GPIO_INT_DEBOUNCE_DISABLE;

    GPIO_Init(&GPIO_InitStruct);

    /* if not enable bee low power mode, no need to enable wakeu pin and gpio interrupt */
    if (beePwrMode == LTPLL_LPWR_ON)
    {
        /* enable input pin wakeup function, default p3_2 */
        System_WakeUp_Pin_Enable(DLPS_HOST_PIN, 1);

        /* GPIO NVIC */
        NVIC_InitTypeDef NVIC_InitStruct;

        NVIC_InitStruct.NVIC_IRQChannel = GPIO6To31_IRQ;
        NVIC_InitStruct.NVIC_IRQChannelPriority = 1;
        NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;

        NVIC_Init(&NVIC_InitStruct);
    }

    return;
}

void LTPLL_HandleDlpsConfig(void)
{
    DBG_BUFFER(MODULE_LTP, LEVEL_INFO,  "------LTPLL_DlpsEnable------", 0);

    /* if Bee does not use low power mode, do nothing */
    if (BeePwrMode == LTPLL_LPWR_OFF)
    {
        return;
    }

    DLPS_INTERRUPT_CONTROL_CB_REG(LTPLL_EnterDlpsRoutin, DLPS_ENTER);
    DLPS_IO_Register();

    DLPS_ENTER_CHECK_CB_REG(LTPLL_EnterDlpsCheck);
    DLPS_IO_RegUserDlpsExitCb(LTPLL_ExitDlpsRoutin);

    return;
}

void LTPLL_HandleDlpsCtrlReq(void* param)
{
    DBG_BUFFER(MODULE_LTP, LEVEL_INFO,  "------LTPLL_PwrCtrl: 0x%x------", 1, *(uint8_t*)param);

    uint8_t* p = NULL;

    /* dlps on */
    if (0x01 == *(uint8_t*)param)
    {
        LPS_MODE_Set(LPM_DLPS_MODE);
    }

    /* dlps off */
    if (0x02 == *(uint8_t*)param)
    {
        LPS_MODE_Set(LPM_ACTIVE_MODE);
    }

    /* power mode pause */
    if (0x03 == *(uint8_t*)param)
    {
        LPS_MODE_Pause();
    }

    /* power mode resume */
    if (0x04 == *(uint8_t*)param)
    {
        LPS_MODE_Resume();
    }

    /* sned dlps_ctrl_rsp */
    p = BTLTPTgtSendBufferAlloc(P_BtLtp, 6);
    *p = 0xFC;
    *(p + 1) = 0x01;
    *(p + 2) = 0x00;
    *(p + 3) = 0x06;
    *(p + 4) = 0x04;
    *(p + 5) = *(uint8_t*)param;
    BTLTPTgtSendLTPMessage(P_BtLtp, p, 0, 6);

    return;
}

/* host power mode config */
void LTPLL_HandlePwrCfgReq(void* param)
{
    uint8_t* p = (uint8_t*)param;
    uint16_t moniter_time = 0;

    /* host support low power mode */
    if (0x01 == *p++)
    {
        HostPwrMode = LTPLL_LPWR_ON;
    }

    /* Bee support low power mode */
    if (0x01 == *p++)
    {
        BeePwrMode = LTPLL_LPWR_ON;
    }
    moniter_time = NETCHAR2SHORT(p);
    DBG_BUFFER(MODULE_LTP, LEVEL_ERROR,  "hsot_power_mode:0x%x, bee_power_mode:0x%x time %d", 3, \
               HostPwrMode, BeePwrMode, moniter_time);

    /* hardware init */
    LTPLL_HandleHwInitReq(HostPwrMode, BeePwrMode);

    /* if not enable low power mode, no need to register dlps callback and moniter timer */
    if (BeePwrMode == LTPLL_LPWR_ON)
    {
        /* create monitor timer, (200 * x) ms */
        MoniterTimer = xTimerCreate((const char*)"Moniter_Timer", \
                                    (moniter_time * 10) / portTICK_RATE_MS, pdFALSE, NULL, MonitorTimerHandler);

        /* dlps config */
        LTPLL_HandleDlpsConfig();

        xTimerStart(MoniterTimer, 10);

        LPS_MODE_Set(LPM_DLPS_MODE);
    }

    /* send pwr_cfg_rsp */
    p = BTLTPTgtSendBufferAlloc(P_BtLtp, 7);
    *p = 0xFC;
    *(p + 1) = 0x01;
    *(p + 2) = 0x00;
    *(p + 3) = 0x07;
    *(p + 4) = 0x02;
    *(p + 5) = HostPwrMode;
    *(p + 6) = BeePwrMode;
    BTLTPTgtSendLTPMessage(P_BtLtp, p, 0, 7);

    return;
}

/**
 * @brief aci handle host command, for test now.
 * @param p_msg the message data buffer.
 * @param cmd ltp protocol command.
 * @param copmsk ltp protocol compask.
 * @param pOpt pointer to the optional parameter field.
 * @param lenPara the size of parameters.
 * @param pPara pointer to the payload field.
 * @return if reuse ltp message buffer.
 * @retval TRUE reuse mesage buffer.
 *         FALSE release message buffer.
*/
BOOL LTPLL_HandleLowPowerCmd(uint8_t* p_msg, uint8_t cmd, uint8_t copmsk, uint8_t* pOpt, uint16_t lenPara, uint8_t* pPara)
{
    DBG_BUFFER(MODULE_LTP, LEVEL_INFO,  "-----LTPLL_HandleLowPowerCmd----cmd:0x%x, lenPara: 0x%x", 2, *pOpt, lenPara);

    uint8_t command = *pOpt;

    switch (command)
    {
    /* config host and bee power mode */
    case 0x01:
        LTPLL_HandlePwrCfgReq((void*)pPara);
        break;

    /* power control */
    case 0x03:
        LTPLL_HandleDlpsCtrlReq((void*)pPara);
        break;

    default:
        break;
    }

    return TRUE;
}
#endif
