/**
*********************************************************************************************************
*               Copyright(c) 2015, Realtek Semiconductor Corporation. All rights reserved.
**********************************************************************************************************
* @file     rtl_log_uart.c
* @brief    This file provides Log UART firmware functions.
* @details
* @author   tifnan_ge
* @date     2015-12-14
* @version  v0.1
*********************************************************************************************************
*/
#include "rtl876x.h"
#include "rtl876x_bitfields.h"
#include "rtl876x_log_uart.h"

#define LCR_DLAB_Set                    ((uint32_t)(1 << 7))
#define LCR_DLAB_Reset                  ((uint32_t)~(1 << 7))

//uint32_t UartFcrRegVal = 0;

/**
  * @brief Initializes the Log UART peripheral according to the specified 
  *   parameters in the LOG_UART_InitStruct
  * @param  UARTx: selected Log UART peripheral.
  * @param  LOG_LOG_UART_InitStruct: pointer to a LOG_UART_InitTypeDef structure that
  *   contains the configuration information for the specified Log UART peripheral
  * @retval None
  */
void LOG_UART_Init(LOG_UART_TypeDef* UARTx, LOG_UART_InitTypeDef* LOG_UART_InitStruct)
{
    assert_param(IS_LOG_UART_PERIPH(UARTx));
    assert_param(IS_LOG_UART_WORD_LENGTH(LOG_UART_InitStruct->wordLen));
    assert_param(IS_LOG_UART_PARITY(LOG_UART_InitStruct->parity));
    assert_param(IS_LOG_UART_STOPBITS(LOG_UART_InitStruct->stopBits));
    assert_param(IS_LOG_UART_AUTO_FLOW_CTRL(LOG_UART_InitStruct->autoFlowCtrl));
    assert_param(IS_LOG_UART_DMA_CFG(LOG_UART_InitStruct->dmaEn));
    assert_param(IS_LOG_UART_FIFO_MODE(LOG_UART_InitStruct->fifo_ctrl));
    assert_param(IS_LOG_UART_RX_FIFO_TRIGGER_LEVEL(LOG_UART_InitStruct->rxTriggerLevel));

    uint32_t regVal = 0;
    uint16_t div;

    //clear DLAB bit
    UARTx->LCR &= LCR_DLAB_Reset;
    
    //enable log uart peripheral & clock
    SYSBLKCTRL->SOC_FUNC_EN |= (1 << 12);
    SYSBLKCTRL->CLK_CTRL |= (1 << 12);

    //read to clear Line Status Reg
    regVal = UARTx->LSR;
    regVal = regVal;    //just fix compile warning

    //clear Tx FIFO & Rx Fifo
    UARTx->INTID_FCR |= (FCR_CLEAR_RX_FIFO_Set | FCR_CLEAR_TX_FIFO_Set);

    //disable all interrupt
    UARTx->DLH_INTCR = 0x00;

    //set baudrate, firstly set DLAB bit
    UARTx->LCR |= LCR_DLAB_Set;
    //set DLL and DLH
    div = (SystemCoreClock *2 + 8 * LOG_UART_InitStruct->baudrate) / (16 * LOG_UART_InitStruct->baudrate);
    UARTx->RB_THR_DLL = (div & 0xff);
    UARTx->DLH_INTCR = ((div & 0xFF00) >> 8);
    //after set baudrate, clear DLAB bit
    UARTx->LCR &= LCR_DLAB_Reset;

    //set LCR reg
    UARTx->LCR = (LOG_UART_InitStruct->parity | LOG_UART_InitStruct->stopBits | LOG_UART_InitStruct->wordLen);
    //set FCR reg
    UARTx->INTID_FCR = (LOG_UART_InitStruct->fifoCtrl | LOG_UART_InitStruct->rxTriggerLevel | LOG_UART_InitStruct->dmaMode | LOG_UART_InitStruct->txTriggerLevel);

    /* auto flow control */
    UARTx->MCR &= (~((1 << 5) | (1 << 1)));
    UARTx->MCR |= LOG_UART_InitStruct->autoFlowCtrl;
    
    return;
}

/**
  * @brief  Deinitializes the Log UART peripheral registers to their default reset values(turn off UART clock).
  * @param  UARTx: selected Log UART peripheral.
  * @retval None
  */
void LOG_UART_DeInit(LOG_UART_TypeDef* UARTx)
{
    assert_param(IS_LOG_UART_PERIPH(UARTx));
    
    //disable log uart peripheral & clock
    SYSBLKCTRL->SOC_FUNC_EN &= (~(1 << 12));
    SYSBLKCTRL->CLK_CTRL &= (~(1 << 12));

    return;
}

/**
  * @brief  Fills each LOG_UART_InitStruct member with its default value.
  * @param  LOG_UART_InitStruct: pointer to an LOG_UART_InitTypeDef structure which will be initialized.
  * @retval None
  */
void LOG_UART_StructInit(LOG_UART_InitTypeDef* LOG_UART_InitStruct)
{
    //115200 default
    LOG_UART_InitStruct->baudrate       = 115200;  //read from efuse

    LOG_UART_InitStruct->parity         = LOG_UART_PARITY_NO_PARTY;
    LOG_UART_InitStruct->stopBits       = LOG_UART_STOP_BITS_1;
    LOG_UART_InitStruct->wordLen        = LOG_UART_WROD_LENGTH_8BIT;
    LOG_UART_InitStruct->dmaMode        = LOG_UART_DMA_MODE0;
    LOG_UART_InitStruct->autoFlowCtrl   = LOG_UART_AUTO_FLOW_CTRL_DIS;
    LOG_UART_InitStruct->rxTriggerLevel = LOG_UART_RX_FIFO_TRIGGER_LEVEL_8BYTE;
    LOG_UART_InitStruct->txTriggerLevel = LOG_UART_TX_FIFO_TRIGGER_LEVEL_EMPTY;
    LOG_UART_InitStruct->fifoCtrl       = LOG_UART_FIFO_EN;
    
    return;
}

/**
  * @brief  Receive data from rx FIFO.
  * @param  UARTx: selected UART peripheral.
  * @param[out]  outBuf: buffer to save data read from Log UART FIFO.
  * @param  count: number of data to be read.
  * @retval None
  */
void LOG_UART_ReceiveData(LOG_UART_TypeDef* UARTx, uint8_t* outBuf, uint16_t count)
{
    /* Check the parameters */
    assert_param(IS_LOG_UART_PERIPH(UARTx));
    
    while((count--) != 0)
    {
        *outBuf++ = (uint8_t)UARTx->RB_THR_DLL;
    }

    return;
}

/**
  * @brief  Send data to tx FIFO.
  * @param  UARTx: selected Log UART peripheral.
  * @param  inBuf: buffer to be written to Tx FIFO.
  * @param  count: number of data to be written.
  * @retval None
  */
void LOG_UART_SendData(LOG_UART_TypeDef* UARTx, const uint8_t* inBuf, uint16_t count)
{
    /* Check the parameters */
    assert_param(IS_LOG_UART_PERIPH(UARTx));

    while((count--) != 0)
    {
        UARTx->RB_THR_DLL = *inBuf++;
    }

    return;
}


