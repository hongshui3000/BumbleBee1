/**
*********************************************************************************************************
*               Copyright(c) 2015, Realtek Semiconductor Corporation. All rights reserved.
*********************************************************************************************************
* @file      log_uart.h
* @brief     header file of log uart driver.
* @details   
* @author    tifnan_ge
* @date      2015-12-14
* @version   v0.1
* *********************************************************************************************************
*/


#ifndef _LOG_UART_H_
#define _LOG_UART_H_

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "rtl876x.h"
#include "rtl876x_bitfields.h"
     
/** @addtogroup RTK_Periph_Driver RTK Periphral Driver
  * @{
  */

/** @addtogroup LOG_UART LOG UART
  * @brief UART driver module
  * @{
  */

/** @defgroup LOG_UART_Exported_Types LOG UART Exported Types
  * @{
  */

/**
 * @brief Log UART initialize parameters
 *
 * UART initialize parameters
 */
typedef struct
{
    uint32_t baudrate;
    uint16_t wordLen;
    uint16_t parity;
    uint16_t stopBits;
    uint16_t fifoCtrl;
    uint16_t autoFlowCtrl;

    uint16_t rxTriggerLevel;
    uint16_t txTriggerLevel;
    uint16_t dmaMode;
}LOG_UART_InitTypeDef, *PLOG_UART_InitTypeDef;

/**
  * @}
  */

/** @defgroup LOG_UART_Exported_Constants Log UART Exported Constants
  * @{
  */

#define IS_LOG_UART_PERIPH(PERIPH) ((PERIPH) == LOG_UART)
#define LOG_UART_TX_FIFO_SIZE           16
#define LOG_UART_RX_FIFO_SIZE           16

/** @defgroup LOG_UART_Interrupts_Definition Log UART Interrupts Definition 
  * @{
  */

#define LOG_UART_INT_RD_AVA                             ((uint16_t)(1 << 0))     //receive data avaliable
#define LOG_UART_INT_FIFO_EMPTY                         ((uint16_t)(1 << 1))
#define LOG_UART_INT_LINE_STS                           ((uint16_t)(1 << 2))
#define LOG_UART_INT_MODEM_STS                          ((uint16_t)(1 << 3))

#define IS_LOG_UART_IT(IT) ((((IT) & 0xFFFFFFF0) == 0x00) && ((IT) != 0x00))

#define IS_LOG_UART_GET_IT(IT) (((IT) == LOG_UART_IT_RD_AVA) || ((IT) == LOG_UART_IT_FIFO_EMPTY)\
                            || ((IT) == LOG_UART_IT_LINE_STS) || ((IT) == LOG_UART_IT_MODEM_STS))
/**
  * @}
  */

/** @defgroup LOG_UART_Interrupt_Identifier UART Interrupt Identifier 
  * @{
  */

#define LOG_UART_INT_ID_CHAR_TIMEOUT                    ((uint16_t)(0x0c << 0))
#define LOG_UART_INT_ID_BUSY_DETECT                     ((uint16_t)(7 << 0))
#define LOG_UART_INT_ID_LINE_STATUS                     ((uint16_t)(6 << 0))
#define LOG_UART_INT_ID_RX_LEVEL_REACH                  ((uint16_t)(4 << 0))
#define LOG_UART_INT_ID_TX_EMPTY                        ((uint16_t)(2 << 0))
#define LOG_UART_INT_ID_NO_PEND_INT                     ((uint16_t)(1 << 0))
#define LOG_UART_INT_ID_MODEM_STATUS                    ((uint16_t)(0 << 0))

#define IS_LOG_UART_IT_ID(ID) (((ID) == LOG_UART_INT_ID_MODEM_STATUS) || ((ID) == LOG_UART_INT_ID_NO_PEND_INT)\
                            || ((ID) == LOG_UART_INT_ID_TX_EMPTY) || ((ID) == LOG_UART_INT_ID_RX_LEVEL_REACH)\
                            || ((ID) == LOG_UART_INT_ID_LINE_STATUS) || ((ID) == LOG_UART_INT_ID_BUSY_DETECT)\
                            || ((ID) == LOG_UART_INT_ID_CHAR_TIMEOUT))
/**
  * @}
  */

/** @defgroup LOG_UART_Flag Log UART Flag 
  * @{
  */
  
#define LOG_UART_FLAG_RX_DATA_RDY                      ((uint16_t)(1 << 0))
#define LOG_UART_FLAG_RX_OVERRUN                       ((uint16_t)(1 << 1))
#define LOG_UART_FLAG_PARTY_ERR                        ((uint16_t)(1 << 2))
#define LOG_UART_FLAG_FRAME_ERR                        ((uint16_t)(1 << 3))
#define LOG_UART_FLAG_BREAK_ERR                        ((uint16_t)(1 << 4))
#define LOG_UART_FLAG_THR_EMPTY                        ((uint16_t)(1 << 5))     //Transmitter Holding Register or Transmitter FIFO empty
#define LOG_UART_FLAG_THR_TSR_EMPTY                    ((uint16_t)(1 << 6))     //Transmitter Holding Register(or tx FIFO) and Transmitter shift Register both empty
#define LOG_UART_FLAG_RX_FIFO_ERR                      ((uint16_t)(1 << 7))

#define IS_LOG_UART_GET_FLAG(FLAG) (((FLAG) == LOG_UART_FLAG_RX_DATA_RDY)   || ((FLAG) == LOG_UART_FLAG_RX_OVERRUN)\
                                    || ((FLAG) == LOG_UART_FLAG_PARTY_ERR) || ((FLAG) == LOG_UART_FLAG_FRAME_ERR)\
                                    || ((FLAG) == LOG_UART_FLAG_BREAK_ERR) || ((FLAG) == LOG_UART_FLAG_THR_EMPTY)\
                                    || ((FLAG) == LOG_UART_FLAG_THR_TSR_EMPTY) || ((FLAG) == LOG_UART_FLAG_RX_FIFO_ERR))
/**
  * @}
  */

/** @defgroup LOG_UART_RX_FIFO_Level Log UART RX FIFO Level 
  * @{
  */

#define LOG_UART_RX_FIFO_TRIGGER_LEVEL_1BYTE            ((uint16_t)(0x00 << 6))
#define LOG_UART_RX_FIFO_TRIGGER_LEVEL_4BYTE            ((uint16_t)(0x01 << 6))
#define LOG_UART_RX_FIFO_TRIGGER_LEVEL_8BYTE            ((uint16_t)(0x02 << 6))
#define LOG_UART_RX_FIFO_TRIGGER_LEVEL_14BYTE           ((uint16_t)(0x03 << 6))

#define IS_LOG_UART_RX_FIFO_TRIGGER_LEVEL(LEVEL) (((LEVEL) == LOG_UART_RX_FIFO_TRIGGER_LEVEL_1BYTE) || (LEVEL == LOG_UART_RX_FIFO_TRIGGER_LEVEL_4BYTE)\
                                || ((LEVEL) == LOG_UART_RX_FIFO_TRIGGER_LEVEL_8BYTE) || (LEVEL == LOG_UART_RX_FIFO_TRIGGER_LEVEL_14BYTE))
/**
  * @}
  */
  
/** @defgroup LOG_UART_TX_FIFO_Level Log UART TX FIFO Level 
  * @{
  */

#define LOG_UART_TX_FIFO_TRIGGER_LEVEL_EMPTY            ((uint16_t)(0x00 << 4))
#define LOG_UART_TX_FIFO_TRIGGER_LEVEL_2BYTE            ((uint16_t)(0x01 << 4))
#define LOG_UART_TX_FIFO_TRIGGER_LEVEL_4BYTE            ((uint16_t)(0x02 << 4))
#define LOG_UART_TX_FIFO_TRIGGER_LEVEL_8BYTE            ((uint16_t)(0x03 << 4))

#define IS_LOG_UART_TX_FIFO_TRIGGER_LEVEL(LEVEL) (((LEVEL) == LOG_UART_TX_FIFO_TRIGGER_LEVEL_EMPTY) || (LEVEL == LOG_UART_TX_FIFO_TRIGGER_LEVEL_2BYTE)\
                                || ((LEVEL) == LOG_UART_TX_FIFO_TRIGGER_LEVEL_4BYTE) || (LEVEL == LOG_UART_TX_FIFO_TRIGGER_LEVEL_8BYTE))
/**
  * @}
  */

/** @defgroup LOG_UART_Parity UART Parity 
  * @{
  */

#define LOG_UART_PARITY_NO_PARTY                        ((uint16_t)(0x00 << 3))
#define LOG_UART_PARITY_ODD                             ((uint16_t)(0x01 << 3))
#define LOG_UART_PARITY_EVEN                            ((uint16_t)(0x03 << 3))

#define IS_LOG_UART_PARITY(PARITY) (((PARITY) == LOG_UART_PARITY_NO_PARTY) || ((PARITY) == LOG_UART_PARITY_ODD)\
                                || ((PARITY) == LOG_UART_PARITY_EVEN))
/**
  * @}
  */

/** @defgroup LOG_UART_DMA_Mode Log UART DMA Mode
  * @{
  */

#define LOG_UART_DMA_MODE0                             ((uint16_t)(0 << 3))
#define LOG_UART_DMA_MODE1                             ((uint16_t)(1 << 3))

#define IS_LOG_UART_DMA_MODE(MODE) (((MODE) == LOG_UART_DMA_MODE0) || ((MODE) == LOG_UART_DMA_MODE1))
/**
  * @}
  */
  
/** @defgroup LOG_UART_FIFO_Ctrl Log UART FIFO Ctrl
  * @{
  */

#define LOG_UART_FIFO_EN                              ((uint16_t)(1 << 0))
#define LOG_UART_FIFO_DIS                             ((uint16_t)(0 << 0))

#define IS_LOG_UART_FIFO_CTRL(CTRL) (((CTRL) == LOG_UART_FIFO_EN) || ((CTRL) == LOG_UART_FIFO_DIS))
/**
  * @}
  */

/** @defgroup LOG_UART_Hardware_Flow_Control UART Hardware Flow Control 
  * @{
  */

#define LOG_UART_AUTO_FLOW_CTRL_EN                      ((uint16_t)((1 << 5) | (1 << 1)))
#define LOG_UART_AUTO_FLOW_CTRL_DIS                     ((uint16_t)0x00)

#define IS_LOG_UART_AUTO_FLOW_CTRL(CTRL) (((CTRL) == LOG_UART_AUTO_FLOW_CTRL_EN) || ((CTRL) == LOG_UART_AUTO_FLOW_CTRL_DIS))

/**
  * @}
  */

/** @defgroup LOG_UART_Wrod_Length UART Wrod Length 
  * @{
  */

#define LOG_UART_WROD_LENGTH_5BIT                       ((uint16_t)(0 << 0))
#define LOG_UART_WROD_LENGTH_6BIT                       ((uint16_t)(1 << 0))
#define LOG_UART_WROD_LENGTH_7BIT                       ((uint16_t)(1 << 1))
#define LOG_UART_WROD_LENGTH_8BIT                       ((uint16_t)(3 << 0))

#define IS_LOG_UART_WORD_LENGTH(LEN) (((LEN) == LOG_UART_WROD_LENGTH_5BIT) || ((LEN) == LOG_UART_WROD_LENGTH_6BIT)\
                                || ((LEN) == LOG_UART_WROD_LENGTH_7BIT) || ((LEN) == LOG_UART_WROD_LENGTH_8BIT))
/**
  * @}
  */

/** @defgroup LOG_UART_Stop_Bits UART Stop Bits
  * @note 1.5bit stop bits only valid when data length is 5bit
  * @{
  */

#define LOG_UART_STOP_BITS_1                            ((uint16_t)(0 << 2))
#define LOG_UART_STOP_BITS_1_5                          ((uint16_t)(1 << 2))
#define LOG_UART_STOP_BITS_2                            ((uint16_t)(1 << 2))
#define IS_LOG_UART_STOPBITS(STOP) (((STOP) == LOG_UART_STOP_BITS_2) || ((STOP) == LOG_UART_STOP_BITS_1) || ((STOP) == LOG_UART_STOP_BITS_1_5))
/**
  * @}
  */

/** @cond private
  * @defgroup Uart_Tx_Rx_FIFO_CLEAR_BIT Uart TRx Fifo Clear Bits
  * @{
  */
  
#define FCR_CLEAR_RX_FIFO_Set           ((uint32_t)(1 << 1))
#define FCR_CLEAR_RX_FIFO_Reset         ((uint32_t)~(1 << 1))

#define FCR_CLEAR_TX_FIFO_Set           ((uint32_t)(1 << 2))
#define FCR_CLEAR_TX_FIFO_Reset         ((uint32_t)~(1 << 2))

/**
  * @}
  * @endcond
  */

/** @defgroup LOG_UART_Exported_Global_Variables UART Exported Global Variables
  * @{
  */

/**
  * @}
  */

/**
  * @}
  */

/** @defgroup LOG_UART_Exported_Functions
  * @{
  */

extern void LOG_UART_Init(LOG_UART_TypeDef* UARTx, LOG_UART_InitTypeDef* LOG_UART_InitStruct);
extern void LOG_UART_DeInit(LOG_UART_TypeDef* UARTx);
extern void LOG_UART_StructInit(LOG_UART_InitTypeDef* LOG_UART_InitStruct);
extern void LOG_UART_ReceiveData(LOG_UART_TypeDef* UARTx, uint8_t* outBuf, uint16_t count);
extern void LOG_UART_SendData(LOG_UART_TypeDef* UARTx, const uint8_t* inBuf, uint16_t count);
__STATIC_INLINE void LOG_UART_INTConfig(LOG_UART_TypeDef* UARTx, uint32_t LOG_UART_IT, FunctionalState newState);
__STATIC_INLINE FlagStatus LOG_UART_GetFlagState(LOG_UART_TypeDef* UARTx, uint32_t LOG_UART_FLAG);
__STATIC_INLINE void LOG_UART_ClearTxFifo(LOG_UART_TypeDef* UARTx);
__STATIC_INLINE void LOG_UART_ClearRxFifo(LOG_UART_TypeDef* UARTx);
__STATIC_INLINE uint8_t LOG_UART_ReceiveByte(LOG_UART_TypeDef* UARTx);
__STATIC_INLINE void LOG_UART_SendByte(LOG_UART_TypeDef* UARTx, uint8_t data);
__STATIC_INLINE uint16_t LOG_UART_GetIID(LOG_UART_TypeDef* UARTx);
__STATIC_INLINE uint8_t LOG_UART_GetTxFifoLevel(LOG_UART_TypeDef* UARTx);

/**
  * @brief  Send one byte to tx FIFO.
  * @param  UARTx: selected UART peripheral.
  * @param  data: byte to send.
  * @retval None
  */
__STATIC_INLINE void LOG_UART_SendByte(LOG_UART_TypeDef* UARTx, uint8_t data)
{
    /* Check the parameters */
    assert_param(IS_LOG_UART_PERIPH(UARTx));
    
    UARTx->RB_THR_DLL = data;
    
    return;
}

/**
  * @brief  read one byte in rx FIFO.
  * @param  UARTx: selected UART peripheral.
  * @retval the byte read.
  */
__STATIC_INLINE uint8_t LOG_UART_ReceiveByte(LOG_UART_TypeDef* UARTx)
{
    /* Check the parameters */
    assert_param(IS_LOG_UART_PERIPH(UARTx));
    
    return (uint8_t)(UARTx->RB_THR_DLL);
}

/**
  * @brief  Get interrupt identifier.
  * @param  UARTx: selected UART peripheral.
  * @retval The interrupt identifier value.
  *   This return value can be one of the following values:
  *     @arg LOG_UART_INT_ID_LINE_STATUS: interrupt identifier--line status interrupt.
  *     @arg LOG_UART_INT_ID_RX_LEVEL_REACH: interrupt identifier--rx trigger level reached interrupt.
  *     @arg LOG_UART_INT_ID_RX_TMEOUT: interrupt identifier--line status interrupt.
  *     @arg LOG_UART_INT_ID_TX_EMPTY: interrupt identifier--line status interrupt.
  *     @arg LOG_UART_INT_ID_MODEM_STATUS: interrupt identifier--line status interrupt.
  */
__STATIC_INLINE uint16_t LOG_UART_GetIID(LOG_UART_TypeDef* UARTx)
{
    /* Check the parameters */
    assert_param(IS_LOG_UART_PERIPH(UARTx));
    
    return (uint16_t)(UARTx->INTID_FCR & (0x0000000E));
}


/**
  * @brief  Clear UART tx FIFO.
  * @param  UARTx: selected UART peripheral.
  * @retval None
  */
__STATIC_INLINE void LOG_UART_ClearTxFifo(LOG_UART_TypeDef* UARTx)
{
    /* Check the parameters */
    assert_param(IS_LOG_UART_PERIPH(UARTx));

    UARTx->INTID_FCR |= FCR_CLEAR_TX_FIFO_Set;

    return;
}

/**
  * @brief  Clear UART rx FIFO.
  * @param  UARTx: selected UART peripheral.
  * @retval None
  */
__STATIC_INLINE void LOG_UART_ClearRxFifo(LOG_UART_TypeDef* UARTx)
{
    /* Check the parameters */
    assert_param(IS_LOG_UART_PERIPH(UARTx));

    UARTx->INTID_FCR |= FCR_CLEAR_RX_FIFO_Set;

    return;
}

/**
  * @brief  Checks whether the specified UART flag is set or not.
  * @param  UARTx: selected UART peripheral.
  * @param  LOG_UART_FLAG: specifies the flag to check. 
  *   This parameter can be one of the following values:
  *     @arg LOG_UART_FLAG_RX_DATA_RDY: rx data is avaliable.
  *     @arg LOG_UART_FLAG_RX_OVERRUN: rx overrun.
  *     @arg LOG_UART_FLAG_PARTY_ERR: parity error.
  *     @arg LOG_UART_FLAG_FRAME_ERR: UARTx frame error.
  *     @arg LOG_UART_FLAG_BREAK_ERR: UARTx break error.
  *     @arg LOG_UART_FLAG_THR_EMPTY: tx FIFO is empty.
  *     @arg LOG_UART_FLAG_THR_TSR_EMPTY: tx FIFO and tx shift reg are both empty.
  *     @arg LOG_UART_FLAG_RX_FIFO_ERR: rx FIFO error.
  * @retval The new state of LOG_UART_FLAG (SET or RESET).
  */
__STATIC_INLINE FlagStatus LOG_UART_GetFlagState(LOG_UART_TypeDef* UARTx, uint32_t LOG_UART_FLAG)
{
    FlagStatus bitstatus = RESET;

    /* Check the parameters */
    assert_param(IS_LOG_UART_PERIPH(UARTx));
    assert_param(IS_LOG_UART_GET_FLAG(LOG_UART_FLAG));
    
    if(UARTx->LSR & LOG_UART_FLAG)
    {
        bitstatus = SET;
    }

    return bitstatus;
}

/**
  * @brief  Enables or disables the specified Log UART interrupts.
  * @param  UARTx: selected UARTx peripheral.
  * @param  LOG_UART_IT: specifies the Log UART interrupts sources to be enabled or disabled. 
  *   This parameter can be any combination of the following values:
  *     @arg LOG_UART_INT_RD_AVA: enable Rx data avaliable interrupt.
  *     @arg LOG_UART_INT_FIFO_EMPTY: enable FIFO empty interrupt.  
  *     @arg LOG_UART_INT_LINE_STS: enable line status interrupt.
  *     @arg LOG_UART_INT_MODEM_STS: enable modem status interrupt.
  * @param  NewState: new state of the specified UART interrupts.
  *   This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
__STATIC_INLINE void LOG_UART_INTConfig(LOG_UART_TypeDef* UARTx, uint32_t LOG_UART_IT, FunctionalState newState)
{
    /* Check the parameters */
    assert_param(IS_LOG_UART_PERIPH(UARTx));
    assert_param(IS_FUNCTIONAL_STATE(newState));
    assert_param(IS_LOG_UART_IT(LOG_UART_IT));

    if (newState == ENABLE)
    {
        /* Enable the selected UARTx interrupts */
        UARTx->DLH_INTCR |= LOG_UART_IT;
    }
    else
    {
        /* Disable the selected UARTx interrupts */
        UARTx->DLH_INTCR &= (uint32_t)~LOG_UART_IT;
    }

    return;
}

/**
  * @brief  Get the bytes number in tx fifo.
  * @param  UARTx: selected Log UARTx peripheral.
  * @return Current Tx FIFO level.
  * @retval None
  */
uint8_t LOG_UART_GetTxFifoLevel(LOG_UART_TypeDef* UARTx)
{
    assert_param(IS_LOG_UART_PERIPH(UARTx));
    
    return UARTx->TFLR;
}

#ifdef __cplusplus
}
#endif

#endif /* _LOG_UART_H_ */

/**
  * @}
  */ 

/**
  * @}
  */ 

/**
  * @}
  */ 

/******************* (C) COPYRIGHT 2015 Realtek Semiconductor *****END OF FILE****/



