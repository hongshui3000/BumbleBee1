#ifndef _UART_H_
#define _UART_H_

//#if defined(ENABLE_LOGGER) || defined(DIRECT_UART_PRINT)

#include "platform.h"

#include <stdarg.h>
#include "DataType.h"

// UART0 address
#define UART_BASE_ADDR              0x40011000
#define UART_REV_BUF_OFF            0x00
#define UART_SEND_BUF_OFF           0x00
#define UART_DLH_OFF                0x04
#define UART_DLL_OFF                0x00
#define UART_INTERRUPT_EN_REG_OFF   0x04
#define UART_INTERRUPT_IDEN_REG_OFF 0x08
#define UART_FIFO_CTL_REG_OFF       0x08
#define UART_LINE_CTL_REG_OFF       0x0c
#define UART_MODEM_CTL_REG_OFF      0x10
#define UART_LINE_STATUS_REG_OFF    0x14
#define UART_MODEM_STATUS_REG_OFF   0x18
#define UART_FIFO_ACCESS_REG_OFF    0x70
#define UART_STATUS_REG_OFF         0x7c
#define UART_TFL_OFF                0x80
#define UART_RFL_OFF                0x84

#define UART_BAUD_RATE_2400         2400
#define UART_BAUD_RATE_4800         4800
#define UART_BAUD_RATE_9600         9600
#define UART_BAUD_RATE_19200        19200
#define UART_BAUD_RATE_38400        38400
#define UART_BAUD_RATE_57600        57600
#define UART_BAUD_RATE_115200       115200
#define UART_BAUD_RATE_921600       921600
#define UART_BAUD_RATE_1152000      1152000

#define UART_PARITY_ENABLE          0x08
#define UART_PARITY_DISABLE         0

#define UART_DATA_LEN_5BIT          0x0
#define UART_DATA_LEN_6BIT          0x1
#define UART_DATA_LEN_7BIT          0x2
#define UART_DATA_LEN_8BIT          0x3

#define UART_STOP_1BIT              0x0
#define UART_STOP_2BIT              0x4

typedef enum uart_state_e
{
	UART_NOT_INIT = 0,      /* UART Not initialized (it must be zero) */
	UART_FREE,
	UART_TRANSMITTING
}UART_STATE_E;

typedef PF_TP_TX_COMPLETED_CB  UART_TX_COMP_FUNC_PTR;
typedef PF_TP_BYTES_RECD_CB    UART_RX_FUNC_PTR;
typedef PF_UART_DBG_RECD_CB    UART_DBG_RECD_FUNC_PTR;

typedef struct uart_t
{
	UART_STATE_E            State;
	UART_TX_COMP_FUNC_PTR   TxCompFunc;
	UART_RX_FUNC_PTR        RxFunc;
	UCHAR                   *DataPtr;
	UINT16                  Len;
	UINT16                  LenWritten;
}UART_T;

#define REV_FIFO_LEN                256

extern UART_T  Uart;
extern UINT8 is_uart_init;
extern UINT16 uart_dbg_wptr;
extern UINT8 uart_dbg_data[REV_FIFO_LEN];


#define UART_READ(y)      RD_32BIT_IO(UART_BASE_ADDR, y)
#define UART_WRITE(y, z)  WR_32BIT_IO(UART_BASE_ADDR, y, z)


UINT32 UARTInit(UINT32 BaudRate, UINT8 parity, UINT8 stop, UINT8 DataLength,
				UINT32 FIFOControl, UINT32 IntEnReg);
UINT32 UARTBytePut(UINT8 data);

BOOLEAN S3C2410UartTransWr(HCI_TRANSPORT_PKT_TYPE pkt_type, UCHAR *buf, UINT16 len);
void MINT_trans_uart_tx(void);
BOOLEAN uart_trans_transport_is_free(void);

#ifdef ENABLE_LOGGER

BOOLEAN S3C2410UartLogInit(UART_TX_COMP_FUNC_PTR tx_comp_func, UART_DBG_RECD_FUNC_PTR dbg_rx_func);
BOOLEAN S3C2410UartLogWr(UCHAR *buf, UINT16 len);
void MINT_logger_uart_tx(void);
BOOLEAN uart_logger_transport_is_free(void);
#endif // ENABLE_LOGGER

#ifdef _DONT_USE_LOG_UART_TX_INT_
UINT8 uart_print_buf(void);
#endif

//#endif //defined(ENABLE_LOGGER) || defined(DIRECT_UART_PRINT)

#endif  /* _UART_H_ */

