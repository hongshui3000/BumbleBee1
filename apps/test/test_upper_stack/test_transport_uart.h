#ifndef _TEST_TRANSPORT_UART_H_
#define _TEST_TRANSPORT_UART_H_

#define DEMO_EVENT_UART_RX             0x01


#define UART_RX_QUEUE_LENGTH       0x40
#define UART_TX_QUEUE_LENGTH       0x800



void test_upperstack_UARTInit(void);
void TestUpperStackDataUartIrqHandle(void);

int
transport_uart_print(
    char *fmt, ...
);


#endif
