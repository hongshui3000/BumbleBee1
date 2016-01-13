#include "test_transport_uart.h"

#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>
#include "board.h"
/* for bee uart io driver interface */
#include "rtl_types.h"
//#include "board.h"
//#include "rtl876x_uart.h"
//#include "rtl876x_pinmux.h"
//#include "rtl876x_io_dlps.h"


extern xQueueHandle test_upperstack_QueueHandleTransportRx;
extern xQueueHandle test_upperstack_QueueHandleEvent;

int transport_uart_sendchar(int ch)
{
    while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET) {}
    USART_SendData(USART1, ch);
    return (ch);
}

int transport_uart_VSprintf(char *buf, const char *fmt, const int *dp)
{
    char *p, *s;

    s = buf;
    for ( ; *fmt != '\0'; ++fmt)
    {
        if (*fmt != '%')
        {
            buf ? *s++ = *fmt : transport_uart_sendchar(*fmt);
            continue;
        }
        if (*++fmt == 's')
        {
            for (p = (char *)*dp++; *p != '\0'; p++)
                buf ? *s++ = *p : transport_uart_sendchar(*p);
        }
        else    /* Length of item is bounded */
        {
            char tmp[20], *q = tmp;
            int alt = 0;
            int shift = 28;

#if 1   //wei patch for %02x
            if ((*fmt  >= '0') && (*fmt  <= '9'))
            {
                int width;
                unsigned char fch = *fmt;
                for (width = 0; (fch >= '0') && (fch <= '9'); fch = *++fmt)
                {
                    width = width * 10 + fch - '0';
                }
                shift = (width - 1) * 4;
            }
#endif

            /*
             * Before each format q points to tmp buffer
             * After each format q points past end of item
             */

            if ((*fmt == 'x') || (*fmt == 'X') || (*fmt == 'p') || (*fmt == 'P'))
            {
                /* With x86 gcc, sizeof(long) == sizeof(int) */
                const long *lp = (const long *)dp;
                long h = *lp++;
                int ncase = (*fmt & 0x20);
                dp = (const int *)lp;
                if ((*fmt == 'p') || (*fmt == 'P'))
                    alt = 1;
                if (alt)
                {
                    *q++ = '0';
                    *q++ = 'X' | ncase;
                }
                for ( ; shift >= 0; shift -= 4)
                    * q++ = "0123456789ABCDEF"[(h >> shift) & 0xF] | ncase;
            }
            else if (*fmt == 'd')
            {
                int i = *dp++;
                char *r;
                if (i < 0)
                {
                    *q++ = '-';
                    i = -i;
                }
                p = q;      /* save beginning of digits */
                do
                {
                    *q++ = '0' + (i % 10);
                    i /= 10;
                }
                while (i);
                /* reverse digits, stop in middle */
                r = q;      /* don't alter q */
                while (--r > p)
                {
                    i = *r;
                    *r = *p;
                    *p++ = i;
                }
            }
            else if (*fmt == 'c')
                *q++ = *dp++;
            else
                *q++ = *fmt;
            /* now output the saved string */
            for (p = tmp; p < q; ++p)
                buf ? *s++ = *p : transport_uart_sendchar(*p);
        }
    }
    if (buf)
        *s = '\0';
    return (s - buf);
}




int
transport_uart_print(
    IN  char *fmt, ...
)
{
    (void)transport_uart_VSprintf(0, fmt, ((const int *)&fmt) + 1);
    return 0;
}

/****************************************************************************/
/* UART interrupt                                                           */
/****************************************************************************/
void USART1_IRQHandler(void)
{
  portBASE_TYPE TaskWoken = pdFALSE;
  uint8_t RxChar;
  uint8_t event  = DEMO_EVENT_UART_RX;

  if (USART_GetITStatus(USART1, USART_IT_RXNE) == SET)
  {


    RxChar = USART_ReceiveData(USART1);

    xQueueSendFromISR(test_upperstack_QueueHandleTransportRx, &RxChar, &TaskWoken);
    xQueueSendFromISR(test_upperstack_QueueHandleEvent, &event, &TaskWoken);
  }

  portEND_SWITCHING_ISR(TaskWoken);
}

/****************************************************************************/
/* UART init                                                                */
/****************************************************************************/
void test_upperstack_UARTInit(void)
{
  USART_InitTypeDef USART_InitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;
  GPIO_InitTypeDef GPIO_InitStructure;
  
  test_upperstack_QueueHandleTransportRx = xQueueCreate(UART_RX_QUEUE_LENGTH, sizeof(char));
  //hIoQueueHandle = xQueueCreate(UART_RX_QUEUE_LENGTH, sizeof(char));
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1 | RCC_APB2Periph_GPIOA, ENABLE);

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init( GPIOA, &GPIO_InitStructure );

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_Init( GPIOA, &GPIO_InitStructure );

  USART_InitStructure.USART_BaudRate = 115200;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No ;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

  USART_Init( USART1, &USART_InitStructure );

  USART_ITConfig( USART1, USART_IT_RXNE, ENABLE );
  USART_ITConfig( USART1, USART_IT_TXE, DISABLE );

  NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = configLIBRARY_KERNEL_INTERRUPT_PRIORITY;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init( &NVIC_InitStructure );

  USART_Cmd( USART1, ENABLE );
}
