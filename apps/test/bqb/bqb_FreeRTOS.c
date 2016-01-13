enum { __FILE_NUM__= 0 };

/**********************************************************************!KA****
 *
 * $Header: /var/lib/cvs/sw/src/app/demo/gattdemo/gattdemo_FreeRTOS.c,v 1.4 2013/12/06 12:42:15 mn Exp $
 *
 * File:        $RCSfile: gattdemo_FreeRTOS.c,v $
 * Version:     $Name: P_SRP1290_V1_0 $
 *
 * Archive:     $Source: /var/lib/cvs/sw/src/app/demo/gattdemo/gattdemo_FreeRTOS.c,v $
 * Revision:    $Revision: 1.4 $
 * Date:        $Date: 2013/12/06 12:42:15 $
 * Author:      $Author: mn $
 *
 * ---------------------------------------------------------------------------
 * !MODULE      [  ]
 * ---------------------------------------------------------------------------
 * !FILE        [  ]
 * !PROGRAM     [  ]
 * !VERSION     [$Name: P_SRP1290_V1_0 $]
 * !GROUP       [  ]
 * !AUTHOR      [$Author: mn $]
 * ---------------------------------------------------------------------------
 *
 *          Copyright (c)           2012 Stollmann E+V GmbH
 *                                  Mendelssohnstr. 15
 *                                  22761 Hamburg
 *                                  Phone: 040/89088-0
 *          The content of this file is Stollmann E+V GmbH confidential
 *          All Rights Reserved
 *
 * ---------------------------------------------------------------------------
 * !DESCRIPTION
 *
 *         GATTDEMO (BlueAPI/GATT API demo) command handling
 *         Use USART1 - 115200 8N1
 * ---------------------------------------------------------------------------
 * !INDEX
 *  ...
 * ---------------------------------------------------------------------------
 * !CONTENTS
 * ---------------------------------------------------------------------------
 * !INCLUDE_REFERENCES
 * ---------------------------------------------------------------------------
 * !HISTORY
 *  Date      Author          Comment
 *  tt.mm.jj                  Initial revision
 *  tt.mm.jj
 ************************************************************************!KA*/


#include "bqb_demo.h"
#include "bqb_cmd.h"

#include <blueapi.h>

#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>

/* for bee  */
//#include "platform_autoconf.h"
#include "rtl_types.h"
#include "trace.h"
//#include "section_config.h"
#include "cycle_queue.h"
#include "trace.h"
#include "bqb.h"
//#include "rtl876x.h"
//#include "rtl876x_rcc.h"
//#include "rtl876x_uart.h"
//#include "rtl876x_pinmux.h"
//#include "rtl876x_nvic.h"
#include "stm32f10x.h"
/****************************************************************************/
/* Events                                                                   */
/****************************************************************************/

#define DEMO_EVENT_UART_RX             0x01
#define DEMO_EVENT_BLUEAPI_MSG         0x02


/****************************************************************************/
/* GattDemo interface                                                       */
/****************************************************************************/

xTaskHandle stGattBQBHandle;

#define MAX_NUMBER_OF_RX_EVENT     0x20
#define MAX_NUMBER_OF_MESSAGE      0x20

#define UART_RX_QUEUE_LENGTH       0x40
#define UART_TX_QUEUE_LENGTH       0x800

static xQueueHandle gattdQueueHandleEvent;
static xQueueHandle gattdQueueHandleMessage;
static xQueueHandle gattdQueueHandleUartRx;

/* globals */
TGATTDemo BQB_Demo /*SRAM_OFF_BD_BSS_SECTION*/;

/* static functions */
static void BQB_UARTInit(void);

/****************************************************************************/
/* Fatal error                                                              */
/****************************************************************************/

static void BQB_FatalError(int Line)
{
//  printf("GATTD: Fatal error (line %d)\n\r", Line);
}


/****************************************************************************/
/* retarget printf                                                          */
/****************************************************************************/

int BQB_Sendchar(int ch)
{
#if 0
    while(UART_GetFlagState(UART, UART_FLAG_THR_TSR_EMPTY) != SET);
    UART_SendByte(UART, ch);
#else
    while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET) {}
    USART_SendData(USART1, ch);
#endif
    return (ch);
}

int BQB_VSprintf(char *buf, const char *fmt, const int *dp)
{
    char *p, *s;

    s = buf;
    for ( ; *fmt != '\0'; ++fmt)
    {
        if (*fmt != '%')
        {
            buf ? *s++ = *fmt : BQB_Sendchar(*fmt);
            continue;
        }
        if (*++fmt == 's')
        {
            for (p = (char *)*dp++; *p != '\0'; p++)
                buf ? *s++ = *p : BQB_Sendchar(*p);
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
#if 0
            else if (*fmt == '@')
            {
                unsigned char *r;
                union
                {
                    long        l;
                    unsigned char   c[4];
                } u;
                const long *lp = (const long *)dp;
                u.l = *lp++;
                dp = (const int *)lp;
                for (r = &u.c[0]; r < &u.c[4]; ++r)
                    q += SprintF(q, "%d.", *r);
                --q;
            }
#endif
#if 0
            else if (*fmt == '!')
            {
                char *r;
                p = (char *)*dp++;
                for (r = p + ETH_ALEN; p < r; ++p)
                    q += SprintF(q, "%hhX:", *p);
                --q;
            }
#endif
            else if (*fmt == 'c')
                *q++ = *dp++;
            else
                *q++ = *fmt;
            /* now output the saved string */
            for (p = tmp; p < q; ++p)
                buf ? *s++ = *p : BQB_Sendchar(*p);
        }
    }
    if (buf)
        *s = '\0';
    return (s - buf);
}


int BQB_Print(IN  char *fmt, ...)
{
    (void)BQB_VSprintf(0, fmt, ((const int *)&fmt) + 1);
    return 0;
}

/****************************************************************************/
/* UART interrupt                                                           */
/****************************************************************************/
//void BQB_UartIrqHandle(void)
void USART1_IRQHandler(void)
{
#if 0
    uint8_t rx_data = 0;
    portBASE_TYPE TaskWoken = pdFALSE;
    uint8_t event  = DEMO_EVENT_UART_RX;
    uint32_t int_status = 0;
    
    /* read interrupt id */
    int_status = UART_GetIID(UART);
    /* disable interrupt */
    UART_INTConfig(UART, UART_INT_RD_AVA | UART_INT_LINE_STS, DISABLE);
    
    switch(int_status)
    {
        /* tx fifo empty */
        case UART_INT_ID_TX_EMPTY:
            /* do nothing */
        break;

        /* rx data valiable */
        case UART_INT_ID_RX_LEVEL_REACH:
        case UART_INT_ID_RX_TMEOUT:
            while(UART_GetFlagState(UART, UART_FLAG_RX_DATA_RDY) == SET)
            {
                UART_ReceiveData(UART, &rx_data, 1);
            }
            
            xQueueSendFromISR(gattdQueueHandleUartRx, &rx_data, &TaskWoken);
            xQueueSendFromISR(gattdQueueHandleEvent, &event, &TaskWoken);     // signal event to GATTDEMO task
        break;
        
        /* receive line status interrupt */
        case UART_INT_ID_LINE_STATUS:
            DBG_BUFFER(MODULE_APP, LEVEL_ERROR, "Line status error!!!!\n", 0);
        break;      

        default:
        break;
    }
    
    /* enable interrupt again */
    UART_INTConfig(UART, UART_INT_RD_AVA | UART_INT_LINE_STS, ENABLE);
    portEND_SWITCHING_ISR(TaskWoken);
#else
    portBASE_TYPE TaskWoken = pdFALSE;
    uint8_t RxChar;
    uint8_t event  = DEMO_EVENT_UART_RX;

    if (USART_GetITStatus(USART1, USART_IT_RXNE) == SET)
    {


    RxChar = USART_ReceiveData(USART1);

    xQueueSendFromISR(gattdQueueHandleUartRx, &RxChar, &TaskWoken);
    xQueueSendFromISR(gattdQueueHandleEvent, &event, &TaskWoken);
    }

    portEND_SWITCHING_ISR(TaskWoken);
#endif
    return;
}

/****************************************************************************/
/* UART init                                                                */
/****************************************************************************/
//extern IRQ_FUN UserIrqFunTable[32+17];
static void BQB_UARTInit(void)
{
#if 0
    /* enable uart clock */
    RCC_PeriphClockCmd(APBPeriph_UART, APBPeriph_UART_CLOCK, ENABLE);
    
    //pinmux config
    Pinmux_Config(P2_4, DATA_UART_TX);
    Pinmux_Config(P2_5, DATA_UART_RX);
    
    //pad config
    Pad_Config(P2_4, PAD_PINMUX_MODE, PAD_IS_PWRON, PAD_PULL_NONE, PAD_OUT_DISABLE, PAD_OUT_LOW);
    Pad_Config(P2_5, PAD_PINMUX_MODE, PAD_IS_PWRON, PAD_PULL_NONE, PAD_OUT_DISABLE, PAD_OUT_LOW);
    
    //uart interrupt config
    UserIrqFunTable[UART_IRQ]		        = (IRQ_FUN)BQB_UartIrqHandle;
    
    //uart init 
    UART_InitTypeDef uartInitStruct;
    UART_StructInit(&uartInitStruct);
    
    uartInitStruct.rxTriggerLevel = UART_RX_FIFO_TRIGGER_LEVEL_1BYTE;
    UART_Init(UART, &uartInitStruct);
    //enable line status interrupt and rx data avaliable interrupt
    UART_INTConfig(UART, UART_INT_RD_AVA | UART_INT_LINE_STS, ENABLE);
    
    /*  Enable UART IRQ  */
    NVIC_InitTypeDef NVIC_InitStruct; 
    NVIC_InitStruct.NVIC_IRQChannel = UART_IRQ;
    NVIC_InitStruct.NVIC_IRQChannelPriority = 1;
    NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
    
    NVIC_Init(&NVIC_InitStruct);
#else
    USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
    GPIO_InitTypeDef GPIO_InitStructure;

    //test_upperstack_QueueHandleTransportRx = xQueueCreate(UART_RX_QUEUE_LENGTH, sizeof(char));
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
#endif
    return;
}


/****************************************************************************/
/* BlueAPI Callback.                                                        */
/****************************************************************************/

void BQB_BlueAPICallback(PBlueAPI_UsMessage pMsg)
{
    unsigned char Event = DEMO_EVENT_BLUEAPI_MSG;

    if (xQueueSend(gattdQueueHandleMessage, &pMsg, 0) == errQUEUE_FULL)
    {
        BQB_FatalError(__LINE__);

        blueAPI_BufferRelease(pMsg);
    }
    else if (xQueueSend(gattdQueueHandleEvent, &Event, 0) == errQUEUE_FULL)     /* signal event to GATTDEMO task */
    {
        BQB_FatalError(__LINE__);
    }
}


/****************************************************************************/
/* TASK                                                                     */
/****************************************************************************/

void BQB_Task(void *pParameters)
{
    char RxChar;
    char Event;

    if (!BQB_SelectDatabase(SMALL_DATABASE))  /* small database default */
    {
        return;    /* to print information later */
    }
    /* init the informatons of services in database */
    if (!BQB_ServiceInfoInit())
    {
        return;
    }
    gattdQueueHandleEvent   = xQueueCreate((MAX_NUMBER_OF_MESSAGE + MAX_NUMBER_OF_RX_EVENT),
                                           sizeof(unsigned char));
    
    gattdQueueHandleMessage = xQueueCreate(MAX_NUMBER_OF_MESSAGE,
                                           sizeof(PBlueAPI_UsMessage));
    
    gattdQueueHandleUartRx = xQueueCreate(UART_RX_QUEUE_LENGTH, sizeof(char));
    
    BQB_UARTInit();
    BQB_CmdInit(&BQB_Demo);

    while (true)
    {
        if (xQueueReceive(gattdQueueHandleEvent, &Event, portTICK_RATE_MS * 1000) == pdPASS)
        {
            if (Event & DEMO_EVENT_UART_RX)      /* User interface */
            {
                while (xQueueReceive(gattdQueueHandleUartRx, &RxChar, 0) == pdPASS)
                {
                    BQB_CmdCollect(&BQB_Demo, &RxChar, sizeof(RxChar));
                }
            }
            
            if (Event & DEMO_EVENT_BLUEAPI_MSG)  /* BlueAPI */
            {
                PBlueAPI_UsMessage pMsg;

                while (xQueueReceive(gattdQueueHandleMessage, &pMsg, 0) == pdPASS)
                {
                    BQB_HandleBlueAPIMessage(&BQB_Demo, pMsg);
                }
            }
        }
    }
}


/**
 * @brief bqb init, select bqb database, init the information of services in database, create BQB_Task.
 *
 * @param  none.
 * @return none
 * @retval void.
*/
void BQB_Init( void )
{
    xTaskCreate(BQB_Task, "GattBQB", GATTDEMO_STACK_SIZE / sizeof(portSTACK_TYPE),
                NULL, 2, &stGattBQBHandle);
}

