/**
*********************************************************************************************************
*               Copyright(c) 2015, Realtek Semiconductor Corporation. All rights reserved.
**********************************************************************************************************
* @file     application.c
* @brief    .
* @details
* @author   kyle_xu
* @date     2015-11-25
* @version  v0.2
*********************************************************************************************************
*/

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include "legacy.h"
#include "bee_message.h"
#include "application.h"
#include "test_avrcp_application.h"


/****************************************************************************/
/* Events                                                                   */
/****************************************************************************/
#define BLUEAPI_MSG_EVENT       0x01
#define EVENT_UART_RX       0x09    //dqy
#define EVENT_IODRIVER_TO_APP   0x05

#define MAX_NUMBER_OF_RX_EVENT     0x20
#define MAX_NUMBER_OF_MESSAGE      0x20

xTaskHandle  hLegacyAppTaskHandle;
xQueueHandle hEventQueueHandle;
xQueueHandle hMessageQueueHandle;
xQueueHandle hIoQueueHandle;

void bee_task_app(void *pvParameters);
void application_task_init()
{
    /* Create APP Task. */
    xTaskCreate(bee_task_app,           /* Pointer to the function that implements the task. */
                "APPTask",              /* Text name for the task.  This is to facilitate debugging only. */
                512,                    /* Stack depth in words. 1KB*/
                NULL,                   /* We are not using the task parameter. */
                1,                      /* This task will run at priority 1. */
                &hLegacyAppTaskHandle   /*  the task handle. */
               );
}


/**
* @brief
*
*
* @param   pvParameters
* @return  void
*/
void bee_task_app(void *pvParameters)
{
    char Event;

    hMessageQueueHandle = xQueueCreate(MAX_NUMBER_OF_MESSAGE, sizeof(PBlueAPI_UsMessage));

    hIoQueueHandle = xQueueCreate(MAX_NUMBER_OF_MESSAGE, sizeof(BEE_IO_MSG));

    hEventQueueHandle = xQueueCreate(MAX_NUMBER_OF_RX_EVENT, sizeof(unsigned char));

    legacy_StartBtStack();

    while (true)
    {
        if (xQueueReceive(hEventQueueHandle, &Event, portMAX_DELAY) == pdPASS)
        {
            if (Event == BLUEAPI_MSG_EVENT)  /* BlueAPI */
            {
                PBlueAPI_UsMessage pMsg;

                while (xQueueReceive(hMessageQueueHandle, &pMsg, 0) == pdPASS)
                {
                    legacy_HandleBlueAPIMessage(pMsg);
                }
            } else if (Event == EVENT_NEWIODRIVER_TO_APP)
            {
                BEE_IO_MSG io_driver_msg_recv;

                while (xQueueReceive(hIoQueueHandle, &io_driver_msg_recv, 0) == pdPASS)
                {
                    AppHandleIODriverMessage(io_driver_msg_recv);
                }
            } else if (Event == EVENT_UART_RX)
            {
                extern xQueueHandle UartRxQueue;
                uint8_t rx;
                while (xQueueReceive(UartRxQueue, &rx, 0) == pdPASS)
                {
                    void avrcp_app_sim_cmd(char rx);
                    avrcp_app_sim_cmd(rx);
                }
            }
        }
    }
}

