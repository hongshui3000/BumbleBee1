/**
*********************************************************************************************************
*               Copyright(c) 2015, Realtek Semiconductor Corporation. All rights reserved.
**********************************************************************************************************
* @file     application.c
* @brief    .
* @details
* @author   ranhui
* @date     2015-03-27
* @version  v0.2
*********************************************************************************************************
*/
//#include "rtl876x.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include "application.h"

#include "peripheral.h"
#include "central.h"
#include "broadcaster.h"
#include "observer.h"

#include "blueapi.h"

#include "test_peripheral_application.h"

TBlueAPIHandle   GAP_BlueAPIHandle;  /* application handle provided by BlueAPI, added by Ethan. */

extern uint8_t  gapPara_profileRole;

/****************************************************************************/
/* Events                                                                   */
/****************************************************************************/
#define BLUEAPI_MSG_EVENT        0x01
#define EVENT_IODRIVER_TO_APP                   0x05

#define BROADCASTER_TASK_PRIORITY          (tskIDLE_PRIORITY + 1)   /* Task priorities. */
#define BROADCASTER_TASK_STACK_SIZE        1024*2

#define MAX_NUMBER_OF_RX_EVENT     0x20
#define MAX_NUMBER_OF_MESSAGE      0x20

xTaskHandle  hMouseAppTaskHandle;
xQueueHandle hEventQueueHandle;
xQueueHandle hMessageQueueHandle;
xQueueHandle hIoQueueHandle;

void bee_task_app(void *pvParameters );
void application_task_init()
{

    /* Create APP Task. */
    xTaskCreate(bee_task_app,    /* Pointer to the function that implements the task. */
                "APPTask",           /* Text name for the task.  This is to facilitate debugging only. */
                256,                 /* Stack depth in words. 1KB*/
                NULL,               /* We are not using the task parameter. */
                1,                  /* This task will run at priority 1. */
                &hMouseAppTaskHandle );             /*  the task handle. */
}


/**
* @brief
*
*
* @param   pvParameters
* @return  void
*/
void bee_task_app(void *pvParameters )
{
    char Event;

    hMessageQueueHandle = xQueueCreate(MAX_NUMBER_OF_MESSAGE,
                                       sizeof(PBlueAPI_UsMessage));

    hIoQueueHandle = xQueueCreate(MAX_NUMBER_OF_MESSAGE,
                                  sizeof(BEE_IO_MSG));

    hEventQueueHandle = xQueueCreate(MAX_NUMBER_OF_RX_EVENT,
                                     sizeof(unsigned char));

    GAP_StartBtStack();

    while (true)
    {
        if (xQueueReceive(hEventQueueHandle, &Event, portMAX_DELAY) == pdPASS)
        {
            if (Event == BLUEAPI_MSG_EVENT)  /* BlueAPI */
            {
                PBlueAPI_UsMessage pMsg;

                while (xQueueReceive(hMessageQueueHandle, &pMsg, 0) == pdPASS)
                {
                    switch(gapPara_profileRole)
                    {
                        case GAP_PROFILE_PERIPHERAL:
                            peripheral_HandleBlueAPIMessage(pMsg);
                            break;
                        case GAP_PROFILE_CENTRAL:
                            central_HandleBlueAPIMessage(pMsg);
                            break;
                        case GAP_PROFILE_BROADCASTER:
                            broadcaster_HandleBlueAPIMessage(pMsg);
                            break;
                        case GAP_PROFILE_OBSERVER:
                            observer_HandleBlueAPIMessage(pMsg);
                            break;
                    }
                }
            }
            else if (Event == EVENT_NEWIODRIVER_TO_APP)
            {
                BEE_IO_MSG io_driver_msg_recv;
                while (xQueueReceive( hIoQueueHandle, &io_driver_msg_recv, 0 ) == pdPASS)
                {
                    AppHandleIODriverMessage(io_driver_msg_recv);
                }
            }
        }
    }
}

