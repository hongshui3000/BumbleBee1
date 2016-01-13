#include "rtl_types.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "trace.h"

static xQueueHandle xQueue;

static void vSenderTask( void *pvParameters )
{
	long lVauleToSend;
	portBASE_TYPE xStatus;
	
	lVauleToSend = (long) pvParameters;
	
	for(;;)
	{
		xStatus = xQueueSendToBack(xQueue, &lVauleToSend, 0);
		if( xStatus != pdPASS )
		{
			//error
			configASSERT(0);
		}
		
		taskYIELD();
	}
}

static void vReceiverTask( void *pvParameters )
{
	long lReceivedValue;
	portBASE_TYPE xStatus;
	const portTickType xTicksToWait = 100 / portTICK_RATE_MS;
	
	for( ;; )
	{
		xStatus = xQueueReceive(xQueue, &lReceivedValue, xTicksToWait);
		if( xStatus == pdPASS)
		{
			//print received data
			long result = lReceivedValue;
			DBG_BUFFER(SUBTYPE_FORMAT, MODULE_APP, LEVEL_INFO, "receive data: %d", 1, result);
			DBG_DIRECT("receive data: %d", result);
		}
		else
		{
			//error
			configASSERT(0);
		}
			
	}
	
}

void test_freertos(void)
{
	xQueue = xQueueCreate( 5, sizeof(long) );
	if(xQueue != NULL)
	{
		xTaskCreate(vSenderTask, "Sender1", 1000, (void*)100, 1, NULL);
		xTaskCreate(vSenderTask, "Sender2", 1000, (void*)200, 1, NULL);
	    
		xTaskCreate(vReceiverTask, "Receiver", 1000, NULL, 2, NULL);
	}
}
