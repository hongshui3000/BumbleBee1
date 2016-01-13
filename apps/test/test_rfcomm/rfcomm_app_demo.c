

//#include <rtl_memory.h>
#include <rfc_api.h>
#include "my_stdio.h"
#include <FreeRTOS.h>
#include <task.h>
#include <timers.h>
#include <os_mem.h>
#include "rfcomm_app_demo.h"

TRfcApp* pRfcServer;
void PrintUTF8(char *str)
{
    while (*str != '\0')
    {
        printf("%c", (*str >= 32 && *str <= 126) ? *str : '?'); // ' ' to '~'
        str++;
    }
}

void appHandleRfcConnectInd(TRfcProMsg msg)
{
	TBtConRespRfc	rfc;
	rfc.frameSize = msg.frameSize;
	rfc.maxCredits = 7;
	rfc.mscStatus = 0;

        rfcSendConResp(msg.handle,msg.dlci,0,rfc);

}
void appHandleRfcConnectRsp(TRfcProMsg msg)
{
	
}
void appHandleRfcDiscInd(TRfcProMsg msg)
{
	
}
void appHandleRfcDiscRsp(TRfcProMsg msg)
{
	
}
void appHandleRfcFlowInd(TRfcProMsg msg)
{
	
}
void appHandleRfcDataInd(TRfcProMsg msg)
{
	printf("appHandleRfcDataInd dataLength:%d",msg.dataLength);
	rfcHandleUpDataResp(msg.handle);
}
void appHandleRfcAuthorizationInd(TRfcProMsg msg)
{
	
}
void rfcMessageHandler( RfcMsgType msg_type,TRfcProMsg msg )
{
	printf("rfcMessageHandler msg_type =%d",msg_type);
	switch(msg_type)
	{
	case RFC_CONNECT_IND:
		appHandleRfcConnectInd(msg);		
		break;
	case RFC_CONNECT_RSP:
		appHandleRfcConnectRsp(msg);
		break;
	case RFC_CONNECT_COMPLETE:
		appHandleRfcConnectRsp(msg);
		break;
	case RFC_DISC_IND:
		appHandleRfcDiscInd(msg);
		break;
	case RFC_DISC_RSP:
		appHandleRfcDiscRsp(msg);
		break;
	case RFC_FLOW_IND:
		appHandleRfcFlowInd(msg);
		break;
	case RFC_DATA_IND:
		appHandleRfcDataInd(msg);
		break;
	case RFC_AUTHORIZATION_IND:
		appHandleRfcAuthorizationInd(msg);
		break;	
	default:
		break;
	}

}
bool rfcomm_server_init(void)
{
	pRfcServer= (TRfcApp*)osMemoryAllocate(RAM_TYPE_DATA_ON, sizeof(TRfcApp));
	if (pRfcServer == NULL)
    	{
        return false;
    	}
	pRfcServer->uuid = UUID_TEST_SERVER;
//	pRfcServer->frameSize = 330;
	rfcRegister(UUID_TEST_SERVER,&pRfcServer->server_channel,rfcMessageHandler);
	printf("rfcomm_app_init rfcRegister server channel: %d \r\n",pRfcServer->server_channel );
	if (pRfcServer->server_channel ==0)
	{
		printf("rfcomm_app_init rfcRegister fail !!\r\n");
        	return false;
	}
	return true;
}
bool rfcomm_app_init(void)
{
	UARTInit();
	
	printf("rfcomm_app_init ready\r\n");
	return true;
}

void rfcomm_app_sim_cmd(char rx)
{

    printf("%c\r\n", rx);
	
    if (rx == 'c') //connect
    {
//	rfcHandleUpConReq(pRfcClient,server_channel,);
	//(TBdAddr bd, uint8_t serverChannel, uint16_t frameSize, uint8_t status, uint8_t maxCredits, uint16_t uuid)
	
    } else if (rx == 'd') //disconnect
    {
//	rfcHandleUpDiscReq//(uint16_t cid);
    } else if (rx == 's') 
    {
  //  	rfcHandleUpDataReq//(TBdAddr bd, uint8_t dlci, uint8_t * pbuffer, uint32_t length)
    } else if (rx == 'f') 
    {
//	rfcHandleUpFlowReq//(uint8_t handle, uint8_t flowStatus, uint8_t sbreak);
	   
    } else if (rx == 'i')
    {
        printf("\r\n------info of app:-------\r\n"); 
 
    }   
    return;
}
