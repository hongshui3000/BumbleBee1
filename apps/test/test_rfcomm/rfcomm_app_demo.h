/**
********************************************************************************************************
Copyright (c) 2015, Realtek Semiconductor Corporation. All rights reserved.
*********************************************************************************************************
* @file       rfcomm_app_demo.h
* @brief     
* @details   
*
* @author  	 
* @date      	 
* @version	v0.1
*/



#define UUID_TEST_CLIENT	0xee
#define UUID_TEST_SERVER 	0xff
#define MAX_RFC_APP_INST	4

typedef struct TRfcChannel 
{	
	uint8_t handle;
	uint8_t dlci;                          /**< protocol service multiplexer, 0 for unused */
	uint16_t frameSize;
	bool creditBased; 
	uint8_t outgoing;
	bool fc;	/*1 blocked, 0 unblocked*/
	uint16_t dataLength;
	uint8_t *buf;
}TRfcChannel;

typedef struct TRfcApp 
{	
	TRfcChannel     link[MAX_RFC_APP_INST];
	uint8_t  uuid;                           /**<  */
	uint8_t  server_channel;

} TRfcApp;

bool rfcomm_app_init(void);
void rfcomm_app_sim_cmd(char rx);



