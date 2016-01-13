
#include "trace.h"
#include "gapbond_legacy.h"
#include "a2dp_api.h"
#include "a2dp_demo.h"

enum { __FILE_NUM__ = 0 };
uint8_t BumbleBeeBd[6] = {0x59,0x83,0x00,0xa0,0xfd,0x74};
uint8_t RemoteBd[6];

bool App_SetConfig(PAppA2dpCallback AppA2dpCallback)
{
	TA2dpResult result;
	TA2dpSetCapabilities local_Caps;
	local_Caps.localEndPointId = A2DP_STREAM_SEID_SRC;
	local_Caps.role = AVDTP_ROLE_INT | AVDTP_ROLE_SRC;
	local_Caps.mediaType = AVDTP_MEDIA_TYPE_AUDIO;
	local_Caps.endPointType = AVDTP_TSEP_SRC;

	local_Caps.Capabilities.samplingFrequency = A2DP_FREQU44100 | A2DP_FREQU48000;
	local_Caps.Capabilities.channelMode = A2DP_MODESTEREO | A2DP_MODEJOINT;
	local_Caps.Capabilities.blockNumber = A2DP_BLOCKS4 | A2DP_BLOCKS8 | A2DP_BLOCKS12 | A2DP_BLOCKS16;
	local_Caps.Capabilities.subbandNumber = A2DP_SUBBANDS4 | A2DP_SUBBANDS8;
	local_Caps.Capabilities.allocMethod = A2DP_ALLOCSNR | A2DP_ALLOCLOUDNESS;
	local_Caps.Capabilities.minBitpool = A2DP_MIN_BITPOOL;
	local_Caps.Capabilities.maxBitpool = A2DP_MAX_BITPOOL;
	result == a2dp_InitConfiguration(local_Caps,AppA2dpCallback);
	if(result == A2DP_NO_RESOURCE)
	{	
		DBG_BUFFER(MODULE_APP, LEVEL_INFO, "allocate memory is fail \r\n",0);
		return FALSE;
	}

	local_Caps.localEndPointId = A2DP_STREAM_SEID_SNK1;
	local_Caps.role = AVDTP_ROLE_ACP | AVDTP_ROLE_SNK;
	local_Caps.mediaType = AVDTP_MEDIA_TYPE_AUDIO;
	local_Caps.endPointType = AVDTP_TSEP_SNK;

	local_Caps.Capabilities.samplingFrequency = A2DP_FREQU16000 | A2DP_FREQU32000 | A2DP_FREQU44100 | A2DP_FREQU48000;
	local_Caps.Capabilities.channelMode = A2DP_MODEMOMO | A2DP_MODEDUAL | A2DP_MODESTEREO | A2DP_MODEJOINT;
	local_Caps.Capabilities.blockNumber = A2DP_BLOCKS4 | A2DP_BLOCKS8 | A2DP_BLOCKS12 | A2DP_BLOCKS16;
	local_Caps.Capabilities.subbandNumber = A2DP_SUBBANDS4 | A2DP_SUBBANDS8;
	local_Caps.Capabilities.allocMethod = A2DP_ALLOCSNR | A2DP_ALLOCLOUDNESS;
	local_Caps.Capabilities.minBitpool = A2DP_MIN_BITPOOL;
	local_Caps.Capabilities.maxBitpool = A2DP_MAX_BITPOOL;
	result == a2dp_InitConfiguration(local_Caps,AppA2dpCallback);
	if(result == A2DP_NO_RESOURCE)
	{	
		DBG_BUFFER(MODULE_APP, LEVEL_INFO, "allocate memory is fail \r\n",0);
		return FALSE;
	}
	/*local_Caps.localEndPointId = A2DP_STREAM_SEID_SNK2;
	a2dp_InitConfiguration(localCaps,AppA2dpCallback);*/	
	DBG_BUFFER(MODULE_APP, LEVEL_INFO, "success set endpoint \r\n",0);
	return TRUE;

}

void App_PrintConfiguration(TA2dpCapabilities *Capabilities, TBdAddr bd)
{
	if(Capabilities->samplingFrequency == SBC_FREQU16000)			
		DBG_BUFFER(MODULE_APP, LEVEL_INFO, "BD=[%2.2x:%2.2x:%2.2x:%2.2x:%2.2x:%2.2x] samplingFreq is 16000 \r\n",6,bd[5],bd[4],bd[3],bd[2],bd[1],bd[0]);				
	else if(Capabilities->samplingFrequency == SBC_FREQU32000)	
		DBG_BUFFER(MODULE_APP, LEVEL_INFO, "BD=[%2.2x:%2.2x:%2.2x:%2.2x:%2.2x:%2.2x] samplingFreq is 32000 \r\n",6,bd[5],bd[4],bd[3],bd[2],bd[1],bd[0]);				
	else if(Capabilities->samplingFrequency == SBC_FREQU44100)	
		DBG_BUFFER(MODULE_APP, LEVEL_INFO, "BD=[%2.2x:%2.2x:%2.2x:%2.2x:%2.2x:%2.2x] samplingFreq is 44100 \r\n",6,bd[5],bd[4],bd[3],bd[2],bd[1],bd[0]);				
	else if(Capabilities->samplingFrequency == SBC_FREQU48000)	
		DBG_BUFFER(MODULE_APP, LEVEL_INFO, "BD=[%2.2x:%2.2x:%2.2x:%2.2x:%2.2x:%2.2x] samplingFreq is 48000 \r\n",6,bd[5],bd[4],bd[3],bd[2],bd[1],bd[0]);				
		
	if(Capabilities->blockNumber == SBC_BLOCKS4)
		DBG_BUFFER(MODULE_APP, LEVEL_INFO, "BD=[%2.2x:%2.2x:%2.2x:%2.2x:%2.2x:%2.2x] block length is 4 \r\n",6,bd[5],bd[4],bd[3],bd[2],bd[1],bd[0]);				
	else if(Capabilities->blockNumber == SBC_BLOCKS8)
		DBG_BUFFER(MODULE_APP, LEVEL_INFO, "BD=[%2.2x:%2.2x:%2.2x:%2.2x:%2.2x:%2.2x] block length is 8 \r\n",6,bd[5],bd[4],bd[3],bd[2],bd[1],bd[0]);	
	else if(Capabilities->blockNumber == SBC_BLOCKS12)
		DBG_BUFFER(MODULE_APP, LEVEL_INFO, "BD=[%2.2x:%2.2x:%2.2x:%2.2x:%2.2x:%2.2x] block length is 12 \r\n",6,bd[5],bd[4],bd[3],bd[2],bd[1],bd[0]);	
	else if(Capabilities->blockNumber == SBC_BLOCKS16)
		DBG_BUFFER(MODULE_APP, LEVEL_INFO, "BD=[%2.2x:%2.2x:%2.2x:%2.2x:%2.2x:%2.2x] block length is 16 \r\n",6,bd[5],bd[4],bd[3],bd[2],bd[1],bd[0]);	

	if(Capabilities->channelMode == SBC_MODE_MONO)
		DBG_BUFFER(MODULE_APP, LEVEL_INFO, "BD=[%2.2x:%2.2x:%2.2x:%2.2x:%2.2x:%2.2x] channels is MONO \r\n",6,bd[5],bd[4],bd[3],bd[2],bd[1],bd[0]); 
	else if(Capabilities->channelMode == SBC_MODE_DUAL)
		DBG_BUFFER(MODULE_APP, LEVEL_INFO, "BD=[%2.2x:%2.2x:%2.2x:%2.2x:%2.2x:%2.2x] channels is Dual \r\n",6,bd[5],bd[4],bd[3],bd[2],bd[1],bd[0]); 
	else if(Capabilities->channelMode == SBC_MODE_STEREO)
		DBG_BUFFER(MODULE_APP, LEVEL_INFO, "BD=[%2.2x:%2.2x:%2.2x:%2.2x:%2.2x:%2.2x] channels is Stereo \r\n",6,bd[5],bd[4],bd[3],bd[2],bd[1],bd[0]);	
	else if(Capabilities->channelMode == SBC_MODE_JOINT)
		DBG_BUFFER(MODULE_APP, LEVEL_INFO, "BD=[%2.2x:%2.2x:%2.2x:%2.2x:%2.2x:%2.2x] channels is Joint \r\n",6,bd[5],bd[4],bd[3],bd[2],bd[1],bd[0]);	
		
	if(Capabilities->allocMethod == SBC_ALLOCLOUDNESS)
		DBG_BUFFER(MODULE_APP, LEVEL_INFO, "BD=[%2.2x:%2.2x:%2.2x:%2.2x:%2.2x:%2.2x] allocation method is Loudness \r\n",6,bd[5],bd[4],bd[3],bd[2],bd[1],bd[0]);	
	else if(Capabilities->allocMethod == SBC_ALLOCSNR)
		DBG_BUFFER(MODULE_APP, LEVEL_INFO, "BD=[%2.2x:%2.2x:%2.2x:%2.2x:%2.2x:%2.2x] allocation method is SNR \r\n",6,bd[5],bd[4],bd[3],bd[2],bd[1],bd[0]); 

	if(Capabilities->subbandNumber == SBC_SUBBANDS4)
		DBG_BUFFER(MODULE_APP, LEVEL_INFO, "BD=[%2.2x:%2.2x:%2.2x:%2.2x:%2.2x:%2.2x] Subbands is 4 \r\n",6,bd[5],bd[4],bd[3],bd[2],bd[1],bd[0]);	
	else if(Capabilities->allocMethod == SBC_SUBBANDS8)
		DBG_BUFFER(MODULE_APP, LEVEL_INFO, "BD=[%2.2x:%2.2x:%2.2x:%2.2x:%2.2x:%2.2x] Subbands is 8 \r\n",6,bd[5],bd[4],bd[3],bd[2],bd[1],bd[0]); 

	DBG_BUFFER(MODULE_APP, LEVEL_INFO, "BD=[%2.2x:%2.2x:%2.2x:%2.2x:%2.2x:%2.2x] minbitpool is %d \r\n",7,bd[5],bd[4],bd[3],bd[2],bd[1],bd[0],Capabilities->minBitpool); 
	DBG_BUFFER(MODULE_APP, LEVEL_INFO, "BD=[%2.2x:%2.2x:%2.2x:%2.2x:%2.2x:%2.2x] maxbitpool is %d \r\n",7,bd[5],bd[4],bd[3],bd[2],bd[1],bd[0],Capabilities->maxBitpool); 
}

void AppA2dpCallback(void *msg,TBdAddr bd,A2dpToAppMsgType AppA2dp_msg)
{ 
	static uint8_t bumblebee_play_state;
	static uint8_t bumblebee_used;
	switch(AppA2dp_msg)
	{
	case PLAYER_STATE:
		{
		    uint8_t play_state = *(int *)msg;
			if(memcmp(BumbleBeeBd,bd,BD_ADDR_SIZE) == 0)
			{
				bumblebee_play_state = play_state;
			}
			if(play_state == A2DP_PLAYER_STATE_STOPPED)
				DBG_BUFFER(MODULE_APP, LEVEL_INFO, "BD=[%2.2x:%2.2x:%2.2x:%2.2x:%2.2x:%2.2x] playstate is Stopped \r\n",6,bd[5],bd[4],bd[3],bd[2],bd[1],bd[0]);
			else if(play_state == A2DP_PLAYER_STATE_PAUSED)
			{
				DBG_BUFFER(MODULE_APP, LEVEL_INFO, "BD=[%2.2x:%2.2x:%2.2x:%2.2x:%2.2x:%2.2x] playstate is Paused \r\n",6,bd[5],bd[4],bd[3],bd[2],bd[1],bd[0]);
				if(memcmp(BumbleBeeBd,bd,BD_ADDR_SIZE) != 0)
				{
					a2dp_Suspend((TBdAddr *)BumbleBeeBd);
				}
			}
			else if(play_state == A2DP_PLAYER_STATE_PLAYING)
			{
				DBG_BUFFER(MODULE_APP, LEVEL_INFO, "BD=[%2.2x:%2.2x:%2.2x:%2.2x:%2.2x:%2.2x] playstate is Playing \r\n",6,bd[5],bd[4],bd[3],bd[2],bd[1],bd[0]);			
				if(memcmp(BumbleBeeBd,bd,BD_ADDR_SIZE) != 0)
				{
					if(bumblebee_used == 1)
						a2dp_Start((TBdAddr *)BumbleBeeBd);				
					else
						a2dp_Connect((TBdAddr *)BumbleBeeBd);
				}
			}
		}
		break;	
	case REGISTER_COMPLETE:
		{
			DBG_BUFFER(MODULE_APP, LEVEL_INFO, "register is complete, begin to connect BumbleBee \r\n",0); 						
			a2dp_Connect((TBdAddr *)BumbleBeeBd);
		}
		break;
	case AUTHORIZATION_INDICATION:	
		memcpy(RemoteBd,bd,6);
		DBG_BUFFER(MODULE_APP, LEVEL_INFO, "BD=[%2.2x:%2.2x:%2.2x:%2.2x:%2.2x:%2.2x] connect request, yes(y) or no(n) \r\n",6,bd[5],bd[4],bd[3],bd[2],bd[1],bd[0]);			
		break;
	case DATA_INDICATION:
		if(bumblebee_play_state == A2DP_PLAYER_STATE_PLAYING)
		{
			a2dp_SendAudioData(msg); //send audio data to Bumblebee		
		}
		break;
	case CONNECT_COMPLETE:
		{	
			A2dpToAppMsg *massage = (A2dpToAppMsg *)msg;
			if(memcmp(BumbleBeeBd,bd,BD_ADDR_SIZE) == 0)
			{
				bumblebee_used = massage->used;
			}
			DBG_BUFFER(MODULE_APP, LEVEL_INFO, "BD=[%2.2x:%2.2x:%2.2x:%2.2x:%2.2x:%2.2x] signal and stream channel are connect \r\n",6,bd[5],bd[4],bd[3],bd[2],bd[1],bd[0]);	
			
			if(massage->player_state == A2DP_PLAYER_STATE_STOPPED)
				DBG_BUFFER(MODULE_APP, LEVEL_INFO, "BD=[%2.2x:%2.2x:%2.2x:%2.2x:%2.2x:%2.2x] playstate is Stopped\r\n",6,bd[5],bd[4],bd[3],bd[2],bd[1],bd[0]);	
			else if(massage->player_state == A2DP_PLAYER_STATE_PAUSED)
				DBG_BUFFER(MODULE_APP, LEVEL_INFO, "BD=[%2.2x:%2.2x:%2.2x:%2.2x:%2.2x:%2.2x] playstate is Paused\r\n",6,bd[5],bd[4],bd[3],bd[2],bd[1],bd[0]);	
			else if(massage->player_state == A2DP_PLAYER_STATE_PLAYING)				
				DBG_BUFFER(MODULE_APP, LEVEL_INFO, "BD=[%2.2x:%2.2x:%2.2x:%2.2x:%2.2x:%2.2x] playstate is Playing\r\n",6,bd[5],bd[4],bd[3],bd[2],bd[1],bd[0]);	
				

			if(massage->sound_channel == A2DP_CHANNELS_MONO)				
				DBG_BUFFER(MODULE_APP, LEVEL_INFO, "BD=[%2.2x:%2.2x:%2.2x:%2.2x:%2.2x:%2.2x] channels is MONO\r\n",6,bd[5],bd[4],bd[3],bd[2],bd[1],bd[0]);					
			else if(massage->sound_channel == A2DP_CHANNELS_STEREO)				
				DBG_BUFFER(MODULE_APP, LEVEL_INFO, "BD=[%2.2x:%2.2x:%2.2x:%2.2x:%2.2x:%2.2x] channels is STEREO\r\n",6,bd[5],bd[4],bd[3],bd[2],bd[1],bd[0]);	
				
			DBG_BUFFER(MODULE_APP, LEVEL_INFO, "BD=[%2.2x:%2.2x:%2.2x:%2.2x:%2.2x:%2.2x] samplingFreq is %d \r\n",7,bd[5],bd[4],bd[3],bd[2],bd[1],bd[0],massage->samplefrequence);	
			a2dp_Start((TBdAddr *)BumbleBeeBd);
		}
		break;
	case GET_CONFIGURATION:
		{	
			TA2dpCapabilities *Capabilities = (TA2dpCapabilities *)msg;
			App_PrintConfiguration(Capabilities,bd);
		}
		break;
	case DISCONNECT_COMPLETE:
		{	
			bumblebee_play_state = 0;
		    bumblebee_used = 0;
			DBG_BUFFER(MODULE_APP, LEVEL_INFO, "BD=[%2.2x:%2.2x:%2.2x:%2.2x:%2.2x:%2.2x] signal and stream channel are disconnect \r\n",6,bd[5],bd[4],bd[3],bd[2],bd[1],bd[0]);	
		}
		break;
	case RESULT:
		{
			TA2dpResult *result = (TA2dpResult *)msg;
			if(*result == A2DP_SUCCESS)
				DBG_BUFFER(MODULE_APP, LEVEL_INFO, "BD=[%2.2x:%2.2x:%2.2x:%2.2x:%2.2x:%2.2x] result is OK \r\n",6,bd[5],bd[4],bd[3],bd[2],bd[1],bd[0]);				
			else
				DBG_BUFFER(MODULE_APP, LEVEL_INFO, "BD=[%2.2x:%2.2x:%2.2x:%2.2x:%2.2x:%2.2x] result is error \r\n",6,bd[5],bd[4],bd[3],bd[2],bd[1],bd[0]);								
		}
		break;
	case TIME_OUT:
		{	
			uint8_t *time_type = (uint8_t *)msg;
			if(*time_type == 0)
				DBG_BUFFER(MODULE_APP, LEVEL_INFO, "BD=[%2.2x:%2.2x:%2.2x:%2.2x:%2.2x:%2.2x] a2dp connect time out \r\n",6,bd[5],bd[4],bd[3],bd[2],bd[1],bd[0]);	
			else if(*time_type == 1)
				DBG_BUFFER(MODULE_APP, LEVEL_INFO, "BD=[%2.2x:%2.2x:%2.2x:%2.2x:%2.2x:%2.2x] avdtp signal command time out \r\n",6,bd[5],bd[4],bd[3],bd[2],bd[1],bd[0]);		
		}
		break;
	}
}

void App_sim_cmd(char rx)
{
	  //uint8_t bd[6]={0x59,0x83,0x00,0xa0,0xfd,0x74}; 蓝牙耳机
	  //uint8_t bd[6]={0x20,0x7e,0xa5,0x7e,0xe7,0xf0};  测试手机
	  //uint8_t bd[6]={0xd5,0x05,0x00,0xb3,0xc8,0x20};  酷派手机
	  if((rx == 'y') || (rx == 'n'))
	  {	
		  if(rx == 'y')
		  {
		  	  GAPBondlegacy_Authorize(RemoteBd, blueAPI_CauseAccept);
		  }else if(rx == 'n')
		  {
		  	  GAPBondlegacy_Authorize(RemoteBd, blueAPI_CauseReject);
		  }
	  }
	  else
	  {
		  TA2dpResult result;
		  if (rx == '0')
		  {   
			  result = a2dp_Connect((TBdAddr *)BumbleBeeBd);  
		  }else if(rx == '1')
		  {
		  	  result = a2dp_Suspend((TBdAddr *)BumbleBeeBd);
		  }else if(rx == '2')
		  {
			  result = a2dp_Start((TBdAddr *)BumbleBeeBd);
		  }else if(rx == '3')
		  {
			  result = a2dp_GetConfiguration((TBdAddr *)BumbleBeeBd);
		  }else if(rx == '4')
		  {
			  result = a2dp_Disconnect((TBdAddr *)BumbleBeeBd);
		  }else if(rx == '5')
		  {
			  result = a2dp_Reconnect((TBdAddr *)BumbleBeeBd);
		  }	  
		  AppA2dpCallback(&result, BumbleBeeBd, RESULT);
	  }
}

bool App_Init(void)
{
	App_SetConfig(AppA2dpCallback);
	if(a2dp_Init() == A2DP_NO_RESOURCE)
	{
		DBG_BUFFER(MODULE_APP, LEVEL_INFO, "a2dp init is fail \r\n",0);	
		return FALSE;
	}
	return TRUE;
}
