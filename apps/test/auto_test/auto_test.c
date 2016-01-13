/**
**********************************************************************************************************
*               Copyright(c) 2014, Realtek Semiconductor Corporation. All rights reserved.
**********************************************************************************************************
* @file     auto_test.c
* @brief    auto test implementation.
* @details  none.
* @author   
* @date     
* @version  v0.1
**********************************************************************************************************
*/
#include "auto_test.h"
/*definition of Auto test task section*/
#define AutoTest_STACK_SIZE        		2048
#define AutoTest_PRIORITY          		(tskIDLE_PRIORITY + 2)   /* Task priorities. */
#define AutoTest_NUMBER_OF_RX_EVENT     0x20
#define AutoTest_UART_RX_EVENT     		0x01

#define BIT_PERI_UART0_EN BIT(0)
#define BIT_SOC_ACTCK_UART0_EN BIT(0)
#define BIT_SOC_ACTCK_TIMER_EN BIT(14)
#define BIT_PERI_I2C1_EN BIT(17)
#define BIT_PERI_I2C0_EN BIT(16)
#define BIT_SOC_ACTCK_I2C1_EN BIT(2)
#define BIT_SOC_ACTCK_I2C0_EN BIT(0)
#define BIT_PERI_SPI1_EN BIT(9)
#define BIT_PERI_SPI0_EN BIT(8)
#define BIT_SOC_ACTCK_SPI1_EN BIT(18)
#define BIT_SOC_ACTCK_SPI0_EN BIT(16)


AutoTest_PacketTypeDef AutoTest_Packet;
/*queue handle for communication between isr and auto test task*/
static xQueueHandle AutoTestQueueHandleEvent;

/* definition of global variable for Auto test */
AutoTest_CmdTypeDef AutoTest_cmd SRAM_OFF_BD_DATA_SECTION;
/* store state for Initialize and Deinitialize function test of uart */
uint8_t REG_PERI_uart_Params[9] SRAM_OFF_BD_DATA_SECTION; 

KEYSCAN_DATA_STRUCT  CurKeyData;

void AutoTestHandle(uint8_t cmd, uint8_t copmsk, uint8_t* pOpt, uint16_t lenPara, uint8_t* pPara);
void ConvertBytesToAutoTestCmd(uint8_t cmd, uint8_t copmsk, uint8_t* pOpt, uint16_t lenPara, uint8_t* pPara, AutoTest_CmdTypeDef *pAutoTest_cmd);
void AutoTestParameterHandle(uint8_t* pPara, uint16_t lenPara, AutoTest_CmdTypeDef *pAutoTest_cmd);

void AutoTestGPIOHandle(AutoTest_CmdTypeDef *pAutoTest_cmd );
void AutoTestGPIOParamHandle(uint8_t* pPara, GPIO_InitTypeDef *pAutoTest_GPIO_param);


void AutoTestPWMHandle(AutoTest_CmdTypeDef *pAutoTest_cmd);
void AutoTestPWMParameterHandle(uint8_t* pPara, AutoTest_pwm_Params *pAutoTest_PWMAdap);

void AutoTestTimerHandle(AutoTest_CmdTypeDef *pAutoTest_cmd );
void AutoTestTimerParamHandle(uint8_t* pPara, AutoTest_Tim_Params *pAutoTestTimerAdap);

void AutoTestUartHandle(uint8_t cmd, uint8_t copmsk, uint8_t* pOpt, uint16_t lenPara, uint8_t* pPara, AutoTest_CmdTypeDef *AutoTest_cmd );
void AutoTestUartParameterHandle(uint8_t* pPara, AutoTest_CmdTypeDef *pAutoTest_cmd);
void AutoTestUartLoopBackHandle(uint8_t cmd, uint8_t copmsk, uint8_t* pOpt,uint16_t lenPara, uint8_t* pPara);
void BaudRateModifyHandle(uint8_t ovsr, uint16_t div, uint16_t ovsr_adj);    


void AutoTestI2CHandle(AutoTest_CmdTypeDef *pAutoTest_cmd );
void AutoTestI2CParamHandle(uint8_t* pPara, AutoTest_i2c_Params *pAutoTest_I2C_Params);

void AutoTestSPIHandle(AutoTest_CmdTypeDef *pAutoTest_cmd );
void AutoTestSPIParamHandle(uint8_t* pSPIPara, AutoTest_spi_Params *pAutoTest_SPI_Params);


void AutoTestKeyScanHandle(AutoTest_CmdTypeDef *pAutoTest_cmd );
void AutoTestKeyScanParamHandle(uint8_t* pParam, AutoTest_keyscan_Params *pkeypad_param);


uint8_t AutoTest_ReadPeriFunRegister(uint32_t base, uint32_t addr, uint32_t BIT_PERI_EN);
void ReadPeriFunRegisterByPeripheralType(InitOrDeinitPeripherialType periType, uint8_t *FUNC_param, uint8_t *CLK_param);
uint8_t AutoTest_ReadPinStatus(uint8_t Pin_Num);




/* API for Auto test task */
void AutoTestTask(void *pParameters);
BOOL PacketDecoder(AutoTest_PacketTypeDef *pAutoTest_Packet, uint16_t size);
unsigned char AutoTestCRC8(LPBYTE pStart, WORD length);
void DataUARTInit(void);
BOOL LoopQueueIsFull(AutoTest_PacketTypeDef *pAutoTest_Packet);
BOOL LoopQueueIsEmpty(AutoTest_PacketTypeDef *pAutoTest_Packet);
void LoopQueueInit(AutoTest_PacketTypeDef *pAutoTest_Packet);
uint8_t AutoTestSendData(uint8_t FunNum, uint8_t *sendbuffer, uint32_t buffer_length);
int AutosendInt32(uint32_t ch);
int Autosendchar(int ch);


BaseType_t AutoTestInit(void)
{
	BaseType_t xReturn = pdFAIL;

	LoopQueueInit(&AutoTest_Packet);
	DataUARTInit();
	xReturn = xTaskCreate( AutoTestTask, "Auto test", AutoTest_STACK_SIZE/sizeof(portSTACK_TYPE), NULL, AutoTest_PRIORITY, NULL );	
	/* start task schedule */
  	vTaskStartScheduler();
	return xReturn;
}

BOOL LoopQueueIsFull(AutoTest_PacketTypeDef *pAutoTest_Packet)
{
	if(pAutoTest_Packet->QueueSize >= pAutoTest_Packet->QueueCapacity)
	{
		return TRUE;
	}
	return FALSE;	
}
BOOL LoopQueueIsEmpty(AutoTest_PacketTypeDef *pAutoTest_Packet)
{
	return pAutoTest_Packet->QueueSize == 0; 	
}

void LoopQueueInit(AutoTest_PacketTypeDef *pAutoTest_Packet)
{
	pAutoTest_Packet->Decoderlength  = 0;				/**< length of decoder payload*/
	pAutoTest_Packet->DecoderIndex 	 = 0;
	pAutoTest_Packet->QueueCapacity  = 256;				/**<equal length of LoopQueue*/
	pAutoTest_Packet->QueueSize 	 = 0;
	pAutoTest_Packet->ReadIndex 	 = 0;				/**< index of read queue */
	pAutoTest_Packet->WriteIndex 	 = 0; 				/**< index of write queue */
	pAutoTest_Packet->PacketLength 	 = 0;
	pAutoTest_Packet->CollectorState = WaitCmd;
}

void AutoTestTask(void *pParameters)
{
	uint8_t event;
	uint16_t queueSize;
	
	/*init event queue  from uart to AutoTest task*/
	AutoTestQueueHandleEvent = xQueueCreate(AutoTest_NUMBER_OF_RX_EVENT, sizeof(unsigned char));

	while(TRUE)
	{
		if(xQueueReceive(AutoTestQueueHandleEvent, &event, portMAX_DELAY) == pdPASS)
		{
			if(event == AutoTest_UART_RX_EVENT)
			{
				do
				{
					taskENTER_CRITICAL();
					queueSize = AutoTest_Packet.QueueSize;
					taskEXIT_CRITICAL();
					print_howie("==%d===\n",__LINE__);
					if(PacketDecoder(&AutoTest_Packet, queueSize))
					{
						AutoTestHandle(AutoTest_Packet.DecoderData[0], AutoTest_Packet.DecoderData[1], &AutoTest_Packet.DecoderData[4], AutoTest_Packet.PacketLength - 9, &AutoTest_Packet.DecoderData[9]);
						print_howie("==%d===\n",__LINE__);
					}
					
					taskENTER_CRITICAL();
					queueSize = AutoTest_Packet.QueueSize;
					taskEXIT_CRITICAL();
				}while(queueSize);
			}
		}
	}
}
#define UART_IRQ    12
void DataUARTInit(void)
{
		UART_DeInit(UART);
		/* turn on UART clock */
    RCC_PeriphClockCmd(APBPeriph_UART, APBPeriph_UART_CLOCK, ENABLE);  
    /******************* uart init *****************************/
    UART_InitTypeDef uartInitStruct;  
    UART_StructInit(&uartInitStruct);
		uartInitStruct.rxTriggerLevel= UART_RX_FIFO_TRIGGER_LEVEL_14BYTE;	
    /* default 115200 baudrate */
    UART_Init(UART, &uartInitStruct); 
		UART_INTConfig(UART, UART_INT_RD_AVA, ENABLE);  
    /*  Enable UART IRQ  */
    NVIC_ClearPendingIRQ((IRQn_Type)UART_IRQ);
    NVIC_SetPriority((IRQn_Type)UART_IRQ, 3);
    NVIC_EnableIRQ((IRQn_Type)UART_IRQ);
}

void Data_Uart_Handler_bak(void)

{
	uint8_t event = AutoTest_UART_RX_EVENT;
	portBASE_TYPE TaskWoken = pdFALSE;
	uint16_t len = 0;
	
	/* read interrupt id */
    UINT32 int_status = UART_GetIID(UART);
	
	/* disable interrupt */
    UART_INTConfig(UART, UART_INT_RD_AVA | UART_INT_LINE_STS, DISABLE);
	//print_howie("have a test line = %d\n",__LINE__);
	
	switch(int_status & 0x0E)
	{
		/* tx fifo empty */
		case 0x02:
		/* do nothing */
		break;

		/* rx data valiable */
		case 0x04:
			if(!LoopQueueIsFull(&AutoTest_Packet))
			{
				AutoTest_Packet.WriteIndex &= (AutoTest_Packet.QueueCapacity -1);
				if(AutoTest_Packet.WriteIndex + 14 < AutoTest_Packet.QueueCapacity)
				{
					UART_ReceiveData(UART,&AutoTest_Packet.LoopQueue[AutoTest_Packet.WriteIndex], 14);
					AutoTest_Packet.WriteIndex = (AutoTest_Packet.WriteIndex + 14)&(AutoTest_Packet.QueueCapacity -1);					
				}
				else
				{
					len = AutoTest_Packet.QueueCapacity - AutoTest_Packet.WriteIndex;
					UART_ReceiveData(UART,&AutoTest_Packet.LoopQueue[AutoTest_Packet.WriteIndex], len);
					AutoTest_Packet.WriteIndex = (AutoTest_Packet.WriteIndex + len) & (AutoTest_Packet.QueueCapacity -1);
					UART_ReceiveData(UART,&AutoTest_Packet.LoopQueue[AutoTest_Packet.WriteIndex], 14 - len);//for what f
					AutoTest_Packet.WriteIndex += 14 - len;
				}
			AutoTest_Packet.QueueSize += 14;
			xQueueSendFromISR(AutoTestQueueHandleEvent, &event, &TaskWoken);
			}
		break;

		/*rx time out*/
		case 0x0c:
			if(!LoopQueueIsFull(&AutoTest_Packet))
			{
				while(UART_GetFlagState(UART, UART_FLAG_RX_DATA_RDY) == SET)
				{
					//Queue wrap
					AutoTest_Packet.WriteIndex &= (AutoTest_Packet.QueueCapacity -1);
					UART_ReceiveData(UART, &AutoTest_Packet.LoopQueue[AutoTest_Packet.WriteIndex++],1);
					AutoTest_Packet.QueueSize++;
				}
			xQueueSendFromISR(AutoTestQueueHandleEvent, &event, &TaskWoken);
			}
		break;
		
		/* receive line status interrupt */
		case 0x06:
		break;

		default:
		break;
	}
	/* enable interrupt again */
    UART_INTConfig(UART, UART_INT_RD_AVA | UART_INT_LINE_STS, ENABLE);
}

BOOL PacketDecoder(AutoTest_PacketTypeDef *pAutoTest_Packet, uint16_t size)
{
	uint16_t decodedSize = size;
	BOOL PaketComplete = FALSE;
	//print_howie("HAVE A TEST==0x%x===\n",pAutoTest_Packet->LoopQueue[pAutoTest_Packet->ReadIndex]);
	print_howie("==%d===\n",__LINE__);
	switch(pAutoTest_Packet->CollectorState)
	{
		case WaitCmd:
		{
			print_howie("==%d===\n",__LINE__);
			print_howie("HAVE A TEST==0x%x===\n",pAutoTest_Packet->LoopQueue[pAutoTest_Packet->ReadIndex]);
			if(size > 0)
			{
				if(pAutoTest_Packet->LoopQueue[pAutoTest_Packet->ReadIndex] == 0xf6)
				{
					pAutoTest_Packet->ReadIndex &= (pAutoTest_Packet->QueueCapacity -1);
					pAutoTest_Packet->DecoderData[pAutoTest_Packet->DecoderIndex++] = pAutoTest_Packet->LoopQueue[pAutoTest_Packet->ReadIndex++]; 
					pAutoTest_Packet->CollectorState = WaitCopmask;
					size--;
				}
				else
				{
					pAutoTest_Packet->ReadIndex &= (pAutoTest_Packet->QueueCapacity -1);
					pAutoTest_Packet->ReadIndex++;
					decodedSize = 1;
					
					break;
				}
			}
			else
			{
				break;
			}
		}
		case WaitCopmask:
		{
			print_howie("==%d===\n",__LINE__);
			print_howie("HAVE A TEST==0x%x===\n",pAutoTest_Packet->LoopQueue[pAutoTest_Packet->ReadIndex]);
			if(size > 0)
			{
				pAutoTest_Packet->ReadIndex &= (pAutoTest_Packet->QueueCapacity -1);
				pAutoTest_Packet->DecoderData[pAutoTest_Packet->DecoderIndex++] = pAutoTest_Packet->LoopQueue[pAutoTest_Packet->ReadIndex++]; 
				pAutoTest_Packet->CollectorState = WaitLp_h;
				size--;
			}
			else
			{
				break;
			}
		}
		case WaitLp_h:
		{
			print_howie("==%d===\n",__LINE__);
			print_howie("HAVE A TEST==0x%x===\n",pAutoTest_Packet->LoopQueue[pAutoTest_Packet->ReadIndex]);
			if(size > 0)
			{
				pAutoTest_Packet->ReadIndex &= (pAutoTest_Packet->QueueCapacity -1);
				pAutoTest_Packet->DecoderData[pAutoTest_Packet->DecoderIndex++] = pAutoTest_Packet->LoopQueue[pAutoTest_Packet->ReadIndex++]; 
				pAutoTest_Packet->CollectorState = WaitLp_l;
				size--;
			}
			else
			{
				break;
			}
		}
		case WaitLp_l:
		{
			print_howie("==%d===\n",__LINE__);
			print_howie("HAVE A TEST==0x%x===\n",pAutoTest_Packet->LoopQueue[pAutoTest_Packet->ReadIndex]);
			if(size > 0)
			{
				pAutoTest_Packet->ReadIndex &= (pAutoTest_Packet->QueueCapacity -1);
				pAutoTest_Packet->PacketLength = pAutoTest_Packet->LoopQueue[pAutoTest_Packet->ReadIndex]; 
				pAutoTest_Packet->DecoderData[pAutoTest_Packet->DecoderIndex++] = pAutoTest_Packet->LoopQueue[pAutoTest_Packet->ReadIndex++];
				pAutoTest_Packet->CollectorState = WaitP1;
				size--;
			}
			else
			{
				break;
			}
		}
		case WaitP1:
		{
			print_howie("==%d===\n",__LINE__);
			print_howie("HAVE A TEST==0x%x===\n",pAutoTest_Packet->LoopQueue[pAutoTest_Packet->ReadIndex]);
			if(size > 0)
			{
				pAutoTest_Packet->ReadIndex &= (pAutoTest_Packet->QueueCapacity -1);
				pAutoTest_Packet->DecoderData[pAutoTest_Packet->DecoderIndex++] = pAutoTest_Packet->LoopQueue[pAutoTest_Packet->ReadIndex++];
				pAutoTest_Packet->CollectorState = WaitP2_h;
				size--;
			}
			else
			{
				break;
			}
		}
		case WaitP2_h:
		{
			print_howie("==%d===\n",__LINE__);
			print_howie("HAVE A TEST==0x%x===\n",pAutoTest_Packet->LoopQueue[pAutoTest_Packet->ReadIndex]);
			if(size > 0)
			{
				pAutoTest_Packet->ReadIndex &= (pAutoTest_Packet->QueueCapacity -1);
				pAutoTest_Packet->DecoderData[pAutoTest_Packet->DecoderIndex++] = pAutoTest_Packet->LoopQueue[pAutoTest_Packet->ReadIndex++];
				pAutoTest_Packet->CollectorState = WaitP2_l;
				size--;
			}
			else
			{
				break;
			}
		}
		case WaitP2_l:
		{
			print_howie("==%d===\n",__LINE__);
			print_howie("HAVE A TEST==0x%x===\n",pAutoTest_Packet->LoopQueue[pAutoTest_Packet->ReadIndex]);
			if(size > 0)
			{
				pAutoTest_Packet->ReadIndex &= (pAutoTest_Packet->QueueCapacity -1);
				pAutoTest_Packet->DecoderData[pAutoTest_Packet->DecoderIndex++] = pAutoTest_Packet->LoopQueue[pAutoTest_Packet->ReadIndex++];
				pAutoTest_Packet->CollectorState = WaitP3;
				size--;
			}
			else
			{
				break;
			}
		}
		case WaitP3:
		{
			print_howie("==%d===\n",__LINE__);
			print_howie("HAVE A TEST==0x%x===\n",pAutoTest_Packet->LoopQueue[pAutoTest_Packet->ReadIndex]);
			if(size > 0)
			{
				pAutoTest_Packet->ReadIndex &= (pAutoTest_Packet->QueueCapacity -1);
				pAutoTest_Packet->DecoderData[pAutoTest_Packet->DecoderIndex++] = pAutoTest_Packet->LoopQueue[pAutoTest_Packet->ReadIndex++];
				pAutoTest_Packet->CollectorState = WaitCRC;
				size--;
			}
			else
			{
				break;
			}
		}
		case WaitCRC:
		{
			print_howie("==%d===\n",__LINE__);
			print_howie("HAVE A TEST==0x%x===\n",pAutoTest_Packet->LoopQueue[pAutoTest_Packet->ReadIndex]);//0xea
			if(size > 0)
			{
				pAutoTest_Packet->ReadIndex &= (pAutoTest_Packet->QueueCapacity -1);
				print_howie("==%d===\n",__LINE__);
				print_howie("HAVE A TEST==0x%x===\n",pAutoTest_Packet->DecoderData[0]);
				print_howie("HAVE A TEST==0x%x===\n",pAutoTest_Packet->DecoderData[1]);
				print_howie("HAVE A TEST==0x%x===\n",pAutoTest_Packet->DecoderData[2]);
				print_howie("HAVE A TEST==0x%x===\n",pAutoTest_Packet->DecoderData[3]);
				print_howie("HAVE A TEST==0x%x===\n",pAutoTest_Packet->DecoderData[4]);
				print_howie("HAVE A TEST==0x%x===\n",pAutoTest_Packet->DecoderData[5]);
				print_howie("HAVE A TEST==0x%x===\n",pAutoTest_Packet->DecoderData[6]);
				print_howie("HAVE A TEST==0x%x===\n",pAutoTest_Packet->DecoderData[7]);				
				
				print_howie("HAVE A TEST==0x%x===\n",AutoTestCRC8(pAutoTest_Packet->DecoderData, 4));//0x15
				
				
				if(AutoTestCRC8(pAutoTest_Packet->DecoderData, 4) == pAutoTest_Packet->LoopQueue[pAutoTest_Packet->ReadIndex]) 
				{
					pAutoTest_Packet->DecoderData[pAutoTest_Packet->DecoderIndex++] = pAutoTest_Packet->LoopQueue[pAutoTest_Packet->ReadIndex++];
					size--;
					print_howie("==%d===\n",__LINE__);
					if(pAutoTest_Packet->PacketLength == 9)
					{
						//decodedSize = size - pAutoTest_Packet->Decoderlength;
						decodedSize -= size;
						pAutoTest_Packet->CollectorState = WaitCmd;
						pAutoTest_Packet->Decoderlength = 0;
						pAutoTest_Packet->DecoderIndex = 0;
						PaketComplete = TRUE;
						print_howie("==%d===\n",__LINE__);
						break;
					}
					else
					{
						pAutoTest_Packet->CollectorState = WaitPayload;
					}
				}
				else
				{
					//CRC check fail
					size--;
					pAutoTest_Packet->CollectorState = WaitCmd;
					pAutoTest_Packet->DecoderIndex = 0;
					pAutoTest_Packet->ReadIndex++;
					//decodedSize = size - pAutoTest_Packet->Decoderlength;
					decodedSize -= size;					
					break;
				}
				
			}
			else
			{
				break;
			}
		}
		case WaitPayload:
		{
			print_howie("==%d===\n",__LINE__);
			if(size > 0)
			{
				while(size--)
				{
					//size--;
					pAutoTest_Packet->Decoderlength++;
					pAutoTest_Packet->ReadIndex &= (pAutoTest_Packet->QueueCapacity -1);
					pAutoTest_Packet->DecoderData[pAutoTest_Packet->DecoderIndex++] = pAutoTest_Packet->LoopQueue[pAutoTest_Packet->ReadIndex++];
					if((pAutoTest_Packet->PacketLength - 9) == pAutoTest_Packet->Decoderlength)
					{
						//decodedSize = size - pAutoTest_Packet->Decoderlength;
						decodedSize -= size;
						pAutoTest_Packet->CollectorState = WaitCmd;
						pAutoTest_Packet->Decoderlength = 0;
						pAutoTest_Packet->DecoderIndex = 0;
						PaketComplete = TRUE;
						break;
					}
				}
			}
			break;
		}
		default:
		{
			break;
		}					
	}
	print_howie("==%d===\n",__LINE__);
	if(decodedSize > 0)
	{
		taskENTER_CRITICAL();
		AutoTest_Packet.QueueSize -= decodedSize;
		taskEXIT_CRITICAL();
	}
return PaketComplete;				
}


/**
 * @used by ltp to conduct auto test module.
 * @param cmd-- auto test cmd
 		   copmsk --use cases of option parameter
 		   pOpt -- address of option parameter
 		   lenPara --payload length of LTP packet about auto test 
 		   pPara -- payload address of LTP packet about auto test
 * @return  none.
*/
void AutoTestHandle(uint8_t cmd, uint8_t copmsk, uint8_t* pOpt, uint16_t lenPara, uint8_t* pPara)
{
	ConvertBytesToAutoTestCmd(cmd, copmsk, pOpt, lenPara, pPara, &AutoTest_cmd);

	/*packet type*/
	switch((AutoTest_cmd.HeaderTypeDef.P2_l >> 4) & 0x0f)
	{
		case 0x01:
		{
			AutoTestPWMHandle(&AutoTest_cmd);
			break;
		}
		case 0x02:
		{	
			AutoTestUartHandle(cmd, copmsk, pOpt,lenPara,pPara,&AutoTest_cmd);
			break;
		}	
		case 0x03:
		{
			AutoTestGPIOHandle(&AutoTest_cmd);
			print_howie("==AutoTestGPIOHandle===");
			break;
		}
		case 0x04:
		{
			AutoTestTimerHandle(&AutoTest_cmd);
			break;
		}
		case 0x05:
		{
			AutoTestI2CHandle(&AutoTest_cmd);
			break;
		}
		case 0x06:
		{
			AutoTestSPIHandle(&AutoTest_cmd);
			break;
		}
		case 0x08:
		{
			//AutoTestIRHandle(&AutoTest_cmd);
		}
		case 0x09:
		{
			AutoTestKeyScanHandle(&AutoTest_cmd);	
		}
		default:
		{
			break;
		}
	}
}

void ConvertBytesToAutoTestCmd(uint8_t cmd, uint8_t copmsk, uint8_t* pOpt, 
                                               uint16_t lenPara, uint8_t* pPara, AutoTest_CmdTypeDef *pAutoTest_cmd)
{	
	/*header*/
	pAutoTest_cmd->HeaderTypeDef.cmd = cmd;
	pAutoTest_cmd->HeaderTypeDef.copmsk = copmsk;
	pAutoTest_cmd->HeaderTypeDef.lp_h = 0;
	pAutoTest_cmd->HeaderTypeDef.lp_l = 0;
	pAutoTest_cmd->HeaderTypeDef.P1 = *pOpt++;
	pAutoTest_cmd->HeaderTypeDef.P2_h = *pOpt++;
	pAutoTest_cmd->HeaderTypeDef.P2_l = *pOpt++;
	pAutoTest_cmd->HeaderTypeDef.P3 = *pOpt;
	
	/*AutoTest  parameter handle*/
	AutoTestParameterHandle(pPara, lenPara, pAutoTest_cmd);	
}

void AutoTestParameterHandle(uint8_t* pPara, uint16_t lenPara, AutoTest_CmdTypeDef *pAutoTest_cmd)
{
	uint8_t data_index = 0;
	
	if(lenPara > 0)
	{
	 	switch ((pAutoTest_cmd->HeaderTypeDef.P2_l>>4)&0x0f)
	 	{
			case 0x1:
			{
				AutoTestPWMParameterHandle(pPara, &pAutoTest_cmd->InitTypeDef.pwm_params);
				break;
			}			
			case 0x2:
			{
				/*uart change baudrate*/
				AutoTestUartParameterHandle(pPara, pAutoTest_cmd);
				break;
			}		
			case 0x3:
			{	
				
				if(pAutoTest_cmd->HeaderTypeDef.P2_l == 0x37)
				{
					/*GPIO write port*/
					pAutoTest_cmd->DataTypeDef.AutoTest_GPIO_Data.port_value= *pPara++;
					pAutoTest_cmd->DataTypeDef.AutoTest_GPIO_Data.port_value=(pAutoTest_cmd->DataTypeDef.AutoTest_GPIO_Data.port_value<<8)+ *pPara++;
					pAutoTest_cmd->DataTypeDef.AutoTest_GPIO_Data.port_value=(pAutoTest_cmd->DataTypeDef.AutoTest_GPIO_Data.port_value<<8)+ *pPara++;
					pAutoTest_cmd->DataTypeDef.AutoTest_GPIO_Data.port_value=(pAutoTest_cmd->DataTypeDef.AutoTest_GPIO_Data.port_value<<8)+ *pPara;
				}
				else
				{
					if(pAutoTest_cmd->HeaderTypeDef.P2_l == 0x32)
					{
						AutoTestGPIOParamHandle(pPara, &pAutoTest_cmd->InitTypeDef.GPIO_params);
						pAutoTest_cmd->DataTypeDef.AutoTest_GPIO_Data.GPIO_voltage_level = *(pPara+6); 
					}
				}	
				
				break;
			}			
			case 0x4:
			{
				AutoTestTimerParamHandle(pPara, &pAutoTest_cmd->InitTypeDef.TIM_params);
				break;
			}
			case 0x5:
			{
				
				/*I2C write data*/
				if((pAutoTest_cmd->HeaderTypeDef.P2_l == 0x54) |(pAutoTest_cmd->HeaderTypeDef.P2_l == 0x5e)|(pAutoTest_cmd->HeaderTypeDef.P2_l == 0x56))
				{
					pAutoTest_cmd->DataTypeDef.AutoTest_I2C_Data.i2c_data_length = lenPara;
					
					/*I2C RepeatRead_ReadDataCount handle*/
					if((pAutoTest_cmd->HeaderTypeDef.P2_l == 0x5e)|(pAutoTest_cmd->HeaderTypeDef.P2_l == 0x56))
					{
						(pAutoTest_cmd->DataTypeDef.AutoTest_I2C_Data.i2c_data_length)--;
						pAutoTest_cmd->DataTypeDef.AutoTest_I2C_Data.ReadDataCount = *pPara++;
					}
					
					/*I2C write data handle*/
					for(data_index=0; data_index < pAutoTest_cmd->DataTypeDef.AutoTest_I2C_Data.i2c_data_length; data_index++)
						{
							pAutoTest_cmd->DataTypeDef.AutoTest_I2C_Data.i2c_write_buffer[data_index] = *pPara++;
						}
				}
				else
				{
					AutoTestI2CParamHandle(pPara, &pAutoTest_cmd->InitTypeDef.I2C_Params);	
				}						
				break;
			}
			/*SPI param*/
			case 0x6:
			{
				
				/* SPI write data */
				if((pAutoTest_cmd->HeaderTypeDef.P2_l == 0x61)| (pAutoTest_cmd->HeaderTypeDef.P2_l == 0x65)| (pAutoTest_cmd->HeaderTypeDef.P2_l == 0x66) || (pAutoTest_cmd->HeaderTypeDef.P2_l == 0x6a))
				{
					pAutoTest_cmd->DataTypeDef.AutoTest_SPI_Data.spi_data_length = lenPara;
					
					/* store test data */
					memcpy(pAutoTest_cmd->DataTypeDef.AutoTest_SPI_Data.spi_write_buffer, pPara, pAutoTest_cmd->DataTypeDef.AutoTest_SPI_Data.spi_data_length);
				}
				/*SPI read data by DMA*/
				else if((pAutoTest_cmd->HeaderTypeDef.P2_l == 0x67)| (pAutoTest_cmd->HeaderTypeDef.P2_l == 0x68))
				{
					
					pAutoTest_cmd->DataTypeDef.AutoTest_SPI_Data.spi_read_length = *pPara++;
				}
				else
				{
					AutoTestSPIParamHandle(pPara, &pAutoTest_cmd->InitTypeDef.SPI_Params);
				}
				break;
			}
			/*dump pin status*/
			case 0x07:
			{
				#if 0
				pAutoTest_cmd->PinStatusTypeDef.AutoTest_pin_status.gpio_group = (GPIOGroupDef)*pPara++;
				pAutoTest_cmd->PinStatusTypeDef.AutoTest_pin_status.gpio_pin_index= *pPara;
				data_index = AutoTest_ReadPinStatus(pAutoTest_cmd->PinStatusTypeDef.AutoTest_pin_status.gpio_group,pAutoTest_cmd->PinStatusTypeDef.AutoTest_pin_status.gpio_pin_index);
				AutoTestSendData(0x70, &data_index, 1);
				#endif
				break;
			}
			/*IR param*/
			case 0x08:
			{
				#if 0
				/*IR send data*/
				if(pAutoTest_cmd->HeaderTypeDef.P2_l == 0x8b)
				{
					pAutoTest_cmd->DataTypeDef.AutoTest_IR_Data.ir_data_length = lenPara;
						
					/*IR write data handle*/
					for(data_index=0; data_index < pAutoTest_cmd->DataTypeDef.AutoTest_IR_Data.ir_data_length; data_index++)
					{
						pAutoTest_cmd->DataTypeDef.AutoTest_IR_Data.ir_write_buffer[data_index] = (IR_KEY_DEF)*pPara++;
					}
				}
				else
				
				{
					//AutoTestIRParamHandle(pPara, &pAutoTest_cmd->InitTypeDef.IRDA_Params, pAutoTest_cmd);
				}	
				#endif
				break;
			}
			/*IR param*/
			case 0x09:
			{
				AutoTestKeyScanParamHandle(pPara, &pAutoTest_cmd->InitTypeDef.keypad_param);	
				break;
			}
			default:
			{
				break;
			}
	 	}
	}
}

uint8_t AutoTestSendData(uint8_t FunNum, uint8_t *payload, uint32_t payload_length)
{
	uint8_t idx = 0;
	uint8_t AutoTest_LTP_Packet[256] = {0xf6,0x8f,0,0,0,0,0,0,0};
	uint8_t *p_send = AutoTest_LTP_Packet; 
	
	AutoTest_LTP_Packet[3] = 9 + payload_length;
	AutoTest_LTP_Packet[6] = FunNum;
	AutoTest_LTP_Packet[8] = AutoTestCRC8(AutoTest_LTP_Packet, 4);
	
    if(payload_length > 0)
    {
		for(idx = 0; idx < payload_length; idx++)
		{
			AutoTest_LTP_Packet[9 + idx] = *payload;
			payload++;
		}
	}
	
	for(idx = 0; idx < AutoTest_LTP_Packet[3]; idx++)
   {
		while(UART_GetFlagState(UART, UART_FLAG_THR_EMPTY)==RESET);
		UART_SendData(UART, p_send++, 1);
   }
	return 1;
}

void AutoTestUartHandle(uint8_t cmd, uint8_t copmsk, uint8_t* pOpt, uint16_t lenPara, uint8_t* pPara, AutoTest_CmdTypeDef *pAutoTest_cmd )
{
	uint8_t idx;
	switch((pAutoTest_cmd->HeaderTypeDef.P2_l & 0x0f))
	{
		case 1:
		{
			AutoTestSendData(0x21, NULL, 0);
			vTaskDelay(200 / portTICK_RATE_MS);
			/* system reset */
			NVIC_SystemReset();	
			break;
		}
		case 2:
		{
			BaudRateModifyHandle(pAutoTest_cmd->InitTypeDef.uart_params.OVSR, pAutoTest_cmd->InitTypeDef.uart_params.DIV, pAutoTest_cmd->InitTypeDef.uart_params.OVSR_ADJ);	
			break;
		}
		case 3:
		{
			AutoTestUartLoopBackHandle(cmd, copmsk, pOpt, lenPara, pPara);
			break;
		}
		case 4:
		{
			AutoTestSendData(0x24, REG_PERI_uart_Params, 8);
			for(idx=0 ; idx<8; idx++)
			{
				REG_PERI_uart_Params[idx] = 0;
			}
			break;
		}
		default:
		{
			break;
		}
	}
}

void AutoTestUartParameterHandle(uint8_t* pPara, AutoTest_CmdTypeDef *pAutoTest_cmd)
{
	if(pAutoTest_cmd->HeaderTypeDef.P2_l == 0x22)
	{
		pAutoTest_cmd->InitTypeDef.uart_params.OVSR= *pPara++;
		pAutoTest_cmd->InitTypeDef.uart_params.DIV= *pPara++;
		pAutoTest_cmd->InitTypeDef.uart_params.DIV = (pAutoTest_cmd->InitTypeDef.uart_params.DIV<<8) + *pPara++;
		pAutoTest_cmd->InitTypeDef.uart_params.OVSR_ADJ= *pPara++;
		pAutoTest_cmd->InitTypeDef.uart_params.OVSR_ADJ = (pAutoTest_cmd->InitTypeDef.uart_params.OVSR_ADJ<<8) + *pPara;
	}
}

void BaudRateModifyHandle(uint8_t ovsr, uint16_t div, uint16_t ovsr_adj)
{	
	/*add for Initialize and Deinitialize function test*/
	ReadPeriFunRegisterByPeripheralType(AutoTest_PeripherialType_DATA_UART, &REG_PERI_uart_Params[0], &REG_PERI_uart_Params[1]);
	/* data uart deinit */
	UART_DeInit(UART);
	ReadPeriFunRegisterByPeripheralType(AutoTest_PeripherialType_DATA_UART, &REG_PERI_uart_Params[2], &REG_PERI_uart_Params[3]);
	

	/* NVIC configuration */
	NVIC_InitTypeDef NVIC_InitStruct;
	NVIC_InitStruct.NVIC_IRQChannel = UART_IRQ;
	NVIC_InitStruct.NVIC_IRQChannelPriority = 0;
	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStruct);

	/* uart init */
	UART_InitTypeDef uartInitStruct;
	uartInitStruct.div = div;
	uartInitStruct.ovsr = ovsr;
	uartInitStruct.ovsr_adj = ovsr_adj;

	uartInitStruct.parity = UART_PARITY_NO_PARTY;
	uartInitStruct.stopBits = UART_STOP_BITS_1;
	uartInitStruct.wordLen = UART_WROD_LENGTH_8BIT;
	uartInitStruct.dmaEn = UART_DMA_DISABLE;
	uartInitStruct.autoFlowCtrl= UART_AUTO_FLOW_CTRL_DIS;
	uartInitStruct.rxTriggerLevel= UART_RX_FIFO_TRIGGER_LEVEL_14BYTE;

	/*add for Initialize and Deinitialize function test*/
	ReadPeriFunRegisterByPeripheralType(AutoTest_PeripherialType_DATA_UART, &REG_PERI_uart_Params[4], &REG_PERI_uart_Params[5]);

	/* System clock configuration */
	RCC_PeriphClockCmd(APBPeriph_UART, APBPeriph_UART_CLOCK, ENABLE);

	/* data uart init */
	
	UART_Init(UART, &uartInitStruct);

	//enable rx interrupt and line status interrupt
	UART_INTConfig(UART, UART_INT_RD_AVA | UART_INT_LINE_STS, ENABLE);

    ReadPeriFunRegisterByPeripheralType(AutoTest_PeripherialType_DATA_UART, &REG_PERI_uart_Params[6], &REG_PERI_uart_Params[7]);

}

void AutoTestUartLoopBackHandle(uint8_t cmd, uint8_t copmsk, uint8_t* pOpt,uint16_t lenPara, uint8_t* pPara)
{
	uint8_t* p_send = NULL;
	uint8_t* p_send_index = NULL;
	uint8_t* pOpt_index = NULL;
	/*length of P1, P2, P3*/
	uint8_t P_length = 4;
	uint8_t i = 0;
	
	pOpt_index = pOpt;
	uint8_t LoopBackBuffer[256];
	p_send_index = p_send = LoopBackBuffer;
	*p_send_index = cmd;
	p_send_index++;
	*p_send_index = copmsk;
	p_send_index++;		
	*p_send_index = (lenPara+9)/256;
	p_send_index++;
	*p_send_index = (lenPara+9)%256;
	p_send_index++;

    while(P_length)
	{
		*p_send_index = *pOpt_index++;
		p_send_index++;
		P_length--;
	}
	*p_send_index = *(pPara - 1);
	p_send_index++;
	
	memcpy(p_send_index, pPara,lenPara);
	for(i = 0; i < lenPara + 9; i++)
   {
     	while(UART_GetFlagState(UART, UART_FLAG_THR_EMPTY)==RESET);
		UART_SendData(UART, p_send++, 1);
   }

}


uint8_t AutoTest_ReadPeriFunRegister(uint32_t base, uint32_t addr, uint32_t BIT_PERI_EN)
{	
	uint32_t IOModule_ENValue = 0;
	uint8_t BIT_PERI_Value = 0;
	
	IOModule_ENValue = HAL_READ32(base,addr);
	if(IOModule_ENValue & BIT_PERI_EN)
	{
		BIT_PERI_Value = 1;
	}
	return BIT_PERI_Value;
}
void ReadPeriFunRegisterByPeripheralType(InitOrDeinitPeripherialType periType, uint8_t *FUNC_param, uint8_t *CLK_param)
{
	 switch(periType)
    {
    	case AutoTest_PeripherialType_PWM:
            {
				break;
        	}
		case AutoTest_PeripherialType_DATA_UART:
            {
				*FUNC_param = AutoTest_ReadPeriFunRegister(PERIPH_REG_BASE, REG_SOC_PERI_FUNC0_EN, BIT_PERI_UART0_EN); 
				*CLK_param = AutoTest_ReadPeriFunRegister(PERIPH_REG_BASE, REG_PESOC_PERI_CLK_CTRL0, BIT_SOC_ACTCK_UART0_EN);
           		break;
        	}
		case AutoTest_PeripherialType_GPIO:
            {
				*FUNC_param = AutoTest_ReadPeriFunRegister(PERIPH_REG_BASE, REG_SOC_PERI_FUNC1_EN, BIT_PERI_GPIO_EN);
				*CLK_param = AutoTest_ReadPeriFunRegister(PERIPH_REG_BASE, REG_PESOC_CLK_CTRL, BIT_SOC_ACTCK_GPIO_EN);
           		break;
        	}
		case AutoTest_PeripherialType_Timer:
            {
				*FUNC_param = AutoTest_ReadPeriFunRegister(PERIPH_REG_BASE, REG_PESOC_CLK_CTRL, BIT_SOC_ACTCK_TIMER_EN);
				break;
        	}
		case AutoTest_PeripherialType_I2C0:
            {
				*FUNC_param = AutoTest_ReadPeriFunRegister(PERIPH_REG_BASE, REG_SOC_PERI_FUNC0_EN, BIT_PERI_I2C0_EN);
				*CLK_param = AutoTest_ReadPeriFunRegister(PERIPH_REG_BASE, REG_PESOC_PERI_CLK_CTRL1, BIT_SOC_ACTCK_I2C0_EN);
           		break;
        	}
		case AutoTest_PeripherialType_I2C1:
            {
				*FUNC_param = AutoTest_ReadPeriFunRegister(PERIPH_REG_BASE, REG_SOC_PERI_FUNC0_EN, BIT_PERI_I2C1_EN);
				*CLK_param = AutoTest_ReadPeriFunRegister(PERIPH_REG_BASE, REG_PESOC_PERI_CLK_CTRL1, BIT_SOC_ACTCK_I2C1_EN);
           		break;
        	}
        case AutoTest_PeripherialType_SPI0:
            {
				*FUNC_param = AutoTest_ReadPeriFunRegister(PERIPH_REG_BASE, REG_SOC_PERI_FUNC0_EN, BIT_PERI_SPI0_EN);
				*CLK_param = AutoTest_ReadPeriFunRegister(PERIPH_REG_BASE, REG_PESOC_PERI_CLK_CTRL0, BIT_SOC_ACTCK_SPI0_EN);
           		break;
        	}
		case AutoTest_PeripherialType_SPI1:
            {
				*FUNC_param = AutoTest_ReadPeriFunRegister(PERIPH_REG_BASE, REG_SOC_PERI_FUNC0_EN, BIT_PERI_SPI1_EN);
				*CLK_param = AutoTest_ReadPeriFunRegister(PERIPH_REG_BASE, REG_PESOC_PERI_CLK_CTRL0, BIT_SOC_ACTCK_SPI1_EN);
           		break;
        	}
		case AutoTest_PeripherialType_Keyscan:
            {
				*FUNC_param = AutoTest_ReadPeriFunRegister(PERIPH_REG_BASE, REG_SOC_PERI_FUNC0_EN, BIT_PERI_KEYSCAN_EN);
				*CLK_param = AutoTest_ReadPeriFunRegister(PERIPH_REG_BASE, REG_PESOC_PERI_CLK_CTRL1, BIT_SOC_ACTCK_KEYSCAN_EN);
           		break;
        	}
		default:
			{
				break;
			}
	}
}

void AutoTestGPIOParamHandle(uint8_t* pPara, GPIO_InitTypeDef *pAutoTest_GPIO_param)
{
	pAutoTest_GPIO_param->GPIO_Pin= GPIO_GetPin(*pPara++);
	pAutoTest_GPIO_param->GPIO_Mode = (GPIOMode_TypeDef)*pPara++;
	pAutoTest_GPIO_param->GPIO_ITCmd = (FunctionalState)*pPara++;	
	pAutoTest_GPIO_param->GPIO_ITTrigger = (GPIOIT_LevelType)*pPara++;
	pAutoTest_GPIO_param->GPIO_ITPolarity= (GPIOIT_PolarityType)*pPara++;
	pAutoTest_GPIO_param->GPIO_ITDebounce = (GPIOIT_DebounceType)*pPara++;
	
}

void AutoTestGPIOHandle(AutoTest_CmdTypeDef *pAutoTest_cmd )
{	
	uint8_t send_data[10];	
	switch((pAutoTest_cmd->HeaderTypeDef.P2_l & 0x0f))
	{
		case 0:
		{

			/*add for Deinit all pin*/
			#if 0
			for (periType = PeripherialType_HCI_UART; periType < PeripherialType_MAX; periType++)
			{
				if ((periType != PeripherialType_DATA_UART)&&(periType != PeripherialType_SWD))
				
				{
					HalDumpOrDeinitPinByPeripheralType(periType, FALSE, TRUE);
				}
			}
			//DumpPinStatus(); 
			#endif
			AutoTestSendData(0x30, NULL, 0);
			break;
		}
		case 1:
		{
			GPIO_MaskINTConfig(pAutoTest_cmd->InitTypeDef.GPIO_params.GPIO_Pin,ENABLE);
			AutoTestSendData(0x31, send_data, 0);
			break;
		}
		case 2:
		{		
			UINT8 pin_num=0x0;
			ReadPeriFunRegisterByPeripheralType(AutoTest_PeripherialType_GPIO,&send_data[0], &send_data[1]);
			for(UINT32 i=0;i<32;i++)
				if(pAutoTest_cmd->InitTypeDef.GPIO_params.GPIO_Pin==BIT(i))
					pin_num=i;
			if((28<=pin_num)&&(pin_num<=31))
					pin_num=pin_num+4;
			RCC_PeriphClockCmd(APBPeriph_GPIO, APBPeriph_GPIO_CLOCK, ENABLE);
			GPIO_Init(&pAutoTest_cmd->InitTypeDef.GPIO_params);
			
			ReadPeriFunRegisterByPeripheralType(AutoTest_PeripherialType_GPIO,&send_data[2], &send_data[3]);
			/*read pinmux status*/
			send_data[4] = AutoTest_ReadPinStatus(pin_num);
			AutoTestSendData(0x32, send_data, 5);
			break;
		}
		case 3:
		{
			UINT8 pin_num=0x0;
			ReadPeriFunRegisterByPeripheralType(AutoTest_PeripherialType_GPIO,&send_data[0], &send_data[1]);
			GPIO_DeInit();
			for(UINT32 i=0;i<32;i++)
				if(pAutoTest_cmd->InitTypeDef.GPIO_params.GPIO_Pin==BIT(i))
					pin_num=i;
			if((28<=pin_num)&&(pin_num<=31))
					pin_num=pin_num+4;
			ReadPeriFunRegisterByPeripheralType(AutoTest_PeripherialType_GPIO,&send_data[2], &send_data[3]);
			/*read pinmux status*/
			send_data[4] = AutoTest_ReadPinStatus(pin_num);
			AutoTestSendData(0x33, send_data, 5);
			break;
		}
		case 4:
		{	
			GPIO_GetINTStatus(pAutoTest_cmd->InitTypeDef.GPIO_params.GPIO_Pin);
			AutoTestSendData(0x34, send_data, 0); 
			break;
		}
		case 5:
		{
			GPIO_WriteBit(pAutoTest_cmd->InitTypeDef.GPIO_params.GPIO_Pin,(BitAction) (pAutoTest_cmd->DataTypeDef.AutoTest_GPIO_Data.GPIO_voltage_level));
			send_data[0] = pAutoTest_cmd->DataTypeDef.AutoTest_GPIO_Data.GPIO_voltage_level;
			AutoTestSendData(0x35, send_data, 1);
			break;
		}
		case 6:
		{
			//send_data[0] = GPIO_Write_V2(&pAutoTest_cmd->InitTypeDef.GPIO_param, pAutoTest_cmd->DataTypeDef.AutoTest_GPIO_Data.GPIO_voltage_level);
			//AutoTestSendData(0x36, send_data, 1);
			break;
		}
		case 7:
		{
			GPIO_Write(pAutoTest_cmd->DataTypeDef.AutoTest_GPIO_Data.port_value);
			AutoTestSendData(0x37, send_data, 0);					
			break;
		}
		case 8:
		{
			GPIO_MaskINTConfig(pAutoTest_cmd->InitTypeDef.GPIO_params.GPIO_Pin,DISABLE);
			AutoTestSendData(0x38, send_data, 0);
			break;
		}
		case 9:
		{
			uint32_t ReadInputData = 0;
			ReadInputData=GPIO_ReadInputData();
			send_data[3]=(ReadInputData&0xff000000)>>24;
			send_data[2]=(ReadInputData&0x00ff0000)>>16;
			send_data[1]=(ReadInputData&0x0000ff00)>>8;
			send_data[0]=ReadInputData&0x0000000ff;
			AutoTestSendData(0x39, send_data, 4);
			break;
		}
		case 0x0a:
		{
			send_data[0]=GPIO_ReadInputDataBit(pAutoTest_cmd->InitTypeDef.GPIO_params.GPIO_Pin);
			AutoTestSendData(0x3a, send_data, 1);
			break;
		}
		case 0x0b:
		{
//			NVIC_InitTypeDef NVIC_InitStruct;
//			if(pAutoTest_cmd->InitTypeDef.GPIO_params.GPIO_Pin==GPIO_Pin_0)
//				NVIC_InitStruct.NVIC_IRQChannel = 9;
//			else if(pAutoTest_cmd->InitTypeDef.GPIO_params.GPIO_Pin==GPIO_Pin_1)
//				NVIC_InitStruct.NVIC_IRQChannel = 10;
//			else if(pAutoTest_cmd->InitTypeDef.GPIO_params.GPIO_Pin==GPIO_Pin_2)
//				NVIC_InitStruct.NVIC_IRQChannel = 9;
//			else if(pAutoTest_cmd->InitTypeDef.GPIO_params.GPIO_Pin==GPIO_Pin_3)
//				NVIC_InitStruct.NVIC_IRQChannel = 9;
//			else if(pAutoTest_cmd->InitTypeDef.GPIO_params.GPIO_Pin==GPIO_Pin_4)
//				NVIC_InitStruct.NVIC_IRQChannel = 9;
//			else if(pAutoTest_cmd->InitTypeDef.GPIO_params.GPIO_Pin==GPIO_Pin_5)
//				NVIC_InitStruct.NVIC_IRQChannel = 9;
//			else
//			NVIC_InitStruct.NVIC_IRQChannel = 9;
//			NVIC_InitStruct.NVIC_IRQChannelPriority = 2;
//			NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
//			NVIC_Init(&NVIC_InitStruct);
			
			GPIO_INTConfig(pAutoTest_cmd->InitTypeDef.GPIO_params.GPIO_Pin,ENABLE);
			
			AutoTestSendData(0x3b, send_data, 0);
			break;
		}
		case 0x0c:
		{
			GPIO_INTConfig(pAutoTest_cmd->InitTypeDef.GPIO_params.GPIO_Pin,DISABLE);
			AutoTestSendData(0x3c, send_data, 0);
			break;
		}
		case 0x0d:
		{
			GPIO_INTConfig(pAutoTest_cmd->InitTypeDef.GPIO_params.GPIO_Pin,ENABLE);
			AutoTestSendData(0x3d, send_data, 0);
			break;
		}
		case 0x0e:
		{
			GPIO_ClearINTPendingBit(pAutoTest_cmd->InitTypeDef.GPIO_params.GPIO_Pin);
			AutoTestSendData(0x3e, send_data, 0);
			break;
		}
		
		case 0x0f:
		{
			AutoTestSendData(0x3f, send_data, 0);
			vTaskDelay(200 / portTICK_RATE_MS);
			NVIC_SystemReset();
			break;
		}
		default:
		{
			break;
		}
	}
}

void Gpio0IntrHandler(void)
{
	uint32_t GPIO_GetINTStatus_Vaule = 0;
	
	GPIO_GetINTStatus_Vaule = GPIO->INTSTATUS;
	GPIO->INTMASK |= GPIO_GetINTStatus_Vaule;	

  AutosendInt32(GPIO_GetINTStatus_Vaule);	
	GPIO_ClearINTPendingBit(GPIO_GetINTStatus_Vaule);
	GPIO->INTMASK &= ~(GPIO_GetINTStatus_Vaule);
}
void Gpio1IntrHandler(void)
{
	uint32_t GPIO_GetINTStatus_Vaule = 0;
	
	GPIO_GetINTStatus_Vaule = GPIO->INTSTATUS;
	GPIO->INTMASK |= GPIO_GetINTStatus_Vaule;	

  AutosendInt32(GPIO_GetINTStatus_Vaule);	
	GPIO_ClearINTPendingBit(GPIO_GetINTStatus_Vaule);
	GPIO->INTMASK &= ~(GPIO_GetINTStatus_Vaule);
}
void Gpio2IntrHandler(void)
{
	uint32_t GPIO_GetINTStatus_Vaule = 0;
	
	GPIO_GetINTStatus_Vaule = GPIO->INTSTATUS;
	GPIO->INTMASK |= GPIO_GetINTStatus_Vaule;	

  AutosendInt32(GPIO_GetINTStatus_Vaule);	
	GPIO_ClearINTPendingBit(GPIO_GetINTStatus_Vaule);
	GPIO->INTMASK &= ~(GPIO_GetINTStatus_Vaule);
}

void Gpio3IntrHandler(void)
{
	uint32_t GPIO_GetINTStatus_Vaule = 0;
	
	GPIO_GetINTStatus_Vaule = GPIO->INTSTATUS;
	GPIO->INTMASK |= GPIO_GetINTStatus_Vaule;	

  AutosendInt32(GPIO_GetINTStatus_Vaule);	
	GPIO_ClearINTPendingBit(GPIO_GetINTStatus_Vaule);
	GPIO->INTMASK &= ~(GPIO_GetINTStatus_Vaule);
}
void Gpio4IntrHandler(void)
{
	uint32_t GPIO_GetINTStatus_Vaule = 0;
	
	GPIO_GetINTStatus_Vaule = GPIO->INTSTATUS;
	GPIO->INTMASK |= GPIO_GetINTStatus_Vaule;	

    AutosendInt32(GPIO_GetINTStatus_Vaule);	
	GPIO_ClearINTPendingBit(GPIO_GetINTStatus_Vaule);
	GPIO->INTMASK &= ~(GPIO_GetINTStatus_Vaule);
}
void Gpio5IntrHandler_bak(void)
{
	uint32_t GPIO_GetINTStatus_Vaule = 0;
	
	GPIO_GetINTStatus_Vaule = GPIO->INTSTATUS;
	GPIO->INTMASK |= GPIO_GetINTStatus_Vaule;	

    AutosendInt32(GPIO_GetINTStatus_Vaule);	
	GPIO_ClearINTPendingBit(GPIO_GetINTStatus_Vaule);
	GPIO->INTMASK &= ~(GPIO_GetINTStatus_Vaule);
}
void Gpio6IntrHandler(void)
{
	uint32_t GPIO_GetINTStatus_Vaule = 0;
	
	GPIO_GetINTStatus_Vaule = GPIO->INTSTATUS;
	GPIO->INTMASK |= GPIO_GetINTStatus_Vaule;	

    AutosendInt32(GPIO_GetINTStatus_Vaule);	
	GPIO_ClearINTPendingBit(GPIO_GetINTStatus_Vaule);
	GPIO->INTMASK &= ~(GPIO_GetINTStatus_Vaule);
}
void Gpio7IntrHandler(void)
{
	uint32_t GPIO_GetINTStatus_Vaule = 0;
	
	GPIO_GetINTStatus_Vaule = GPIO->INTSTATUS;
	GPIO->INTMASK |= GPIO_GetINTStatus_Vaule;	

    AutosendInt32(GPIO_GetINTStatus_Vaule);	
	GPIO_ClearINTPendingBit(GPIO_GetINTStatus_Vaule);
	GPIO->INTMASK &= ~(GPIO_GetINTStatus_Vaule);
}
void Gpio8IntrHandler(void)
{
	uint32_t GPIO_GetINTStatus_Vaule = 0;
	
	GPIO_GetINTStatus_Vaule = GPIO->INTSTATUS;
	GPIO->INTMASK |= GPIO_GetINTStatus_Vaule;	

    AutosendInt32(GPIO_GetINTStatus_Vaule);	
	GPIO_ClearINTPendingBit(GPIO_GetINTStatus_Vaule);
	GPIO->INTMASK &= ~(GPIO_GetINTStatus_Vaule);
}
void Gpio9IntrHandler(void)
{
	uint32_t GPIO_GetINTStatus_Vaule = 0;
	
	GPIO_GetINTStatus_Vaule = GPIO->INTSTATUS;
	GPIO->INTMASK |= GPIO_GetINTStatus_Vaule;	

    AutosendInt32(GPIO_GetINTStatus_Vaule);	
	GPIO_ClearINTPendingBit(GPIO_GetINTStatus_Vaule);
	GPIO->INTMASK &= ~(GPIO_GetINTStatus_Vaule);
}
void Gpio10IntrHandler(void)
{
	uint32_t GPIO_GetINTStatus_Vaule = 0;
	
	GPIO_GetINTStatus_Vaule = GPIO->INTSTATUS;
	GPIO->INTMASK |= GPIO_GetINTStatus_Vaule;	

    AutosendInt32(GPIO_GetINTStatus_Vaule);	
	GPIO_ClearINTPendingBit(GPIO_GetINTStatus_Vaule);
	GPIO->INTMASK &= ~(GPIO_GetINTStatus_Vaule);
}
void Gpio11IntrHandler(void)
{
	uint32_t GPIO_GetINTStatus_Vaule = 0;
	
	GPIO_GetINTStatus_Vaule = GPIO->INTSTATUS;
	GPIO->INTMASK |= GPIO_GetINTStatus_Vaule;	

    AutosendInt32(GPIO_GetINTStatus_Vaule);	
	GPIO_ClearINTPendingBit(GPIO_GetINTStatus_Vaule);
	GPIO->INTMASK &= ~(GPIO_GetINTStatus_Vaule);
}
void Gpio12IntrHandler(void)
{
	uint32_t GPIO_GetINTStatus_Vaule = 0;
	
	GPIO_GetINTStatus_Vaule = GPIO->INTSTATUS;
	GPIO->INTMASK |= GPIO_GetINTStatus_Vaule;	

    AutosendInt32(GPIO_GetINTStatus_Vaule);	
	GPIO_ClearINTPendingBit(GPIO_GetINTStatus_Vaule);
	GPIO->INTMASK &= ~(GPIO_GetINTStatus_Vaule);
}
void Gpio13IntrHandler(void)
{
	uint32_t GPIO_GetINTStatus_Vaule = 0;
	
	GPIO_GetINTStatus_Vaule = GPIO->INTSTATUS;
	GPIO->INTMASK |= GPIO_GetINTStatus_Vaule;	

    AutosendInt32(GPIO_GetINTStatus_Vaule);	
	GPIO_ClearINTPendingBit(GPIO_GetINTStatus_Vaule);
	GPIO->INTMASK &= ~(GPIO_GetINTStatus_Vaule);
}
void Gpio14IntrHandler(void)
{
	uint32_t GPIO_GetINTStatus_Vaule = 0;
	
	GPIO_GetINTStatus_Vaule = GPIO->INTSTATUS;
	GPIO->INTMASK |= GPIO_GetINTStatus_Vaule;	

    AutosendInt32(GPIO_GetINTStatus_Vaule);	
	GPIO_ClearINTPendingBit(GPIO_GetINTStatus_Vaule);
	GPIO->INTMASK &= ~(GPIO_GetINTStatus_Vaule);
}
void Gpio15IntrHandler(void)
{
	uint32_t GPIO_GetINTStatus_Vaule = 0;
	
	GPIO_GetINTStatus_Vaule = GPIO->INTSTATUS;
	GPIO->INTMASK |= GPIO_GetINTStatus_Vaule;	

    AutosendInt32(GPIO_GetINTStatus_Vaule);	
	GPIO_ClearINTPendingBit(GPIO_GetINTStatus_Vaule);
	GPIO->INTMASK &= ~(GPIO_GetINTStatus_Vaule);
}

void Gpio16IntrHandler(void)
{
	uint32_t GPIO_GetINTStatus_Vaule = 0;
	
	GPIO_GetINTStatus_Vaule = GPIO->INTSTATUS;
	GPIO->INTMASK |= GPIO_GetINTStatus_Vaule;	

    AutosendInt32(GPIO_GetINTStatus_Vaule);	
	GPIO_ClearINTPendingBit(GPIO_GetINTStatus_Vaule);
	GPIO->INTMASK &= ~(GPIO_GetINTStatus_Vaule);
}

void Gpio17IntrHandler(void)
{
	uint32_t GPIO_GetINTStatus_Vaule = 0;
	
	GPIO_GetINTStatus_Vaule = GPIO->INTSTATUS;
	GPIO->INTMASK |= GPIO_GetINTStatus_Vaule;	

    AutosendInt32(GPIO_GetINTStatus_Vaule);	
	GPIO_ClearINTPendingBit(GPIO_GetINTStatus_Vaule);
	GPIO->INTMASK &= ~(GPIO_GetINTStatus_Vaule);
}
void Gpio18IntrHandler(void)
{
	uint32_t GPIO_GetINTStatus_Vaule = 0;
	
	GPIO_GetINTStatus_Vaule = GPIO->INTSTATUS;
	GPIO->INTMASK |= GPIO_GetINTStatus_Vaule;	

    AutosendInt32(GPIO_GetINTStatus_Vaule);	
	GPIO_ClearINTPendingBit(GPIO_GetINTStatus_Vaule);
	GPIO->INTMASK &= ~(GPIO_GetINTStatus_Vaule);
}
void Gpio19IntrHandler(void)
{
	uint32_t GPIO_GetINTStatus_Vaule = 0;
	
	GPIO_GetINTStatus_Vaule = GPIO->INTSTATUS;
	GPIO->INTMASK |= GPIO_GetINTStatus_Vaule;	

    AutosendInt32(GPIO_GetINTStatus_Vaule);	
	GPIO_ClearINTPendingBit(GPIO_GetINTStatus_Vaule);
	GPIO->INTMASK &= ~(GPIO_GetINTStatus_Vaule);
}
void Gpio20IntrHandler(void)
{
	uint32_t GPIO_GetINTStatus_Vaule = 0;
	
	GPIO_GetINTStatus_Vaule = GPIO->INTSTATUS;
	GPIO->INTMASK |= GPIO_GetINTStatus_Vaule;	

    AutosendInt32(GPIO_GetINTStatus_Vaule);	
	GPIO_ClearINTPendingBit(GPIO_GetINTStatus_Vaule);
	GPIO->INTMASK &= ~(GPIO_GetINTStatus_Vaule);
}
void Gpio21IntrHandler(void)
{
	uint32_t GPIO_GetINTStatus_Vaule = 0;
	
	GPIO_GetINTStatus_Vaule = GPIO->INTSTATUS;
	GPIO->INTMASK |= GPIO_GetINTStatus_Vaule;	

    AutosendInt32(GPIO_GetINTStatus_Vaule);	
	GPIO_ClearINTPendingBit(GPIO_GetINTStatus_Vaule);
	GPIO->INTMASK &= ~(GPIO_GetINTStatus_Vaule);
}
void Gpio22IntrHandler(void)
{
	uint32_t GPIO_GetINTStatus_Vaule = 0;
	
	GPIO_GetINTStatus_Vaule = GPIO->INTSTATUS;
	GPIO->INTMASK |= GPIO_GetINTStatus_Vaule;	

    AutosendInt32(GPIO_GetINTStatus_Vaule);	
	GPIO_ClearINTPendingBit(GPIO_GetINTStatus_Vaule);
	GPIO->INTMASK &= ~(GPIO_GetINTStatus_Vaule);
}
void Gpio23IntrHandler(void)
{
	uint32_t GPIO_GetINTStatus_Vaule = 0;
	
	GPIO_GetINTStatus_Vaule = GPIO->INTSTATUS;
	GPIO->INTMASK |= GPIO_GetINTStatus_Vaule;	

    AutosendInt32(GPIO_GetINTStatus_Vaule);	
	GPIO_ClearINTPendingBit(GPIO_GetINTStatus_Vaule);
	GPIO->INTMASK &= ~(GPIO_GetINTStatus_Vaule);
}
void Gpio24IntrHandler(void)
{
	uint32_t GPIO_GetINTStatus_Vaule = 0;
	
	GPIO_GetINTStatus_Vaule = GPIO->INTSTATUS;
	GPIO->INTMASK |= GPIO_GetINTStatus_Vaule;	

    AutosendInt32(GPIO_GetINTStatus_Vaule);	
	GPIO_ClearINTPendingBit(GPIO_GetINTStatus_Vaule);
	GPIO->INTMASK &= ~(GPIO_GetINTStatus_Vaule);
}
void Gpio25IntrHandler(void)
{
	uint32_t GPIO_GetINTStatus_Vaule = 0;
	
	GPIO_GetINTStatus_Vaule = GPIO->INTSTATUS;
	GPIO->INTMASK |= GPIO_GetINTStatus_Vaule;	

    AutosendInt32(GPIO_GetINTStatus_Vaule);	
	GPIO_ClearINTPendingBit(GPIO_GetINTStatus_Vaule);
	GPIO->INTMASK &= ~(GPIO_GetINTStatus_Vaule);
}
void Gpio26IntrHandler(void)
{
	uint32_t GPIO_GetINTStatus_Vaule = 0;
	
	GPIO_GetINTStatus_Vaule = GPIO->INTSTATUS;
	GPIO->INTMASK |= GPIO_GetINTStatus_Vaule;	

    AutosendInt32(GPIO_GetINTStatus_Vaule);	
	GPIO_ClearINTPendingBit(GPIO_GetINTStatus_Vaule);
	GPIO->INTMASK &= ~(GPIO_GetINTStatus_Vaule);
}
void Gpio27IntrHandler(void)
{
	uint32_t GPIO_GetINTStatus_Vaule = 0;
	
	GPIO_GetINTStatus_Vaule = GPIO->INTSTATUS;
	GPIO->INTMASK |= GPIO_GetINTStatus_Vaule;	

    AutosendInt32(GPIO_GetINTStatus_Vaule);	
	GPIO_ClearINTPendingBit(GPIO_GetINTStatus_Vaule);
	GPIO->INTMASK &= ~(GPIO_GetINTStatus_Vaule);
}
void Gpio28IntrHandler(void)
{
	uint32_t GPIO_GetINTStatus_Vaule = 0;
	
	GPIO_GetINTStatus_Vaule = GPIO->INTSTATUS;
	GPIO->INTMASK |= GPIO_GetINTStatus_Vaule;	

    AutosendInt32(GPIO_GetINTStatus_Vaule);	
	GPIO_ClearINTPendingBit(GPIO_GetINTStatus_Vaule);
	GPIO->INTMASK &= ~(GPIO_GetINTStatus_Vaule);
}
void Gpio29IntrHandler(void)
{
	uint32_t GPIO_GetINTStatus_Vaule = 0;
	
	GPIO_GetINTStatus_Vaule = GPIO->INTSTATUS;
	GPIO->INTMASK |= GPIO_GetINTStatus_Vaule;	

    AutosendInt32(GPIO_GetINTStatus_Vaule);	
	GPIO_ClearINTPendingBit(GPIO_GetINTStatus_Vaule);
	GPIO->INTMASK &= ~(GPIO_GetINTStatus_Vaule);
}
void Gpio30IntrHandler(void)
{
	uint32_t GPIO_GetINTStatus_Vaule = 0;
	
	GPIO_GetINTStatus_Vaule = GPIO->INTSTATUS;
	GPIO->INTMASK |= GPIO_GetINTStatus_Vaule;	

    AutosendInt32(GPIO_GetINTStatus_Vaule);	
	GPIO_ClearINTPendingBit(GPIO_GetINTStatus_Vaule);
	GPIO->INTMASK &= ~(GPIO_GetINTStatus_Vaule);
}
void Gpio31IntrHandler(void)
{
	uint32_t GPIO_GetINTStatus_Vaule = 0;
	
	GPIO_GetINTStatus_Vaule = GPIO->INTSTATUS;
	GPIO->INTMASK |= GPIO_GetINTStatus_Vaule;	

    AutosendInt32(GPIO_GetINTStatus_Vaule);	
	GPIO_ClearINTPendingBit(GPIO_GetINTStatus_Vaule);
	GPIO->INTMASK &= ~(GPIO_GetINTStatus_Vaule);
}

uint8_t AutoTest_ReadPinStatus(uint8_t Pin_Num)
{
	UINT32 reg_addr  = 0x0;
    UINT32 reg_value = 0x0;
    UINT8 pin_status = 0x0;
	uint8_t GPIOGroup=	Pin_Num/8;
	uint8_t GPIOIndex= Pin_Num%8;
    reg_addr =  GPIOGroup * 8 + (GPIOIndex/4)*4;
    reg_value = HAL_READ32(PINMUX_REG_BASE, reg_addr);
	pin_status = (reg_value>>(8*(GPIOIndex%4)))&0x7f;
	return pin_status;
}


void AutoTestTimerParamHandle(uint8_t* pPara, AutoTest_Tim_Params *pAutoTest_TimerAdap)
{


}



void AutoTestTimerHandle(AutoTest_CmdTypeDef *pAutoTest_cmd )
{	

	
}


void Timer1IntrHandler_bak(void)
{
   
}

void Timer2IntrHandler_bak(void)
{
   
}

void Timer3IntrHandler_bak(void)
{
   
}

void Timer4IntrHandler_bak(void)
{
 
}

void Timer5IntrHandler_bak(void)
{
    
}

void Timer6IntrHandler_bak(void)
{
 
}

void Timer7IntrHandler_bak(void)
{

}

void AutoTestPWMParameterHandle(uint8_t* pPara, AutoTest_pwm_Params *pAutoTest_PWMAdap)
{

	
}

void AutoTestPWMHandle(AutoTest_CmdTypeDef *pAutoTest_cmd )
{

}


void AutoTestI2CParamHandle(uint8_t* pPara, AutoTest_i2c_Params *pAutoTest_I2C_Params)
{
	
}


void AutoTestI2CHandle(AutoTest_CmdTypeDef *pAutoTest_cmd )
{	

}


void I2C0IntrHandler(void)
{	  
	
    
}

void I2C1IntrHandler(void)
{	  
	

    
}


void AutoTestSPIParamHandle(uint8_t* pSPIPara, AutoTest_spi_Params *pAutoTest_SPI_Params)
{


}


void AutoTestSPIHandle(AutoTest_CmdTypeDef *pAutoTest_cmd )
{	
	
}


void AutoTestKeyScanParamHandle(uint8_t* pParam, AutoTest_keyscan_Params *pkeypad_param)
{

	
}


void AutoTestKeyScanHandle(AutoTest_CmdTypeDef *pAutoTest_cmd)
{
	
}

void KeyscanIntrHandler(void)
{   
  
}
unsigned char AutoTestCRC8(LPBYTE pStart, WORD length)
{
    unsigned char fcs = 0xff;

    while (length--)
    {
        fcs = crc8EtsTable[fcs ^ *pStart++];
    }
    return 0xff - fcs;
}

int AutosendInt32(uint32_t ch)
{
	while(1)
	{
		if(UART_GetFlagState(UART, UART_FLAG_THR_TSR_EMPTY)==SET)
		{
			break;
		}
	}
	UART_SendData(UART,(UINT8*)&ch, 4);
	return (ch);
}

int Autosendchar(int ch)
{
	while(1)
	{
		if(UART_GetFlagState(UART, UART_FLAG_THR_TSR_EMPTY)==SET)
		{
			break;
		}
	}
	UART_SendData(UART, (UINT8*)&ch, 1);	
	return (ch);
}

