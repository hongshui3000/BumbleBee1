#include "rtl876x_gdma.h"
//#include "rtl876x_nvic.h"
#include "trace.h"

uint8_t GDMA_SendBuffer[256];
uint8_t GDMA_RecvBuffer[256];

#define GDMA0_CHANNEL0_IRQ 20

#define LOG_UART_GDMA 1

void UARTSendDataByGDMA(void)
{
    uint16_t i = 0;
    GDMA_InitTypeDef GDMA_InitStruct;    
    
    /*--------------initialize test buffer which for sending data to UART---------------------*/
    for(i=0; i<256; i++)
    {   
        GDMA_SendBuffer[i] = i;
    }
    //DBG_DIRECT("111222333444555");
//#if LOG_UART_GDMA    
    /*--------------GDMA init-----------------------------*/
    GDMA_StructInit(&GDMA_InitStruct);      
	GDMA_InitStruct.GDMA_ChannelNum      = 0;
  	GDMA_InitStruct.GDMA_DIR 			 = GDMA_DIR_MemoryToPeripheral;
  	GDMA_InitStruct.GDMA_BufferSize 	 = 256;//determine total transfer size
  	GDMA_InitStruct.GDMA_SourceInc       = DMA_SourceInc_Inc;
	GDMA_InitStruct.GDMA_DestinationInc  = DMA_DestinationInc_Fix;
    GDMA_InitStruct.GDMA_SourceDataSize  = GDMA_DataSize_Byte;
	GDMA_InitStruct.GDMA_DestinationDataSize = GDMA_DataSize_Byte;
	GDMA_InitStruct.GDMA_SourceMsize	  = GDMA_Msize_1;
	GDMA_InitStruct.GDMA_DestinationMsize = GDMA_Msize_1;   
	GDMA_InitStruct.GDMA_SourceAddr 	 = (uint32_t)GDMA_SendBuffer;
	GDMA_InitStruct.GDMA_DestinationAddr = (uint32_t)(&(LOG_UART->RB_THR_DLL));
	GDMA_InitStruct.GDMA_TransferType    = 15;   //GDMA_TransferType_UART0_TX;
	GDMA_InitStruct.GDMA_ChannelPriority = 3;//channel prority between 0 to 6
    
    GDMA_Init(GDMA_Channel0, &GDMA_InitStruct);
    
    /* Config destination handshaking interface */
	GDMA_Channel0->CFG_HIGH |= (GDMA_InitStruct.GDMA_TransferType) << 11;

//#elif DATA_UART_GDMA  
//    GDMA_StructInit(&GDMA_InitStruct);      
//	GDMA_InitStruct.GDMA_ChannelNum      = 0;
//  	GDMA_InitStruct.GDMA_DIR 			 = GDMA_DIR_MemoryToPeripheral;
//  	GDMA_InitStruct.GDMA_BufferSize 	 = 256;//determine total transfer size
//  	GDMA_InitStruct.GDMA_SourceInc       = DMA_SourceInc_Inc;
//	GDMA_InitStruct.GDMA_DestinationInc  = DMA_DestinationInc_Fix;
//    GDMA_InitStruct.GDMA_SourceDataSize  = GDMA_DataSize_Byte;
//	GDMA_InitStruct.GDMA_DestinationDataSize = GDMA_DataSize_Byte;
//	GDMA_InitStruct.GDMA_SourceMsize	  = GDMA_Msize_1;
//	GDMA_InitStruct.GDMA_DestinationMsize = GDMA_Msize_1;   
//	GDMA_InitStruct.GDMA_SourceAddr 	 = (uint32_t)GDMA_SendBuffer;
//	GDMA_InitStruct.GDMA_DestinationAddr = (uint32_t)(&(UART->RB_THR));
//	GDMA_InitStruct.GDMA_TransferType    = 0;   //GDMA_TransferType_UART0_TX;
//	GDMA_InitStruct.GDMA_ChannelPriority = 3;//channel prority between 0 to 6
//    
//    GDMA_Init(GDMA_Channel0, &GDMA_InitStruct);

//#elif MEM_TO_MEM_GDMA

//    GDMA_StructInit(&GDMA_InitStruct);      
//	GDMA_InitStruct.GDMA_ChannelNum      = 0;
//  	GDMA_InitStruct.GDMA_DIR 			 = GDMA_DIR_MemoryToMemory;
//  	GDMA_InitStruct.GDMA_BufferSize 	 = 256;//determine total transfer size
//  	GDMA_InitStruct.GDMA_SourceInc       = DMA_SourceInc_Inc;
//	GDMA_InitStruct.GDMA_DestinationInc  = DMA_DestinationInc_Inc;
//    GDMA_InitStruct.GDMA_SourceDataSize  = GDMA_DataSize_Byte;
//	GDMA_InitStruct.GDMA_DestinationDataSize = GDMA_DataSize_Byte;
//	GDMA_InitStruct.GDMA_SourceMsize	  = GDMA_Msize_1;
//	GDMA_InitStruct.GDMA_DestinationMsize = GDMA_Msize_1;   
//	GDMA_InitStruct.GDMA_SourceAddr 	 = (uint32_t)GDMA_SendBuffer;
//	GDMA_InitStruct.GDMA_DestinationAddr = (uint32_t)GDMA_RecvBuffer;
//	GDMA_InitStruct.GDMA_TransferType    = 15;   //GDMA_TransferType_UART0_TX;
//	GDMA_InitStruct.GDMA_ChannelPriority = 3;//channel prority between 0 to 6
//    
//    GDMA_Init(GDMA_Channel0, &GDMA_InitStruct);
//    
//    /* Config destination handshaking interface */
//	//GDMA_Channel0->CFG_HIGH |= (GDMA_InitStruct.GDMA_TransferType) << 11;

//#endif
    
    /*-----------------GDMA IRQ init-------------------*/
//    NVIC_InitTypeDef NVIC_InitStruct;
//	NVIC_InitStruct.NVIC_IRQChannel = 20; //GDMA0_CHANNEL0_IRQ;
//	NVIC_InitStruct.NVIC_IRQChannelPriority = 1;
//	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
//	NVIC_Init(&NVIC_InitStruct);
    
    
    /*  Enable GDMA IRQ  */
    NVIC_ClearPendingIRQ((IRQn_Type)GDMA0_CHANNEL0_IRQ);
    NVIC_SetPriority((IRQn_Type)GDMA0_CHANNEL0_IRQ, 1);
    NVIC_EnableIRQ((IRQn_Type)GDMA0_CHANNEL0_IRQ);
    
    
    GDMA_INTConfig(0, GDMA_INT_Transfer,ENABLE);
    
    /*-----------------start to send data-----------*/
    GDMA_Cmd(0, ENABLE);
    
}

void GDMA0_Channel0_Handler(void)
{
    while(1);
}


