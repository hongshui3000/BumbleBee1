#if TEST_GDMA_I2S_FW_SIM_WITH_CLARK
   
#include "rtl876x.h"
#include "rtl876x_gdma.h"

void test_GDMA_MemoryToMemory(void)    
{
	GDMA_InitTypeDef GDMA_InitStruct;
    uint8_t i = 0;
	uint8_t GDMA_TestBuffer[100];
    uint8_t GDMA_TestReceiveBuffer[100];
        
     /*test buffer*/
    for(i=0; i<100; i++)
    {   
        GDMA_TestBuffer[i] = i + 1;
    }
    for(i=0; i<100; i++)
    {   
        GDMA_TestReceiveBuffer[i] = 0;
    }
    
	/*GDMA initial*/
    GDMA_InitStruct.GDMA_ChannelNum          = 0;
  	GDMA_InitStruct.GDMA_DIR 			     = GDMA_DIR_MemoryToMemory;
  	GDMA_InitStruct.GDMA_BufferSize 	     = 10;
  	GDMA_InitStruct.GDMA_SourceInc           = DMA_SourceInc_Inc;
	GDMA_InitStruct.GDMA_DestinationInc      = DMA_DestinationInc_Inc;

	GDMA_InitStruct.GDMA_SourceDataSize      = GDMA_DataSize_Word;
	GDMA_InitStruct.GDMA_DestinationDataSize = GDMA_DataSize_Word;
	GDMA_InitStruct.GDMA_SourceMsize	     = GDMA_Msize_8;
	GDMA_InitStruct.GDMA_DestinationMsize    = GDMA_Msize_8;
	
	GDMA_InitStruct.GDMA_SourceAddr 	     = (uint32_t)GDMA_TestBuffer;
	GDMA_InitStruct.GDMA_DestinationAddr     = (uint32_t)GDMA_TestReceiveBuffer;
	GDMA_InitStruct.GDMA_TransferType        = GDMA_TransferType_Other;
	GDMA_InitStruct.GDMA_ChannelPriority     = 0;// it can be 0 to 6 which 6 is highest priority
     
    GDMA_Init(GDMA_Channel0, &GDMA_InitStruct);
    
    GDMA_INTConfig(0, GDMA_INT_Transfer, ENABLE);
   
    /* GDMA IRQ */
    NVIC_ClearPendingIRQ(GDMA0_Channel0_IRQn);
    NVIC_SetPriority(GDMA0_Channel0_IRQn, 1);
    NVIC_EnableIRQ(GDMA0_Channel0_IRQn);
   
    GDMA_Cmd(0, ENABLE);
}



void test_GDMA_I2S_FW_SIM_WITH_CLARK(void)
{
    //LOG_RAW("%s", __FUNCTION__);
    
    test_GDMA_MemoryToMemory();
    
}

#endif // TEST_GDMA_I2S_FW_SIM_WITH_CLARK
