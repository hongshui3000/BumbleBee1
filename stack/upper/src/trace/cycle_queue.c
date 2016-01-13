/**
********************************************************************************************************
Copyright (c) 2015, Realtek Semiconductor Corporation. All rights reserved.
*********************************************************************************************************
* @file 	  cycle_queue.c
* @brief	  Cycle Queue functions Implementation.
* @details	 
*
* @author	lory_xu
* @date 		2015-07-13
* @version	v0.1
*/

enum { __FILE_NUM__= 0 };

#include <os_intr.h>
#include <cycle_queue.h>
#include <diag.h>

#define MIN(a,b)                        (((a) < (b))? (a):(b))

#define MAX_BUFFER_SIZE 2048

/** @detailed  cyc_buffer stores all the printing information which will be print in the PrintTask */
uint8_t cyc_buffer[MAX_BUFFER_SIZE];
volatile uint16_t pRead = 0;
volatile uint16_t pWrite = 0;

/**
* @brief  check if write queue empty
* 
* @param 
*
* @return  
*/
uint8_t IsCycQueueEmpty()
{
	return pRead == pWrite;
}

/**
* @brief  check if write queue full
* 
* @param 
*
* @return	
*/
uint8_t IsCycQueueFull()
{
	return ((pWrite + 1)&(MAX_BUFFER_SIZE - 1)) == pRead;
}

/**
* @brief  get data size in cycle queue
* 
* @param 
*
* @return	 
*/
uint16_t CycQueueSize()
{
	return (pWrite - pRead + MAX_BUFFER_SIZE)&(MAX_BUFFER_SIZE - 1);
}

/**
* @brief  remain cycle queue size
* 
* @param 
*
* @return	 
*/
uint16_t CycQueueRemainSize()
{
	return (MAX_BUFFER_SIZE - pWrite + pRead - 1)&(MAX_BUFFER_SIZE - 1);
}

/**
* @brief  write data to cycle queue
* 
* @param pWriteBuf: data buf
* @param length: data length
*
* @return	 
*/
uint8_t CycQueueWrite(uint8_t *pWriteBuf, uint16_t length)
{
	uint8_t ret = TRUE;
	int  s;
	static uint8_t isBufferFull = FALSE;

	if(cyc_buffer == NULL)
	{
		return FALSE;
	}

	s = osInterruptDisable();
	
	if(CycQueueRemainSize() >= length)
	{
	    isBufferFull = FALSE;
	    if( (pWrite + length) <= MAX_BUFFER_SIZE )
	    {
	        memcpy(cyc_buffer + pWrite, pWriteBuf, length);
	        pWrite = (pWrite + length)&(MAX_BUFFER_SIZE - 1);
	    }
	    else
	    {
	        uint16_t part_length = MAX_BUFFER_SIZE - pWrite;
	        memcpy(cyc_buffer + pWrite, pWriteBuf, part_length);
	        memcpy(cyc_buffer, (pWriteBuf + part_length), (length - part_length));
	        pWrite = length - part_length;
	    }
	}
	else
	{
	    if(isBufferFull == FALSE)
	    {
	        isBufferFull = TRUE;
	    }
	    ret = FALSE;
	}
	osInterruptEnable(s);

	return ret;
}

/**
* @brief  upate read ponit of cycle queue
* 
* @param SendSize
*
* @return	 
*/
void UpdateQueueRead(uint16_t SendSize)
{
	int s = osInterruptDisable();
	pRead = (pRead + SendSize) &  (MAX_BUFFER_SIZE - 1); 
	osInterruptEnable(s);
}

