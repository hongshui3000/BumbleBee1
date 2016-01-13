/**
********************************************************************************************************
Copyright (c) 2015, Realtek Semiconductor Corporation. All rights reserved.
*********************************************************************************************************
* @file 	  cycle_queue.h
* @brief	  Log cycle buffer implementation head file.
* @details	 
*
* @author	lory_xu
* @date 		2015-07-13
* @version	v0.1
*/

#ifndef _CYCLE_QUEUE_H_
#define _CYCLE_QUEUE_H_

#include <blueapi_types.h>

#ifdef __cplusplus
extern "C" {
#endif

/** @brief  Log Buffer */
extern uint8_t cyc_buffer[];

/** @brief  Read pointer of Log Buffer */
extern volatile uint16_t pRead;

/** @brief  Write pointer of Log Buffer */
extern volatile uint16_t pWrite;

/**
 * @brief Check log buffer is empty or not.
 * @param  void.
 * @return  Status of log buffer.
 * @retval TURE if empty, FAILE if not empty.
*/
uint8_t IsCycQueueEmpty(void);

/**
 * @brief Check log buffer is full or not.
 * @param  void.
 * @return  Status of log buffer.
 * @retval TURE if full, FAILE if not full.
*/
uint8_t IsCycQueueFull(void);
 
/**
 * @brief Return the number of data in log buffer.
 * @param  void.
 * @return  the number of data in log buffer.
*/
uint16_t CycQueueSize(void);

/** 
 * @brief  Return the available buffer size in log buffer.
 * @param  void.
 * @return  the available buffer size in log buffer.
*/
uint16_t CycQueueRemainSize(void);

/**
 * @brief Put data in log buffer.
 * @param  pWriteBuf -- Start address of write buffer.
 * @param  length -- Length of write buffer.
 * @return  the status of operation.
 * @retval TURE if success, FAILE if fail.
*/
uint8_t CycQueueWrite(uint8_t *pWriteBuf, uint16_t length);
void UpdateQueueRead(uint16_t SendSize);
/**
 * @brief Retrieve data in log buffer and then print them.
 * @param  AvailableSize -- The max number of data can be print. (The value is determined by Log uart buffer)
 * @return  The actual number of print data.
*/
uint16_t CycQueueReadandPrint(uint8_t AvailableSize);

/**
 * @brief Retrieve all data in log buffer and print them when exception happens like hard fault.
             This funcion is usually used for debugging.
 * @param  void.
 * @return  VOID.
*/
void CycQueueDumpWhenException(void);

#ifdef __cplusplus
}
#endif
  
#endif
