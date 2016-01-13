/**
********************************************************************************************************
Copyright (c) 2015, Realtek Semiconductor Corporation. All rights reserved.
*********************************************************************************************************
* @file 	  diag.h
* @brief	  app log api
* @details	 
*
* @author	lory_xu
* @date 		2015-07-13
* @version	v0.1
*/

#ifndef _DIAG_H_
#define _DIAG_H_

#include <cycle_queue.h>
#include <rtl_types.h>

/** @brief  Debug Log mask */
extern uint32_t ConfigDebug[];

/** @brief Log Module Definition */
typedef enum _MODULE_DEFINE_ 
{
    MODULE_OS			= 0,
    /** BEE hardware modules*/
	MODULE_KEYSCAN		= 1,
	MODULE_QDECODE		= 2,
    MODULE_IR 			= 3,
	MODULE_3DG 			= 4,
	MODULE_ADC 			= 5,
	MODULE_GDMA 		= 6,
	MODULE_I2C 			= 7,
	MODULE_RTC 			= 8,
	MODULE_SPI 			= 9,
	MODULE_TIMER 		= 10,
	MODULE_UART 		= 11,
	MODULE_EFLASH 		= 12,
	MODULE_GPIO      = 13,
	MODULE_PINMUX  = 14,
	MODULE_PWM = 15, 
	
    /** BEE software modules*/
	MODULE_DFU          = 23,
	MODULE_DTM          = 24,
	MODULE_LTP          = 25,
	MODULE_APP          = 26,
	MODULE_PROFILE      = 27,
    MODULE_FRAMEWORK	= 28,
    MODULE_LOWERSTACK	= 29,
    MODULE_UPPERSTACK	= 30,
    MODULE_DRIVERTASK	= 31,
	MODULE_NUMs			= 32
} MODULE_DEFINE;

/** @brief Log Level Definition */
typedef enum _LEVEL_DEFINE_ 
{
	LEVEL_ERROR			= 0,
	LEVEL_WARN			= 1,
	LEVEL_INFO			= 2,
	LEVEL_TRACE			= 3,

	LEVEL_NUMs			= 4
} LEVEL_DEFINE;

#define MODULE_START_INDEX 220
/**
 * @brief Debug Log Mask Set.
 * @param  config[4] --  Print log if bit MODULE_DEFINE of config[LEVEL_DEFINE] is 1;
 * @return  void.
*/
void set_debug_mask(uint32_t config[]);

/** @brief LOG_RAW is reserved, use DBG_DIRECT macro instead */
uint32_t LOG_RAW(IN  char *fmt, ...);

/** @brief LOG_BUFFER is reserved, use DBG_BUFFER macro instead */
uint8_t LOG_BUFFER(uint8_t color, uint8_t module, uint16_t file_num, uint16_t line_num, uint32_t log_str_index, uint8_t index_compressed, uint8_t para_num, ...);

//#define RELEASE_VERSION

#ifdef RELEASE_VERSION

#define DBG_DIRECT(...)
#define DBG_BUFFER(MODULE, LEVEL, pFormat, para_num, ...)

#else

/** 
  * @brief  DBG_DIRECT is used to print log without buffer or PrintTask.
  * DBG_DIRECT is used when:
  *	1. Before PrintTask starts; 
  *	2. In Hard Fault Error ISR;
  *	3. Warning when PrintTask's buffer is full.
  *	...
*/
#define DBG_DIRECT(...)     do {\
				LOG_RAW(__VA_ARGS__);\
			}while(0)
//#define TRACE_DATA        __attribute__((section ("TRACE"))) __attribute__(aligned(4))
//#define TRACE_DATA        __attribute__((section ("TRACE")))
#define SECTION(_name) __attribute__ ((__section__(_name)))

//#define TRACE_DATA SECTION("TRACE") __attribute__((aligned(4)))
#define BIT(_n)   (uint32_t)(1U<<(_n))

/** 
 * @brief  DBG_BUFFER is used to print log with buffer and PrintTask.
 *  PrintTask's priority is the lowest, so Print log will not block any normal task.
*/
#define DBG_BUFFER(MODULE, LEVEL, pFormat, para_num, ...)     do {\
	if ((LEVEL < LEVEL_NUMs) && (MODULE < MODULE_NUMs) && (ConfigDebug[LEVEL] & BIT(MODULE))) {\
	static const char traceFormat[] TRACE_DATA = pFormat;\
	LOG_BUFFER(0, MODULE_START_INDEX+MODULE, __FILE_NUM__, __LINE__, (uint32_t)traceFormat, 1, para_num, ##__VA_ARGS__); \
	}\
}while(0)

#endif


/**
 * @brief Converts a character into hexadecimal num.
 * @param  c -- a character.
 * @return the hexadecimal num.
*/
uint8_t c2hex(uint8_t c);

/**
 * @brief Converts a hexadecimal num into a character.
 * @param  hex -- a hexadecimal num.
 * @return the character.
*/
uint8_t hex2c(uint8_t hex);

/**
 * @brief return the check sum of hex386.
 * @param  buf -- data buffer.
 * @paramsize -- size of data buffer.
 * @return the check sum of hex386.
*/
uint8_t cs_for_hex386(uint8_t *buf, uint32_t size);

#endif //_DIAG_H_
