/**
*********************************************************************************************************
*               Copyright(c) 2015, Realtek Semiconductor Corporation. All rights reserved.
**********************************************************************************************************
* @file     auto_test.h
* @brief    auto test header file.
* @details  none.
* @author   yue
* @date     2015-06-03
* @version  v0.1
*********************************************************************************************************
*/
#ifndef _AUTO_TEST_H_
#define _AUTO_TEST_H_

#include "trace.h"
#include "FreeRTOS.h"
#include "task.h"
#include <queue.h>
#include "rtl876x_rcc.h"
#include "rtl876x_pinmux.h"
#include "rtl876x_nvic.h"
#include "rtl876x_uart.h"
#include "rtl876x.h"
#include "rtl_endian.h"
#include "rtl876x_gpio.h"
#include "rtl876x_tim.h"
#include "rtl876x_pwm.h"
#include "rtl876x_i2c.h"
#include "rtl876x_spi.h"
#include "rtl876x_keyscan.h"


#define  FAR
typedef        unsigned char    UCHAR,* PUCHAR, * PBYTE;
typedef        unsigned short   WORD, USHORT, * PUSHORT, * PWORD;
typedef        unsigned char    FAR * LPBYTE;


#define AUTOTEST_DBG_EN 			0x01

/*uart parameter section*/
typedef struct
{	
	UINT8 					OVSR;
	UINT16 					DIV;
	UINT16 					OVSR_ADJ;
	
}AutoTest_uart_Params;

/*timer parameter section*/
typedef struct
{	
	UINT8 					TimerId;
	TIM_TimeBaseInitTypeDef TIM_InitStruct;
}AutoTest_Tim_Params;

/*pwm parameter section*/
typedef struct
{	
	UINT8 					pwm_selection;
	uint16_t 				TIM_ClockSrc;
	uint32_t 				TIM_Period;
	uint8_t 				Pin_Num;
	PWM_InitTypeDef			PWM_InitStruct;
}AutoTest_pwm_Params;

/*i2c parameter section*/
typedef struct
{	
	UINT8 					i2c_index;
	uint8_t                 i2c_scl_pin_num;
	uint8_t                 i2c_sda_pin_num;
	I2C_InitTypeDef  		I2C_InitStructure;
}AutoTest_i2c_Params;

/*spi parameter section*/
typedef struct
{	
	UINT8 					spi_index;
	uint8_t                	spi_clk_pin_num;
	uint8_t                	spi_mosi_pin_num;
	uint8_t                	spi_miso_pin_num;
	uint8_t                	spi_cs0_pin_num;
	SPI_InitTypeDef  		SPI_InitStructure;
}AutoTest_spi_Params;

/*keyscan parameter section*/
typedef struct
{	
	KEYSCAN_InitTypeDef 	keyScanInitStruct;
	uint8_t                	keyscan_column1_pin_num;
	uint8_t                	keyscan_column2_pin_num;
	uint8_t                	keyscan_column3_pin_num;
	uint8_t                	keyscan_column4_pin_num;
	uint8_t                	keyscan_row1_pin_num;
	uint8_t                	keyscan_row2_pin_num;
	uint8_t                	keyscan_row3_pin_num;
	uint8_t                	keyscan_row4_pin_num;
	
}AutoTest_keyscan_Params;


/*GPIO data section*/
typedef union
{	
	UCHAR					GPIO_voltage_level;
	uint32_t				port_value;
}AutoTest_GPIO_DataType;

/*I2C write data*/
typedef struct
{
 	UCHAR					i2c_write_buffer[256];
	UCHAR					i2c_data_length;
	UCHAR					ReadDataCount;
	
}AutoTest_I2C_DataType;

/*SPI data parameter*/
typedef struct
{
 	UINT8					spi_write_buffer[256];
	UINT8					spi_data_length;
	UINT8				    spi_read_length;
}AutoTest_SPI_DataType;

typedef struct
{
    uint32_t length;
    struct
    {
        uint8_t column: 5;   /**< keyscan column buffer data   */
        uint8_t row: 3;      /**< keyscan raw buffer data      */
    } key[26];
}KEYSCAN_DATA_STRUCT, *PKEYSCAN_DATA_STRUCT;


typedef enum 
{ 
 	WaitCmd, 
  	WaitCopmask, 
  	WaitLp_h, 
  	WaitLp_l,
  	WaitP1,
  	WaitP2_h,
  	WaitP2_l,
  	WaitP3,
  	WaitCRC,
  	WaitPayload
}WaitState;

typedef struct _AutoTest_PacketTypeDef
{
	uint8_t 	LoopQueue[256];
	uint8_t 	DecoderData[256];
	uint16_t 	Decoderlength;				/**< length of decoder payload*/
	uint16_t 	DecoderIndex;
	uint16_t 	QueueCapacity;				/**<equal length of LoopQueue*/
	uint16_t 	QueueSize;
	uint16_t 	ReadIndex;					/**< index of read queue */
	uint16_t 	WriteIndex; 				/**< index of write queue */
	uint16_t 	PacketLength;
	WaitState  	CollectorState;
}AutoTest_PacketTypeDef;

/* Init and Deinit test */
typedef enum
{
	AutoTest_PeripherialType_PWM,              /**< Peripheral function 0: PWM */
	AutoTest_PeripherialType_Timer,            /**< Peripheral function 1: Timer*/
	AutoTest_PeripherialType_GPIO,             /**< Peripheral function 2: GPIO */
	AutoTest_PeripherialType_DATA_UART,        /**< Peripheral function 3: DATA UART */
    AutoTest_PeripherialType_I2C0,             /**< Peripheral function 4: I2C0 */
    AutoTest_PeripherialType_I2C1,             /**< Peripheral function 5: I2C1 */
    AutoTest_PeripherialType_SPI0,     		   /**< Peripheral function 6: SPI0 */   
    AutoTest_PeripherialType_SPI1,     		   /**< Peripheral function 7: SPI1 */
    AutoTest_PeripherialType_IR,               /**< Peripheral function 8: IR */    
    AutoTest_PeripherialType_SPI_2WIRE,        /**< Peripheral function 9: SPI 2 wire */  
    AutoTest_PeripherialType_Keyscan,        /**< Peripheral function 10: Keyscan*/ 
} InitOrDeinitPeripherialType;



/* AutoTest command */
typedef struct	_AutoTest_CmdTypeDef 
{
	/*header*/
	struct
	{
		UCHAR 					 cmd ;
		UCHAR 					 copmsk ;
		UCHAR 					 lp_h ;
		UCHAR 					 lp_l;
		UCHAR 					 P1;
		UCHAR 					 P2_h;
		UCHAR 					 P2_l;
		UCHAR 					 P3;
	}HeaderTypeDef;
	/* IO module initialization parameter */
	

	union
	{	
	
		AutoTest_pwm_Params 	 pwm_params ;
		AutoTest_uart_Params 	 uart_params;
		GPIO_InitTypeDef         GPIO_params;
		AutoTest_Tim_Params		 TIM_params;
		AutoTest_i2c_Params  	 I2C_Params;
		AutoTest_spi_Params  	 SPI_Params;
		AutoTest_keyscan_Params  keypad_param;
		
	}InitTypeDef;
	/* test data section */
	union
	{
		AutoTest_GPIO_DataType   AutoTest_GPIO_Data;
		AutoTest_I2C_DataType 	 AutoTest_I2C_Data;
		AutoTest_SPI_DataType 	 AutoTest_SPI_Data;
		//KEYSCAN_BUFFER_STRUCT    AutoTest_Key_Buffer;
	}DataTypeDef;
	
}AutoTest_CmdTypeDef;




/* export functions */
extern BaseType_t AutoTestInit(void);

/*----------------------------------------------------------------------------
 * FCS lookup table.
 * generator polynomial: x**8 + x**2 + x + 1
 * -------------------------------------------------------------------------*/

STATIC const BYTE FAR crc8EtsTable[256] =
{
    0x00, 0x91, 0xE3, 0x72, 0x07, 0x96, 0xE4, 0x75,
    0x0E, 0x9F, 0xED, 0x7C, 0x09, 0x98, 0xEA, 0x7B,
    0x1C, 0x8D, 0xFF, 0x6E, 0x1B, 0x8A, 0xF8, 0x69,
    0x12, 0x83, 0xF1, 0x60, 0x15, 0x84, 0xF6, 0x67,
    0x38, 0xA9, 0xDB, 0x4A, 0x3F, 0xAE, 0xDC, 0x4D,
    0x36, 0xA7, 0xD5, 0x44, 0x31, 0xA0, 0xD2, 0x43,
    0x24, 0xB5, 0xC7, 0x56, 0x23, 0xB2, 0xC0, 0x51,
    0x2A, 0xBB, 0xC9, 0x58, 0x2D, 0xBC, 0xCE, 0x5F,
    0x70, 0xE1, 0x93, 0x02, 0x77, 0xE6, 0x94, 0x05,
    0x7E, 0xEF, 0x9D, 0x0C, 0x79, 0xE8, 0x9A, 0x0B,
    0x6C, 0xFD, 0x8F, 0x1E, 0x6B, 0xFA, 0x88, 0x19,
    0x62, 0xF3, 0x81, 0x10, 0x65, 0xF4, 0x86, 0x17,
    0x48, 0xD9, 0xAB, 0x3A, 0x4F, 0xDE, 0xAC, 0x3D,
    0x46, 0xD7, 0xA5, 0x34, 0x41, 0xD0, 0xA2, 0x33,
    0x54, 0xC5, 0xB7, 0x26, 0x53, 0xC2, 0xB0, 0x21,
    0x5A, 0xCB, 0xB9, 0x28, 0x5D, 0xCC, 0xBE, 0x2F,
    0xE0, 0x71, 0x03, 0x92, 0xE7, 0x76, 0x04, 0x95,
    0xEE, 0x7F, 0x0D, 0x9C, 0xE9, 0x78, 0x0A, 0x9B,
    0xFC, 0x6D, 0x1F, 0x8E, 0xFB, 0x6A, 0x18, 0x89,
    0xF2, 0x63, 0x11, 0x80, 0xF5, 0x64, 0x16, 0x87,
    0xD8, 0x49, 0x3B, 0xAA, 0xDF, 0x4E, 0x3C, 0xAD,
    0xD6, 0x47, 0x35, 0xA4, 0xD1, 0x40, 0x32, 0xA3,
    0xC4, 0x55, 0x27, 0xB6, 0xC3, 0x52, 0x20, 0xB1,
    0xCA, 0x5B, 0x29, 0xB8, 0xCD, 0x5C, 0x2E, 0xBF,
    0x90, 0x01, 0x73, 0xE2, 0x97, 0x06, 0x74, 0xE5,
    0x9E, 0x0F, 0x7D, 0xEC, 0x99, 0x08, 0x7A, 0xEB,
    0x8C, 0x1D, 0x6F, 0xFE, 0x8B, 0x1A, 0x68, 0xF9,
    0x82, 0x13, 0x61, 0xF0, 0x85, 0x14, 0x66, 0xF7,
    0xA8, 0x39, 0x4B, 0xDA, 0xAF, 0x3E, 0x4C, 0xDD,
    0xA6, 0x37, 0x45, 0xD4, 0xA1, 0x30, 0x42, 0xD3,
    0xB4, 0x25, 0x57, 0xC6, 0xB3, 0x22, 0x50, 0xC1,
    0xBA, 0x2B, 0x59, 0xC8, 0xBD, 0x2C, 0x5E, 0xCF
};
#endif
