/**
************************************************************************************************************
*				Copyright(c) 2014, Realtek Semiconductor Corporation. All rights reserved.
************************************************************************************************************
* @file		trace.h
* @brief		Log print and setting interface head file.
* @author	lory_xu
* @date 		2014-05
* @version	v0.1					   
*************************************************************************************************************
*/

#ifndef _TRACE_H_
#define _TRACE_H_

#include "platform_conf.h"
#include "rtl_types.h"
#include "section_config.h"

/** @brief Log Type Definition */
typedef enum tagLOG_Type {
    SUBTYPE_FORMAT                = 0,
    SUBTYPE_STRING                ,
    SUBTYPE_BINARY                ,
    SUBTYPE_CMD_SNOOP             = 16 +0,
    SUBTYPE_EVT_SNOOP             ,
    SUBTYPE_ACL_TX_SNOOP          ,
    SUBTYPE_ACL_RX_SNOOP          ,
    SUBTYPE_SCO_TX_SNOOP          ,
    SUBTYPE_SCO_RX_SNOOP          ,
    SUBTYPE_BDADDR1               = 32 + 0,
    SUBTYPE_BDADDR2               ,
    SUBTYPE_RAMDATA1              ,
    SUBTYPE_RAMDATA2              ,
    SUBTYPE_RAMDATA3              ,
    SUBTYPE_RAMDATA4              ,
    SUBTYPE_RAMDATA5              ,
    SUBTYPE_RAMDATA6              ,
    SUBTYPE_RAMDATA7              ,
    SUBTYPE_RAMDATA8              ,
}tLOG_Type;


/** @brief Log Module Definition */
typedef enum tagMODULE_ID {
    /* BEE hardware modules*/
    MODULE_KEYSCAN                 = 0,
    MODULE_QDECODE                 = 1,
    MODULE_IR                      = 2,
    MODULE_3DG                     = 3,
    MODULE_ADC                     = 4,
    MODULE_GDMA                    = 5,
    MODULE_I2C                     = 6,
    MODULE_RTC                     = 7,
    MODULE_SPI                     = 8,
    MODULE_TIMER                   = 9,
    MODULE_UART                    = 10,
    MODULE_EFLASH                  = 11,
    MODULE_GPIO                    = 12,
    MODULE_PINMUX                  = 13,
    MODULE_PWM                     = 14, 

    /* BEE software modules*/
    MODULE_DFU                     = 54,
    MODULE_DTM                     = 55,
    MODULE_LTP                     = 56,
    MODULE_APP                     = 57,
    MODULE_PROFILE                 = 58,
    MODULE_FRAMEWORK               = 59,
    MODULE_LOWERSTACK              = 60,
    MODULE_UPPERSTACK              = 61,
    MODULE_DRIVERTASK              = 62,
    MODULE_OS                      = 63,

    MODULE_NUMs                    = 64
} tMODULE_ID;

/** @brief Log Level Definition */
typedef enum tagLOG_LEVEL {
    LEVEL_ERROR			= 0,
    LEVEL_WARN			= 1,
    LEVEL_INFO			= 2,
    LEVEL_TRACE			= 3,
    LEVEL_NUMs			= 4
} tLOG_LEVEL;

/** DBG_LEVEL is used to control the log printed by DBG_BUFFER().
 * -1                  : Print None
  * LEVEL_ERROR : Print ERROR
  * LEVEL_WARN  : Print ERROR, WARN
  * LEVEL_INFO    : Print ERROR, WARN, INFO
  * LEVEL_TRACE  : Print ERROR, WARN, INFO, TRACE
  */
#define DBG_LEVEL                   LEVEL_TRACE

#define MODULE_GROUP_SIZE           32
#define MODULE_GROUP_BITs           5
#define MODULE_GROUP_NUM            (MODULE_NUMs + MODULE_GROUP_SIZE -1) >> MODULE_GROUP_BITs


/** @brief  Log Level and Modules Mask */
extern uint32_t gTraceLevelModules[][MODULE_GROUP_NUM];

/**
 * @brief Log Trace and Modules Mask.
 * @param  uint32_t config[LEVEL_NUMs][2];
 * @return  void.
*/
void LogTraceModulesMask(uint32_t levelModules[][MODULE_GROUP_NUM]);

/** @brief LogDirect is reserved, use DBG_DIRECT macro instead */
void LogDirect(IN  char *fmt, ...);

/** @brief LogBufferLowerStack is reserved, use DBG_LOWERSTACK macro instead */
uint8_t LogBufferLowerStack(uint32_t control, uint16_t log_str_index, uint8_t para_num, ...);
/** @brief LogBufferLowerStackData is reserved, use DBG_LOWERSTACKDATA macro instead */
uint8_t LogBufferLowerStackData(uint32_t control, uint16_t log_str_index, uint16_t length, uint8_t* pStr);

/** @brief LogBufferFormat is reserved, use DBG_BUFFER macro instead */
uint8_t LogBufferFormat(uint32_t type_module_mixed, uint32_t log_str_index, uint8_t para_num, ...);
/** @brief LogBufferSNOOP is reserved, use DBG_SNOOP macro instead */
uint8_t LogBufferSNOOP(uint32_t type_module_mixed, uint16_t dataLength, uint8_t *pBinaryData);

const char *traceRamData1(char *pData);
const char *traceRamData2(char *pData);
const char *traceRamData1(char *pData);
const char *traceRamData2(char *pData);
const char *traceRamData3(char *pData);
const char *traceRamData4(char *pData);
const char *traceRamData5(char *pData);
const char *traceRamData6(char *pData);
const char *traceRamData7(char *pData);
const char *traceRamData8(char *pData);

#define TYPE_MODULE_COMBI(type, module)    (uint32_t)((type<<16) | module)
#define GET_TYPE(type_module_combi)        (uint8_t)((type_module_combi>>16) & 0xFF)
#define GET_MODULE(type_module_combi)      (uint8_t)(type_module_combi & 0xFF)

/** 
  * @brief  DBG_DIRECT is used to print log without buffer or PrintTask.
  * DBG_DIRECT is used when:
  *	1. Before PrintTask starts; 
  *	2. In Hard Fault Error ISR;
  *	3. Warning when PrintTask's buffer is full.
  *	...
*/
#define DBG_DIRECT(...)     do {\
        LogDirect(__VA_ARGS__);\
    }while(0)

#define DBG_LOWERSTACK(color, file_num, line_num, log_str_index, para_num, ...)     do {\
	  LogBufferLowerStack((color<<24)|(file_num<<16)|line_num, (uint16_t)log_str_index, para_num, ##__VA_ARGS__); \
    }while(0)

#define DBG_LOWERSTACKDATA(color, file_num, line_num, log_str_index, length, pStr)     do {\
	  LogBufferLowerStackData((color<<24)|(file_num<<16)|line_num, (uint16_t)log_str_index, length, pStr); \
    }while(0)

#define DBG_SNOOP(type, module, level, length, pBinary)    do {\
        if (gTraceLevelModules[level][module>>MODULE_GROUP_BITs] & BIT(module&(MODULE_GROUP_SIZE -1))) {\
            LogBufferSNOOP(TYPE_MODULE_COMBI(type, module), length, pBinary);\
        }\
    }while(0)

/** 
 * @brief  DBG_BUFFER is used to print log. -- For all modules except lower stack
*/
#define DBG_BUFFER(type, module, level, pFormat, para_num,...)     \
        DBG_BUFFER_##level(type, module, pFormat, para_num, ##__VA_ARGS__)

/** 
 * @brief  Don't use DBG_BUFFER_FORMAT_CORE directly, use DBG_BUFFER instead.
*/
#define DBG_BUFFER_FORMAT_CORE(type, module, pFormat, para_num, ...)    do {\
        static const char traceFormat[] TRACE_DATA = pFormat;\
        LogBufferFormat(TYPE_MODULE_COMBI(type, module),  (uint32_t)traceFormat, para_num, ##__VA_ARGS__); \
    }while(0)

/** 
 * @brief  Don't use DBG_BUFFER_LEVEL_XXX directly, use DBG_BUFFER instead.
*/
#if (DBG_LEVEL >= LEVEL_ERROR)
#define DBG_BUFFER_LEVEL_ERROR(type, module, pFormat, para_num, ...)     do {\
        if (gTraceLevelModules[LEVEL_ERROR][module>>MODULE_GROUP_BITs] & BIT(module&(MODULE_GROUP_SIZE -1))) {\
            DBG_BUFFER_FORMAT_CORE(type, module, pFormat, para_num, ##__VA_ARGS__);\
        }\
    }while(0)
#else
#define DBG_BUFFER_LEVEL_ERROR(type, module, pFormat, para_num, ...) 
#endif

#if (DBG_LEVEL >= LEVEL_WARN)
#define DBG_BUFFER_LEVEL_WARN(type, module, pFormat, para_num, ...)     do {\
       if (gTraceLevelModules[LEVEL_WARN][module>>MODULE_GROUP_BITs] & BIT(module&(MODULE_GROUP_SIZE -1))){\
            DBG_BUFFER_FORMAT_CORE(type, module, pFormat, para_num, ##__VA_ARGS__);\
        }\
    }while(0)
#else
#define DBG_BUFFER_LEVEL_WARN
#endif

#if (DBG_LEVEL >= LEVEL_INFO)
#define DBG_BUFFER_LEVEL_INFO(type, module, pFormat, para_num, ...)     do {\
        if (gTraceLevelModules[LEVEL_INFO][module>>MODULE_GROUP_BITs] & BIT(module&(MODULE_GROUP_SIZE -1))) {\
            DBG_BUFFER_FORMAT_CORE(type, module, pFormat, para_num, ##__VA_ARGS__);\
        }\
    }while(0)
#else
#define DBG_BUFFER_LEVEL_INFO
#endif

#if (DBG_LEVEL >= LEVEL_TRACE)
#define DBG_BUFFER_LEVEL_TRACE(type, module, pFormat, para_num, ...)     do {\
        if (gTraceLevelModules[LEVEL_TRACE][module>>MODULE_GROUP_BITs] & BIT(module&(MODULE_GROUP_SIZE -1))) {\
            DBG_BUFFER_FORMAT_CORE(type, module, pFormat, para_num, ##__VA_ARGS__);\
        }\
    }while(0)
#else
#define DBG_BUFFER_LEVEL_TRACE
#endif


/** 
 * @brief  LOG Interface for FW Sim.
*/
#if DBG_FWSIM_LOG    
    void DBG_FWsim_Log(char* pMsg)
    {
        if(!pMsg)
            return;
        
        while(*pMsg)
        {
            char c = *pMsg;            
            HAL_WRITE8(FAKE_UART_ADDRESS, 0, c);
            ++pMsg;        
        }
    }
#else
    #define DBG_FWsim_Log(...)
#endif

#endif //_TRACE_H_
