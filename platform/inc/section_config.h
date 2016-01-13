/**
************************************************************************************************************
*               Copyright(c) 2014, Realtek Semiconductor Corporation. All rights reserved.
************************************************************************************************************
* @file     section_config.h
* @brief    Memory section define.
* @author   lory_xu
* @date     2014-05
* @version  v0.1
*************************************************************************************************************
*/

#ifndef _SECTION_CONFIG_H_
#define _SECTION_CONFIG_H_

#include "rtl_types.h"

/***************************************  TRACE  *****************************************/
#define TRACE_DATA SECTION(".TRACE") __attribute__((aligned(4)))

/***************************************  Ram   *****************************************/
/** @brief  Put the .data symbol in data off memory */
#define SRAM_OFF_BD_DATA_SECTION SECTION(".bdsram.off.data")
/** @brief  Put the .bss symbol in data off memory */
#define SRAM_OFF_BD_BSS_SECTION SECTION_ZI(".bdsram.off.bss")
#define SRAM_OFF_BD_HEAP_SECTION SECTION_ZI(".bdsram.off.heap")
/** @brief  Default put the .data symbol in data on memory */
#define SRAM_ON_BD_DATA_SECTION
/** @brief  Default put the .bss symbol in data on memory */
#define SRAM_ON_BD_BSS_SECTION
#define SRAM_ON_BD_HEAP_SECTION SECTION_ZI(".bdsram.on.heap")
#define SRAM_OFF_BF_BSS_SECTION SECTION_ZI(".bfsram.off.bss")
#define SRAM_ON_BF_BSS_SECTION SECTION_ZI(".bfsram.on.bss")
#define PATCH_POINTER_SECTION
#define ON_RUNNING_STACK SECTION_ZI(".on.running.stack")

#endif //_SECTION_CONFIG_H_
