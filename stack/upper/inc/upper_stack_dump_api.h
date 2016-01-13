#ifndef _UPPER_STACK_DUMP_API_H_
#define _UPPER_STACK_DUMP_API_H_

#include <stdint.h>
#include <rtl_types.h>

//void blueAPI_dump_messageOffset(void);
void blueAPI_dump_ProtocolData(void);
void blueAPI_dump_all_pools(void);
//void blueAPI_dump_memory(void);
//void blueAPI_dump_BlueAPIDsMessageLength(void);
//void blueAPI_dump_BlueAPIUsMessageLength(void);
//uint16_t blueAPI_dump_DataRamOff_Module(void);
//uint16_t blueAPI_dump_DataRamOn_Module(void);
uint16_t blueAPI_dump_BufRamOff_Module(void);
//void blueAPI_dump_gatt_queue(void);
#endif

