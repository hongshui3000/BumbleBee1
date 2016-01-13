/**
********************************************************************************************************
Copyright (c) 2015, Realtek Semiconductor Corporation. All rights reserved.
*********************************************************************************************************
* @file       trace_binary.c
* @brief     Trace target (STM32)
* @details   
*
* @author   gordon
* @date      2015-07-13
* @version	v0.1
*/

#include <flags.h>
#include <rtl_types.h>
#include <trace_binary.h>
#include <stextif.h>
#include <os_mem.h>
#include <os_intr.h>

extern uint8_t gSequenceNumber;

#ifndef MAX_LOG_MESSAGE_LEN
#define MAX_LOG_MESSAGE_LEN 128
#endif

/**
* @brief  trace printf
*
* @param  Mid
* @param  pFormat
* @param  Count
*
* @return 
*/
void tracePrintf(uint8_t Mid, uint32_t level, const char *pFormat, int Count, ...)
{

    uint32_t      Arg;
    VA_LIST    pVaList;
    uint8_t  pTraceData[MAX_LOG_MESSAGE_LEN];
    uint8_t *     pData ;
    uint16_t       Length;
    uint8_t   Type;
    int i ;
    uint32_t      TimeStamp = TRACE_TIMESTAMP();
    uint8_t       Checksum;
    int         s;

    if (Count > 8)
    {
        DebuggerBreak();
        return;
    }

    Length = TRACE_HEADER_LENGTH + (2 + (Count * 4));
    if (Length > MAX_LOG_MESSAGE_LEN)
    {
        DebuggerBreak();
        return;
    }
    Type         = TRACE_TYPE_FORMAT;

	pData = pTraceData;
	pTraceData[0] = TRACE_SYNC_CODE;
	pTraceData[1] = Length;
    pTraceData[4] = (uint8_t)((TimeStamp) & 0xff);
    pTraceData[5] = (uint8_t)(((TimeStamp) >> 8) & 0xff);
    pTraceData[6] = Type;

    VA_START(pVaList, Count);

    pData   = &pTraceData[TRACE_HEADER_LENGTH];
    pTraceData[7] = (uint8_t)(((uint32_t)(pFormat) >> 2) & 0xff);
    pTraceData[8] = (uint8_t)(((uint32_t)(pFormat) >> 10) & 0xff);

    pData += 2;
    while (Count--)
    {
        Arg = VA_ARG(pVaList, uint32_t);
        LE_DWORD2EXTRN(pData, Arg);
        pData += 4;
    }
    VA_END(pVaList);

    s = osInterruptDisable();

	gSequenceNumber++;
	pTraceData[2] = gSequenceNumber;		
	pData = pTraceData;

    Checksum = *pData++;
    for (i = 1; i < TRACE_CHECKSUM_DATA_LENGTH; i++)
    {
        Checksum ^= *pData++;
    }
    pTraceData[3] = Checksum;
	
	stTraceOutput(pTraceData, Length);
    osInterruptEnable(s);
}

/**
* @brief  trace binary
*
* @param  Mid
* @param  pkt_type
* @param  DataLength
* @param  pBinaryData
*
* @return 
*/
void traceBinary(uint8_t Mid, uint8_t pkt_type, uint16_t DataLength, uint8_t *pBinaryData)
{
    uint8_t  pTraceData[MAX_LOG_MESSAGE_LEN];
    uint8_t *     pData ;
    uint16_t       Length;
    uint8_t   Type;
    int i ;
    uint32_t      TimeStamp = TRACE_TIMESTAMP();
    uint8_t       Checksum;
    int         s;
    Length = TRACE_HEADER_LENGTH + (1 + DataLength);
    if (Length > MAX_LOG_MESSAGE_LEN)
    {
		Length = MAX_LOG_MESSAGE_LEN;
		DataLength = Length - TRACE_HEADER_LENGTH - 1;
    }
    Type         = TRACE_TYPE_BINARY;

    pData = pTraceData;

    pTraceData[0] = TRACE_SYNC_CODE;
    pTraceData[1] = Length;
    pTraceData[4] = (uint8_t)((TimeStamp) & 0xff);
    pTraceData[5] = (uint8_t)(((TimeStamp) >> 8) & 0xff);
    pTraceData[6] = Type;

    pData    = &pTraceData[TRACE_HEADER_LENGTH];
    *pData++ = pkt_type;
    memcpy((PVOID)pData, pBinaryData, DataLength);
    s = osInterruptDisable();
	gSequenceNumber++;
	pTraceData[2] = gSequenceNumber;
	pData = pTraceData;
    Checksum = *pData++;
    for (i = 1; i < TRACE_CHECKSUM_DATA_LENGTH; i++)
    {
        Checksum ^= *pData++;
    }
    pTraceData[3] = Checksum;

	stTraceOutput(pTraceData, Length);
    osInterruptEnable(s);
}
/**
* @brief  trace out string
* 
* @param  mid
* @param  DataLength: string length
* @param  pString: string
* @return  
*
*/ 
void traceString(uint8_t Mid, uint8_t DataLength, char *pString)
{
	/*nothing*/
}

#if 0
/**
* @brief  trace bdaddr
*
* @param  Mid
* @param  pBdAddr
* @param  Type
*
* @return 
*/
static const char *traceBdAddr(uint8_t Mid, uint32_t level, const char *pBdAddr, uint8_t Type)
{
    uint8_t  pTraceData[MAX_LOG_MESSAGE_LEN];
    uint8_t *     pData ;
    uint16_t       Length;
    int i ;
    uint32_t      Data;
    uint32_t      TimeStamp = TRACE_TIMESTAMP();
    uint8_t       Checksum;
    int         s;

    Length = TRACE_HEADER_LENGTH + (2 * 4);
    if (Length > MAX_LOG_MESSAGE_LEN)
    {
        DebuggerBreak();
        return ((const char *)((0xBDBD << 8) | Type));
    }

    pData = pTraceData;

    pTraceData[0] = TRACE_SYNC_CODE;
    pTraceData[1] = Length;
    pTraceData[4] = (uint8_t)((TimeStamp) & 0xff);
    pTraceData[5] = (uint8_t)(((TimeStamp) >> 8) & 0xff);

    pTraceData[6] = Type;

    pData = &pTraceData[TRACE_HEADER_LENGTH];
    if (pBdAddr != (const char *)0)
    {
        Data = ((pBdAddr[5] << 24) + (pBdAddr[4] << 16) + (pBdAddr[3] << 8) + pBdAddr[2]);
        LE_DWORD2EXTRN(pData, Data);
        pData += 4;
        Data = ((pBdAddr[1] << 8) + pBdAddr[0]);
    }
    else
    {
        Data = 0;
        LE_DWORD2EXTRN(pData, Data);
        pData += 4;
    }
    LE_DWORD2EXTRN(pData, Data);
    pData += 4;
    s = osInterruptDisable();

	gSequenceNumber++;
	pTraceData[2] = gSequenceNumber;
	pData = pTraceData;

    Checksum = *pData++;
    for (i = 1; i < TRACE_CHECKSUM_DATA_LENGTH; i++)
    {
        Checksum ^= *pData++;
    }
    pTraceData[3] = Checksum;

	stTraceOutput(pTraceData, Length);
    osInterruptEnable(s);

    return ((const char *)((0xBDBD << 8) | Type));

}

/**
* @brief  trace bdaddr1
*
* @param  Mid
* @param  pBdAddr
*
* @return 
*/
const char *traceBdAddr1(uint8_t Mid, uint32_t level, const char *pBdAddr)
{
    return (traceBdAddr(Mid, level, pBdAddr, TRACE_TYPE_BDADDR1));
}

/**
* @brief  trace bdaddr2
*
* @param  Mid
* @param  pBdAddr
*
* @return 
*/
const char *traceBdAddr2(uint8_t Mid, uint32_t level, const char *pBdAddr)
{
    return (traceBdAddr(Mid, level, pBdAddr, TRACE_TYPE_BDADDR2));
}

/**
* @brief  trace ram data
*
* @param  Mid
* @param  pData
* @param  Type
*
* @return 
*/
const char *traceRamData(uint8_t Mid, uint32_t level, char *pData, uint8_t Type)
{

    uint8_t  pTraceData[MAX_LOG_MESSAGE_LEN];
    uint8_t *     pDataTemp ;
    uint16_t       Length;
    int i ;
    uint32_t      TimeStamp = TRACE_TIMESTAMP();
    uint8_t       Checksum;
    int         s;

    Length = strlen(pData) + TRACE_HEADER_LENGTH;
    if (Length > MAX_LOG_MESSAGE_LEN)
    {
        DebuggerBreak();
        return ((const char *)((0xDADD << 8) | Type));
    }

    pDataTemp = pTraceData;

    pTraceData[0] = TRACE_SYNC_CODE;
    pTraceData[1] = Length;
    pTraceData[4] = (uint8_t)((TimeStamp) & 0xff);
    pTraceData[5] = (uint8_t)(((TimeStamp) >> 8) & 0xff);
    pTraceData[6] = Type;

    memcpy(&pTraceData[TRACE_HEADER_LENGTH], pData, (Length - TRACE_HEADER_LENGTH));
    s = osInterruptDisable();

	gSequenceNumber++;
	pTraceData[2] = gSequenceNumber;
	pDataTemp = pTraceData;

    Checksum = *pDataTemp++;
    for (i = 1; i < TRACE_CHECKSUM_DATA_LENGTH; i++)
    {
        Checksum ^= *pDataTemp++;
    }
    pTraceData[3] = Checksum;
	stTraceOutput(pTraceData, Length);
    osInterruptEnable(s);

    return ((const char *)((0xDADD << 8) | Type));
}

/**
* @brief  trace ram data1
*
* @param  Mid
* @param  pData
*
* @return 
*/
const char *traceRamData1(uint8_t Mid, uint32_t level, char *pData)
{
    return (traceRamData(Mid, level, pData, TRACE_TYPE_RAMDATA1));
}

/**
* @brief  trace ram data2
*
* @param  Mid
* @param  pData
*
* @return 
*/
const char *traceRamData2(uint8_t Mid, uint32_t level, char *pData)
{
    return (traceRamData(Mid, level, pData, TRACE_TYPE_RAMDATA2));
}

/**
* @brief  trace ram data3
*
* @param  Mid
* @param  pData
*
* @return 
*/
const char *traceRamData3(uint8_t Mid, uint32_t level, char *pData)
{
    return (traceRamData(Mid, level, pData, TRACE_TYPE_RAMDATA3));
}

/**
* @brief  trace ram data4
*
* @param  Mid
* @param  pData
*
* @return 
*/
const char *traceRamData4(uint8_t Mid, uint32_t level, char *pData)
{
    return (traceRamData(Mid, level, pData, TRACE_TYPE_RAMDATA4));
}

/**
* @brief  trace ram data5
*
* @param  Mid
* @param  pData
*
* @return 
*/
const char *traceRamData5(uint8_t Mid, uint32_t level, char *pData)
{
    return (traceRamData(Mid, level, pData, TRACE_TYPE_RAMDATA5));
}

/**
* @brief  trace ram data6
*
* @param  Mid
* @param  pData
*
* @return 
*/
const char *traceRamData6(uint8_t Mid, uint32_t level, char *pData)
{
    return (traceRamData(Mid, level, pData, TRACE_TYPE_RAMDATA6));
}

/**
* @brief  trace ram data7
*
* @param  Mid
* @param  pData
*
* @return 
*/
const char *traceRamData7(uint8_t Mid, uint32_t level, char *pData)
{
    return (traceRamData(Mid, level, pData, TRACE_TYPE_RAMDATA7));
}

/**
* @brief  trace ram data8
*
* @param  Mid
* @param  pData
*
* @return 
*/
const char *traceRamData8(uint8_t Mid, uint32_t level, char *pData)
{
    return (traceRamData(Mid, level, pData, TRACE_TYPE_RAMDATA8));
}

void traceLevel(uint32_t level)
{
    otp_str_data.gEfuse_UpperStack_s.traceMask  = level;
}
#endif

/**
* @brief  Trace init
*
* @param  
*
* @return 
*/
int  traceInit(void)
{
   	stTraceInit();
	  return(TRUE);
}

