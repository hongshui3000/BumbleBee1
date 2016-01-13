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

#if !defined(__TRACE_BINARY_H)
#define      __TRACE_BINARY_H

#include <flags.h>
#include <stdint.h>
#include <stdbool.h>
#include <section_config.h>

#define MID_BT_HDP           0x6D      /**< BT Health device Profile         */
#define MID_BTLTP            0x6E      /**< BT Local Transport Protocol       */
#define MID_BLUEHDP_API      0x6F      /**< BlueHDP API debounce & forward */

#define MID_BLUEFACE         0x90      /**< Bluetooth Interface Application   */
#define MID_BT_HCI           0x91      /**< HCI                               */
#define MID_BT_L2C           0x92      /**< L2CAP                             */
#define MID_BT_RFC           0x93      /**< RFCOMM                            */
#define MID_BT_SDP           0x94      /**< SDP                               */
#define MID_BT_OBEX          0x96      /**< OBEX                              */
#define MID_BT_BNEP          0x9A      /**< Bluetooth Network Encapsulation Protocol */
#define MID_BT_AVDTP         0x9B      /**< Audio Video Distribution Transport Protocol */
#define MID_BT_GATT          0x9C      /**< GATT */
#define MID_BT_MESHSR        0x9D      /**< meshed network dual mode */

#define MID_TRACE            0xC0      /**< Trace Module                      */

#define MID_BTSECMAN         0xD2      /**< bluetooth security manager 2.1    */
#define MID_BT_HSP           0xED      /**< bluetooth headset profile     */
#define MID_BT_AVP           0xEF      /**< bluetooth AV profile          */

#define MID_OS               0xFF      /**< OS object ID                      */

#define TRACE_DIRECTION_MESSAGE_SEND         0
#define TRACE_DIRECTION_MESSAGE_RECEIVE      1

#define TRACE_DIRECTION_BINARY_UPSTREAM         0x00
#define TRACE_DIRECTION_BINARY_DOWNSTREAM    0x80


#define TRACE_PKT_TYPE_BT_SNOOP_HCI_ACL_TX         0x02
#define TRACE_PKT_TYPE_BT_SNOOP_HCI_ACL_RX         0x12
#define TRACE_PKT_TYPE_BT_SNOOP_HCI_CMD              0x21
#define TRACE_PKT_TYPE_BT_SNOOP_HCI_EVT               0x34


#define TRACE_PKT_TYPE_HCI               0x40
#define TRACE_PKT_TYPE_L2C               0x50
#define TRACE_PKT_TYPE_GATT               0x60
#define TRACE_PKT_TYPE_BTSM               0x70
#define TRACE_PKT_TYPE_BLUEFACE          0x80
#define TRACE_PKT_TYPE_BLUEAPI_OSIF_DS          0x90
#define TRACE_PKT_TYPE_BLUEAPI_OSIF_US          0x91
#define TRACE_PKT_TYPE_APPL                                 0xA0

void tracePrintf(uint8_t Mid, uint32_t level, const char * pFormat, int Count, ...);

void traceMessage(uint8_t Mid, uint8_t MessageDirection, uint16_t Qid, uint8_t Length, uint8_t *pMessage);


void traceBinary(uint8_t Mid, uint8_t pkt_type, uint16_t Length, uint8_t *pData);
void traceString(uint8_t Mid, uint8_t Length, char *pString);

const char *traceBdAddr1(uint8_t Mid, uint32_t level, const char *pBdAddr);
const char *traceBdAddr2(uint8_t Mid, uint32_t level,const char *pBdAddr);
const char *traceRamData1(uint8_t Mid,uint32_t level, char *pData);
const char *traceRamData2(uint8_t Mid,uint32_t level, char *pData);
const char *traceRamData3(uint8_t Mid,uint32_t level, char *pData);
const char *traceRamData4(uint8_t Mid,uint32_t level, char *pData);
const char *traceRamData5(uint8_t Mid,uint32_t level, char *pData);
const char *traceRamData6(uint8_t Mid,uint32_t level, char *pData);
const char *traceRamData7(uint8_t Mid, uint32_t level,char *pData);
const char *traceRamData8(uint8_t Mid, uint32_t level,char *pData);

void traceLevel(uint32_t level);


#define CONFIG_DEBUG_UPPERSTACK_LOG

#ifdef CONFIG_DEBUG_UPPERSTACK_LOG

#define BTRACE_MESSAGE_SEND(Qid, Length, pMessage)        \
                 traceMessage(TRACE_MODULE_ID, TRACE_DIRECTION_MESSAGE_SEND, Qid, Length, pMessage)
#define BTRACE_MESSAGE_RECEIVE(Qid, Length, pMessage)     \
                 traceMessage(TRACE_MODULE_ID, TRACE_DIRECTION_MESSAGE_RECEIVE, Qid, Length, pMessage)

#define BTRACE_BINARY_UPSTREAM(PktType, Length, pData)             \
    traceBinary(TRACE_MODULE_ID, PktType, Length, pData)
#define BTRACE_BINARY_DOWNSTREAM(PktType, Length, pData)           \
    traceBinary(TRACE_MODULE_ID, PktType, Length, pData)

//#define BTRACE_STRING(Length, pString)   traceString(TRACE_MODULE_ID, Length, pString)
#define BTRACE_BDADDR1(level, pBdAddr)          traceBdAddr1(TRACE_MODULE_ID, level, (const char *)(pBdAddr))
#define BTRACE_BDADDR2(level, pBdAddr)          traceBdAddr2(TRACE_MODULE_ID, level, (const char *)(pBdAddr))
#define BTRACE_RAMDATA1(level, pData)           traceRamData1(TRACE_MODULE_ID, level, (char *)(pData))
#define BTRACE_RAMDATA2(level, pData)           traceRamData2(TRACE_MODULE_ID, level, (char *)(pData))
#define BTRACE_RAMDATA3(level, pData)           traceRamData3(TRACE_MODULE_ID, level, (char *)(pData))
#define BTRACE_RAMDATA4(level, pData)           traceRamData4(TRACE_MODULE_ID, level, (char *)(pData))
#define BTRACE_RAMDATA5(level, pData)           traceRamData5(TRACE_MODULE_ID, level, (char *)(pData))
#define BTRACE_RAMDATA6(level, pData)           traceRamData6(TRACE_MODULE_ID, level, (char *)(pData))
#define BTRACE_RAMDATA7(level, pData)           traceRamData7(TRACE_MODULE_ID, level, (char *)(pData))
#define BTRACE_RAMDATA8(level, pData)           traceRamData8(TRACE_MODULE_ID, level, (char *)(pData))

#define BTRACE_PRINTF_0(level, pFormat)                                                  \
    {                                                                               \
        static const char traceFormat[] TRACE_DATA = pFormat;                         \
        tracePrintf(TRACE_MODULE_ID, level, traceFormat, 0);                                 \
    }
#define BTRACE_PRINTF_1(level, pFormat, Arg1)                                            \
    {                                                                               \
        static const char traceFormat[] TRACE_DATA = pFormat;                         \
        tracePrintf(TRACE_MODULE_ID, level, traceFormat, 1,                                  \
                    Arg1);                                                                    \
    }
#define BTRACE_PRINTF_2(level, pFormat, Arg1, Arg2)                                      \
    {                                                                               \
        static const char traceFormat[] TRACE_DATA = pFormat;                         \
        tracePrintf(TRACE_MODULE_ID, level, traceFormat, 2,                                  \
                    Arg1, Arg2);                                                              \
    }
#define BTRACE_PRINTF_3(level, pFormat, Arg1, Arg2, Arg3)                                \
    {                                                                               \
        static const char traceFormat[] TRACE_DATA = pFormat;                         \
        tracePrintf(TRACE_MODULE_ID, level, traceFormat, 3,                                  \
                    Arg1, Arg2, Arg3);                                                        \
    }
#define BTRACE_PRINTF_4(level, pFormat, Arg1, Arg2, Arg3, Arg4)                          \
    {                                                                               \
        static const char traceFormat[] TRACE_DATA = pFormat;                         \
        tracePrintf(TRACE_MODULE_ID, level, traceFormat, 4,                                  \
                    Arg1, Arg2, Arg3, Arg4);                                                  \
    }
#define BTRACE_PRINTF_5(level, pFormat, Arg1, Arg2, Arg3, Arg4, Arg5)                    \
    {                                                                               \
        static const char traceFormat[] TRACE_DATA = pFormat;                         \
        tracePrintf(TRACE_MODULE_ID, level, traceFormat, 5,                                  \
                    Arg1, Arg2, Arg3, Arg4,                                                   \
                    Arg5);                                                                    \
    }
#define BTRACE_PRINTF_6(level, pFormat, Arg1, Arg2, Arg3, Arg4, Arg5, Arg6)              \
    {                                                                               \
        static const char traceFormat[] TRACE_DATA = pFormat;                         \
        tracePrintf(TRACE_MODULE_ID, level, traceFormat, 6,                                  \
                    Arg1, Arg2, Arg3, Arg4,                                                   \
                    Arg5, Arg6);                                                              \
    }
#define BTRACE_PRINTF_7(level, pFormat, Arg1, Arg2, Arg3, Arg4, Arg5, Arg6, Arg7)        \
    {                                                                               \
        static const char traceFormat[] TRACE_DATA = pFormat;                         \
        tracePrintf(TRACE_MODULE_ID, level, traceFormat, 7,                                  \
                    Arg1, Arg2, Arg3, Arg4,                                                   \
                    Arg5, Arg6, Arg7);                                                        \
    }
#define BTRACE_PRINTF_8(level, pFormat, Arg1, Arg2, Arg3, Arg4, Arg5, Arg6, Arg7, Arg8)  \
    {                                                                               \
        static const char traceFormat[] TRACE_DATA = pFormat;                         \
        tracePrintf(TRACE_MODULE_ID, level, traceFormat, 8,                                  \
                    Arg1, Arg2, Arg3, Arg4,                                                   \
                    Arg5, Arg6, Arg7, Arg8);                                                  \
    }

#else


#define BTRACE_MESSAGE_SEND(level, Qid, Length, pMessage)
#define BTRACE_MESSAGE_RECEIVE(level, Qid, Length, pMessage)

#define BTRACE_BINARY_UPSTREAM(PktType, Length, pData)
#define BTRACE_BINARY_DOWNSTREAM(PktType, Length, pData)

#define BTRACE_STRING(level, Length, pString)

#define BTRACE_BDADDR1(level, pBdAddr)
#define BTRACE_BDADDR2(level, pBdAddr)
#define BTRACE_RAMDATA1(level, pData)
#define BTRACE_RAMDATA2(level, pData)
#define BTRACE_RAMDATA3(level, pData)
#define BTRACE_RAMDATA4(level, pData)
#define BTRACE_RAMDATA5(level, pData)
#define BTRACE_RAMDATA6(level, pData)
#define BTRACE_RAMDATA7(level, pData)
#define BTRACE_RAMDATA8(level, pData)

#define BTRACE_PRINTF_0(level, pFormat)
#define BTRACE_PRINTF_1(level, pFormat, Arg1)
#define BTRACE_PRINTF_2(level, pFormat, Arg1, Arg2)
#define BTRACE_PRINTF_3(level, pFormat, Arg1, Arg2, Arg3)
#define BTRACE_PRINTF_4(level, pFormat, Arg1, Arg2, Arg3, Arg4)
#define BTRACE_PRINTF_5(level, pFormat, Arg1, Arg2, Arg3, Arg4, Arg5)
#define BTRACE_PRINTF_6(level, pFormat, Arg1, Arg2, Arg3, Arg4, Arg5, Arg6)
#define BTRACE_PRINTF_7(level, pFormat, Arg1, Arg2, Arg3, Arg4, Arg5, Arg6, Arg7)
#define BTRACE_PRINTF_8(level, pFormat, Arg1, Arg2, Arg3, Arg4, Arg5, Arg6, Arg7, Arg8)

#endif

int traceInit(void);

#include <trace_binary_def.h>
#include <trace_binary_mod.h>


#endif
