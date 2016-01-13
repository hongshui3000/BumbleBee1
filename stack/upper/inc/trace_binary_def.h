/**
********************************************************************************************************
Copyright (c) 2015, Realtek Semiconductor Corporation. All rights reserved.
*********************************************************************************************************
* @file       trace_binary_def.h
* @brief     Binary Trace
* @details   
*
* @author   gordon
* @date      2015-07-13
* @version	v0.1
*/

#if !defined(__TRACE_BINARY_DEF_H)
#define      __TRACE_BINARY_DEF_H

#include <flags.h>

#if !defined(__ENDIALIG_H)
#endif

#include <os_sched.h>

/****************************************************************************/
/* Trace packet header                                                      */
/****************************************************************************/
/* 0:  SYNC                                                                 */
/* 1:  Length                                                               */
/* 2:  SequenceNumber                                                       */
/* 3:  Checksum                                                             */
/* 4:  Timestamp                                                            */
/* 8:  ModuleID                                                             */
/* 9:  Type                                                                 */
/****************************************************************************/

#define TRACE_SYNC_CODE             0x7E

typedef struct _TTracePacketHeader
{
  uint8_t    Sync;              /* 0x7e */
  uint8_t    Length;            /* total length of the packet */
  uint8_t    SequenceNumber;    /* sequence number */
  uint8_t    Checksum;          /* exclusive or of the sync code, the length field and sequence number */
} TTracePacketHeader;
typedef TTracePacketHeader *PTracePacketHeader;

#define TRACE_CHECKSUM_DATA_LENGTH      (sizeof(uint8_t) +      /* Sync   */ \
                                         sizeof(uint8_t) +      /* Length */ \
                                         sizeof(uint8_t))       /* SequenceNumber */
#define TRACE_PACKET_HEADER_LENGTH      (TRACE_CHECKSUM_DATA_LENGTH + sizeof(uint8_t) /* checksum */)

#define TRACE_HEADER_LENGTH             (TRACE_PACKET_HEADER_LENGTH +                    \
                                         sizeof(uint16_t)                 + /* TimeStamp */    \
                                         sizeof(uint8_t))                    /* Type */
                     

/****************************************************************************/
/* Definitions                                                              */
/****************************************************************************/

#define TRACE_VERSION_MAJOR                  1    /* V1.0 */
#define TRACE_VERSION_MINOR                  0

#define TRACE_TIMESTAMP()                    osGetSystemTime()

#define TRACE_TYPE_RESET                     0
#define TRACE_TYPE_FORMAT                    1
#define TRACE_TYPE_MESSAGE                   2
#define TRACE_TYPE_BINARY                    3
#define TRACE_TYPE_STRING                    4
#define TRACE_TYPE_BDADDR1                   5
#define TRACE_TYPE_BDADDR2                   6
#define TRACE_TYPE_RAMDATA1                  7
#define TRACE_TYPE_RAMDATA2                  8
#define TRACE_TYPE_RAMDATA3                  9
#define TRACE_TYPE_RAMDATA4                  10
#define TRACE_TYPE_RAMDATA5                  11
#define TRACE_TYPE_RAMDATA6                  12
#define TRACE_TYPE_RAMDATA7                  13
#define TRACE_TYPE_RAMDATA8                  14

/* Bit definition - string and binary only */
#define TRACE_TYPE_MORE_DATA                 0x80
#define TRACE_TYPE_LAST_PACKET               0x40   /* not all data can be sent */
                                                    /* out of memory or limited by flag */

#define TRACE_TYPE_MASK                      0x3F


#endif /* !defined(__TRACE_BINARY_DEF_H) */
