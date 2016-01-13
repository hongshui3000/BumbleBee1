/**
********************************************************************************************************
Copyright (c) 2015, Realtek Semiconductor Corporation. All rights reserved.
*********************************************************************************************************
* @file        l2c_extf.h
* @brief      Bluetooth L2CAP Layer
*          	     Extended features
* @details   
*
* @author   	gordon
* @date      	2015-07-13
* @version	v0.1
*/

#ifndef __L2C_EXTF_H
#define __L2C_EXTF_H

#include <crc16btx.h>          /* Polynomial: X**0 + X**2 + X**15 + X**16 */

#ifdef __cplusplus
extern "C" {
#endif


/****************************************************************************/
/*  frame definition                                                        */
/****************************************************************************/

                                        /* Frame Format                     */
#define FORMAT_I_FRAME    0             /* xxxx xxx0 Information Transfer   */
#define FORMAT_S_FRAME    1             /* xxxx xx01 Supervisory            */

#define FORMAT_I_FRAME_MASK    1
#define FORMAT_S_FRAME_MASK    3

#define S_FRAME_MASK      0x000f        /* xxxx ss01                        */

                                        /* Frame Type                       */
#define TYPE_I_FRAME      0x0000        /* xxxx xxx0 Information Frame      */

#define TYPE_RR_FRAME     0x0001        /* xxxx 0001 Receive Ready          */
#define TYPE_REJ_FRAME    0x0005        /* xxxx 0101 Reject                 */
#define TYPE_RNR_FRAME    0x0009        /* xxxx 1001 Receive Not Ready      */
#define TYPE_SREJ_FRAME   0x000D        /* xxxx 1101 Select Reject          */


#define RETRANSMISSION_DISABLE_BIT   0x0080 /* octet 1 */
#define FINAL_BIT                    0x0080 /* octet 1 */
#define POLL_BIT                     0x0010 /* octet 1 */

#define TXSEQ_MASK              0x007E  /* TxSeg mask  octet 1 */
#define REQSEQ_MASK             0x3F00  /* ReqSeg mask octet 2 */
#define SAR_MASK                0xC000  /* Segmentation and Reassembly mask octet 2 */
#define MODULO_MASK             0x3F    /* 6 bits sequence number */

#define L2CAP_SAR_UNSEGMENTED   0x0000  /* 00 - Unsegmented L2CAP SDU */
#define L2CAP_SAR_START         0x4000  /* 01 - Start of L2CAP SDU */
#define L2CAP_SAR_END           0x8000  /* 10 - End of L2CAP SDU */
#define L2CAP_SAR_CONTINUATION  0xC000  /* 11 - Continuation of L2CAP SDU */


#define L2CAP_CONTROL_LENGTH         2  /* 16 bits Control */
#define L2CAP_FCS_LENGTH             2  /* 16 bits Frame Check Sequence */
#define L2CAP_SDU_LENGTH_FIELD_SIZE  2  /* 16 bits SDU length field */

#define L2CAP_ERTM_OFFSET         (ACL_HDR_LENGTH          + \
                                   L2CAP_HDR_LENGTH        + \
                                   BT_L1_HCI_DS_OFFSET)


/****************************************************************************/
/*  default value                                                           */
/****************************************************************************/

#define L2CAP_MAX_TRANSMIT            2
#define L2CAP_RETRANSMISSION_TIMEOUT  2000
#define L2CAP_MONITOR_TIMEOUT         12000


#define L2CAP_MAX_RX_IFRAME       3   /* default 3 */       /**<Max. I frames send to upper layer*/

#define LFSR_LOAD_VALUE           0x0000


/****************************************************************************/
/*  S Frame buffers                                                         */
/****************************************************************************/

#define L2CAP_SFRAME_LENGTH       (L2CAP_CONTROL_LENGTH + L2CAP_FCS_LENGTH)
#define L2CAP_SFRAME_BUFFER       (4 * L2CAP_MAX_ENHANCED_FEATURE_CHANNELS)

#define L2CAP_SFRAME_OFFSET       L2CAP_ERTM_OFFSET

/****************************************************************************/
/*  Definition of extended channel                                          */
/****************************************************************************/

typedef struct _L2CAP_CHANNEL_EXT
{
  BOOL          Used;
  uint8_t          TxQueueID;               /* queue ID for transmit queue */
  uint8_t          SentQueueID;             /* queue ID for sent queue */
  uint8_t          RxQueueID;               /* queue ID for receive queue */
  BOOL          SREJMode;
  BOOL          FinalBitExpected;        /* TRUE -> WAIT_ACK */

  BOOL          RemoteBusy;
  BOOL          LocalBusy;
  BOOL          RetransmissionTimerStarted;
  BOOL          MonitorTimerStarted;
  BOOL          AckTimerStarted;

  BOOL          TxSeqSequenceError;      /* TRUE -> REJ_SENT */
  BOOL          AckSent;
  BOOL          SendFinalBit;            /* Poll command received after sending REJ */
  BOOL          SendREJ;

  uint16_t          AckTime;
  uint16_t          Control;                 /* S Frame: Poll or Final Bit */
  uint16_t          SAR;
  uint16_t          TotalLength;
  uint16_t          MissingFrameCount;       /* SM: missing or discarded frame count */
  uint8_t          DIFF;                    /* number of unacknowledged I frames */
  uint8_t          RxCount;
  uint8_t          RetransmitCount;
  uint8_t          RetransmitCountIFrame;
  uint8_t          RxIFrameCount;

  uint8_t          RxIFrameMaxCount;

  uint8_t          FrameType;
  uint8_t          ReqSeq;                  /* N(R) Receive Sequence Number */
  uint8_t          TxSeq;                   /* N(S) Send Sequence Number */
  uint8_t          NextTxSeq;               /* V(S) Next Send Sequence Number */
  uint8_t          ExpectedAckSeq;          /* V(SU) Expected Ack Sequence Number */
  uint8_t          ExpectedTxSeq;           /* V(R) Expected Send Sequence Number */
} T_L2CAP_CHANNEL_EXT;
typedef T_L2CAP_CHANNEL_EXT *P_L2CAP_CHANNEL_EXT;

#if F_BT_L2C_ENHANCED_FEATURE_SUPPORT //champion

void l2cHandleTIMER_EXPIREDMonitor(P_L2CAP_CHANNEL pChan);
void l2cHandleTIMER_EXPIREDRetransmission(P_L2CAP_CHANNEL pChan);
void l2cHandleTIMER_EXPIREDAck(P_L2CAP_CHANNEL pChan);
void l2cHandleDownstreamData(P_L2CAP_CHANNEL pChan, MESSAGE_P pMsg);
void l2cHandleUpstreamPacket(P_L2CAP_CHANNEL pChan, MESSAGE_P pMsg);
void l2cStateIsOpened(P_L2CAP_CHANNEL pChan);
void l2cStateIsClosed(P_L2CAP_CHANNEL pChan);

#else
#define l2cHandleTIMER_EXPIREDMonitor(pChan)
#define l2cHandleTIMER_EXPIREDRetransmission(pChan)
#define l2cHandleTIMER_EXPIREDAck(pChan)
#define l2cHandleDownstreamData(pChan, pMsg)
#define l2cHandleUpstreamPacket(pChan, pMsg)
#define l2cStateIsOpened(pChan) 
#define l2cStateIsClosed(pChan)
#endif

#ifdef __cplusplus
}
#endif

#endif
