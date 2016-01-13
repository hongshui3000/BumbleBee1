/**
********************************************************************************************************
Copyright (c) 2015, Realtek Semiconductor Corporation. All rights reserved.
*********************************************************************************************************
* @file        l2c_extf.c
* @brief      Bluetooth L2CAP Layer
*          	     Extended features
* @details   
*
* @author   	gordon
* @date      	2015-07-13
* @version	v0.1
*/

#include <flags.h>
#include <os_message.h>
#include <blueface.h>
#include <btcommon.h>
#include <l2c.h>
#include <btglib.h>

#define X_L2C_SREJ_MODE                          1
#define X_L2C_REJ_POLL_BIT                       0   /**< 0->P=0, POLL_BIT->P=1 */
#define X_L2C_IGNORE_IFRAME_IN_LOCALBUSY_STATE   0
#define X_L2C_DISCARD_IFRAME                     0
#define X_L2C_DISCONNECT_LINK_ON__INV_FRAME      0

#if F_BT_L2C_ENHANCED_FEATURE_SUPPORT //champion
void l2cHandleIFrame(P_L2CAP_CHANNEL pChan, MESSAGE_P pMsg);

void l2cStartRETRANSMISSIONTimeout(P_L2CAP_CHANNEL pChan)
{
    MESSAGE_T Message;

    if (pChan->pExtF->RetransmissionTimerStarted == FALSE)
    {
        pChan->pExtF->RetransmissionTimerStarted = TRUE;

        L2CAP_TRACE_VERBOSITY_3(L2CAP_TRACE_PRINTF_1(L2CAP_TRACE_MASK_TRACE,
                                "l2cStartRETRANSMISSIONTimeout for ID 0x%X", pChan->LocalCid));

        hwSetTimer(TIMER_TYPE_TICKS, L2CAP_RETRANSMISSION_TID, (uint8_t)pChan->LocalCid, pChan->ConfParaLocal.flowControl.retransmissionTimeout / TIMER_TICK, l2cQueueID);
    }
}

void l2cStopRETRANSMISSIONTimeout(P_L2CAP_CHANNEL pChan)
{
	MESSAGE_T Message;

	if (pChan->pExtF->RetransmissionTimerStarted == TRUE)
	{
		pChan->pExtF->RetransmissionTimerStarted = FALSE;

		L2CAP_TRACE_VERBOSITY_3(L2CAP_TRACE_PRINTF_1(L2CAP_TRACE_MASK_TRACE,
		                        "l2cStopRETRANSMISSIONTimeout() stopping RETRANSMISSION timer for id 0x%X",
		                        pChan->LocalCid));

		hwResetTimer(TIMER_TYPE_TICKS, L2CAP_RETRANSMISSION_TID, (uint8_t)pChan->LocalCid, l2cQueueID);
	}
}

void l2cStartMONITORTimeout(P_L2CAP_CHANNEL pChan)
{
	MESSAGE_T Message;

	if (pChan->pExtF->MonitorTimerStarted == FALSE)
	{
		pChan->pExtF->MonitorTimerStarted = TRUE;

		L2CAP_TRACE_VERBOSITY_3(L2CAP_TRACE_PRINTF_1(L2CAP_TRACE_MASK_TRACE,
		                        "l2cStartMONITORTimeout() for ID 0x%X", pChan->LocalCid));

		hwSetTimer(TIMER_TYPE_TICKS,L2CAP_MONITOR_TID,(uint8_t)pChan->LocalCid,TIMER_TYPE_TICKS,l2cQueueID);
	}
}

void l2cStopMONITORTimeout(P_L2CAP_CHANNEL pChan)
{
	MESSAGE_T Message;

	if (pChan->pExtF->MonitorTimerStarted == TRUE)
	{
		pChan->pExtF->MonitorTimerStarted = FALSE;

		L2CAP_TRACE_VERBOSITY_3(L2CAP_TRACE_PRINTF_1(L2CAP_TRACE_MASK_TRACE,
		                        "l2cStopMONITORTimeout() stopping MONITOR timer for id 0x%X", pChan->LocalCid));

		hwResetTimer(TIMER_TYPE_TICKS, L2CAP_MONITOR_TID, (uint8_t)pChan->LocalCid, l2cQueueID);
	}
}

#if defined(L2CAP_ACK_TID)
void l2cStartACKTimeout(P_L2CAP_CHANNEL pChan)
{
	MESSAGE_T Message;

	if (pChan->pExtF->AckTimerStarted == FALSE)
	{
		pChan->pExtF->AckTimerStarted = TRUE;

		L2CAP_TRACE_VERBOSITY_3(L2CAP_TRACE_PRINTF_1(L2CAP_TRACE_MASK_TRACE,
		                        "l2cStartACKTimeout() for ID 0x%X", pChan->LocalCid));
		hwSetTimer(TIMER_TYPE_TICKS, L2CAP_ACK_TID,(uint8_t)pChan->LocalCid, pChan->pExtF->AckTime, l2cQueueID);
	}
}

void l2cStopACKTimeout(P_L2CAP_CHANNEL pChan)
{
	MESSAGE_T Message;

	if (pChan->pExtF->AckTimerStarted == TRUE)
	{
		pChan->pExtF->AckTimerStarted = FALSE;

		L2CAP_TRACE_VERBOSITY_3(L2CAP_TRACE_PRINTF_1(L2CAP_TRACE_MASK_TRACE,
		                        "l2cStopACKTimeout() stopping ACK timer for id 0x%X", pChan->LocalCid));

		hwResetTimer(TIMER_TYPE_TICKS, L2CAP_ACK_TID, (uint8_t)pChan->LocalCid, l2cQueueID);
	}
}
#endif  /* defined(L2CAP_ACK_TID) */

#if F_BT_L2C_ENHANCED_FEATURE_SUPPORT
void l2cHandleERTMError(P_L2CAP_CHANNEL pChan, uint16_t Status)
{
    l2cStopRTXTimeout(pChan);
    l2cStopCONFIGTimeout(pChan);   /* stop timeout */
	
    l2cChangeState(pChan, l2cStateWaitForL2C_DISC_RESP);
    l2cSendL2C_DISC_IND(pChan, Status | L2CAP_ERR);

    l2cSendL2CAP_DISCONNECTION_REQUEST(pChan);

 //   l2cChangeState(pChan, l2cStateWaitForL2C_DISC_RESP);
}
#endif

void l2cCloseChannel(P_L2CAP_CHANNEL pChan, uint16_t Status)
{
#if defined(L2CAP_ACK_TID)
	l2cStopACKTimeout(pChan);
#endif
	l2cStopMONITORTimeout(pChan);
	l2cStopRETRANSMISSIONTimeout(pChan);

	l2cHandleERTMError(pChan, Status);
}

void l2cDiscardTxMessage(MESSAGE_P pMsg)
{
	osBufferRelease(pMsg->MData.DataCB.BufferAddress);
}

void l2cBuildControl(P_L2CAP_CHANNEL pChan, uint8_t * pFrame, uint16_t SAR)
{
	uint16_t Control = SAR;             /**< Bit 0 = 0 -> I frame */

	Control |= (pChan->pExtF->NextTxSeq << 1);         /**<  TxSeq */
	if (pChan->Mode != L2CAP_MODE_STREAMING)  /**< Streaming mode: ReqSeq = 0 */
	{
		Control |= (pChan->pExtF->ExpectedTxSeq << 8);   /**< ReqSeq */
	}

	SHORT2CHAR(pFrame, Control);      /**< write 16 bits control */
	pChan->pExtF->RxIFrameCount = 0;
}

void l2cBuildFcs(MESSAGE_P pMsg, uint16_t L2CapSize, uint16_t cid)
{
	uint8_t * pPdu;
	uint16_t   wtmp;

	pPdu = pMsg->MData.DataCB.BufferAddress + pMsg->MData.DataCB.Offset;

	L2CapSize += L2CAP_FCS_LENGTH;

	pPdu = pPdu - 4;                    /**< pos ptr 4 bytes back for L2c header */
	SHORT2CHAR(pPdu, L2CapSize);        /**< write L2Cap packet length */
	SHORT2CHAR(pPdu + 2, cid);          /**< and channel ID */

	wtmp = btxfcs(LFSR_LOAD_VALUE, pPdu, (uint16_t)(pMsg->MData.DataCB.Length + 4));
	SHORT2CHAR(pPdu + pMsg->MData.DataCB.Length + 4, wtmp); /**< write 16 bits FCS */
	pMsg->MData.DataCB.Length += L2CAP_FCS_LENGTH;
}

#if (L2CAP_TRACE_VERBOSITY_COUNT != 0) 

STATIC const uint8_t  SFrameNameRR[]   = "RR";
STATIC const uint8_t  SFrameNameRNR[]  = "RNR";
STATIC const uint8_t  SFrameNameREJ[]  = "REJ";
STATIC const uint8_t  SFrameNameSREJ[] = "SREJ";

LPCSTR l2cGetSFrameName(uint16_t SFrame)
{
	LPCSTR pSFrame = (LPCSTR)0;

	switch (SFrame)
	{
	case TYPE_RR_FRAME:
		pSFrame = (LPCSTR)SFrameNameRR;
		break;
	case TYPE_RNR_FRAME:
		pSFrame = (LPCSTR)SFrameNameRNR;
		break;
	case TYPE_REJ_FRAME:
		pSFrame = (LPCSTR)SFrameNameREJ;
		break;
	case TYPE_SREJ_FRAME:
		pSFrame = (LPCSTR)SFrameNameSREJ;
		break;
	}
	return(pSFrame);
}
#endif  /**< (L2CAP_TRACE_VERBOSITY_COUNT != 0) */

void l2cSendFrame(P_L2CAP_CHANNEL pChan, MESSAGE_P pMsg)
{
    if (FALSE == gL2cDsFragmentationSupport)
    {
	    l2cSendL2CAP_PDU(pChan->pHciDesc->handle, pMsg,
	                       pMsg->MData.DataCBChan.Length,
	                       pChan->RemoteCid,
	                       TRUE);
    }
    else
    {
	    pMsg->MData.DataCBChan.Channel = pChan->RemoteCid;
	    osMessageSend(pChan->pHciDesc->dsBufQueue, pMsg);
	    l2cFragmentDATA_REQ(pChan->pHciDesc);
    }
}

void l2cSendSFrame(P_L2CAP_CHANNEL pChan, uint16_t SFrame, uint16_t PollFinalBit)
{
	P_L2CAP_CHANNEL_EXT pExtF = pChan->pExtF;
	MESSAGE_T           Msg;
	uint8_t *              pFrame;
	uint16_t                Control;

	if (osBufferGet(pL2c->SFramePool, (uint16_t)(L2CAP_SFRAME_LENGTH + L2CAP_SFRAME_OFFSET),
	                                (PVOID  *)&Msg.MData.DataCBChan.BufferAddress))
	{
		L2CAP_TRACE_PRINTF_1(L2CAP_TRACE_MASK_ERROR,
		                        "!!l2cSendSFrame: Could not allocate memory %X",
		                        L2CAP_SFRAME_LENGTH + L2CAP_SFRAME_OFFSET);
		return;
	}

	Msg.MData.DataCBChan.Flag = DATA_CB_RELEASE;

	pFrame = Msg.MData.DataCBChan.BufferAddress + L2CAP_SFRAME_OFFSET;

	if (SFrame == TYPE_RR_FRAME && pExtF->LocalBusy == TRUE)
	{
		SFrame = TYPE_RNR_FRAME;
	}

	if (SFrame == TYPE_REJ_FRAME && pExtF->SREJMode)
	{
		SFrame = TYPE_SREJ_FRAME;
	}

	Control  = SFrame;
	Control |= (pExtF->ExpectedTxSeq << 8) | PollFinalBit;

	SHORT2CHAR(pFrame, Control);      /**< write 16 bits control */

	Msg.MData.DataCBChan.Offset = L2CAP_SFRAME_OFFSET;
	Msg.MData.DataCBChan.Length = L2CAP_CONTROL_LENGTH;


	L2CAP_TRACE_VERBOSITY_3(L2CAP_TRACE_PRINTF_4(L2CAP_TRACE_MASK_TRACE,
	                      "L2C<- l2cSendSFrame %s ReqSeq=%2.2x P=%c F=%c",
	                      l2cGetSFrameName(SFrame),
	                      pExtF->ExpectedTxSeq,
	                      ((PollFinalBit & POLL_BIT) ? '1':'0'),
	                      ((PollFinalBit & FINAL_BIT) ? '1':'0')));

	pExtF->AckSent      = TRUE;
	pExtF->RxIFrameCount = 0;
#if defined(L2CAP_ACK_TID)
	if (pExtF->AckTimerStarted)
	l2cStopACKTimeout(pChan);
#endif

	if (PollFinalBit & POLL_BIT)
	{
		pExtF->FinalBitExpected = TRUE;
		l2cStopMONITORTimeout(pChan);
		l2cStartMONITORTimeout(pChan);
	}

	if (pChan->Fcs)
	{
		l2cBuildFcs(&Msg, Msg.MData.DataCBChan.Length, pChan->RemoteCid);
	}

	l2cSendFrame(pChan, &Msg);
}

void l2cSendIFrame(P_L2CAP_CHANNEL pChan)
{
	P_L2CAP_CHANNEL_EXT pExtF = pChan->pExtF;
	MESSAGE_T           Msg;

	if (pExtF->RemoteBusy)            /**< R = 1 */
	{
		return;
	}

	if (pExtF->FinalBitExpected)
	{
		return;
	}

	/* F_BT_L2C_ENHANCED_CONFORMANCE */

	while (pExtF->DIFF < pChan->ConfParaRemote.flowControl.txWindowSize && /* window not full */
	     osMessageReceive(pExtF->TxQueueID, &Msg) == 0)
	{
		uint16_t SAR;
		uint16_t Flag;
		uint16_t Length;
		uint16_t Offset;

		Flag = Msg.MData.DataCBChan.Flag;
		if (Flag & DATA_CB_BLOCK_FIRST)
		{
	  		SAR = L2CAP_SAR_START;
		}
		else if (Flag & DATA_CB_BLOCK_MIDDLE)
		{
	  		SAR = L2CAP_SAR_CONTINUATION;
		}
		else if (Flag & DATA_CB_BLOCK_LAST)
		{
	  		SAR = L2CAP_SAR_END;
		}
		else
		{
	  		SAR = L2CAP_SAR_UNSEGMENTED;
		}

		if (pChan->Mode == L2CAP_MODE_STREAMING)
		{
			Msg.MData.DataCBChan.Flag |= DATA_CB_NON_RELIABLE_DATA;
		}
		else
		{
			Msg.MData.DataCBChan.Flag &= ~DATA_CB_RELEASE;
		}

		Length = Msg.MData.DataCBChan.Length;  /* original length and offset in sent queue */
		Offset = Msg.MData.DataCBChan.Offset;

		Msg.MData.DataCBChan.Length += L2CAP_CONTROL_LENGTH;
		Msg.MData.DataCBChan.Offset -= L2CAP_CONTROL_LENGTH;

		l2cBuildControl(pChan, Msg.MData.DataCBChan.BufferAddress +
		                       Msg.MData.DataCBChan.Offset,
		                       SAR);

		L2CAP_TRACE_VERBOSITY_3(L2CAP_TRACE_PRINTF_5(L2CAP_TRACE_MASK_TRACE,
		                              "L2C<- l2cSendIFrame SAR=%1.1x TxSeq=%2.2x ReqSeq=%2.2x F=%c Len=%4.4x",
		                              (uint8_t)(SAR >> 14),
		                              pExtF->NextTxSeq,
		                              pExtF->ExpectedTxSeq,
		                              '0',
		                              Msg.MData.DataCBChan.Length));

		if (pChan->Fcs)
		{
			l2cBuildFcs(&Msg, Msg.MData.DataCBChan.Length, pChan->RemoteCid);
		}

		/* F_BT_L2C_ENHANCED_CONFORMANCE */
		l2cSendFrame(pChan, &Msg);

		pExtF->AckSent = TRUE;
		l2cStopACKTimeout(pChan);

		if (pChan->Mode == L2CAP_MODE_ENHANCED_RETRANSMISSION)
		{
			if (pChan->pExtF->RetransmissionTimerStarted == FALSE)
			{
				pChan->pExtF->RetransmitCountIFrame = 0;
			}

			Msg.MData.DataCBChan.Length = Length;  /* original length and offset in sent queue */
			Msg.MData.DataCBChan.Offset = Offset;
			osMessageSend(pExtF->SentQueueID, &Msg);

			l2cStartRETRANSMISSIONTimeout(pChan);

			pExtF->DIFF++;
		}

		pExtF->NextTxSeq = (pExtF->NextTxSeq + 1) & MODULO_MASK;
	}
}

void l2cSendUnacknowledgedIFrame(P_L2CAP_CHANNEL pChan)
{
	P_L2CAP_CHANNEL_EXT pExtF = pChan->pExtF;
	MESSAGE_T           Msg;
	uint16_t                Control;
	uint8_t                Count;

	Count            = pExtF->DIFF;
	pExtF->DIFF      = 0;
	pExtF->NextTxSeq = pExtF->ExpectedAckSeq;

	L2CAP_TRACE_VERBOSITY_3(L2CAP_TRACE_PRINTF_1(L2CAP_TRACE_MASK_TRACE,
	                      "L2C<- l2cSendUnacknowledgedIFrame %2.2x I Frame in sent queue", Count));
	while (Count)
	{
		BOOL SendFrame;
		uint16_t Length;
		uint16_t Offset;

		if (osMessageReceive(pExtF->SentQueueID, &Msg))
		{
			return;
		}
		Length = Msg.MData.DataCBChan.Length;  /* original length and offset in sent queue */
		Offset = Msg.MData.DataCBChan.Offset;

		SendFrame = FALSE;

		if (pExtF->FrameType == TYPE_SREJ_FRAME) /* send only one frame         */
		{
			if (pExtF->Control & POLL_BIT)    /* P = 1 */
			{
				if (pExtF->DIFF == 0)           /* first not acknowleged frame */
				{
					SendFrame = TRUE;
				}
			}
			else                               /* P = 0 */
			{
				Control = CHAR2SHORT(Msg.MData.DataCBChan.BufferAddress + Msg.MData.DataCB.Offset - L2CAP_CONTROL_LENGTH);
				if (((Control & TXSEQ_MASK) >> 1) == pExtF->ReqSeq)
				{
					SendFrame = TRUE;
				}
			}
		}
		else
		{
			SendFrame = TRUE;
		}

		if (SendFrame)
		{
			Msg.MData.DataCB.Offset     -= L2CAP_CONTROL_LENGTH;
			Msg.MData.DataCBChan.Length += L2CAP_CONTROL_LENGTH;

			Control = CHAR2SHORT(Msg.MData.DataCBChan.BufferAddress + Msg.MData.DataCB.Offset);
			l2cBuildControl(pChan, Msg.MData.DataCBChan.BufferAddress +
			                     Msg.MData.DataCBChan.Offset,
			                     (uint16_t)(Control & SAR_MASK));

			if (pExtF->Control & POLL_BIT)
			{
				pExtF->Control &=~POLL_BIT;
				Control  = CHAR2SHORT(Msg.MData.DataCBChan.BufferAddress + Msg.MData.DataCB.Offset);
				Control |= FINAL_BIT;
				SHORT2CHAR(Msg.MData.DataCBChan.BufferAddress + Msg.MData.DataCB.Offset, Control);
			}

			if (pChan->Fcs)
			{
				l2cBuildFcs(&Msg, Msg.MData.DataCBChan.Length, pChan->RemoteCid);
			}

			l2cSendFrame(pChan, &Msg);

			L2CAP_TRACE_VERBOSITY_3(L2CAP_TRACE_PRINTF_5(L2CAP_TRACE_MASK_TRACE,
			                      "L2C<- l2cSendUnacknowledgedIFrame SAR=%1.1x TxSeq=%2.2x ReqSeq=%2.2x F=%c len=%4.4x",
			                      (uint8_t)((Control & SAR_MASK) >> 14),
			                      ((Control & TXSEQ_MASK) >> 1),
			                      ((Control & REQSEQ_MASK) >> 8),
			                      ((Control & FINAL_BIT) ? '1' : '0'),
			                      Msg.MData.DataCBChan.Length));
		}

		pExtF->AckSent = TRUE;

		l2cStopMONITORTimeout(pChan);
		l2cStopRETRANSMISSIONTimeout(pChan);
		l2cStartRETRANSMISSIONTimeout(pChan);

		Msg.MData.DataCBChan.Length = Length;  /* original length and offset in sent queue */
		Msg.MData.DataCBChan.Offset = Offset;
		osMessageSend(pExtF->SentQueueID, &Msg);
		pExtF->NextTxSeq = (pExtF->NextTxSeq + 1) & MODULO_MASK;
		pExtF->DIFF++;

		Count--;
	}
}

/****************************************************************************/
/*  Check local busy                                                        */
/****************************************************************************/

void l2cCheckLocalBusy(P_L2CAP_CHANNEL pChan)
{
	if (pChan->pExtF->LocalBusy && pChan->pExtF->RxCount <= 1)
	{
		pChan->pExtF->LocalBusy = FALSE;
		pChan->pExtF->RetransmitCount = 0;
		l2cSendSFrame(pChan, TYPE_RR_FRAME, POLL_BIT);
	}
}

/****************************************************************************/
/*  Buffer CallBack                                                         */
/****************************************************************************/
void l2cRxBufferCallBack(uint32_t Handle)
{
	P_L2CAP_CHANNEL_EXT pExtF;
	P_L2CAP_CHANNEL     pChan;
	BOOL                LocalBusy;

	pChan = (P_L2CAP_CHANNEL)Handle;
	if (pChan->magic == 0)                /* channel is closed ? */
	{
		return;
	}

	pExtF = pChan->pExtF;

	LocalBusy = pExtF->LocalBusy;

	if (LocalBusy && pExtF->RxCount <= 1)
	{
		if (pChan->Mode == L2CAP_MODE_ENHANCED_RETRANSMISSION)
		{
			/* if S Frame with P=1 is sent, sends RR frame if F=1 received */
			if (!pExtF->FinalBitExpected)
			{
				pExtF->LocalBusy              = FALSE;
				pChan->pExtF->RetransmitCount = 0;
				l2cSendSFrame(pChan, TYPE_RR_FRAME, POLL_BIT);
			}
		}
		else
		{
			pExtF->LocalBusy = FALSE;
		}
	}

	if (pExtF->RxCount)
	{
		pExtF->RxCount--;
	}

#if defined(L2CAP_ACK_TID)
	if (pExtF->AckTimerStarted && pExtF->RxCount == 0)
	l2cSendSFrame(pChan, TYPE_RR_FRAME, 0);
#endif
}

/****************************************************************************/
/*  Evaluate frame                                                          */
/****************************************************************************/

BOOL l2cEvaluateFrame(P_L2CAP_CHANNEL pChan, MESSAGE_P pMsg)
{
	P_L2CAP_CHANNEL_EXT pExtF;
	BOOL                ReturnValue = TRUE;
	uint8_t *              pFrame;
	uint16_t                Control;
	uint16_t                Length;

	if (pMsg->MData.DataCBChan.Length < (L2CAP_CONTROL_LENGTH + pChan->FcsLength))
	return(FALSE);                  /* data too short */

	pExtF = pChan->pExtF;

	pFrame = pMsg->MData.DataCBChan.BufferAddress + pMsg->MData.DataCBChan.Offset;

	pMsg->MData.DataCBChan.Length -= pChan->FcsLength;

	Length  = pMsg->MData.DataCBChan.Length;
	Control = CHAR2SHORT(pFrame);
	pFrame += L2CAP_CONTROL_LENGTH;

	pMsg->MData.DataCBChan.Length -= L2CAP_CONTROL_LENGTH;
	pMsg->MData.DataCBChan.Offset += L2CAP_CONTROL_LENGTH;

	pExtF->ReqSeq = (uint8_t)((Control & REQSEQ_MASK) >> 8);
	pExtF->SAR    = (Control & SAR_MASK);

	if ((Control & FORMAT_I_FRAME_MASK) == FORMAT_I_FRAME)
	{
	pExtF->Control   = Control;
	pExtF->FrameType = TYPE_I_FRAME;
	pExtF->TxSeq     = (uint8_t)((Control & TXSEQ_MASK) >> 1);

	switch (pExtF->SAR)
	{
	case L2CAP_SAR_UNSEGMENTED:
		if (pExtF->TotalLength != 0)
		{
			L2CAP_TRACE_PRINTF_1(L2CAP_TRACE_MASK_ERROR,
		                      "!!l2cEvaluateFrame: unsegmented SDU TLen=%d", pExtF->TotalLength);
		}
		break;
	case L2CAP_SAR_START:
		pExtF->TotalLength = CHAR2SHORT(pFrame);

		pFrame                        += L2CAP_SDU_LENGTH_FIELD_SIZE;
		pMsg->MData.DataCBChan.Length -= L2CAP_SDU_LENGTH_FIELD_SIZE;
		pMsg->MData.DataCBChan.Offset += L2CAP_SDU_LENGTH_FIELD_SIZE;
	case L2CAP_SAR_CONTINUATION:
		if (pExtF->TotalLength < pMsg->MData.DataCBChan.Length)
		{
			L2CAP_TRACE_PRINTF_2(L2CAP_TRACE_MASK_ERROR,
			                      "!!l2cEvaluateFrame: TLen=%d rxlen=%d",
			                      pExtF->TotalLength, pMsg->MData.DataCBChan.Length);
		}
		break;
	case L2CAP_SAR_END:
		if (pExtF->TotalLength != pMsg->MData.DataCBChan.Length)
		{
			L2CAP_TRACE_PRINTF_2(L2CAP_TRACE_MASK_ERROR,
		                      "!!l2cEvaluateFrame: TLen=%d rxlen=%d",
		                      pExtF->TotalLength, pMsg->MData.DataCBChan.Length);
		}
		break;
	default:
		break;
	}

	L2CAP_TRACE_VERBOSITY_3(L2CAP_TRACE_PRINTF_5(L2CAP_TRACE_MASK_TRACE,
	                        "L2C-> l2cEvaluateFrame: I frame TxSeq=%2.2x ReqSeq=%2.2x SAR=%1.1x TLen=%4.4x F=%c",
	                        pExtF->TxSeq,
	                        pExtF->ReqSeq,
	                        (uint8_t)(pExtF->SAR >> 14),
	                        pExtF->TotalLength,
	                        Control & FINAL_BIT ? '1' : '0'));

#if (X_L2C_DISCARD_IFRAME)
		if (pChan->Mode == L2CAP_MODE_ENHANCED_RETRANSMISSION)
		{
			L2CAP_TRACE_VERBOSITY_3(L2CAP_TRACE_PRINTF_0(L2CAP_TRACE_MASK_TRACE,
			                      "l2cEvaluateFrame: TestVersion (X_L2C_DISCARD_IFRAME)"));
			ReturnValue = FALSE;
		}
#endif  /* (X_L2C_DISCARD_IFRAME) */
	}
	else
	{
		if ((Control & FORMAT_S_FRAME_MASK) == FORMAT_S_FRAME)  /* S frame */
		{
			switch (Control & S_FRAME_MASK)
			{
			case TYPE_RR_FRAME:
			case TYPE_REJ_FRAME:
			case TYPE_RNR_FRAME:
			case TYPE_SREJ_FRAME:
				pExtF->Control   = Control;
				pExtF->FrameType = Control & S_FRAME_MASK;

				L2CAP_TRACE_VERBOSITY_3(L2CAP_TRACE_PRINTF_4(L2CAP_TRACE_MASK_TRACE,
				                            "L2C-> l2cEvaluateFrame: %s frame ReqSeq=%2.2x SAR=%1.1x",
				                            l2cGetSFrameName(pExtF->FrameType & S_FRAME_MASK),
				                            pExtF->ReqSeq,
				                            (uint8_t)(pExtF->SAR >> 14),
				                            0));

				if (Length != L2CAP_CONTROL_LENGTH)
				{
					L2CAP_TRACE_PRINTF_1(L2CAP_TRACE_MASK_ERROR,
					                    "!!l2cEvaluateFrame: S frame too long %d", Length);
#if (X_L2C_DISCONNECT_LINK_ON__INV_FRAME)
					l2cCloseChannel(pChan, L2CAP_ERR_EM_FRAME_INV);
#else
					assert(FALSE);
#endif
					ReturnValue = FALSE;
				}
				break;

		    default:
				L2CAP_TRACE_PRINTF_1(L2CAP_TRACE_MASK_ERROR,
				                      "!!l2cEvaluateFrame: unknown S frame %4.4x", Control);
#if (X_L2C_DISCONNECT_LINK_ON__INV_FRAME)
				l2cCloseChannel(pChan, L2CAP_ERR_EM_FRAME_INV);
#else
				assert(FALSE);
#endif
				ReturnValue = FALSE;
				break;
			}
		}
		else
		{
			L2CAP_TRACE_PRINTF_1(L2CAP_TRACE_MASK_ERROR,
			                  "!!l2cEvaluateFrame unknown frame %4.4x", Control);
#if (X_L2C_DISCONNECT_LINK_ON__INV_FRAME)
			l2cCloseChannel(pChan, L2CAP_ERR_EM_FRAME_INV);
#else
			assert(FALSE);
#endif
			ReturnValue = FALSE;               /* unknown frame */
		}
	}

	return(ReturnValue);
}

/****************************************************************************/
/*  discard acknowledged I frames                                           */
/****************************************************************************/

void l2cDiscardAcknowledgedIFrame(P_L2CAP_CHANNEL pChan)
{
	MESSAGE_T Msg;

	if (osMessageReceive(pChan->pExtF->SentQueueID, &Msg) == 0)
	{
		l2cDiscardTxMessage(&Msg);
	}
}

/****************************************************************************/
/*  check ReqSeq: NextTxSeq - ReqSeq <= DIFF) ?                             */
/*  in:     -                                                               */
/*  out:    errorr == 0 (FALSE)                                             */
/*          ok.    != 0 (TRUE)                                              */
/****************************************************************************/

BOOL l2cCheckReqSeq(P_L2CAP_CHANNEL pChan)
{
	P_L2CAP_CHANNEL_EXT pExtF = pChan->pExtF;

	if (((pExtF->NextTxSeq - pExtF->ReqSeq) & MODULO_MASK) <= pExtF->DIFF)
	{
		return(TRUE);
	}
	else
	{
		return(FALSE);
	}
}

/****************************************************************************/
/*  calculate DIFF (NextTxSeq - ReqSeq)                                     */
/****************************************************************************/

void l2cCalculateDIFF(P_L2CAP_CHANNEL pChan)
{
	P_L2CAP_CHANNEL_EXT pExtF = pChan->pExtF;

	pExtF->DIFF = (pExtF->NextTxSeq - pExtF->ReqSeq) & MODULO_MASK;
}

/****************************************************************************/
/*  update ExpectedAckSeq                                                   */
/****************************************************************************/

void l2cUpdateExpectedAckSeq(P_L2CAP_CHANNEL pChan)
{
	P_L2CAP_CHANNEL_EXT pExtF = pChan->pExtF;
	uint16_t                AckCount = 0;

	l2cCalculateDIFF(pChan);

	while (pExtF->ExpectedAckSeq != pExtF->ReqSeq)
	{
		l2cDiscardAcknowledgedIFrame(pChan);
		pExtF->ExpectedAckSeq = (pExtF->ExpectedAckSeq + 1) & MODULO_MASK;
		AckCount++;
	}

	if (pExtF->ReqSeq == pExtF->NextTxSeq)      /* all I frame acknowledged   */
	{
		pExtF->RetransmitCountIFrame = 0;

		l2cStopRETRANSMISSIONTimeout(pChan);
	}
	else
	{
		if (pChan->Mode == L2CAP_MODE_ENHANCED_RETRANSMISSION)
		{
			if (AckCount)
			{
				l2cStopRETRANSMISSIONTimeout(pChan);
			}
			if (!pExtF->RemoteBusy)
			{
				l2cStartRETRANSMISSIONTimeout(pChan);
			}
		}
	}
}

/****************************************************************************/
/*  Check Rx queue                                                          */
/****************************************************************************/

void l2cCheckRxQueue(P_L2CAP_CHANNEL pChan)
{
	P_L2CAP_CHANNEL_EXT pExtF = pChan->pExtF;
	uint16_t                Count;

	osMessageQueueElementCountGet(pExtF->RxQueueID, &Count);   /* get I frame count    */
	while (Count)
	{
		MESSAGE_T Msg;

		if (osMessageReceive(pExtF->RxQueueID, &Msg) == 0)
		{
			uint8_t *    pFrame;
			uint16_t      Control;

			pFrame  = Msg.MData.DataCBChan.BufferAddress + Msg.MData.DataCBChan.Offset;
			pFrame -= L2CAP_CONTROL_LENGTH;

			Control = CHAR2SHORT(pFrame);

			Control &=~REQSEQ_MASK;                /* clear ReqSeq */
			Control |= (pExtF->ReqSeq << 8);
			pExtF->Control   = Control;
			pExtF->SAR       = (Control & SAR_MASK);
			pExtF->TxSeq     = (uint8_t)((Control & TXSEQ_MASK) >> 1);
			pExtF->FrameType = TYPE_I_FRAME;

			l2cHandleIFrame(pChan, &Msg);
		}
		else
		{
			L2CAP_TRACE_PRINTF_1(L2CAP_TRACE_MASK_ERROR,
			                      "!!l2cCheckRxQueue: internal error - element count (%d) mismatch", Count);
			assert(FALSE);
		}
		Count--;
	}
}

/****************************************************************************/
/*  handle I Frame                                                          */
/****************************************************************************/

void l2cHandleIFrame(P_L2CAP_CHANNEL pChan, MESSAGE_P pMsg)
{
	BOOL                ReqSeqOK;
	P_L2CAP_CHANNEL_EXT pExtF = pChan->pExtF;

	L2CAP_TRACE_VERBOSITY_3(L2CAP_TRACE_PRINTF_4(L2CAP_TRACE_MASK_TRACE,
	                      "L2C-> l2cHandleIFrame() TxSeq=%2.2x (expected=%2.2x) ReqSeg=%2.2x F=%c",
	                      pExtF->TxSeq,
	                      pExtF->ExpectedTxSeq,
	                      pExtF->ReqSeq,
	                      (pExtF->Control & FINAL_BIT) ? '1':'0'));

	if (pExtF->LocalBusy && pExtF->RxCount >= L2CAP_MAX_RX_IFRAME)
	{
		L2CAP_TRACE_VERBOSITY_3(L2CAP_TRACE_PRINTF_0(L2CAP_TRACE_MASK_TRACE,
		                        "L2C-> l2cHandleIFrame() local busy -> I Frame ignored"));
		return;
	}

	if (pChan->Mode == L2CAP_MODE_STREAMING)
	{
		ReqSeqOK = TRUE;                          /* ignore ReqSeq */

		if (pExtF->TxSeq != pExtF->ExpectedTxSeq)   /* TxSeq != ExpectedTxSeq */
		{
			pExtF->MissingFrameCount += ((pExtF->TxSeq - pExtF->ExpectedTxSeq) & MODULO_MASK);

			pExtF->ExpectedTxSeq = pExtF->TxSeq;
		}
	}
	else
	{
		ReqSeqOK = l2cCheckReqSeq(pChan);
	}

	if (pExtF->TxSeq == pExtF->ExpectedTxSeq)   /* TxSeq == ExpectedTxSeq     */
	{
		BOOL TxSeqSequenceError = pExtF->TxSeqSequenceError;

		pExtF->TxSeqSequenceError = FALSE;        /* expected TxSeq received    */

		pExtF->ExpectedTxSeq = (pExtF->ExpectedTxSeq + 1) & MODULO_MASK;

		if (pChan->Mode != L2CAP_MODE_STREAMING)
		{
			l2cUpdateExpectedAckSeq(pChan);
		}

		if (ReqSeqOK == TRUE)
		{
			osBufferCallBackSet(pMsg->MData.DataCBChan.BufferAddress, l2cRxBufferCallBack, (uint32_t)pChan);

			if (pExtF->SAR == L2CAP_SAR_UNSEGMENTED)
			{
				pMsg->MData.DataCBChan.TotalLength = pMsg->MData.DataCBChan.Length;
			}
			else
			{
				pMsg->MData.DataCBChan.TotalLength = pExtF->TotalLength;
				pExtF->TotalLength -= pMsg->MData.DataCBChan.Length;
			}
			switch (pExtF->SAR)
			{
			case L2CAP_SAR_START:
				pMsg->MData.DataCBChan.Flag |= DATA_CB_BLOCK_FIRST | DATA_CB_MORE;
				break;
			case L2CAP_SAR_CONTINUATION:
				pMsg->MData.DataCBChan.Flag |= DATA_CB_BLOCK_MIDDLE | DATA_CB_MORE;
				break;
		    case L2CAP_SAR_END:
				pMsg->MData.DataCBChan.Flag |= DATA_CB_BLOCK_LAST;
				break;
		    case L2CAP_SAR_UNSEGMENTED:
				pMsg->MData.DataCBChan.Flag &= ~(DATA_CB_BLOCK_FIRST  |                                            DATA_CB_BLOCK_MIDDLE |                                            DATA_CB_BLOCK_LAST);
		    default:
				break;
			}

			/* F_BT_L2C_ENHANCED_CONFORMANCE */
			{
				if ((pChan->Mode == L2CAP_MODE_STREAMING &&  /* Discard I frame UL has too much data buffers */
				 pExtF->RxCount > BT_L2C_MAX_RX_BUFFER_COUNT)   ||
				(pExtF->MissingFrameCount &&            /* only Streaming: Discard not a start segment or unsegmented I frame */
				 pMsg->MData.DataCBChan.Flag & (DATA_CB_BLOCK_MIDDLE | DATA_CB_BLOCK_LAST)))
				{
					pExtF->MissingFrameCount++;
				}
				else
			    {
					osMessageSend(pChan->usQueueID, pMsg);
					pMsg->MData.DataCBChan.Flag &=~DATA_CB_RELEASE;
					pExtF->RxCount++;
		    	}
		  	}

			pExtF->AckSent = FALSE;

			if (pChan->Mode == L2CAP_MODE_ENHANCED_RETRANSMISSION)
			{
				BOOL LocalBusy = pExtF->LocalBusy;

				pExtF->RxIFrameCount++;

				if (pExtF->FinalBitExpected && pExtF->Control & FINAL_BIT)
				{
					pExtF->FinalBitExpected = FALSE;
					l2cStopMONITORTimeout(pChan);
					l2cCheckLocalBusy(pChan);

					if (TxSeqSequenceError == FALSE)  /* REJ sent -> don't send unacknowledged IFrame */
					{
						l2cSendUnacknowledgedIFrame(pChan);
					}
				}

				if (pExtF->RxCount >= L2CAP_MAX_RX_IFRAME)
				{
					pExtF->LocalBusy = TRUE;
				}

				if (pExtF->LocalBusy && !LocalBusy)
				{
					if (!pExtF->FinalBitExpected)
					{
						pExtF->RetransmitCount = 0;
						l2cSendSFrame(pChan, TYPE_RNR_FRAME, POLL_BIT);
					}
				}
				else
				{
					{
#if defined(L2CAP_ACK_TID)
						if (pExtF->RxIFrameMaxCount > pExtF->RxIFrameCount)
						{
							if (pExtF->AckTime)
							{
								l2cStartACKTimeout(pChan);
							}
							else
							{
								if (!pExtF->FinalBitExpected)
								{
									l2cSendSFrame(pChan, TYPE_RR_FRAME, 0);
								}
							}
						}
						else
#endif
						{
							if (!pExtF->FinalBitExpected)
							{
								l2cSendSFrame(pChan, TYPE_RR_FRAME, 0);
							}
						}
				  	}
				}

				if (pExtF->SREJMode && TxSeqSequenceError)
				{
					l2cCheckRxQueue(pChan);

					if (pExtF->SendFinalBit) /* response to send ? */
					{
						pExtF->SendFinalBit = FALSE;
						l2cSendSFrame(pChan, TYPE_RR_FRAME, FINAL_BIT);
					}
				}
			}
		}
	}
	else
	{                                     /* unexpected TxSeq                 */
		l2cUpdateExpectedAckSeq(pChan);

		if (pExtF->TxSeqSequenceError == FALSE)
		{
			pExtF->RetransmitCount = 0;
			if (pExtF->SREJMode)
			{
				/* F_BT_L2C_ENHANCED_CONFORMANCE */
				{
					if (pExtF->FinalBitExpected && pExtF->SREJMode)
					pExtF->SendREJ = TRUE;          /* send REJ later */
					else
					l2cSendSFrame(pChan, TYPE_REJ_FRAME, POLL_BIT);
				}
			}
			else
			{
				/* F_BT_L2C_ENHANCED_CONFORMANCE */
				l2cSendSFrame(pChan, TYPE_REJ_FRAME, X_L2C_REJ_POLL_BIT);
			}

#if defined(L2CAP_ACK_TID)
			if (pExtF->AckTimerStarted)
			{
				l2cStopACKTimeout(pChan);
			}
#endif

		  	pExtF->TxSeqSequenceError = TRUE;
		}
		else
		{
			if (ReqSeqOK == FALSE && pChan->Mode == L2CAP_MODE_ENHANCED_RETRANSMISSION)
			{
				l2cCloseChannel(pChan, L2CAP_ERR_EM_RXTXSEQ_INV);
				return;
			}
		}

		if (pExtF->SREJMode)
		{
			osMessageSend(pExtF->RxQueueID, pMsg);
			pMsg->MData.DataCBChan.Flag &=~DATA_CB_RELEASE;
		}
	}
	l2cSendIFrame(pChan);           /* try to send I frame */
}

/****************************************************************************/
/*  handle RR Frame                                                         */
/****************************************************************************/

void l2cHandleRRFrame(P_L2CAP_CHANNEL pChan)
{
	L2CAP_TRACE_VERBOSITY_3(L2CAP_TRACE_PRINTF_3(L2CAP_TRACE_MASK_TRACE,
                          "L2C-> l2cHandleRRFrame() ReqSeg=%2.2x P=%c F=%c",
                          pChan->pExtF->ReqSeq,
                          (pChan->pExtF->Control & POLL_BIT)? '1':'0',
                          (pChan->pExtF->Control & FINAL_BIT)? '1':'0'));

	if (pChan->Mode == L2CAP_MODE_STREAMING)
	{
		return;
	}

	if (l2cCheckReqSeq(pChan) == FALSE)
	{
		l2cCloseChannel(pChan, L2CAP_ERR_EM_RXSEQ_INV);
		return;
	}

	l2cUpdateExpectedAckSeq(pChan);

	{
		P_L2CAP_CHANNEL_EXT pExtF      = pChan->pExtF;

		pExtF->RemoteBusy = FALSE;
		if (pExtF->Control & POLL_BIT)
		{
			/* F_BT_L2C_ENHANCED_CONFORMANCE */
			{
				if (pExtF->TxSeqSequenceError && pExtF->SREJMode)   /* SREJ sent ? */
				{
					/* F_BT_L2C_ENHANCED_CONFORMANCE */
					{
					/* Don't send second SREJ but send F=1 later */
						pExtF->SendFinalBit = TRUE;
					}
				}
			    else
			    {
					l2cSendSFrame(pChan, TYPE_RR_FRAME, FINAL_BIT);
					pExtF->Control &=~POLL_BIT;

			/* F_BT_L2C_ENHANCED_CONFORMANCE */
			    }
		/* F_BT_L2C_ENHANCED_CONFORMANCE */
		  }
	}

    if (pExtF->FinalBitExpected && pExtF->Control & FINAL_BIT)
    {
		pExtF->FinalBitExpected = FALSE;
		l2cStopMONITORTimeout(pChan);

		if (pExtF->DIFF &&
		  (pExtF->RetransmitCount + 1) >= pChan->ConfParaRemote.flowControl.maxTransmit)
		{
			l2cCloseChannel(pChan, L2CAP_ERR_RXSEQ_INV);
			return;
		}
		else
		{
			l2cCheckLocalBusy(pChan);

			if (pExtF->TxSeqSequenceError)
			{
				l2cStartMONITORTimeout(pChan);
			}
			else
			{
				if (!pExtF->FinalBitExpected)  /* flag can be set in l2cCheckLocalBusy() */
				{
					l2cSendUnacknowledgedIFrame(pChan);
				}
			}

			if ((pExtF->SendREJ && pExtF->SREJMode) && !pExtF->FinalBitExpected)
			{
				pExtF->SendREJ = FALSE;
				l2cSendSFrame(pChan, TYPE_REJ_FRAME, POLL_BIT);
			}
			}

		/* F_BT_L2C_ENHANCED_CONFORMANCE */
		}

	/* F_BT_L2C_ENHANCED_CONFORMANCE */
	}
	l2cSendIFrame(pChan);           /* try to send I frame */
}

/****************************************************************************/
/*  handle RNR Frame                                                        */
/****************************************************************************/

void l2cHandleRNRFrame(P_L2CAP_CHANNEL pChan)
{
  L2CAP_TRACE_VERBOSITY_3(L2CAP_TRACE_PRINTF_3(L2CAP_TRACE_MASK_TRACE,
                          "L2C-> l2cHandleRNRFrame() ReqSeg=%2.2x P=%c F=%c",
                          pChan->pExtF->ReqSeq,
                          (pChan->pExtF->Control & POLL_BIT)? '1':'0',
                          (pChan->pExtF->Control & FINAL_BIT)? '1':'0'));

  if (pChan->Mode == L2CAP_MODE_ENHANCED_RETRANSMISSION)
  {
    P_L2CAP_CHANNEL_EXT pExtF = pChan->pExtF;

    if (l2cCheckReqSeq(pChan) == FALSE)
    {
      l2cCloseChannel(pChan, L2CAP_ERR_EM_RXSEQ_INV);
      return;
    }

    pExtF->RemoteBusy = TRUE;
    if (pExtF->Control & POLL_BIT)
    {
      if (pExtF->TxSeqSequenceError && pExtF->SREJMode)   /* SREJ sent ? */
        pExtF->SendFinalBit = TRUE;
      else
        l2cSendSFrame(pChan, TYPE_RR_FRAME, FINAL_BIT);
    }

    if (pExtF->FinalBitExpected && pExtF->Control & FINAL_BIT)
    {
      pExtF->FinalBitExpected = FALSE;
      l2cStopMONITORTimeout(pChan);

      if (pExtF->DIFF &&
          (pExtF->RetransmitCount + 1) >= pChan->ConfParaRemote.flowControl.maxTransmit)
      {
        l2cCloseChannel(pChan, L2CAP_ERR_RXSEQ_INV);
        return;
      }
      else
      {
        l2cCheckLocalBusy(pChan);
      }

      if (pExtF->TxSeqSequenceError)
        l2cStartMONITORTimeout(pChan);
    }

    l2cUpdateExpectedAckSeq(pChan);

    if ((pExtF->SendREJ && pExtF->SREJMode) && !pExtF->FinalBitExpected)
    {
      pExtF->SendREJ = FALSE;
      l2cSendSFrame(pChan, TYPE_REJ_FRAME, POLL_BIT);
    }
  }

/* (F_BT_L2C_ENHANCED_CONFORMANCE) */
}

/****************************************************************************/
/*  handle REJ Frame                                                          */
/****************************************************************************/

void l2cHandleREJFrame(P_L2CAP_CHANNEL pChan)
{
  P_L2CAP_CHANNEL_EXT pExtF         = pChan->pExtF;
#if (L2CAP_TRACE_VERBOSITY_COUNT >= 3)
  uint16_t                Count;

  osMessageQueueElementCountGet(pExtF->SentQueueID, &Count);
  L2CAP_TRACE_VERBOSITY_3(L2CAP_TRACE_PRINTF_4(L2CAP_TRACE_MASK_TRACE,
                          "L2C-> l2cHandleREJFrame() ReqSeg=%2.2x P=%c F=%c Count=%x",
                          pExtF->ReqSeq,
                          (pExtF->Control & POLL_BIT)? '1':'0',
                          (pExtF->Control & FINAL_BIT)? '1':'0',
                          Count));
#endif  /* (L2CAP_TRACE_VERBOSITY_COUNT >= 3) */

  if (pChan->Mode == L2CAP_MODE_STREAMING)
    return;

  if (l2cCheckReqSeq(pChan) == FALSE)
  {
    l2cCloseChannel(pChan, L2CAP_ERR_EM_RXSEQ_INV);
     return;
  }

  pExtF->RemoteBusy = FALSE;

  if ((pExtF->FrameType == TYPE_REJ_FRAME) ||
      (pExtF->FrameType == TYPE_SREJ_FRAME && pExtF->Control & POLL_BIT))
  {
/* F_BT_L2C_ENHANCED_CONFORMANCE */
      l2cUpdateExpectedAckSeq(pChan);
  }
/* F_BT_L2C_ENHANCED_CONFORMANCE */
    l2cSendUnacknowledgedIFrame(pChan);
  l2cSendIFrame(pChan);              /* try to send I frame */
}

/****************************************************************************/
/*  handle downstream data                                                  */
/****************************************************************************/

void l2cHandleDownstreamData(P_L2CAP_CHANNEL pChan, MESSAGE_P pMsg)
{
  uint16_t   minOffset;
  uint16_t   BufferLength;

  minOffset = L2CAP_ERTM_OFFSET + L2CAP_CONTROL_LENGTH;

  if (pMsg->MData.DataCBChan.Flag & DATA_CB_BLOCK_FIRST)
    minOffset += L2CAP_SDU_LENGTH_FIELD_SIZE;

  if (pMsg->MData.DataCBChan.Offset < minOffset)
    assert(FALSE);

  osBufferLength(pMsg->MData.DataCBChan.BufferAddress, &BufferLength);
  if ((BufferLength - pMsg->MData.DataCBChan.Offset - pMsg->MData.DataCBChan.Length) < pChan->FcsLength)
    assert(FALSE);

  if (pMsg->MData.DataCBChan.Flag & DATA_CB_BLOCK_FIRST)
  {
    pMsg->MData.DataCBChan.Offset -= L2CAP_SDU_LENGTH_FIELD_SIZE;
    pMsg->MData.DataCBChan.Length += L2CAP_SDU_LENGTH_FIELD_SIZE;

    SHORT2CHAR(pMsg->MData.DataCBChan.BufferAddress + pMsg->MData.DataCBChan.Offset,
               pMsg->MData.DataCBChan.TotalLength);      /* write 16 bits SDU length */
  }

  osMessageSend(pChan->pExtF->TxQueueID, pMsg);
  l2cSendIFrame(pChan);           /* try to send I frame */
}

/****************************************************************************/
/*  handle upstream packet                                                  */
/****************************************************************************/

void l2cHandleUpstreamPacket(P_L2CAP_CHANNEL pChan, MESSAGE_P pMsg)
{
  if (l2cEvaluateFrame(pChan, pMsg))
  {
    switch (pChan->pExtF->FrameType)
    {
      case TYPE_I_FRAME:
        l2cHandleIFrame(pChan, pMsg);
        break;

      case TYPE_RR_FRAME:
        l2cHandleRRFrame(pChan);
        break;

      case TYPE_REJ_FRAME:
      case TYPE_SREJ_FRAME:
        l2cHandleREJFrame(pChan);
        break;

      case TYPE_RNR_FRAME:
        l2cHandleRNRFrame(pChan);
        break;

      default:
        break;
    }
  }

  if (pMsg->MData.DataCBChan.Flag & DATA_CB_RELEASE)
    osBufferRelease((PVOID)pMsg->MData.DataCBChan.BufferAddress);
}

/****************************************************************************/
/*  State is opened                                                         */
/****************************************************************************/

void l2cStateIsOpened(P_L2CAP_CHANNEL pChan)
{
  P_L2CAP_CHANNEL_EXT pExtF = pChan->pExtF;

  if (pChan->ConfParaLocal.fcs != pChan->ConfParaRemote.fcs)
    pChan->ConfParaLocal.fcs = 1;    /* use default: 16-bit FCS */

  pChan->ConfParaRemote.fcs = pChan->ConfParaLocal.fcs;
  pChan->Fcs                = pChan->ConfParaLocal.fcs;

  if (pChan->Fcs)
    pChan->FcsLength = L2CAP_FCS_LENGTH;
  else
    pChan->FcsLength = 0;

  pChan->ExtFLength = L2CAP_CONTROL_LENGTH + pChan->FcsLength;

  if (pChan->Mode == L2CAP_MODE_STREAMING)
  {
    pChan->ConfParaRemote.flowControl.txWindowSize = 0xFF;     /* no Ack in streaming mode */
  }
  else
  {
    pExtF->SREJMode = X_L2C_SREJ_MODE;
/* F_BT_L2C_ENHANCED_CONFORMANCE */

#if defined(L2CAP_ACK_TID)
    pExtF->AckTime = pChan->ConfParaRemote.flowControl.retransmissionTimeout / TIMER_TICK / 2;
    if (pExtF->AckTime < 30)  /* < 300 ms ? */
    {
      pExtF->AckTime          = 0;
      pExtF->RxIFrameMaxCount = 1;
    }
    else
    {
      pExtF->RxIFrameMaxCount = (pChan->RespTxWindowSize <= 2) ? 1 : (pChan->RespTxWindowSize >> 1) + 1;
    }
#endif
/* F_BT_L2C_ENHANCED_CONFORMANCE */
  }

  pExtF->RxIFrameCount = 0;
}

/****************************************************************************/
/*  State is closeed                                                        */
/****************************************************************************/

void l2cStateIsClosed(P_L2CAP_CHANNEL pChan)
{
  MESSAGE_T msg;

  l2cStopMONITORTimeout(pChan);
  l2cStopRETRANSMISSIONTimeout(pChan);
  l2cStopACKTimeout(pChan);

  /* empty the transmit queue */
  while (osMessageReceive(pChan->pExtF->TxQueueID, &msg) == 0)
  {
    osBufferRelease(msg.MData.DataCB.BufferAddress);
    L2CAP_TRACE_PRINTF_0(L2CAP_TRACE_MASK_TRACE,"l2cStateIsClosed: discard TX message");
  }
  /* empty the sent queue */
  while (osMessageReceive(pChan->pExtF->SentQueueID, &msg) == 0)
  {
    osBufferRelease(msg.MData.DataCB.BufferAddress);
    L2CAP_TRACE_PRINTF_0(L2CAP_TRACE_MASK_TRACE,"l2cStateIsClosed: discard SENT message");
  }
  /* empty the rx queue */
  while (osMessageReceive(pChan->pExtF->RxQueueID, &msg) == 0)
  {
    osBufferRelease(msg.MData.DataCB.BufferAddress);
    L2CAP_TRACE_PRINTF_0(L2CAP_TRACE_MASK_TRACE,"l2cStateIsClosed: discard RX message");
  }
}

/****************************************************************************/
/*  Monitor timer expired                                                   */
/****************************************************************************/

void l2cHandleTIMER_EXPIREDMonitor(P_L2CAP_CHANNEL pChan)
{
  P_L2CAP_CHANNEL_EXT pExtF = pChan->pExtF;

  if (pExtF->MonitorTimerStarted == TRUE)
  {
    pExtF->MonitorTimerStarted = FALSE;

 /* F_BT_L2C_ENHANCED_CONFORMANCE */
    {
      if (pExtF->FinalBitExpected)
        pExtF->RetransmitCount++;

      if (pChan->ConfParaRemote.flowControl.maxTransmit == 0 || pExtF->RetransmitCount < pChan->ConfParaRemote.flowControl.maxTransmit)
      {
        if (pExtF->TxSeqSequenceError)
          l2cSendSFrame(pChan, TYPE_REJ_FRAME, X_L2C_REJ_POLL_BIT);
        else
          l2cSendSFrame(pChan, TYPE_RR_FRAME, POLL_BIT);
      }
      else
      {
        l2cCloseChannel(pChan, L2CAP_ERR_EM_NO_RESPONSE);
        return;
      }
      l2cStartMONITORTimeout(pChan);
    }
  }
}

/****************************************************************************/
/*  Retransmission timer expired                                            */
/****************************************************************************/

void l2cHandleTIMER_EXPIREDRetransmission(P_L2CAP_CHANNEL pChan)
{
  if (pChan->pExtF->RetransmissionTimerStarted == TRUE)
  {
    P_L2CAP_CHANNEL_EXT pExtF = pChan->pExtF;

    pChan->pExtF->RetransmissionTimerStarted = FALSE;

    if (pExtF->FinalBitExpected)
      pExtF->RetransmitCountIFrame++;

    if (pChan->ConfParaRemote.flowControl.maxTransmit == 0 || pExtF->RetransmitCountIFrame < pChan->ConfParaRemote.flowControl.maxTransmit)
    {
      pExtF->RetransmitCount = 0;
      l2cSendSFrame(pChan, TYPE_RR_FRAME, POLL_BIT);
    }
    else
    {
      l2cCloseChannel(pChan, L2CAP_ERR_EM_NO_RESPONSE);
    }
  }
}

#if defined(L2CAP_ACK_TID)
/****************************************************************************/
/*  ACK timer expired                                                       */
/****************************************************************************/

void l2cHandleTIMER_EXPIREDAck(P_L2CAP_CHANNEL pChan)
{
	if (pChan->pExtF->AckTimerStarted == TRUE)
	{
		pChan->pExtF->AckTimerStarted = FALSE;
		if (pChan->pExtF->AckSent == FALSE)
		{
	  		l2cSendSFrame(pChan, TYPE_RR_FRAME, 0);
		}
	}
}
#endif
 
#endif // F_BT_L2C_ENHANCED_FEATURE_SUPPORT //champion
