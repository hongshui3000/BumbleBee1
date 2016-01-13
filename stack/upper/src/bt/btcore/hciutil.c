/**
********************************************************************************************************
Copyright (c) 2015, Realtek Semiconductor Corporation. All rights reserved.
*********************************************************************************************************
* @file       hciutil.c
* @brief     handle hci related info(command, event, data etc.)
* @details   
*
* @author  	gordon
* @date      	2015-06-26
* @version	v0.1
*/

#include <flags.h>
#include <os_message.h>
#include <os_pool.h>
#include <message.h>
#include <btcommon.h>
#include <hci.h>
#include <hci_llif.h>
#include <hci_api.h>
#include <btsm_api.h>
#include <hci_code.h>
#include <upper_stack_global.h>

#define TRACE_MODULE_ID     			MID_BT_HCI


/**
* @brief		Generate Intra-Layer Error Code from Peer-to-Perr Error Code 
*
* @param	code
*
* @return	Error Code
*
*/
uint16_t hciStatus(uint8_t code)
{
    if (code == 0)
	{
    	return 0;
	}
	
    return (uint16_t) (HCI_ERR | code);
}

/**
* @brief  return allocated des or allocate new descriptor
*
* @param  bd: 
* @param  handle:
* @param  conType
* @param  bdType
*
* @return  
*
*/
ThciHandleDesc * hciNewHandle(uint8_t * bd, uint16_t handle, uint8_t conType, uint8_t bdType)
{
    ThciHandleDesc * lh;
    ThciHandleDesc * fh = NULL; 
    int i;

    HCI_TRACE_PRINTF_2(HCI_TRACE_MASK_TRACE, "HCI: new handle bd %s hdl %x", TRACE_BDADDR1(HCI_TRACE_MASK_TRACE, bd), handle);

    for (i = 0; i < otp_str_data.gEfuse_UpperStack_s.num_link_don; i++)
    {
        lh = &pHCI->phciDescDon[i];
        if (lh->used)
        {
            if (lh->handle == handle)
            {
                HCI_TRACE_PRINTF_1(HCI_TRACE_MASK_TRACE, "hciNewHandle: handle already existed %x", handle);

                lh->conType = conType;
                memcpy(lh->bd, bd, BD_ADDR_SIZE);
                return lh;
            }
        }
        else
        {
            fh = lh;
            goto TagEND;
        }
    }		
	for (i = 0; i < otp_str_data.gEfuse_UpperStack_s.num_link_doff; i++)
    {
        lh = &pHCI->phciDescDoff[i];
        if (lh->used)
        {
            if (lh->handle == handle)
            {
                HCI_TRACE_PRINTF_1(HCI_TRACE_MASK_TRACE, "hciNewHandle: handle already existed %x", handle);

                lh->conType = conType;
                memcpy(lh->bd, bd, BD_ADDR_SIZE);
                return lh;
            }
        }
        else
        {
            fh = lh;
            goto TagEND;
        }
    }
    fh = NULL;
    
TagEND:

    if (fh)
    {
        fh->used            = TRUE;
        fh->handle          = handle;
        fh->conType         = conType;
        fh->bdType          = bdType;
        memcpy(fh->bd, bd, BD_ADDR_SIZE);
        fh->dsAclCount      = 0;
        fh->leDsAclCount    = 0;
        fh->currentRole     = HCI_ROLE_UNKNOWN;
        
#if F_BT_BREDR_SUPPORT        
        fh->switchRoleTries = SWITCH_ROLE_TRIES;
        fh->piconetType     = pHCI->piconetType;
        fh->link_policy     = pHCI->link_policy;
        fh->supervisionTimeout = pHCI->link_supervision_timeout;

        fh->nModeState      = HCI_MODE_STATE_ACTIVE;

        if (fh->conType == BT_CONNECTION_TYPE_BR_ACL)
        {
            pHCI->aclHandleCnt++;

			btsmHandleHciNewHandle(fh->bdType, fh->bd);

            /* check local & remote side for SSP support */
            if (pHCI->hciVersion > HCI_VERSION_20)
            {
                pHCI->transactionHandle = handle;
                hciCommandWordParameter(HCI_READ_REMOTE_SUPPORTED_FEATURES, fh->handle);
				hciLaterEntry();
            }
            else
            {
				btsmHandleHciAclConnection(fh->bd, ACL_CONNECTION_NONSSP);
            }

        }
        else if (fh->conType == BT_CONNECTION_TYPE_LE)
        {
#endif
            pHCI->leAclHandleCnt++;
            hciConfigureNoConnectionsInd(NEW_LE_CONNECTION, fh->bd, fh->bdType);
#if F_BT_BREDR_SUPPORT
        }
#endif
    }
    return fh;
}

/**
* @brief  remove hci handle
* 
* @param  handle:  hci handle
*
* @return 
*
*/
void hciRemoveHandle(uint16_t handle)
{
	ThciHandleDesc * hd;
	MESSAGE_T msg;

	HCI_TRACE_PRINTF_1(HCI_TRACE_MASK_TRACE,"HCI: remove handle  hdl %x", handle);
	hd = hciFindHandle(pHCI, handle);
	if (!hd)
	{
    	return;
	}

	/* read and release all waiting messages */
	while (osMessageReceive(hd->dsAclQueueID, &msg) == 0)
	{
	    if (msg.MData.DataCB.Flag & DATA_CB_RELEASE)
    	{
        	osBufferRelease(msg.MData.DataCB.BufferAddress);
    	}
	    HCI_TRACE_PRINTF_1(HCI_TRACE_MASK_TRACE,"hciRemoveHandle: discard DS data message, handle %x", handle);
	}

	hd->used = FALSE;
#if F_BT_BREDR_SUPPORT
	hd->encryptionStatus = HCI_ENCRYPTION_STATE_OFF;
	if (hd->conType == BT_CONNECTION_TYPE_BR_ACL)
	{
	    pHCI->aclHandleCnt--;
        hciConfigureNoConnectionsInd(CLEAR_ACL_CONNECTION, hd->bd, hd->bdType);
	}
	else if (hd->conType == BT_CONNECTION_TYPE_LE)
	{
#endif
	    pHCI->leAclHandleCnt--;
        hciConfigureNoConnectionsInd(CLEAR_LE_CONNECTION, hd->bd, hd->bdType);
#if F_BT_BREDR_SUPPORT
	}
#endif
}

/**
* @brief  get hci descriptor by handle
* 
* @param  handle:  hci handle
*
* @return 
*
*/
ThciHandleDesc * hciFindHandle(PHCI pHCI, uint16_t handle)
{
    ThciHandleDesc * lh;
    int i;

    for (i = 0; i < otp_str_data.gEfuse_UpperStack_s.num_link_don; i++)
    {
        lh = &pHCI->phciDescDon[i];
        if (lh->used && lh->handle == handle)
            return lh;
    }

    for (i = 0; i < otp_str_data.gEfuse_UpperStack_s.num_link_doff; i++)
    {
        lh = &pHCI->phciDescDoff[i];
        if (lh->used && lh->handle == handle)
            return lh;
    }
	HCI_TRACE_PRINTF_1(HCI_TRACE_MASK_ERROR, "!!hciFindHandle: handle(0x%x) not find", handle);

    return NULL;
}


/**
* @brief  get hci handle by bdaddr
*
* @param  bd:bd addr
* @param  conType: connection type
*
* @return  
*
*/
ThciHandleDesc * hciFindBd(uint8_t * bd, uint8_t conType)
{
	ThciHandleDesc * lh;
	int i;

	for (i=0; i < otp_str_data.gEfuse_UpperStack_s.num_link_don; i++)
	{
		lh = &pHCI->phciDescDon[i];
		if (lh->used && memcmp(lh->bd,bd,BD_ADDR_SIZE)==0 && lh->conType == conType)
			return lh;
	}
	for (i=0; i < otp_str_data.gEfuse_UpperStack_s.num_link_doff; i++)
	{
		lh = &pHCI->phciDescDoff[i];
		if (lh->used && memcmp(lh->bd,bd,BD_ADDR_SIZE)==0 && lh->conType == conType)
			return lh;
	}

	return NULL;
}

/**
* @brief  send hci msg to downstream
* 
* @param  queueID:  Queue ID
* @param  offset:  data offset in msg
* @param  buf: data buf
* @param  length:  data buf length
*
* @return 
*
*/
void hciSendDsMessageFromBuffer(uint16_t queueID, uint16_t offset, uint8_t * buf, uint16_t length)
{
	MESSAGE_T msg;

	msg.Command                    = PH_DATA_REQ;
	msg.MData.DataCB.BufferAddress = buf;
	msg.MData.DataCB.Offset        = offset;
	msg.MData.DataCB.Flag          = DATA_CB_RELEASE;
	msg.MData.DataCB.Length        = length;

	if (queueID == pHCI->dsQueueID)
	{
		hciLLWrite(&msg);
	}
	else
	{
		osMessageSend(queueID, &msg);
	}
}

/**
* @brief  send hci command to downstream
* 
* @param  queueID:  Queue ID
* @param  command:  Hci Command opcode
* @param  buf: Command buf
* @param  length:  Command buf length
*
* @return 
*
*/
void hciSendDsCommand(uint16_t queueID, uint16_t command, uint8_t * buf, uint16_t length)
{
	uint8_t * myBuf;
	uint16_t   pos;

	/*get buf from systempool*/
	if (osBufferGet(BTSystemPoolID, (uint16_t)(pHCI->WriteOffset + HCI_OFFSET + length), (void  *)&myBuf))
	{
		HCI_TRACE_PRINTF_1(HCI_TRACE_MASK_ERROR,"!!hciSendDsCommand: Could not allocate memory %d", pHCI->WriteOffset + HCI_OFFSET + length);
		assert(FALSE);
		return;
	}

	/*prepare Packet: type: CMD_PKT + opcode + param_length + param*/
	pos = pHCI->WriteOffset;
	myBuf[pos++] = CMD_PKT;
	SHORT2CHAR(myBuf+pos, command); pos += 2;
	myBuf[pos++] = (uint8_t)length;

	memcpy(myBuf + pos, buf, length);

	hciSendDsMessageFromBuffer(queueID, pHCI->WriteOffset, myBuf, length + HCI_OFFSET);
}

/**
* @brief  send no param command
*
* @param  command: command opcode
*
* @return  
*
*/
void hciCommandNoParameter(uint16_t command)
{
	hciSendDsCommand(pHCI->hciQueueID, command, NULL, 0);
	hciLaterEntry();
}

/**
* @brief  send command with byte param
*
* @param  command: command opcode
* @param  parameter: byte param
*
* @return  
*
*/
void hciCommandByteParameter(uint16_t command, uint8_t parameter)
{
    uint8_t mbuf[1];
    uint16_t pos = 0;

    mbuf[pos++] = parameter;
    hciSendDsCommand(pHCI->hciQueueID, command, mbuf, pos);
}

/**
* @brief  send command with word param
*
* @param  command: command opcode
* @param  parameter: word param
*
* @return  
*
*/
void hciCommandWordParameter(uint16_t command, uint16_t parameter)
{
    uint8_t mbuf[2];
    uint16_t pos = 0;

    SHORT2CHAR(mbuf+pos, parameter); pos += 2;
    hciSendDsCommand(pHCI->hciQueueID, command, mbuf, pos);
}

/**
* @brief  send command with word byte param
*
* @param  command: command opcode
* @param  param1: word param
* @param  param2: byte param
*
* @return  
*
*/
void hciCommandWordByteParameter(uint16_t command, uint16_t param1, uint8_t param2)
{
    uint8_t mbuf[3];
    uint16_t pos = 0;

    SHORT2CHAR(mbuf+pos, param1); pos += 2;
    mbuf[pos++] = param2;
    hciSendDsCommand(pHCI->hciQueueID, command, mbuf, pos);
}

/**
* @brief  send command with word word param
*
* @param  command: command opcode
* @param  param1: param 1 (word)
* @param  param2: param 2 (word)
*
* @return  
*
*/
void hciCommandWordWordParameter(uint16_t command, uint16_t param1, uint16_t param2)
{
	uint8_t mbuf[4];
	uint16_t pos = 0;

	SHORT2CHAR(mbuf+pos, param1); pos += 2;
	SHORT2CHAR(mbuf+pos, param2); pos += 2;
	hciSendDsCommand(pHCI->hciQueueID, command, mbuf, pos);
}


/**
* @brief  send command with bdaddr param
*
* @param  command: command opcode
* @param  bd: bdaddr
*
* @return  
*
*/
void hciCommandBDAddrParameter(uint16_t command, uint8_t * bd)
{
    hciSendDsCommand(pHCI->hciQueueID, command, bd, BD_ADDR_SIZE);
}

/**
* @brief  send command with bdaddr byte param
*
* @param  command: command opcode
* @param  bd: bdaddr
* @param  parameter: byte param
*
* @return  
*
*/
void hciCommandBDAddrByteParameter(uint16_t command, uint8_t * bd, uint8_t parameter)
{
    uint8_t mbuf[BD_ADDR_SIZE +1];
    uint16_t pos = 0;

    memcpy(mbuf+pos, bd, BD_ADDR_SIZE); pos += BD_ADDR_SIZE;
    mbuf[pos++] = parameter;
    hciSendDsCommand(pHCI->hciQueueID, command, mbuf, pos);
}

/**
* @brief  send set event mask command
* @param  pHCI
* @param  mask: mask
*
* @return  
*
*/
void hciCommandSetEventMask(uint8_t * mask, BOOL le)
{
	uint8_t mbuf[HCI_MAX_SHORT_CMD_SIZE];
	uint16_t pos = 0;

	mbuf[pos++] = mask[0];
	mbuf[pos++] = mask[1];
	mbuf[pos++] = mask[2];
	mbuf[pos++] = mask[3];
	mbuf[pos++] = mask[4];
	mbuf[pos++] = mask[5];
	mbuf[pos++] = mask[6];
	mbuf[pos++] = mask[7];
	
	if(le)
	{
		hciSendDsCommand(pHCI->hciQueueID, HCI_LE_SET_EVENT_MASK, mbuf, pos);
	}
	else
	{
		hciSendDsCommand(pHCI->hciQueueID, HCI_SET_EVENT_MASK, mbuf, pos);
	}

}

/**
* @brief  send hci disconnect command
* 
* @param  handle:  hci handle
* @param  cause: disconnect reason
*
* @return 
*
*/
void hciCommandDisconnect(uint16_t handle, uint8_t cause)
{
	uint8_t mbuf[HCI_MAX_SHORT_CMD_SIZE];
	uint16_t pos = 0;

	/*check disconnect reason*/
	assert(cause == HCI_ERR_AUTHENTICATION_FAILED
	       || cause == HCI_ERR_OTHER_END_TERMINATE_13
	       || cause == HCI_ERR_OTHER_END_TERMINATE_14
	       || cause == HCI_ERR_OTHER_END_TERMINATE_15
	       || cause == HCI_ERR_UNSUPPORTED_REMOTE_FEATURE
	       || cause == HCI_ERR_PAIRING_WITH_UNIT_KEY_NOT_SUPP
	       || cause == HCI_ERR_UNACCEPTABLE_CONNECTION_INTERVAL
	      );

	SHORT2CHAR(mbuf+pos, handle); pos += 2;
	mbuf[pos++] = cause;

	hciSendDsCommand(pHCI->hciTransQueueID2, HCI_DISCONNECT, mbuf, pos);
}

void hciConfigureNoConnectionsInd(uint8_t indId, uint8_t * bd, uint8_t bdType)
{
    THCI_CONF_NO_CONNECTIONS *pnoConnectionsInd;

    if (osBufferGet(BTSystemPoolID, sizeof(THCI_CONF_NO_CONNECTIONS), (PVOID  *)&pnoConnectionsInd) )
    {
        HCI_TRACE_PRINTF_0(HCI_TRACE_MASK_ERROR,"!!hciConfigureNoConnectionsInd cannot allocate buffer");
        return;
    }

    pnoConnectionsInd->indId = indId;
#if F_BT_BREDR_SUPPORT
    pnoConnectionsInd->noAclConnections = pHCI->aclHandleCnt;

    pnoConnectionsInd->noScoConnections = 0;
#endif
    pnoConnectionsInd->noLeConnections = pHCI->leAclHandleCnt;

    if (indId != CLEAR_ALL_CONNECTIONS && bd)
    {
        pnoConnectionsInd->bdType = bdType;
        memcpy((PVOID)(pnoConnectionsInd->bd),(PVOID)( bd), BD_ADDR_SIZE);
    }

    btsmSendMsgHciConfigureInd(HCI_CONF_NO_CONNECTIONS, sizeof(THCI_CONF_NO_CONNECTIONS), (uint8_t *)pnoConnectionsInd);
}
