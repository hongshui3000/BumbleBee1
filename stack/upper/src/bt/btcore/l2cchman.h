/**
********************************************************************************************************
Copyright (c) 2015, Realtek Semiconductor Corporation. All rights reserved.
*********************************************************************************************************
* @file       l2cchman.h
* @brief     channel management functions for L2CAP protocol layer
* @details   
*
* @author   	gordon
* @date      	2015-06-29
* @version	v0.1
*/

#ifndef __L2CCHMAN_H
#define __L2CCHMAN_H

#ifdef __cplusplus
extern "C" { 
#endif
P_ACLPOOLDESC l2cGetHciDesc(LPCBdAddr remoteBd, uint8_t conType);
#if (F_BT_L2C_ENHANCED_FEATURE_SUPPORT)
P_L2CAP_CHANNEL_EXT l2cGetFreeExtFData(void);
void l2cDeleteExtFData(P_L2CAP_CHANNEL pChan);
#else 
#define l2cGetFreeExtFData()
#define l2cDeleteExtFData(pChan)
#endif
P_L2CAP_CHANNEL l2cChannelCreate(uint16_t rcid, LPCBdAddr remoteBd, uint8_t conType);
void l2cInsertChannel(P_L2CAP_CHANNEL pNewChannnel);
int  l2cDeleteChannel(P_L2CAP_CHANNEL pChannel, BOOL HCIDisconnected);

/** typedef for a CallBack function used by some search functions when a channel was found */
typedef void ( * Channel_found_callback)(P_L2CAP_CHANNEL pChannel, PCVOID pPar);

P_L2CAP_CHANNEL l2cSearchLcid(uint16_t lcid);
P_L2CAP_CHANNEL l2cSearchHciHandle(uint16_t hciHdl);
P_L2CAP_CHANNEL l2cSearchCmdId(uint8_t cmdid);
P_L2CAP_CHANNEL l2cSearchBdCallback(TCBdAddr bd, Channel_found_callback cb, PCVOID pPar);
P_L2CAP_CHANNEL l2cCheckRcid(PC_ACLPOOLDESC hciDesc, uint16_t remote_cid);
int l2cCheckFlushTO(PC_L2CAP_CHANNEL pChan, uint16_t FlushTO);
void          l2cFreeHciDesc(P_ACLPOOLDESC pHciDesc);
P_ACLPOOLDESC l2cSearchHciCloseId(uint8_t closeID);
P_ACLPOOLDESC l2cSearchHciDescByHciHandle(uint16_t hdl);
P_ACLPOOLDESC l2cSearchHciDescBySigIdSent(uint8_t SigIdSent);

#if F_BT_LE_BT41_SUPPORT
void l2cInitChannelList(void);
P_L2CAP_CHANNEL l2cSearchRcid(PC_ACLPOOLDESC pHciDesc, uint16_t rcid);
void l2cChanRelease(P_L2CAP_CHANNEL pChan);
#endif

#ifdef __cplusplus
}
#endif

#endif /* #ifndef __L2CCHMAN_H */
