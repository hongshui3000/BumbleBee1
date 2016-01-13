
#ifndef __AVCTP_H
#define __AVCTP_H

/** Add Includes here **/
#include "rtl_types.h"
#include "blueapi_types.h"
#include "bterrcod.h"
#include "bee_message.h"
#include <sdp_code.h>
/**dqy   */
#include "avrcp_api.h"

#define AVCTP_L2CAP_MTU_SIZE  		1691    //maximum SDU size ?

//#define AVCTP_HEADER_LENGTH 3

#define AVCTP_MESSAGE_TYPE_COMMAND      0
#define AVCTP_MESSAGE_TYPE_RESPONSE     1

bool avctp_ProtocolInit(void);
bool avctp_Connect(TBdAddr remote_bd);
bool avctp_Disconnect(uint16_t cid);

#define avctpSendData(cid, avrcp_data, length, transact, crtype) avctp_SendData2Buf(cid, avrcp_data, length, NULL, 0, transact, crtype)
int avctp_SendData2Buf(uint16_t cid, uint8_t *avrcp_data, short avrcp_length, uint8_t *avrcp_data2, short length2, uint8_t transact, uint8_t crtype);


#endif
