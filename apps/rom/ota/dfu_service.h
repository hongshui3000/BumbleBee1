/*
 *  Routines to access hardware
 *
 *  Copyright (c) 2014 Realtek Semiconductor Corp.
 *
 *  This module is a confidential and proprietary property of RealTek and
 *  possession or use of this module requires written permission of RealTek.
 */

#ifndef _DFU_SERVICE_H_
#define  _DFU_SERVICE_H_



#define GATT_UUID_DFU_SERVICE                      0x8762

#define GATT_UUID_DFU_PACKET                      0x8763
#define GATT_UUID_DFU_CONTROL_POINT        0x8764


#define DFU_ERR_PROC_ALREADY_IN_PROGRESS    0x80
#define DFU_ERR_CCCD_IMPROPERLY_CONFIGURED  0x81

#define DFU_OPCODE_MIN                                              0x00
#define DFU_OPCODE_START_DFU                                0x01
#define DFU_OPCODE_RECEIVE_FW_IMAGE_INFO        0x02
#define DFU_OPCODE_VALID_FW                                 0x03
#define DFU_OPCODE_ACTIVE_IMAGE_RESET               0x04
#define DFU_OPCODE_SYSTEM_RESET                         0x05
#define DFU_OPCODE_REPORT_TARGET_INFO               0x06
#define DFU_OPCODE_PKT_RX_NOTIFICATION_REQ      0x07
#define DFU_OPCODE_MAX                                              0x08


#define DFU_OPCODE_NOTIFICATION                          0x10

//length of each control point procedure

#define DFU_LENGTH_START_DFU                                (1+12)
#define DFU_LENGTH_RECEIVE_FW_IMAGE_INFO        (1+2+4)
#define DFU_LENGTH_VALID_FW                                 (1+2)
#define DFU_LENGTH_ACTIVE_IMAGE_RESET               0x01
#define DFU_LENGTH_SYSTEM_RESET                         0x01
#define DFU_LENGTH_REPORT_TARGET_INFO               (1+2)
#define DFU_LENGTH_PKT_RX_NOTIFICATION_REQ      (1+2)


#define DFU_NOTIFY_LENGTH_ARV                           3
#define DFU_NOTIFY_LENGTH_REPORT_TARGET_INFO             (3+2+4)
#define DFU_NOTIFY_LENGTH_PKT_RX_NOTIFICATION         (3+2)


#define INDEX_DFU_PACKET_VALUE                          0x02
#define INDEX_DFU_CONTROL_POINT_CHAR_VALUE  0x04



typedef struct _TSTART_DFU_PARA
{
    UINT16 nOffSet;
    UINT16 nSignature;
    UINT16 nImageVersion;
    UINT16 nCRC16;
    UINT16 nImageLength;
    UINT16 nReserved;
} TSTART_DFU_PARA;

typedef struct _TPKT_RX_NOTIFICATION_REQ
{
    uint16_t PacketNum;
} TPKT_RX_NOTIFICATION_REQ;


typedef struct _DFUControlPoint
{
    uint8_t opCode;
    union
    {
        TSTART_DFU_PARA StartDfu;
        TPKT_RX_NOTIFICATION_REQ PktRxNotifyReq;
    } p;
} TDFUControlPoint, * PDFUControlPoint;


/*Notifications defined here*/

typedef struct _TNOTIFICATION_TARGET_IMAGE_INFO
{

    uint16_t nOrigFwVersion;
    uint32_t nImageUpdateOffset;
} TNOTIFICATION_TARGET_IMAGE_INFO;

typedef struct _TNOTIFICATION_REPORT_PKT_NUM
{
    uint16_t PacketNum;
} TNOTIFICATION_REPORT_PKT_NUM;


#define DFU_ARV_SUCCESS                                             0x01
#define DFU_ARV_FAIL_INVALID_PARAMETER              0x02
#define DFU_ARV_FAIL_OPERATION                              0x03
#define DFU_ARV_FAIL_DATA_SIZE_EXCEEDS_LIMIT    0x04
#define DFU_ARV_FAIL_CRC_ERROR                              0x05

typedef struct _DFUNotification
{
    uint8_t opCode;
    uint8_t reqOpCode;
    uint8_t respValue;
    union
    {
        TNOTIFICATION_TARGET_IMAGE_INFO NotifyTargetImageInfo;
        TNOTIFICATION_REPORT_PKT_NUM NotifyPktNum;
    } p;
} TDFUNotification, * PDFUNotification;


/* attribute index / client characteristic configuration descriptor (CCCD) pair */
typedef struct _GATTAttrIdxCCCD
{
    uint16_t    wAttribIndex;
    uint16_t    wCCCBits;
} TGATTAttrIdxCCCD, * PGATTAttrIdxCCCD;



uint16_t  dfuServiceAttribPut( void* pDfuCb, int iAttribIndex,
                               uint16_t wLength, uint8_t * pValue);


#endif

