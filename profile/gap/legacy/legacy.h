/**
 *********************************************************************************************************
 *               Copyright(c) 2015, Realtek Semiconductor Corporation. All rights reserved.
 *********************************************************************************************************
 * @file      legacy.h
 * @brief    Head file for gap legacy role
 * @details
 * @author    kyle_xu
 * @date      2015-11-15
 * @version   v0.1
 *
 **********************************************************************************************************
 */

#ifndef __LEGACY_H__
#define __LEGACY_H__

#include <gap.h>
#include <blueapi.h>
#include <rtl_types.h>
#include <blueapi_types.h>
#include <profile_common.h>

#ifdef __cplusplus
extern "C" {
#endif

/** max supported sdp record number, must be less than BT_SDP_SERVER_MAX_SERVICES-1 in upper stack */
#define MAX_SDPRECORD_NUM   5
#define MAX_SDPRECORD_LEN   125   /**< max sdp record length */

#define GAP_DEVICEROLE_DONTCARE                 blueAPI_BRDeviceRoleDontCare
#define GAP_DEVICEROLE_MASTER_PREFERRED         blueAPI_BRDeviceRoleMasterPreferred
#define GAP_DEVICEROLE_MASTER_REQUIRED          blueAPI_BRDeviceRoleMasterRequired
#define GAP_DEVICEROLE_SLAVE_PREFERRED          blueAPI_BRDeviceRoleSlavePreferred
#define GAP_DEVICEROLE_SLAVE_REQUIRED           blueAPI_BRDeviceRoleSlaveRequired

#define GAP_LINKPOLICY_DISABLEALL               blueAPI_BRLinkPolicyDisableAll
#define GAP_LINKPOLICY_ENABLE_ROLESWITCH        blueAPI_BRLinkPolicyEnableRoleSwitch
#define GAP_LINKPOLICY_ENABLE_SNIFFMODE         blueAPI_BRLinkPolicyEnableSniffMode

#define GAP_RADIOMODE_VISIABLE_CONNECTABLE      blueAPI_RadioVisibleConnectable
#define GAP_RADIOMODE_VISIABLE                  blueAPI_RadioVisible
#define GAP_RADIOMODE_CONNECTABLE               blueAPI_RadioConnectable
#define GAP_RADIOMODE_NONDISCOVERABLE           blueAPI_RadioNonDiscoverable
#define GAP_RADIOMODE_OFF                       blueAPI_RadioOff

#define GAP_PAGESCAN_TYPE_STANDARD              blueAPI_BRPageScanTypeStandard
#define GAP_PAGESCAN_TYPE_INTERLACED            blueAPI_BRPageScanTypeInterlaced

#define GAP_INQUIRYSCAN_TYPE_STANDARD           blueAPI_BRInquiryScanTypeStandard
#define GAP_INQUIRYSCAN_TYPE_INTERLACED         blueAPI_BRInquiryScanTypeInterlaced

#define GAP_INQUIRYMODE_STANDARDRESULT          blueAPI_BRStandardInquiryResult
#define GAP_INQUIRYMODE_RESULTWITHRSSI          blueAPI_BRInquiryResultWithRSSI
#define GAP_INQUIRYMODE_EXTENDEDRESULT          blueAPI_BRExtendedInquiryResult

/** @brief  SDP upstream message type send to upper layer */
typedef enum
{
    INQUIRY_DEVICE_INFO,        /**< device information during inquiry */
    INQUIRY_RSP,                /**< inquiry response */
    SDP_DISCOVERY_INFO,         /**< SDP discover informateion */
    SDP_DISCOVERY_RSP,          /**< SDP discover response */
    GATT_SDP_DISCOVERY_INFO,    /**< GATT SDP discover informateion */
    GATT_SDP_DISCOVERY_RSP,     /**< GATT SDP discover response */
    DEVICE_INFO                 /**< DID information */
}TLegacyUsMsg;

/** @brief SDP record structure */
typedef struct _TSDPRecord
{
    uint16_t    length;     /**< SDP record length */
    void *      buf;        /**< point to SDP record */
} TSDPRecord;

/** @brief SDP records structure,  remember all SDP record */
typedef struct _TSDPRecords
{
    uint8_t     total_num;                  /**< number of SDP records upper layer added */
    uint8_t     register_num;               /**< number of SDP records registered to upper stack */
    TSDPRecord  record[MAX_SDPRECORD_NUM];  /**< SDP record array */
} TSDPRecords;

/* legacy callback functionn to handle gap related message */
typedef void (* pLegacyCallBack)(void *buf, TLegacyUsMsg legacy_msg);

/**
 * @brief   Count DES sequence length from format describing string
 *    <>  : Sequence w. 8 bit length field
 *    []  : Sequence w. 16 bit length field
 *    {}  : Sequence w. 32 bit length field
 *    U   : UUID - 16, a 16 bit value follows
 *    2U  : same as U
 *    4U: : UUID - 32, a 16 bit value follows
 *    6U  : UUID - 128, a 16 bit value follows, that is expanded to a 128 bit uuid
 *    8U    UUID - 128, a pointer to a 16 byte field follows, that is used as a 128 bit uuid
 *    Y   : UINT - 128 a pointer to a 16 byte field follows,
 *    X   : UINT - 64  use 2 parameters upper 32 bits and lower 32 bits
 *    L   : UINT - 32
 *    I   : UINT - 16
 *    B   : UINT - 8
 *    y   : SINT - 128 a pointer to a 16 byte field follows,
 *    x   : SINT - 64  use 2 parameters upper 32 bits and lower 32 bits
 *    l   : SINT - 32
 *    i   : SINT - 16
 *    b   : SINT - 8
 *    O   : BOOL
 *    S   : String
 *    R   : URL
 *
 *@param format
 *
 * @return  length of des sequence or 0 when format error
 *
 */
#define legacy_SDPRecordLength blueAPI_SDPRecordLength

/**
 * @brief   Create DES sequence from format describing string
 *    <>  : Sequence w. 8 bit length field
 *    []  : Sequence w. 16 bit length field
 *    {}  : Sequence w. 32 bit length field
 *    U   : UUID - 16, a 16 bit value follows
 *    2U  : same as U
 *    4U: : UUID - 32, a 16 bit value follows
 *    6U  : UUID - 128, a 16 bit value follows, that is expanded to a 128 bit uuid
 *    8U    UUID - 128, a pointer to a 16 byte field follows, that is used as a 128 bit uuid
 *    Y   : UINT - 128 a pointer to a 16 byte field follows,
 *    X   : UINT - 64  use 2 parameters upper 32 bits and lower 32 bits
 *    L   : UINT - 32
 *    I   : UINT - 16
 *    B   : UINT - 8
 *    y   : SINT - 128 a pointer to a 16 byte field follows,
 *    x   : SINT - 64  use 2 parameters upper 32 bits and lower 32 bits
 *    l   : SINT - 32
 *    i   : SINT - 16
 *    b   : SINT - 8
 *    O   : BOOL
 *    S   : String
 *    R   : URL
 *
 * @param buf: DES sequence is created in buf
 * @param format
 *
 * @return  length of des sequence or 0 when format error
 *
 */
#define legacy_SDPCreateDes    blueAPI_SDPCreateDes

/* set legacy gap parameter */
TGAP_STATUS legacy_SetGapParameter(uint16_t param, uint8_t len, void *pvalue);
/* get legacy gap parameter */
TGAP_STATUS legacy_GetGapParameter(uint16_t param, void *pvalue);
/* start inquiry */
TGAP_STATUS legacy_StartInquiry(bool limited_inquiry, uint8_t timeout, pLegacyCallBack callback);
/* stop inquiry */
TGAP_STATUS legacy_StopInquiry(void);

/* handle legacy blueAPI message */
bool legacy_HandleBlueAPIMessage(PBlueAPI_UsMessage pmsg);

void legacy_Init(void);
bool legacy_StartBtStack(void);
void legacy_RegisterCB(pfnAPPHandleInfoCB_t pfunc);
bool legacy_AddSDPRecord(void *pbuf, uint16_t length);
TGAP_STATUS legacy_StartSDPDiscovery(uint8_t *remote_bd, uint16_t uuid, bool did_discovery, pLegacyCallBack callback);
TGAP_STATUS legacy_StartGATTSDPDiscovery(uint8_t *remote_bd, uint16_t uuid, bool did_discovery, pLegacyCallBack callback);
TGAP_STATUS legacy_SetRadioMode(uint8_t radio_mode, bool limited_discoverable);
TGAP_STATUS legacy_SetDIDEIR(uint16_t vendor_id, uint16_t id_source, uint16_t product_id, uint16_t product_version);
TGAP_STATUS legacy_SetExtraEIR(uint8_t *pdata);

#ifdef __cplusplus
}
#endif    /*  __cplusplus */
#endif    /*  __LEGACY_GAP_H__*/
