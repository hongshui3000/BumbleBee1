enum { __FILE_NUM__ = 0 };

/**
 ***************************************************************************************
 *               Copyright(c) 2015, Realtek Semiconductor Corporation. All rights reserved.
 ***************************************************************************************
 * @file      legacy.c
 * @brief    This file provides gap legacy role functions
 * @details
 * @author  kyle_xu
 * @date     2015-11-18
 * @version  v0.1
 ***************************************************************************************
 */
#include <FreeRTOS.h>

#include <gap.h>
#include <mpa.h>
#include <trace.h>
#include <string.h>
#include <blueapi.h>
#include <legacy.h>
#include <gapbond_legacy.h>
#include <profile_common.h>
#include <common_defs.h>

#define DEFAULT_PAGESCAN_WINDOW         0x12
#define DEFAULT_PAGESCAN_INTERVAL       0x800
#define DEFAULT_PAGE_TIMEOUT            0x2000
#define DEFAULT_SUPVISIONTIMEOUT        0x7D00
#define DEFAULT_INQUIRYSCAN_WINDOW      0x12
#define DEFAULT_INQUIRYSCAN_INTERVAL    0x1000

static uint8_t LegacyState = GAP_LEGACY_STATE_INIT;
static uint8_t LocalBDAddr[B_ADDR_LEN];
static uint8_t DeviceName[GAP_DEVICE_NAME_LEN] = "legacy";
static uint32_t ClassOfDevice = 0x000900;

static uint8_t DeviceRole = GAP_DEVICEROLE_DONTCARE;
static uint8_t LinkPolicy = GAP_LINKPOLICY_ENABLE_ROLESWITCH | GAP_LINKPOLICY_ENABLE_SNIFFMODE;
static uint16_t SupervisionTimeout = DEFAULT_SUPVISIONTIMEOUT;

static uint8_t RadioMode = GAP_RADIOMODE_VISIABLE_CONNECTABLE;
static bool LimitedDiscoverble = false;

static uint8_t PageScanType      = GAP_PAGESCAN_TYPE_STANDARD;
static uint16_t PageScanInterval = DEFAULT_PAGESCAN_INTERVAL;
static uint16_t PageScanWindow   = DEFAULT_PAGESCAN_WINDOW;
static uint16_t PageTimeout      = DEFAULT_PAGE_TIMEOUT;

static uint8_t InquiryScanType      = GAP_INQUIRYSCAN_TYPE_STANDARD;
static uint16_t InquiryScanWindow   = DEFAULT_INQUIRYSCAN_WINDOW;
static uint16_t InquiryScanInterval = DEFAULT_INQUIRYSCAN_INTERVAL;

static uint8_t InquiryMode = GAP_INQUIRYMODE_EXTENDEDRESULT;

static uint16_t ConnHandle = 0;
static uint16_t DownstreamPoolID = 0;
static uint16_t DownstreamDataOffset = 0;
static uint16_t MaxTPDUSize = 0;
static uint8_t  MaxTPDUdsCredits = 0;

static uint8_t ConnectedDevAddr[B_ADDR_LEN] = {0};
static uint8_t DisconnectedReason = 0;

static pLegacyCallBack LegacyCallBack = NULL;
static TSDPRecords SDPRecords;
static pfnAPPHandleInfoCB_t pfnProfileCB = NULL;
static bool InquirySDPOutgoing = false;

/**
 * @brief       set a GAP parameter.
 *
 *  NOTE: You can call this function with a GAP Parameter ID and it will set the
 *        GAP Parameter.  GAP Parameter IDs are defined in gap.h.  Also,
 *        the "len" field must be set to the size of a "uint16_t" and the
 *        "pvalue" field must point to a "uint16".
 *
 * @param       param - Profile parameter ID
 * @param       len - length of data to write
 * @param       pvalue - pointer to data to write.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate
 *          data type (example: data type of uint16 will be cast to
 *          uint16 pointer).
 *
 * @return      gapAPI_CauseSuccess or gapAPI_InvalidRange or gapAPI_InvalidPara
 */
TGAP_STATUS legacy_SetGapParameter(uint16_t param, uint8_t len, void *pvalue)
{
    TGAP_STATUS ret = gapAPI_CauseSuccess;

    switch (param)
    {
    case GAPPRRA_DEVICE_NAME:
        if (len <= GAP_DEVICE_NAME_LEN)
        {
            memset(DeviceName, 0, GAP_DEVICE_NAME_LEN);
            memcpy(DeviceName, pvalue, len);
        }
        else
        {
            ret = gapAPI_InvalidRange;
        }
        break;

    case GAPPARA_CLASS_OF_DEVICE:
        if (len == sizeof(uint32_t))
        {
            ClassOfDevice = *((uint32_t*)pvalue);
        }
        else
        {
            ret = gapAPI_InvalidRange;
        }
        break;

    case GAPPARA_DEVICE_ROLE:
        if (len == sizeof(uint8_t))
        {
            DeviceRole = *((uint8_t*)pvalue);
        }
        else
        {
            ret = gapAPI_InvalidRange;
        }
        break;

    case GAPPARA_SUPERVISIONTIMEOUT:
        if (len == sizeof(uint16_t))
        {
            SupervisionTimeout = *((uint16_t*)pvalue);
        }
        else
        {
            ret = gapAPI_InvalidRange;
        }
        break;

    case GAPPARA_LINK_POLICY:
        if (len == sizeof(uint8_t))
        {
            LinkPolicy = *((uint8_t*)pvalue);
        }
        else
        {
            ret = gapAPI_InvalidRange;
        }
        break;

    case GAPPARA_RADIOMODE:
        if (len == sizeof(uint8_t))
        {
            RadioMode = *((uint8_t*)pvalue);
        }
        else
        {
            ret = gapAPI_InvalidRange;
        }
        break;

    case GAPPARA_LIMITEDDISCOVERABLE:
        if (len == sizeof(bool))
        {
            LimitedDiscoverble = *((bool*)pvalue);
        }
        else
        {
            ret = gapAPI_InvalidRange;
        }
        break;

    case GAPPARA_PAGESCAN_TYPE:
        if (len == sizeof(uint8_t))
        {
            PageScanType = *((uint8_t*)pvalue);
        }
        else
        {
            ret = gapAPI_InvalidRange;
        }
        break;

    case GAPPARA_PAGESCAN_INTERVAL:
        if (len == sizeof (uint16_t))
        {
            PageScanInterval = *((uint16_t*)pvalue);
        }
        else
        {
            ret = gapAPI_InvalidRange;
        }
        break;

    case GAPPARA_PAGESCAN_WINDOW:
        if (len == sizeof(uint16_t))
        {
            PageScanWindow = *((uint16_t*)pvalue);
        }
        else
        {
            ret = gapAPI_InvalidRange;
        }
        break;

    case GAPPARA_PAGETIMEOUT:
        if (len == sizeof(uint16_t))
        {
            PageTimeout = *((uint16_t*)pvalue);
        }
        else
        {
            ret = gapAPI_InvalidRange;
        }
        break;

    case GAPPARA_INQUIRYSCAN_TYPE: 
        if (len == sizeof(uint8_t))
        {
            InquiryScanType = *((uint8_t*)pvalue);
        }
        else
        {
            ret = gapAPI_InvalidRange;
        }
        break;

    case GAPPARA_INQUIRYSCAN_INTERVAL:
        if (len == sizeof(uint16_t))
        {
            InquiryScanInterval = *((uint16_t*)pvalue);
        }
        else
        {
            ret = gapAPI_InvalidRange;
        }
        break;

    case GAPPARA_INQUIRYSCAN_WINDOW:
        if (len == sizeof(uint16_t))
        {
            InquiryScanWindow = *((uint16_t*)pvalue);
        }
        else
        {
            ret = gapAPI_InvalidRange;
        }
        break;

    case GAPPARA_INQUIRYMODE:
        if (len == sizeof(uint8_t))
        {
            InquiryMode =  *((uint8_t*)pvalue);
        }
        else
        {
            ret = gapAPI_InvalidRange;
        }
        break;

    default:
        ret = gapAPI_InvalidPara;
        break;
    }

    return ret;
}

/**
 * @brief       get a GAP Role parameter.
 *
 *  NOTE: You can call this function with a GAP Peripheral Parameter ID and it will get a
 *        GAP Peripheral Parameter.  GAP Peripheral Parameters are defined in (gap.h).  Also, the
 *        "pvalue" field must point to a "uint16".
 *
 * @param       param - Profile parameter ID: @ref GAP_PERIPHERAL_PARAMETERS
 * @param       pvalue - pointer to location to get the value.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate
 *          data type (example: data type of uint16 will be cast to
 *          uint16 pointer).
 *
 * @return      gapAPI_CauseSuccess or gapAPI_InvalidPara (invalid paramID)
 */
TGAP_STATUS legacy_GetGapParameter(uint16_t param, void *pvalue)
{
    TGAP_STATUS ret = gapAPI_CauseSuccess;
    switch (param)
    {
    case GAPPARA_BD_ADDR:
        memcpy(pvalue, LocalBDAddr, B_ADDR_LEN);
        break;

    case GAPPRRA_CONNHANDLE:
        *((uint16_t*)pvalue) = ConnHandle;
        break;

    case GAPPRRA_DSPOOLID:
        *((uint16_t*)pvalue) = DownstreamPoolID;
        break;

    case GAPPRRA_DSDATAOFFSET:
        *((uint16_t*)pvalue) = DownstreamDataOffset;
        break;

    case GAPPRRA_MAXTPDUSIZE:
        *((uint16_t*)pvalue) = MaxTPDUSize;
        break;

    case GAPPRRA_MAXTPDUDSCREDITS:
        *((uint8_t*)pvalue) = MaxTPDUdsCredits;
        break;

    case GAPPRRA_DISCONNECTED_REASON:
        *((uint8_t*)pvalue) = DisconnectedReason;
        break;

    default:
        ret = gapAPI_InvalidPara;
        break;
    }

    return ret;
}

/**
 * @brief   initialize default parameters for legacy role
 * @param   void
 * @return  void
 */
void legacy_GapParaInit(void)
{
    SDPRecords.total_num = 0;
    SDPRecords.register_num = 0;

    LegacyState = GAP_LEGACY_STATE_INIT;

    ConnHandle = 0;

    ClassOfDevice = 0x000900;

    DeviceRole = GAP_DEVICEROLE_DONTCARE;
    LinkPolicy = GAP_LINKPOLICY_ENABLE_ROLESWITCH | GAP_LINKPOLICY_ENABLE_SNIFFMODE;
    SupervisionTimeout = DEFAULT_SUPVISIONTIMEOUT;

    RadioMode = GAP_RADIOMODE_VISIABLE_CONNECTABLE;
    LimitedDiscoverble = false;

    PageScanType = GAP_PAGESCAN_TYPE_STANDARD;
    PageScanInterval = DEFAULT_PAGESCAN_INTERVAL;
    PageScanWindow = DEFAULT_PAGESCAN_WINDOW;
    PageTimeout = DEFAULT_PAGE_TIMEOUT;

    InquiryScanType = GAP_INQUIRYSCAN_TYPE_STANDARD;
    InquiryScanWindow = DEFAULT_INQUIRYSCAN_WINDOW;
    InquiryScanInterval = DEFAULT_INQUIRYSCAN_INTERVAL;

    InquiryMode = GAP_INQUIRYMODE_EXTENDEDRESULT;
}

/**
 * @brief   initialize legacy role
 * @param   void
 * @return  void
 */
void legacy_Init(void)
{
    legacy_GapParaInit();
    GAPBondlegacy_ParaInit();
    mpa_Init();
}

/**
 * @brief      legacy role start bt stack 
 * @param   void
 * @return     bool
 */
bool legacy_StartBtStack(void)
{
    return GAP_StartBtStack();
}

/**
 * @brief      legacy role start inquiry 
 * @param   limited_inquiry    true to start limited inquiry
 * @param   timeout   inquiry timeou value
 * @param   callback    callback function to handle inquiry result and inquiry reponse
 * @return     gapAPI_CauseSuccess or gapAPI_IncorrectMode or gapAPI_ErrorUnknown
 */
TGAP_STATUS legacy_StartInquiry(bool limited_inquiry, uint8_t timeout, pLegacyCallBack callback)
{
    if (InquirySDPOutgoing == true)
    {
        return gapAPI_IncorrectMode;
    }
    else
    {
        if (blueAPI_InquiryReq(limited_inquiry, 0, timeout))
        {
            LegacyState = GAP_LEGACY_STATE_INQUIRYING;
            LegacyCallBack = callback;
            InquirySDPOutgoing = true;
            return gapAPI_CauseSuccess;
        }
        else
        {
            return gapAPI_ErrorUnknown;
        }
    }
}

/**
 * @brief      legacy role stop inquiry 
 * @param    void
 * @return    gapAPI_CauseSuccess or gapAPI_IncorrectMode or gapAPI_ErrorUnknown
 */
TGAP_STATUS legacy_StopInquiry(void)
{
    if (LegacyState != GAP_LEGACY_STATE_INQUIRYING)
    {
        return gapAPI_IncorrectMode;
    }
    else
    {
        if (blueAPI_InquiryReq(0, 1, 0))
        {
            LegacyState = GAP_LEGACY_STATE_IDLE;
            InquirySDPOutgoing = false;
            return gapAPI_CauseSuccess;
        }
        else
        {
            return gapAPI_ErrorUnknown;
        }
    }
}

#if 0
/**
 * @brief terminates the existing connection.
 *
 * @return      gapAPI_CauseSuccess or gapAPI_IncorrectMode
 */
TGAP_STATUS legacy_Disconnect(void)
{
    if (LegacyState == GAP_LEGACY_STATE_CONNECTED)
    {
        blueAPI_DisconnectMDLReq(ConnHandle, blueAPI_CauseConnectionDisconnect);
    }
    else
    {
        return (gapAPI_IncorrectMode);
    }
    return gapAPI_CauseSuccess;
}
#endif

/**
 * @brief      add sdp record buffer which will be register to sdp layer
 * @param   pbuf    point to sdp record buffer
 * @param   length  sdp record buffer
 * @return     gapAPI_InvalidRange or gapAPI_CauseSuccess
 */
bool legacy_AddSDPRecord(void *pbuf, uint16_t length)
{
    if (SDPRecords.total_num >= MAX_SDPRECORD_NUM)
    {
        DBG_BUFFER(MODULE_PROFILE, LEVEL_ERROR,
                   "legacy_GetSDPBuffer error while record number %d more than allowed %d",
                   2, SDPRecords.total_num, MAX_SDPRECORD_NUM
                   );
        return false;
    }
    else
    {
        SDPRecords.record[SDPRecords.total_num].buf = pbuf;
        SDPRecords.record[SDPRecords.total_num].length = length;
        SDPRecords.total_num++;
        return true;
    }
}

/**
 * @brief     start sdp discovery
 * @param   remote_bd    remote bd address
 * @param   uuid       uuid what to discover
 * @param   callback  callback function to handle sdp discovery result and reponse
 * @return     gapAPI_CauseSuccess or gapAPI_IncorrectMode
 */
TGAP_STATUS legacy_StartSDPDiscovery(uint8_t *remote_bd, uint16_t uuid, bool did_discovery, pLegacyCallBack callback)
{
    if (InquirySDPOutgoing == true)
    {
        DBG_BUFFER(MODULE_PROFILE, LEVEL_ERROR, "Only one SDP discover can be handled once", 0);
        return gapAPI_IncorrectMode;
    }
    else
    {
        blueAPI_SDPDiscoveryReq(remote_bd, (TBlueAPI_MDEPDataType)uuid, did_discovery);
        LegacyCallBack = callback;
        InquirySDPOutgoing = true;
        return gapAPI_CauseSuccess;
    }
}

/**
 * @brief     start GATT sdp discovery
 * @param   remote_bd    remote bd address
 * @param   uuid       uuid what to discover
 * @param   callback  callback function to handle GATT sdp discovery result and reponse
 * @return     gapAPI_CauseSuccess or gapAPI_IncorrectMode
 */
TGAP_STATUS legacy_StartGATTSDPDiscovery(uint8_t *remote_bd, uint16_t uuid, bool did_discovery, pLegacyCallBack callback)
{
    if (InquirySDPOutgoing == true)
    {
        DBG_BUFFER(MODULE_PROFILE, LEVEL_ERROR, "Only one SDP discover can be handled once", 0);
        return gapAPI_IncorrectMode;
    }
    else
    {
        blueAPI_GATTSDPDiscoveryReq(remote_bd, uuid, did_discovery);
        LegacyCallBack = callback;
        InquirySDPOutgoing = true;
        return gapAPI_CauseSuccess;
    }
}

/**
 * @brief register callback function to send events to application
 * @param pfunc  callback function.
 * @return void
 */
void legacy_RegisterCB(pfnAPPHandleInfoCB_t pfunc)
{
    pfnProfileCB = pfunc;
}

/**
 * @brief  set radio mode
 * @param radio_mode    radio mode to set, see definition in legacy.h
 * @param limited_discoverable   true to enable limited discoverable
 * @return TGAP_STATUS
 */
TGAP_STATUS legacy_SetRadioMode(uint8_t radio_mode, bool limited_discoverable)
{
    RadioMode = radio_mode;
    LimitedDiscoverble = limited_discoverable;
    blueAPI_RadioModeSetReq((TBlueAPI_RadioMode)RadioMode, LimitedDiscoverble);
    return gapAPI_CauseSuccess;
}

/**
 * @brief  set device id in eir data
 * @param vendor_id    vendor id
 * @param id_source    vendor id source
  * @param product_id    product id
 * @param product_version   product version
 * @return TGAP_STATUS
 */
TGAP_STATUS legacy_SetDIDEIR(uint16_t vendor_id, uint16_t id_source,
                                   uint16_t product_id, uint16_t product_version
                                   )
{
    blueAPI_DeviceConfigDIDSetReq(vendor_id, id_source, product_id, product_version);
    return gapAPI_CauseSuccess;
}

/**
 * @brief  add extra data in eir
 * @param pdata    point to extra eir data
 * @return TGAP_STATUS
 */
TGAP_STATUS legacy_SetExtraEIR(uint8_t *pdata)
{
    blueAPI_DeviceConfigExtraEIRSetReq(pdata);
    return gapAPI_CauseSuccess;
}

/**
 * @brief  send sdp register complete event to application
 * @param status    status of sdp register result.
 * @return void
*/
void legacy_SDPRegisterComplete(uint8_t status)
{
    TEventInfoCBs_t data = {0};

    data.eventId = PROFILE_EVT_SDP_REG_COMPLETE;
    data.sParas[0] = status;

    DBG_BUFFER(MODULE_PROFILE, LEVEL_INFO, "legacy_SDPRegisterComplete Status = %d", 1, status);

    if (pfnProfileCB)
    {
        pfnProfileCB(ProfileAPI_ServiceUndefined, (void*)&data);
    }
}

/**
 * @brief  register sdp record
 * @param void
 * @return void
 */
void legacy_RegisterSDPRecord(void)
{
    if (SDPRecords.register_num == SDPRecords.total_num)
    {
        legacy_SDPRegisterComplete(0);
    }
    else
    {
        blueAPI_SDPRegister(SDPRecords.record[SDPRecords.register_num].buf, 0);
    }
}

/**
 * @brief  process blueAPI_InquiryDeviceInfo message from bt stack, send inquiry result to APP
 * @param  pinfo    point to blueAPI_InquiryDeviceInfo message
 * @return void
 */
void legacy_HandleInquiryDeviceInfo(PBlueAPI_InquiryDeviceInfo pinfo)
{
    if(LegacyCallBack)
    {
        LegacyCallBack((void *)pinfo, INQUIRY_DEVICE_INFO);
    }
}

/**
 * @brief  process blueAPI_InquiryRsp message from bt stack, send inquiry response to APP
 * @param  prsp     point to blueAPI_InquiryRsp message
 * @return void
 */
void legacy_HandleInquiryRsp(PBlueAPI_InquiryRsp prsp)
{
    LegacyState = GAP_LEGACY_STATE_IDLE;
    GAP_StateNotificationCB(LegacyState);
    if(LegacyCallBack)
    {
        InquirySDPOutgoing = false;
        LegacyCallBack((void *)prsp, INQUIRY_RSP);
    }
}

/**
 * @brief  process blueAPI_SDPRegisterRsp message from bt stack
 * @param  prsp     point to blueAPI_SDPRegisterRsp message
 * @return void
 */
void legacy_HandleSDPRegisterRsp(PBlueAPI_SDPRegisterRsp prsp)
{
    if (prsp->cause)
    {
        DBG_BUFFER(MODULE_PROFILE, LEVEL_ERROR, "legacy register sdp error: 0x%x", 1, prsp->cause);
    }
    else
    {
        SDPRecords.register_num++;
        legacy_RegisterSDPRecord();
    }
}

/**
 * @brief  process blueAPI_SDPEndpointInd message from bt stack, send sdp discovery info to APP
               information in message:
               remote_MDEP_ID - protocol descripter list, for l2cap it is PSM, for rfcomm it is server channel number
               remote_MDEP_DataType -  service class id list UUID
               remote_MDEP_Description - service name
               remoteversion - remote version
               supportedfeatures - remote supported features
 * @param  pind    point to blueAPI_SDPEndpointInd message
 * @return void
 */
void legacy_HandleSDPEndpointInd(PBlueAPI_SDPEndpointInd pind)
{
    if (LegacyCallBack)
    {
        LegacyCallBack((void *)pind, SDP_DISCOVERY_INFO);
    }
    blueAPI_SDPEndpointConf(pind->serviceHandle, blueAPI_CauseSuccess);
}

/**
 * @brief  process blueAPI_SDPDiscoveryRsp message from bt stack, send sdp discovery response to APP
 * @param  prsp     point to blueAPI_SDPDiscoveryRsp message
 * @return void
 */
void legacy_HandleSDPDiscoveryRsp(PBlueAPI_SDPDiscoveryRsp prsp)
{
    if (LegacyCallBack)
    {
        InquirySDPOutgoing = false;
        LegacyCallBack((void *)prsp, SDP_DISCOVERY_RSP);
    }
}

/**
 * @brief  process blueAPI_GATTSDPDiscoveryInd message from bt stack, send GATT sdp discovery info to APP
 * @param  pind    point to blueAPI_GATTSDPDiscoveryInd message
 * @return void
 */
void legacy_HandleGATTSDPDiscoveryInd(PBlueAPI_GATTSDPDiscoveryInd pind)
{
    if (LegacyCallBack)
    {
        LegacyCallBack((void *)pind, GATT_SDP_DISCOVERY_INFO);
    }
    blueAPI_GATTSDPDiscoveryConf(pind->serviceHandle, blueAPI_CauseSuccess);
}

/**
 * @brief  process blueAPI_GATTSDPDiscoveryRsp message from bt stack, send GATT sdp discovery response to APP
 * @param  prsp     point to blueAPI_GATTSDPDiscoveryRsp message
 * @return void
 */
void legacy_HandleGATTSDPDiscoveryRsp(PBlueAPI_GATTSDPDiscoveryRsp prsp)
{
    if (LegacyCallBack)
    {
        InquirySDPOutgoing = false;
        LegacyCallBack((void *)prsp, GATT_SDP_DISCOVERY_RSP);
    }
}

/**
 * @brief  process blueAPI_DIDDeviceInd message from bt stack
 * @param  pind     point to blueAPI_DIDDeviceInd message
 * @return void
 */
void legacy_HandleDIDDeviceInd(PBlueAPI_DIDDeviceInd pind)
{
    if (LegacyCallBack)
    {
        LegacyCallBack((void *)pind, DEVICE_INFO);
    }

    blueAPI_DIDDeviceConf(pind->serviceHandle, blueAPI_CauseSuccess);
}

/**
 * @brief  process blueAPI_ConnectMDLRsp message from bt stack
 * @param  prsp     point to blueAPI_ConnectMDLRsp message
 * @return void
 */
void legacy_HandleConnectMDLRsp(PBlueAPI_ConnectMDLRsp prsp)
{
    if (prsp->cause == blueAPI_CauseSuccess)
    {
        memcpy(ConnectedDevAddr, prsp->remote_BD, B_ADDR_LEN);
    }
}

/**
 * @brief  process blueAPI_MCLStatusInfo message from bt stack
 * @param  pinfo     point to blueAPI_MCLStatusInfo message
 * @return void
 */
void legacy_HandleMCLStatusInfo(PBlueAPI_MCLStatusInfo pinfo)
{

}

/**
 * @brief  process blueAPI_ConnectMDLInfo message from bt stack
 * @param  pinfo     point to blueAPI_ConnectMDLInfo message
 * @return void
 */
void legacy_HandleConnectMDLInfo(PBlueAPI_ConnectMDLInfo pinfo)
{
    LegacyState = GAP_LEGACY_STATE_CONNECTED;
    GAP_StateNotificationCB(LegacyState);

    ConnHandle = pinfo->local_MDL_ID;
    DownstreamPoolID = pinfo->dsPoolID;
    DownstreamDataOffset = pinfo->dsDataOffset;
    MaxTPDUSize = pinfo->maxTPDUSize;
    MaxTPDUdsCredits = pinfo->maxTPDUdsCredits;
}

/**
 * @brief  process blueAPI_DisconnectMDLRsp message from bt stack
 * @param  prsp     point to blueAPI_DisconnectMDLRsp message
 * @return void
 */
static void legacy_HandleDisconnectMDLRsp(PBlueAPI_DisconnectMDLRsp prsp)
{

}

/**
 * @brief  process blueAPI_DisconnectMDLInd message from bt stack
 * @param  pind     point to blueAPI_DisconnectMDLInd message
 * @return void
 */
void legacy_HandleDisconnectMDLInd(PBlueAPI_DisconnectMDLInd pind)
{
    DisconnectedReason = pind->cause;
    blueAPI_DisconnectMDLConf(pind->local_MDL_ID);
}

/**
 * @brief  process blueAPI_DeleteMDLInfo message from bt stack
 * @param  pinfo     point to blueAPI_DeleteMDLInfo message
 * @return void
 */
void legacy_HandleDeleteMDLInfo(PBlueAPI_DeleteMDLInfo pinfo)
{

}

/**
 * @brief  process blueAPI_InternalEventInfo message from bt stack
 * @param  pinfo     point to blueAPI_InternalEventInfo message
 * @return void
 */
void legacy_HandleInternalEventInfo(PBlueAPI_InternalEventInfo pinfo)
{
#ifdef GAP_DEBUG_FLAG
    DBG_BUFFER(MODULE_PROFILE, LEVEL_INFO, "legacy_HandleInternalEventInfo: eventType=0x%x.",
               1, pinfo->eventType);
#endif
}

/**
 * @brief  process blueAPI_ACLStatusInfo message from bt stack
 * @param  pinfo     point to blueAPI_ACLStatusInfo message
 * @return void
 */
void legacy_HandleACLStatusInfo(PBlueAPI_ACLStatusInfo pinfo)
{
    switch (pinfo->status)
    {
    case blueAPI_ACLConnectedActive:
        break;

    case blueAPI_ACLConnectionDisconnected:
        LegacyState = GAP_LEGACY_STATE_IDLE;
        GAP_StateNotificationCB(LegacyState);
        break;

    default:
        break;
    }
}

/**
 * @brief  process blueAPI_ReleaseRsp message from bt stack
 * @param  prsp     point to blueAPI_ReleaseRsp message
 * @return void
 */
void legacy_HandleReleaseRsp(PBlueAPI_ReleaseRsp prsp)
{
}

/**
  * @brief  process blueAPI_EventRegisterRsp message from bt stack.
  * @param  prsp - message sent from upper stack.
  * @return void
  */
void legacy_HandleRegisterRsp(PBlueAPI_RegisterRsp prsp)
{
#ifdef GAP_DEBUG_FLAG
    DBG_BUFFER(MODULE_PROFILE, LEVEL_INFO, "GAP_Handle_RegisterRsp: cause=%d", 1, prsp->cause);
#endif
    if (prsp->cause == blueAPI_CauseSuccess)
    {
        /* Upper Stack register success */
    }
    else
    {
        /* Upper Stack register fail */
    }
}

/**
  * @brief  process blueAPI_EventActInfo message from bt stack.
  * @param  pinfo - message sent from upper stack.
  * @return void
  */
void legacy_HandleActInfo(PBlueAPI_ActInfo pinfo)
{
#ifdef GAP_DEBUG_FLAG
    DBG_BUFFER(MODULE_PROFILE, LEVEL_INFO, "GAP_Handle_ActInfo: cause=%d", 1, pinfo->cause);
#endif

    if (pinfo->cause == blueAPI_CauseSuccess)
    {
        memcpy(LocalBDAddr, pinfo->local_BD, B_ADDR_LEN );
        DBG_BUFFER(MODULE_PROFILE,LEVEL_INFO,"My Local legacy BD -- 0x%2x,0x%2x,0x%2x,0x%2x,0x%2x,0x%2x", 6,
                    pinfo->local_BD[5], pinfo->local_BD[4], pinfo->local_BD[3],
                    pinfo->local_BD[2], pinfo->local_BD[1], pinfo->local_BD[0]);

        blueAPI_DeviceConfigDeviceSetReq(ClassOfDevice, DeviceName);
    }
}

/**
 * @brief  process blueAPI_DeviceConfigSetRsp message from bt stack
 * @param  prsp     point to blueAPI_DeviceConfigSetRsp message
 * @return void
 */
void legacy_HandleDeviceConfigSetRsp(PBlueAPI_DeviceConfigSetRsp prsp)
{
    switch (prsp->opCode)
    {
    case blueAPI_DeviceConfigDevice:
        blueAPI_DeviceConfigPagescanSetReq((TBlueAPI_BRPageScanType)PageScanType,
                                            blueAPI_BRPageScanRepMode_Manual,
                                            PageScanInterval, PageScanWindow, PageTimeout
                                            );
        break;

    case blueAPI_DeviceConfigPagescan:
        blueAPI_DeviceConfigInquiryscanSetReq((TBlueAPI_BRInquiryScanType)InquiryScanType,
                                              InquiryScanInterval, InquiryScanWindow);
        break;

    case blueAPI_DeviceConfigInquiryscan:
        blueAPI_DeviceConfigInquiryModeReq((TBlueAPI_BRInquiryMode)InquiryMode);
        break;

    case blueAPI_DeviceConfigInquirymode:
        blueAPI_DeviceConfigLinkpolicySetReq((TBlueAPI_BRLinkPolicy)LinkPolicy,
                                     (TBlueAPI_BRDeviceRole)DeviceRole,
                                     SupervisionTimeout
                                     );
        break;

    case blueAPI_DeviceConfigLinkpolicy:
        blueAPI_RadioModeSetReq((TBlueAPI_RadioMode)RadioMode, LimitedDiscoverble);
        break;

    default:
        break;
    }
}

/**
 * @brief  process blueAPI_RadioModeSetRsp message from bt stack
 * @param  prsp     point to blueAPI_RadioModeSetRsp message
 * @return void
 */
void legacy_HandleRadioModeSetRsp(PBlueAPI_RadioModeSetRsp prsp)
{
    if (prsp->cause == blueAPI_CauseSuccess)
    {
        GAPBondlegacy_SetPairable();
        legacy_RegisterSDPRecord();
    }
}

/**
 * @brief      process TBlueAPI_UsMessage message from bt stack, only legacy gap 
 *                related messages are handled here
 * @param    pmsg  - pointer to TBlueAPI_UsMessage message
 * @return     bool
 */
bool legacy_ProcessGapEvent(PBlueAPI_UsMessage pmsg)
{
    switch (pmsg->Command)
    {
    case blueAPI_EventRegisterRsp:
        legacy_HandleRegisterRsp(&pmsg->p.RegisterRsp);
        break;

    case blueAPI_EventActInfo:
        legacy_HandleActInfo(&pmsg->p.ActInfo);
        break;

    case blueAPI_EventDeviceConfigSetRsp:
        legacy_HandleDeviceConfigSetRsp(&pmsg->p.DeviceConfigSetRsp);
        break;

    case blueAPI_EventRadioModeSetRsp:
        legacy_HandleRadioModeSetRsp(&pmsg->p.RadioModeSetRsp);
        break;

    case blueAPI_EventInquiryRsp:
        legacy_HandleInquiryRsp(&pmsg->p.InquiryRsp);
        break;
    
    case blueAPI_EventInquiryDeviceInfo:
        legacy_HandleInquiryDeviceInfo(&pmsg->p.InquiryDeviceInfo);
        break;

    case blueAPI_EventSDPRegisterRsp:
        legacy_HandleSDPRegisterRsp(&pmsg->p.SDPRegisterRsp);
        break;

    case blueAPI_EventSDPEndpointInd:
        legacy_HandleSDPEndpointInd(&pmsg->p.SDPEndpointInd);
        break;

    case blueAPI_EventSDPDiscoveryRsp:
        legacy_HandleSDPDiscoveryRsp(&pmsg->p.SDPDiscoveryRsp);
        break;

    case blueAPI_EventGATTSDPDiscoveryRsp:
        legacy_HandleGATTSDPDiscoveryRsp(&pmsg->p.GATTSDPDiscoveryRsp);
        break;

    case blueAPI_EventGATTSDPDiscoveryInd:
        legacy_HandleGATTSDPDiscoveryInd(&pmsg->p.GATTSDPDiscoveryInd);
        break;

    case blueAPI_EventDIDDeviceInd:
        legacy_HandleDIDDeviceInd(&pmsg->p.DIDDeviceInd);
        break;

    case blueAPI_EventConnectMDLRsp:
        legacy_HandleConnectMDLRsp(&pmsg->p.ConnectMDLRsp);
        break;

    case blueAPI_EventMCLStatusInfo:
        legacy_HandleMCLStatusInfo(&pmsg->p.MCLStatusInfo);
        break;

    case blueAPI_EventConnectMDLInfo:
        legacy_HandleConnectMDLInfo(&pmsg->p.ConnectMDLInfo);
        break;

    case blueAPI_EventDisconnectMDLRsp:
        legacy_HandleDisconnectMDLRsp(&pmsg->p.DisconnectMDLRsp);
        break;

    case blueAPI_EventDisconnectMDLInd:
        legacy_HandleDisconnectMDLInd(&pmsg->p.DisconnectMDLInd);
        break;

    case blueAPI_EventDeleteMDLInfo:
        legacy_HandleDeleteMDLInfo(&pmsg->p.DeleteMDLInfo);
        break;

    case blueAPI_EventACLStatusInfo:
        legacy_HandleACLStatusInfo(&pmsg->p.ACLStatusInfo);
        break;

    case blueAPI_EventInternalEventInfo:
        legacy_HandleInternalEventInfo(&pmsg->p.InternalEventInfo);
        break;

    case blueAPI_EventReleaseRsp:
        legacy_HandleReleaseRsp(&pmsg->p.ReleaseRsp);
        break;

    default:
        break;
    }
    return true;
}

/**
 * @brief      process TBlueAPI_UsMessage message from bt stack
 * @param    pmsg  - pointer to TBlueAPI_UsMessage message
 *
 * @return     bool
 */
bool legacy_HandleBlueAPIMessage(PBlueAPI_UsMessage pmsg)
{
    legacy_ProcessGapEvent(pmsg);
    GAPBondlegacy_ProcessEvent(pmsg);
    mpa_HandleMessage(pmsg);

    if (pmsg)
    {
        blueAPI_BufferRelease(pmsg);
        pmsg = NULL;
    }

    return true;
}
