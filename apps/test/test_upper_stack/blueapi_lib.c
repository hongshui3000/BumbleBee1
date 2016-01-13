enum { __FILE_NUM__= 0 };



#include "rtl_types.h"
#include <blueapi_types.h>


/* trace data must be located in flash, it will be used also by the application */
#undef  TRACE_DATA
#define TRACE_DATA

static const char unknownString[] TRACE_DATA = "???";
extern int sprintf(char *msg, const char *fmt, ...);

/****************************************************************************
 * blueAPI_StringizeBd()
 ****************************************************************************/
const char * blueAPI_StringizeBd(uint8_t * bd)
{
    static char buf[20];
    static const char fmt[] = "%X:%X:%X:%X:%X:%X";
    uint8_t BD0[BLUE_API_BD_SIZE] = { 0 };

    if (bd == NULL)
        bd = BD0;

    sprintf(buf, fmt, bd[5], bd[4], bd[3], bd[2], bd[1], bd[0]);
    return buf;
} /* blueAPI_StringizeBd */



/****************************************************************************
 * blueAPI_CauseString()
 ****************************************************************************/
const char * blueAPI_CauseString(TBlueAPI_Cause cause)
{
    static const char pCauseSuccess[]              TRACE_DATA = "success";
    static const char pCauseAccept[]               TRACE_DATA = "accept";
    static const char pCauseReject[]               TRACE_DATA = "reject";
    static const char pCauseResourceError[]        TRACE_DATA = "resourceError";
    static const char pCauseInvalidParameter[]     TRACE_DATA = "invalidParameter";
    static const char pCauseInvalidState[]         TRACE_DATA = "invalidState";
    static const char pCauseConnectionDisconnect[] TRACE_DATA = "connectionDisconnect";
    static const char pCauseConnectionLost[]       TRACE_DATA = "connectionLost";
    static const char pCauseAuthenticationFailed[] TRACE_DATA = "authFailed";
    static const char pCauseInitTimeout[]          TRACE_DATA = "initTimeout";
    static const char pCauseInitOutofSync[]        TRACE_DATA = "initOutofSync";
    static const char pCauseInitHardwareFailure[]  TRACE_DATA = "initHWFail";
    static const char pCauseLowerLayerError[]      TRACE_DATA = "lowerLayerError";
    static const char pCauseAddressNotResolved[]   TRACE_DATA = "addressNotResolved";
    static const char pCauseUnspecified[]          TRACE_DATA = "Unspecified";
    static const char pCauseNotSupported[]         TRACE_DATA = "notSupported";

    switch (cause)
    {
    case blueAPI_CauseSuccess:
        return pCauseSuccess;
    case blueAPI_CauseAccept:
        return pCauseAccept;
    case blueAPI_CauseReject:
        return pCauseReject;
    case blueAPI_CauseResourceError:
        return pCauseResourceError;
    case blueAPI_CauseInvalidParameter:
        return pCauseInvalidParameter;
    case blueAPI_CauseInvalidState:
        return pCauseInvalidState;
    case blueAPI_CauseConnectionDisconnect:
        return pCauseConnectionDisconnect;
    case blueAPI_CauseConnectionLost:
        return pCauseConnectionLost;
    case blueAPI_CauseAuthenticationFailed:
        return pCauseAuthenticationFailed;
    case blueAPI_CauseInitTimeout:
        return pCauseInitTimeout;
    case blueAPI_CauseInitOutofSync:
        return pCauseInitOutofSync;
    case blueAPI_CauseInitHardwareFailure:
        return pCauseInitHardwareFailure;
    case blueAPI_CauseLowerLayerError:
        return pCauseLowerLayerError;
    case blueAPI_CauseAddressNotResolved:
        return pCauseAddressNotResolved;
    case blueAPI_CauseUnspecified:
        return pCauseUnspecified;
    case blueAPI_CauseNotSupported:
        return pCauseNotSupported;
    default:
        return unknownString;
    }
} /* blueAPI_CauseString */

/****************************************************************************
 * blueAPI_CommandString()
 ****************************************************************************/
const char * blueAPI_CommandString(TBlueAPI_Command Command)
{
    static const char pConnectMDLReq[]       TRACE_DATA = "ConnectMDLReq";
    static const char pConnectMDLRsp[]       TRACE_DATA = "ConnectMDLRsp";
    static const char pDisconnectMDLReq[]    TRACE_DATA = "DisconnectMDLReq";
    static const char pDisconnectMDLRsp[]    TRACE_DATA = "DisconnectMDLRsp";
    static const char pDisconnectMDLInd[]    TRACE_DATA = "DisconnectMDLInd";
    static const char pDisconnectMDLConf[]   TRACE_DATA = "DisconnectMDLConf";
    static const char pCreateMDLInd[]        TRACE_DATA = "CreateMDLInd";
    static const char pCreateMDLConf[]       TRACE_DATA = "CreateMDLConf";
    static const char pConnectMDLInfo[]      TRACE_DATA = "ConnectMDLInfo";
    static const char pDeleteMDLInfo[]       TRACE_DATA = "DeleteMDLInfo";
    static const char pMCLStatusInfo[]       TRACE_DATA = "MCLStatusInfo";
    static const char pACLStatusInfo[]       TRACE_DATA = "ACLStatusInfo";
    static const char pRegisterReq[]         TRACE_DATA = "RegisterReq";
    static const char pRegisterRsp[]         TRACE_DATA = "RegisterRsp";

    static const char pActInfo[]             TRACE_DATA = "ActInfo";
    static const char pInternalEventInfo[]   TRACE_DATA = "InternalEventInfo";

    static const char pDeviceConfigSetReq[]  TRACE_DATA = "DeviceConfigSetReq";
    static const char pDeviceConfigSetRsp[]  TRACE_DATA = "DeviceConfigSetRsp";


    static const char pGATTServiceRegisterReq[]        TRACE_DATA = "GATTServiceRegisterReq";
    static const char pGATTServiceRegisterRsp[]        TRACE_DATA = "GATTServiceRegisterRsp";
    static const char pGATTAttributeUpdateReq[]        TRACE_DATA = "GATTAttributeUpdateReq";
    static const char pGATTAttributeUpdateRsp[]        TRACE_DATA = "GATTAttributeUpdateRsp";
    static const char pGATTAttributeUpdateStatusInd[]  TRACE_DATA = "GATTAttributeUpdateStatusInd";
    static const char pGATTAttributeUpdateStatusConf[] TRACE_DATA = "GATTAttributeUpdateStatusConf";
    static const char pGATTAttributeReadInd[]          TRACE_DATA = "GATTAttributeReadInd";
    static const char pGATTAttributeReadConf[]         TRACE_DATA = "GATTAttributeReadConf";
    static const char pGATTAttributeWriteInd[]         TRACE_DATA = "GATTAttributeWriteInd";
    static const char pGATTAttributeWriteConf[]        TRACE_DATA = "GATTAttributeWriteConf";
#if 0
	static const char pGATTAttributePrepareWriteInd[]  TRACE_DATA = "GATTAttributePrepareWriteInd";
	static const char pGATTAttributePrepareWriteConf[] TRACE_DATA = "GATTAttributePrepareWriteConf";
	static const char pGATTAttributeExecuteWriteInd[]  TRACE_DATA = "GATTAttributeExecuteWriteInd";
	static const char pGATTAttributeExecuteWriteConf[] TRACE_DATA = "GATTAttributeExecuteWriteConf";
#endif
    static const char pGATTAttributeWriteCommandInfo[] TRACE_DATA = "GATTAttributeWriteCommandInfo";
    static const char pGATTCCCDInfo[]                  TRACE_DATA = "GATTCCCDInfo";
    static const char pGATTDiscoveryReq[]              TRACE_DATA = "GATTDiscoveryReq";
    static const char pGATTDiscoveryRsp[]              TRACE_DATA = "GATTDiscoveryRsp";
    static const char pGATTDiscoveryInd[]              TRACE_DATA = "GATTDiscoveryInd";
    static const char pGATTDiscoveryConf[]             TRACE_DATA = "GATTDiscoveryConf";
    static const char pGATTAttributeReadReq[]          TRACE_DATA = "GATTAttributeReadReq";
    static const char pGATTAttributeReadRsp[]          TRACE_DATA = "GATTAttributeReadRsp";
#if 0
	static const char pGATTAttributeReadMultipleReq[]  TRACE_DATA = "GATTAttributeReadMultipleReq";
    static const char pGATTAttributeReadMultipleRsp[]  TRACE_DATA = "GATTAttributeReadMultipleRsp";
#endif
    static const char pGATTAttributeWriteReq[]         TRACE_DATA = "GATTAttributeWriteReq";
    static const char pGATTAttributeWriteRsp[]         TRACE_DATA = "GATTAttributeWriteRsp";
#if 0
	static const char pGATTAttributePrepareWriteReq[]  TRACE_DATA = "GATTAttributePrepareWriteReq";
    static const char pGATTAttributePrepareWriteRsp[]  TRACE_DATA = "GATTAttributePrepareWriteRsp";
	static const char pGATTAttributeExecuteWriteReq[]  TRACE_DATA = "GATTAttributeExecuteWriteReq";
    static const char pGATTAttributeExecuteWriteRsp[]  TRACE_DATA = "GATTAttributeExecuteWriteRsp";
#endif
    static const char pGATTAttributeInd[]              TRACE_DATA = "GATTAttributeInd";
    static const char pGATTAttributeConf[]             TRACE_DATA = "GATTAttributeConf";
    static const char pGATTAttributeNotificationInfo[] TRACE_DATA = "GATTAttributeNotificationInfo";

    static const char pGATTSecurityReq[]               TRACE_DATA = "GATTSecurityReq";
    static const char pGATTSecurityRsp[]               TRACE_DATA = "GATTSecurityRsp";
    static const char pGATTServerStoreInd[]            TRACE_DATA = "GATTServerStoreInd";
    static const char pGATTServerStoreConf[]           TRACE_DATA = "GATTServerStoreConf";
    static const char pGATTMtuSizeInfo[]               TRACE_DATA = "GATTMtuSizeInfo";
    static const char pPairableModeSetReq[]            TRACE_DATA = "PairableModeSetReq";
    static const char pPairableModeSetRsp[]            TRACE_DATA = "PairableModeSetRsp";

    static const char pUserPasskeyReqInd[]             TRACE_DATA = "UserPasskeyReqInd";
    static const char pUserPasskeyReqConf[]            TRACE_DATA = "UserPasskeyReqConf";
    static const char pUserPasskeyReqReplyReq[]        TRACE_DATA = "UserPasskeyReqReplyReq";
    static const char pUserPasskeyReqReplyRsp[]        TRACE_DATA = "UserPasskeyReqReplyRsp";
    static const char pUserPasskeyNotificationInfo[]   TRACE_DATA = "UserPasskeyNotificationInfo";

    static const char pRemoteOOBDataReqInd[]           TRACE_DATA = "RemoteOOBDataReqInd";
    static const char pRemoteOOBDataReqConf[]          TRACE_DATA = "RemoteOOBDataReqConf";


    static const char pAuthResultInd[]                 TRACE_DATA = "AuthResultInd";
    static const char pAuthResultConf[]                TRACE_DATA = "AuthResultConf";
    static const char pAuthResultRequestInd[]          TRACE_DATA = "AuthResultRequestInd";
    static const char pAuthResultRequestConf[]         TRACE_DATA = "AuthResultRequestConf";

    static const char pLEAdvertiseReq[]             TRACE_DATA = "AdvertiseReq";
    static const char pLEAdvertiseRsp[]             TRACE_DATA = "AdvertiseRsp";
    static const char pLEAdvertiseParameterSetReq[] TRACE_DATA = "AdvertiseParameterSetReq";
    static const char pLEAdvertiseParameterSetRsp[] TRACE_DATA = "AdvertiseParameterSetRsp";
    static const char pLEAdvertiseDataSetReq[]      TRACE_DATA = "AdvertiseDataSetReq";
    static const char pLEAdvertiseDataSetRsp[]      TRACE_DATA = "AdvertiseDataSetRsp";
    static const char pLEScanReq[]                  TRACE_DATA = "ScanReq";
    static const char pLEScanRsp[]                  TRACE_DATA = "ScanRsp";
    static const char pLEScanInfo[]                 TRACE_DATA = "ScanInfo";
    static const char pLEModifyWhitelistReq[]       TRACE_DATA = "ModifyWhitelistReq";
    static const char pLEModifyWhitelistRsp[]       TRACE_DATA = "ModifyWhitelistRsp";
    static const char pLEConnectionUpdateReq[]      TRACE_DATA = "ConnectionUpdateReq";
    static const char pLEConnectionUpdateRsp[]      TRACE_DATA = "ConnectionUpdateRsp";
    static const char pLEConnectionUpdateInd[]      TRACE_DATA = "ConnectionUpdateInd";
    static const char pLEConnectionUpdateConf[]     TRACE_DATA = "ConnectionUpdateConf";
    static const char pLEConnectionParameterInfo[]  TRACE_DATA = "ConnectionParameterInfo";
    static const char pLEPrivacyModeReq[]           TRACE_DATA = "LEPrivacyModeReq";
    static const char pLEPrivacyModeRsp[]           TRACE_DATA = "LEPrivacyModeRsp";


    static const char pIdle[]                       TRACE_DATA = "-non-";

    switch (Command)
    {
    case blueAPI_EventConnectMDLReq:
        return pConnectMDLReq;
    case blueAPI_EventConnectMDLRsp:
        return pConnectMDLRsp;
    case blueAPI_EventDisconnectMDLReq:
        return pDisconnectMDLReq;
    case blueAPI_EventDisconnectMDLRsp:
        return pDisconnectMDLRsp;
    case blueAPI_EventDisconnectMDLInd:
        return pDisconnectMDLInd;
    case blueAPI_EventDisconnectMDLConf:
        return pDisconnectMDLConf;
    case blueAPI_EventCreateMDLInd:
        return pCreateMDLInd;
    case blueAPI_EventCreateMDLConf:
        return pCreateMDLConf;
    case blueAPI_EventConnectMDLInfo:
        return pConnectMDLInfo;
    case blueAPI_EventDeleteMDLInfo:
        return pDeleteMDLInfo;
    case blueAPI_EventMCLStatusInfo:
        return pMCLStatusInfo;
    case blueAPI_EventACLStatusInfo:
        return pACLStatusInfo;
    case blueAPI_EventRegisterReq:
        return pRegisterReq;
    case blueAPI_EventRegisterRsp:
        return pRegisterRsp;

    case blueAPI_EventActInfo:
        return pActInfo;
    case blueAPI_EventInternalEventInfo:
        return pInternalEventInfo;

    case blueAPI_EventDeviceConfigSetReq:
        return pDeviceConfigSetReq;
    case blueAPI_EventDeviceConfigSetRsp:
        return pDeviceConfigSetRsp;

    case blueAPI_EventGATTServiceRegisterReq:
        return pGATTServiceRegisterReq;
    case blueAPI_EventGATTServiceRegisterRsp:
        return pGATTServiceRegisterRsp;

    case blueAPI_EventGATTAttributeUpdateReq:
        return pGATTAttributeUpdateReq;
    case blueAPI_EventGATTAttributeUpdateRsp:
        return pGATTAttributeUpdateRsp;
    case blueAPI_EventGATTAttributeUpdateStatusInd:
        return pGATTAttributeUpdateStatusInd;
    case blueAPI_EventGATTAttributeUpdateStatusConf:
        return pGATTAttributeUpdateStatusConf;
    case blueAPI_EventGATTAttributeReadInd:
        return pGATTAttributeReadInd;
    case blueAPI_EventGATTAttributeReadConf:
        return pGATTAttributeReadConf;
    case blueAPI_EventGATTAttributeWriteInd:
        return pGATTAttributeWriteInd;
    case blueAPI_EventGATTAttributeWriteConf:
        return pGATTAttributeWriteConf;
#if 0
	case blueAPI_EventGATTAttributePrepareWriteInd:
        return pGATTAttributePrepareWriteInd;
	case blueAPI_EventGATTAttributePrepareWriteConf:
        return pGATTAttributePrepareWriteConf;
	case blueAPI_EventGATTAttributeExecuteWriteInd:
        return pGATTAttributeExecuteWriteInd;
	case blueAPI_EventGATTAttributeExecuteWriteConf:
        return pGATTAttributeExecuteWriteConf;
#endif
    case blueAPI_EventGATTAttributeWriteCommandInfo:
        return pGATTAttributeWriteCommandInfo;
    case blueAPI_EventGATTCCCDInfo:
        return pGATTCCCDInfo;
    case blueAPI_EventGATTDiscoveryReq:
        return pGATTDiscoveryReq;
    case blueAPI_EventGATTDiscoveryRsp:
        return pGATTDiscoveryRsp;
    case blueAPI_EventGATTDiscoveryInd:
        return pGATTDiscoveryInd;
    case blueAPI_EventGATTDiscoveryConf:
        return pGATTDiscoveryConf;
    case blueAPI_EventGATTAttributeReadReq:
        return pGATTAttributeReadReq;
    case blueAPI_EventGATTAttributeReadRsp:
        return pGATTAttributeReadRsp;
#if 0
	case blueAPI_EventGATTAttributeReadMultipleReq:
        return pGATTAttributeReadMultipleReq;
    case blueAPI_EventGATTAttributeReadMultipleRsp:
        return pGATTAttributeReadMultipleRsp;
#endif
    case blueAPI_EventGATTAttributeWriteReq:
        return pGATTAttributeWriteReq;
    case blueAPI_EventGATTAttributeWriteRsp:
        return pGATTAttributeWriteRsp;
#if 0
	case blueAPI_EventGATTAttributePrepareWriteReq:
        return pGATTAttributePrepareWriteReq;
    case blueAPI_EventGATTAttributePrepareWriteRsp:
        return pGATTAttributePrepareWriteRsp;
	case blueAPI_EventGATTAttributeExecuteWriteReq:
        return pGATTAttributeExecuteWriteReq;
    case blueAPI_EventGATTAttributeExecuteWriteRsp:
        return pGATTAttributeExecuteWriteRsp;
#endif
    case blueAPI_EventGATTAttributeInd:
        return pGATTAttributeInd;
    case blueAPI_EventGATTAttributeConf:
        return pGATTAttributeConf;
    case blueAPI_EventGATTAttributeNotificationInfo:
        return pGATTAttributeNotificationInfo;

    case blueAPI_EventGATTSecurityReq:
        return pGATTSecurityReq;
    case blueAPI_EventGATTSecurityRsp:
        return pGATTSecurityRsp;
    case blueAPI_EventGATTServerStoreInd:
        return pGATTServerStoreInd;
    case blueAPI_EventGATTServerStoreConf:
        return pGATTServerStoreConf;
    case blueAPI_EventGATTMtuSizeInfo:
        return pGATTMtuSizeInfo;
    case blueAPI_EventPairableModeSetReq:
        return pPairableModeSetReq;
    case blueAPI_EventPairableModeSetRsp:
        return pPairableModeSetRsp;
    case blueAPI_EventUserPasskeyReqInd:
        return pUserPasskeyReqInd;
    case blueAPI_EventUserPasskeyReqConf:
        return pUserPasskeyReqConf;
    case blueAPI_EventUserPasskeyReqReplyReq:
        return pUserPasskeyReqReplyReq;
    case blueAPI_EventUserPasskeyReqReplyRsp:
        return pUserPasskeyReqReplyRsp;
    case blueAPI_EventUserPasskeyNotificationInfo:
        return pUserPasskeyNotificationInfo;
    case blueAPI_EventRemoteOOBDataReqInd:
        return pRemoteOOBDataReqInd;
    case blueAPI_EventRemoteOOBDataReqConf:
        return pRemoteOOBDataReqConf;


    case blueAPI_EventAuthResultInd:
        return pAuthResultInd;
    case blueAPI_EventAuthResultConf:
        return pAuthResultConf;
    case blueAPI_EventAuthResultRequestInd:
        return pAuthResultRequestInd;
    case blueAPI_EventAuthResultRequestConf:
        return pAuthResultRequestConf;

    case blueAPI_EventLEAdvertiseReq:
        return pLEAdvertiseReq;
    case blueAPI_EventLEAdvertiseRsp:
        return pLEAdvertiseRsp;
    case blueAPI_EventLEAdvertiseParameterSetReq:
        return pLEAdvertiseParameterSetReq;
    case blueAPI_EventLEAdvertiseParameterSetRsp:
        return pLEAdvertiseParameterSetRsp;
    case blueAPI_EventLEAdvertiseDataSetReq:
        return pLEAdvertiseDataSetReq;
    case blueAPI_EventLEAdvertiseDataSetRsp:
        return pLEAdvertiseDataSetRsp;
    case blueAPI_EventLEScanReq:
        return pLEScanReq;
    case blueAPI_EventLEScanRsp:
        return pLEScanRsp;
    case blueAPI_EventLEScanInfo:
        return pLEScanInfo;
    case blueAPI_EventLEModifyWhitelistReq:
        return pLEModifyWhitelistReq;
    case blueAPI_EventLEModifyWhitelistRsp:
        return pLEModifyWhitelistRsp;
    case blueAPI_EventLEConnectionUpdateReq:
        return pLEConnectionUpdateReq;
    case blueAPI_EventLEConnectionUpdateRsp:
        return pLEConnectionUpdateRsp;
    case blueAPI_EventLEConnectionUpdateInd:
        return pLEConnectionUpdateInd;
    case blueAPI_EventLEConnectionUpdateConf:
        return pLEConnectionUpdateConf;
    case blueAPI_EventLEConnectionParameterInfo:
        return pLEConnectionParameterInfo;
    case blueAPI_EventLEPrivacyModeReq:
        return pLEPrivacyModeReq;
    case blueAPI_EventLEPrivacyModeRsp:
        return pLEPrivacyModeRsp;

    case blueAPI_EventIdle:
        return pIdle;
    default:
        return unknownString;
    }
} /* blueAPI_CommandString */


/****************************************************************************
 * blueAPI_MCLStatusString()
 ****************************************************************************/
const char * blueAPI_MCLStatusString(TBlueAPI_MCLStatus status)
{
    static const char pMCLIdle[]                    TRACE_DATA = "idle";
    static const char pMCLControlConnected[]        TRACE_DATA = "CtrlCted";
    static const char pMCLDataConnecting[]          TRACE_DATA = "DataCting";
    static const char pMCLDataConnected[]           TRACE_DATA = "DataCted";
    static const char pMCLDataDisconnecting[]       TRACE_DATA = "DataDting";
    static const char pMCLDataListening[]           TRACE_DATA = "DataListen";
    static const char pMCLReleased[]                TRACE_DATA = "released";

    switch (status)
    {
    case blueAPI_MCLIdle:
        return pMCLIdle;
    case blueAPI_MCLControlConnected:
        return pMCLControlConnected;
    case blueAPI_MCLDataConnecting:
        return pMCLDataConnecting;
    case blueAPI_MCLDataConnected:
        return pMCLDataConnected;
    case blueAPI_MCLDataDisconnecting:
        return pMCLDataDisconnecting;
    case blueAPI_MCLDataListening:
        return pMCLDataListening;
    case blueAPI_MCLReleased:
        return pMCLReleased;
    default:
        return unknownString;
    }
} /* blueAPI_MCLStatusString */

/****************************************************************************
 * blueAPI_ACLStatusString()
 ****************************************************************************/
const char * blueAPI_ACLStatusString(TBlueAPI_ACLStatus status)
{
    static const char pACLConnectedActive[]        TRACE_DATA = "conActive";
    static const char pACLAuthenticationStarted[]  TRACE_DATA = "authStarted";
    static const char pACLAuthenticationSuccess[]  TRACE_DATA = "authSuccess";
    static const char pACLAuthenticationFailure[]  TRACE_DATA = "authFail";
    static const char pACLConnectionEncrypted[]    TRACE_DATA = "conEncrypt";
    static const char pACLConnectionDisconnected[] TRACE_DATA = "conDiscon";
    static const char pACLConnectionNotEncrypted[] TRACE_DATA = "conNotEncrypt";
    static const char pACLAddressResolved[]        TRACE_DATA = "AddressResolved";
    static const char pACLConnectedLinkStatus[]    TRACE_DATA = "conLinkStatus";

    switch (status)
    {
    case blueAPI_ACLConnectedActive:
        return pACLConnectedActive;
    case blueAPI_ACLAuthenticationStarted:
        return pACLAuthenticationStarted;
    case blueAPI_ACLAuthenticationSuccess:
        return pACLAuthenticationSuccess;
    case blueAPI_ACLAuthenticationFailure:
        return pACLAuthenticationFailure;
    case blueAPI_ACLConnectionEncrypted:
        return pACLConnectionEncrypted;
    case blueAPI_ACLConnectionDisconnected:
        return pACLConnectionDisconnected;
    case blueAPI_ACLConnectionNotEncrypted:
        return pACLConnectionNotEncrypted;
    case blueAPI_ACLAddressResolved:
        return pACLAddressResolved;
    case blueAPI_ACLConnectedLinkStatus:
        return pACLConnectedLinkStatus;
    default:
        return unknownString;
    }
} /* blueAPI_ACLStatusString */

/****************************************************************************
 * blueAPI_InternalEventTypeString()
 ****************************************************************************/
const char * blueAPI_InternalEventTypeString(TBlueAPI_InternalEventType eventType)
{
    static const char pInternalEventInvalidCreateMDLConf[]      TRACE_DATA = "InvalidCreateMDLConf";
    static const char pInternalEventInvalidDisconnectMDLConf[]  TRACE_DATA = "InvalidDisconnectMDLConf";
    static const char pInternalEventInvalidSecurityConf[]       TRACE_DATA = "InvalidSecurityConf";
    static const char pInternalEventInvalidRemoteEvent[]        TRACE_DATA = "InvalidRemoteEvent";
    static const char pInternalEventCommunicationTimeout[]      TRACE_DATA = "CommunicationTimeout";
    static const char pInternalEventInvalidGATTAttributeReadConf[]   TRACE_DATA = "InvalidGATTAttributeReadConf";
    static const char pInternalEventInvalidGATTAttributeWriteConf[]  TRACE_DATA = "InvalidGATTAttributeWriteConf";
    static const char pInternalEventInvalidGATTDiscoveryConf[]       TRACE_DATA = "InvalidGATTDiscoveryConf";
    static const char pInternalEventInvalidGATTAttributeConf[]       TRACE_DATA = "InvalidGATTAttributeConf";
    static const char pInternalEventInvalidLEConnectionUpdateConf[]  TRACE_DATA = "InvalidLEConnectionUpdateConf";


    switch (eventType)
    {
    case blueAPI_InternalEventInvalidCreateMDLConf:
        return pInternalEventInvalidCreateMDLConf;
    case blueAPI_InternalEventInvalidDisconnectMDLConf:
        return pInternalEventInvalidDisconnectMDLConf;
    case blueAPI_InternalEventInvalidSecurityConf:
        return pInternalEventInvalidSecurityConf;
    case blueAPI_InternalEventInvalidRemoteEvent:
        return pInternalEventInvalidRemoteEvent;
    case blueAPI_InternalEventCommunicationTimeout:
        return pInternalEventCommunicationTimeout;
    case blueAPI_InternalEventInvalidGATTAttributeReadConf:
        return pInternalEventInvalidGATTAttributeReadConf;
    case blueAPI_InternalEventInvalidGATTAttributeWriteConf:
        return pInternalEventInvalidGATTAttributeWriteConf;
    case blueAPI_InternalEventInvalidGATTDiscoveryConf:
        return pInternalEventInvalidGATTDiscoveryConf;
    case blueAPI_InternalEventInvalidGATTAttributeConf:
        return pInternalEventInvalidGATTAttributeConf;
    case blueAPI_InternalEventInvalidLEConnectionUpdateConf:
        return pInternalEventInvalidLEConnectionUpdateConf;
    default:
        return unknownString;
    }
} /* blueAPI_InternalEventTypeString */

/****************************************************************************
 * blueAPI_AuthRequirementsString()
 ****************************************************************************/
const char * blueAPI_AuthRequirementsString(TBlueAPI_AuthRequirements authReq)
{
    static const char pAuthNoMITMRequiredNoStore[]           TRACE_DATA = "NoMITM-NoStore";
    static const char pAuthMITMRequiredNoStore[]             TRACE_DATA = "MITM-NoStore";
    static const char pAuthNoMITMRequiredBonding[]           TRACE_DATA = "NoMITM-Bond";
    static const char pAuthMITMRequiredBonding[]             TRACE_DATA = "MITM-Bond";


    switch (authReq)
    {
    case blueAPI_AuthNoMITMRequiredNoStore:
        return pAuthNoMITMRequiredNoStore;
    case blueAPI_AuthMITMRequiredNoStore:
        return pAuthMITMRequiredNoStore;
    case blueAPI_AuthNoMITMRequiredBonding:
        return pAuthNoMITMRequiredBonding;
    case blueAPI_AuthMITMRequiredBonding:
        return pAuthMITMRequiredBonding;

    default:
        return unknownString;
    }
} /* blueAPI_AuthRequirementsString */

/****************************************************************************
 * blueAPI_IOCapabilitiesString()
 ****************************************************************************/
const char * blueAPI_IOCapabilitiesString(TBlueAPI_IOCapabilities ioCaps)
{
    static const char pIOCapDisplayOnly[]      TRACE_DATA = "DisplayOnly";
    static const char pIOCapDisplayYesNo[]     TRACE_DATA = "DisplayYesNo";
    static const char pIOCapKeyboardOnly[]     TRACE_DATA = "KeyboardOnly";
    static const char pIOCapNoIO[]             TRACE_DATA = "NoIOCaps";
    static const char pIOCapKeyboardDisplay[]  TRACE_DATA = "KeyboardDisplay";

    switch (ioCaps)
    {
    case blueAPI_IOCapDisplayOnly:
        return pIOCapDisplayOnly;
    case blueAPI_IOCapDisplayYesNo:
        return pIOCapDisplayYesNo;
    case blueAPI_IOCapKeyboardOnly:
        return pIOCapKeyboardOnly;
    case blueAPI_IOCapNoIO:
        return pIOCapNoIO;
    case blueAPI_IOCapKeyboardDisplay:
        return pIOCapKeyboardDisplay;
    default:
        return unknownString;
    }
} /* blueAPI_IOCapabilitiesString */

