

#ifndef __PTA_METER_H__
#define __PTA_METER_H__

#include "bt_fw_hci_external_defines.h"
#include "bt_fw_types.h"

/*
#define PTA_EXTENSION

//#ifdef PTA_EXTENSION
#define A2DP_FLUSHABLE
#define FTP_ESTIMATION
#define HID_ESTIMATION
#define PTA_PROFILE_ESTIMATION_OVER_INQUIRY
//#endif
*/
typedef struct  // union
{
	UINT16 	wFHSServiceClass;
	//struct
	//{
 	UCHAR	bInformation_23;		
	UCHAR	bTelephony_22;
	UCHAR	bAudio_21;
	UCHAR    bObjectTransfer_20;
	UCHAR	bCapturing_19;
	UCHAR	bRendering_18;	
	UCHAR    bNetworking_17;
	UCHAR	bPositioning_16;
	UCHAR    bReserved_15;
	UCHAR    bReserved_14;
	UCHAR    bLimitedDiscoverableMode_13;	
	//}
	//PTA_FHS_SERVICE_CLASSES_STRUCT;
}
PTA_FHS_SERVICE_CLASSES_STRUCT;


typedef struct
{
 	UINT16	bMajorDeviceClasses;		
	UCHAR	bMisc;
	UCHAR	bComputer;
	UCHAR	bPhone;
	UCHAR	bLAN_Network;
	UCHAR	bAudio_Video;
	UCHAR	bPeripheral;
	UCHAR 	bImaging;
	UCHAR 	bWearable;
	UCHAR 	bToy;
	UCHAR    bUncategorized;
	UCHAR	bReserved;
}
PTA_FHS_MAJOR_CLASSES_STRUCT;

// depends on the major classes
typedef struct
{
  	UINT8	bMinorDeviceClasses;
	UCHAR	bWearableHeadsetDevice;
	UCHAR	bHandsFreeDevice;
	UCHAR	bMicrophone;
	UCHAR	bLoudspeaker;
	UCHAR    bHeadphones;
	UCHAR	bPortableAudio;
	UCHAR    bCarAudio;
	UCHAR    bSetTopBox;
	UCHAR    bHiFiAudioDevice;
	UCHAR	bVCR;
	UCHAR	bVideoCamera;
	UCHAR	bCamcorder;
	UCHAR	bVideoMonitor;
	UCHAR	bVideoDisplayAndLoudspeaker;
	UCHAR	bVideoConferencing;
	UCHAR	bGaming;
	
}
PTA_FHS_AV_MINOR_CLASSES_STRUCT;


typedef struct
{
	// keyboard mouse
	UINT8	bMinorDeviceClasses;
	UCHAR	bNonKeyboardMouse;
	UCHAR	bKeyboard;
	UCHAR	bMouse;
	UCHAR	bComboKeyboardMouse;
	// 
	UCHAR 	bJoystick;
	UCHAR	bGamepad;
	UCHAR	bRemoteControl;
	UCHAR 	bSensingDevice;
	UCHAR	bDigitizerTablet;
	UCHAR    bCardReader;
}
PTA_FHS_PERIPHERAL_MINOR_CLASSES_STRUCT;

/*
typedef struct
{
 	UINT32	dwLAP;		
	UINT8	bEIR;
	UINT8	bSR;
	UINT8	bUAP;
	UINT16	wNAP;
	UINT32	dwCoD;
	UINT8	bLT_ADDR;	
}
PTA_FHS_FORMAT_STRUCT;
*/


// total 18 bytes
typedef struct
{
 	UINT8	bBD_ADDR[6];		
	UINT8	bEIR;
	UINT8	bSR;
	UINT8	bCoD[3];
	UINT8	bLT_ADDR;	
	UINT32   dwCLK;
	UINT8	bPageScanMode;
}
PTA_FHS_FORMAT_STRUCT;


typedef struct PTA_METER_S_ {    
    UINT32 dwPtaACLTxCnt;
    UINT32 dwPtaACLRxCnt;
    UINT32 dwPtaSCOTxCnt;
    UINT32 dwPtaSCORxCnt;
    UINT32 dwPtaACLTxCntStore;
    UINT32 dwPtaACLRxCntStore;
    UINT32 dwPtaSCOTxCntStore;
    UINT32 dwPtaSCORxCntStore;	 
    UINT16 dwPtaTimer1Counter;
    UINT16 wA2DPSerialNumber;
    UINT16 wA2DPSerialNumber_q;
    UINT16 wHIDSerialNumber;
    UINT16 wHIDSerialNumber_q;
    UINT16 dwTXRetryCnt;
    UINT16 dwTXRetryCntStore;
    UINT16 HID_connection_handle;
    UINT16 A2DP_connection_handle;
    UINT16 FTP_connection_handle;
    UINT8  FTP_Exist;
    UINT8  HID_Exist;
    UINT8  A2DP_Exist;
    UINT8  bPtaMeterSwitch;
    UINT8  bPtaTimer1Counter;
    UINT8  bSBCPtaTimer1Counter;
    UINT8  bHIDDismissCounter;
    UINT8  bFTPDismissCounter;
    UINT8  bA2DPAccumulateCounter;
    UINT8  SBC_Flag;
    UINT8  SBC_Find;
    UINT8  bSBCAccumulateCounter;
    UINT8  bSBCDismissCounter;
    UINT8  A2DP_Find;
    UINT8  bHIDAccumulateCounter;
    UINT8 bConn_q;
    UINT8 bInqPage_q;
    UINT8 bACLBusy_q;
     UINT8 bSCOConn_q;
    UINT8 bSCOBusy_q;
    UINT8 bFTP_q;
    UINT8 bA2DP_q;
    UINT8 bHID_q;
    UINT8 bRetryCntTimer;
    UINT8 bMailboxBtAutoReport;
    UINT16 bRetry_Cnt_Timer_50ms_by_n;
    UINT16 bRetry_Cnt_Threshold;
UINT8 bTXRetryOverRun_flag_q;
UINT8 ReInit_flag;
UINT8 bI2cEnable;
//UINT8 bConn_flag;
//UINT8 bInqPage_flag;
//UINT8 bACLBusy_flag;
//UINT8 bSCOConn_flag;
//UINT8 bSCOBusy_flag;
//UINT8 bFTP_flag;
//UINT8 bA2DP_flag;
//UINT8 bHID_flag;
//UINT8 bTXRetryOverRun_flag;
} PTA_METER_S;

#ifdef PTA_EXTENSION
extern PTA_METER_S pta_meter_var;
extern TimerHandle_t hTimerIDPtaHciMeter;
extern TimerHandle_t hTimerIDPta50msMeter; //20120809 morgan

//20120809 morgan add for profile estimation
#define MAX_SBC_DISMISS_PACKET_NUM 	15
#define MAX_A2DP_ACCUMULATE_NUM		10
#define MAX_HID_DISMISS_PACKET_NUM 	20
#define MAX_FTP_DISMISS_CNT			3
#define FTP_THROUGHPUT_THRESHOLD  		0x4000	//0x1a9c8 //0x2972a 
#define FTP_THROUGHPUT_THRESHOLD_BR  	0x14000	//0x1a9c8 //0x2972a 
#define FTP_THROUGHPUT_THRESHOLD_EDR  	0x1A000	//0x1a9c8 //0x2972a 
//#define RETRY_CNT_TIMER_50ms			10
//#define TXRETRYCNT_THRESHOLD			1
#ifdef PTA_PROFILE_ESTIMATION_OVER_INQUIRY
extern PTA_FHS_FORMAT_STRUCT sPtaFHS;
extern PTA_FHS_SERVICE_CLASSES_STRUCT sPtaFHSServiceClass;
extern PTA_FHS_MAJOR_CLASSES_STRUCT sPtaFHSMajorClass;
extern PTA_FHS_AV_MINOR_CLASSES_STRUCT sPtaFHSAVMinorClass;
extern PTA_FHS_PERIPHERAL_MINOR_CLASSES_STRUCT sPtaFHSPhiMinorClass;
#endif

void fnPtaHciTrafficMeasureHandle(TimerHandle_t timer);
void fnPtaHciMeterInit();
void fnPtaHciMeterDeinit();
void fnPtaRegisterInit(void);

//20120809 morgan add
void fnPtaHci50msHandle(TimerHandle_t timer);
void change_PTA_priority(UINT16 reg_val);

#ifdef PTA_PROFILE_ESTIMATION_OVER_INQUIRY
void fnPtaFhsParser(UCHAR *pbuffer);
void fnGetServiceClasses(PTA_FHS_FORMAT_STRUCT * pFhsFormatData);
void fnGetMajorClasses(PTA_FHS_FORMAT_STRUCT * pFhsFormatData);
void fnGetAVMinorClasses(PTA_FHS_FORMAT_STRUCT * pFhsFormatData);
void fnGetPeriMinorClasses(PTA_FHS_FORMAT_STRUCT * pFhsFormatData);
#endif

#ifdef SOURCECODE_TEST_CLOSE_FW_PROFILE_ESTIMATE

#ifdef A2DP_SBC_SYNCWORD
API_RESULT pta_a2dp_sbc_estimation2(UCHAR *buffer);
#endif

#ifdef A2DP_FLUSHABLE
API_RESULT pta_a2dp_estimation(HCI_ACL_DATA_PKT *ppkt);
#endif

#ifdef FTP_ESTIMATION
API_RESULT pta_ftp_estimation(HCI_ACL_DATA_PKT *ppkt);
#endif

#endif

#ifdef HID_ESTIMATION
void pta_hid_estimation(UINT16 conn_handle);
#endif

#endif

#endif
