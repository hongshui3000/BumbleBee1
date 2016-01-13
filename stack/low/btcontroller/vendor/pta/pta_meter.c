enum { __FILE_NUM__= 126 };

#include "mint_os.h"
#include "bt_fw_os.h"
#include "platform.h"
#include "pta_meter.h"
#include "logger.h"
#include "mem.h"
#include "lmp.h"
#include "le_hci_4_0.h"
#include "lc_internal.h"
#include "mailbox.h"
#include "lmp_ch_assessment.h"
#ifdef _BT_ONLY_
#include "new_io.h"
#endif

TimerHandle_t hTimerIDPtaHciMeter = NULL;
TimerHandle_t hTimerIDPta50msMeter = NULL;

PTA_METER_S pta_meter_var;

#ifdef BT_DRIVER_PROVIDE_PROFILE_INFO 
UINT8 g_host_set_profile;
UINT8 g_driver_report_sco_state;
UINT8 g_driver_report_hid_state;                
UINT8 g_driver_report_a2dp_state;
UINT8 g_driver_report_ftp_state;
UINT16 g_driver_sco_connection_handle;
UINT16 g_driver_hid_connection_handle;
UINT16 g_driver_a2dp_connection_handle;
UINT16 g_driver_ftp_connection_handle;
#endif

#ifdef PTA_PROFILE_ESTIMATION_OVER_INQUIRY
PTA_FHS_FORMAT_STRUCT sPtaFHS;
PTA_FHS_SERVICE_CLASSES_STRUCT sPtaFHSServiceClass;
PTA_FHS_MAJOR_CLASSES_STRUCT sPtaFHSMajorClass;
PTA_FHS_AV_MINOR_CLASSES_STRUCT sPtaFHSAVMinorClass;
PTA_FHS_PERIPHERAL_MINOR_CLASSES_STRUCT sPtaFHSPhiMinorClass;
#endif

UINT8 bEnableIgnoreWlanActive = 1;

/*****************************************************************/
/*                      morgan put here temporary                         */
/*****************************************************************/

#ifdef PTA_EXTENSION

void change_PTA_priority(UINT16 reg_val)
{
	UINT8 i;
    for (i = 0; i < 4; i++)
    {
        UINT16 reg_addr = 0x212 + ( i << 1 );
        BB_write_baseband_register(reg_addr, reg_val);
        //RT_BT_LOG(WHITE, DAPE_TEST_LOG213, 2, reg_addr, reg_val);
    }
}

void fnPtaHci50msHandle(TimerHandle_t timer)
{

#ifdef SOURCECODE_TEST_CLOSE_FW_PROFILE_ESTIMATE
    if (pta_meter_var.SBC_Find || pta_meter_var.A2DP_Find)
    {

        //RT_BT_LOG(RED,PTA_DBG_SBC_GENERAL_DISPLAY,2,4,pta_meter_var.bSBCDismissCounter);
        if  ( pta_meter_var.wA2DPSerialNumber == pta_meter_var.wA2DPSerialNumber_q )
        { 
            pta_meter_var.bSBCDismissCounter ++;
        }		
        else
        {
            pta_meter_var.bSBCDismissCounter=0;
        }

        if ( pta_meter_var.bSBCDismissCounter >= MAX_SBC_DISMISS_PACKET_NUM )
        {
            pta_meter_var.A2DP_Exist = 0;
            pta_meter_var.A2DP_connection_handle = 0xffff;
            pta_meter_var.SBC_Find = 0;
            pta_meter_var.A2DP_Find = 0;
            pta_meter_var.bSBCDismissCounter = 0;				
        }	   
    }
    //RT_BT_LOG(GRAY, PTA_HCI_TD_RX_TRANSPORT,2,pta_meter_var.wA2DPSerialNumber_q, pta_meter_var.wA2DPSerialNumber);

    pta_meter_var.wA2DPSerialNumber_q = pta_meter_var.wA2DPSerialNumber;
#endif

    // 20120813 add for BT status auto report status transittion check

    // ========================================
    UINT8	bTXRetryOverRun_flag = 0;
    pta_meter_var.bRetryCntTimer ++;
    if ( pta_meter_var.bRetryCntTimer >= pta_meter_var.bRetry_Cnt_Timer_50ms_by_n )
    {
        pta_meter_var.bRetryCntTimer = 0;
        if ( pta_meter_var.dwTXRetryCnt >= pta_meter_var.bRetry_Cnt_Threshold )
        {
            bTXRetryOverRun_flag = 1;
        }
	// 20120906 morgan add
	else
	{
		bTXRetryOverRun_flag = 0;
	}
        pta_meter_var.dwTXRetryCntStore = pta_meter_var.dwTXRetryCnt;	
        pta_meter_var.dwTXRetryCnt = 0;
    }
    //========================================	


    if ( pta_meter_var.bMailboxBtAutoReport && pta_meter_var.bI2cEnable )
    {
        UINT8 bInqPage_flag;
        UINT8 bSCOConn_flag;    
        UINT8 bConn_flag;
        UINT8 bACLBusy_flag;
        UINT8 bSCOBusy_flag;
        UINT8 bA2DP_flag;
        UINT8 bFTP_flag;
        UINT8 bHID_flag;
        
        bInqPage_flag = (( (lmp_self_device_data.lc_cur_dev_state) & 0x03) ==0)?0:1;
        bSCOConn_flag = ((lmp_self_device_data.number_of_esco_connections + lmp_self_device_data.total_no_of_sco_conn)>0)?1:0;
        bConn_flag =  ( lmp_self_device_data.number_of_acl_conn > 0 )? 1:0;		
        bACLBusy_flag =  ((pta_meter_var.dwPtaACLTxCnt + pta_meter_var.dwPtaACLRxCnt+ pta_meter_var.dwPtaACLTxCntStore + pta_meter_var.dwPtaACLRxCntStore) > 0x05 ) ? 1:0;	// means ACL has transport
        bSCOBusy_flag = ((pta_meter_var.dwPtaSCOTxCnt + pta_meter_var.dwPtaSCORxCnt + pta_meter_var.dwPtaSCOTxCntStore + pta_meter_var.dwPtaSCORxCntStore) > 0x05 )?1:0;
        bA2DP_flag = pta_meter_var.A2DP_Exist;
        bFTP_flag = pta_meter_var.FTP_Exist;
        bHID_flag = pta_meter_var.HID_Exist;

        if ((bInqPage_flag != pta_meter_var.bInqPage_q) 	||
            (bConn_flag != pta_meter_var.bConn_q)			||
            (bACLBusy_flag != pta_meter_var.bACLBusy_q) 	||
            (bSCOConn_flag != pta_meter_var.bSCOConn_q)	||
            (bSCOBusy_flag != pta_meter_var.bSCOBusy_q)	||
            (bFTP_flag != pta_meter_var.bFTP_q)			||
            (bA2DP_flag != pta_meter_var.bA2DP_q)			||
            (bTXRetryOverRun_flag != pta_meter_var.bTXRetryOverRun_flag_q )	||
            //(bTXRetryOverRun_flag ==1 )		||
            (bHID_flag != pta_meter_var.bHID_q))
        {
            mailbox_bt_report_info(0x27);
            
            RT_BT_LOG(YELLOW, PTA_DISPLAY_BT_INFO_MESSAGE, 12,
                bConn_flag, bSCOConn_flag,
                bInqPage_flag,
                pta_meter_var.dwPtaACLRxCnt+pta_meter_var.dwPtaACLTxCnt,
                pta_meter_var.dwPtaSCORxCnt+pta_meter_var.dwPtaSCOTxCnt,
                pta_meter_var.dwTXRetryCntStore, pta_meter_var.HID_Exist,
                pta_meter_var.A2DP_Exist, pta_meter_var.FTP_Exist,1,1,1);
        }        
        pta_meter_var.bInqPage_q = bInqPage_flag;
        pta_meter_var.bConn_q = bConn_flag;
        pta_meter_var.bACLBusy_q = bACLBusy_flag;
        pta_meter_var.bSCOConn_q = bSCOConn_flag;
        pta_meter_var.bSCOBusy_q = bSCOBusy_flag;
        pta_meter_var.bFTP_q = bFTP_flag;
        pta_meter_var.bA2DP_q = bA2DP_flag;
        pta_meter_var.bHID_q = bHID_flag;
        pta_meter_var.bTXRetryOverRun_flag_q = bTXRetryOverRun_flag;
    }
}

void fnPtaHciTrafficMeasureHandle(TimerHandle_t timer)
{
    UINT16 wReTxSlotCnt;
    UINT16 wTxSlotCnt;
    UINT16 wRxSlotCnt;
    UINT16 wSlotTemp;

    // here report tx rx retry counter
    wSlotTemp = BB_read_baseband_register(0x1f8); // check tx rx retry counter
    wReTxSlotCnt = ( wSlotTemp >> 8 );

    wSlotTemp = BB_read_baseband_register(0x1fA);
    wRxSlotCnt = 	 wSlotTemp & 0x00FF; 			// check tx rx retry counter	
    wTxSlotCnt = ((wSlotTemp & 0xFF00 )>>8); 	// check tx rx retry counter

    // here is the timer function
    //======  2 sec timer ==========	

    // for HID check
    // clear counter
    pta_meter_var.bHIDAccumulateCounter = 0;

    pta_meter_var.wHIDSerialNumber_q= pta_meter_var.wHIDSerialNumber;
	if (  pta_meter_var.HID_Exist )
	{
		if  ( pta_meter_var.wHIDSerialNumber == pta_meter_var.wHIDSerialNumber_q ) // pta_meter_var.SBC_Flag != 1 
		{ 
			pta_meter_var.bHIDDismissCounter ++;
		}		
		else
		{
			pta_meter_var.bHIDDismissCounter=0;
		}
		
		if ( pta_meter_var.bHIDDismissCounter >= MAX_HID_DISMISS_PACKET_NUM ) // ||bA2DPDismissCounter >=  MAX_SBC_DISMISS_PACKET_NUM)
		{
            pta_meter_var.HID_Exist = 0;
            BB_write_baseband_register( 0x190, 0x0000);
            pta_meter_var.HID_connection_handle = 0xffff;			
            pta_meter_var.bHIDDismissCounter = 0;			
			bHIDCount = 0;
		}	   
   	}
	//RT_BT_LOG(GRAY, PTA_HCI_TD_RX_TRANSPORT,2,pta_meter_var.wA2DPSerialNumber_q, pta_meter_var.wA2DPSerialNumber);
	 //UINT32 RegValue_temp;
	 //RegValue_temp = VENDOR_READ(MAILBOX_CTL_REG);	
	 //RT_BT_LOG(GREEN, PTA_DISPLAY_REGISTER_MESSAGE,2,MAILBOX_CTL_REG,RegValue_temp);	
	//mailbox_bt_report_info(); 
/*
	if ( (pta_meter_var.dwPtaACLTxCnt + pta_meter_var.dwPtaACLRxCnt+ pta_meter_var.dwPtaACLTxCntStore + pta_meter_var.dwPtaACLRxCntStore) > FTP_THROUGHPUT_THRESHOLD )
	{
		pta_meter_var.FTP_Exist= 1;		
	}
	else
	{
		pta_meter_var.bFTPDismissCounter ++;
	}
*/
#ifdef SOURCECODE_TEST_CLOSE_FW_PROFILE_ESTIMATE
	if ( a2dp_rate ==0 && pta_meter_var.A2DP_Exist) // edr
	{
		if ( ( pta_meter_var.dwPtaACLTxCntStore + pta_meter_var.dwPtaACLRxCntStore) > FTP_THROUGHPUT_THRESHOLD_EDR )		
		{
			pta_meter_var.FTP_Exist= 1;		
		}		
		else
		{
			pta_meter_var.bFTPDismissCounter ++;
		}
	}
	else if ( a2dp_rate ==1  && pta_meter_var.A2DP_Exist)//br
	{
		if ( ( pta_meter_var.dwPtaACLTxCntStore + pta_meter_var.dwPtaACLRxCntStore) > FTP_THROUGHPUT_THRESHOLD_BR )		
		{
			pta_meter_var.FTP_Exist= 1;		
		}
		else
		{
			pta_meter_var.bFTPDismissCounter ++;
		}	
	}
	else
	{
		if ( ( pta_meter_var.dwPtaACLTxCntStore + pta_meter_var.dwPtaACLRxCntStore) > FTP_THROUGHPUT_THRESHOLD )		
		{
			pta_meter_var.FTP_Exist= 1;		
		}		
		else
		{
			pta_meter_var.bFTPDismissCounter ++;
		}
	}
	

	if ( ( pta_meter_var.FTP_Exist &&  (pta_meter_var.bFTPDismissCounter == MAX_FTP_DISMISS_CNT)) || 
          (pta_meter_var.bFTPDismissCounter >= (MAX_FTP_DISMISS_CNT + 1) ) )
	{
		pta_meter_var.FTP_Exist = 0;
		pta_meter_var.bFTPDismissCounter = 0;
	}
#endif 


#ifdef BT_DBG_REPORT
//start of debug report =========================================================================================
    UINT8 bConnection = 0;
    UINT8 bSCOeSCO = 0;
    UINT8 bInQPage = 0;
    UINT8 bACLConn = 0; 
    UINT8 bSCOBusy = 0;
    UINT16 bHID = 0;
    UINT16 bA2DP = 0;
    UINT16 bFTP = 0;
    UINT16 retry_count;

    if (  lmp_self_device_data.number_of_acl_conn >0 )
    {
        bConnection = 1;
        if  ((pta_meter_var.dwPtaACLTxCnt + pta_meter_var.dwPtaACLRxCnt+ pta_meter_var.dwPtaACLTxCntStore + pta_meter_var.dwPtaACLRxCntStore) > 0x05 ) // means ACL has transport
        {
                bACLConn = 1;                   
        }
    }

    if ((lmp_self_device_data.number_of_esco_connections > 0) ||
        (lmp_self_device_data.total_no_of_sco_conn > 0))
    { 
        bSCOeSCO = 1;
        bConnection = 1;                            
        if ((pta_meter_var.dwPtaSCOTxCnt + pta_meter_var.dwPtaSCORxCnt + pta_meter_var.dwPtaSCOTxCntStore + pta_meter_var.dwPtaSCORxCntStore) > 0x05 )
        {
             bSCOBusy = 1;
        }
    }

    if ( lmp_self_device_data.lc_cur_dev_state != 0 )
    {
        bInQPage = 1;
    }

        UINT16 bCeIndex;

    if (pta_meter_var.HID_Exist)
    {
		 bHID = 1;
        if (LMP_GET_CE_INDEX_FROM_CONN_HANDLE(pta_meter_var.HID_connection_handle,&bCeIndex) != API_SUCCESS )                             
        {
            // no connection exist
            pta_meter_var.HID_Exist = 0;
            BB_write_baseband_register( 0x190, 0x0000);
            pta_meter_var.HID_connection_handle = 0xFFFF;
			bHID = 0;
        }
    }   

    //pta_meter_var.A2DP_Exist handle by A2DP estimation
    bA2DP = pta_meter_var.A2DP_Exist;

    if (bA2DP)
    {
        if (LMP_GET_CE_INDEX_FROM_CONN_HANDLE(pta_meter_var.A2DP_connection_handle,&bCeIndex) != API_SUCCESS )                                
        {
            // no connection exist
            pta_meter_var.A2DP_Exist = 0;
            pta_meter_var.A2DP_connection_handle = 0xFFFF;
			bA2DP = 0;
        }
    }                   

	// FTP
	bFTP = pta_meter_var.FTP_Exist;
    retry_count = pta_meter_var.dwTXRetryCntStore;
    RT_BT_LOG(YELLOW, PTA_DISPLAY_BT_INFO_MESSAGE, 12, bConnection, bSCOeSCO, bInQPage, pta_meter_var.dwPtaACLRxCnt+pta_meter_var.dwPtaACLTxCnt, pta_meter_var.dwPtaSCORxCnt+pta_meter_var.dwPtaSCOTxCnt,pta_meter_var.dwTXRetryCntStore, pta_meter_var.HID_Exist, pta_meter_var.A2DP_Exist, pta_meter_var.FTP_Exist,wReTxSlotCnt,wTxSlotCnt,wRxSlotCnt); 						    
//end of debug report =========================================================================================
#endif

        //pta_meter_var.dwTXRetryCntStore = pta_meter_var.dwTXRetryCnt;		
    pta_meter_var.dwPtaACLTxCntStore = pta_meter_var.dwPtaACLTxCnt;
    pta_meter_var.dwPtaACLRxCntStore = pta_meter_var.dwPtaACLRxCnt;
    pta_meter_var.dwPtaSCOTxCntStore = pta_meter_var.dwPtaSCOTxCnt;
    pta_meter_var.dwPtaSCORxCntStore = pta_meter_var.dwPtaSCORxCnt;  

    pta_meter_var.dwPtaACLTxCnt = 0;
    pta_meter_var.dwPtaACLRxCnt = 0;
    pta_meter_var.dwPtaSCOTxCnt = 0;
    pta_meter_var.dwPtaSCORxCnt = 0;  
        //pta_meter_var.dwTXRetryCnt = 0;
    pta_meter_var.dwPtaTimer1Counter = 0;

}



void fnPtaRegisterInit()
{
// ============================================//
// ASIC 8821a test setting ( 20120801 )
// ============================================//
// morgan add register define description
// 0xfa : tx_confirm_delay
// 0x1a6 : "bit 9:8 " "1" : 3 wire mode , "2" : RTK mode
// 0x1a6 : "bit 5 " : reserved set bit 5 = 1	( ignore wlan act )

// 0xf4 : bit 0 : pta enable , bit1~6 , pta_ctrl , bit7 : pri_mask_select

// 0x1f8 : bit 0 : pta meter enable , bit 1:6 : pta meter packet counter ( used for meter period )
// 0x1f8 : bit 8~15 : retx slot cnt ( total retry packet counter )
// 0x1fa : bit 0~7 : rx slot cnt  bit8~15 : tx slot cnt

// 0x1fe : for debug test only ( not use in ASIC )

// 0x190 : bit 3 ~13 : hid_sign1 ~ hid_sign11 ( hid_sign1~7 : master role , connection , 
// 0x19E : bit8 : hid_sign ( not use , change to 0x190 )

// table switch use  0xf4   bit7 : pri_mask_select : "0" :  0x212 0x214  "1" : 0x216 0x218
// Normal table of PTA 
// 0x212 : TX
// 0x214 : RX
// TDMA table of PTA
// 0x216 : TX
// 0x218 : RX

// table : default value - 0x1ff
// 15 --- 14 --- 13 --- 12 --- 11 ---10 ---9 ---8 ---7 ---6 --- 5 --- 4 -----3------ 2 ----- 1 ----- 0
// rsv    le_ce   le_scan le_ini  le_adv   hid    rsv   esco  sco   mss   sniff   park  page_scan  page  inq_scan   inq

    BB_write_baseband_register(0xf4, 0x01);
    BB_write_baseband_register(0x1f8, 0x7F); // enable pta retry meter , total check counter = 0x3f ( 63 )
    // 0x1a6 set in pta_init = 0x220 
/*
    UINT16 temp_reg_data;     		
    temp_reg_data =BB_read_baseband_register(0x1a6);   // ( set RTK mode  )
    temp_reg_data = temp_reg_data | 0x0200;  // set bit 9 = 1 
    temp_reg_data = temp_reg_data & 0xFEFF;  // set bit 8 = 0
	//temp_reg_data = temp_reg_data | 0x0020;  // set bit 5 = 1	( ignore wlan act )
	temp_reg_data = temp_reg_data & 0xFFDF;  // set bit 5 = 1	( ignore wlan act )
    BB_write_baseband_register(0x1a6, temp_reg_data);
*/		
	BB_write_baseband_register(0x216, 0xffa);
	BB_write_baseband_register(0x218, 0xffa);
	BB_write_baseband_register(0x212, 0xffa);
	BB_write_baseband_register(0x214, 0xffa);		

#ifdef _BT_ONLY_
//20121119 morgan add for I2C enable
	// write system on area enable I2C
	UINT32 pta_i2c_enable = 0;

	// indirect read
	//indirect_access_read_syson_reg(0x42,0);
	pta_i2c_enable = (RD_32BIT_SYSON_IO(0x40)) | 0x00800000;

	// indirect write
	//indirect_access_write_syson_reg
	WR_32BIT_SYSON_IO(0x40,pta_i2c_enable);
#endif
	
// ============================================//
// FPGA test setting
// ============================================//       
//20120216_for_pta_debug port
/***** PTA debug area ******/
/*
      	VENDOR_WRITE(0x30, (VENDOR_READ(0x30)&BIT31) | 0x40);
      	BB_write_baseband_register(0x1fe, 0x26);
*/

// pta enable
//    	BB_write_baseband_register(0xf4, 0x01);
//       BB_write_baseband_register(0x7a, 0x10);
/*
     	BB_write_baseband_register(0xfa, 0x70);
	UINT16 temp_reg_data;     		
	temp_reg_data =BB_read_baseband_register(0x1a6);
	temp_reg_data = temp_reg_data | 0x0200;  // set bit 9 = 1 
	temp_reg_data = temp_reg_data & 0xFEFF;  // set bit 8 = 0
	temp_reg_data = temp_reg_data | 0x0020;  // set bit 5 = 1	( ignore wlan act )
    	BB_write_baseband_register(0x1a6, temp_reg_data);

	//UINT16 wdata;
        //wdata = ((in_data[0]>>16)&0xFF) ? 0x100 : 0x000;   
       BB_write_baseband_register(0x19E, 0x100);                        
	BB_write_baseband_register(0x1fe, 0x09); // change debug port to BT TX and RX
	BB_write_baseband_register(0x1f8, 0x7F); // check tx rx retry counter
*/			 
}

void fnPtaHciMeterVarInit(void)
{
    memset(&pta_meter_var, 0, sizeof(PTA_METER_S));
    pta_meter_var.bPtaMeterSwitch = 1;
    pta_meter_var.HID_connection_handle = 0xFFFF;
    pta_meter_var.A2DP_connection_handle = 0xFFFF;
    //pta_meter_var.FTP_connection_handle = 0xFFFF;
    pta_meter_var.bRetry_Cnt_Threshold=3;
    pta_meter_var.bRetry_Cnt_Timer_50ms_by_n=40;     
    pta_meter_var.ReInit_flag=1;
    pta_meter_var.bMailboxBtAutoReport=0;
	pta_meter_var.bI2cEnable=0;

    // AFH
    // 20120927 morgan add for afh channel recover range
    bBelowLow_good_channel = 24;
    bLow_good_channel = 32;
    bLowMiddle_good_channel = 40;
    bMiddle_good_channel = 46;
    bLowUpper_good_channel = 52;
    bUpper_good_channel = 59;
    bFullRandomRecover = 0;	
    bFastFullOn = 1;
    bBtPsdMode =3;   
}

void fnPtaHciMeterInit(void)
{
    fnPtaHciMeterVarInit();

    if (IS_USE_PTA_METER)
    {
        if (hTimerIDPtaHciMeter != NULL)
        {
            OS_DELETE_TIMER(&hTimerIDPtaHciMeter);
        }

        //RT_BT_LOG(RED, PTA_DISPLAY_MESSAGE, 1,1);

        OS_CREATE_TIMER(PERIODIC_TIMER, &hTimerIDPtaHciMeter,
                fnPtaHciTrafficMeasureHandle, NULL, 0);

        // 2nd timer function
        if (hTimerIDPta50msMeter != NULL )
        {
            OS_DELETE_TIMER(&hTimerIDPta50msMeter);
        }

        OS_CREATE_TIMER(PERIODIC_TIMER, &hTimerIDPta50msMeter,
                fnPtaHci50msHandle, NULL, 0);

#ifdef _CCH_RETENTION_FLOW_FOR_DLPS_
        if(sleep_mode_param.dlps_restore_flow == FALSE)
#endif
        {
            RT_BT_LOG(RED, PTA_DISPLAY_MESSAGE3, 2, hTimerIDPtaHciMeter,
                                                hTimerIDPta50msMeter);
        }

        OS_START_TIMER(hTimerIDPtaHciMeter, 2000);
        OS_START_TIMER(hTimerIDPta50msMeter, 50);			 

        fnPtaRegisterInit();        
    }	
}


void fnPtaHciMeterDeinit(void)
{
    fnPtaHciMeterVarInit();    

    if (hTimerIDPtaHciMeter != NULL)
    {
        OS_DELETE_TIMER(&hTimerIDPtaHciMeter);
    }	

    if (hTimerIDPta50msMeter != NULL)
    {
        OS_DELETE_TIMER(&hTimerIDPta50msMeter);
    }		
}


#ifdef PTA_PROFILE_ESTIMATION_OVER_INQUIRY

void fnPtaFhsParser(UCHAR * pFhsBuffer)
{

	// 20111027 morgan display FHS information
	// FHS total = 18 byte
	//PTA_DISPLAY_FHS_MESSAGE 12006//"<PTA FHS> display FHS message = 0x%x"
	//sPtaFHS.bUAP =   *((fhs_pkt_buffer->fhs_pkt)+8);
	//sPtaFHS.wNAP =   (UINT16)(*((fhs_pkt_buffer->fhs_pkt)+9))<<8 |  *((fhs_pkt_buffer->fhs_pkt)+10);

	sPtaFHS.bBD_ADDR[0] = (*(pFhsBuffer+4))<<2 | (*(pFhsBuffer+5))>>6; 	 //LAP  // LSB
	sPtaFHS.bBD_ADDR[1] = (*(pFhsBuffer+5))<<2 | (*(pFhsBuffer+6))>>6; ; 	// LAP
	sPtaFHS.bBD_ADDR[2] = (*(pFhsBuffer+6))<<2 | (*(pFhsBuffer+7))>>6; ; 	// LAP
	
	sPtaFHS.bBD_ADDR[3] = *(pFhsBuffer+8); 	// UAP
	sPtaFHS.bBD_ADDR[4] = *(pFhsBuffer+9); 	// NAP
	sPtaFHS.bBD_ADDR[5] = *(pFhsBuffer+10); 	// NAP MSB
	
	sPtaFHS.bEIR =  		(*(pFhsBuffer+7)>>2)&0x01;
	sPtaFHS.bSR =   		*(pFhsBuffer+7)>>4;
	sPtaFHS.bCoD[0] = 	*(pFhsBuffer+11);
	sPtaFHS.bCoD[1] = 	*(pFhsBuffer+12); 		
	sPtaFHS.bCoD[2] = 	*(pFhsBuffer+13);
	sPtaFHS.bLT_ADDR =  	*(pFhsBuffer+14)&0x07;
	//sPtaFHS.dwCoD =  ((UINT32)(*((fhs_pkt_buffer->fhs_pkt)+11))) |  ((UINT32)(*((fhs_pkt_buffer->fhs_pkt)+12))<<8 ) | ((UINT32)(*((fhs_pkt_buffer->fhs_pkt)+13))<<16);		
	

	RT_BT_LOG(YELLOW, PTA_DISPLAY_FHS_MESSAGE, 10, sPtaFHS.bEIR ,sPtaFHS.bBD_ADDR[5],sPtaFHS.bBD_ADDR[4],
		sPtaFHS.bBD_ADDR[3], sPtaFHS.bBD_ADDR[2], sPtaFHS.bBD_ADDR[1],sPtaFHS.bBD_ADDR[0], 
		sPtaFHS.bCoD[2],sPtaFHS.bCoD[1],sPtaFHS.bCoD[0], sPtaFHS.bLT_ADDR);

/*			
	RT_BT_LOG(YELLOW, PTA_DISPLAY_FHS_MESSAGE, 10, sPtaFHS.bEIR ,sPtaFHS.bSR,*(pFhsBuffer+10),
	 *(pFhsBuffer+9),sPtaFHS.bUAP, *(pFhsBuffer+11), *(pFhsBuffer+12), *(pFhsBuffer+13),sPtaFHS.bLT_ADDR);
*/

#ifdef PTA_PROFILE_ESTIMATION_OVER_INQUIRY
	fnGetServiceClasses(&sPtaFHS);
	fnGetMajorClasses(&sPtaFHS);		

/*
	// 20111027 morgan display FHS information
	// FHS total = 18 byte
	//PTA_DISPLAY_FHS_MESSAGE 12006//"<PTA FHS> display FHS message = 0x%x"
	//sPtaFHS.dwLAP = *((fhs_pkt_buffer->fhs_pkt)+2),		
	sPtaFHS.bEIR =  (*((fhs_pkt_buffer->fhs_pkt)+7)>>2)&0x01;
	sPtaFHS.bSR =   *((fhs_pkt_buffer->fhs_pkt)+7)>>4;
	sPtaFHS.bUAP =   *((fhs_pkt_buffer->fhs_pkt)+8);
	sPtaFHS.wNAP =   (UINT16)(*((fhs_pkt_buffer->fhs_pkt)+9))<<8 |  *((fhs_pkt_buffer->fhs_pkt)+10);
	sPtaFHS.dwCoD =  ((UINT32)(*((fhs_pkt_buffer->fhs_pkt)+11))) |  ((UINT32)(*((fhs_pkt_buffer->fhs_pkt)+12))<<8 ) | ((UINT32)(*((fhs_pkt_buffer->fhs_pkt)+13))<<16);		
	sPtaFHS.bLT_ADDR =  *((fhs_pkt_buffer->fhs_pkt)+14)&0x07;					
	//"<PTA FHS> display FHS message EIR=0x%x SR=0x%x UAP=0x%x NAP=0x%x CoD=0x%x LT_ADDR=0x%x"			
	RT_BT_LOG(YELLOW, PTA_DISPLAY_FHS_MESSAGE, 10, sPtaFHS.bEIR ,sPtaFHS.bSR,*((fhs_pkt_buffer->fhs_pkt)+10),
	 *((fhs_pkt_buffer->fhs_pkt)+9),sPtaFHS.bUAP, *((fhs_pkt_buffer->fhs_pkt)+11), *((fhs_pkt_buffer->fhs_pkt)+12), *((fhs_pkt_buffer->fhs_pkt)+13),sPtaFHS.bLT_ADDR);
*/
// major service class	
	if ((*(pFhsBuffer+13) & 0x01) == 0x01 )	// bit 16
	{
		RT_BT_LOG(YELLOW, PTA_DISPLAY_FHS_SERVICE_MESSAGE_POSITION,0,0); 
	}
	else if (( *(pFhsBuffer+13) & 0x02) == 0x02 ) // bit 17
	{
		RT_BT_LOG(YELLOW, PTA_DISPLAY_FHS_SERVICE_MESSAGE_NETWORK,0,0); 
	}
	else if ( (*(pFhsBuffer+13) & 0x04) == 0x04 )// bit 18
	{
		RT_BT_LOG(YELLOW, PTA_DISPLAY_FHS_SERVICE_MESSAGE_RENDERING,0,0); 
	}
	else if ( (*(pFhsBuffer+13) & 0x08) == 0x08 )	// bit 19
	{
		RT_BT_LOG(YELLOW, PTA_DISPLAY_FHS_SERVICE_MESSAGE_CAPTURE,0,0); 
	}
	else if ( (*(pFhsBuffer+13) & 0x10) == 0x10 ) // bit 20
	{
		RT_BT_LOG(YELLOW, PTA_DISPLAY_FHS_SERVICE_MESSAGE_OBJTRANSFER,0,0); 
	}
	else if ( (*(pFhsBuffer+13) & 0x20) == 0x20 ) // bit 21
	{
		RT_BT_LOG(YELLOW, PTA_DISPLAY_FHS_SERVICE_MESSAGE_AUDIO,0,0); 
	}		
	else if ( (*(pFhsBuffer+13) & 0x40) == 0x40 ) // bit 22
	{
		RT_BT_LOG(YELLOW, PTA_DISPLAY_FHS_SERVICE_MESSAGE_TELEPHONY,0,0); 
	}
	else if ( (*(pFhsBuffer+13) & 0x80 )== 0x80 ) // bit 23
	{
		RT_BT_LOG(YELLOW, PTA_DISPLAY_FHS_SERVICE_MESSAGE_INFORMATION,0,0); 
	}
	else
	{
		RT_BT_LOG(YELLOW, PTA_DISPLAY_FHS_MESSAGE_UNKNOWN,0,0); 
	}


// major device class
	if ( (*(pFhsBuffer+12) & 0x1F) == 0x01 )
	{
		RT_BT_LOG(GREEN, PTA_DISPLAY_FHS_MESSAGE_COMPUTER,0,0); 
	}

	else if ( (*(pFhsBuffer+12) & 0x1F )== 0x02 )
	{
		RT_BT_LOG(GREEN, PTA_DISPLAY_FHS_MESSAGE_PHONE,0,0); 
	}

	else if (  (*(pFhsBuffer+12) & 0x1F )== 0x03 )
	{
		RT_BT_LOG(GREEN, PTA_DISPLAY_FHS_MESSAGE_LAN,0,0); 
	}
	else if (( *(pFhsBuffer+12) & 0x1F )== 0x04 )
	{
		RT_BT_LOG(GREEN, PTA_DISPLAY_FHS_MESSAGE_AUDIOVIDEO,0,0); 
	}

	else if ( (*(pFhsBuffer+12) & 0x1F) == 0x05 )
	{
		RT_BT_LOG(GREEN, PTA_DISPLAY_FHS_MESSAGE_PERIPHERAL,0,0); 
	}

	else if (  (*(pFhsBuffer+12) & 0x1F )== 0x06 )
	{
		RT_BT_LOG(GREEN, PTA_DISPLAY_FHS_MESSAGE_IMAGE,0,0); 
	}
	else if ( (*(pFhsBuffer+12) & 0x1F) == 0x08 )
	{
		RT_BT_LOG(GREEN, PTA_DISPLAY_FHS_MESSAGE_TOY,0,0); 
	}
	else
	{
		RT_BT_LOG(GREEN, PTA_DISPLAY_FHS_MESSAGE_UNKNOWN,0,0); 
	}
#endif		
}


void fnGetServiceClasses(PTA_FHS_FORMAT_STRUCT * pFhsFormatData)
{
		
	//bMinorDeviceClasses = ((UINT16)(pFhsFormatData->bCoD[2]))<<8 | ((UINT16)(pFhsFormatData->bCoD[1]));
	//clear sPtaFHSServiceClass		
	sPtaFHSServiceClass.bInformation_23 = 0;
	sPtaFHSServiceClass.bTelephony_22 = 0;
	sPtaFHSServiceClass.bAudio_21 = 0;
	sPtaFHSServiceClass.bObjectTransfer_20= 0;
	sPtaFHSServiceClass.bCapturing_19= 0;
	sPtaFHSServiceClass.bRendering_18= 0;	
	sPtaFHSServiceClass.bNetworking_17= 0;
	sPtaFHSServiceClass.bPositioning_16= 0;		
	sPtaFHSServiceClass.bReserved_15= 0;
	sPtaFHSServiceClass.bReserved_14= 0;
	sPtaFHSServiceClass.bLimitedDiscoverableMode_13= 0;	

	sPtaFHSServiceClass.bInformation_23 = (pFhsFormatData->bCoD[2])& 0x80;
	sPtaFHSServiceClass.bTelephony_22 = (pFhsFormatData->bCoD[2])& 0x40;
	sPtaFHSServiceClass.bAudio_21 = (pFhsFormatData->bCoD[2])& 0x20;
	sPtaFHSServiceClass.bObjectTransfer_20= (pFhsFormatData->bCoD[2])& 0x10;
	sPtaFHSServiceClass.bCapturing_19= (pFhsFormatData->bCoD[2])& 0x08;
	sPtaFHSServiceClass.bRendering_18= (pFhsFormatData->bCoD[2])& 0x04;	
	sPtaFHSServiceClass.bNetworking_17= (pFhsFormatData->bCoD[2])& 0x02;
	sPtaFHSServiceClass.bPositioning_16= (pFhsFormatData->bCoD[2])& 0x01;		
	sPtaFHSServiceClass.bReserved_15= (pFhsFormatData->bCoD[1])& 0x80;
	sPtaFHSServiceClass.bReserved_14= (pFhsFormatData->bCoD[1])& 0x40;
	sPtaFHSServiceClass.bLimitedDiscoverableMode_13= (pFhsFormatData->bCoD[1])& 0x20;	


	if (sPtaFHSServiceClass.bPositioning_16 == 0x01 )	// bit 16
	{
		RT_BT_LOG(YELLOW, PTA_DISPLAY_FHS_SERVICE_MESSAGE_POSITION,0,0); 
	}
	else if ( sPtaFHSServiceClass.bNetworking_17 == 0x02 ) // bit 17
	{
		RT_BT_LOG(YELLOW, PTA_DISPLAY_FHS_SERVICE_MESSAGE_NETWORK,0,0); 
	}
	else if ( sPtaFHSServiceClass.bRendering_18 == 0x04 )// bit 18
	{
		RT_BT_LOG(YELLOW, PTA_DISPLAY_FHS_SERVICE_MESSAGE_RENDERING,0,0); 
	}
	else if (sPtaFHSServiceClass.bCapturing_19== 0x08 )	// bit 19
	{
		RT_BT_LOG(YELLOW, PTA_DISPLAY_FHS_SERVICE_MESSAGE_CAPTURE,0,0); 
	}
	else if ( sPtaFHSServiceClass.bObjectTransfer_20 == 0x10 ) // bit 20
	{
		RT_BT_LOG(YELLOW, PTA_DISPLAY_FHS_SERVICE_MESSAGE_OBJTRANSFER,0,0); 
	}
	else if (sPtaFHSServiceClass.bAudio_21 == 0x20 ) // bit 21
	{
		RT_BT_LOG(YELLOW, PTA_DISPLAY_FHS_SERVICE_MESSAGE_AUDIO,0,0); 
	}		
	else if ( sPtaFHSServiceClass.bTelephony_22  == 0x40 ) // bit 22
	{
		RT_BT_LOG(YELLOW, PTA_DISPLAY_FHS_SERVICE_MESSAGE_TELEPHONY,0,0); 
	}
	else if ( sPtaFHSServiceClass.bInformation_23 == 0x80 ) // bit 23
	{
		RT_BT_LOG(YELLOW, PTA_DISPLAY_FHS_SERVICE_MESSAGE_INFORMATION,0,0); 
	}
	else
	{
		RT_BT_LOG(YELLOW, PTA_DISPLAY_FHS_MESSAGE_SERVICE_UNKNOWN,0,0); 
	}			
}

void fnGetMajorClasses(PTA_FHS_FORMAT_STRUCT * pFhsFormatData)
{
	PTA_FHS_PERIPHERAL_MINOR_CLASSES_STRUCT s_tempPtaFHSPhiMinorClass;

	// clear sPtaFHSMajorClass
	sPtaFHSMajorClass.bAudio_Video = 0;
	sPtaFHSMajorClass.bComputer = 0;
	sPtaFHSMajorClass.bImaging = 0;
	sPtaFHSMajorClass.bLAN_Network = 0;
	sPtaFHSMajorClass.bMisc = 0;
	sPtaFHSMajorClass.bPeripheral = 0;
	sPtaFHSMajorClass.bPhone = 0;
	sPtaFHSMajorClass.bReserved = 0;
	sPtaFHSMajorClass.bToy = 0;
	sPtaFHSMajorClass.bUncategorized = 0;
	sPtaFHSMajorClass.bWearable = 0;

	//memset(&s_tempPtaFHSPhiMinorClass,0,11);
	
	s_tempPtaFHSPhiMinorClass.bCardReader = 0;
	s_tempPtaFHSPhiMinorClass.bComboKeyboardMouse =0;
	s_tempPtaFHSPhiMinorClass.bDigitizerTablet =0;
	s_tempPtaFHSPhiMinorClass.bGamepad =0;
	s_tempPtaFHSPhiMinorClass.bJoystick =0;
	s_tempPtaFHSPhiMinorClass.bKeyboard =0;
	s_tempPtaFHSPhiMinorClass.bMinorDeviceClasses =0;
	s_tempPtaFHSPhiMinorClass.bMouse =0;
	s_tempPtaFHSPhiMinorClass.bNonKeyboardMouse =0;
	s_tempPtaFHSPhiMinorClass.bRemoteControl =0;
	s_tempPtaFHSPhiMinorClass.bSensingDevice =0;
	


	switch ( pFhsFormatData->bCoD[1] )
	{
	case 0:
		sPtaFHSMajorClass.bMisc = 1;
		RT_BT_LOG(YELLOW, PTA_DISPLAY_FHS_MESSAGE_UNKNOWN,0,0); 
		break;
	case 1:
		sPtaFHSMajorClass.bComputer = 1;
		RT_BT_LOG(GREEN, PTA_DISPLAY_FHS_MESSAGE_COMPUTER,0,0); 
		break;
	case 2:
		sPtaFHSMajorClass.bPhone = 1;
		RT_BT_LOG(GREEN, PTA_DISPLAY_FHS_MESSAGE_PHONE,0,0);
		break;
	case 3:
		sPtaFHSMajorClass.bLAN_Network = 1;
		RT_BT_LOG(GREEN, PTA_DISPLAY_FHS_MESSAGE_LAN,0,0); 
		break;
	case 4:
		sPtaFHSMajorClass.bAudio_Video = 1;
		RT_BT_LOG(GREEN, PTA_DISPLAY_FHS_MESSAGE_AUDIOVIDEO,0,0); 
		break;
	case 5:
		sPtaFHSMajorClass.bPeripheral = 1;
		RT_BT_LOG(GREEN, PTA_DISPLAY_FHS_MESSAGE_PERIPHERAL,0,0); 
		fnGetPeriMinorClasses(pFhsFormatData);
        memcpy(&s_tempPtaFHSPhiMinorClass, &sPtaFHSPhiMinorClass, 
               sizeof(PTA_FHS_PERIPHERAL_MINOR_CLASSES_STRUCT));
		break;
	case 6:
		sPtaFHSMajorClass.bImaging = 1;
		RT_BT_LOG(GREEN, PTA_DISPLAY_FHS_MESSAGE_IMAGE,0,0); 
		break;
	case 7:
		sPtaFHSMajorClass.bWearable = 1;
		RT_BT_LOG(GREEN, PTA_DISPLAY_FHS_MESSAGE_WEARABLE,0,0); 
		break;
	case 8:
		sPtaFHSMajorClass.bToy= 1;
		RT_BT_LOG(GREEN, PTA_DISPLAY_FHS_MESSAGE_TOY,0,0); 
		break;		
	default:
		sPtaFHSMajorClass.bReserved= 1;
		RT_BT_LOG(GREEN, PTA_DISPLAY_FHS_MESSAGE_UNKNOWN,0,0); 
	}
}

void fnGetAVMinorClasses(PTA_FHS_FORMAT_STRUCT * pFhsFormatData)
{
	//memset(&sPtaFHSAVMinorClass,0,17);
}

void fnGetPeriMinorClasses(PTA_FHS_FORMAT_STRUCT * pFhsFormatData)
{
	UCHAR bKeyboardMouse = 0;
	
	//memset(&sPtaFHSPhiMinorClass,0,11);

	bKeyboardMouse = ( pFhsFormatData->bCoD[0] )>>6;

	switch ( bKeyboardMouse )
	{
	case 0:
		sPtaFHSPhiMinorClass.bNonKeyboardMouse = 1;
		RT_BT_LOG(GREEN, PTA_DISPLAY_FHS_MESSAGE_NON_MOUSE_KEYBOARD,0,0); 
		break;
	case 1:
		sPtaFHSPhiMinorClass.bKeyboard = 1;
		RT_BT_LOG(GREEN, PTA_DISPLAY_FHS_MESSAGE_KEYBOARD,0,0); 
		break;
	case 2:
		sPtaFHSPhiMinorClass.bMouse = 1;
		RT_BT_LOG(GREEN, PTA_DISPLAY_FHS_MESSAGE_MOUSE,0,0); 
		break;
	case 3:
		sPtaFHSPhiMinorClass.bComboKeyboardMouse = 1;
		RT_BT_LOG(GREEN, PTA_DISPLAY_FHS_MESSAGE_COMBO_MOUSE_KEYBOARD,0,0); 
		break;
	default:
		sPtaFHSPhiMinorClass.bNonKeyboardMouse = 1;
	}
}
#endif

/* ================================= */
/* morgan add for A2DP estimation by SBC syncword */
//==================================================
//
//	A2DP estimation : pta_a2dp_sbc_estimation2
//
//==================================================

#ifdef SOURCECODE_TEST_CLOSE_FW_PROFILE_ESTIMATE

#ifdef A2DP_SBC_SYNCWORD 
API_RESULT pta_a2dp_sbc_estimation2(UCHAR *buffer)
{

    UCHAR RTP_header_CC;
    UCHAR Media_packet_Flag;
    UINT16 L2CAP_CID;	 
    Media_packet_Flag = 0;

    HCI_ACL_DATA_PKT *ppkt;
    ppkt = (HCI_ACL_DATA_PKT*)buffer;

    // ================================================ 
    //	20120801 morgan add SBC mode 
    // ================================================
    // check if the  SBC "syncword" is available 

    // step 1 HCI header length = 4 ( fixed length )
    // check if  HCI contains the first L2CAP packet
    // ppkt->packet_boundary_flag == 0x02 or ppkt->packet_boundary_flag == 0x00 

    // step 2 L2CAP header length = 4 ( fixed length )
    // L2CAP chennel 
    // example : 
    // 0~15 : Length : 0x275 = 629
    // 16~31 : Channel ID = 0x89

    // step 3 Media packet header :  length = 12 ( variable length , at least 12 bytes , usually 12 bytes ) 
    // ( AVDTP : RTP format ) CC = byte8 bit4~7
    RTP_header_CC = ( *(buffer+8) ) & 0xf0;
    // some other parser here, but not ready

    // step 4 media packet header ( fixed length  = 1 )

    // step 5 SBC frame  :    media payload 
    // If "bit 7" = 0 : check next byte as SBC frame header
    // If "bit 7" = 1 : check if bit 6 = 1 , then check next byte as SBC frame header			
    // media payload , SBC frame header , syncword = 0x9C	

    pta_meter_var.SBC_Flag = 0;
    // condition 2 : fragment SBC
    if ((*(buffer+20) & 0x80))
    {
        // set the media payload flag
        Media_packet_Flag = 1;			
    }                    

    // condition 1 : no fragment of SBC
    if (!Media_packet_Flag && ((*(buffer+21)) == 0x9C) )  // bit 7 = 0
    {
        pta_meter_var.SBC_Flag = 1;
        // here store L2CAP CID
        L2CAP_CID = *(buffer+7) | ((*(buffer+8)) << 8) ;
        //RT_BT_LOG(RED,  PTA_DBG_DISPLAY, 1, L2CAP_CID);					
    }
    else if (Media_packet_Flag && 
            ((*(buffer+20) & 0x40 ) == 0x40) &&  (*(buffer+21) == 0x9C) ) 
    {                            
        pta_meter_var.SBC_Flag = 1;
        Media_packet_Flag = 0;			
        //RT_BT_LOG(RED,  PTA_DBG_DISPLAY, 0, 0);			
    }

    // ================================================ 				              
    /* check packet boundary flag of HCI  ( 00: non flushable HCI packet ) */
    /* 01 continuing fragment packet */
    /*10 first flushable packet */
    /* 11 reserved */
    //RT_BT_LOG(RED, PTA_HCI_TD_TX_TRANSPORT, 2,ppkt->acl_data_total_length, ppkt->packet_boundary_flag);

    //PKT_Boundary_Flag = ppkt->packet_boundary_flag;

    if ((ppkt->packet_boundary_flag == 0x02)  || pta_meter_var.SBC_Flag)  // it is a flushable packet
    {
        //ppkt->connection_handle;
        // here save the information to database
        // get connection handle
        // get ce indexs by connection handle
        // save the info to ce
        //API_RESULT lmp_get_CE_index_from_conn_handle(UINT16 conn_handle, UINT16* ce_index)
        UINT16  bCeIndex=0;
        //LMP_CONNECTION_ENTITY* psConnEntity;

#ifdef PTA_B_LOG            
        RT_BT_LOG(RED, PTA_DISPLAY_HCI_MESSAGE_FLUSHABLE, 0, 0);
#endif

        if (LMP_GET_CE_INDEX_FROM_CONN_HANDLE(ppkt->connection_handle, &bCeIndex)
                                            != API_SUCCESS)
        {
            return API_FAILURE;
        }     
        // morgan add 20111222  
        // morgan modify 20120330
        if ( ppkt->packet_boundary_flag == 0x02 )
        {
            pta_meter_var.bA2DPAccumulateCounter ++;
            pta_meter_var.wA2DPSerialNumber ++;
        }                                           
        else
        {
            pta_meter_var.bSBCAccumulateCounter ++;
            pta_meter_var.wA2DPSerialNumber ++;
        }

        if( pta_meter_var.bA2DPAccumulateCounter >= (MAX_A2DP_ACCUMULATE_NUM-2) )
        {
            //RT_BT_LOG(GREEN,PTA_DBG_SBC_GENERAL_DISPLAY,2,1,0);
            pta_meter_var.A2DP_Find = 1;
        }               
        if ( pta_meter_var.bSBCAccumulateCounter >= MAX_A2DP_ACCUMULATE_NUM )
        {                           
            //RT_BT_LOG(GREEN,PTA_DBG_SBC_GENERAL_DISPLAY,2,2,0);
            pta_meter_var.SBC_Find = 1;
        }

        if (pta_meter_var.A2DP_Find || pta_meter_var.SBC_Find)
        {       
            pta_meter_var.A2DP_Exist = 1;
            pta_meter_var.A2DP_connection_handle = ppkt->connection_handle;
            pta_meter_var.bA2DPAccumulateCounter = 0;
            pta_meter_var.bSBCAccumulateCounter =0;
        }
    }            
    return API_SUCCESS;
}
#endif


//==================================================
//
//	A2DP estimation : pta_a2dp_estimation
//
//==================================================
/* morgan add for A2DP estimation */
#ifdef A2DP_FLUSHABLE 
API_RESULT pta_a2dp_estimation(HCI_ACL_DATA_PKT *ppkt)
{
    /* check packet boundary flag of HCI  ( 00: non flushable HCI packet ) */
    /* 01 continuing fragment packet */
    /*10 first flushable packet */
    /* 11 reserved */
    //RT_BT_LOG(RED, PTA_HCI_TD_TX_TRANSPORT, 2,ppkt->acl_data_total_length, ppkt->packet_boundary_flag);


    if (ppkt->packet_boundary_flag == 0x02 )  // it is a flushable packet
    	{               
        //ppkt->connection_handle;
        // here save the information to database
        // get connection handle
        // get ce indexs by connection handle
        // save the info to ce

        UINT16  bCeIndex;
		
        #ifdef PTA_B_LOG
        RT_BT_LOG(RED, PTA_DISPLAY_HCI_MESSAGE_FLUSHABLE, 0, 0);
        #endif
        
        if (LMP_GET_CE_INDEX_FROM_CONN_HANDLE(ppkt->connection_handle, &bCeIndex)
                != API_SUCCESS)
        {
             return API_FAILURE;
        }

        // morgan add 20111222  
        // morgan modify 20120329
        pta_meter_var.bA2DPAccumulateCounter ++;
        

        if( pta_meter_var.bA2DPAccumulateCounter >= 3 )
        {
            pta_meter_var.A2DP_Exist = 1;
            pta_meter_var.A2DP_connection_handle = ppkt->connection_handle;
            pta_meter_var.bA2DPAccumulateCounter = 0;
 		}
    }        	
   				

    return API_SUCCESS;
}
#endif /* morgan add for A2DP estimation */
//==================================================
    				
 



//==================================================
//
//	FTP estimation : pta_ftp_estimation
//
//==================================================
#ifdef FTP_ESTIMATION /* morgan add for FTP estimation */
API_RESULT pta_ftp_estimation(HCI_ACL_DATA_PKT *ppkt)
    {
    return API_SUCCESS;        
}
#endif  /* morgan add for FTP estimation */

#endif  /*end SOURCECODE_TEST_CLOSE_FW_PROFILE_ESTIMATE */

//==================================================
//
//	HID estimation : pta_hid_estimation
//
//==================================================
#ifdef HID_ESTIMATION
void pta_hid_estimation(UINT16 conn_handle)
{
    //morgan
    UINT16 ce_index;
    LMP_CONNECTION_ENTITY* ce_ptr;
    UINT8 HID_am_addr;
    UINT8 HID_pid;
    UINT16 HID_wdata;	

    if (conn_handle >= LL_HCI_MIN_CONN_HANDLE)
    {
        /* connection handle of le acl link */
        return;
    }

    if (LMP_GET_CE_INDEX_FROM_CONN_HANDLE(conn_handle, &ce_index)
        != API_SUCCESS)
    {       
#ifdef PATCH_PTA_EXTENSION_DEBUG_LOG 	
        RT_BT_LOG(RED, PTA_DISPLAY_MESSAGE2, 1, conn_handle);
#endif
    }
    else
    {
        ce_ptr = &lmp_connection_entity[ce_index];  
        if(ce_ptr->ce_status == LMP_SNIFF_MODE )
        {
                //RT_BT_LOG(RED, PTA_DISPLAY_HCI_MESSAGE_SNIFF_MODE, 0, 0);
    		// 20120809 morgan modify	
    		pta_meter_var.wHIDSerialNumber ++;					
    		//RT_BT_LOG(RED, PTA_DISPLAY_HCI_MESSAGE_SNIFF_MODE, 3, ce_ptr->sniff_interval, ce_ptr->sniff_attempt, pta_meter_var.bHIDAccumulateCounter);
    		pta_meter_var.bHIDAccumulateCounter ++;
    		if ( pta_meter_var.bHIDAccumulateCounter >= 4 )
    		{
                pta_meter_var.HID_connection_handle = conn_handle;
                pta_meter_var.HID_Exist = 1;
    			pta_meter_var.bHIDAccumulateCounter = 0;
    		}

                // 20120313 HID grant high prority 	
                //LMP_GET_CE_INDEX_FROM_CONN_HANDLE
                //LMP_GET_AM_ADDR_FROM_CE
                // 20120814 : HID grant HID high priority enabled by mailbox cmd
                	//20120913 : move HID setting to HID profile estimation
                
    		if( bHIDenable ==1 && bHIDCount == 0)
    		{
    	       	 if( pta_meter_var.HID_Exist )
            	{
                    if (LMP_GET_CE_INDEX_FROM_CONN_HANDLE(pta_meter_var.HID_connection_handle, &ce_index) == API_SUCCESS)
            		{
                        UCHAR  lut_index;
                        HID_am_addr = lmp_connection_entity[ce_index].am_addr;
                        HID_pid = lmp_connection_entity[ce_index].phy_piconet_id;						    					
                        //RT_BT_LOG(RED,PTA_DISPLAY_HID_MESSAGE2,6,pta_meter_var.HID_Exist, 0, HID_am_addr,HID_pid,pta_meter_var.HID_connection_handle,conn_handle  );
                        lut_index = lc_get_lut_index_from_phy_piconet_id(HID_am_addr, HID_pid);
                        HID_wdata = ( BIT2 << lut_index);
                        BB_write_baseband_register( 0x190, HID_wdata);
    		      		bHIDCount =1;
            		}
            	}
    		}
        }   
        else
        {
            //RT_BT_LOG(YELLOW, PTA_DISPLAY_HCI_MESSAGE_SNIFF_MODE,3,ce_ptr->sniff_interval, ce_ptr->sniff_attempt, ce_ptr->sniff_timeout);
        }
    }                   
}
#endif /* endif of HID_ESTIMATION */


#endif


