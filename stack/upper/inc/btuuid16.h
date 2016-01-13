/**
********************************************************************************************************
Copyright (c) 2015, Realtek Semiconductor Corporation. All rights reserved.
*********************************************************************************************************
* @file       btuuid16.h
* @brief      
* @details   
*
* @author   	gordon
* @date      	2015-07-13
* @version	v0.1
*/

/* ---------------------------------------------------------------------------
 * Project      : Bluetooth Stack
 * FileName     : $RCSfile: btuuid16.h,v $
 * Created      : 2005/01/27
 * Last access  : $Date: 2007/05/15 08:35:02 $
 * Archive      : $Source: /var/lib/cvs/sw/inc/btuuid16.h,v $
 * Revision:    : $Revision: 1.6 $
 * Version:     : $Name: P_BLB1290_V1_0 $
 * Copyright    : (C) Stollmann E+V GmbH, Hamburg 2003
 * Author       : Olaf Rollwagen
 * Compiler     : ANSI C compatible
 * Environment  : Windows compatible
 * Contents     : Bluetooth uuid16 definitions
 * Note         :
 * History      : $Log: btuuid16.h,v $
 * History      : Revision 1.6  2007/05/15 08:35:02  jw
 * History      : added pbap
 * History      :
 * History      : Revision 1.5  2005/04/21 09:37:01  elli
 * History      : Not the righth filename case
 * History      :
 * History      : Revision 1.3  2005/01/31 09:21:11  or
 * History      : first version for mpa.h
 * History      :
 * History      : Revision 1.2  2005/01/28 13:43:04  or
 * History      : now the log works
 * History      :
 * -------------------------------------------------------------------------*/

#if ! defined (__BTUUID16_H)
#define __BTUUID16_H

/** extracted from service_discovery.html of Bluetooth SIG. */

#define BT_UUID16_ServiceDiscoveryServerServiceClassID      0x1000
#define BT_UUID16_BrowseGroupDescriptorServiceClassID       0x1001
#define BT_UUID16_PublicBrowseGroup                             0x1002
#define BT_UUID16_SerialPort                                    0x1101
#define BT_UUID16_LANAccessUsingPPP                             0x1102
#define BT_UUID16_DialupNetworking                              0x1103
#define BT_UUID16_IrMCSync                                      0x1104
#define BT_UUID16_OBEXObjectPush                                0x1105
#define BT_UUID16_OBEXFileTransfer                              0x1106
#define BT_UUID16_IrMCSyncCommand                               0x1107
#define BT_UUID16_Headset                                       0x1108
#define BT_UUID16_CordlessTelephony                             0x1109
#define BT_UUID16_AudioSource                                   0x110A
#define BT_UUID16_AudioSink                                     0x110B
#define BT_UUID16_AV_RemoteControlTarget                       0x110C
#define BT_UUID16_AdvancedAudioDistribution                     0x110D
#define BT_UUID16_AV_RemoteControl                             0x110E
#define BT_UUID16_VideoConferencing                             0x110F
#define BT_UUID16_Intercom                                      0x1110
#define BT_UUID16_Fax                                           0x1111
#define BT_UUID16_HeadsetAudioGateway                           0x1112
#define BT_UUID16_WAP                                           0x1113
#define BT_UUID16_WAP_CLIENT                                    0x1114
#define BT_UUID16_PANU                                          0x1115
#define BT_UUID16_NAP                                           0x1116
#define BT_UUID16_GN                                            0x1117
#define BT_UUID16_DirectPrinting                                0x1118
#define BT_UUID16_ReferencePrinting                             0x1119
#define BT_UUID16_Imaging                                       0x111A
#define BT_UUID16_ImagingResponder                              0x111B
#define BT_UUID16_ImagingAutomaticArchive                       0x111C
#define BT_UUID16_ImagingReferencedObjects                      0x111D
#define BT_UUID16_Handsfree                                     0x111E
#define BT_UUID16_HandsfreeAudioGateway                         0x111F
#define BT_UUID16_DirectPrintingReferenceObjectsService         0x1120
#define BT_UUID16_ReflectedUI                                   0x1121
#define BT_UUID16_BasicPrinting                                 0x1122
#define BT_UUID16_PrintingStatus                                0x1123
#define BT_UUID16_HumanInterfaceDeviceService                   0x1124
#define BT_UUID16_HardcopyCableReplacement                      0x1125
#define BT_UUID16_HCR_Print                                     0x1126
#define BT_UUID16_HCR_Scan                                      0x1127
#define BT_UUID16_Common_ISDN_Access                            0x1128
#define BT_UUID16_VideoConferencingGW                           0x1129
#define BT_UUID16_UDI_MT                                        0x112A
#define BT_UUID16_UDI_TA                                        0x112B
#define BT_UUID16_AudioVideo                                    0x112C
#define BT_UUID16_SIM_Access                                    0x112D
#define BT_UUID16_PhonebookAccess                               0x112f
#define BT_UUID16_PnPInformation                                0x1200
#define BT_UUID16_GenericNetworking                             0x1201
#define BT_UUID16_GenericFileTransfer                           0x1202
#define BT_UUID16_GenericAudio                                  0x1203
#define BT_UUID16_GenericTelephony                              0x1204
#define BT_UUID16_UPNP_Service                                  0x1205
#define BT_UUID16_UPNP_IP_Service                               0x1206
#define BT_UUID16_ESDP_UPNP_IP_PAN                              0x1300
#define BT_UUID16_ESDP_UPNP_IP_LAP                              0x1301
#define BT_UUID16_ESDP_UPNP_L2CAP                               0x1302
#define BT_UUID16_VideoSource                                   0x1303
#define BT_UUID16_VideoSink                                     0x1304

#endif /**< !defined(__BTUUID16_H) */
