/**
********************************************************************************************************
Copyright (c) 2015, Realtek Semiconductor Corporation. All rights reserved.
*********************************************************************************************************
* @file       sdp_code.h
* @brief      SDP Protocol Layer: Header File with Constant Definitions
* @details   
*
* @author   	gordon
* @date      	2015-06-29
* @version	v0.1
*/

#ifndef __SDP_CODE_H
#define __SDP_CODE_H
#include <rtl_types.h>

/** sdp function codes from bt spec 1.0, sdp, p. 345 */
#define SDP_SUCCESS                              0
#define SDP_ERROR_RESPONSE                       1
#define SDP_SERVICE_SEARCH_REQUEST               2
#define SDP_SERVICE_SEARCH_RESPONSE              3
#define SDP_SERVICE_ATTRIBUTE_REQUEST            4
#define SDP_SERVICE_ATTRIBUTE_RESPONSE           5
#define SDP_SERVICE_SEARCH_ATTRIBUTE_REQUEST     6
#define SDP_SERVICE_SEARCH_ATTRIBUTE_RESPONSE    7

/** sdp error codes, now blueFace.h */

#define PSM_SDP                        0x01
#define PSM_RFCOMM                     0x03
#define PSM_HID_CONTROL                0x11
#define PSM_HID_INTERRUPT              0x13
#define PSM_AVCTP                      0x17
#define PSM_AVDTP                      0x19

/** SDP uuids, p. 1021 ff */
#define UUID_SDP                              0x0001
#define UUID_UDP                              0x0002
#define UUID_RFCOMM                           0x0003
#define UUID_TCP                              0x0004
#define UUID_TCSBIN                           0x0005
#define UUID_TCSAT                            0x0006
#define UUID_ATT                              0x0007
#define UUID_OBEX                             0x0008
#define UUID_IP                               0x0009
#define UUID_FTP                              0x000A
#define UUID_HTTP                             0x000C
#define UUID_WSP                              0x000E
#define UUID_BNEP                             0x000F
#define UUID_UPNP                             0x0010
#define UUID_HIDP                             0x0011
#define UUID_HC_CONTROL                       0x0012
#define UUID_HC_DATA                          0x0014
#define UUID_HC_NOTIFICATION                  0x0016
#define UUID_AVCTP                            0x0017
#define UUID_AVDTP                            0x0019
#define UUID_CMTP                             0x001B
#define UUID_HDP_CONTROL_CHANNEL              0x001E
#define UUID_HDP_DATA_CHANNEL                 0x001F

#define UUID_L2CAP                            0x0100

#define UUID_SERVICE_DISCOVERY_SERVER         0x1000
#define UUID_BROWSE_GROUP_DESCRIPTOR          0x1001
#define UUID_PUBLIC_BROWSE_GROUP              0x1002
#define UUID_SERIALPORT                       0x1101
#define UUID_LANACCESS                        0x1102
#define UUID_DIALUPNETWORKING                 0x1103
#define UUID_IRMCSYNC                         0x1104
#define UUID_OBEXOBJECTPUSH                   0x1105
#define UUID_OBEXFILETRANSFER                 0x1106
#define UUID_IRMSYNCCOMMAND                   0x1107
#define UUID_HEADSET                          0x1108
#define UUID_CORDLESSTELEPHONY                0x1109
#define UUID_AUDIOSOURCE                      0x110A
#define UUID_AUDIOSINK                        0x110B
#define UUID_AVREMOTECONTROLTARGET            0x110C
#define UUID_ADVANCEDAUDIODISTRIBUTION        0x110D
#define UUID_AVREMOTECONTROL                  0x110E
#define UUID_VIDEOCONFERENCING                0x110F
#define UUID_INTERCOM                         0x1110
#define UUID_FAX                              0x1111
#define UUID_HEADSETAUDIOGATEWAY              0x1112
#define UUID_WAP                              0x1113
#define UUID_WAP_CLIENT                       0x1114
#define UUID_PANU                             0x1115
#define UUID_NAP                              0x1116
#define UUID_GN                               0x1117
#define UUID_DIRECTPRINTING                   0x1118
#define UUID_REFERENCEPRINTING                0x1119
#define UUID_IMAGING                          0x111A
#define UUID_IMAGINGRESPONDER                 0x111B
#define UUID_IMAGINGAUTOMATICARCHIVE          0x111C
#define UUID_REFERENCEDOBJECTS                0x111D
#define UUID_HANDSFREE                        0x111E
#define UUID_HANDSFREEAUDIOGATEWAY            0x111F
#define UUID_DIRECTPRINTINGREFOBJECTSSERVICE  0x1120
#define UUID_REFLECTEDUI                      0x1121
#define UUID_BASICPRINTING                    0x1122
#define UUID_PRINTINGSTATUS                   0x1123
#define UUID_HUMANINTERFACEDEVICESERVICE      0x1124
#define UUID_HARDCOPYCABLEREPLACEMENT         0x1125
#define UUID_HCR_PRINT                        0x1126
#define UUID_HCR_SCAN                         0x1127
#define UUID_COMMON_ISDN_ACCESS               0x1128
#define UUID_VIDEOCONFERENCINGGW              0x1129
#define UUID_SIM                              0x112D
#define UUID_PBAP                             0x112F
#define UUID_PNPINFORMATION                   0x1200
#define UUID_GENERICNETWORKING                0x1201
#define UUID_GENERICFILETRANSFER              0x1202
#define UUID_GENERICAUDIO                     0x1203
#define UUID_GENERICTELEPHONY                 0x1204
#define UUID_HDP_PROFILE                      0x1400
#define UUID_HDP_PROFILE_SOURCE               0x1401
#define UUID_HDP_PROFILE_SINK                 0x1402
#define UUID_GAP                              0x1800
#define UUID_GATT                             0x1801

/** SDP Data Element Types, p.341 */
#define SDP_TYP_NULL                          0
#define SDP_TYP_UINT                          1
#define SDP_TYP_SINT                          2
#define SDP_TYP_UUID                          3
#define SDP_TYP_STRING                        4
#define SDP_TYP_BOOL                          5
#define SDP_TYP_SEQUENCE                      6
#define SDP_TYP_ALTERNATE                     7
#define SDP_TYP_URL                           8
#define SDP_TYP_RESERVED                      9

/** this one is an internal one: create SDP_TYP_SEQUENCE with uint16_t length */
#define SDP_TYP_SEQUENCE_WORD                10
#define SDP_TYP_SEQUENCE_DWORD               11

/** SDP Attribute Values, p.1023 */
#define SDP_ATTR_SERVICERECORDHANDLE            0x0000
#define SDP_ATTR_SERVICECLASSIDLIST             0x0001
#define SDP_ATTR_SERVICERECORDSTATE             0x0002
#define SDP_ATTR_SERVICEID                      0x0003
#define SDP_ATTR_PROTOCOLDESCRIPTORLIST         0x0004
#define SDP_ATTR_BROWSEGROUPLIST                0x0005
#define SDP_ATTR_LANGUAGEBASEATTRIBUTEIDLIST    0x0006
#define SDP_ATTR_SERVICEINFOTIMETOLIVE          0x0007
#define SDP_ATTR_SERVICEAVAILABILITY            0x0008
#define SDP_ATTR_BLUETOOTHPROFILEDESCRIPTORLIST 0x0009
#define SDP_ATTR_DOCUMENTATIONURL               0x000A
#define SDP_ATTR_CLIENTEXECUTABLEURL            0x000B
#define SDP_ATTR_ICON10                         0x000C
#define SDP_ATTR_ADDIT_PROTOCOLDESCRIPTORLIST   0x000D

#define SDP_ATTR_SERVICENAME                    0x0000 /**< these attrib values are base values */
#define SDP_ATTR_SERVICEDESCRIPTION             0x0001 /**< these attrib values are base values */
#define SDP_ATTR_PROVIDERNAME                   0x0002 /**< these attrib values are base values */

#define SDP_ATTR_VERSIONNUMBERLIST              0x0200
#define SDP_ATTR_GROUPID                        0x0200
#define SDP_ATTR_IPSUBNET                       0x0200
#define SDP_ATTR_SERVICEDATABASESTATE           0x0201

/** Profile specific Attribute IDs */
#define SDP_ATTR_SERVICEVERSION                 0x0300
#define SDP_ATTR_EXTERNALNETWORK                0x0301
#define SDP_ATTR_SUPPORTEDDATASTORESLIST        0x0301
#define SDP_ATTR_FAXCLASS1SUPPORT               0x0302
#define SDP_ATTR_REMOTEAUDIOVOLUMECONTROL       0x0302
#define SDP_ATTR_FAXCLASS20SUPPORT              0x0303
#define SDP_ATTR_SUPPORTED_FORMATS_LIST         0x0303
#define SDP_ATTR_FAXCLASS2SUPPORT               0x0304
#define SDP_ATTR_AUDIOFEEDBACKSUPPORT           0x0305
#define SDP_ATTR_NETWORKADDRESS                 0x0306
#define SDP_ATTR_WAPGATEWAY                     0x0307
#define SDP_ATTR_HOMEPAGEURL                    0x0308
#define SDP_ATTR_WAPSTACKTYPE                   0x0309
#define SDP_ATTR_SECURITYDESCRIPTION            0x030A
#define SDP_ATTR_NETACCESSTYPE                  0x030B
#define SDP_ATTR_MAXNETACCESSRATE               0x030C
#define SDP_ATTR_SUPPORTEDCAPABILITIES          0x0310
#define SDP_ATTR_SUPPORTEDFEATURES              0x0311
#define SDP_ATTR_SUPPORTEDFUNCTIONS             0x0312
#define SDP_ATTR_TOTALIMAGINGCAPACITY           0x0313
#define SDP_ATTR_SUPPORTEDREPOSITORIES          0x0314

/** Hardcopy Cable Replacement */
#define SDP_ATTR_HCRP_1248ID                    0x0300
#define SDP_ATTR_HCRP_DEVICENAME                0x0302
#define SDP_ATTR_HCRP_FRIENDLYNAME              0x0304

/** Human Interface Device */
#define SDP_ATTR_HID_DEVICERELEASENUMBER        0x0200
#define SDP_ATTR_HID_PARSERVERSION              0x0201
#define SDP_ATTR_HID_DEVICESUBCLASS             0x0202
#define SDP_ATTR_HID_COUNTRYCODE                0x0203
#define SDP_ATTR_HID_VIRTUALCABLE               0x0204
#define SDP_ATTR_HID_RECONNECTINITIATE          0x0205
#define SDP_ATTR_HID_DESCRIPTORLIST             0x0206
#define SDP_ATTR_HID_LANGIDBASELIST             0x0207
#define SDP_ATTR_HID_SDPDISABLE                 0x0208
#define SDP_ATTR_HID_BATTERYPOWER               0x0209
#define SDP_ATTR_HID_REMOTEWAKE                 0x020A
#define SDP_ATTR_HID_PROFILEVERSION             0x020B
#define SDP_ATTR_HID_SUPERVISIONTIMEOUT         0x020C
#define SDP_ATTR_HID_NORMALLYCONNECTABLE        0x020D
#define SDP_ATTR_HID_BOOTDEVICE                 0x020E

/** Health Device Profile */
#define SDP_ATTR_HDP_SUPPORTEDFEATURES           0x0200
#define SDP_ATTR_HDP_DATAEXCHANGE_SPECIFICATION  0x0301
#define SDP_ATTR_HDP_MCAP_SUPPORTED_PROCEDURES   0x0302

#define SDP_HDP_DATAEXCHANGE_SPEC_IEEE11073           1

#define SDP_ATTR_HDP_MCAP_PROC_NON                 0x00
#define SDP_ATTR_HDP_MCAP_PROC_RECONNECT_INIT      0x02
#define SDP_ATTR_HDP_MCAP_PROC_RECONNECT_ACCEPT    0x04
#define SDP_ATTR_HDP_MCAP_PROC_CLOCK_SYNC          0x08
#define SDP_ATTR_HDP_MCAP_PROC_CLOCK_SYNC_MASTER   0x10

/** Device IP profile */
#define SDP_ATTR_DIP_SPECIFICATION_ID           0x0200
#define SDP_ATTR_DIP_VENDOR_ID                  0x0201
#define SDP_ATTR_DIP_PRODUCT_ID                 0x0202
#define SDP_ATTR_DIP_PRODUCT_VERSION            0x0203
#define SDP_ATTR_DIP_PRIMARY_RECORD             0x0204
#define SDP_ATTR_DIP_VENDOR_ID_SOURCE           0x0205

#define SDP_LAST_ATTRIBUTE                      0x0400 /**< Some Margin left */

#define SDP_BASE_LANG_OFFSET                    0x0100 /**< Offset to base language */

/** Encodings for SDP_ATTR_LANGUAGEBASEATTRIBUTEIDLIST, p. 360 */
#define SDP_LANGUAGE_ENGLISH                    0x656e /**< this is "en", according to ISO 639:1988 */
#define SDP_LANGUAGE_GERMAN                     0x6465 /**< this is "de", according to ISO 639:1988 */
#define SDP_CHARCODE_UTF8                       106    /**< UTF8 encoding, cf. http://www.isi.edu/in-notes/iana/assignments/character-sets */

/**  BT 2.1 : Extended Inquiry Response */
#define SDP_EIR_FLAGS              1
#define SDP_EIR_UUID16_MORE        2
#define SDP_EIR_UUID16             3
#define SDP_EIR_UUID32_MORE        4
#define SDP_EIR_UUID32             5
#define SDP_EIR_UUID128_MORE       6
#define SDP_EIR_UUID128            7
#define SDP_EIR_NAME_SHORT         8
#define SDP_EIR_NAME               9
#define SDP_EIR_TX_POWERLEVEL     10
#define SDP_EIR_COD               13  /* only OOB */
#define SDP_EIR_SSP_HASH_C        14  /* only OOB */
#define SDP_EIR_SSP_HASH_R        15  /* only OOB */
#define SDP_EIR_MANUFACTURER     255
#define SDP_EIR_BUFFERSIZE       240

/** SDP related conversion operators */

/** Get int16_t from 2 CHARS, Big-Endian format STANDARD NETWORK uint8_t ORDER */
#define NETCHAR2SHORT(p) ((*((p)+1)) & 0xff) + ((*(p)) << 8)

#define NETCHAR2LONG(p) ((uint32_t)(*((p)+3)) & 0xff) + ((uint32_t)(*((p)+2)) << 8) \
                      + ((uint32_t)(*((p)+1)) << 16)  + ((uint32_t)(*((p)+0)) << 24)

#define NETSHORT2CHAR(p,w)                 \
    *((p)+1) = (uint8_t)((w) & 0xff);         \
    *(p)     = /*lint -e(572,778)*/ (uint8_t)(((w)>>8) & 0xff)

#define NETLONG2CHAR(p,w)                   \
    *((p)+3) = (uint8_t)((w) & 0xff);          \
    *((p)+2) = /*lint -e(572,778)*/ (uint8_t)(((w)>>8) & 0xff);     \
    *((p)+1) = /*lint -e(572,778)*/ (uint8_t)(((w)>>16) & 0xff);    \
    *((p)+0) = /*lint -e(572,778)*/ (uint8_t)(((w)>>24) & 0xff)

uint32_t sdpCreateDes(uint8_t *buf, uint8_t *format,...);

#endif

/** end of SDP related conversion operators */
