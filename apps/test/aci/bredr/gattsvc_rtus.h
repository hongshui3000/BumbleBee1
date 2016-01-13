/**
********************************************************************************************************
Copyright (c) 2015, Realtek Semiconductor Corporation. All rights reserved.
*********************************************************************************************************
* @file       Gattsvc_rtus.h
* @brief     GATT built-in RTUS (Reference Time Update Service)
* @details   
*
* @author   	gordon
* @date      	2015-07-09
* @version	v0.1
*/

#ifndef __GATTSVC_RTUS_H
#define __GATTSVC_RTUS_H


#ifndef __FLAGS_H
//#include <flags.h>
#endif
#ifndef __BASETYPE_H
//#include <rtl_types.h>
#endif
#ifndef __GATT_H
#include <gatt.h>
#endif



#ifdef __cplusplus
extern "C" {
#endif


/** service definition */
extern const TAttribAppl gattSvcRTUS[];

extern const int gattSvcRTUSSize;
extern const int gattSvcRTUSNbrOfAttrib;

/** RTUS Reference Time Update Service */
#define GATT_UUID_REFERENCE_TIME_UPDATE        0x1806

#define GATT_UUID_CHAR_RTUS_CONTROL_POINT      0x2A16
#define GATT_UUID_CHAR_RTUS_STATE              0x2A17
/** attribute indices signaled to GATT server application. */
/** do NOT change these values !!!!                        */

#define GATT_SVC_RTUS_CONTROL_POINT_INDEX    2  /**< Time UpdateControl Point value */
#define GATT_SVC_RTUS_STATE_INDEX            4  /**< Time Update State value */


#ifdef __cplusplus
}
#endif

#endif  /**< __GATTSVC_RTUS_H */
