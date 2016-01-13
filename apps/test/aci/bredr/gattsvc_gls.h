/**
********************************************************************************************************
Copyright (c) 2015, Realtek Semiconductor Corporation. All rights reserved.
*********************************************************************************************************
* @file       Gattsvc_gls.h
* @brief     GATT built-in GLS (Glucose Service)
* @details   
*
* @author   	gordon
* @date      	2015-07-13
* @version	v0.1
*/

#ifndef __GATTSVC_GLS_H
#define __GATTSVC_GLS_H


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
extern const TAttribAppl gattSvcGLS[];

extern const int gattSvcGLSSize;
extern const int gattSvcGLSNbrOfAttrib;

/** GLC Glucose Service */
#define GATT_UUID_GLUCOSE                      0x1808

#define GATT_UUID_CHAR_GLS_MEASUREMENT         0x2A18
#define GATT_UUID_CHAR_GLS_MEASUREMENT_CTXT    0x2A34
#define GATT_UUID_CHAR_GLS_FEATURES            0x2A51
#define GATT_UUID_CHAR_GLS_RACP                0x2A52
/** attribute indices signaled to GATT server application. */
/** do NOT change these values !!!!                        */

#define GATT_SVC_GLS_MEASUREMENT_INDEX           2  /**< index measurement value         */
#define GATT_SVC_GLS_MEASUREMENT_CCCD_INDEX      3  /**< index of CCCD                   */
#define GATT_SVC_GLS_MEASUREMENT_CTXT_INDEX      5  /**< index measurement context value */
#define GATT_SVC_GLS_MEASUREMENT_CTXT_CCCD_INDEX 6  /**< index of CCCD                   */
#define GATT_SVC_GLS_FEATURE_INDEX               8  /**< index of feature value          */
#define GATT_SVC_GLS_RACP_INDEX                 10  /**< index of RACP value             */
#define GATT_SVC_GLS_RACP_CCCD_INDEX            11  /**< index of CCCD                   */


#ifdef __cplusplus
}
#endif

#endif  /**< __GATTSVC_GLS_H */
