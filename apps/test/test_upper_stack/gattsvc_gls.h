/**********************************************************************!MA*
*
* $Header: /var/lib/cvs/sw/inc/gattsvc_gls.h,v 1.1.2.1 2013/08/15 08:31:15 mn Exp $
*
* File:        $RCSfile: gattsvc_gls.h,v $
* Version:     $Name: P_SRP1290_V1_0 $
*
* Archive:     $Source: /var/lib/cvs/sw/inc/gattsvc_gls.h,v $
* Revision:    $Revision: 1.1.2.1 $
* Date:        $Date: 2013/08/15 08:31:15 $
* Author:      $Author: mn $
*
* ------------------------------------------------------------------------
* !MODULE      [  ]
* ------------------------------------------------------------------------
* !FILE        [  ]
* !PROGRAM     [  ]
* !VERSION     [$Name: P_SRP1290_V1_0 $]
* !GROUP       [  ]
* !AUTHOR      [$Author: mn $]
* ------------------------------------------------------------------------
*
*          Copyright (c)           2013 Stollmann E+V GmbH
*                                  Mendelssohnstr. 15D
*                                  22761 Hamburg
*                                  Phone: 040/89088-0
 *          The content of this file is Stollmann E+V GmbH confidential
*          All Rights Reserved
*
* ------------------------------------------------------------------------
* !DESCRIPTION
*  GATT built-in GLS (Glucose Service) definitions
*
*
* ------------------------------------------------------------------------
* !INDEX
*  ...
* ------------------------------------------------------------------------
* !CONTENTS
* ------------------------------------------------------------------------
* !INCLUDE_REFERENCES
* ------------------------------------------------------------------------
* !HISTORY
*
**********************************************************************!HE*/

#ifndef __GATTSVC_GLS_H
#define __GATTSVC_GLS_H




#ifndef __GATT_H
#include <gatt.h>
#endif



#ifdef __cplusplus
extern "C" {
#endif


/*---------------------------------------------------------------------------
 * service definition
 *--------------------------------------------------------------------------*/

extern const TAttribAppl gattSvcGLS[];

extern const int gattSvcGLSSize;
extern const int gattSvcGLSNbrOfAttrib;

/* attribute indices signaled to GATT server application. */
/* do NOT change these values !!!!                        */

#define GATT_SVC_GLS_MEASUREMENT_INDEX           2  /* index measurement value         */
#define GATT_SVC_GLS_MEASUREMENT_CCCD_INDEX      3  /* index of CCCD                   */
#define GATT_SVC_GLS_MEASUREMENT_CTXT_INDEX      5  /* index measurement context value */
#define GATT_SVC_GLS_MEASUREMENT_CTXT_CCCD_INDEX 6  /* index of CCCD                   */
#define GATT_SVC_GLS_FEATURE_INDEX               8  /* index of feature value          */
#define GATT_SVC_GLS_RACP_INDEX                 10  /* index of RACP value             */
#define GATT_SVC_GLS_RACP_CCCD_INDEX            11  /* index of CCCD                   */


#ifdef __cplusplus
}
#endif

#endif  /* __GATTSVC_GLS_H */
