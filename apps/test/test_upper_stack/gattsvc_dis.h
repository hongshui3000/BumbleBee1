/**********************************************************************!MA*
*
* $Header: /var/lib/cvs/sw/inc/gattsvc_dis.h,v 1.1.2.1 2013/08/15 08:31:15 mn Exp $
*
* File:        $RCSfile: gattsvc_dis.h,v $
* Version:     $Name: P_SRP1290_V1_0 $
*
* Archive:     $Source: /var/lib/cvs/sw/inc/gattsvc_dis.h,v $
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
*  GATT built-in DIS (Device Information Service) definitions
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

#ifndef __GATTSVC_DIS_H
#define __GATTSVC_DIS_H



#ifndef __GATT_H
#include <gatt.h>
#endif



#ifdef __cplusplus
extern "C" {
#endif


/*---------------------------------------------------------------------------
 * service definition
 *--------------------------------------------------------------------------*/

extern const TAttribAppl gattSvcDIS[];

extern const int gattSvcDISSize;
extern const int gattSvcDISNbrOfAttrib;

/* attribute indices signaled to GATT server application. */
/* do NOT change these values !!!!                        */

#define GATT_SVC_DIS_SYSTEM_ID_INDEX          2   /* index of System ID value         */
#define GATT_SVC_DIS_MANUFACTURER_NAME_INDEX  4   /* index of Manufacturer Name value */
#define GATT_SVC_DIS_MODEL_NUMBER_INDEX       6   /* index of Model Number value      */


#ifdef __cplusplus
}
#endif

#endif  /* __GATTSVC_DIS_H */
