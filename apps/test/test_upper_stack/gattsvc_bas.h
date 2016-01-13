/**********************************************************************!MA*
*
* $Header: /var/lib/cvs/sw/inc/gattsvc_bas.h,v 1.1.2.1 2013/08/15 08:31:15 mn Exp $
*
* File:        $RCSfile: gattsvc_bas.h,v $
* Version:     $Name: P_SRP1290_V1_0 $
*
* Archive:     $Source: /var/lib/cvs/sw/inc/gattsvc_bas.h,v $
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
*  GATT built-in BAS (Battery Service) definitions
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

#ifndef __GATTSVC_BAS_H
#define __GATTSVC_BAS_H



#ifndef __GATT_H
#include <gatt.h>
#endif



#ifdef __cplusplus
extern "C" {
#endif


/*---------------------------------------------------------------------------
 * service definition
 *--------------------------------------------------------------------------*/

extern const TAttribAppl gattSvcBAS[];

extern const int gattSvcBASSize;
extern const int gattSvcBASNbrOfAttrib;

/* attribute indices signaled to GATT server application. */
/* do NOT change these values !!!!                        */

#define GATT_SVC_BAS_BATTERY_LEVEL_INDEX    2     /* battery level value */


#ifdef __cplusplus
}
#endif

#endif  /* __GATTSVC_BAS_H */
