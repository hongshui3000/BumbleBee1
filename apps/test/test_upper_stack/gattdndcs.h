 

#ifndef __GATTDNDCS_H
#define __GATTDNDCS_H



#if !defined(__GATT_H)
#include <gatt.h>
#endif



/*----- Next DST Change (mandatory) definitions ------*/
typedef struct _TimeWithDST
{
    TIMESTAMP         DateTime;
    UINT8             DSTOffset;
} TTimeWithDST, * PTimeWithDST;


/* DSTOffset values */
#define DST_OFFSET_STANDARD                      0
#define DST_OFFSET_HALF_AN_HOUR_DAYLIGHT_TIME    2
#define DST_OFFSET_DAYLIGHT_TIME                 4
#define DST_OFFSET_DOUBLE_DAYLIGHT_TIME          8


/*--- prototypes of NDCS specific routines ---*/


#endif  /* __GATTDNDCS_H */

