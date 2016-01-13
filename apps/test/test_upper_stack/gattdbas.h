 

#ifndef __GATTDBAS_H
#define __GATTDBAS_H



#if !defined(__GATT_H)
#include <gatt.h>
#endif



/*----- Battery Level (mandatory) definitions ------*/
typedef struct _BatteryLevel
{
    UINT8     Level;                 /* level in percent (0 - 100) */
} TBatteryLevel, * PBatteryLevel;


/*--- prototypes of BAS specific routines ---*/


#endif  /* __GATTDBAS_H */

