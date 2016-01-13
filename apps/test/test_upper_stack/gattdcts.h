

#ifndef __GATTDCTS_H
#define __GATTDCTS_H



#if !defined(__GATT_H)
#include <gatt.h>
#endif



/*----- Current Time (mandatory) definitions ------*/
typedef struct _DayDateTime
{
    TIMESTAMP         DateTime;
    UINT8             DayOfTheWeek;
} TDayDateTime, * PDayDateTime;

typedef struct _ExactTime256
{
    TDayDateTime      DayDateTime;
    UINT8             Fractions256;
} TExactTime256, * PExactTime256;


typedef struct _CurrentTime
{
    TExactTime256     ExactTime256;
    UINT8             AdjustReason;
} TCurrentTime, * PCurrentTime;


/* Day of the Week values */
#define CTS_WEEKDAY_MONDAY           1
#define CTS_WEEKDAY_TUESDAY          2
#define CTS_WEEKDAY_WEDNESAY         3
#define CTS_WEEKDAY_THURSDAY         4
#define CTS_WEEKDAY_FRIDAY           5
#define CTS_WEEKDAY_SATURDAY         6
#define CTS_WEEKDAY_SUNDAY           7

/* Adjust Reason bits */
#define CTS_ADJUST_REASON_NOT_PRESENT             0
#define CTS_ADJUST_REASON_MANUAL_TIME_UPDATE      1
#define CTS_ADJUST_REASON_EXT_REF_TIME_UPDATE     2
#define CTS_ADJUST_REASON_CHANGE_OF_TIME_ZONE     4
#define CTS_ADJUST_REASON_CHANGE_OF_DST           8   /* change of Daylight Saving Time */


/*----- Local Time Information (optional) definitions ------*/
typedef struct _LocalTimeInfo
{
    int8_t       TimeZone;        /* UTC+f(value>), value=-48,..,56 */
    UINT8       DSTOffset;
} TLocalTimeInfo, * PLocalTimeInfo;

#define CTS_DST_OFFSET_STANDARD             0
#define CTS_DST_OFFSET_HALF_AN_HOUR         2
#define CTS_DST_OFFSET_ONE_HOUR             4
#define CTS_DST_OFFSET_TWO_HOURS            8


/*----- Reference Time Information (optional) definitions ------*/
typedef struct _ReferenceTimeInfo
{
    UINT8       Source;
    UINT8       Accuracy;
    UINT8       DaysSinceUpdate;
    UINT8       HoursSinceUpdate;
} TReferenceTimeInfo, * PReferenceTimeInfo;

#define CTS_TIME_SOURCE_UNKNOWN         0
#define CTS_TIME_SOURCE_NTP             1  /* Network Time Protocol */
#define CTS_TIME_SOURCE_GPS             2
#define CTS_TIME_SOURCE_RADIO           3
#define CTS_TIME_SOURCE_MANUAL          4
#define CTS_TIME_SOURCE_ATOMIC_CLOCK    5
#define CTS_TIME_SOURCE_CELLULAR_NET    6

#define CTS_TIME_ACCURACY_OUT_OF_RANGE  0xFE
#define CTS_TIME_ACCURACY_UNKNOWN       0xFF


/*--- prototypes of CTS specific routines ---*/


#endif  /* __GATTDCTS_H */

