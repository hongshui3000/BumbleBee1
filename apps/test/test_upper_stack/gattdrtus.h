 

#ifndef __GATTDRTUS_H
#define __GATTDRTUS_H



#if !defined(__GATT_H)
#include <gatt.h>
#endif



/*----- Reference Time Update (mandatory) definitions ------*/
/*-- Time Update Control Point --*/
typedef struct _TimeUpdateCP
{
    UINT8             Cmd;             /* command */
} TTimeUpdateCP, * PTimeUpdateCP;

/* command values */
#define RTUS_CP_CMD_GET_REF_UPDATE      1
#define RTUS_CP_CMD_CANCEL_REF_UPDATE   2


/*-- Time Update State --*/
typedef struct _TimeUpdateState
{
    UINT8             CurrentState;
    UINT8             Result;
} TTimeUpdateState, * PTimeUpdateState;

/* current state values */
#define RTUS_CURRENT_STATE_IDLE                0
#define RTUS_CURRENT_STATE_UPDATE_PENDING      1

/* result values */
#define RTUS_RESULT_SUCCESS                0
#define RTUS_RESULT_CANCELED               1
#define RTUS_RESULT_NO_CONNECTION_TO_REF   2
#define RTUS_RESULT_REF_RESP_WITH_ERROR    3
#define RTUS_RESULT_TIMEOUT                4
#define RTUS_RESULT_UPDATE_NOT_ATTEMPTED   5


/*--- prototypes of RTUS specific routines ---*/


#endif  /* __GATTDRTUS_H */

