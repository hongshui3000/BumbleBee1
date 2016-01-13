/***************************************************************************
 Copyright (C) MindTree Ltd.
 This module is a confidential and proprietary property of MindTree and
 a possession or use of this module requires written permission of MindTree.
 ***************************************************************************/

#ifndef _H_OS_TIMER_
#define _H_OS_TIMER_

#include "mint_os.h"

/*******************************************/
/* the definition of the type of the timer */
/*******************************************/
typedef enum timer_type {
    ONESHOT,
    PERIODIC
} OS_TIMER_TYPE;

/********************************************/
/* the definition of the state of the timer */
/********************************************/
typedef enum os_timer_state {
    TIMER_FREE,
    TIMER_CREATED,
    TIMER_RUNNING    
}OS_TIMER_STATE;

/********************************************/
/* the structure of the OS Timer            */
/********************************************/
typedef struct os_timer {
    union {
        struct {
            OS_HANDLE       handle;                 /* current timer handle */
            UINT8          cal_table_index;         /* the index of calendar table */
            UINT8          timer_count;             /* the counter of timer (rounds) */
            UINT8          timer_type;              /* the type of timer (OS_TIMER_TYPE) */
            UINT8          state;                   /* current state of timer (OS_TIMER_STATE) */
            UINT32          timer_value;            /* the expired time of timer */
            OS_TIMEOUT_HANDLER  timeout_function;   /* the callback function when the 
                                                       timer is timeout*/
        };
        UINT32 DWord[4];
    };
    void            *args;                  /* the pointer of the arguments of 
                                               callback function */
    struct os_timer *next;                  /* the pointer of next timer */

#ifdef _CCH_LPS_
    UINT32          sniff_tick_timer; /*now for lmp response timer for lps*/
#endif
} OS_TIMER;

typedef union os_timer_compare {
    struct {    
        OS_HANDLE       handle;                 /* current timer handle */
        UINT8          cal_table_index;         /* the index of calendar table */
        UINT8          timer_count;             /* the counter of timer (rounds) */
        UINT8          timer_type;              /* the type of timer (OS_TIMER_TYPE) */
        UINT8          state;                   /* current state of timer (OS_TIMER_STATE) */   
        UINT32          timer_value;            /* the expired time of timer */
        OS_TIMEOUT_HANDLER  timeout_function;   /* the pointer of the arguments of 
                                                   callback function */
    };
    UINT32 DWord[4];
} OS_TIMER_COMPARE;



void        OS_timer_mgmt_init(void);

void        OS_timer_init(void);


OS_HANDLE   OS_create_timer (OS_TIMER_TYPE timer_type,
                             OS_TIMEOUT_HANDLER timer_handler,
                             OS_ADDRESS args );

OS_STATUS   OS_delete_timer ( OS_HANDLE timer_handle );

OS_STATUS   OS_start_timer ( OS_HANDLE timer_handle, UINT32 value );

OS_STATUS   OS_stop_timer ( OS_HANDLE timer_handle );

UINT32 OS_is_timer_running( OS_HANDLE timer_handle );

#ifdef _NEW_SW_TIMER_DISPATCH_
void OS_time_out_timers_new (UINT16 cur_index);
#else 
void OS_time_out_timers (UINT16 cur_index);
#endif

void  OS_timer_task_function ( OS_SIGNAL *signal );

#endif /* _H_OS_TIMER_ */


