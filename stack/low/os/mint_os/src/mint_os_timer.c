/***************************************************************************
 Copyright (C) MindTree Ltd.
 This module is a confidential and proprietary property of MindTree and
 a possession or use of this module requires written permission of MindTree.
 ***************************************************************************/
/********************************* Logger *************************/
enum { __FILE_NUM__= 67 };
/********************************* Logger *************************/
#include "bz_log_defines.h"
#include "mint_os_timer_internal.h"
#include "platform.h"
#include "bt_fw_os.h"
#include "UartPrintf.h"
#ifdef DEBUG_PDU_RESPONSE_TIMER
#include "lmp_defines.h"
#include "bt_fw_config.h"
#include "lc.h"
#endif
#ifdef _SECURE_CONN_TEST_LOG
extern TIMER_ID g_timer_temp;
#endif
#ifdef _FIX_CROSSBAR_ERROR_DEBUG_
#include "mem.h"
#endif

UINT16 current_index = 0; /* current entry of calendar_table */
OS_TIMER timer_pool[OS_MAX_TIMER_BUCKETS]; /* the pool of the timer */
OS_TIMER *free_list_head; /* the head of free timer list */
OS_TIMER *free_list_tail; /* the tail of free timer list */
OS_TIMER *calendar_table[OS_MAX_TIMER_BUCKETS]; /* the pointer of calendar table set */
UINT32 timer_value; /* the tick counts of the timer */

extern  OS_HANDLE timer_task_handle;

#ifdef DEBUG_PDU_RESPONSE_TIMER
UINT8 g_lmp_pdu_response_timer_check_flag = FALSE;
extern void lmp_response_timeout_handler(TIMER_ID timer_handle, void *index);
#endif

void OS_get_free_timer(OS_TIMER **new_timer);
void OS_return_timer(OS_TIMER *timer);

/**************************************************************************
 * Function     : OS_timer_task_function
 *
 * Description  : This is the code for the timer task. The timer ISR sends signal
 *              SIGNAL_TIMER_TICK to the timer task if there are pending s/w
 *              timers at the current index of the calendar table. The timer task
 *              calls time out handlers of each of these timers and returns.
 *              Using this approach, we can be sure that we are not doing much
 *              of execution in timer ISR which executes at highest priority.
 *
 * Parameters   : Pointer to a OS_SIGNAL structure.
 *
 * Returns      : None
 *
 * Side Effects : None
 *
 *************************************************************************/
void  OS_timer_task_function ( OS_SIGNAL *signal )
{
    switch ( signal->type )
    {
    case SIGNAL_BEGIN:
        /*
         * install the timer handler
         */
        break;

    case SIGNAL_END:
        break;

    case TIMER_TICK_SIGNAL:
#ifdef _NEW_SW_TIMER_DISPATCH_
        OS_time_out_timers_new ((UINT16)((UINT32)signal->param));
#else        
        OS_time_out_timers((UINT16)((UINT32)signal->param));
#endif
        break;

    default:
        TIMER_INF(ILLEGAL_SIGNAL_RECEIVED_IN_TIMER_TASK,1,signal->type);
        break;
    }
}

/**************************************************************************
 * Function     : OS_time_out_timers
 *
 * Description  : If the timer at the current index is ready to expire it calls
 *              the timer handler and returns the timer to the free
 *              timer pool. If the timer is of PERIODIC type, It restarts
 *              the timer with its timeout value. If it a ONESHOT timer it is
 *              stopped.
 *
 * Parameters   : cur_index: current index of calendar_table
 *
 * Returns      : None
 *
 * Side Effects : None
 *
 *************************************************************************/
#ifdef _FIX_CROSSBAR_ERROR_DEBUG_
extern SECTION_SRAM OS_TIMER dbg_os_timer;
//#include "mem.h"

#endif

#ifdef _NEW_SW_TIMER_DISPATCH_
UINT8 g_node_num = 0;
void OS_time_out_timers_new (UINT16 cur_index)
{
    DEF_CRITICAL_SECTION_STORAGE;
    OS_TIMER *start = NULL;
    OS_TIMER *prev = NULL;
    OS_TIMER *cur = NULL;
    OS_TIMER_COMPARE node_array[OS_MAX_TIMER_BUCKETS];    
    UINT32 bm_entry_mask[(OS_MAX_TIMER_BUCKETS + 31)/ 32] = {0};
    UINT8 dw_unit_cnt;
    UCHAR loop_count = 0;
    UINT8 i;
    
    OS_TIMER os_timer_tmp;

    g_node_num = 0;
    
    MINT_OS_ENTER_CRITICAL();
    start = calendar_table[cur_index]; 
    
    while (start != NULL)
    {
        cur = start;
    
        if (((UINT32)start < (UINT32)timer_pool) ||
           ((UINT32)start >= (UINT32)(timer_pool + OS_MAX_TIMER_BUCKETS)) ||
            (((UINT32)((UINT32)start - (UINT32)timer_pool) % sizeof(OS_TIMER)) != 0))
        {
            RT_BT_LOG(RED, MSG_SW_TIMER_DISPATCH_WRONG_NODE, 5, 
                    cur_index, loop_count, prev, prev->timeout_function,
                    start);
            break;
        }
            
        if ((loop_count >= OS_MAX_TIMER_BUCKETS) || (start->handle >= OS_MAX_TIMER_BUCKETS))
        {
            RT_BT_LOG(RED, MSG_SW_TIMER_DISPATCH_ERROR, 5, 
                                    cur_index, prev, prev->timeout_function,
                                    start, start->timeout_function);
            break;        
        }
    
        if (start->state != TIMER_RUNNING)
        {
            /* invalid node */
            RT_BT_LOG(YELLOW, TIMER_IS_IN_THE_CALENDAR_TABLE_BUT_NOT_RUNNING, 2, 
                                        start->handle, start->timeout_function);           
            break;
        }

        /* check current is valid single linked-list strcture ? - austin */
        dw_unit_cnt = start->handle >> 5;
        if (bm_entry_mask[dw_unit_cnt] & (1 << (start->handle & 0x1F))) 
        {
            RT_BT_LOG(RED, MSG_SW_TIMER_DISPATCH_RING_ERROR, 3, 
                            cur_index, start, start->timeout_function);
            break;
        }
        else
        {
            bm_entry_mask[dw_unit_cnt] |= (1 << (start->handle & 0x1F));
        }

        if (start->timer_count == 0)
        {
            /* Remove the timer from the calendar table */
            if (prev == NULL)
            {
                /* update calendar table */
                if(calendar_table[cur_index] == NULL)
                {
                    MINT_OS_EXIT_CRITICAL();
                    return; /* Work around */
                }
                else
                {
                    if (start->timeout_function != NULL)
                    {
                        start->state = TIMER_CREATED;

                        /* store partial content of timer entry */
                        node_array[g_node_num].DWord[0] = start->DWord[0];
                        node_array[g_node_num].DWord[1] = start->DWord[1];
                        node_array[g_node_num].DWord[2] = start->DWord[2];
                        node_array[g_node_num].DWord[3] = start->DWord[3];   
                        g_node_num++;
                    }
                    calendar_table[cur_index] = start->next;  
                    start = start->next;
                    cur->next = NULL;
                }
            }
            else
            {
                if (start->timeout_function != NULL)
                {
                    start->state = TIMER_CREATED;
                                            
                    /* store partial content of timer entry */
                    node_array[g_node_num].DWord[0] = start->DWord[0];
                    node_array[g_node_num].DWord[1] = start->DWord[1];
                    node_array[g_node_num].DWord[2] = start->DWord[2];
                    node_array[g_node_num].DWord[3] = start->DWord[3];   
                    g_node_num++;
                }
                prev->next = start->next; 
                start = start->next;
                cur->next = NULL;
            }     
        }
        else
        {
            start->timer_count--;
            prev = start;

#ifdef DEBUG_PDU_RESPONSE_TIMER
            if (g_lmp_pdu_response_timer_check_flag &&
                (start->state == TIMER_RUNNING) && 
                (start->timeout_function == lmp_response_timeout_handler))
            {
                UINT8 i; 
                LMP_CONNECTION_ENTITY *lmp_ce;                   
                UINT32 clk;
                
                for (i = 0; i < LMP_MAX_CE_DATABASE_ENTRIES; i++)
                {
                    lmp_ce = &lmp_connection_entity[i];
                    if (lmp_ce->lmp_response_timer_handle == start->handle)                            
                    {
                        lc_get_clock_in_scatternet(&clk, lmp_ce->phy_piconet_id);
                        
#ifdef _CCH_LPS_							
                         RT_BT_LOG(RED, LPS_LOG_032, 11,
                                clk, i, lmp_ce->phy_piconet_id, 
                                lmp_ce->am_addr,
                                lmp_ce->sent_pdu, 
                                lmp_ce->sent_lmp_pdu_opcode,
                                lmp_ce->sent_lmp_pdu_ext_opcode,
                                lmp_ce->lmp_expected_pdu_opcode,
                                start->handle,
                                start->timer_count,
                                start->sniff_tick_timer);
#else
                         RT_BT_LOG(RED, LMP_MSG_RES_TIMER_INFO_PRO, 10,
                                clk, i, lmp_ce->phy_piconet_id, 
                                lmp_ce->am_addr,
                                lmp_ce->sent_pdu, 
                                lmp_ce->sent_lmp_pdu_opcode,
                                lmp_ce->sent_lmp_pdu_ext_opcode,
                                lmp_ce->lmp_expected_pdu_opcode,
                                start->handle,
                                start->timer_count);
#endif

                        break;
                    }                
                }
            }
#endif
            start = start->next;
        }         
        loop_count++;
    }   
    
    MINT_OS_EXIT_CRITICAL();

    OS_TIMER *pentry;
    
    for (i = 0; i < g_node_num; i++)
    {
        if (node_array[i].handle >= OS_MAX_TIMER_BUCKETS)
        {
            RT_BT_LOG(RED, MSG_SW_TIMER_DISPATCH_NVALID_CONTENT, 5, 
                                    i, node_array[i].DWord[0],
                                        node_array[i].DWord[1], 
                                        node_array[i].DWord[2], 
                                        node_array[i].DWord[3]);
            continue;
        }
        
        pentry = &timer_pool[node_array[i].handle];

        memcpy((UINT8*)&os_timer_tmp, (UINT8*)pentry, sizeof(OS_TIMER));

        if ((node_array[i].DWord[0] ^ pentry->DWord[0]) ||
            (node_array[i].DWord[1] ^ pentry->DWord[1]) ||
            (node_array[i].DWord[2] ^ pentry->DWord[2]) ||
            (node_array[i].DWord[3] ^ pentry->DWord[3]))
        {
            /* any parameters are changed */
            continue;
        }

#ifdef _FIX_CROSSBAR_ERROR_DEBUG_
        memcpy((UINT8*)&dbg_os_timer, (UINT8*)pentry, sizeof(OS_TIMER));
#endif

        //Invoke the Timeout handler.
        os_timer_tmp.timeout_function (os_timer_tmp.handle, os_timer_tmp.args);
        
        /*
         * If it is a periodic timer and it has not been deleted or
         * stopped in the handler start it again
         */
        if ((os_timer_tmp.timer_type == PERIODIC) && (os_timer_tmp.state == TIMER_CREATED))
        {
            OS_start_timer (os_timer_tmp.handle, os_timer_tmp.timer_value);
        }
    }
}
#else
void OS_time_out_timers (UINT16 cur_index)
{
    DEF_CRITICAL_SECTION_STORAGE;
    OS_TIMER *timer = NULL;
    OS_TIMER *start = NULL;
    OS_TIMER *prev = NULL;
    UCHAR loop_count = 0;

    start = calendar_table[cur_index];

    /* check any pending timer schedulers in the list at the calender table */
    while (start != NULL)
    {
        timer = start;

        if (timer->timer_count == 0)
        {
            loop_count++;
            if (loop_count > 10)
            {
                start->next = NULL;
                start = NULL;
                break;
            }

            MINT_OS_ENTER_CRITICAL();
            if (timer->state == TIMER_RUNNING)
            {
                /* Remove the timer from the calendar table */
                if (prev == NULL)
                {
                    if(calendar_table[cur_index] == NULL)
                    {
                        MINT_OS_EXIT_CRITICAL();
                        break; /* Work around */
                    }
                    else
                    {
                        start = start->next;
                        calendar_table[cur_index] = start;
                    }
                }
                else
                {
                    prev->next = start->next;
                    start = start->next;
                }

                timer->state = TIMER_CREATED;
                timer->next = NULL;

#ifdef _FIX_CROSSBAR_ERROR_DEBUG_
                memcpy((UINT8*)&dbg_os_timer, (UINT8*)timer, sizeof(OS_TIMER));
#endif
                
                MINT_OS_EXIT_CRITICAL();

                if (timer->timeout_function != NULL)
                {
                    //Invoke the Timeout handler.
                    timer->timeout_function (timer->handle,timer->args);

#ifdef  _FIX_CROSSBAR_ERROR_
                    /*
                     * If it is a periodic timer and it has not been deleted or
                     * stopped in the handler start it again
                     */
                    if ((timer->timer_type == PERIODIC) &&
                            (timer->state == TIMER_CREATED))
                    {
                        OS_start_timer (timer->handle, timer->timer_value);
                    }
#endif
                }
                else
                {
                    TIMER_ERR(NO_TIMEOUT_FUNCTION_SPECIFIED_FOR_HANDLE, 1, 
                                                            timer->handle );

                }

#ifndef  _FIX_CROSSBAR_ERROR_
                /*
                 * If it is a periodic timer and it has not been deleted or
                 * stopped in the handler start it again
                 */
                if ((timer->timer_type == PERIODIC) &&
                        (timer->state == TIMER_CREATED))
                {
                    OS_start_timer (timer->handle, timer->timer_value);
                }
#endif
            }
            else /* else of if (timer->state == TIMER_RUNNING) */
            {
#ifdef _CHECK_MINT_SW_TIMER_
                RT_BT_LOG(YELLOW, TIMER_IS_IN_THE_CALENDAR_TABLE_BUT_NOT_RUNNING, 1, 
                                                                timer->handle);
#endif

                MINT_OS_EXIT_CRITICAL();

#ifdef _ENHANCE_SW_TIMER_RACE_CONDITION_ 
                break;    
#endif            
            } /* end of if (timer->state == TIMER_RUNNING) */
        }
        else /* else of if (timer->timer_count == 0) */
        {
            MINT_OS_ENTER_CRITICAL();
            if(timer->state == TIMER_RUNNING)
            {
                timer->timer_count --;
                prev    = start;
                start   = start->next;
                MINT_OS_EXIT_CRITICAL();
            }
            else
            {
#ifdef _CHECK_MINT_SW_TIMER_
                RT_BT_LOG(YELLOW, ERROR_TIMER_LEAK, 3,
                         timer->state,  timer->timeout_function, start->next);
#endif

                MINT_OS_EXIT_CRITICAL();

#ifdef _ENHANCE_SW_TIMER_RACE_CONDITION_ 
                break;    
#endif            
            }
            
#ifdef DEBUG_PDU_RESPONSE_TIMER
            if (g_lmp_pdu_response_timer_check_flag &&
                (timer->state == TIMER_RUNNING) && 
                (timer->timeout_function == lmp_response_timeout_handler))
                {
                    UINT8 i; 
                    LMP_CONNECTION_ENTITY *lmp_ce;                   
                    UINT32 clk;
                    
                    for (i = 0; i < LMP_MAX_CE_DATABASE_ENTRIES; i++)
                    {
                        lmp_ce = &lmp_connection_entity[i];
                        if (lmp_ce->lmp_response_timer_handle == timer->handle)                            
                        {
                            lc_get_clock_in_scatternet(&clk, lmp_ce->phy_piconet_id);
                            
#ifdef _CCH_LPS_							
                             RT_BT_LOG(RED, LPS_LOG_032, 11,
                                    clk, i, lmp_ce->phy_piconet_id, 
                                    lmp_ce->am_addr,
                                    lmp_ce->sent_pdu, 
                                    lmp_ce->sent_lmp_pdu_opcode,
                                    lmp_ce->sent_lmp_pdu_ext_opcode,
                                    lmp_ce->lmp_expected_pdu_opcode,
                                    timer->handle,
                                    timer->timer_count,
                                    timer->sniff_tick_timer);
#else
                             RT_BT_LOG(RED, LMP_MSG_RES_TIMER_INFO_PRO, 10,
                                    clk, i, lmp_ce->phy_piconet_id, 
                                    lmp_ce->am_addr,
                                    lmp_ce->sent_pdu, 
                                    lmp_ce->sent_lmp_pdu_opcode,
                                    lmp_ce->sent_lmp_pdu_ext_opcode,
                                    lmp_ce->lmp_expected_pdu_opcode,
                                    timer->handle,
                                    timer->timer_count);
#endif

                            break;
                        }                
                    }
                }
#endif
        } /* end of if (timer->timer_count == 0) */
    } /* end of while (start != NULL) */
}
#endif /* _NEW_SW_TIMER_DISPATCH_ */

/**************************************************************************
 * Function     : OS_timer_mgmt_init
 *
 * Description  : This function intialises the timer managment component. It
 *              primarily initialises the calendar table and makes the list
 *              of free timers from the timer pool.
 *
 * Parameters   : None
 *
 * Returns      : OK
 *
 * Side Effects : None
 *
 *************************************************************************/
void OS_timer_mgmt_init(void)
{
    int index;

    for(index = 0; index < OS_MAX_TIMER_BUCKETS; index++)
    {
        calendar_table[index] = NULL;
    }

    for ( index = 0; index < OS_MAX_TIMER_BUCKETS - 1; index++ )
    {
        timer_pool[index].handle            = index;
        timer_pool[index].cal_table_index   = 0;
        timer_pool[index].timeout_function  = NULL;
        timer_pool[index].args              = NULL;
        timer_pool[index].timer_type        = ONESHOT;
        timer_pool[index].next              = &timer_pool[index+1];
        timer_pool[index].state             = TIMER_FREE;
#ifdef _CCH_LPS_
        timer_pool[index].sniff_tick_timer  = 0;
#endif
    }

    timer_pool[index].handle            = OS_MAX_TIMER_BUCKETS - 1;
    timer_pool[index].cal_table_index   = 0;
    timer_pool[index].timeout_function  = NULL;
    timer_pool[index].args              = NULL;
    timer_pool[index].timer_type        = ONESHOT;
    timer_pool[index].next              = NULL;
    timer_pool[index].state             = TIMER_FREE;
#ifdef _CCH_LPS_
    timer_pool[index].sniff_tick_timer  = 0;
#endif

    free_list_head = &timer_pool[0];     
    free_list_tail = &timer_pool[index];
    current_index = 0;
    timer_value = 0;
}

/**************************************************************************
 * Function     : OS_get_free_timer
 *
 * Description  : To get the timer unit from the free list 
 *
 * Parameters   : new_timer: the pointer of new timer unit from the free list
 *
 * Returns      : None
 *
 * Side Effects :
 *
 *************************************************************************/
void OS_get_free_timer(OS_TIMER **new_timer)
{
    *new_timer = NULL;

    if(free_list_head == NULL)
    {
        TIMER_ERR(ALL_TIMERS_ARE_USED_UP,0,0);
        return;
    }

    *new_timer = free_list_head;
    
    /* Get the next free timer */
    free_list_head = free_list_head->next;
    (*new_timer)->next = NULL;

    if (free_list_head == NULL)
    {
        /* empty */
        free_list_tail = NULL;
    }
}


/**************************************************************************
 * Function     : OS_return_timer
 *
 * Description  : To return the timer unit back the free list  
 *
 * Parameters   : timer: returned timer 
 *
 * Returns      : None
 *
 * Side Effects :
 *
 *************************************************************************/
void OS_return_timer(OS_TIMER *timer)
{
    DEF_CRITICAL_SECTION_STORAGE;
    MINT_OS_ENTER_CRITICAL();
    timer->state = TIMER_FREE;
    timer->next = NULL;

    if(free_list_head == NULL)
    {
        free_list_head = timer;
    }
    else
    {
        free_list_tail->next = timer;
    }
    free_list_tail = timer;    
    MINT_OS_EXIT_CRITICAL();
}


/**************************************************************************
 * Function     : OS_create_timer
 *
 * Description  :
 *
 * Parameters   : Type of timer to be created, timer handler and arguments to
 *              timer handler, if any.
 *
 * Returns      : Handle to the timer.
 *
 * Side Effects : None
 *
 *************************************************************************/
OS_HANDLE   OS_create_timer (OS_TIMER_TYPE timer_type,
                             OS_TIMEOUT_HANDLER timer_handler,
                             OS_ADDRESS args )
{
    OS_TIMER    *new_timer = NULL;
    DEF_CRITICAL_SECTION_STORAGE;

    if((timer_type != ONESHOT) && (timer_type != PERIODIC))
    {
        TIMER_ERR(INVALID_TIMER_TYPE,0,0);
        return OS_INVALID_HANDLE;
    }

    if(timer_handler == NULL)
    {
        TIMER_ERR(INVALID_TIMEOUT_FUNCTION,0,0);
        return OS_INVALID_HANDLE;
    }
    MINT_OS_ENTER_CRITICAL();    

    /* Get a Free timer from the pool */
    OS_get_free_timer(&new_timer);

    if(new_timer == NULL)
    {
        MINT_OS_EXIT_CRITICAL();
        TIMER_ERR(DID_NOT_GET_ANY_FREE_TIMERS,0,0);
        HCI_LOG_ERROR(LOG_LEVEL_HIGH, TIMER_CREATION_FAILED,0,0);
        return OS_INVALID_HANDLE;
    }

    new_timer->timer_type       = timer_type;
    new_timer->timeout_function = timer_handler;
    new_timer->args             = args;
    new_timer->state            = TIMER_CREATED;
    new_timer->next             = NULL;

    MINT_OS_EXIT_CRITICAL();

    return(new_timer->handle);
}


/**************************************************************************
* Function      : OS_delete_timer
*
* Description   :
*
* Parameters    : Timer handle.
*
* Returns       : ERROR if timer is running and it cannot be stopped, else OK.
*
* Side Effects  : None
*
*************************************************************************/

OS_STATUS   OS_delete_timer ( OS_HANDLE timer_handle )
{
    OS_TIMER *timer;

    if((timer_handle < 0) || (timer_handle >= OS_MAX_TIMER_BUCKETS))
    {
        HCI_LOG_ERROR(LOG_LEVEL_HIGH, INVALID_TIMER_HANDLE,1,timer_handle);
        return ERROR;
    }

    timer = &timer_pool[timer_handle];

    if(timer->state == TIMER_FREE)
    {
        HCI_LOG_ERROR(LOG_LEVEL_HIGH, TIMER_IS_NOT_RUNNING, 1,timer_handle);
        return ERROR;
    }

    if(timer->state == TIMER_RUNNING)
    {
        TIMER_INF(STOPING_A_RUNNING_TIMER,0,0);
        OS_stop_timer(timer_handle);
    }

    OS_return_timer(timer);

    return (OK);
}


/**************************************************************************
* Function      : OS_start_timer
*
* Description   :
*
* Parameters    : A handle to the timer and a time out period.
*
* Returns       : OK
*
* Side Effects  : None
*
*************************************************************************/

OS_STATUS   OS_start_timer ( OS_HANDLE timer_handle, UINT32 value )
{
    OS_TIMER    *timer;
    UINT16      new_index;

    DEF_CRITICAL_SECTION_STORAGE;
    if((timer_handle < 0) || (timer_handle >= OS_MAX_TIMER_BUCKETS))
    {
        TIMER_ERR(INVALID_TIMER_HANDLE_1,0,0);
        return ERROR;
    }

    if(value == 0)
    {
        TIMER_ERR(INVALID_TIMER_VALUE_OF_0,0,0);
        return ERROR;
    }

    MINT_OS_ENTER_CRITICAL();

    timer = &timer_pool[timer_handle];

    if(timer->state == TIMER_FREE)
    {
        MINT_OS_EXIT_CRITICAL();
        return ERROR;
    }
    
    if(timer->state == TIMER_RUNNING)
    {
        TIMER_ERR(TIMER_IS_ALREADY_RUNNING,0,0);
        OS_stop_timer(timer_handle);
    }

    /*
     * Calculate the position of timer in the calendar table
     * depending on the timer value
     */
    new_index = (UINT16)((current_index + value) & (OS_MAX_TIMER_BUCKETS - 1));

    /* Store the Timeout value and the timer count */
    timer->timer_value = value;
    timer->timer_count = (UINT16)(value / OS_MAX_TIMER_BUCKETS);

    if((new_index == current_index) && (timer->timer_count == 0))
    {
        /* the handling has no problem (if time_value is N*OS_MAX_TIMER_BUCKETS,
           the expired time will be adjusted ?) - austin  */
        new_index = (UINT16)((new_index + 1) & (OS_MAX_TIMER_BUCKETS - 1));
    }
    
    timer->cal_table_index  = new_index;
    timer->state            = TIMER_RUNNING;
    timer->next             = NULL;

    /* insert the timer in the calendar table */
    if ( calendar_table[new_index] == NULL )
    {
        calendar_table[new_index] = timer;
    }
    else
    {
        OS_TIMER    *timerCur;
        timerCur = calendar_table[new_index];
        while (timerCur->next != NULL)
        {
            timerCur = timerCur->next;
        }
        timerCur->next = timer;
    }

#ifdef _CCH_LPS_
    timer->sniff_tick_timer = 0;
#endif

    MINT_OS_EXIT_CRITICAL();

    return (OK);
}


/**************************************************************************
* Function      : OS_stop_timer
*
* Description   :
*
* Parameters    : a handle to the timer and a time out period.
*
* Returns       : OK
*
* Side Effects  : None
*
*************************************************************************/

OS_STATUS   OS_stop_timer ( OS_HANDLE timer_handle )
{
    OS_TIMER    *timer;
    OS_TIMER    *cal_table_ptr,*prev_ptr;
    UINT16      cal_table_index;
    DEF_CRITICAL_SECTION_STORAGE;

    if((timer_handle < 0) || (timer_handle >= OS_MAX_TIMER_BUCKETS))
    {
        TIMER_ERR(INVALID_TIMER_HANDLE_1,0,0);
        return ERROR;
    }

    MINT_OS_ENTER_CRITICAL();

    timer = &timer_pool[timer_handle];

    if(timer->state != TIMER_RUNNING)
    {
        MINT_OS_EXIT_CRITICAL();
        return ERROR;
    }

#ifdef _CCH_LPS_
    timer->sniff_tick_timer = 0;
#endif
    
    /* Remove the timer from the calendar table */
    cal_table_index     = timer->cal_table_index;
    cal_table_ptr       = calendar_table[cal_table_index];
    prev_ptr            = NULL;

    /* check the stopped timer handle in the list of calendar table */
    while(cal_table_ptr != NULL)
    {
        if(cal_table_ptr->handle == timer_handle)
        {
            break;
        }
        prev_ptr        = cal_table_ptr;
        cal_table_ptr   = cal_table_ptr->next;
    }

    if(cal_table_ptr == NULL)
    {
        /*
         * This case might happen when the timer is being stopped within the
         * handler return SUCCESS here
         */
        TIMER_ERR(TIMER_IS_NOT_IN_THE_CALENDAR_TABLE,0,0);
        timer->next     = NULL;
        timer->state = TIMER_CREATED;

        MINT_OS_EXIT_CRITICAL();
        return OK;
    }

    /* chain the list of calendar table */
    if(prev_ptr == NULL)
    {
        /* This is the first timer in the Calendar Table */
        calendar_table[cal_table_index] = timer->next;
    }
    else
    {
        prev_ptr->next = timer->next;
    }
    timer->next     = NULL;
    timer->state    = TIMER_CREATED;

    MINT_OS_EXIT_CRITICAL();

    return (OK);
}


/**************************************************************************
 * Function   : OS_is_timer_running
 *
 * Description: Checks if a timer is running.
 *
 * Parameters : Timer id and address where residual period is returned.
 *
 * Returns    : TRUE or FALSE
 *
 * Side Effects : None
 *
 *************************************************************************/
UINT32 OS_is_timer_running(OS_HANDLE timer_handle)
{
    UINT32 status;
    DEF_CRITICAL_SECTION_STORAGE;
    
    if(timer_handle == OS_INVALID_HANDLE)
    {
        return FALSE;
    }

    MINT_OS_ENTER_CRITICAL();

    status =
        ((timer_pool[(timer_handle)].state == TIMER_RUNNING) ? TRUE: FALSE);

    MINT_OS_EXIT_CRITICAL();

    return status;

}

void pf_timer_handle_tick(void)
{
    /* Increase the time out value */
    timer_value++;
    
    if (calendar_table[current_index] != NULL )
    {
        OS_SIGNAL timer_tick_signal;

        timer_tick_signal.type = TIMER_TICK_SIGNAL;
        timer_tick_signal.param = (OS_ADDRESS)((UINT32)current_index);        
        OS_ISR_SEND_SIGNAL_TO_TASK(timer_task_handle,timer_tick_signal);
    }
    
    current_index = (UINT16)((current_index + 1) & (OS_MAX_TIMER_BUCKETS - 1));
}

