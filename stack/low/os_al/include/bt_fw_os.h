/***************************************************************************
 Copyright(C)MindTree Ltd.
 This module is a confidential and proprietary property of MindTree and
 a possession or use of this module requires written permission of MindTree.
 ***************************************************************************/

#ifndef _BT_FW_OS_H_
#define _BT_FW_OS_H_

#include "mint_os.h"
#include "timers.h"

/*
 * \file
 *  Defines the MINT OS application programming interface (API).
 */

/**
 * \addtogroup mint_os MINT OS
 *  Defines the MINT OS application programming interface (API).

MINT_OS supports what protocol stack implementation need from an OS:
 -# Task Management Services
 -# Timer Management Services
 -# Buffer Management Services
 -# Event/Signal Management Services

\section tsk_mgmt_sec Task Management Services

The protocol stack requires a lightweight multitasking framework and hence the
scheduling algorithm used for task scheduling is priority based non-
preemptive scheduling. To that extent, the task management services have been
designed to be simple functions with a common prototype having static
priorities, which are invoked only by the MINT_OS dispatcher.

Therefore, some considerations and restrictions for any task are:
- A task will never invoke another task directly, but will only be invoked by
  the MINT_OS dispatcher
- A task will always run to completion in a finite time for each signal type
  for which it is entered. The finite time duration is application specific and
  is determined based on the role of the task in the overall scheme of
  computations. Thus the task shall divide its processing into finite segments,
  short enough not to delay rescheduling of other higher priority tasks which
  are ready and long enough to complete meaningful and atomic computations.
- A task will never block, never directly disable interrupts or never perform an
  infinite busy loop. Instead, it should always await a signal.
- A task will never be pre-empted for each invocation, except by an interrupt
  service routine. Thus, the task can have unrestricted access to any global
  data for each invocation, provided the global data is not shared with an ISR.
- A task will be characterized by the attributes viz., task priority,
  busy-period-upper-bound and idle-period-lower-bound. The latter two attributes
  are used to support an extended multi-tasking policy though available, is not
  tested.

\section tmr_mgmt_sec Timer Management Services

Any protocol software will have to manage multiple simultaneous timers, while
the underlying hardware device may provide very limited number of timers.
Therefore, the OS Timer Management module is included to multiplex a large
number of timers on top of a single hardware timer.

The design constraints and parameters for the timer module are:
- Number of simultaneous timers
- Resolution and diversity in the granularity of the timers
- In protocol software, many of the events are driven by the underlying
  periodicity present in the physical layer The ratio of timers started to
  expired timers and stopped timers
- In a normal operation of the system, most timers will be stopped before it
  expires. For example, a response timer will be stopped when the response
  arrives

\section buf_mgmt_sec Buffer Management Services
Most run-time systems provide a dynamic memory allocation mechanism such as
malloc() for dynamic creation of objects. But these suffer from the problems
of fragmentation due to variable size allocations and also of
non-deterministic delays. For protocol systems, most memory buffer
requirements are for messages (PDU or inter-layer interface) and hence are of
known maximum size. Therefore, the buffer management component of the MINT_OS
module will be based on the notion of multiple fixed-size Buffer Pools. These
pools are configurable in terms of size and number of buffers. They are also
created at initialization time and only managed henceforth.

\section evt_mgmt_sec Event/Signal Management Services
The inter-task communication mechanism in the protocol software is that of
event signals. The protocol software defines a global set of signals. The
MINT_OS task management services provide only one API to deliver the signal to
a particular task. This API works transparently when protocol tasks are
embedded in multiple native OS processes, by using appropriate native ITC
mechanisms like mailboxes or message queues.

The MINT_OS takes care of accepting the signals from external entities (tasks
or ISRs) queued at the mailboxes, and routes them to the appropriate task to
which the signals is destined.

Signals are globally defined in the system by unique signal type enumerations.
Signals can be grouped into internal and external signals. Internal signals
are generated and responded to by tasks that are within the MINT_OS tasks.
Thus, they require no special mailboxes or other IPC mechanisms of the native
OS.

\section mint_os_config Configuration:
- OS_TOTAL_TASKS          Maximum number of tasks
- OS_MAX_HEAP_MEMORY      Maximum Memory allocated for MINT OS.
                          Inclusive of buffer pool memory requirements.
- OS_MAX_POOLS            Maximum Number of buffer pools
- OS_MAX_TIMER_BUCKETS    Maximum Number of Timers

\section example_sec Example:
\include mint_os.c

 * @{ */

#ifndef USE_FREERTOS
/**
 * Timer ID/handle.
 */
typedef     OS_HANDLE           TIMER_ID;
#endif
/**
 * Task ID/handle.
 */
typedef     OS_HANDLE           TASK_ID;
/**
 * Buffer Pool ID/handle.
 */
typedef     OS_HANDLE           POOL_ID;

/* generic errors */
#define     BT_ERROR_OK                         ((API_RESULT)0x00)
#define     BT_ERROR_INVALID_PARAMETER          ((API_RESULT)0x01)
#define     BT_ERROR_INSUFFICIENT_RESOURCE      ((API_RESULT)0x02)
#define     BT_ERROR_INVALID_OPERATION          ((API_RESULT)0x03)
#define     BT_ERROR_INTERNAL_ERROR             ((API_RESULT)0x04)

/*
 * Generic ERROR return in case of failure of any os call.
 */
#define     BT_ERROR                            ((API_RESULT)0x05)

API_RESULT  os_wrapper_create_pool(POOL_ID *p_pool_id, unsigned int num_buf,
                                            unsigned int buf_size, UINT8 m_alignment, 
                                            UINT8 m_compensate, RAM_TYPE mem_mode );

API_RESULT  os_wrapper_reset_pool(POOL_ID *pool_id);
API_RESULT  os_wrapper_delete_pool(POOL_ID pool_id);
API_RESULT  os_wrapper_alloc_buffer(POOL_ID pool_id, void **pp_buffer);
API_RESULT  os_wrapper_free_buffer(POOL_ID pool_id, void *p_buffer);
UINT32 os_wrapper_get_free_buffer(POOL_ID pool_id);

/* Timer Management API */
API_RESULT os_wrapper_create_timer(unsigned char timer_type, TimerHandle_t *timer_id, TimerCallbackFunction_t timer_func, void* arg, unsigned long value);
API_RESULT os_wrapper_delete_timer(TimerHandle_t *timer_id);
API_RESULT os_wrapper_start_timer(TimerHandle_t timer_id, unsigned long value);
API_RESULT os_wrapper_stop_timer(TimerHandle_t timer_id, unsigned short *value);
UINT32 os_wrapper_is_timer_running(TimerHandle_t timer_handle);
UINT32 os_wrapper_get_time(void);
void os_wrapper_set_time(UINT32 time);

/* Task Management API */
API_RESULT  os_wrapper_create_task(TASK_ID *task_id, char* name,
								   unsigned char priority,
								   OS_TASK_FUNCTION task_func,
								   unsigned int stack_size,
								   unsigned short busy_period);
API_RESULT  os_wrapper_reset_task(TASK_ID  task_id);
API_RESULT  os_wrapper_delete_task(TASK_ID  task_id);
API_RESULT  os_wrapper_send_signal_to_task(TASK_ID task_id, OS_SIGNAL signal);
API_RESULT  os_wrapper_send_signal_to_task_head(TASK_ID task_id, OS_SIGNAL signal);

/* ===================== Buffer Management API ========================== */
/**
 * \addtogroup mint_os_buf_mgmt Buffer Management API
 *  Defines the MINT OS buffer management API. The buffer management involves
 *  creation/deletion of buffer pools and allocation/freeing of buffers. A
 *  system can have any number of buffer pools and each buffer pool can have
 *  any number of fixed size buffers.
 * @{ */

/**
 * Creates a buffer pool and returns a valid pool id (\a P_POOL_ID) on
 * success.
 *
 * \param[out] P_POOL_ID Pointer to store the newly allocated pool id.
 *                       This #POOL_ID to be used for all further buffer
 *                       pool transaction.
 * \param[in] NUM_BUF Number of buffers to create in the pool.
 * \param[in] BUF_SIZE The size (in bytes) of the buffer.
 *
 * \return BT_ERROR_OK on successful operation. BT_ERROR otherwise.
 * \sa OS_RESET_POOL, OS_ALLOC_BUFFER, OS_FREE_BUFFER, OS_GET_FREE_BUFFERS.
 * 
 */
#define   OS_CREATE_POOL(P_POOL_ID, NUM_BUF, BUF_SIZE, M_ALI, M_COM, MEM_MODE) \
					os_wrapper_create_pool ( P_POOL_ID, NUM_BUF, BUF_SIZE,     \
					                        M_ALI, M_COM, MEM_MODE )

/**
 * Resets the given buffer pool. It frees all the buffers in the pool. This
 * operation is equivalent to reclaiming (#OS_FREE_BUFFER) all the allocated
 * (#OS_ALLOC_BUFFER) buffers from the specified buffer pool.
 *
 * \param[in] P_POOL_ID Pointer to the pool id to be reset.
 *
 * \return BT_ERROR_OK on successful operation. Any valid BT_ERROR error code
 *         otherwise.
 * \sa OS_CREATE_POOL, OS_ALLOC_BUFFER, OS_FREE_BUFFER, OS_GET_FREE_BUFFERS.
 */
#define     OS_RESET_POOL(P_POOL_ID) \
                    os_wrapper_reset_pool(P_POOL_ID)

/**
 * Deletes the given pool id.
 *
 * \param[in] POOL_ID Pool id to be deleted.
 *
 * \return BT_ERROR_OK on successful operation. BT_ERROR otherwise.
 *
 * \deprecated This function is not supported in newer versions.
 */
#define OS_DELETE_POOL(pool_id) \
                    os_wrapper_delete_pool(pool_id)

/**
 * Allocates a buffer from the given \a POOL_ID and stores it in \a PP_BUFFER.
 *
 * \param[in] POOL_ID Pool id from which the buffer has to be allocated.
 * \param[out] PP_BUFFER Pointer to store the allocated buffer if any.
 *
 * \return BT_ERROR_OK on successful operation. BT_ERROR otherwise.
 * \sa OS_CREATE_POOL, OS_RESET_POOL, OS_FREE_BUFFER, OS_GET_FREE_BUFFERS.
 */
#define  OS_ALLOC_BUFFER(pool_id, PP_BUFFER) \
                    os_wrapper_alloc_buffer(pool_id, PP_BUFFER)



/**
 * Returns the given buffer back to the buffer pool.
 *
 * \param[in] POOL_ID Pool id of the buffer to be returned/freed.
 * \param[in] P_BUFFER Buffer to be returned/freed.
 *
 * \return BT_ERROR_OK on successful operation. BT_ERROR otherwise.
 * \sa OS_CREATE_POOL, OS_RESET_POOL, OS_ALLOC_BUFFER, OS_GET_FREE_BUFFERS.
 */
#define   OS_FREE_BUFFER(pool_id, P_BUFFER) \
                    os_wrapper_free_buffer(pool_id, P_BUFFER)

/**
 * Returns the number of free buffers available in the buffer pool
 * given by \a POOL_ID.
 *
 * \param[in] POOL_ID Pool id of the buffer pool.
 *
 * \return Number of free buffers in the given \a POOL_ID.
 * \sa OS_CREATE_POOL, OS_RESET_POOL, OS_ALLOC_BUFFER, OS_FREE_BUFFER.
 */
#define     OS_GET_FREE_BUFFERS(pool_id) \
                    os_wrapper_get_free_buffer(pool_id)

/** @} end: mint_os_buf_mgmt */

/* ===================== Timer Management API ========================== */
/**
 * \addtogroup mint_os_timer_mgmt Timer Management API
 *  Defines the MINT OS Timer management API. The timer management involves
 *  creation/deletion, starting/stopping of the timers.
 * @{ */

/**
 * One shot timer. The timer is not reloaded automatically after the timeout.
 */
#define  ONESHOT_TIMER              ((unsigned char)0)
/**
 * Periodic timer. The timer is reloaded automatically after the timeout.
 * It has to be explicitly stopped in order to stop the timer.
 */
#define  PERIODIC_TIMER             ((unsigned char)1)

/**
 * Creates a new timer and returns its timer handle on success.
 *
 * \param[in] TIMER_TYPE Type of the timer to be created (#ONESHOT_TIMER,
 *            #PERIODIC_TIMER).
 * \param[out] P_TIMER_ID Pointer to store the timer id/handle of the newly
 *             created timer.
 * \param[in] TIMER_PROC Timeout handler to be invoked upon timeout.
 * \param[in] TIMER_ARG User defined argument to be passed to be \a TIMER_PROC
 *            during the timeout invocation.
 * \param[in] TIMER_VAL Timer value.
 *
 * \return BT_ERROR_OK on successful operation. BT_ERROR otherwise.
 *
 * \sa OS_TIMEOUT_HANDLER, OS_DELETE_TIMER, OS_START_TIMER, OS_STOP_TIMER,
 * OS_IS_TIMER_RUNNING, OS_GET_TIME, OS_SET_TIME.
 */
#define OS_CREATE_TIMER(timer_type, P_TIMER_ID, TIMER_PROC, TIMER_ARG, TIMER_VAL) \
        os_wrapper_create_timer(timer_type, P_TIMER_ID, TIMER_PROC,     \
                                         TIMER_ARG, TIMER_VAL)

/**
 * Deletes the timer given by \a P_TIMER_ID.
 *
 * \param[in,out] P_TIMER_ID Pointer to timer ID/Handle to be deleted. It also
 *                           makes the P_TIMER_ID invalid.
 *
 * \return BT_ERROR_OK on successful operation. BT_ERROR otherwise.
 * \sa OS_CREATE_TIMER, OS_START_TIMER, OS_STOP_TIMER, OS_IS_TIMER_RUNNING,
 * OS_GET_TIME, OS_SET_TIME.
 */
#define     OS_DELETE_TIMER(P_TIMER_ID) \
                    os_wrapper_delete_timer(P_TIMER_ID)

/**
 * Starts the given timer (\a TIMER_ID) with the given timeout value (\a
 * TIMER_VAL).
 *
 * \param[in] TIMER_ID ID of the timer to be started.
 * \param[in] TIMER_VAL Timeout value.
 *
 * \return BT_ERROR_OK on successful operation. BT_ERROR otherwise.
 * \sa OS_CREATE_TIMER, OS_DELETE_TIMER, OS_STOP_TIMER, OS_IS_TIMER_RUNNING,
 * OS_GET_TIME, OS_SET_TIME.
 */
#define     OS_START_TIMER(timer_id, TIMER_VAL) \
                    os_wrapper_start_timer(timer_id, TIMER_VAL)

/**
 * Stops the given timer (\a TIMER_ID) and returns the residual time.
 *
 * \param[in] TIMER_ID ID of the timer to be stopped.
 * \param[out] VALUE Pointer to store the residual time (time to actual timeout
 *                   value).
 *
 * \return BT_ERROR_OK on successful operation. BT_ERROR otherwise.
 * \sa OS_CREATE_TIMER, OS_DELETE_TIMER, OS_START_TIMER, OS_IS_TIMER_RUNNING,
 * OS_GET_TIME, OS_SET_TIME.
 */
#define     OS_STOP_TIMER(timer_id, VALUE) \
                    os_wrapper_stop_timer(timer_id, VALUE)

/**
 * Returns whether the given timer (\a TIMER_ID) is running or not.
 *
 * \param[in] TIMER_ID ID of timer for which the running status is required.
 *
 * \return TRUE, if the timer is running. FALSE, otherwise.
 * \sa OS_CREATE_TIMER, OS_DELETE_TIMER, OS_START_TIMER, OS_STOP_TIMER,
 * OS_GET_TIME, OS_SET_TIME.
 */
#define     OS_IS_TIMER_RUNNING(timer_id) \
                    os_wrapper_is_timer_running(timer_id)

/* ===================== Task Management API ========================== */
/**
 * \addtogroup mint_os_task_mgmt Task Management API
 *  Defines the MINT OS Task management API. The task management involves
 *  creation/deletion of tasks and inter-task communication.
 * @{ */

/**
 * Initialize MINT OS. This must be called before using any other MINT OS API's.
 *
 * \param None.
 * \return None.
 *
 * \sa OS_CREATE_TASK, OS_DELETE_TASK, OS_SEND_SIGNAL_TO_TASK, OS_SCHEDULER
 */

/**
 * Creates a task managed by MINT OS and returns its task id on success.
 *
 * \param[out] P_TASK_ID Pointer to store the task id of the newly created task.
 * The #TASK_ID to be used for any further task operations.
 * \param[in] TASK_NAME Name of the task.
 * \param[in] TASK_PRIO Priority of the task (It must be unique for each task
 *                      in the system). Smaller the value, higher the task
 *                      priority.
 * \param[in] TASK_FUNC Task function (#OS_TASK_FUNCTION). This is function
 *                      which will be executed whenever a signal is posted to
 *                      this task.
 * \param[in] TASK_STACK Task signal queue size. It determines the number of
 *                       outstanding signals that can be present for the task
 *                       at any point of time.
 * \param[in] TASK_BUSY_PERIOD Task busy period.
 *
 * \return BT_ERROR_OK on successful operation. BT_ERROR otherwise.
 *
 * \sa OS_TASK_FUNCTION, OS_INIT, OS_DELETE_TASK, OS_SEND_SIGNAL_TO_TASK, OS_SCHEDULER.
 */
#define     OS_CREATE_TASK(P_TASK_ID, TASK_NAME, TASK_PRIO, TASK_FUNC, TASK_STACK, TASK_BUSY_PERIOD) \
                    os_wrapper_create_task(P_TASK_ID, TASK_NAME, TASK_PRIO, TASK_FUNC, TASK_STACK,  \
                                         TASK_BUSY_PERIOD)

/**
 * Resets the task given by \a TASK_ID.
 *
 * \param[in] TASK_ID ID/Handle of the task to be resetted.
 *
 * \return BT_ERROR_OK on successful operation. Any valid BT_ERROR error code
 *         otherwise.
 * \sa OS_INIT, OS_CREATE_TASK, OS_SEND_SIGNAL_TO_TASK, OS_SCHEDULER.
 */
#define     OS_RESET_TASK(task_id) \
                    os_wrapper_reset_task(task_id)
/**
 * Deletes the task given by \a TASK_ID.
 *
 * \param[in] TASK_ID ID/Handle of the task to be deleted.
 *
 * \return BT_ERROR_OK on successful operation. Any valid BT_ERROR error code
 *         otherwise.
 * \sa OS_INIT, OS_CREATE_TASK, OS_SEND_SIGNAL_TO_TASK, OS_SCHEDULER.
 */
#define     OS_DELETE_TASK(task_id) \
                    os_wrapper_delete_task(task_id)

/**
 * Sends the given signal (\a SIGNAL) to the given task (\a TASK_ID).
 * This function results in queuing of the signal(\a SIGNAL) in the target
 * task's(\a TASK_ID) signal queue, thus making the task eligible to run in
 * the next scheduling scan cycle. 
 * 
 *
 * \param[in] TASK_ID ID of the task to which the signal has to be sent.
 * \param[in] SIGNAL Signal to be sent (#OS_SIGNAL).
 *
 * \return BT_ERROR_OK on successful operation. BT_ERROR if the TASK_ID is
 * invalid or the signal queue is full.
 * \sa OS_INIT, OS_CREATE_TASK, OS_DELETE_TASK, OS_SCHEDULER.
 */
#define  OS_SEND_SIGNAL_TO_TASK(task_id, SIGNAL) \
                    os_wrapper_send_signal_to_task(task_id, SIGNAL)

#define OS_ISR_SEND_SIGNAL_TO_TASK(task_id, SIGNAL) \
    OS_SEND_SIGNAL_TO_TASK(task_id, SIGNAL)

#define  OS_SEND_SIGNAL_TO_TASK_HEAD(task_id, SIGNAL) \
                    os_wrapper_send_signal_to_task_head(task_id, SIGNAL)

#define OS_ISR_SEND_SIGNAL_TO_TASK_HEAD(task_id, SIGNAL) \
    OS_SEND_SIGNAL_TO_TASK_HEAD(task_id, SIGNAL)

/**
 * Starts the scheduling process, allowing MINT OS to manage the tasks that
 * have been created.  Before OS_SCHEDULER() is called, ensure that all the
 * initialization of the essential parts of the system like OS(OS_init),
 * platform, etc. are completed.
 *
 * \param None.
 * \return None.
 *
 * \sa OS_INIT, OS_CREATE_TASK, OS_DELETE_TASK, OS_SEND_SIGNAL_TO_TASK.
 */

/** @} end: mint_os_task_mgmt */

/** @} end: mint_os */

#define     OS_DEBUG_EXECUTE_TASK(task_id)/* null_macro */

extern POOL_ID dma_rx_pool_id;
extern POOL_ID dma_tx_pool_id;

INLINE UINT16 os_get_reserved_buffers(void);

void  os_reserve_buffer(void);

void  os_free_reserved_buffer(void);

void os_reset_reserved_buffer(void);


INLINE UINT8  os_read_occupied_le_reserved_buffers(void);

INLINE void os_occupy_le_reserve_buffers_from_isr(UINT8 buffer_num);

INLINE void os_occupy_le_reserve_buffers(UINT8 buffer_num);

INLINE void os_release_one_le_reserved_buffer(void);

INLINE void os_reset_le_reserved_buffer(void);

#endif  /*_BT_FW_OS_H_*/

