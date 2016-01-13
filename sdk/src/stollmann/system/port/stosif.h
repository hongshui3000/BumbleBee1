/**********************************************************************!MA*
 *
 * $Header: /var/lib/cvs/sw/src/system/port/stosif.h,v 1.2 2013/12/03 13:11:51 hg Exp $
 *
 * File:        $RCSfile: stosif.h,v $
 * Version:     $Name:
 *
 * Archive:     $Source: /var/lib/cvs/sw/src/system/port/stosif.h,v $
 *
 * ------------------------------------------------------------------------
 * !MODULE      [  ]
 * ------------------------------------------------------------------------
 * !FILE        [  ]
 * !PROGRAM     [  ]
 * !VERSION     [$Name:]
 * !GROUP       [  ]
 * ------------------------------------------------------------------------
 *
 *          Copyright (c)           2010 Stollmann E+V GmbH
 *                                  Mendelssohnstr. 15
 *                                  22761 Hamburg
 *                                  Phone: 040/89088-0
 *          All Rights Reserved
 *
 * ------------------------------------------------------------------------
 * !DESCRIPTION
 *          Stollmann OS interface
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
 *  Date      Author          Comment
 *  tt.mm.jj                  Initial revision
 *  tt.mm.jj
 * ------------------------------------------------------------------------
 *
 * $Log: stosif.h,v $
 * Revision 1.2  2013/12/03 13:11:51  hg
 * issue #11539
 * [ELISA III P] create ISDN strack library.
 *
 * Revision 1.1  2013/05/23 10:44:42  mn
 * issue 0008559
 * Moved from osif/port.
 *
 * Revision 1.7  2012/06/26 11:19:54  mn
 * Changed size of stack.
 *
 * Revision 1.6  2011/10/31 09:30:27  sn
 * Remove OSIF critical task sections, no longer used.
 *
 * Revision 1.5  2011/09/30 09:03:43  sn
 * include memory allocate/free in external OSIF interface.
 *
 * Revision 1.4  2011/02/09 15:20:40  sn
 * change type of PSW from integer to unsigned long
 *
 * Revision 1.3  2010/12/22 15:24:09  sn
 * added osFatalError
 *
 * Revision 1.2  2010/12/01 13:01:01  sn
 * addes init of critical sections.
 *
 * Revision 1.1  2010/11/23 13:54:37  mn
 * Initial revision.
 *
 **********************************************************************!HE*/

/****************************************************************************
*
* to customer:
*
* Only this define may be changed by the customer
*
****************************************************************************/

/* Linux */
#define LNX_KTHREAD 1

/****************************************************************************
*
* ATTENTION to customer:
*
* DO NOT change this file
*
****************************************************************************/

/****************************************************************************/
/* Interface definition                                                     */
/****************************************************************************/

#if !defined(ST_STACK_SIZE)
#define ST_STACK_SIZE    (0x600)
#endif

/* OSIF is now ready, OsifActive must be called */
typedef void (* TOsifReady)                (void);
/* Init critical section */
typedef void *(* TInitCriticalSection)     (void);
/* Enter critical section */
typedef unsigned long (* TEnterCriticalSection)      (void *pParam);
/* Exit critical section */
typedef void (* TExitCriticalSection)      (void *pParam, unsigned long psw);
/* Interrupt enter */
typedef void (* TInterruptEnter)           (void);
/* Interrupt enter */
typedef void (* TInterruptExit)            (void);
/* memory allocate */
typedef void *(* TMemoryAllocate)         (int size);
/* memory free */
typedef void (* TMemoryFree)               (void * pBlock);

/* Fatal Error */
typedef void (* TFatalError)               (char * pFile, int iLine);

/* called after TOsifReady */
typedef void (* TOsifActive)            (void);

/** Get system time in milliseconds */
typedef unsigned long (* TGetTime)(void);

/** Get timer IDs */
typedef void (* TGetTimerID)(void **pHandle, unsigned char *pQueueID,
        unsigned char *pTimerID, unsigned short *pTimerChannel);

/** Start software timer */
typedef void (* TStartTimer)(void **pHandle, unsigned char QueueID, unsigned char TimerID,
        unsigned short TimerChannel, unsigned long IntervalInMS, void (*pTimerCallback)(void *));

/** Restart software timer */
typedef void (* TRestartTimer)(void **pHandle, unsigned char QueueID, unsigned char TimerID,
        unsigned short TimerChannel, unsigned long IntervalInMS, void (*pTimerCallback)(void *));

/** Stop software timer */
typedef void (* TStopTimer)(void **pHandle);

/** Delete software timer */
typedef void (* TDeleteTimer)(void **pHandle);

typedef struct _TStOSIF
{
    TOsifReady                  OsifReady;
    TInitCriticalSection        InitCriticalSection;
    TEnterCriticalSection       EnterCriticalSection;
    TExitCriticalSection        ExitCriticalSection;
    TInterruptEnter             InterruptEnter;
    TInterruptExit              InterruptExit;
    TMemoryAllocate             MemoryAllocate;
    TMemoryFree                 MemoryFree;
    TFatalError                 FatalError;
    TOsifActive                 OsifActive;
    TGetTime                    GetTime;
    TGetTimerID                 GetTimerID;
    TStartTimer                 StartTimer;
    TRestartTimer               RestartTimer;
    TStopTimer                  StopTimer;
    TDeleteTimer                DeleteTimer;
} TStOSIF;
typedef TStOSIF *PStOSIF;


/****************************************************************************/
/* Prototypes                                                               */
/****************************************************************************/

int stOsifInit(PStOSIF pStOSIF);

/* embOS */
void stInitEmbOS(void);
void stExitEmbOS(void);

/* FreeRTOS */
void stInitFreeRTOS(void);
unsigned long osGetTime(void);
void osGetTimerID(void **pHandle, unsigned char *pQueueID,
                unsigned short *pTimerID, unsigned short *pTimerChannel);
void osStartTimer(void **pHandle, unsigned char QueueID, unsigned char TimerID,
                unsigned short TimerChannel, unsigned long IntervalInMS, void (*pTimerCallback)(void *));
void osRestartTimer(void **pHandle, unsigned char QueueID, unsigned char TimerID,
                unsigned short TimerChannel, unsigned long IntervalInMS, void (*pTimerCallback)(void *));
void osStopTimer(void **pHandle);
void osDeleteTimer(void **pHandle);

/* Linux */
int stLnxInit(void);
int stLnxStop(void);


