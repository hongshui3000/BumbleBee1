#ifndef _APP_QUEUE_HE_
#define _APP_QUEUE_HE_

#include <rtl_types.h>


struct QueueElement                                /* dummy definition      */
{
    struct QueueElement *Next;                       /* point to next element */
    UINT8                 data[2];                    /* user data             */
};
typedef struct QueueElement ELEMENT_T;

typedef ELEMENT_T          *ELEMENT_P;

typedef struct
{
    ELEMENT_P First;                                 /* first element         */
    ELEMENT_P Last;                                  /* last element          */
    UINT16      ElementCount;                          /* element count         */
} QUEUE_T, *QUEUE_P;




#if defined (__cplusplus)
extern "C" {
#endif

void  AppQueueIn     (QUEUE_P QueuePtr, void *QueueElementPtr);
void *AppQueueOut    (QUEUE_P QueuePtr);
void  AppQueueInsert (QUEUE_P QueuePtr, void *QueueElementPtr, void *NewQueueElementPtr);
void  AppQueueDelete (QUEUE_P QueuePtr, void *QueueElementPtr);


#if defined (__cplusplus)
}
#endif

#endif 
