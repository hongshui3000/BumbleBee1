#include "pingpong_buffer.h"
#include "rtl_assert.h"
#include <string.h>

#define MAX_BUFFER_SIZE 2048

static uint8_t Buffer1[MAX_BUFFER_SIZE];
static uint8_t Buffer2[MAX_BUFFER_SIZE];
static uint8_t *pInputBuf;         //e.g. store the log info.
static uint8_t *pOutputBuf;        //e.g. print through GDMA
uint16_t inputBuf_size;
uint16_t outputBuf_size;

/* --------------------------- static functions --------------------------- */
static void PPB_BufSwitch(void)
{
    uint8_t *pTemp;

    RTL_ASSERT(outputBuf_size == 0);

    //switch buffer
    pTemp = pInputBuf;
    pInputBuf = pOutputBuf;
    pOutputBuf = pTemp;

    //update size
    outputBuf_size = inputBuf_size;
    inputBuf_size = 0;

    //Start GDMA
}

static bool PPB_IsOutputBufFree(void)
{
    return (outputBuf_size == 0);
}

/* --------------------------- external functions --------------------------- */
void PPB_Init(void)
{
    pInputBuf = Buffer1;
    pOutputBuf = Buffer2;
    inputBuf_size = 0;
    outputBuf_size = 0;
}

void PPB_Write(const uint8_t *source, uint16_t size)
{
    portENTER_CRITICAL();

    if( size > (MAX_BUFFER_SIZE - inputBuf_size) )  //Input Buffer is full
    {
        if(PPB_IsOutputBufFree())
        {
            //Output Buffer is free
            PPB_BufSwitch();
        }
        else
        {
            //error handling
            TODO;
        }
    }

    memcpy(pInputBuf+inputBuf_size, source, size);
    inputBuf_size += size;

    portEXIT_CRITICAL();
}

void PPB_ClearOutputBuf(void)
{
    portENTER_CRITICAL();
    outputBuf_size = 0;
    portEXIT_CRITICAL();
}

