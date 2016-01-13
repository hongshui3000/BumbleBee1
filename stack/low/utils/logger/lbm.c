/***************************************************************************
 Copyright (C) MindTree Ltd.
 This module is a confidential and proprietary property of MindTree and
 a possession or use of this module requires written permission of MindTree.
 ***************************************************************************/

/**
 * \file lbm.c
 *  Implmentation of Logger Buffer Manager (LBM) interface. Logger buffer is
 *  nothing but a circular queue implementation with the queue_get operation
 *  customized to handle wrap arounds.
 *
 * \author Santhosh kumar M
 * \date 2008-01-03
 */

/* ========================= Include File Section ========================= */
#include "lbm.h"
#include "mem.h"
#include "bz_debug.h"
#include "bt_fw_os.h"

#ifdef ENABLE_LOGGER

/* ====================== Macro Declaration Section ======================= */
/* ==================== Structure Declaration Section ===================== */
/* ===================== Variable Declaration Section ===================== */
/**
 * LBM Queue to store the logger data.
 */
//BZ_STATIC 
lbm_queue_t lbm_queue;


/* ================== Static Function Prototypes Section ================== */


/* ===================== Function Definition Section ====================== */
void lbm_init(void)
{
    lbm_queue.rd_ptr = 0;
    lbm_queue.wr_ptr = 0;
    lbm_queue.length = 0;
}

#ifndef _DONT_USE_LOG_UART_TX_INT_
BOOLEAN lbm_get(UCHAR** p_buf, UINT16* p_len)
{
    DEF_CRITICAL_SECTION_STORAGE;

    MINT_OS_ENTER_CRITICAL();
    /* Check if we have any data to return */
    if (lbm_queue.length == 0)
    {
        MINT_OS_EXIT_CRITICAL();
        return FALSE;   /* We have no data to return */
    }

    /* Determine the number of bytes to return. We want to choose a length
     * that doesn't make the buffer to wrap around. It helps us to avoid
     * returning two pointers.
     */
    *p_buf = &lbm_queue.buf[lbm_queue.rd_ptr];
    *p_len = lbm_queue.length;
    if (lbm_queue.length > (LBM_BUFFER_SIZE-lbm_queue.rd_ptr))
    {
        /* The data in the buffer seems to be wrapped around and hence lets
         * take the data till the end of the buffer.
         */
        *p_len = (LBM_BUFFER_SIZE-lbm_queue.rd_ptr);
    }

    /* Check whether the chosen length is bound by the user preferred value
     * i.e. LBM_MAX_GET_SIZE
     */
    if (*p_len > LBM_MAX_GET_SIZE)
    {
        *p_len = LBM_MAX_GET_SIZE;
    }

    /* Update the queue */
    /* consume *p_len bytes from the queue */
    lbm_queue.rd_ptr = (lbm_queue.rd_ptr + *p_len) & (LBM_BUFFER_SIZE-1);
    lbm_queue.length = lbm_queue.length - *p_len;
    MINT_OS_EXIT_CRITICAL();

    return TRUE;
}
#endif

BOOLEAN lbm_put(const UCHAR* buf, UINT16 len)
{
    UINT16 available_len;
    UINT16 wr_ptr;
    UINT16 i;
    
    DEF_CRITICAL_SECTION_STORAGE;

    MINT_OS_ENTER_CRITICAL();
    /* Check if we have sufficient space to store the data */
    available_len = LBM_BUFFER_SIZE - lbm_queue.length;
    if (available_len < len)
    {
        MINT_OS_EXIT_CRITICAL();
        return FALSE;   /* LBM can't hold this data */
    }

    wr_ptr = lbm_queue.wr_ptr;

    /* Consume the buffer now and differ the copying until we exit the
     * critical section.
     */
    lbm_queue.wr_ptr = (wr_ptr+len) & (LBM_BUFFER_SIZE-1);
    lbm_queue.length = lbm_queue.length + len;
    MINT_OS_EXIT_CRITICAL();

    /* Copy the data into circular buffer */
    for (i = 0; i < len; i++)
    {
        lbm_queue.buf[wr_ptr] = buf[i];
        wr_ptr = (wr_ptr+1) & (LBM_BUFFER_SIZE-1);
    }

    return TRUE;
}

#endif //#ifdef ENABLE_LOGGER
