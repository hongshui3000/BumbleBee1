/***************************************************************************
 Copyright (C) MindTree Ltd.
 This module is a confidential and proprietary property of MindTree and
 a possession or use of this module requires written permission of MindTree.
 ***************************************************************************/

/**
 * \file lbm.h
 *  Logger Buffer Manager(LBM) Interface. The LBM manages the logger data in a
 *  circular buffer.
 *
 * \author Santhosh kumar M
 * \date 2008-01-03
 */

#ifndef _LBM_H_
#define _LBM_H_

/* ========================= Include File Section ========================= */
// delete by guochunxia 
//#ifndef LBM_TEST_FRAMEWORK
#include "platform.h"

//{{{added by andy_liu 20090318
#ifdef ENABLE_LOGGER
//}}}added by andy_liu 20090318

#define LBM_BUFFER_SIZE       2048 //(1<<15) /* Must be a power of 2 */
/**
* Maximum number of bytes that can be returned by #lbm_get function.
*/
#define LBM_MAX_GET_SIZE           63

/* ==================== Structure Declaration Section ===================== */
/**
* LBM Internal Queue.
*/
typedef struct
{
	UCHAR buf[LBM_BUFFER_SIZE]; /**< Buffer to hold the data */
	UINT16 rd_ptr;              /**< Read pointer */
	UINT16 wr_ptr;              /**< Write pointer */
	UINT16 length;              /**< Number of bytes in the log buffer */
} lbm_queue_t;


extern lbm_queue_t lbm_queue;


/* ============================= API Section ============================== */
/**
* Initialize Logger Buffer Manager.
*
* \param None.
*
* \return None.
*/
void lbm_init(void);

#ifndef _DONT_USE_LOG_UART_TX_INT_
/**
* Returns \a *p_len bytes of data from the logger buffer. It also returns the
* pointer to the buffer (\a *p_buf) from which \a *p_len data bytes can be
* read. Number of bytes to return is bound by #LBM_MAX_GET_SIZE.
*
* \param[out] p_buf Pointer to store the returned buffer.
* \param[out] p_len Number of bytes returned.
*
* \return TRUE, if ther operation was successful. FALSE, otherwise.
*/
BOOLEAN lbm_get(UCHAR** p_buf, UINT16* p_len);
#endif

/**
* Stores \a len bytes of data from \a buf in the logger buffer.
*
* \param[in] buf Input buffer.
* \param[in] len Number of bytes to store in the logger buffer.
*
* \return TRUE, if the operation was successful. FALSE, otherwise.
*/
BOOLEAN lbm_put(const UCHAR* buf, UINT16 len);

//{{{added by andy_liu 20090318
#endif// ENABLE_LOGGER
//}}}added by andy_liu 20090318


#endif /* _LBM_H_ */

