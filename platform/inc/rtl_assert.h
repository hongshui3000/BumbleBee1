/**
************************************************************************************************************
*               Copyright(c) 2014-2015, Realtek Semiconductor Corporation. All rights reserved.
************************************************************************************************************
* @file     rtl_assert.h
* @brief    Assert definition
* @author   lory_xu
* @date     2015-03
* @version  v0.1
*************************************************************************************************************
*/

#ifndef __RTL_ASSERT_H__
#define __RTL_ASSERT_H__

#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"

/** @addtogroup Platform
  * @{
  */

/** @addtogroup Platform_Basic Basic Definition
  * @{
  */

/** @addtogroup Platform_Basic_Assert Assert
  * @details Assert definition
  * @{
  */

#if defined(RELEASE_VERSION)
#define RTL_ASSERT(x)
#else
#define RTL_ASSERT(x) configASSERT(x)
#endif

/** End of Platform_Basic_Assert
  * @}
  */

/** End of Platform_Basic
  * @}
  */

/** End of Platform
  * @}
  */

#endif /* __RTL_ASSERT_H__ */
