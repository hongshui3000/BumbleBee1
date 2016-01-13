/**
************************************************************************************************************
*               Copyright(c) 2014-2015, Realtek Semiconductor Corporation. All rights reserved.
************************************************************************************************************
* @file     rtl_delay.h
* @brief    Sleep functions
* @author   lory_xu
* @date     2015-03
* @version  v0.1
*************************************************************************************************************
*/

#ifndef __RTL_DELAY_H__
#define __RTL_DELAY_H__

#include "rtl_types.h"

/** @addtogroup Platform
  * @{
  */

/** @addtogroup Platform_Function Hardware Function
  * @{
  */

/** @addtogroup Platform_Function_Delay Delay
  * @details Platform delay functions
  * @{
  */

/** @brief Delay \a t MS */
/**
  * @brief  Delay \a t MS
  * @param  t: counter
  * @retval None
  */
void delayMS(uint32_t t);

/** @brief Delay \a t US */
/**
  * @brief  Delay \a t US
  * @param  t: counter
  * @retval None
  */
void delayUS(uint32_t t);
/** @endcond */

/** End of Platform_Function_Delay
  * @}
  */

/** End of Platform_Function
  * @}
  */

/** End of Platform
  * @}
  */

#endif /* __RTL_DELAY_H__ */
