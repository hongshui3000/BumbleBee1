/**
*****************************************************************************************
*     Copyright(c) 2015, Realtek Semiconductor Corporation. All rights reserved.
*****************************************************************************************
  * @file     IPSS.h
  * @brief    Head file for using internet protocol support service.
  * @details  Demonstration of different kinds of service interfaces.
  * @author   Vera
  * @date     2015-10-30
  * @version  v0.1
  * *************************************************************************************
  */

/* Define to prevent recursive inclusion */
#ifndef _IPSS_H_
#define _IPSS_H_

#ifdef __cplusplus
extern "C"  {
#endif      /* __cplusplus */

/* Add Includes here */
#include "profile_api.h"


/** @addtogroup IPSS IPSS
  * @brief Internet Protocol Support Service
  * @{
  */
 

/** @defgroup IPSS_Service_Exported_Constants IPSS Service Exported Constants
  * @brief macros that other .c files may use all defined here
  * @{
  */
  


///@cond
/** @brief  UUID of Internet Protocol Support Service. */
#define GATT_UUID_IPSS              0x1820
///@endcond





/** @} End of IPSS_Service_Exported_Constants */



/** @defgroup IPSS_Service_Exported_Functions IPSS Service Exported Functions
  * @brief functions that other .c files may use all defined here
  * @{
  */
extern uint8_t IPSS_AddService(void* pFunc);

/** @} End of IPSS_Service_Exported_Functions */



/** @} End of IPSS */

#ifdef __cplusplus
}
#endif

#endif /* _IPSS_H_ */
