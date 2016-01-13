/**
 **************************************************************************************
 *     Copyright(c) 2015, Realtek Semiconductor Corporation. All rights reserved.
 **************************************************************************************
 * @file     profile_common.h
 * @brief    Head file for legacy and LE profile common function
 * @details
 * @author   kyle_xu
 * @date     2015-11-19
 * @version  v0.1
 * *************************************************************************************
 */

#ifndef __PROFILE_COMMON_H__
#define __PROFILE_COMMON_H__

#ifdef  __cplusplus
extern "C" {
#endif
#include <rtl_types.h>
#include <bterrcod.h>


/** @defgroup Callback_Events Callback Events
  * @brief Callback events, used to notify application from profile layer.
  * @{
  */
#define PROFILE_EVT_SRV_REG_COMPLETE            0x01
#define PROFILE_EVT_SEND_DATA_COMPLETE          0x02
#define PROFILE_EVT_SDP_REG_COMPLETE            0x03
/** @} */

/** @defgroup General_Service_ID General Service ID
  * @brief service ID for general profile events.
  * @{
  */
#define ProfileAPI_ServiceUndefined             0xff
/** @} */

///@cond
/** @brief Max parameters of profile event to inform application. */
#define PROFILE_MAX_INFO_PARAM                  0x01
///@endcond

/** @defgroup TEventInfoCBs_t TEventInfoCBs_t
  * @brief data for profile to inform application.
  * @{
  */
typedef struct _TEventInfoCBs_t
{
    uint8_t     eventId;                    /**<  @brief EventId defined upper */
    uint16_t    sParas[PROFILE_MAX_INFO_PARAM]; /**<  @brief automatically parsed parameters */
} TEventInfoCBs_t;
/** @} End of TEventInfoCBs_t */

typedef TAppResult (* pfnAPPHandleInfoCB_t)(uint8_t serviceId, void *pPara);

#ifdef  __cplusplus
}
#endif  /*  __cplusplus */
#endif  /* __PROFILE_COMMON_H__*/
