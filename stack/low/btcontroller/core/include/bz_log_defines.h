/***************************************************************************
 Copyright (C) MindTree Ltd.
 This module is a confidential and proprietary property of MindTree and
 a possession or use of this module requires written permission of MindTree.
 ***************************************************************************/
#ifndef _BZ_LOG_DEFINES_H_
#define _BZ_LOG_DEFINES_H_

#include "logger.h"
#define LOGGER_CONN_HANDLE 0xFF



/*Definition of various module level functions. These definitions call the Log function with the appropriate format*/

//------------------------------------------------------------------------------------------------
//HCI_LOG_ERROR(LOG_LEVEL_LOW, str_index, para_num, para_list);
//-->
//LOG_ERROR(LOG_HCI_MODULE, LOG_LEVEL_LOW, str_index, para_num, para_list);
//-->
//LOG(LOG_TYPE_ERROR, LOG_LEVEL_LOW, LOG_RX_DIR, LOG_HCI_MODULE, str_index, para_num, "A", para_list)
//LOG(LOG_TYPE_ERROR, UINT8 log_level, UINT8 log_dir, 
//	UINT8 log_module, UINT16 log_str_index,
//	UINT8 para_num, const char *format, ...);
//#define LOG_ERROR(log_module, log_level, log_str_index, para_num, ...)
//	LOG(LOG_TYPE_ERROR, log_level, LOG_RX_DIR, log_module, log_str_index, para_num, "A", __VA_ARGS__)
//--------------------------------------------------------------------------------------------------
#define HCI_LOG_ERROR(...) \
		LOG_ERROR(LOG_HCI_MODULE, __VA_ARGS__)
#define HCI_LOG_WARNING(...) \
		LOG_WARNING(LOG_HCI_MODULE,__VA_ARGS__)
#define HCI_LOG_INFO(...) \
		LOG_INFO(LOG_HCI_MODULE,__VA_ARGS__)
#define HCI_LOG_TRACE(...) \
		LOG_TRACE(LOG_HCI_MODULE,__VA_ARGS__)

//color----------------------------------------------------------
#define COLOR_HCI_LOG_ERROR(color, ...) \
	COLOR_LOG_ERROR(color, LOG_HCI_MODULE, __VA_ARGS__)
#define COLOR_HCI_LOG_WARNING(color, ...) \
	COLOR_LOG_WARNING(color, LOG_HCI_MODULE,__VA_ARGS__)
#define COLOR_HCI_LOG_INFO(color, ...) \
	COLOR_LOG_INFO(color, LOG_HCI_MODULE,__VA_ARGS__)
#define COLOR_HCI_LOG_TRACE(color, ...) \
	COLOR_LOG_TRACE(color, LOG_HCI_MODULE,__VA_ARGS__)


#define HCI_LOG_DATA(log_level, log_str_index, log_dir, Length, Ptr) \
		    LOG_DATA(LOG_HCI_MODULE, log_level, Length, log_dir, log_str_index, Ptr)
#define HCI_LOG_PDU(log_level, log_str_index, log_dir, Length, Ptr) \
		LOG_PDU(LOG_HCI_MODULE, log_level, Length, log_dir, log_str_index, Ptr)
#define HCI_LOG_COMMAND(log_level, log_str_index, log_dir, Length, Ptr) \
		LOG_COMMAND(LOG_HCI_MODULE, log_level, Length, log_dir, log_str_index, Ptr)
#define HCI_LOG_EVENT(log_level, log_str_index, log_dir, Length, Ptr) \
		LOG_EVENT(LOG_HCI_MODULE, log_level, Length, log_dir, log_str_index, Ptr)
//color----------------------------------------------------------
#define COLOR_HCI_LOG_DATA(color, log_level, log_str_index, log_dir, Length, Ptr) \
	COLOR_LOG_DATA(color, LOG_HCI_MODULE, log_level, Length, log_dir, log_str_index, Ptr)
#define COLOR_HCI_LOG_PDU(color, log_level, log_str_index, log_dir, Length, Ptr) \
	COLOR_LOG_PDU(color, LOG_HCI_MODULE, log_level, Length, log_dir, log_str_index, Ptr)
#define COLOR_HCI_LOG_COMMAND(color, log_level, log_str_index, log_dir, Length, Ptr) \
	COLOR_LOG_COMMAND(color, LOG_HCI_MODULE, log_level, Length, log_dir, log_str_index, Ptr)
#define COLOR_HCI_LOG_EVENT(color, log_level, log_str_index, log_dir, Length, Ptr) \
	COLOR_LOG_EVENT(color, LOG_HCI_MODULE, log_level, Length, log_dir, log_str_index, Ptr)

//---------------------------------------------------------------------------------------------------
#define LMP_LOG_ERROR(...) \
		LOG_ERROR(LOG_LMP_MODULE,__VA_ARGS__)
#define LMP_LOG_WARNING(...) \
		LOG_WARNING(LOG_LMP_MODULE,__VA_ARGS__)
#define LMP_LOG_INFO(...) \
		LOG_INFO(LOG_LMP_MODULE,__VA_ARGS__)
#define LMP_LOG_TRACE(...) \
		LOG_TRACE(LOG_LMP_MODULE,__VA_ARGS__)

#define COLOR_LMP_LOG_ERROR(color, ...) \
		COLOR_LOG_ERROR(color, LOG_LMP_MODULE,__VA_ARGS__)
#define COLOR_LMP_LOG_WARNING(color, ...) \
		COLOR_LOG_WARNING(color, LOG_LMP_MODULE,__VA_ARGS__)
#define COLOR_LMP_LOG_INFO(color, ...) \
		COLOR_LOG_INFO(color, LOG_LMP_MODULE,__VA_ARGS__)
#define COLOR_LMP_LOG_TRACE(color, ...) \
		COLOR_LOG_TRACE(color, LOG_LMP_MODULE,__VA_ARGS__)

#define LMP_LOG_DATA(log_level, log_str_index, log_dir, Length, Ptr) \
		LOG_DATA(LOG_LMP_MODULE, log_level, Length, log_dir, log_str_index, Ptr)
#define LMP_LOG_PDU(log_level, log_str_index, log_dir, Length, Ptr) \
		LOG_PDU(LOG_LMP_MODULE, log_level, Length, log_dir, log_str_index, Ptr)
#define LMP_LOG_COMMAND(log_level, log_str_index, log_dir, Length, Ptr) \
		LOG_COMMAND(LOG_LMP_MODULE, log_level, Length, log_dir, log_str_index, Ptr)
#define LMP_LOG_EVENT(log_level, log_str_index, log_dir, Length, Ptr) \
		LOG_EVENT(LOG_LMP_MODULE, log_level, Length, log_dir, log_str_index, Ptr)

#define COLOR_LMP_LOG_DATA(color, log_level, log_str_index, log_dir, Length, Ptr) \
		COLOR_LOG_DATA(color, LOG_LMP_MODULE, log_level, Length, log_dir, log_str_index, Ptr)
#define COLOR_LMP_LOG_PDU(color, log_level, log_str_index, log_dir, Length, Ptr) \
		COLOR_LOG_PDU(color, LOG_LMP_MODULE, log_level, Length, log_dir, log_str_index, Ptr)
#define COLOR_LMP_LOG_COMMAND(color, log_level, log_str_index, log_dir, Length, Ptr) \
		COLOR_LOG_COMMAND(color, LOG_LMP_MODULE, log_level, Length, log_dir, log_str_index, Ptr)
#define COLOR_LMP_LOG_EVENT(color, log_level, log_str_index, log_dir, Length, Ptr) \
		COLOR_LOG_EVENT(color, LOG_LMP_MODULE, log_level, Length, log_dir, log_str_index, Ptr)

//---------------------------------------------------------------------------------------------------


#define LC_LOG_ERROR(...) \
		LOG_ERROR(LOG_LC_MODULE,__VA_ARGS__)
#define LC_LOG_WARNING(...) \
		LOG_WARNING(LOG_LC_MODULE,__VA_ARGS__)
#define LC_LOG_INFO(...) \
		LOG_INFO(LOG_LC_MODULE,__VA_ARGS__)
#define LC_LOG_TRACE(...) \
		LOG_TRACE(LOG_LC_MODULE,__VA_ARGS__)

#define COLOR_LC_LOG_ERROR(color, ...) \
		COLOR_LOG_ERROR(color, LOG_LC_MODULE,__VA_ARGS__)
#define COLOR_LC_LOG_WARNING(color, ...) \
		COLOR_LOG_WARNING(color, LOG_LC_MODULE,__VA_ARGS__)
#define COLOR_LC_LOG_INFO(color, ...) \
		COLOR_LOG_INFO(color, LOG_LC_MODULE,__VA_ARGS__)
#define COLOR_LC_LOG_TRACE(color, ...) \
		COLOR_LOG_TRACE(color, LOG_LC_MODULE,__VA_ARGS__)


#define LC_LOG_DATA(log_level, log_str_index, log_dir, Length, Ptr) \
		LOG_DATA(LOG_LC_MODULE, log_level, Length, log_dir, log_str_index, Ptr)
#define LC_LOG_PDU(log_level, log_str_index, log_dir, Length, Ptr) \
		LOG_PDU(LOG_LC_MODULE, log_level, Length, log_dir, log_str_index, Ptr)
#define LC_LOG_COMMAND(log_level, log_str_index, log_dir, Length, Ptr) \
		LOG_COMMAND(LOG_LC_MODULE, log_level, Length, log_dir, log_str_index, Ptr)
#define LC_LOG_EVENT(log_level, log_str_index, log_dir, Length, Ptr) \
		LOG_EVENT(LOG_LC_MODULE, log_level, Length, log_dir, log_str_index, Ptr)

#define COLOR_LC_LOG_DATA(color, log_level, log_str_index, log_dir, Length, Ptr) \
		COLOR_LOG_DATA(color, LOG_LC_MODULE, log_level, Length, log_dir, log_str_index, Ptr)
#define COLOR_LC_LOG_PDU(color, log_level, log_str_index, log_dir, Length, Ptr) \
		COLOR_LOG_PDU(color, LOG_LC_MODULE, log_level, Length, log_dir, log_str_index, Ptr)
#define COLOR_LC_LOG_COMMAND(color, log_level, log_str_index, log_dir, Length, Ptr) \
		COLOR_LOG_COMMAND(color, LOG_LC_MODULE, log_level, Length, log_dir, log_str_index, Ptr)
#define COLOR_LC_LOG_EVENT(color, log_level, log_str_index, log_dir, Length, Ptr) \
		COLOR_LOG_EVENT(color, LOG_LC_MODULE, log_level, Length, log_dir, log_str_index, Ptr)


#define RADIO_LOG_INFO(...) \
		LOG_INFO(LOG_LC_MODULE,__VA_ARGS__)
#define COLOR_RADIO_LOG_INFO(color, ...) \
		COLOR_LOG_INFO(color, LOG_LC_MODULE,__VA_ARGS__)

#define ESCO_LOG_INFO(...) \
		//LOG_INFO(LOG_LC_MODULE,__VA_ARGS__)

#define ESCO_LOG_ERROR(...) \
		//LOG_INFO(LOG_LC_MODULE,__VA_ARGS__)


#define SRI_PRINT(msg, ...)     HCI_LOG_INFO(LOG_LEVEL_LOW, msg, __VA_ARGS__)
#define SRE_PRINT(msg, ...)     HCI_LOG_ERROR(LOG_LEVEL_HIGH, msg, __VA_ARGS__)
#define COLOR_SRI_PRINT(color, msg, ...)     COLOR_HCI_LOG_INFO(color, LOG_LEVEL_LOW, msg, __VA_ARGS__)
#define COLOR_SRE_PRINT(color, msg, ...)     COLOR_HCI_LOG_ERROR(color, LOG_LEVEL_HIGH, msg, __VA_ARGS__)


////////////////////////////////////////current null log macro functions/////////////////////
#ifdef TD_DEBUG					/* Transport Driver Debugs */
#define TD_ERR(...)  			LOG_ERROR(LOG_TRANSPORT_MODULE, LOG_LEVEL_HIGH, __VA_ARGS__)
#define TD_INF(...)				LOG_INFO(LOG_TRANSPORT_MODULE, LOG_LEVEL_LOW, __VA_ARGS__)
#define COLOR_TD_ERR(color, ...)  	COLOR_LOG_ERROR(color, LOG_TRANSPORT_MODULE, LOG_LEVEL_HIGH, __VA_ARGS__)
#define COLOR_TD_INF(color, ...)	COLOR_LOG_INFO(color, LOG_TRANSPORT_MODULE, LOG_LEVEL_LOW, __VA_ARGS__)
#else
#define TD_ERR  null_macro
#define TD_INF	null_macro
#define COLOR_TD_ERR  null_macro
#define COLOR_TD_INF  null_macro
#endif

#ifdef BT_FW_HCI_DEBUG			/*HCI Debugs */
#define BT_FW_HCI_ERR(...)  	LOG_ERROR(LOG_HCI_MODULE, LOG_LEVEL_HIGH, __VA_ARGS__)
#define BT_FW_HCI_INF(...) 		LOG_INFO(LOG_HCI_MODULE, LOG_LEVEL_LOW, __VA_ARGS__)
#define COLOR_BT_FW_HCI_ERR(color, ...)  	COLOR_LOG_ERROR(color, LOG_HCI_MODULE, LOG_LEVEL_HIGH, __VA_ARGS__)
#define COLOR_BT_FW_HCI_INF(color, ...) 	COLOR_LOG_INFO(color, LOG_HCI_MODULE, LOG_LEVEL_LOW, __VA_ARGS__)
#else
#define BT_FW_HCI_ERR	null_macro	
#define BT_FW_HCI_INF	null_macro
#define COLOR_BT_FW_HCI_ERR	null_macro	
#define COLOR_BT_FW_HCI_INF	null_macro
#endif

#ifdef LMP_DEBUG				/* LMP Debugs */
#define LMP_ERR(...)			LOG_ERROR(LOG_LMP_MODULE, LOG_LEVEL_HIGH, __VA_ARGS__)
#define LMP_INF(...)			LOG_INFO(LOG_LMP_MODULE, LOG_LEVEL_LOW, __VA_ARGS__)
#define COLOR_LMP_ERR(color, ...)	COLOR_LOG_ERROR(color, LOG_LMP_MODULE, LOG_LEVEL_HIGH, __VA_ARGS__)
#define COLOR_LMP_INF(color, ...)	COLOR_LOG_INFO(color, LOG_LMP_MODULE, LOG_LEVEL_LOW, __VA_ARGS__)
#else
#define LMP_ERR		null_macro
#define LMP_INF		null_macro
#define COLOR_LMP_ERR	null_macro
#define COLOR_LMP_INF	null_macro
#endif

#ifdef LC_DEBUG					/* Link Controller */
#define LC_ERR(...)				LOG_ERROR(LOG_LC_MODULE, LOG_LEVEL_HIGH, __VA_ARGS__)
#define COLOR_LC_ERR(color, ...)	COLOR_LOG_ERROR(color, LOG_LC_MODULE, LOG_LEVEL_HIGH, __VA_ARGS__)
#else
#define LC_ERR		null_macro
#define COLOR_LC_ERR	null_macro
#endif

#ifdef SCO_HCI_DEBUG
#define SCO_HCI_INF(...)		LOG_INFO(LOG_LMP_MODULE, LOG_LEVEL_LOW, __VA_ARGS__)
#define COLOR_SCO_HCI_INF(color, ...)	COLOR_LOG_INFO(color, LOG_LMP_MODULE, LOG_LEVEL_LOW, __VA_ARGS__)
#else  
#define SCO_HCI_INF(...)
#define COLOR_SCO_HCI_INF(color, ...)
#endif 

#ifdef HI_DRIVER_DEBUG			/* Host interface driver Debugs */
#define HI_DRIVER_TRC2(...) 	LOG_TRACE(LOG_HI_MODULE, LOG_LEVEL_LOW, __VA_ARGS__)
#define COLOR_HI_DRIVER_TRC2(color, ...) 	COLOR_LOG_TRACE(color, LOG_HI_MODULE, LOG_LEVEL_LOW, __VA_ARGS__)
#else
#define HI_DRIVER_TRC2  null_macro
#define COLOR_HI_DRIVER_TRC2  null_macro
#endif

#ifdef TIMER_DBG
#define TIMER_ERR(...)			LOG_ERROR(LOG_TIMER_MODULE, LOG_LEVEL_HIGH, __VA_ARGS__)
#define TIMER_INF(...)			LOG_INFO(LOG_TIMER_MODULE, LOG_LEVEL_LOW, __VA_ARGS__)
#define COLOR_TIMER_ERR(color, ...)		COLOR_LOG_ERROR(color, LOG_TIMER_MODULE, LOG_LEVEL_HIGH, __VA_ARGS__)
#define COLOR_TIMER_INF(color, ...)		COLOR_LOG_INFO(color, LOG_TIMER_MODULE, LOG_LEVEL_LOW, __VA_ARGS__)
#else
#define TIMER_ERR		null_macro
#define TIMER_INF		null_macro
#define COLOR_TIMER_ERR		null_macro
#define COLOR_TIMER_INF		null_macro
#endif


#ifdef OS_DEBUG					/* OS Debugs */
#define OS_ERR(...)				LOG_ERROR(LOG_OS_MODULE, LOG_LEVEL_HIGH, __VA_ARGS__)
#define OS_INF(...)				LOG_INFO(LOG_OS_MODULE, LOG_LEVEL_LOW, __VA_ARGS__)
#define COLOR_OS_ERR(color, ...)	COLOR_LOG_ERROR(color, LOG_OS_MODULE, LOG_LEVEL_HIGH, __VA_ARGS__)
#define COLOR_OS_INF(color, ...)	COLOR_LOG_INFO(color, LOG_OS_MODULE, LOG_LEVEL_LOW, __VA_ARGS__)
#else
#define OS_ERR		null_macro
#define OS_INF		null_macro
#define COLOR_OS_ERR	null_macro
#define COLOR_OS_INF	null_macro
#endif


#ifdef AFH_DEBUG
#define AFH_ERR(...)			LOG_ERROR(LOG_AFH_MODULE, LOG_LEVEL_HIGH, __VA_ARGS__)
#define AFH_INF(...)			LOG_INFO(LOG_AFH_MODULE, LOG_LEVEL_LOW, __VA_ARGS__)
#define AFH_TRC(...)			LOG_TRACE(LOG_AFH_MODULE, LOG_LEVEL_LOW, __VA_ARGS__)
#define COLOR_AFH_ERR(colro, ...)	COLOR_LOG_ERROR(color, LOG_AFH_MODULE, LOG_LEVEL_HIGH, __VA_ARGS__)
#define COLOR_AFH_INF(color, ...)	COLOR_LOG_INFO(color, LOG_AFH_MODULE, LOG_LEVEL_LOW, __VA_ARGS__)
#define COLOR_AFH_TRC(color, ...)	COLOR_LOG_TRACE(color, LOG_AFH_MODULE, LOG_LEVEL_LOW, __VA_ARGS__)
#else
#define AFH_ERR		null_macro
#define AFH_INF		null_macro
#define AFH_TRC		null_macro
#define COLOR_AFH_ERR		null_macro
#define COLOR_AFH_INF		null_macro
#define COLOR_AFH_TRC		null_macro
#endif


#ifdef CH_ASMNT_DBG
#define CH_ASMNT_INF(...)		LOG_INFO(LOG_CH_MODULE, LOG_LEVEL_LOW, __VA_ARGS__)
#define COLOR_CH_ASMNT_INF(color, ...)	COLOR_LOG_INFO(color, LOG_CH_MODULE, LOG_LEVEL_LOW, __VA_ARGS__)
#else
#define CH_ASMNT_INF    null_macro
#define COLOR_CH_ASMNT_INF    null_macro
#endif


#ifdef ESCO_DEBUG
#define ESCO_ERR(...)			LOG_ERROR(LOG_ESCO_MODULE, LOG_LEVEL_HIGH, __VA_ARGS__)
#define ESCO_TRC(...)			LOG_TRACE(LOG_ESCO_MODULE, LOG_LEVEL_LOW, __VA_ARGS__)
#define ESCO_INF(...)			LOG_INFO(LOG_ESCO_MODULE, LOG_LEVEL_LOW, __VA_ARGS__)
#define COLOR_ESCO_ERR(color, ...)	COLOR_LOG_ERROR(color, LOG_ESCO_MODULE, LOG_LEVEL_HIGH, __VA_ARGS__)
#define COLOR_ESCO_TRC(color, ...)	COLOR_LOG_TRACE(color, LOG_ESCO_MODULE, LOG_LEVEL_LOW, __VA_ARGS__)
#define COLOR_ESCO_INF(color, ...)	COLOR_LOG_INFO(color, LOG_ESCO_MODULE, LOG_LEVEL_LOW, __VA_ARGS__)
#else
#define ESCO_ERR		null_macro
#define ESCO_TRC		null_macro
#define ESCO_INF		null_macro
#define COLOR_ESCO_ERR		null_macro
#define COLOR_ESCO_TRC		null_macro
#define COLOR_ESCO_INF		null_macro
#endif


////////////////////////////////////////current non-null log macro functions/////////////////////
#ifndef ENABLE_LOGGER
#define VER_INF 	null_macro
#define VER_ERR		null_macro
#define COLOR_VER_INF 	null_macro
#define COLOR_VER_ERR		null_macro
#else
#define VER_INF(...)	 		LOG_INFO(LOG_UNSPECIFIED_MODULE, LOG_LEVEL_LOW, __VA_ARGS__)  
#define VER_ERR(...)			LOG_ERROR(LOG_UNSPECIFIED_MODULE, LOG_LEVEL_HIGH, __VA_ARGS__) 
#define COLOR_VER_INF(color, ...)	COLOR_LOG_INFO(color, LOG_UNSPECIFIED_MODULE, LOG_LEVEL_LOW, __VA_ARGS__)  
#define COLOR_VER_ERR(color, ...)	COLOR_LOG_ERROR(color, LOG_UNSPECIFIED_MODULE, LOG_LEVEL_HIGH, __VA_ARGS__)
#endif

#ifdef BT_FW_DEBUG
#define BT_FW_ERR(...) 			LOG_ERROR(LOG_BT_MODULE, LOG_LEVEL_HIGH, __VA_ARGS__)
#define COLOR_BT_FW_ERR(color, ...) 	COLOR_LOG_ERROR(color, LOG_BT_MODULE, LOG_LEVEL_HIGH, __VA_ARGS__)
#else
#define BT_FW_ERR  null_macro
#define COLOR_BT_FW_ERR  null_macro
#endif

#endif /* _BZ_LOG_DEFINES_H_ */

