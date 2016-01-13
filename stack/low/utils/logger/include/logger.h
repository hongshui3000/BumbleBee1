/***************************************************************************
 Copyright (C) MindTree Ltd.
 This module is a confidential and proprietary property of MindTree and
 a possession or use of this module requires written permission of MindTree.
 ***************************************************************************/
#ifndef _LOGGER_
#define _LOGGER_

#include <stdarg.h>

#include "platform.h"
#include "DataType.h"
#include <LOG1.h>




//=======================================================================================================================
//We tried to offer a kind of message header code, which will take necessary information to be used in host logger tools.
//They are 
//LOG_SYNC(0xA5A5)																//16bits
//LOG_TYPE(4bit),	LOG_LEVEL(3bit),	LOG_DIR(1bit),							//8	bits
//LOG_MODULE(4bit), LOG_TIMESTAMP(1bit), LOG_WITH_DATA(1bit), Reserved(2bit),	//8	bits
//LOG_STRING_INDEX(16bit)														//16bits
//perhaps Time_Stamp															//32bits
//perhaps Data_Length															//32bits
//perhaps Data																	//xxx Bytes
//=======================================================================================================================
//There maybe 2 kinds of log messages in our system:
//[1]MSG_HEADER(32bits)(+time_stamp) 
//[2]MSG_HEADER(32bits)(+time_stamp) + data_length(32bits) + data

//-----------------------------------
//LOG_COLOR(3bits)
//-----------------------------------
#define GREEN				1
#define RED					2
#define WHITE				3
#define BLUE				4
#define YELLOW				5
#define GRAY				6


//------------------------------------
//LOG_TYPE(12bit)
//------------------------------------
#define LOG_TYPE_MASK		0x01F       /* allowed log type mask */
#define	LOG_TYPE_ERROR		0x001
#define LOG_TYPE_WARNING	0x002	
#define LOG_TYPE_INFO		0x004
#define LOG_TYPE_PDU		0x008
#define LOG_TYPE_DATA		0x010
#define LOG_TYPE_TRACE		0x020
#define LOG_TYPE_ASSERT		0x040
#define LOG_TYPE_COMMAND	0x080
#define LOG_TYPE_EVENT		0x100
#define LOG_TYPE_SYNC		0x200

//------------------------------------
//LOG_LEVEL(5bit)
//------------------------------------
#define LOG_LEVEL_MASK      0x03        /* allowed log mask */
#define LOG_LEVEL_CRITICAL  0x01        /* critical level */
#define LOG_LEVEL_HIGH      0x02        /* high level */
#define LOG_LEVEL_MEDIUM    0x04        /* medium level */
#define LOG_LEVEL_LOW       0x08		/* low level */
#define LOG_LEVEL_TRACE     0x10        /* trace level */

//------------------------------------
//LOG_DIR(1bit)
//------------------------------------
#define LOG_DIR_MASK		0x01
#define LOG_RX_DIR			0x00
#define LOG_TX_DIR			0x01


//------------------------------------
//LOG_MODULE(4bit)
//------------------------------------
#define LOG_MASK_MODULE			0xF0
#define LOG_UNSPECIFIED_MODULE	0x00
#define LOG_HCI_MODULE			0x10
#define LOG_LMP_MODULE			0x20
#define LOG_LC_MODULE			0x30
#define LOG_TRANSPORT_MODULE	0x40
//#define //LOG_BASEBAND_MODULE,
//#define //LOG_TESTSYSTEM_MODULE,
//#define //LOG_TESTCASES_MODULE,
#define LOG_BT_MODULE			0x50
#define LOG_HI_MODULE			0x60
#define LOG_TIMER_MODULE		0x70	
#define LOG_OS_MODULE			0x80
//#define //LOG_FC_MODULE,
#define LOG_AFH_MODULE			0x90
//#define //LOG_LAM_MODULE,
#define LOG_CH_MODULE			0xA0
#define LOG_ESCO_MODULE			0xB0

//------------------------------------
//LOG_TIMESTAMP(1bit)
//------------------------------------
#define LOG_TIMESTAMP_MASK		0x08
#define LOG_TIMESTAMP_DISABLE	0x00
#define LOG_TIMESTAMP_ENABLE	0x08

//------------------------------------
//LOG_WITH_DATA(1bit)
//------------------------------------
#define LOG_WITHDATA_MASK		0x04
#define LOG_WITHDATA_NO			0x00
#define LOG_WITHDATA_YES		0x04

#define LOG_STATUS_ENABLE		0x01
#define LOG_STATUS_DISABLE		0x00

#define UART_FIFO_MAX_LENGTH        16

#define FOUR_ALIGNMENT          0x3
#define TWO_ALIGNMENT           0x1


/*Global Variables that control the Log Information*/
typedef struct LOG_LogControl 
{
//{{{modified by andy_liu 20090318
//	UINT32 TimeStampEnable:1;
//	UINT32 FileNameEnable:1;
//	UINT32 LineNumberEnable:1;
//	UINT32 EncryptionEnable:2;
//	UINT32 LogLevel:3;
//}}}modified by andy_liu 20090318
	UINT8 TimeStampEnable;
	UINT8 LogLevel;
}LOG_LogControl;

typedef struct UART_DBG_PKT
{
    union {
        UINT8  byte;
        struct {
            UINT8 cmd_type     :6; /* bit[5:0] */
            UINT8 is_16b       :1; /* bit[6] */
            UINT8 is_write     :1; /* bit[7] */
        };
    };
}UART_DBG_PKT_TYPE;

//UART RX Debug Function
enum UART_RX_DBG_CMD {
UART_SET_PTA = 10,                  // 0xA
UART_SET_TDMA_TBT_TIME,             // 0xB
UART_SET_TDMA_PERIOD_TIME,          // 0xC
UART_SET_HID_PROFILE,               // 0xD
UART_SET_PTA_DTMA_RETRY_INDEX,      // 0xE
UART_SET_IGNORE_WLAN_ACT,           // 0xF
UART_ENABLE_A2DP_PROFILE_CONTROL,   //0x10
UART_SET_TDMA_PKT_TYPE_INDEX,       //0x11
};

typedef UCHAR (*LOGGER_TRANS_IS_FREE_CB)(void);
typedef BOOLEAN (*LOGGER_TRANS_WRITE_CB)(UCHAR* buf, UINT16 len);
typedef void (*LOGGER_TRANS_COMPLETE_CB)(UCHAR *buf, UINT16 len, UINT8 type);

LOGGER_TRANS_COMPLETE_CB logger_init(LOGGER_TRANS_WRITE_CB log_write_cb,
                                     LOGGER_TRANS_IS_FREE_CB log_free_cb);
UCHAR LOG(UINT8 color, UINT16 file_num, UINT16 line_num,  
		  UINT16 log_str_index, UINT8 para_num, const char *format, ...);

UCHAR LOG_DATA_FUNCTION(UINT8 color, UINT16 file_num, UINT16 line_num,  
						UINT16 log_str_index, UINT16 Length, UCHAR* Ptr);

void  uart_dbg_receive_packet(UCHAR* buf, UINT16 len);
void uart_dbg_receive_packet_delayed(UINT16 rptr, UINT16 len);


UCHAR bz_logger_init(void);
//{{{delete by andy_liu 20090319
//UCHAR bz_logger_shutdown(void);
//}}}delete by andy_liu 20090319
UCHAR bz_logger_send_raw_data(void);
UCHAR bz_uart_dbg_send_raw_data(void);

#define pf_service_logger()     bz_logger_send_raw_data()
#define pf_service_uart_dbg()   bz_uart_dbg_send_raw_data();
/*
 * Macro defintions for easy use.
 */
#define null_macro(a,...)\
;

#ifdef ENABLE_LOGGER

#define LOG_ERROR(log_module, log_level, log_str_index, para_num, ...) \
    {if (((log_level) & LOG_LEVEL_MASK) && (LOG_TYPE_MASK & LOG_TYPE_ERROR)) { \
	LOG(0, __FILE_NUM__, __LINE__, log_str_index, para_num, "A", __VA_ARGS__); \
    }}
#define LOG_WARNING(log_module, log_level, log_str_index, para_num, ...) \
    {if (((log_level) & LOG_LEVEL_MASK) && (LOG_TYPE_MASK & LOG_TYPE_WARNING)) { \
    LOG(0, __FILE_NUM__, __LINE__, log_str_index, para_num, "A", __VA_ARGS__); \
    }}
#define LOG_INFO(log_module, log_level, log_str_index, para_num, ...) \
    {if (((log_level) & LOG_LEVEL_MASK) && (LOG_TYPE_MASK & LOG_TYPE_INFO)) { \
	LOG(0, __FILE_NUM__, __LINE__, log_str_index, para_num, "A", __VA_ARGS__);\
    }}	

#define LOG_TRACE(log_module, log_level, log_str_index, para_num, ...) \
    {if (((log_level) & LOG_LEVEL_MASK) && (LOG_TYPE_MASK & LOG_TYPE_TRACE)) { \
	LOG(0, __FILE_NUM__, __LINE__, log_str_index, para_num, "A", __VA_ARGS__);\
    }}

#define LOG_PDU(log_module, log_level, Length, log_dir, log_str_index, Ptr) \
    {if (((log_level) & LOG_LEVEL_MASK) && (LOG_TYPE_MASK & LOG_TYPE_PDU)) { \
	LOG_DATA_FUNCTION(0, __FILE_NUM__, __LINE__, log_str_index, Length, Ptr);\
    }}

#define LOG_DATA(log_module, log_level, Length, log_dir, log_str_index, Ptr) \
    {if (((log_level) & LOG_LEVEL_MASK) && (LOG_TYPE_MASK & LOG_TYPE_DATA)) { \
	LOG_DATA_FUNCTION(0, __FILE_NUM__, __LINE__, log_str_index, Length, Ptr);\
    }}

#define LOG_COMMAND(log_module, log_level, Length, log_dir, log_str_index, Ptr) \
    {if (((log_level) & LOG_LEVEL_MASK) && (LOG_TYPE_MASK & LOG_TYPE_COMMAND)) { \
	LOG_DATA_FUNCTION(0, __FILE_NUM__, __LINE__, log_str_index, Length, Ptr);\
    }}

#define LOG_EVENT(log_module, log_level, Length, log_dir, log_str_index, Ptr) \
    {if (((log_level) & LOG_LEVEL_MASK) && (LOG_TYPE_MASK & LOG_TYPE_EVENT)) { \
	LOG_DATA_FUNCTION(0, __FILE_NUM__, __LINE__, log_str_index, Length, Ptr);\
    }}

#define COLOR_LOG_ERROR(color, log_module, log_level, log_str_index, para_num, ...) \
    {if (((log_level) & LOG_LEVEL_MASK) && (LOG_TYPE_MASK & LOG_TYPE_ERROR)) { \
	LOG(color, __FILE_NUM__, __LINE__, log_str_index, para_num, "A", __VA_ARGS__);\
    }}

#define COLOR_LOG_WARNING(color, log_module, log_level, log_str_index, para_num, ...) \
    {if (((log_level) & LOG_LEVEL_MASK) && (LOG_TYPE_MASK & LOG_TYPE_WARNING)) { \
	LOG(color, __FILE_NUM__, __LINE__, log_str_index, para_num, "A", __VA_ARGS__);\
    }}

#define RT_BT_LOG(color, log_str_index, para_num, ...) \
	LOG(color, __FILE_NUM__, __LINE__, log_str_index, para_num, "A", __VA_ARGS__);
    
#define COLOR_LOG_INFO(color, log_module, log_level, log_str_index, para_num, ...) \
    {if (((log_level) & LOG_LEVEL_MASK) && (LOG_TYPE_MASK & LOG_TYPE_INFO)) { \
	LOG(color, __FILE_NUM__, __LINE__, log_str_index, para_num, "A", __VA_ARGS__);\
    }}

#define COLOR_LOG_TRACE(color, log_module, log_level, log_str_index, para_num, ...) \
    {if (((log_level) & LOG_LEVEL_MASK) && (LOG_TYPE_MASK & LOG_TYPE_TRACE)) { \
	LOG(color, __FILE_NUM__, __LINE__, log_str_index, para_num, "A", __VA_ARGS__);\
    }}

#define COLOR_LOG_PDU(color, log_module, log_level, Length, log_dir, log_str_index, Ptr) \
    if (((log_level) & LOG_LEVEL_MASK) && (LOG_TYPE_MASK & LOG_TYPE_PDU)) { \
	LOG_DATA_FUNCTION(color, __FILE_NUM__, __LINE__, log_str_index, Length, Ptr);\
    }

#define COLOR_LOG_DATA(color, log_module, log_level, Length, log_dir, log_str_index, Ptr) \
    {if (((log_level) & LOG_LEVEL_MASK) && (LOG_TYPE_MASK & LOG_TYPE_DATA)) { \
	LOG_DATA_FUNCTION(color, __FILE_NUM__, __LINE__, log_str_index, Length, Ptr);\
    }}

#define COLOR_LOG_COMMAND(color, log_module, log_level, Length, log_dir, log_str_index, Ptr) \
    {if (((log_level) & LOG_LEVEL_MASK) && (LOG_TYPE_MASK & LOG_TYPE_COMMAND)) { \
	LOG_DATA_FUNCTION(color, __FILE_NUM__, __LINE__, log_str_index, Length, Ptr);\
    }}

#define COLOR_LOG_EVENT(color, log_module, log_level, Length, log_dir, log_str_index, Ptr) \
    {if (((log_level) & LOG_LEVEL_MASK) && (LOG_TYPE_MASK & LOG_TYPE_EVENT)) { \
	LOG_DATA_FUNCTION(color, __FILE_NUM__, __LINE__, log_str_index, Length, Ptr);\
    }}
    
//{{{noted by liuyong 20091223============================================

//***********************************************************************  
//***This method provide a format_print method for quick debug, but   ***   
//***it is not an efficiency method and do not use in release version!***
//***********************************************************************

#ifdef RT_TMP_LOG_ENABLE
#define LOG_TYPE_TMP_STR	0xA0

UCHAR realtek_tmp_debug_log(UINT8 color, UINT16 file_num, UINT16 line_num, 
							const char *format, ...);

#ifdef FOR_SIMULATION
#define RT_TMP_LOG          //UARTPrintf
#define COLOR_RT_TMP_LOG    //COLOR_UARTPrintf
#else //FOR_SIMULATION

#define RT_TMP_LOG(...) \
	realtek_tmp_debug_log(0, __FILE_NUM__, __LINE__, __VA_ARGS__)
#define COLOR_RT_TMP_LOG(color, ...) \
	realtek_tmp_debug_log(color, __FILE_NUM__, __LINE__, __VA_ARGS__)

#endif // FOR_SIMULATION

#else  // not RT_TMP_LOG_ENABLE

#ifdef FOR_SIMULATION
#if 1 /* let simulation fw use same API as real fw */
#define RT_TMP_LOG(...)
#define COLOR_RT_TMP_LOG(color, ...)
#else
#define RT_TMP_LOG UARTPrintf
#define COLOR_RT_TMP_LOG COLOR_UARTPrintf
#endif
#else //FOR_SIMULATION
#define RT_TMP_LOG(...)
#define COLOR_RT_TMP_LOG(color, ...)
#endif //FOR_SIMULATION

#endif//RT_TMP_LOG_ENABLE

//***********************************************************************
//************************ Chris Kuo Add ******************************** 
//************************ For tracing code *****************************
//***********************************************************************
#ifdef _USB_DMA_DBG_
#define DMA_DBG_LOG(color, log_str_index, para_num, ...) \
	LOG(color, __FILE_NUM__, __LINE__, log_str_index, para_num, "A", __VA_ARGS__)
#else
#define DMA_DBG_LOG(color, log_str_index, para_num, ...)
#endif

#ifdef _YL_LPS_LOG_
#define LPS_DBG_LOG(color, log_str_index, para_num, ...) \
	LOG(color, __FILE_NUM__, __LINE__, log_str_index, para_num, "A", __VA_ARGS__)
#else
#define LPS_DBG_LOG(color, log_str_index, para_num, ...)
#endif


#ifdef _LPS_LOG_EN_
#define LPS_CCH_LOG(color, log_str_index, para_num, ...) \
	LOG(color, __FILE_NUM__, __LINE__, log_str_index, para_num, "A", __VA_ARGS__)
#else
#define LPS_CCH_LOG(color, log_str_index, para_num, ...)
#endif

#ifdef _RTK8723_UART_INIT_

#ifdef _UART_LOG_
#define DMA_UART_LOG(color, log_str_index, para_num, ...) \
	LOG(color, __FILE_NUM__, __LINE__, log_str_index, para_num, "A", __VA_ARGS__)
#else
#define DMA_UART_LOG(color, log_str_index, para_num, ...)
#endif
#ifdef _UART_DBG_LOG_
#define DMA_UART_DBG_LOG(color, log_str_index, para_num, ...) \
	LOG(color, __FILE_NUM__, __LINE__, log_str_index, para_num, "A", __VA_ARGS__)
#else
#define DMA_UART_DBG_LOG(color, log_str_index, para_num, ...)
#endif

#else
#define DMA_UART_LOG(color, log_str_index, para_num, ...)
#define DMA_UART_DBG_LOG(color, log_str_index, para_num, ...)
#endif

#ifdef _MEMORY_DBG_
#define MEM_DBG_LOG(color, log_str_index, para_num, ...) \
	LOG(color, __FILE_NUM__, __LINE__, log_str_index, para_num, "A", __VA_ARGS__)
#else
#define MEM_DBG_LOG(color, log_str_index, para_num, ...)
#endif

#ifdef _DMA_TX_DBG_LOG_
#define DMA_TX_DBG_LOG(color, log_str_index, para_num, ...) \
	LOG(color, __FILE_NUM__, __LINE__, log_str_index, para_num, "A", __VA_ARGS__)
#else
#define DMA_TX_DBG_LOG(color, log_str_index, para_num, ...)
#endif


#ifdef _TWO_MAC_DBG_
#define MAC_DBG_LOG(color, log_str_index, para_num, ...) \
	LOG(color, __FILE_NUM__, __LINE__, log_str_index, para_num, "A", __VA_ARGS__)
#else
#define MAC_DBG_LOG(color, log_str_index, para_num, ...)
#endif

#ifdef _ENABLE_GPS_DBG_
#define GPS_DBG_LOG(color, log_str_index, para_num, ...) \
	LOG(color, __FILE_NUM__, __LINE__, log_str_index, para_num, "A", __VA_ARGS__)
#else
#define GPS_DBG_LOG(color, log_str_index, para_num, ...)
#endif

#ifdef LE_MODE_EN
#define LL_LOG_TRACE(color, log_str_index, para_num, ...) \
	LOG(color, __FILE_NUM__, __LINE__, log_str_index, para_num, "A", __VA_ARGS__)
#define LL_LOG_DATA(Length, log_str_index, Ptr) \
	LOG_DATA_FUNCTION(0, __FILE_NUM__, __LINE__, log_str_index, Length, Ptr)	
#else
#define LL_LOG_TRACE(color, log_str_index, para_num, ...)
#define LL_LOG_DATA(Length, log_str_index, Ptr)
#endif

#if defined(_SUPPORT_CSB_TRANSMITTER_) || defined(_SUPPORT_CSB_RECEIVER_)
#define CSB_LOG(color, log_level, log_str_index, para_num, ...) \
    {if ((log_level) & LOG_LEVEL_MASK) { \
    LOG(color, __FILE_NUM__, __LINE__, log_str_index, para_num, "A", __VA_ARGS__);\
    }}
#define CSB_LOG_DATA(Length, log_str_index, Ptr) \
	LOG_DATA_FUNCTION(0, __FILE_NUM__, __LINE__, log_str_index, Length, Ptr)	
#else
#define CSB_LOG(color, log_level, log_str_index, para_num, ...)
#define CSB_LOG_DATA(Length, log_str_index, Ptr)
#endif

#ifdef CONFIG_TV_FEATURE
#define TV_LOG(color, log_level, log_str_index, para_num, ...) \
    do { \
        if ((log_level) & LOG_LEVEL_MASK) \
        { \
            LOG(color, __FILE_NUM__, __LINE__, log_str_index, para_num, "A", __VA_ARGS__); \
        } \
    } while (0)
#define TV_LOG_DATA(Length, log_str_index, Ptr) \
    LOG_DATA_FUNCTION(0, __FILE_NUM__, __LINE__, log_str_index, Length, Ptr)
#else
#define TV_LOG(color, log_level, log_str_index, para_num, ...)
#define TV_LOG_DATA(Length, log_str_index, Ptr)
#endif

#ifdef CONFIG_FM
#define FM_LOG(color, log_level, log_str_index, para_num, ...) \
    do { \
        if ((log_level) & LOG_LEVEL_MASK) \
        { \
            LOG(color, __FILE_NUM__, __LINE__, log_str_index, para_num, "A", __VA_ARGS__); \
        } \
    } while (0)
#define FM_LOG_DATA(Length, log_str_index, Ptr) \
    LOG_DATA_FUNCTION(0, __FILE_NUM__, __LINE__, log_str_index, Length, Ptr)
#else
#define FM_LOG(color, log_level, log_str_index, para_num, ...)
#define FM_LOG_DATA(Length, log_str_index, Ptr)
#endif /* CONFIG_FM */

#if defined(_MT8852B_DBG_) || defined(_DUT_MODE_DBG_)
#define MT8852B_DBG(color, log_str_index, para_num, ...) \
	LOG(color, __FILE_NUM__, __LINE__, log_str_index, para_num, "A", __VA_ARGS__)
#else
#define MT8852B_DBG(color, log_str_index, para_num, ...)
#endif

#ifdef _ENABLE_BTON_POWER_SAVING_DBG_
#define POW_DBG_LOG(color, log_str_index, para_num, ...) \
	LOG(color, __FILE_NUM__, __LINE__, log_str_index, para_num, "A", __VA_ARGS__)
#else
#define POW_DBG_LOG(color, log_str_index, para_num, ...)
#endif

#ifdef _MAILBOX_DEBUG_
#define MAILBOX_LOG(color, log_str_index, para_num, ...) \
	LOG(color, __FILE_NUM__, __LINE__, log_str_index, para_num, "A", __VA_ARGS__)
#else
#define MAILBOX_LOG(color, log_str_index, para_num, ...)
#endif

#ifdef _RF0380_DBG_
#define RF0380_LOG(color, log_str_index, para_num, ...) \
	LOG(color, __FILE_NUM__, __LINE__, log_str_index, para_num, "A", __VA_ARGS__)
#define RF0380_FAST_LOG(...) \
	realtek_tmp_debug_log(0, __FILE_NUM__, __LINE__, __VA_ARGS__)

#else
#define RF0380_LOG(color, log_str_index, para_num, ...)
#define RF0380_FAST_LOG(...)
#endif

#ifdef _BT_SW_POOL_DBG_
#define BT_SW_POOL_LOG(color, log_str_index, para_num, ...) \
	LOG(color, __FILE_NUM__, __LINE__, log_str_index, para_num, "A", __VA_ARGS__)
#else
#define BT_SW_POOL_LOG(color, log_str_index, para_num, ...)
#endif


//}}}noted by liuyong 20091223==================================

#else /* ENABLE_LOGGER */
#define LOG_ERROR(log_module, log_level, log_str_index, para_num, ...) 
#define LOG_WARNING(log_module, log_level, log_str_index, para_num, ...)
#define LOG_INFO(log_module, log_level, log_str_index, para_num, ...)
#define LOG_TRACE(log_module, log_level, log_str_index, para_num, ...)
#define LOG_PDU(log_module, log_level, Length, log_dir, log_str_index, Ptr)
#define LOG_DATA(log_module, log_level, Length, log_dir, log_str_index, Ptr)
#define LOG_COMMAND(log_module, log_level, Length, log_dir, log_str_index, Ptr)
#define LOG_EVENT(log_module, log_level, Length, log_dir, log_str_index, Ptr)

#ifdef FOR_SIMULATION
//#define RT_TMP_LOG UARTPrintf
#define RT_TMP_LOG(...)
#else
#define RT_TMP_LOG(...)
#endif

#define COLOR_LOG_ERROR(color, log_module, log_level, log_str_index, para_num, ...) 
#define COLOR_LOG_WARNING(color, log_module, log_level, log_str_index, para_num, ...)
#define RT_BT_LOG(color, log_str_index, para_num, ...)
#define COLOR_LOG_INFO(color, log_module, log_level, log_str_index, para_num, ...)
#define COLOR_LOG_TRACE(color, log_module, log_level, log_str_index, para_num, ...)
#define COLOR_LOG_PDU(color, log_module, log_level, Length, log_dir, log_str_index, Ptr)
#define COLOR_LOG_DATA(color, log_module, log_level, Length, log_dir, log_str_index, Ptr)
#define COLOR_LOG_COMMAND(color, log_module, log_level, Length, log_dir, log_str_index, Ptr)
#define COLOR_LOG_EVENT(color, log_module, log_level, Length, log_dir, log_str_index, Ptr)

#ifdef FOR_SIMULATION
//#define COLOR_RT_TMP_LOG COLOR_UARTPrintf
#define COLOR_RT_TMP_LOG(color, ...)
#else
#define COLOR_RT_TMP_LOG(color, ...)
#endif

#define POW_DBG_LOG(color, log_str_index, para_num, ...)
#define DMA_UART_LOG(color, log_str_index, para_num, ...)
#define DMA_UART_DBG_LOG(color, log_str_index, para_num, ...)
#define DMA_DBG_LOG(color, log_str_index, para_num, ...)
#define MEM_DBG_LOG(color, log_str_index, para_num, ...)
#define DMA_TX_DBG_LOG(color, log_str_index, para_num, ...)
#define MAC_DBG_LOG(color, log_str_index, para_num, ...)
#define GPS_DBG_LOG(color, log_str_index, para_num, ...)
#define LL_LOG_TRACE(color, log_str_index, para_num, ...)
#define LL_LOG_DATA(Length, log_str_index, Ptr)
#define CSB_LOG(color, log_level, log_str_index, para_num, ...)
#define CSB_LOG_DATA(Length, log_str_index, Ptr)
#define MT8852B_DBG(color, log_str_index, para_num, ...)
#define MAILBOX_LOG(color, log_str_index, para_num, ...)
#define RF0380_LOG(color, log_str_index, para_num, ...)
#define RF0380_FAST_LOG(...)
#define LPS_DBG_LOG(color, log_str_index, para_num, ...)
#define BT_SW_POOL_LOG(color, log_str_index, para_num, ...)
#define LPS_CCH_LOG(color, log_str_index, para_num, ...)

#endif /* ENABLE_LOGGER */

#endif /* _LOGGER_ */

