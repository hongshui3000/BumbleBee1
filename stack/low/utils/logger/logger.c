/***************************************************************************
 Copyright (C) MindTree Ltd.
 This module is a confidential and proprietary property of MindTree and
 a possession or use of this module requires written permission of MindTree.
 ***************************************************************************/

/********************************* Logger *************************/
enum { __FILE_NUM__= 87 };
/********************************* Logger *************************/

#include "logger.h"
#include "mem.h"
#include "bt_fw_os.h"
#include "bb_driver.h"
#include "bt_fw_os.h"
#include "bz_debug.h"
#include "lbm.h"
#include "mailbox.h"
#include "lc_internal.h"
#include "otp.h"
#include "mint_os.h"
#include "uart.h"
#include "pta.h"
#include "hci_vendor_defines.h"
#include "dma_usb.h"

#if defined(_SUPPORT_FW_READ_SIE_)
#include "new_io.h"
#endif


#ifdef ENABLE_LOGGER

#ifdef _ROM_CODE_PATCHED_
PF_ROM_CODE_PATCH_FUNC rcp_log = NULL;        
#endif

#ifdef _DAPE_LOG_ADD_INDEX
UINT8 g_log_index = 0;
#endif

#define MAX_LOG_MESSAGE_LEN 128//256//1024
LOG_LogControl LogControl= {1,LOG_LEVEL_HIGH};

extern OS_HANDLE isr_extended_task_handle;

/** Logger functions */
BOOLEAN (*logger_transport_write)(UCHAR *buf, UINT16 len);
BOOLEAN (*logger_transport_is_free)(void);

/* The logger module should not be used before it is initialized. Since the
 * logger task is the low priority task and all the other tasks get to execute
 * before it, these tasks log some data before logger is initialized.
 * To avoid this issue, 'is_logger_initialized' variable is used.
 */
static BOOLEAN is_logger_initialized = FALSE;
UINT8 uart_send_buf[16];
UINT16 uart_dbg_len = 0;
OS_HANDLE bz_logger_task_handle;

void pf_os_trigger_logger_task(void)
{
    OS_SIGNAL signal;
    signal.type = LOGGER_LOG_DATA_SIGNAL;
    OS_SEND_SIGNAL_TO_TASK(bz_logger_task_handle, signal);
}

void pf_os_trigger_uar_dbg_task(void)
{
    OS_SIGNAL signal;
    signal.type = UART_DBG_DATA_SIGNAL;
    OS_SEND_SIGNAL_TO_TASK(bz_logger_task_handle, signal);
}

UCHAR bz_logger_task(OS_SIGNAL *signal_ptr)
{
    switch(signal_ptr->type)
    {
        case LOGGER_LOG_DATA_SIGNAL:
            pf_service_logger();
            break;
        case UART_DBG_DATA_SIGNAL:
            pf_service_uart_dbg();
            break;		
        default:
            /* Not the registered signal */
            break;
    }

    return BT_FW_SUCCESS;
}

void logger_write_to_log_buffer(UCHAR *Data, int Length)
{
    if ((!is_logger_initialized) || IS_NOT_GENERATE_LOG_MSG)
    {
        return;
    }


    /* Put it into the logger circular buffer */
    if (lbm_put(Data, (UINT16)Length) == FALSE)
    {
        /* Logger buffer full */
    }


#ifdef _DONT_USE_LOG_UART_TX_INT_
    /* Do not need to check and send signal via log UART */
    if (dbg_vendor_log_interface == VENDOR_LOG_PACKET_TYPE_INVALID)
    {
        return;
    }
#endif

    /* Send it to host, if possible */
    if (logger_transport_is_free() == TRUE)
    {
        pf_os_trigger_logger_task();
    }
}

void logger_handle_tx_completed(UCHAR* buf, UINT16 len, UINT8 type)
{
    pf_os_trigger_logger_task();
}

LOGGER_TRANS_COMPLETE_CB logger_init(LOGGER_TRANS_WRITE_CB log_write_cb,
                                     LOGGER_TRANS_IS_FREE_CB log_free_cb)
{
    logger_transport_write = log_write_cb;
    logger_transport_is_free = log_free_cb;
    return logger_handle_tx_completed;
}

UCHAR bz_logger_init(void)
{

    pf_logger_transport_init(NULL, logger_handle_tx_completed, 
                             uart_dbg_receive_packet);

    is_logger_initialized = TRUE;

    lbm_init();

    //---Create Logger task---
    if(OS_CREATE_TASK( &bz_logger_task_handle, LOGGER_TASK_NAME,
                       LOGGER_TASK_PRI,
                       (OS_TASK_FUNCTION)bz_logger_task,
                       LOGGER_TASK_Q_SIZE,
                       LOGGER_TASK_BUSY_PERIOD) != BT_ERROR_OK)
    {
    }
    return API_SUCCESS;
}


UCHAR bz_logger_send_raw_data(void)
{    
    /* Send logger data to host, if possible */
    if (IS_SUPPORT_HCI_LOG_PKT &&
        (dbg_vendor_log_interface != VENDOR_LOG_PACKET_TYPE_INVALID))
    {
        hci_vendor_generate_log_data_packet();
    }
#ifndef _DONT_USE_LOG_UART_TX_INT_
    else
    {
        UCHAR* buf;
        UINT16 len;
    
        if (logger_transport_is_free() && lbm_get(&buf, &len))
        {
            if (!logger_transport_write(buf, len))
            {
                /* Logger transport write failed */
            }
        }
    }
#endif    
    return API_SUCCESS;
}

UCHAR bz_uart_dbg_send_raw_data(void)
{
    /* Send logger data to host, if possible */
    if (logger_transport_is_free())
    {
        if (!logger_transport_write(uart_send_buf, uart_dbg_len))
        {
            /* Logger transport write failed */
        }
    }
    else
    {
        pf_os_trigger_uar_dbg_task();
    }
    return API_SUCCESS;
}

/////////////////////////////////////////////////////////tmp log//////////////////////////////////////////////////

//{{{added by andy_liu 20090327

//{{{noted by liuyong 20091223============================================

//***********************************************************************
//***This method provide a format_print method for quick debug, but   ***
//***it is not an efficiency method and do not use in release version!***
//***********************************************************************

#ifdef RT_TMP_LOG_ENABLE
#define		IsDigit(x)	( ((x) >= '0') && ((x) <= '9') )
#define		Ctod(x)		( (x) - '0')
static void GenPrint(UINT16 *len, UINT8 *str, UINT8 *fmt, va_list ap);
static INT32 PrintChar(UINT8 *, UINT8, INT32, INT32);
static INT32 PrintString(UINT8 *, UINT8 *, INT32, INT32);
static INT32 PrintNum(UINT8 *, unsigned long, INT32, INT32, INT32, INT32, UINT8, INT32);

UCHAR realtek_tmp_debug_log(UINT8 color, UINT16 file_num, UINT16 line_num,
                            const char *format, ...)
{
    UINT8 buf[128];
    UINT16 length;

    va_list ap;
    va_start(ap, format);
    GenPrint(&length, buf, (UINT8*)format, ap);
    va_end(ap);

    LOG_DATA_FUNCTION(color, file_num, line_num, 0, length, buf);

    return 0;
}

void GenPrint(UINT16 *len, UINT8 *str, UINT8 *fmt, va_list ap)
{

#define 	OUTPUT(s, l)  \
    {\
        for(i=0; i<l; i++) \
        {\
            *pBuf = *(s+i);\
            pBuf++;\
        }\
        str_length+=l;\
    }

    UINT8 i;

    UINT16 str_length = 0;
    UINT8 *pBuf = str;

    UINT8 buf[64];

    UINT8 c;
    UINT8 *s;
    long int num;

    INT32 longFlag;
    INT32 negFlag;
    INT32 width;
    INT32 prec;
    INT32 ladjust;
    UINT8 padc;

    INT32 length;

    for(;;)
    {
        {
            /* scan for the next '%' */
            UINT8 *fmtStart = fmt;
            while ( (*fmt != '\0') && (*fmt != '%'))
            {
                fmt ++;
            }

            /* flush the string found so far */
            OUTPUT(fmtStart, fmt-fmtStart);

            /* are we hitting the end? */
            if (*fmt == '\0') break;
        }

        /* we found a '%' */
        fmt ++;

        /* check for long */
        if (*fmt == 'l')
        {
            longFlag = 1;
            fmt ++;
        }
        else
        {
            longFlag = 0;
        }

        /* check for other prefixes */
        width = 0;
        prec = -1;
        ladjust = 0;
        padc = ' ';

        if (*fmt == '-')
        {
            ladjust = 1;
            fmt ++;
        }

        if (*fmt == '0')
        {
            padc = '0';
            fmt++;
        }

        if (IsDigit(*fmt))
        {
            while (IsDigit(*fmt))
            {
                width = 10 * width + Ctod(*fmt++);
            }
        }

        if (*fmt == '.')
        {
            fmt ++;
            if (IsDigit(*fmt))
            {
                prec = 0;
                while (IsDigit(*fmt))
                {
                    prec = prec*10 + Ctod(*fmt++);
                }
            }
        }


        /* check format flag */
        negFlag = 0;
        switch (*fmt)
        {
            case 'b':
                if (longFlag)
                {
                    num = va_arg(ap, long int);
                }
                else
                {
                    num = va_arg(ap, INT32);
                }
                length = PrintNum(buf, num, 2, 0, width, ladjust, padc, 0);
                OUTPUT(buf, length);
                break;

            case 'd':
            case 'D':
                if (longFlag)
                {
                    num = va_arg(ap, long int);
                }
                else
                {
                    num = va_arg(ap, INT32);
                }
                if (num < 0)
                {
                    num = - num;
                    negFlag = 1;
                }
                length = PrintNum(buf, num, 10, negFlag, width, ladjust, padc, 0);
                OUTPUT(buf, length);
                break;

            case 'o':
            case 'O':
                if (longFlag)
                {
                    num = va_arg(ap, long int);
                }
                else
                {
                    num = va_arg(ap, INT32);
                }
                length = PrintNum(buf, num, 8, 0, width, ladjust, padc, 0);
                OUTPUT(buf, length);
                break;

            case 'u':
            case 'U':
                if (longFlag)
                {
                    num = va_arg(ap, long int);
                }
                else
                {
                    num = va_arg(ap, INT32);
                }
                length = PrintNum(buf, num, 10, 0, width, ladjust, padc, 0);
                OUTPUT(buf, length);
                break;

            case 'x':
                if (longFlag)
                {
                    num = va_arg(ap, long int);
                }
                else
                {
                    num = va_arg(ap, INT32);
                }
                length = PrintNum(buf, num, 16, 0, width, ladjust, padc, 0);
                OUTPUT(buf, length);
                break;

            case 'X':
                if (longFlag)
                {
                    num = va_arg(ap, long int);
                }
                else
                {
                    num = va_arg(ap, INT32);
                }
                length = PrintNum(buf, num, 16, 0, width, ladjust, padc, 1);
                OUTPUT(buf, length);
                break;

            case 'c':
                c = (UINT8)va_arg(ap, INT32);
                length = PrintChar(buf, c, width, ladjust);
                OUTPUT(buf, length);
                break;

            case 's':
                s = (UINT8*)va_arg(ap, UINT8 *);
                length = PrintString(buf, s, width, ladjust);
                OUTPUT(buf, length);
                break;

            case '\0':
                fmt --;
                break;

            default:
                /* output this UINT8 as it is */
                OUTPUT(fmt, 1);
        }	/* switch (*fmt) */

        fmt ++;
    }		/* for(;;) */

    /* special termination call */
    OUTPUT("\0", 1);

    *len = str_length;
}


/* --------------- local help functions --------------------- */
INT32
PrintChar(UINT8 * buf, UINT8 c, INT32 length, INT32 ladjust)
{
    INT32 i;

    if (length < 1) 
    {
        length = 1;
    }

    if (ladjust)
    {
        *buf = c;
        for (i=1; i< length; i++) 
        {
            buf[i] = ' ';
        }
    }
    else
    {
        for (i=0; i< length-1; i++) 
        {
            buf[i] = ' ';
        }
        buf[length - 1] = c;
    }
    return length;
}

INT32
PrintString(UINT8 * buf, UINT8* s, INT32 length, INT32 ladjust)
{
    INT32 i;
    INT32 len=0;
    UINT8* s1 = s;
    while (*s1++) len++;
    if (length < len) 
    {
        length = len;
    }

    if (ladjust)
    {
        for (i=0; i< len; i++) 
        {
            buf[i] = s[i];
        }
        for (i=len; i< length; i++) 
        {
            buf[i] = ' ';
        }
    }
    else
    {
        for (i=0; i< length-len; i++) 
        {
            buf[i] = ' ';
        }
        for (i=length-len; i < length; i++) 
        {
            buf[i] = s[i-length+len];
        }
    }
    return length;
}

INT32
PrintNum(UINT8 * buf, unsigned long u, INT32 base, INT32 negFlag,
         INT32 length, INT32 ladjust, UINT8 padc, INT32 upcase)
{
    /* algorithm :
    *  1. prints the number from left to right in reverse form.
    *  2. fill the remaining spaces with padc if length is longer than
    *     the actual length
    *     TRICKY : if left adjusted, no "0" padding.
    *		    if negtive, insert  "0" padding between "0" and number.
    *  3. if (!ladjust) we reverse the whole string including paddings
    *  4. otherwise we only reverse the actual string representing the num.
    */

    INT32 actualLength =0;
    UINT8 *p = buf;
    INT32 i;

    do
    {
        INT32 tmp = u %base;
        if (tmp <= 9)
        {
            *p++ = '0' + tmp;
        }
        else if (upcase)
        {
            *p++ = 'A' + tmp - 10;
        }
        else
        {
            *p++ = 'a' + tmp - 10;
        }
        u /= base;
    }
    while (u != 0);

    if (negFlag)
    {
        *p++ = '-';
    }

    /* figure out actual length and adjust the maximum length */
    actualLength = p - buf;
    if (length < actualLength) 
    {
        length = actualLength;
    }

    /* add padding */
    if (ladjust)
    {
        padc = ' ';
    }
    if (negFlag && !ladjust && (padc == '0'))
    {
        for (i = actualLength-1; i< length-1; i++) 
        {
            buf[i] = padc;
        }
        buf[length -1] = '-';
    }
    else
    {
        for (i = actualLength; i< length; i++) 
        {
            buf[i] = padc;
        }
    }


    /* prepare to reverse the string */
    {
        INT32 begin = 0;
        INT32 end;
        if (ladjust)
        {
            end = actualLength - 1;
        }
        else
        {
            end = length -1;
        }

        while (end > begin)
        {
            UINT8 tmp = buf[begin];
            buf[begin] = buf[end];
            buf[end] = tmp;
            begin ++;
            end --;
        }
    }
    return length;
}

#endif//RT_TMP_LOG_ENABLE

//}}}added by andy_liu 20090327

//}}}noted by liuyong 20091223============================================



///////////////////////////////////////////////////logger functions////////////////////////////////////
UCHAR LOG_DATA_FUNCTION(UINT8 color, UINT16 file_num, UINT16 line_num,
                        UINT16 log_str_index, UINT16 Length, UCHAR* Ptr)
{
    UINT16 log_length;
    UCHAR l_msg[MAX_LOG_MESSAGE_LEN];

    if ((!is_logger_initialized) || IS_NOT_GENERATE_LOG_MSG)
    {
        return API_FAILURE;
    }

    log_length = 0;

    l_msg[0] = l_msg[1] = 0xA5;
    l_msg[2] = ((color&0x7)<<5)				//color
               | 0x04						//1=has file number
               | 0x02;						//1=has line number

    l_msg[3] = 0x5A;						//reserved byte
    l_msg[4] = (UCHAR)(log_str_index & 0xff);
    l_msg[5] = (UCHAR)((log_str_index & 0xff00)>>8);
    l_msg[6] = 0x00;
    l_msg[7] = 0x00;

    l_msg[8] = (UCHAR)(file_num&0xff);			//8 bits file_num
    l_msg[9] = (UCHAR)(line_num&0xff);			//8 bits line_num
    l_msg[10] = (UCHAR)((line_num&0xff00)>>8);	//8 bits line_num

    if(Length>0)
    {
        l_msg[2] |= 0x10;					//has str para
        l_msg[11] = (UCHAR)Length;
        log_length = 12;

        memcpy(l_msg+log_length, Ptr, Length);	//Length*8bits data
        log_length+=Length;
    }
    else
    {
        //has no paras
        log_length = 11;
    }

    logger_write_to_log_buffer(l_msg, log_length);
    //	LOG_SYNC_PKT(LogType);

    return API_SUCCESS;
}


UCHAR LOG(UINT8 color, UINT16 file_num, UINT16 line_num,
          UINT16 log_str_index, UINT8 para_num, const char *format, ...)
{
    UINT8 log_length, i;
    va_list argp;
    UCHAR l_msg[MAX_LOG_MESSAGE_LEN];

    if ((!is_logger_initialized) || IS_NOT_GENERATE_LOG_MSG)
    {
        return API_FAILURE;
    }

#ifdef _DAPE_LOG_ADD_INDEX
    UINT8 loc_idx = g_log_index;
    g_log_index++;
#endif

    log_length = 0;

    l_msg[0] = l_msg[1] = 0xA5;				//syntax
    l_msg[2] = ((color&0x7)<<5)				//color
               | 0x04						//1=has file number
               | 0x02;						//1=has line number

    l_msg[3] = 0x5A;						//syntax
    l_msg[4] = log_str_index & 0xff;
    l_msg[5] = log_str_index >>8;
#ifdef _DAPE_LOG_ADD_INDEX
    l_msg[6] = loc_idx;
#else
    l_msg[6] = 0x00;
#endif    
    l_msg[7] = 0x00;

    l_msg[8] = (UCHAR)(file_num&0xff);		//8 bits file_num
    l_msg[9] = (UCHAR)(line_num&0xff);		//8 bits line_num
    l_msg[10] = (UCHAR)((line_num&0xff00)>>8);	//8 bits line_num

    if(para_num>0)
    {
        INT32* pInt32;

        l_msg[2] |= 0x08;					//has str para
        l_msg[11] = (UCHAR)para_num;
        log_length = 12;

        pInt32 = (INT32 *)(l_msg+log_length);
        va_start(argp, format);
        for(i=0; i<para_num; i++)			//para_num*32bits data
        {
            *pInt32 = va_arg(argp, int);
            pInt32++;
            log_length+=4;
        }
        va_end(argp);
    }
    else
    {
        //has no paras
        log_length = 11;
    }

#ifdef _ROM_CODE_PATCHED_
    if (rcp_log != NULL)
    {
        /* hehe.. we can use log's some matched parameters 
           to patch some procedure - austin */
        rcp_log(l_msg, log_length);
    }      
#endif

    /*The Last Part is the Data*/
    logger_write_to_log_buffer(l_msg, (int)log_length);

    return API_SUCCESS;
}

void uart_dbg_receive_packet(UCHAR* buf, UINT16 len)
{
    OS_SIGNAL sig_send;

    /* Send signal to background */
    sig_send.type = ISR_EXT_UART_RX;
    sig_send.param = (OS_ADDRESS)((UINT32)(buf - uart_dbg_data));
    sig_send.ext_param = (OS_ADDRESS)((UINT32)len);    
    OS_ISR_SEND_SIGNAL_TO_TASK(isr_extended_task_handle, sig_send);
}

void uart_dbg_receive_packet_delayed(UINT16 rptr, UINT16 len)
{   

    UINT16 reg_offset = 0;
    UINT32 reg_value = 0;
    UINT32 read_value = 0xDEADDEAD;
    UINT8 modem_rf_addr = 0, i;
    UART_DBG_PKT_TYPE uart_dbg_pkt;
    GENERAL_CONTROL_S_TYPE bt_general_ctrl;
    UINT8 buf[16];
    UINT8 alignment_check = 1;
    UINT16 ori_ptr = rptr;
    
    //the setting value loaded from efuse
    bt_general_ctrl.d16 = otp_str_data.general_control;    

    for (i = 0; i < len; i++)
    {
        buf[i] = uart_dbg_data[rptr];
        rptr = (rptr + 1) & (REV_FIFO_LEN - 1);
    }

    uart_dbg_pkt.byte = buf[0];

    RT_BT_LOG(WHITE, MSG_LOG_UART_CMD_GENERAL, 10, 
        len, ori_ptr, buf[0], buf[1], buf[2], buf[3], buf[4], buf[5], buf[6], buf[7]); 

#ifdef _NEW_MODEM_PI_ACCESS_                
    if ( (uart_dbg_pkt.cmd_type < 10) || (uart_dbg_pkt.cmd_type == MODEM_REGISTER_CMD_PI))
#else
    if (uart_dbg_pkt.cmd_type < 10)
#endif
    {
        reg_offset = buf[2] | (buf[3]<<8);
        reg_value = buf[4] | (buf[5]<<8) | (buf[6]<<16) | (buf[7]<<24);
        modem_rf_addr = (UCHAR) (reg_offset & 0xFF);

        //RT_BT_LOG(WHITE,MSG_LOG_UART_RX_INPUT, 5, 
        //                uart_dbg_pkt.cmd_type, uart_dbg_pkt.is_16b,
        //                uart_dbg_pkt.is_write, reg_offset, reg_value);

        if ((uart_dbg_pkt.is_16b))
        {
            //Check if the address is 2 byte alignment
           if ((reg_offset & TWO_ALIGNMENT))
           {
               alignment_check = 0;
           }
        }
        else
        {
            //Check if the address is 4 byte alignment
            if ((reg_offset & FOUR_ALIGNMENT))
            {
                alignment_check = 0;
            }                
        }

        if (!alignment_check)
        {
            RT_BT_LOG(RED, REG_ALIGNMENT_ERROR, 1, reg_offset);    
            return;
        }
        
        if (bt_general_ctrl.b.log_uart_rx_en)
        {            
            mailbox_reg_offset = reg_offset;

            if (uart_dbg_pkt.cmd_type == MODEM_REGISTER_CMD)
            {
                read_value = mailbox_read_write_rf_modem_register(modem_rf_addr, TYPE_MODEM, 
                                            reg_value, uart_dbg_pkt.is_write);
            }
            else if (uart_dbg_pkt.cmd_type == RF_REGISTER_CMD)
            {
                read_value = mailbox_read_write_rf_modem_register(modem_rf_addr, TYPE_RF, 
                                            reg_value, uart_dbg_pkt.is_write);
            }
#ifdef _NEW_MODEM_PI_ACCESS_                
            else if (uart_dbg_pkt.cmd_type == MODEM_REGISTER_CMD_PI)
            {
                UINT8 modem_page = (UINT8) ( (reg_offset>>8) & 0xFF);
                read_value = mailbox_read_write_modem_register_pi(modem_page, modem_rf_addr, 
                                               reg_value, uart_dbg_pkt.is_write);
            }
#endif                                        
#ifdef _ENABLE_MAILBOX_
            else
            {
                UINT32 correct_base_reg_addr;            
                correct_base_reg_addr = mailbox_reg_base_addr[uart_dbg_pkt.cmd_type];
                read_value = mailbox_read_write_register(uart_dbg_pkt.is_16b, uart_dbg_pkt.is_write, 
                                    correct_base_reg_addr);
            }
#endif

            if (!uart_dbg_pkt.is_write)
            {
                RT_BT_LOG(BLUE,MAILBOX_READ_VALUE,1, read_value);                    
            }
        }
    }
    else
    {
#if defined(_SUPPORT_LOGRX_READ_SIE_) && defined(_SUPPORT_FW_READ_SIE_)
            //else if(uart_dbg_pkt.cmd_type == SIE_REGISTER)
            //{
            if(buf[0] == 0xFE)
                {
                read_value = indirect_read_sie(0, buf[1]);
                RT_BT_LOG(BLUE,READ_SIE_DATA,2, buf[1], read_value);
                }
                
            //}
#endif

    }
}

#endif /* ENABLE_LOGGER */

