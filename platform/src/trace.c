/*
 *  Routines to access hardware
 *
 *  Copyright (c) 2013 Realtek Semiconductor Corp.
 *
 *  This module is a confidential and proprietary property of RealTek and
 *  possession or use of this module requires written permission of RealTek.
 */

#include <stdarg.h>
#include <string.h>
#include "rtl_types.h"
#include "trace.h"
#include "section_config.h"
#include "patch_framework.h"
#include "FreeRTOS.h"
#include "task.h"
#include "pingpong_buffer.h"
#include "rtl876x_log_uart.h"

/** @brief  LOG Message Type */
typedef enum tagLOG_MESSAGE_TYPE {
    TYPE_BUMBLEBEE3 = 32,
} tLOG_MESSAGE_TYPE;

#define LOG_MESSAGE_SYNC_CODE         0x7E
#define MAX_LOG_MESSAGE_LEN           128
#define MAX_ARGUMENT_SIZE             20
#define DiagPutChar                   LogUartTxChar


uint32_t gTraceLevelModules[LEVEL_NUMs][MODULE_GROUP_NUM] = {
    {0xFFFFFFFF, 0xFFFFFFFF},
    {0xFFFFFFFF, 0xFFFFFFFF},
    {0xFFFFFFFF, 0xFFFFFFFF},
    {0xFFFFFFFF, 0xFFFFFFFF}
};

uint8_t gSequenceNumber = 0xFF;

void LogUartPrint(const uint8_t *source, uint16_t size);

static void LogUartTxChar(const uint8_t ch)
{
    while(LOG_UART_GetFlagState(LOG_UART, LOG_UART_FLAG_THR_TSR_EMPTY) != SET);
        LOG_UART_SendData(LOG_UART, &ch, 1);
}

static int VSprintf(char *buf, const char *fmt, const int *dp)
{
	char *p, *s;

	s = buf;
	for ( ; *fmt != '\0'; ++fmt) {
		if (*fmt != '%') {
			if(buf)
				*s++ = *fmt;
			else
				DiagPutChar(*fmt);
			continue;
		}
		if (*++fmt == 's') {
			for (p = (char *)*dp++; *p != '\0'; p++)
			{
				if(buf)
					*s++ = *p;
				else
					DiagPutChar(*p);
			}
		}
		else {	/* Length of item is bounded */
			char tmp[20], *q = tmp;
			int alt = 0;
			int shift = 28;

#if 1   //wei patch for %02x
			if ((*fmt  >= '0') && (*fmt  <= '9'))
			{
				int width;
				unsigned char fch = *fmt;
		                for (width=0; (fch>='0') && (fch<='9'); fch=*++fmt)
		                {    width = width * 10 + fch - '0';
		                }
				  shift=(width-1)*4;
			}
#endif

			/*
			 * Before each format q points to tmp buffer
			 * After each format q points past end of item
			 */

			if ((*fmt == 'x')||(*fmt == 'X') || (*fmt == 'p') || (*fmt == 'P')) {
				/* With x86 gcc, sizeof(long) == sizeof(int) */
				const long *lp = (const long *)dp;
				long h = *lp++;
				int ncase = (*fmt & 0x20);
				dp = (const int *)lp;
				if((*fmt == 'p') || (*fmt == 'P'))
					alt=1;
				if (alt) {
					*q++ = '0';
					*q++ = 'X' | ncase;
				}
				for ( ; shift >= 0; shift -= 4)
					*q++ = "0123456789ABCDEF"[(h >> shift) & 0xF] | ncase;
			}
			else if (*fmt == 'd') {
				int i = *dp++;
				char *r;
				if (i < 0) {
					*q++ = '-';
					i = -i;
				}
				p = q;		/* save beginning of digits */
				do {
					*q++ = '0' + (i % 10);
					i /= 10;
				} while (i);
				/* reverse digits, stop in middle */
				r = q;		/* don't alter q */
				while (--r > p) {
					i = *r;
					*r = *p;
					*p++ = i;
				}
			}
#if 0
			else if (*fmt == '@') {
				unsigned char *r;
				union {
					long		l;
					unsigned char	c[4];
				} u;
				const long *lp = (const long *)dp;
				u.l = *lp++;
				dp = (const int *)lp;
				for (r = &u.c[0]; r < &u.c[4]; ++r)
					q += SprintF(q, "%d.", *r);
				--q;
			}
#endif
#if 0
			else if (*fmt == '!') {
				char *r;
				p = (char *)*dp++;
				for (r = p + ETH_ALEN; p < r; ++p)
					q += SprintF(q, "%hhX:", *p);
				--q;
			}
#endif
			else if (*fmt == 'c')
				*q++ = *dp++;
			else
				*q++ = *fmt;
			/* now output the saved string */
			for (p = tmp; p < q; ++p)
			{
				if(buf)
					*s++ = *p;
				else
					DiagPutChar(*p);
			}
		}
	}
	if (buf)
		*s = '\0';
	return (s - buf);
}

void LogDirect(IN  char *fmt, ...)
{
    uint16_t log_length = 0;
    char l_msg[MAX_LOG_MESSAGE_LEN];
    uint32_t TimeStamp;
    uint16_t message_length = 0;
    uint16_t i=0;
    uint8_t loc_seq;

    taskENTER_CRITICAL();
    gSequenceNumber++;
    loc_seq = gSequenceNumber;
    TimeStamp = xTaskGetTickCount() * 10;
    taskEXIT_CRITICAL();

/*
	Byte: Description
	0:		sync(0x7E)
	1:		length
	2:		seqno
	3:		CheckSum
	4-5:	       TimeStamp
	6:		Type = 16 (Defined in Bee1)
	7:           char*
*/

    l_msg[0] = LOG_MESSAGE_SYNC_CODE;				//syntax
    l_msg[2] = loc_seq;  			//seqno
    l_msg[4] = (uint8_t)(TimeStamp & 0xff);		//TimeStamp
    l_msg[5] = (uint8_t)((TimeStamp>>8) & 0xff);
    l_msg[6] = 16;     //type defined in Bee1

    log_length = 7;

    //Todo: buffer overflow
    message_length = VSprintf(l_msg+7, fmt, ((const int *)&fmt)+1);
    log_length += message_length;

    l_msg[1] = log_length;
    l_msg[3] = l_msg[0] ^ l_msg[1] ^ l_msg[2];

    if(pPatch_LOG_RAW)
        pPatch_LOG_RAW(l_msg, &log_length);

    for(i=0; i<log_length; ++i)
        DiagPutChar(l_msg[i]);
}

uint8_t LogBufferFormat(uint32_t type_module_mixed, uint32_t log_str_index, uint8_t para_num, ...)
{
    uint8_t log_length, i;
    va_list arg;
    uint8_t l_msg[MAX_LOG_MESSAGE_LEN];
    uint32_t TimeStamp = xTaskGetTickCount() * 10;

/*
        Byte: Description
        0: 		sync(0x7E)
        1: 		length
        2: 		seqno
        3: 		CheckSum
        4-5: 	TimeStamp
        6:		Type = TYPE_BUMBLEBEE3

        [SUBTYPE_FORMAT]
        7:         SubType: SUBTYPE_FORMAT
        8:         module
        9-10: 	Message Index
        11: 	Num of args: N
        12(Totol: 4*N): args

        [SUBTYPE_STRING]
        7:         SubType: SUBTYPE_STRING
        8:         module
        9-10: 	Message Index
        11:        String

        [SUBTYPE_BINARY]
        7:         SubType: SUBTYPE_BINARY                     
        8:         module
        9-10: 	Message Index
        11:        Binary
*/

    l_msg[0] = LOG_MESSAGE_SYNC_CODE;				        //syntax
    l_msg[4] = (uint8_t)(TimeStamp & 0xff);		//TimeStamp
    l_msg[5] = (uint8_t)((TimeStamp>>8) & 0xff);  
    l_msg[6] = TYPE_BUMBLEBEE3; 
    l_msg[7] = GET_TYPE(type_module_mixed);
    l_msg[8] = GET_MODULE(type_module_mixed);
    log_str_index = log_str_index>>2;
    l_msg[9] = (uint8_t)((log_str_index) & 0xff);
    l_msg[10] = (uint8_t)((log_str_index>>8) & 0xff);

    if(para_num > 0)
    {
        switch(l_msg[7])
        {
        case SUBTYPE_FORMAT:
            {
            uint32_t param;
            if(para_num > MAX_ARGUMENT_SIZE)  //Avoid memory overflow
                para_num = MAX_ARGUMENT_SIZE;

            l_msg[11] = para_num;
            log_length = 12;

            va_start(arg, para_num);
            for(i=0; i<para_num; i++)			//para_num*32bits data
            {
                param = va_arg(arg, int);
                l_msg[log_length] = (uint8_t)(param&0xff);
                l_msg[log_length+1] = (uint8_t)((param&0xff00)>>8);
                l_msg[log_length+2] = (uint8_t)((param&0xff0000)>>16);
                l_msg[log_length+3] = (uint8_t)((param&0xff000000)>>24);
                log_length+=4;
            }
            va_end(arg);
            }
            break;
            
        case SUBTYPE_STRING:
            if (para_num == 1)
            {
                uint8_t dataLength;
                char *pString;
                
                va_start(arg, para_num);
                pString = va_arg(arg, char *);
                va_end(arg);
                
                dataLength = strlen(pString);  //Not include 0.
                log_length += dataLength;
                if (log_length > MAX_LOG_MESSAGE_LEN)
                {
                    log_length = MAX_LOG_MESSAGE_LEN;
                    dataLength = log_length - 11;
                }
                memcpy(&l_msg[11], pString, dataLength);
            }
            else
                return 1;
            break;
            
        case SUBTYPE_BINARY:
            if (para_num == 2)
            {
                uint8_t dataLength;  //param0: length of binary data
                char *pData;            //param1: binary data address
                
                va_start(arg, para_num);
                dataLength = va_arg(arg, uint32_t);
                pData = va_arg(arg, char *);
                va_end(arg);
                
                log_length += dataLength;
                if (log_length > MAX_LOG_MESSAGE_LEN)
                {
                    log_length = MAX_LOG_MESSAGE_LEN;
		       dataLength = log_length - 11;
                }
                memcpy(&l_msg[11], pData, dataLength);
            }
            else
                return 1;
            break;
        default:
            break;
        }
    }
    else
    {
        //has no paras
        log_length = 11;
    }

    l_msg[1] = log_length;				        //length

    taskENTER_CRITICAL();
    gSequenceNumber++;
    l_msg[2] = gSequenceNumber;  			//seqno
    l_msg[3] = l_msg[0]^l_msg[1]^l_msg[2] ;		//CheckSum

    if(pPatch_LOG_BUFFER)
        pPatch_LOG_BUFFER(l_msg, &log_length);
    
    PPB_Write(l_msg, log_length);  //TODO
    taskEXIT_CRITICAL();

    return 0;
}

uint8_t LogBufferSNOOP(uint32_t type_module_mixed, uint16_t dataLength, uint8_t *pBinaryData)
{
    uint8_t log_length;
    uint8_t l_msg[MAX_LOG_MESSAGE_LEN];
    uint32_t TimeStamp = xTaskGetTickCount() * 10;

/*
        Byte: Description
        0: 		sync(0x7E)
        1: 		length
        2: 		seqno
        3: 		CheckSum
        4-5: 	TimeStamp
        6:		Type = TYPE_BUMBLEBEE3
        7:         SubType:  
                                  SUBTYPE_CMD_SNOOP, SUBTYPE_EVT_SNOOP, SUBTYPE_ACL_TX_SNOOP,  
                                  SUBTYPE_ACL_RX_SNOOP, SUBTYPE_SCO_TX_SNOOP, SUBTYPE_SCO_RX_SNOOP 
        8:         module
        9:         Binary
*/

    l_msg[0] = LOG_MESSAGE_SYNC_CODE;				        //syntax
    l_msg[4] = (uint8_t)(TimeStamp & 0xff);		//TimeStamp
    l_msg[5] = (uint8_t)((TimeStamp>>8) & 0xff);  
    l_msg[6] = TYPE_BUMBLEBEE3; 
    l_msg[7] = GET_TYPE(type_module_mixed);
    l_msg[8] = GET_MODULE(type_module_mixed);
    
    log_length = 9;

    log_length += dataLength;
    if (log_length > MAX_LOG_MESSAGE_LEN)
    {
        log_length = MAX_LOG_MESSAGE_LEN;
        dataLength = log_length - 9;
    }
    memcpy(&l_msg[9], pBinaryData, dataLength);

    l_msg[1] = log_length;				        //length

    taskENTER_CRITICAL();
    gSequenceNumber++;
    l_msg[2] = gSequenceNumber;  			//seqno
    l_msg[3] = l_msg[0]^l_msg[1]^l_msg[2] ;		//CheckSum
    
    PPB_Write(l_msg, log_length);  //TODO
    taskEXIT_CRITICAL();

    return 0; 
}


/****************************************************************************/
/* Trace BD Addr                                                                                                     */
/****************************************************************************/

const char *traceBdAddr(const char *pBdAddr, uint8_t SubType)
{
/*
        Byte: Description
        0:      sync(0x7E)
        1:      length
        2:      seqno
        3:      CheckSum
        4-5:    TimeStamp
        6:      Type = TYPE_BUMBLEBEE3
        7:       SubType: SUBTYPE_BDADDR1~SUBTYPE_BDADDR2
        8-13:  BdAddr

*/
    uint8_t l_msg[MAX_LOG_MESSAGE_LEN];
    uint32_t TimeStamp = xTaskGetTickCount() * 10;

    l_msg[0] = LOG_MESSAGE_SYNC_CODE;
    l_msg[1] = 14;
    l_msg[4] = (uint8_t)((TimeStamp) & 0xff);
    l_msg[5] = (uint8_t)(((TimeStamp) >> 8) & 0xff);
    l_msg[6] = TYPE_BUMBLEBEE3;
    l_msg[7] = SubType;

    memcpy(&l_msg[8], pBdAddr, 6);

    taskENTER_CRITICAL();	 
    gSequenceNumber++;
    l_msg[2] = gSequenceNumber;
    l_msg[3] = l_msg[0]^l_msg[1]^l_msg[2] ;		//CheckSum

    PPB_Write(l_msg, 14);  //TODO
    taskEXIT_CRITICAL();

    return ((const char *)((0xBDBD << 8) | SubType));
    
}

const char *traceBdAddr1(const char *pBdAddr)
{
    return (traceBdAddr(pBdAddr, SUBTYPE_BDADDR1));
}

const char *traceBdAddr2(const char *pBdAddr)
{
    return (traceBdAddr(pBdAddr, SUBTYPE_BDADDR2));
}

/****************************************************************************/
/* Trace Ram data                                                                                                     */
/****************************************************************************/

const char *traceRamData(char *pData, uint8_t SubType)
{
/*
        Byte: Description
        0:      sync(0x7E)
        1:      length
        2:      seqno
        3:      CheckSum
        4-5:    TimeStamp
        6:      Type = TYPE_BUMBLEBEE3
        7:       SubType: SUBTYPE_RAMDATA1~SUBTYPE_RAMDATA8
        8:      pData
*/
    uint8_t log_length;
    uint8_t l_msg[MAX_LOG_MESSAGE_LEN];
    uint32_t TimeStamp = xTaskGetTickCount() * 10;
    uint8_t dataLength;

    l_msg[0] = LOG_MESSAGE_SYNC_CODE;
    l_msg[1] = 14;
    l_msg[4] = (uint8_t)((TimeStamp) & 0xff);
    l_msg[5] = (uint8_t)(((TimeStamp) >> 8) & 0xff);
    l_msg[6] = TYPE_BUMBLEBEE3;
    l_msg[7] = SubType;

    log_length = 8;

    dataLength = strlen(pData);  //Not include 0.
    log_length += dataLength;
    if (log_length > MAX_LOG_MESSAGE_LEN)
    {
        log_length = MAX_LOG_MESSAGE_LEN;
        dataLength = log_length - 8;
    }
    memcpy(&l_msg[8], pData, dataLength);
    
    taskENTER_CRITICAL();	 
    gSequenceNumber++;
    l_msg[2] = gSequenceNumber;
    l_msg[3] = l_msg[0]^l_msg[1]^l_msg[2] ;		//CheckSum

    PPB_Write(l_msg, log_length);  //TODO
    taskEXIT_CRITICAL();

    return ((const char *)((0xDADD << 8) | SubType));
}

const char *traceRamData1(char *pData)
{
    return (traceRamData(pData, SUBTYPE_RAMDATA1));
}

const char *traceRamData2(char *pData)
{
    return (traceRamData(pData, SUBTYPE_RAMDATA2));
}

const char *traceRamData3(char *pData)
{
    return (traceRamData(pData, SUBTYPE_RAMDATA3));
}

const char *traceRamData4(char *pData)
{
    return (traceRamData(pData, SUBTYPE_RAMDATA4));
}

const char *traceRamData5(char *pData)
{
    return (traceRamData(pData, SUBTYPE_RAMDATA5));
}

const char *traceRamData6(char *pData)
{
    return (traceRamData(pData, SUBTYPE_RAMDATA6));
}

const char *traceRamData7(char *pData)
{
    return (traceRamData(pData, SUBTYPE_RAMDATA7));
}

const char *traceRamData8(char *pData)
{
    return (traceRamData(pData, SUBTYPE_RAMDATA8));
}


uint8_t LogBufferLowerStack(uint32_t control, uint16_t log_str_index, uint8_t para_num, ...)
{
    uint8_t log_length, i;
    va_list argp;
    uint8_t l_msg[MAX_LOG_MESSAGE_LEN];
    uint32_t TimeStamp = xTaskGetTickCount() * 10;
    uint8_t color = (uint8_t)(control>>24);
    uint8_t file_num = (uint8_t)((control>>16)&0xFF);
    uint16_t line_num = (uint16_t)(control&0x0000FFFF);

    if ((gTraceLevelModules[LEVEL_ERROR][MODULE_LOWERSTACK>>MODULE_GROUP_BITs] & BIT(MODULE_LOWERSTACK&31))  == 0)
        return 0;

/*
        Byte: Description
        0: 		sync(0x7E)
        1: 		length
        2: 		seqno
        3: 		CheckSum
        4-5: 	TimeStamp
        6: 		Type = 249 (Defined in Bee1)
        7:
            bit 0: index compressed or not. (Must be 0)
            bit 1: line Num exist. (Must be 1)
            bit 2: file Num exist. (Must be 1)
            bit 3-4: args: interger or str. 
            bit 5-7: color
        8-9: 	Message Index
        10: 	file Num
        11-12: 	line Num
        13: 	Num of args: N
        14(Totol: 4*N): args
*/

    l_msg[0] = LOG_MESSAGE_SYNC_CODE;				        //syntax
    l_msg[4] = (uint8_t)(TimeStamp & 0xff);		//TimeStamp
    l_msg[5] = (uint8_t)((TimeStamp>>8) & 0xff);
    l_msg[6] = 249; 
    l_msg[7] = ((color & 0x7)<<5) | 6;			//color
    l_msg[8] = (uint8_t)((log_str_index) & 0xff);
    l_msg[9] = (uint8_t)((log_str_index>>8) & 0xff);
    l_msg[10] = (uint8_t)(file_num&0xff);		//8 bits file_num
    l_msg[11] = (uint8_t)(line_num&0xff);		//8 bits line_num
    l_msg[12] = (uint8_t)((line_num&0xff00)>>8);	//8 bits line_num

    if(para_num>0)
    {
        int temp;

        if(para_num > MAX_ARGUMENT_SIZE)  //Avoid memory overflow
            para_num = MAX_ARGUMENT_SIZE;

        l_msg[7] |= 0x08;					//has integer para
        l_msg[13] = (uint8_t)para_num;
        log_length = 14;

        va_start(argp, para_num);
        for(i=0; i<para_num; i++)			//para_num*32bits data
        {
            temp = va_arg(argp, int);
            l_msg[log_length] = (uint8_t)(temp&0xff);
            l_msg[log_length+1] = (uint8_t)((temp&0xff00)>>8);
            l_msg[log_length+2] = (uint8_t)((temp&0xff0000)>>16);
            l_msg[log_length+3] = (uint8_t)((temp&0xff000000)>>24);
            log_length+=4;
        }
        va_end(argp);
    }
    else
    {
        //has no paras
        log_length = 13;
    }

    l_msg[1] = log_length;				        //length

    taskENTER_CRITICAL();
    gSequenceNumber++;
    l_msg[2] = gSequenceNumber;  			//seqno
    l_msg[3] = l_msg[0]^l_msg[1]^l_msg[2] ;		//CheckSum

    if(pPatch_LOG_BUFFER)
        pPatch_LOG_BUFFER(l_msg, &log_length);

    PPB_Write(l_msg, log_length);  //TODO
    taskEXIT_CRITICAL();

    return 0;
}


uint8_t LogBufferLowerStackData(uint32_t control, uint16_t log_str_index, uint16_t length, uint8_t* pStr)
{
    uint8_t log_length;
    uint8_t l_msg[MAX_LOG_MESSAGE_LEN];
    uint32_t TimeStamp = xTaskGetTickCount() * 10;
    uint8_t color = (uint8_t)(control>>24);
    uint8_t file_num = (uint8_t)((control>>16)&0xFF);
    uint16_t line_num = (uint16_t)(control&0x0000FFFF);

    if ((gTraceLevelModules[LEVEL_ERROR][MODULE_LOWERSTACK>>MODULE_GROUP_BITs] & BIT(MODULE_LOWERSTACK&31))  == 0)
        return 0;

/*
	Byte: Description
	0:		sync(0x7E)
	1:		length
	2:		seqno
	3:		CheckSum
	4-5: 	TimeStamp
	6: 		Type = 249 (Defined in Bee1)
	7:
	     bit 0: index compressed or not. (Must be 0)
            bit 1: line Num exist. (Must be 1)
            bit 2: file Num exist. (Must be 1)
            bit 3-4: args: interger or str. 
            bit 5-7: color
	8-9:	Message Index
	10: 	file Num
	11-12:	line Num
	13: 	length of string: N
	14(Totol: N): string
*/

    l_msg[0] = LOG_MESSAGE_SYNC_CODE;			//syntax
    l_msg[4] = (uint8_t)(TimeStamp & 0xff);		//TimeStamp
    l_msg[5] = (uint8_t)((TimeStamp>>8) & 0xff);
    l_msg[6] = 249; 
    l_msg[7] = ((color&0x7)<<5) | 6;			//color
    l_msg[8] = (uint8_t)((log_str_index) & 0xff);
    l_msg[9] = (uint8_t)((log_str_index>>8) & 0xff);
    l_msg[10] = (uint8_t)(file_num&0xff);		//8 bits file_num
    l_msg[11] = (uint8_t)(line_num&0xff);		//8 bits line_num
    l_msg[12] = (uint8_t)((line_num&0xff00)>>8);	//8 bits line_num


    if(length>0)
    {
        l_msg[7] |= 0x10;					//has str para
        l_msg[13] = (uint8_t)length;
        log_length = 14;

        if(length > (MAX_LOG_MESSAGE_LEN - log_length))
            length = MAX_LOG_MESSAGE_LEN - log_length;

        memcpy(l_msg+log_length, pStr, length);	//Length*8bits data
        log_length += length;
    }
    else
    {
        //has no paras
        log_length = 13;
    }

    l_msg[1] = log_length;

    taskENTER_CRITICAL();
    gSequenceNumber++;
    l_msg[2] = gSequenceNumber;  			        //seqno
    l_msg[3] = l_msg[0] ^ l_msg[1] ^ l_msg[2];

    if(pPatch_LOG_DATA)
        pPatch_LOG_DATA(l_msg, &log_length);
    
    PPB_Write(l_msg, log_length);
    taskEXIT_CRITICAL();

    return 0;
}

void LogTraceModulesMask(uint32_t levelModules[][MODULE_GROUP_NUM])
{
    uint8_t i, j;
    for( i = 0; i < LEVEL_NUMs; ++i )
    {
        for(j = 0; j < MODULE_GROUP_NUM; ++j)
        {
            gTraceLevelModules[i][j] = levelModules[i][j];
        }
    }
}

//Temp, will use gdma instead
void LogUartPrint(const uint8_t *source, uint16_t size)
{
	uint8_t i;
	uint8_t blkcount;
    uint8_t remainder;
	
	/* wait tx fifo empty */
    while(LOG_UART_GetFlagState(LOG_UART, LOG_UART_FLAG_THR_TSR_EMPTY) != SET);
	
	blkcount = size / LOG_UART_TX_FIFO_SIZE;
    remainder = size % LOG_UART_TX_FIFO_SIZE;
	
	/* send block bytes(16 bytes) */
    for(i = 0; i < blkcount; i++)
    {
        LOG_UART_SendData(LOG_UART, &source[16 * i], 16);
        //LOG_UART_GetTxFifoLevel(LOG_UART);  //get tx fifo level
        /* wait tx fifo empty */
        while(LOG_UART_GetFlagState(LOG_UART, LOG_UART_FLAG_THR_TSR_EMPTY) != SET);
    }
    
    /* send left bytes */
    LOG_UART_SendData(LOG_UART, &source[16 * i], remainder);
}


