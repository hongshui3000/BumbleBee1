/**
********************************************************************************************************
Copyright (c) 2015, Realtek Semiconductor Corporation. All rights reserved.
*********************************************************************************************************
* @file 	  diag.c
* @brief	  app log api
* @details	 
*
* @author	lory_xu
* @date 		2015-07-13
* @version	v0.1
*/

enum { __FILE_NUM__= 0 };

#include <stdarg.h>
#include <diag.h>
#include <cycle_queue.h>
#include <stextif.h>
#include <os_sched.h>
#include <os_intr.h>

uint32_t ConfigDebug[LEVEL_NUMs] = {0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff};

#define DiagPutChar LogUartTxChar

void LogUartTxChar(char fmt){}
#define MAX_LOG_MESSAGE_LEN 128//256//1024
#define MAX_ARGUMENT_SIZE 20
uint8_t gSequenceNumber = 0xFF;

/**
* @brief add arg to string  
* 
* @param  buf
* @param  fmt
* @param  dp
*
* @return  
*/
int VSprintf(char *buf, const char *fmt, const int *dp)
{
	char *p, *s;

	s = buf;
	for ( ; *fmt != '\0'; ++fmt) 
	{
		if (*fmt != '%') 
		{
			if(buf)
			{
				*s++ = *fmt;
			}
			else
			{
				DiagPutChar(*fmt);
			}
			continue;
		}
		if (*++fmt == 's') 
		{
			for (p = (char *)*dp++; *p != '\0'; p++)
			{
				if(buf)
				{
					*s++ = *p;
				}
				else
				{
					DiagPutChar(*p);
				}
			}
		}
		else 
		{	/**< Length of item is bounded */
			char tmp[20], *q = tmp;
			int alt = 0;
			int shift = 28;

			if ((*fmt  >= '0') && (*fmt  <= '9'))
			{
				int width;
				unsigned char fch = *fmt;
				for (width=0; (fch>='0') && (fch<='9'); fch=*++fmt)
				{    
					width = width * 10 + fch - '0';
				}
				shift=(width-1)*4;
			}

			/**
			 * Before each format q points to tmp buffer
			 * After each format q points past end of item
			 */

			if ((*fmt == 'x')||(*fmt == 'X') || (*fmt == 'p') || (*fmt == 'P')) 
			{
				/* With x86 gcc, sizeof(long) == sizeof(int) */
				const long *lp = (const long *)dp;
				long h = *lp++;
				int ncase = (*fmt & 0x20);
				dp = (const int *)lp;
				if((*fmt == 'p') || (*fmt == 'P'))
				{
					alt=1;
				}
				if (alt) 
				{
					*q++ = '0';
					*q++ = 'X' | ncase;
				}
				for ( ; shift >= 0; shift -= 4)
				{
					*q++ = "0123456789ABCDEF"[(h >> shift) & 0xF] | ncase;
				}
			}
			else if (*fmt == 'd') 
			{
				int i = *dp++;
				char *r;
				if (i < 0) 
				{
					*q++ = '-';
					i = -i;
				}
				p = q;		/* save beginning of digits */
				do 
				{
					*q++ = '0' + (i % 10);
					i /= 10;
				} while (i);
				/* reverse digits, stop in middle */
				r = q;		/* don't alter q */
				while (--r > p) 
				{
					i = *r;
					*r = *p;
					*p++ = i;
				}
			}
			else if (*fmt == 'c')
			{
				*q++ = *dp++;
			}
			else
			{
				*q++ = *fmt;
			}
			/* now output the saved string */
			for (p = tmp; p < q; ++p)
			{
				if(buf)
				{
					*s++ = *p;
				}
				else
				{
					DiagPutChar(*p);
				}
			}
		}
	}
	if (buf)
	{
		*s = '\0';
	}
	return (s - buf);
}

/**
* @brief raw data log print
* 
* @param  fmt
*
* @return  
*/
uint32_t
LOG_RAW(
    char *fmt, ...
)
{
	uint16_t log_length = 0;
	char l_msg[MAX_LOG_MESSAGE_LEN];
	uint32_t TimeStamp;
	uint16_t message_length = 0;
	uint16_t i=0;
	uint8_t loc_seq;

	int s = osInterruptDisable();
	gSequenceNumber++;
	loc_seq = gSequenceNumber;
	TimeStamp = osGetSystemTime();

	osInterruptEnable(s);

	/**
	*	Byte: Description
	*	0:		sync(0x7E)
	*	1:		length
	*	2:		seqno
	*	3:		CheckSum
	*	4-5:	TimeStamp
	*	6:		Type = 16
	*	7:      char*
	*/
	l_msg[0] = 0x7E;				/**< syntax */ 
	l_msg[2] = loc_seq;  			/**< seqno */
	l_msg[4] = (UCHAR)(TimeStamp & 0xff);		/**< TimeStamp */
	l_msg[5] = (UCHAR)((TimeStamp>>8) & 0xff);

	l_msg[6] = 16;							/**< Type(TRACE_TYPE_Undecod) */
	log_length = 7;
	message_length = VSprintf(l_msg+7, fmt, ((const int *)&fmt)+1);
	log_length += message_length;

	l_msg[1] = log_length;
	l_msg[3] = l_msg[0] ^ l_msg[1] ^ l_msg[2];

	for(i = 0; i < log_length; ++i)
	{
		DiagPutChar(l_msg[i]);    
	}

	return 0;
}

/**
* @brief buf log print
* 
* @param  color
* @param  module
* @param  file_num
* @param  line_num
* @param  log_str_index
* @param  index_compressed
* @param  para_num
*
* @return  
*/
uint8_t LOG_BUFFER(uint8_t color, uint8_t module, uint16_t file_num, uint16_t line_num,
          uint32_t log_str_index, uint8_t index_compressed, uint8_t para_num, ...)
{
	uint8_t log_length, i;
	va_list argp;
	UCHAR l_msg[MAX_LOG_MESSAGE_LEN];
	uint32_t TimeStamp = osGetSystemTime();
	int s;

	/**
	*	Byte: Description
	*	0: 		sync(0x7E)
	*	1: 		length
	*	2: 		seqno
	*	3: 		CheckSum
	*	4-5: 	TimeStamp
	*	6: 		Type = 15
	*	7:
	*	bit 0: index compressed or not
	*	bit 1: line Num exist
	*	bit 2: file Num exist
	*	bit 3-4: args
	*	bit 5-7: color
	*	8-9: 	Message Index
	*	10: 	file Num
	*	11-12: 	line Num
	*	13: 	Num of args: N
	*	14(Totol: 4*N): args
	*/
	l_msg[0] = 0x7E;				        /**< syntax */
	l_msg[4] = (UCHAR)(TimeStamp & 0xff);		/**< TimeStamp */
	l_msg[5] = (UCHAR)((TimeStamp>>8) & 0xff);
	l_msg[6] = module; 
	l_msg[7] = ((color&0x7)<<5)				/**< color */
	           | 0x04						/**< 1=has file number */
	           | 0x02;						/**< 1=has line number */

	if(index_compressed)
	{
		l_msg[7] |= 0x01;                   /**< index is compressed*/
		l_msg[8] = (UCHAR)((log_str_index>>2) & 0xff);
	    l_msg[9] = (UCHAR)((log_str_index>>10) & 0xff);
	}
	else
	{
	    l_msg[8] = (UCHAR)((log_str_index) & 0xff);
	    l_msg[9] = (UCHAR)((log_str_index>>8) & 0xff);
	}

	l_msg[10] = (UCHAR)(file_num&0xff);		/**< 8 bits file_num*/
	l_msg[11] = (UCHAR)(line_num&0xff);		/**<  bits line_num */
	l_msg[12] = (UCHAR)((line_num&0xff00)>>8);	/**< 8 bits line_num */

	if(para_num>0)
	{
		int temp;

	    if(para_num > MAX_ARGUMENT_SIZE)  /**< Avoid memory overflow*/
    	{
    		para_num = MAX_ARGUMENT_SIZE;
    	}

	    l_msg[7] |= 0x08;					/**< has str para*/
	    l_msg[13] = (UCHAR)para_num;
	    log_length = 14;

	    va_start(argp, para_num);
	    for(i=0; i<para_num; i++)			/**< para_num*32bits data*/
	    {
	        temp = va_arg(argp, int);
			l_msg[log_length] = (UCHAR)(temp&0xff);
			l_msg[log_length+1] = (UCHAR)((temp&0xff00)>>8);
			l_msg[log_length+2] = (UCHAR)((temp&0xff0000)>>16);
			l_msg[log_length+3] = (UCHAR)((temp&0xff000000)>>24);
	        log_length+=4;
	    }
	    va_end(argp);
	}
	else
	{
	    /**has no paras*/
	    log_length = 13;
	}

	l_msg[1] = log_length;				        /**< length*/

	s = osInterruptDisable();
	gSequenceNumber++;
	l_msg[2]  = gSequenceNumber;
	l_msg[3] = l_msg[0]^l_msg[1]^l_msg[2] ;		/**< CheckSum*/

	stTraceOutput(l_msg, log_length);
	osInterruptEnable(s);

	return 0;
}

/**
* @brief  set debug mask
* 
* @param  config
*
* @return  
*/
void set_debug_mask(uint32_t config[])
{
	ConfigDebug[LEVEL_ERROR]    = config[0];
	ConfigDebug[LEVEL_WARN]     = config[1];
	ConfigDebug[LEVEL_INFO]     = config[2];
	ConfigDebug[LEVEL_TRACE]    = config[3];
}

/**
 * @brief Converts a character into hexadecimal num.
 *
 * @param  c -- a character.
 *
 * @return the hexadecimal num.
*/
uint8_t c2hex(uint8_t c)
{
	uint8_t hex = 0;

	if (c >= '0' && c <= '9')
	{
		hex = c - '0';
	}
	else if (c >= 'A' && c <= 'F')
	{
		hex = c - 'A' + 0xa;
	}
	else if (c >= 'a' && c <= 'f')
	{
		hex = c - 'a' + 0xa;
	}

	return hex;
}
/**
 * @brief Converts a hexadecimal num into a character.
 * 
 *@param  hex -- a hexadecimal num.
 *
 *@return the character.
*/
uint8_t hex2c(uint8_t hex)
{
	uint8_t c = 0;

	if (hex <= 9)
	{
		c = hex + '0';
	}
	else if(hex >= 0xA && hex <= 0xF)
	{
		c = hex - 0xA + 'A';
	}

	return c;
}

/**
 * @brief return the check sum of hex386.
 *
 * @param  buf -- data buffer.
 * @paramsize -- size of data buffer.
 *
 * @return the check sum of hex386.
*/
uint8_t cs_for_hex386(uint8_t *buf, uint32_t size)
{
	uint8_t ret = 0;

	while (size) 
	{
		size -= 2;
		ret += ((c2hex(buf[size]) << 4) + c2hex(buf[size + 1]));
	}

	return (0xFF - ret + 1);
}

