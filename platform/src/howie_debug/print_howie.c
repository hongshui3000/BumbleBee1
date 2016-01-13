#include "rtl876x_uart.h"
#include "rtl876x_log_uart.h"
#include "print_howie.h"
#include <stdarg.h>
#include <string.h>

#define	OUTBUFSIZE	1024
#define	INBUFSIZE	1024


static  char g_pcOutBuf[OUTBUFSIZE];

void putc (unsigned char data);


int vs_printf(char *buf, const char *fmt, const int *dp)
{
	char *p, *s;

	s = buf;
	for ( ; *fmt != '\0'; ++fmt) {
		if (*fmt != '%') {
			if(buf)
				*s++ = *fmt;
			else
				putc(*fmt);
			continue;
		}
		if (*++fmt == 's') {
			for (p = (char *)*dp++; *p != '\0'; p++)
			{
				if(buf)
					*s++ = *p;
				else
					putc(*p);
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
			else if (*fmt == 'f') {
				serial_puts("error cannt use float!\n");
//				float i = *dp++;
//				char *r;
//				if (i < 0) {
//					*q++ = '-';
//					i = -i;
//				}
//				p = q;		/* save beginning of digits */
			}
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
					putc(*p);
			}
		}
	}
	if (buf)
		*s = '\0';
	return (s - buf);
}



/*
 * Output a single byte to the serial port.
 */



int print_howie(const char *fmt, ...)
{
	int i;
	int len;
	va_list args;

	va_start(args, fmt);
	len = vs_printf(g_pcOutBuf,fmt,((const int *)&fmt)+1);
	va_end(args);
	for (i = 0; i < strlen(g_pcOutBuf); i++)
	{
		putc(g_pcOutBuf[i]);
	}
	return len;
}

void putc (unsigned char data)
{
#if 0
	/* wait for room in the transmit FIFO */
	while(UART_GetFlagState(UART, UART_FLAG_THR_EMPTY) != SET);

	if (data=='\n'){
		UART->RB_THR='\r';
		UART->RB_THR='\n';
	}
	else
	UART->RB_THR = data;
#endif
	
#if 1
	extern void LOG_UART_SendData(LOG_UART_TypeDef* UARTx, const uint8_t* inBuf, uint16_t count);
  while(LOG_UART_GetFlagState(LOG_UART, LOG_UART_FLAG_THR_TSR_EMPTY) != SET);
      LOG_UART_SendData(LOG_UART, &data, 1);
	
#endif
}

void serial_puts (const char *s)
{
	while (*s) {
		putc (*s++);
	}
}

/*
 * Read a single byte from the serial port. Returns 1 on success, 0
 * otherwise. When the function is succesfull, the character read is
 * written into its argument c.
 */
int serial_tstc (void)
{
	int temp;
	return temp;
}




unsigned char getc (void)
{
	int rv;

	for(;;) {
		rv = serial_tstc();
		if(rv > 0)
		{
			return (uint8_t)(UART->RB_THR);
		}
	}
}

