/*
 * Copyright (C) 2001 MontaVista Software Inc.
 * Author: Jun Sun, jsun@mvista.com or jsun@junsun.net
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 *
 */
#if defined(FOR_SIMULATION)
#include	"print.h"
#include    "datatype.h"

/* macros */
#define		IsDigit(x)	( ((x) >= '0') && ((x) <= '9') )
#define		Ctod(x)		( (x) - '0')

/* forward declaration */
extern INT32 PrintChar(UINT8 *, UINT8, INT32, INT32);
extern INT32 PrintString(UINT8 *, UINT8 *, INT32, INT32);
extern INT32 PrintNum(UINT8 *, unsigned long, INT32, INT32, INT32, INT32, UINT8, INT32);

/* private variable */
static const UINT8 theFatalMsg[] = "fatal error in GenPrint!";

/* -*-
 * A low level printf() function.
 */
void GenPrint(void (*output)(void *, UINT8 *, INT32),
              void * arg,
              UINT8 *fmt,
              va_list ap)
{
#define 	OUTPUT(arg, s, l)  \
{ if (((l) < 0) || ((l) > LP_MAX_BUF)) { \
            (*output)(arg, (UINT8*)theFatalMsg, sizeof(theFatalMsg)-1); for(;;); \
        } else { \
            (*output)(arg, s, l); \
        } \
    }

    UINT8 buf[LP_MAX_BUF];

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
            while((*fmt != '\0') && (*fmt != '%'))
            {
                fmt ++;
            }

            /* flush the string found so far */
            OUTPUT(arg, fmtStart, fmt-fmtStart);

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
                OUTPUT(arg, buf, length);
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
                OUTPUT(arg, buf, length);
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
                OUTPUT(arg, buf, length);
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
                OUTPUT(arg, buf, length);
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
                OUTPUT(arg, buf, length);
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
                OUTPUT(arg, buf, length);
                break;

            case 'c':
                c = (UINT8)va_arg(ap, INT32);
                length = PrintChar(buf, c, width, ladjust);
                OUTPUT(arg, buf, length);
                break;

            case 's':
                s = (UINT8*)va_arg(ap, UINT8 *);
                length = PrintString(buf, s, width, ladjust);
                OUTPUT(arg, buf, length);
                break;

            case '\0':
                fmt --;
                break;

            default:
                /* output this UINT8 as it is */
                OUTPUT(arg, fmt, 1);
        }	/* switch (*fmt) */

        fmt ++;
    }		/* for(;;) */

    /* special termination call */
    OUTPUT(arg, '\0', 1);
}


/* --------------- local help functions --------------------- */
INT32
PrintChar(UINT8 * buf, UINT8 c, INT32 length, INT32 ladjust)
{
    INT32 i;

    if (length < 1) length = 1;
    if (ladjust)
    {
        *buf = c;
        for (i=1; i< length; i++) buf[i] = ' ';
    }
    else
    {
        for (i=0; i< length-1; i++) buf[i] = ' ';
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
    if (length < len) length = len;

    if (ladjust)
    {
        for (i=0; i< len; i++) buf[i] = s[i];
        for (i=len; i< length; i++) buf[i] = ' ';
    }
    else
    {
        for (i=0; i< length-len; i++) buf[i] = ' ';
        for (i=length-len; i < length; i++) buf[i] = s[i-length+len];
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
    if (length < actualLength) length = actualLength;

    /* add padding */
    if (ladjust)
    {
        padc = ' ';
    }
    if (negFlag && !ladjust && (padc == '0'))
    {
        for (i = actualLength-1; i< length-1; i++) buf[i] = padc;
        buf[length -1] = '-';
    }
    else
    {
        for (i = actualLength; i< length; i++) buf[i] = padc;
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

    /* adjust the string pointer */
    return length;
}
#endif//defined(DIRECT_UART_PRINT) || defined(FOR_SIMULATION)
