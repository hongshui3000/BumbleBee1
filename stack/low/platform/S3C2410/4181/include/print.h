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

#ifndef _print_h_
#define _print_h_

#if defined(FOR_SIMULATION)

#include <stdarg.h>
#include "DataType.h"

/* this is the maximum width for a variable */
#define		LP_MAX_BUF	80

/* -*-
 * output function takes an void pointer which is passed in as the
 * second argument in GenPrint().  This black-box argument gives output
 * function a way to track state.
 *
 * The second argument in output function is a pointer to UINT8 buffer.
 * The third argument specifies the number of chars to outputed.
 *
 * output function cannot assume the buffer is null-terminated after
 * l number of chars.
 */
void GenPrint(void (*output)(void *, UINT8 *, INT32), 
	      void * arg,
	      UINT8 *fmt, 
	      va_list ap);

#endif// defined(DIRECT_UART_PRINT) || defined(FOR_SIMULATION)

#endif

